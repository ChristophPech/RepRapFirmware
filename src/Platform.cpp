/****************************************************************************************************

 RepRapFirmware - Platform: RepRapPro Ormerod with Duet controller

 Platform contains all the code and definitions to deal with machine-dependent things such as control
 pins, bed area, number of extruders, tolerable accelerations and speeds and so on.

 -----------------------------------------------------------------------------------------------------

 Version 0.1

 18 November 2012

 Adrian Bowyer
 RepRap Professional Ltd
 http://reprappro.com

 Licence: GPL

 ****************************************************************************************************/

#include "Platform.h"

#include "Heating/Heat.h"
#include "Movement/DDA.h"
#include "Movement/Move.h"
#include "Network.h"
#include "RepRap.h"
#include "Webserver.h"
#include "Libraries/Math/Isqrt.h"

//#include "sam/drivers/tc/tc.h"
//#include "sam/drivers/hsmci/hsmci.h"
//#include "sd_mmc.h"

#ifdef DUET_NG
# include "TMC2660.h"
# include "FirmwareUpdater.h"
#endif

#include <climits>
#include <malloc.h>

extern char _end;
char *sbrk(int i) { return 0; };

#ifdef DUET_NG
const uint16_t driverPowerOnAdcReading = (uint16_t)(4096 * 10.0/PowerFailVoltageRange);			// minimum voltage at which we initialise the drivers
const uint16_t driverPowerOffAdcReading = (uint16_t)(4096 * 9.5/PowerFailVoltageRange);			// voltages below this flag the drivers as unusable
const uint16_t driverOverVoltageAdcReading = (uint16_t)(4096 * 29.0/PowerFailVoltageRange);		// voltages above this cause driver shutdown
const uint16_t driverNormalVoltageAdcReading = (uint16_t)(4096 * 27.5/PowerFailVoltageRange);	// voltages at or below this are normal
#endif

const uint8_t memPattern = 0xA5;

static uint32_t fanInterruptCount = 0;				// accessed only in ISR, so no need to declare it volatile
const uint32_t fanMaxInterruptCount = 32;			// number of fan interrupts that we average over
static volatile uint32_t fanLastResetTime = 0;		// time (microseconds) at which we last reset the interrupt count, accessed inside and outside ISR
static volatile uint32_t fanInterval = 0;			// written by ISR, read outside the ISR

const float minStepPulseTiming = 0.2;				// we assume that we always generate step high and low times at least this wide without special action

const int Heater0LogicalPin = 0;
const int Fan0LogicalPin = 20;
const int EndstopXLogicalPin = 40;
const int Special0LogicalPin = 60;
const int DueX5Gpio0LogicalPin = 100;

//#define MOVE_DEBUG

#ifdef MOVE_DEBUG
unsigned int numInterruptsScheduled = 0;
unsigned int numInterruptsExecuted = 0;
uint32_t nextInterruptTime = 0;
uint32_t nextInterruptScheduledAt = 0;
uint32_t lastInterruptTime = 0;
#endif

// Global functions

// Urgent initialisation function
// This is called before general init has been done, and before constructors for C++ static data have been called.
// Therefore, be very careful what you do here!
void UrgentInit()
{
#ifdef DUET_NG
	// When the reset button is pressed on pre-production Duet WiFi boards, if the TMC2660 drivers were previously enabled then we get
	// uncommanded motor movements if the STEP lines pick up any noise. Try to reduce that by initialising the pins that control the drivers early here.
	// On the production boards the ENN line is pulled high and that prevents motor movements.
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		pinMode(STEP_PINS[drive], OUTPUT_LOW);
		pinMode(DIRECTION_PINS[drive], OUTPUT_LOW);
		pinMode(ENABLE_PINS[drive], OUTPUT_HIGH);
	}
#endif
}

// Arduino initialise and loop functions
// Put nothing in these other than calls to the RepRap equivalents
void setup()
{
	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char* heapend = sbrk(0);
	register const char * stack_ptr asm ("sp");
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}

	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	//SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

	// When doing a software reset, we disable the NRST input (User reset) to prevent the negative-going pulse that gets generated on it
	// being held in the capacitor and changing the reset reason from Software to User. So enable it again here. We hope that the reset signal
	// will have gone away by now.
#ifndef RSTC_MR_KEY_PASSWD
// Definition of RSTC_MR_KEY_PASSWD is missing in the SAM3X ASF files
# define RSTC_MR_KEY_PASSWD (0xA5u << 24)
#endif
	//RSTC->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN;	// ignore any signal on the NRST pin for now so that the reset reason will show as Software

	// Go on and do the main initialisation
	reprap.Init();
}

void loop()
{
	reprap.Spin();
}

extern "C"
{
	// This intercepts the 1ms system tick. It must return 'false', otherwise the Arduino core tick handler will be bypassed.
	int sysTickHook()
	{
		reprap.Tick();
		return 0;
	}

	// Exception handlers
	// By default the Usage Fault, Bus Fault and Memory Management fault handlers are not enabled,
	// so they escalate to a Hard Fault and we don't need to provide separate exception handlers for them.
	// We can get information on what caused a Hard Fault form the status registers.
	// Also get the program counter when the exception occurred.
	void prvGetRegistersFromStack(const uint32_t *pulFaultStackAddress)
	{
		reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::hardFault, pulFaultStackAddress + 5);
	}

	// The fault handler implementation calls a function called prvGetRegistersFromStack()
	void HardFault_Handler() {}

	// We could set up the following fault handlers to retrieve the program counter in the same way as for a Hard Fault,
	// however these exceptions are unlikely to occur, so for now we just report the exception type.
	void NMI_Handler        () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::NMI); }
	void SVC_Handler		() { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::otherFault); }
	void DebugMon_Handler   () { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::otherFault); }
	void PendSV_Handler		() { reprap.GetPlatform()->SoftwareReset((uint16_t)SoftwareResetReason::otherFault); }
}

// ZProbeParameters class

void ZProbeParameters::Init(float h)
{
	adcValue = Z_PROBE_AD_VALUE;
	xOffset = yOffset = 0.0;
	height = h;
	calibTemperature = 20.0;
	temperatureCoefficient = 0.0;	// no default temperature correction
	diveHeight = DEFAULT_Z_DIVE;
	probeSpeed = DEFAULT_PROBE_SPEED;
	travelSpeed = DEFAULT_TRAVEL_SPEED;
	recoveryTime = extraParam = 0.0;
	invertReading = false;
}

float ZProbeParameters::GetStopHeight(float temperature) const
{
	return ((temperature - calibTemperature) * temperatureCoefficient) + height;
}

bool ZProbeParameters::WriteParameters(FileStore *f, unsigned int probeType) const
{
	scratchString.printf("G31 T%u P%d X%.1f Y%.1f Z%.2f\n", probeType, adcValue, xOffset, yOffset, height);
	return f->Write(scratchString.Pointer());
}

//*************************************************************************************************
// Platform class

Platform::Platform() :
		board(DEFAULT_BOARD_TYPE), active(false), errorCodeBits(0),
		auxGCodeReply(nullptr), fileStructureInitialised(false), tickState(0), debugCode(0)
#ifdef DUET_NG
		, lastWarningMillis(0)
#endif
{
	// Output
	auxOutput = new OutputStack();
	aux2Output = new OutputStack();
	usbOutput = new OutputStack();

	// Files

	massStorage = NULL;// new MassStorage(this);

	for (size_t i = 0; i < MAX_FILES; i++)
	{
		files[i] = new FileStore(this);
	}
}

//*******************************************************************************************************************

void Platform::Init()
{
	// Deal with power first
	pinMode(ATX_POWER_PIN, OUTPUT_LOW);

	SetBoardType(BoardType::Auto);

	// Real-time clock
	realTime = 0;

	// Comms
	baudRates[0] = MAIN_BAUD_RATE;
	baudRates[1] = AUX_BAUD_RATE;
#if NUM_//SERIAL_CHANNELS >= 2
	baudRates[2] = AUX2_BAUD_RATE;
#endif
	commsParams[0] = 0;
	commsParams[1] = 1;							// by default we require a checksum on data from the aux port, to guard against overrun errors
#if NUM_//SERIAL_CHANNELS >= 2
	commsParams[2] = 0;
#endif

	auxDetected = false;
	auxSeq = 0;

	//SERIAL_MAIN_DEVICE.begin(baudRates[0]);
	//SERIAL_AUX_DEVICE.begin(baudRates[1]);		// this can't be done in the constructor because the Arduino port initialisation isn't complete at that point

	compatibility = marlin;						// default to Marlin because the common host programs expect the "OK" response to commands
	ARRAY_INIT(ipAddress, DefaultIpAddress);
	ARRAY_INIT(netMask, DefaultNetMask);
	ARRAY_INIT(gateWay, DefaultGateway);

#if defined(DUET_NG) && defined(DUET_WIFI)
	memset(macAddress, 0xFF, sizeof(macAddress));
#else
	ARRAY_INIT(macAddress, DefaultMacAddress);
#endif

	zProbeType = 0;	// Default is to use no Z probe switch
	zProbeAxes = Z_PROBE_AXES;
	SetZProbeDefaults();

	// We need to initialise at least some of the time stuff before we call MassStorage::Init()
	addToTime = 0.0;
	lastTimeCall = 0;
	lastTime = Time();
	longWait = lastTime;

	// File management
	//massStorage->Init();

	for (size_t file = 0; file < MAX_FILES; file++)
	{
		files[file]->Init();
	}

	fileStructureInitialised = true;

#if !defined(DUET_NG) && !defined(__RADDS__)
	//mcpDuet.begin();							// only call begin once in the entire execution, this begins the I2C comms on that channel for all objects
	//mcpExpansion.setMCP4461Address(0x2E);		// not required for mcpDuet, as this uses the default address
#endif

	// DRIVES
	ARRAY_INIT(endStopPins, END_STOP_PINS);
	ARRAY_INIT(maxFeedrates, MAX_FEEDRATES);
	ARRAY_INIT(accelerations, ACCELERATIONS);
	ARRAY_INIT(driveStepsPerUnit, DRIVE_STEPS_PER_UNIT);
	ARRAY_INIT(instantDvs, INSTANT_DVS);

#if !defined(DUET_NG) && !defined(__RADDS__)
	// Motor current setting on Duet 0.6 and 0.8.5
	ARRAY_INIT(potWipes, POT_WIPES);
	senseResistor = SENSE_RESISTOR;
	maxStepperDigipotVoltage = MAX_STEPPER_DIGIPOT_VOLTAGE;
	stepperDacVoltageRange = STEPPER_DAC_VOLTAGE_RANGE;
	stepperDacVoltageOffset = STEPPER_DAC_VOLTAGE_OFFSET;
#endif

	// Z PROBE
	zProbePin = Z_PROBE_PIN;
	zProbeAdcChannel = PinToAdcChannel(zProbePin);
	InitZProbe();		// this also sets up zProbeModulationPin

	// AXES
	ARRAY_INIT(axisMaxima, AXIS_MAXIMA);
	ARRAY_INIT(axisMinima, AXIS_MINIMA);

	idleCurrentFactor = DEFAULT_IDLE_CURRENT_FACTOR;

	// SD card interfaces
	for (size_t i = 0; i < NumSdCards; ++i)
	{
	}

	// Motors
	// Disable parallel writes to all pins. We re-enable them for the step pins.
	PIOA->PIO_OWDR = 0xFFFFFFFF;
	PIOB->PIO_OWDR = 0xFFFFFFFF;
	PIOC->PIO_OWDR = 0xFFFFFFFF;
	PIOD->PIO_OWDR = 0xFFFFFFFF;

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		enableValues[drive] = false;				// assume active low enable signal
		directions[drive] = true;					// drive moves forwards by default

		// Map axes and extruders straight through
		if (drive < MAX_AXES)
		{
			axisDrivers[drive].numDrivers = 1;
			axisDrivers[drive].driverNumbers[0] = (uint8_t)drive;
			endStopType[drive] =
#if defined(DUET_NG) || defined(__RADDS__)
									EndStopType::lowEndStop;	// default to low endstop
#else
									(drive == Y_AXIS)
									? EndStopType::lowEndStop	// for Ormerod 2/Huxley/Mendel compatibility
									: EndStopType::noEndStop;	// for Ormerod/Huxley/Mendel compatibility
#endif
			endStopLogicLevel[drive] = true;					// assume all endstops use active high logic e.g. normally-closed switch to ground
		}

		if (drive >= MIN_AXES)
		{
			extruderDrivers[drive - MIN_AXES] = (uint8_t)drive;
			SetPressureAdvance(drive - MIN_AXES, 0.0);
		}
		driveDriverBits[drive] = CalcDriverBitmap(drive);

		// Set up the control pins and endstops
		pinMode(STEP_PINS[drive], OUTPUT_LOW);
		pinMode(DIRECTION_PINS[drive], OUTPUT_LOW);
		//pinMode(ENABLE_PINS[drive], OUTPUT_HIGH);				// this is OK for the TMC2660 CS pins too

		const PinDescription& pinDesc = g_APinDescription[STEP_PINS[drive]];
		//pinDesc.pPort->PIO_OWER = pinDesc.ulPin;				// enable parallel writes to the step pins

		motorCurrents[drive] = 0.0;
		motorCurrentFraction[drive] = 1.0;
		driverState[drive] = DriverStatus::disabled;
	}

	slowDriverStepPulseClocks = 0;								// no extended driver timing configured yet
	slowDrivers = 0;											// assume no drivers need extended step pulse timing

#ifdef DUET_NG
	// Test for presence of a DueX2 or DueX5 expansion board and work out how many TMC2660 drivers we have
	// The SX1509B has an independent power on reset, so give it some time
	delay(200);
	expansionBoard = DuetExpansion::Init();
	switch (expansionBoard)
	{
	case ExpansionBoardType::DueX2:
		numTMC2660Drivers = 7;
		break;
	case ExpansionBoardType::DueX5:
		numTMC2660Drivers = 10;
		break;
	case ExpansionBoardType::none:
	case ExpansionBoardType::DueX0:
	default:
		numTMC2660Drivers = 5;									// assume that additional drivers are dumb enable/step/dir ones
		break;
	}

	// Initialise TMC2660 driver module
	driversPowered = false;
	TMC2660::Init(ENABLE_PINS, numTMC2660Drivers);

	// Set up the VSSA sense pin. Older Duet WiFis don't have it connected, so we enable the pulldown resistor to keep it inactive.
	{
		pinMode(VssaSensePin, INPUT_PULLUP);
		delayMicroseconds(10);
		const bool vssaHighVal = digitalRead(VssaSensePin);
		pinMode(VssaSensePin, INPUT_PULLDOWN);
		delayMicroseconds(10);
		const bool vssaLowVal = digitalRead(VssaSensePin);
		vssaSenseWorking = vssaLowVal || !vssaHighVal;
		if (vssaSenseWorking)
		{
			pinMode(VssaSensePin, INPUT);
		}
	}
#endif

	// Allow extrusion ancilliary PWM to use FAN0 even if FAN0 has not been disabled, for backwards compatibility
	extrusionAncilliaryPwmValue = 0.0;
	extrusionAncilliaryPwmFrequency = DefaultPinWritePwmFreq;
	extrusionAncilliaryPwmLogicalPin = Fan0LogicalPin;

	configuredHeaters = (DefaultBedHeater >= 0) ? (1 << DefaultBedHeater) : 0;
	heatSampleTicks = HEAT_SAMPLE_TIME * SecondsToMillis;

	// Fans
	InitFans();

	// Hotend configuration
	nozzleDiameter = NOZZLE_DIAMETER;
	filamentWidth = FILAMENT_WIDTH;

#if SUPPORT_INKJET
	// Inkjet

	inkjetBits = INKJET_BITS;
	if (inkjetBits >= 0)
	{
		inkjetFireMicroseconds = INKJET_FIRE_MICROSECONDS;
		inkjetDelayMicroseconds = INKJET_DELAY_MICROSECONDS;

		inkjetSerialOut = INKJET_//SERIAL_OUT;
		pinMode(inkjetSerialOut, OUTPUT_LOW);

		inkjetShiftClock = INKJET_SHIFT_CLOCK;
		pinMode(inkjetShiftClock, OUTPUT_LOW);

		inkjetStorageClock = INKJET_STORAGE_CLOCK;
		pinMode(inkjetStorageClock, OUTPUT_LOW);

		inkjetOutputEnable = INKJET_OUTPUT_ENABLE;
		pinMode(inkjetOutputEnable, OUTPUT_HIGH);

		inkjetClear = INKJET_CLEAR;
		pinMode(inkjetClear, OUTPUT_HIGH);
	}
#endif

	// MCU temperature monitoring - doesn't work in RADDS due to pin assignmentgs and SAM3X chip bug
#ifndef __RADDS__
	AnalogInEnableChannel(temperatureAdcChannel, true);
	currentMcuTemperature = highestMcuTemperature = 0;
	lowestMcuTemperature = 4095;
	mcuAlarmTemperature = 80.0;			// need to set the quite high here because the sensor is not be calibrated yet
#endif
	mcuTemperatureAdjust = 0.0;

#ifdef DUET_NG
	// Power monitoring
	vInMonitorAdcChannel = PinToAdcChannel(PowerMonitorVinDetectPin);
	pinMode(PowerMonitorVinDetectPin, AIN);
	AnalogInEnableChannel(vInMonitorAdcChannel, true);
	currentVin = highestVin = 0;
	lowestVin = 9999;
	numUnderVoltageEvents = numOverVoltageEvents = 0;
#endif

	// Clear the spare pin configuration

	// Kick everything off
	lastTime = Time();
	longWait = lastTime;
	InitialiseInterrupts();		// also sets 'active' to true
}

void Platform::InvalidateFiles(const FATFS *fs)
{
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		files[i]->Invalidate(fs);
	}
}

bool Platform::AnyFileOpen(const FATFS *fs) const
{
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (files[i]->IsOpenOn(fs))
		{
			return true;
		}
	}
	return false;
}

// Specify which thermistor channel a particular heater uses
void Platform::SetThermistorNumber(size_t heater, size_t thermistor)
{
	heaterTempChannels[heater] = thermistor;

	// Initialize the associated SPI temperature sensor?
	if (thermistor >= FirstThermocoupleChannel && thermistor < FirstThermocoupleChannel + MaxSpiTempSensors)
	{
	}
	else if (thermistor >= FirstRtdChannel && thermistor < FirstRtdChannel + MaxSpiTempSensors)
	{
	}

	//reprap.GetHeat()->ResetFault(heater);
}

int Platform::GetThermistorNumber(size_t heater) const
{
	return heaterTempChannels[heater];
}

void Platform::SetZProbeDefaults()
{
	switchZProbeParameters.Init(0.0);
	irZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
	alternateZProbeParameters.Init(Z_PROBE_STOP_HEIGHT);
}

void Platform::InitZProbe()
{
	zProbeOnFilter.Init(0);
	zProbeOffFilter.Init(0);

#if defined(DUET_NG) || defined(__RADDS__)
	zProbeModulationPin = Z_PROBE_MOD_PIN;
#else
	//zProbeModulationPin = (board == BoardType::Duet_07 || board == BoardType::Duet_085) ? Z_PROBE_MOD_PIN07 : Z_PROBE_MOD_PIN;
#endif


}

// Return the Z probe data.
// The ADC readings are 12 bits, so we convert them to 10-bit readings for compatibility with the old firmware.
int Platform::GetZProbeReading() const
{
	int zProbeVal = 0;			// initialised to avoid spurious compiler warning
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (zProbeType)
		{
		case 1:		// Simple or intelligent IR sensor
		case 3:		// Alternate sensor
		case 4:		// Switch connected to E0 endstop input
		case 5:		// Switch connected to Z probe input
		case 6:		// Switch connected to E1 endstop input
			zProbeVal = (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));
			break;

		case 2:		// Dumb modulated IR sensor.
			// We assume that zProbeOnFilter and zProbeOffFilter average the same number of readings.
			// Because of noise, it is possible to get a negative reading, so allow for this.
			zProbeVal = (int) (((int32_t) zProbeOnFilter.GetSum() - (int32_t) zProbeOffFilter.GetSum()) / (int)(4 * Z_PROBE_AVERAGE_READINGS));
			break;

		case 7:		// Delta humming probe
			zProbeVal = (int) ((zProbeOnFilter.GetSum() + zProbeOffFilter.GetSum()) / (8 * Z_PROBE_AVERAGE_READINGS));	//TODO this is temporary
			break;

		default:
			return 0;
		}
	}

	return (GetCurrentZProbeParameters().invertReading) ? 1000 - zProbeVal : zProbeVal;
}

// Return the Z probe secondary values.
int Platform::GetZProbeSecondaryValues(int& v1, int& v2)
{
	if (zProbeOnFilter.IsValid() && zProbeOffFilter.IsValid())
	{
		switch (zProbeType)
		{
		case 2:		// modulated IR sensor
			v1 = (int) (zProbeOnFilter.GetSum() / (4 * Z_PROBE_AVERAGE_READINGS));	// pass back the reading with IR turned on
			return 1;
		default:
			break;
		}
	}
	return 0;
}

void Platform::SetZProbeAxes(uint32_t axes)
{
	zProbeAxes = axes;
}

// Get our best estimate of the Z probe temperature
float Platform::GetZProbeTemperature()
{
	const int8_t bedHeater = reprap.GetHeat()->GetBedHeater();
	if (bedHeater >= 0)
	{
		TemperatureError err;
		const float temp = GetTemperature(bedHeater, err);
		if (err == TemperatureError::success)
		{
			return temp;
		}
	}
	return 25.0;							// assume 25C if we can't read he bed temperature
}

float Platform::ZProbeStopHeight()
{
	return GetCurrentZProbeParameters().GetStopHeight(GetZProbeTemperature());
}

float Platform::GetZProbeDiveHeight() const
{
	return GetCurrentZProbeParameters().diveHeight;
}

float Platform::GetZProbeStartingHeight()
{
	const ZProbeParameters& params = GetCurrentZProbeParameters();
	return params.diveHeight + max<float>(params.GetStopHeight(GetZProbeTemperature()), 0.0);
}

float Platform::GetZProbeTravelSpeed() const
{
	return GetCurrentZProbeParameters().travelSpeed;
}

void Platform::SetZProbeType(int pt)
{
	zProbeType = (pt >= 0 && pt <= 7) ? pt : 0;
	InitZProbe();
}

void Platform::SetProbing(bool isProbing)
{
	if (zProbeType > 3)
	{
		// For Z probe types other than 1/2/3 we set the modulation pin high at the start of a probing move and low at the end
		digitalWrite(zProbeModulationPin, isProbing);
	}
}

const ZProbeParameters& Platform::GetZProbeParameters(int32_t probeType) const
{
	switch (probeType)
	{
	case 1:
	case 2:
	case 5:
		return irZProbeParameters;
	case 3:
	case 7:
		return alternateZProbeParameters;
	case 4:
	case 6:
	default:
		return switchZProbeParameters;
	}
}

void Platform::SetZProbeParameters(int32_t probeType, const ZProbeParameters& params)
{
	switch (probeType)
	{
	case 1:
	case 2:
	case 5:
		irZProbeParameters = params;
		break;

	case 3:
	case 7:
		alternateZProbeParameters = params;
		break;

	case 4:
	case 6:
	default:
		switchZProbeParameters = params;
		break;
	}
}

// Return true if the specified point is accessible to the Z probe
bool Platform::IsAccessibleProbePoint(float x, float y) const
{
	x -= GetCurrentZProbeParameters().xOffset;
	y -= GetCurrentZProbeParameters().yOffset;
	return (reprap.GetMove()->IsDeltaMode())
			? x * x + y * y < reprap.GetMove()->GetDeltaParams().GetPrintRadiusSquared()
			: x >= axisMinima[X_AXIS] && y >= axisMinima[Y_AXIS] && x <= axisMaxima[X_AXIS] && y <= axisMaxima[Y_AXIS];
}

// Return true if we must home X and Y before we home Z (i.e. we are using a bed probe)
bool Platform::MustHomeXYBeforeZ() const
{
	return (zProbeType != 0) && ((zProbeAxes & (1 << Z_AXIS)) != 0);
}

// Check the prerequisites for updating the main firmware. Return True if satisfied, else print as message and return false.
bool Platform::CheckFirmwareUpdatePrerequisites()
{
	return true;
}

// Update the firmware. Prerequisites should be checked before calling this.
void Platform::UpdateFirmware()
{
	
}

// Send the beep command to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::Beep(int freq, int ms)
{
	MessageF(AUX_MESSAGE, "{\"beep_freq\":%d,\"beep_length\":%d}\n", freq, ms);
}

// Send a short message to the aux channel. There is no flow control on this port, so it can't block for long.
void Platform::SendMessage(const char* msg)
{
	OutputBuffer *buf;
	if (OutputBuffer::Allocate(buf))
	{
		buf->copy("{\"message\":");
		buf->EncodeString(msg, strlen(msg), false, true);
		buf->cat("}\n");
		Message(AUX_MESSAGE, buf);
	}
}

// Note: the use of floating point time will cause the resolution to degrade over time.
// For example, 1ms time resolution will only be available for about half an hour from startup.
// Personally, I (dc42) would rather just maintain and provide the time in milliseconds in a uint32_t.
// This would wrap round after about 49 days, but that isn't difficult to handle.
float Platform::Time()
{
	unsigned long now = micros();
	if (now < lastTimeCall) // Has timer overflowed?
	{
		addToTime += ((float) ULONG_MAX) * TIME_FROM_REPRAP;
	}
	lastTimeCall = now;
	return addToTime + TIME_FROM_REPRAP * (float) now;
}

void Platform::Exit()
{
	// Close all files
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		while (files[i]->inUse)
		{
			files[i]->Close();
		}
	}

	// Release the aux output stack (should release the others too!)
	while (auxGCodeReply != nullptr)
	{
		auxGCodeReply = OutputBuffer::Release(auxGCodeReply);
	}

	// Stop processing data. Don't try to send a message because it will probably never get there.
	active = false;

	// Close down USB and serial ports
	//SERIAL_MAIN_DEVICE.end();
	//SERIAL_AUX_DEVICE.end();

}

Compatibility Platform::Emulating() const
{
	return (compatibility == reprapFirmware) ? me : compatibility;
}

void Platform::SetEmulating(Compatibility c)
{
	if (c != me && c != reprapFirmware && c != marlin)
	{
		Message(GENERIC_MESSAGE, "Attempt to emulate unsupported firmware.\n");
		return;
	}
	if (c == reprapFirmware)
	{
		c = me;
	}
	compatibility = c;
}

void Platform::UpdateNetworkAddress(byte dst[4], const byte src[4])
{
	for (uint8_t i = 0; i < 4; i++)
	{
		dst[i] = src[i];
	}
}

void Platform::SetIPAddress(byte ip[])
{
	UpdateNetworkAddress(ipAddress, ip);
}

void Platform::SetGateWay(byte gw[])
{
	UpdateNetworkAddress(gateWay, gw);
}

void Platform::SetNetMask(byte nm[])
{
	UpdateNetworkAddress(netMask, nm);
}

// Flush messages to USB and aux, returning true if there is more to send
bool Platform::FlushMessages()
{
	// Write non-blocking data to the AUX line
	OutputBuffer *auxOutputBuffer = auxOutput->GetFirstItem();
	if (auxOutputBuffer != nullptr)
	{
		size_t bytesToWrite = 0;
		if (bytesToWrite > 0)
		{
			//SERIAL_AUX_DEVICE.write(auxOutputBuffer->Read(bytesToWrite), bytesToWrite);
		}

		if (auxOutputBuffer->BytesLeft() == 0)
		{
			auxOutputBuffer = OutputBuffer::Release(auxOutputBuffer);
			auxOutput->SetFirstItem(auxOutputBuffer);
		}
	}

	// Write non-blocking data to the second AUX line
	OutputBuffer *aux2OutputBuffer = aux2Output->GetFirstItem();
	if (aux2OutputBuffer != nullptr)
	{
		aux2OutputBuffer = OutputBuffer::Release(aux2OutputBuffer);
	}

	// Write non-blocking data to the USB line
	OutputBuffer *usbOutputBuffer = usbOutput->GetFirstItem();
	if (usbOutputBuffer != nullptr)
	{
	}

	return auxOutput->GetFirstItem() != nullptr
		|| aux2Output->GetFirstItem() != nullptr
		|| usbOutput->GetFirstItem() != nullptr;
}

void Platform::Spin()
{
	if (!active)
		return;

	// Check if any files are supposed to be closed
	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (files[i]->closeRequested)
		{
			// We cannot do this in ISRs, so do it here
			files[i]->Close();
		}
	}

	// Try to flush messages to serial ports
	(void)FlushMessages();

	// Thermostatically-controlled fans
	for (size_t fan = 0; fan < NUM_FANS; ++fan)
	{
		//fans[fan].Check();
	}

	// Check the MCU max and min temperatures
#ifndef __RADDS__
	if (currentMcuTemperature > highestMcuTemperature)
	{
		highestMcuTemperature= currentMcuTemperature;
	}
	if (currentMcuTemperature < lowestMcuTemperature && currentMcuTemperature != 0)
	{
		lowestMcuTemperature = currentMcuTemperature;
	}
#endif

	// Diagnostics test
	if (debugCode == (int)DiagnosticTestType::TestSpinLockup)
	{
		for (;;) {}
	}

#ifdef DUET_NG
	// Check whether the TMC drivers need to be initialised.
	// The tick ISR also looks for over-voltage events, but it just disables the driver without changing driversPowerd or numOverVoltageEvents
	if (driversPowered)
	{
		if (currentVin < driverPowerOffAdcReading)
		{
			++numUnderVoltageEvents;
			driversPowered = false;
		}
		else if (currentVin > driverOverVoltageAdcReading)
		{
			driversPowered = false;
			++numOverVoltageEvents;
		}
	}
	else if (currentVin >= driverPowerOnAdcReading && currentVin <= driverNormalVoltageAdcReading)
	{
		driversPowered = true;
	}
	TMC2660::SetDriversPowered(driversPowered);

	// Check for a VSSA fault
	const uint32_t now = millis();
	if (vssaSenseWorking && now - lastWarningMillis > MinimumWarningInterval && digitalRead(VssaSensePin))
	{
		Message(GENERIC_MESSAGE, "Error: VSSA fault\n");
		lastWarningMillis = now;
	}
#endif

	// Update the time
	if (realTime != 0)
	{
		if (millis() - timeLastUpdatedMillis >= 1000)
		{
			++realTime;							// this assumes that time_t is a seconds-since-epoch counter, which is not guaranteed by the C standard
			timeLastUpdatedMillis += 1000;
		}
	}

	ClassReport(longWait);
}

// Perform a software reset. 'stk' points to the program counter on the stack if the cause is an exception, otherwise it is nullptr.
void Platform::SoftwareReset(uint16_t reason, const uint32_t *stk)
{

}

//*****************************************************************************************************************
// Interrupts

#ifndef DUET_NG
void NETWORK_TC_HANDLER()
{
	//reprap.GetNetwork()->Interrupt();
}
#endif

void FanInterrupt()
{
	++fanInterruptCount;
	if (fanInterruptCount == fanMaxInterruptCount)
	{
		uint32_t now = micros();
		fanInterval = now - fanLastResetTime;
		fanLastResetTime = now;
		fanInterruptCount = 0;
	}
}

void Platform::InitialiseInterrupts()
{
	// Tick interrupt for ADC conversions
	tickState = 0;
	currentHeater = 0;

	active = true;							// this enables the tick interrupt, which keeps the watchdog happy
}

#if 0	// not used
void Platform::DisableInterrupts()
{
	NVIC_DisableIRQ(STEP_IRQN);
#ifdef DUET_NG
	NVIC_DisableIRQ(NETWORK_IRQN);
#endif
}
#endif

//*************************************************************************************************

// Debugging variables
//extern "C" uint32_t longestWriteWaitTime, shortestWriteWaitTime, longestReadWaitTime, shortestReadWaitTime;
//extern uint32_t maxRead, maxWrite;

// This diagnostics function is the first to be called, so it calls Message to start with.
// All other messages generated by this and other diagnostics functions must call AppendMessage.
void Platform::Diagnostics(MessageType mtype)
{

}

void Platform::DiagnosticTest(int d)
{
}

uint32_t _estack=1;		// this is defined in the linker script

// Return the stack usage and amount of memory that has never been used, in bytes
void Platform::GetStackUsage(uint32_t* currentStack, uint32_t* maxStack, uint32_t* neverUsed) const
{

}

void Platform::ClassReport(float &lastTime)
{
	const Module spinningModule = reprap.GetSpinningModule();
	if (reprap.Debug(spinningModule))
	{
		if (Time() - lastTime >= LONG_TIME)
		{
			lastTime = Time();
			MessageF(HOST_MESSAGE, "Class %s spinning.\n", moduleName[spinningModule]);
		}
	}
}

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

// See http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation

// BETA is the B value
// RS is the value of the series resistor in ohms
// R_INF is R0.exp(-BETA/T0), where R0 is the thermistor resistance at T0 (T0 is in kelvin)
// Normally T0 is 298.15K (25 C).

// If the A->D converter has a range of 0..1023 and the measured voltage is V (between 0 and 1023)
// then the thermistor resistance, R = V.RS/(1024 - V)
// and the temperature, T = BETA/ln(R/R_INF)
// To get degrees celsius (instead of kelvin) add -273.15 to T

// Result is in degrees celsius

float Platform::GetTemperature(size_t heater, TemperatureError& err)
{
	// Note that at this point we're actually getting an averaged ADC read, not a "raw" temp.  For thermistors,
	// we're getting an averaged voltage reading which we'll convert to a temperature.
	if (IsThermistorChannel(heater))
	{
		const volatile ThermistorAveragingFilter& filter = thermistorFilters[heater];
		if (filter.IsValid())
		{
			const int32_t averagedReading = filter.GetSum()/(ThermistorAverageReadings >> Thermistor::AdcOversampleBits);
			const float temp = thermistors[heater].CalcTemperature(averagedReading);

			if (temp < MinimumConnectedTemperature)
			{
				// thermistor is disconnected
				err = TemperatureError::openCircuit;
				return ABS_ZERO;
			}

			err = TemperatureError::success;
			return temp;
		}

		// Filter is not ready yet
		err = TemperatureError::busBusy;
		return BAD_ERROR_TEMPERATURE;
	}

	err = TemperatureError::unknownChannel;
	return BAD_ERROR_TEMPERATURE;
}

// See if we need to turn on a thermostatically-controlled fan
bool Platform::AnyHeaterHot(uint16_t heaters, float t)
{
	return true;
}

// Power is a fraction in [0,1]
void Platform::SetHeater(size_t heater, float power)
{
}

void Platform::UpdateConfiguredHeaters()
{
	configuredHeaters = 0;
}

EndStopHit Platform::Stopped(size_t drive) const
{
	if (drive < DRIVES && endStopPins[drive] != NoPin)
	{
		if (drive >= reprap.GetGCodes()->GetNumAxes())
		{
			// Endstop not used for an axis, so no configuration data available.
			// To allow us to see its status in DWC, pretend it is configured as a high-end active high endstop.
			if (ReadPin(endStopPins[drive]))
			{
				return EndStopHit::highHit;
			}
		}
		else if (endStopType[drive] == EndStopType::noEndStop)
		{
			// No homing switch is configured for this axis, so see if we should use the Z probe
			if (zProbeType > 0 && drive < reprap.GetGCodes()->GetNumAxes() && (zProbeAxes & (1 << drive)) != 0)
			{
				return GetZProbeResult();			// using the Z probe as a low homing stop for this axis, so just get its result
			}
		}
		else if (ReadPin(endStopPins[drive]) == endStopLogicLevel[drive])
		{
			return (endStopType[drive] == EndStopType::highEndStop) ? EndStopHit::highHit : EndStopHit::lowHit;
		}
	}
	return EndStopHit::noStop;
}

// Get the statues of all the endstop inputs, regardless of what they are used for. Used for triggers.
uint32_t Platform::GetAllEndstopStates() const
{
	uint32_t rslt = 0;
	for (unsigned int drive = 0; drive < DRIVES; ++drive)
	{
		const Pin pin = endStopPins[drive];
		if (pin != NoPin && ReadPin(pin))
		{
			rslt |= (1 << drive);
		}
	}
	return rslt;
}

// Return the Z probe result. We assume that if the Z probe is used as an endstop, it is used as the low stop.
EndStopHit Platform::GetZProbeResult() const
{
	const int zProbeVal = GetZProbeReading();
	const int zProbeADValue = GetCurrentZProbeParameters().adcValue;
	return (zProbeVal >= zProbeADValue) ? EndStopHit::lowHit
			: (zProbeVal * 10 >= zProbeADValue * 9) ? EndStopHit::lowNear	// if we are at/above 90% of the target value
				: EndStopHit::noStop;
}

// Write the Z probe parameters to file
bool Platform::WriteZProbeParameters(FileStore *f) const
{
	bool ok = f->Write("; Z probe parameters\n");
	if (ok)
	{
		ok = irZProbeParameters.WriteParameters(f, 1);
	}
	if (ok)
	{
		ok = alternateZProbeParameters.WriteParameters(f, 3);
	}
	if (ok)
	{
		ok = switchZProbeParameters.WriteParameters(f, 4);
	}
	return ok;
}

// This is called from the step ISR as well as other places, so keep it fast
void Platform::SetDirection(size_t drive, bool direction)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			SetDriverDirection(axisDrivers[drive].driverNumbers[i], direction);
		}
	}
	else if (drive < DRIVES)
	{
		SetDriverDirection(extruderDrivers[drive - numAxes], direction);
	}
}

// Enable a driver. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDriver(size_t driver)
{
	if (driver < DRIVES && driverState[driver] != DriverStatus::enabled)
	{
		driverState[driver] = DriverStatus::enabled;
		UpdateMotorCurrent(driver);						// the current may have been reduced by the idle timeout

#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			TMC2660::EnableDrive(driver, true);
		}
		else
		{
#endif
#if defined(DUET_NG)
		}
#endif
	}
}

// Disable a driver
void Platform::DisableDriver(size_t driver)
{
	if (driver < DRIVES)
	{
#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			TMC2660::EnableDrive(driver, false);
		}
		else
		{
#endif
#if defined(DUET_NG)
		}
#endif
		driverState[driver] = DriverStatus::disabled;
	}
}

// Enable the drivers for a drive. Must not be called from an ISR, or with interrupts disabled.
void Platform::EnableDrive(size_t drive)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			EnableDriver(axisDrivers[drive].driverNumbers[i]);
		}
	}
	else if (drive < DRIVES)
	{
		EnableDriver(extruderDrivers[drive - numAxes]);
	}
}

// Disable the drivers for a drive
void Platform::DisableDrive(size_t drive)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			DisableDriver(axisDrivers[drive].driverNumbers[i]);
		}
	}
	else if (drive < DRIVES)
	{
		DisableDriver(extruderDrivers[drive - numAxes]);
	}
}

// Set drives to idle hold if they are enabled. If a drive is disabled, leave it alone.
// Must not be called from an ISR, or with interrupts disabled.
void Platform::SetDriversIdle()
{
	for (size_t driver = 0; driver < DRIVES; ++driver)
	{
		if (driverState[driver] == DriverStatus::enabled)
		{
			driverState[driver] = DriverStatus::idle;
			UpdateMotorCurrent(driver);
		}
	}
}

// Set the current for a drive. Current is in mA.
void Platform::SetDriverCurrent(size_t driver, float currentOrPercent, bool isPercent)
{
	if (driver < DRIVES)
	{
		if (isPercent)
		{
			motorCurrentFraction[driver] = 0.01 * currentOrPercent;
		}
		else
		{
			motorCurrents[driver] = currentOrPercent;
		}
		UpdateMotorCurrent(driver);
	}
}

// Set the current for all drivers on an axis or extruder. Current is in mA.
void Platform::SetMotorCurrent(size_t drive, float currentOrPercent, bool isPercent)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			SetDriverCurrent(axisDrivers[drive].driverNumbers[i], currentOrPercent, isPercent);
		}

	}
	else if (drive < DRIVES)
	{
		SetDriverCurrent(extruderDrivers[drive - numAxes], currentOrPercent, isPercent);
	}
}

// This must not be called from an ISR, or with interrupts disabled.
void Platform::UpdateMotorCurrent(size_t driver)
{
	if (driver < DRIVES)
	{
		float current = motorCurrents[driver];
		if (driverState[driver] == DriverStatus::idle)
		{
			current *= idleCurrentFactor;
		}
		else
		{
			current *= motorCurrentFraction[driver];
		}

#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			TMC2660::SetCurrent(driver, current);
		}
		// else we can't set the current
#elif defined (__RADDS__)
		// we can't set the current on RADDS
#else
		unsigned short pot = (unsigned short)((0.256*current*8.0*senseResistor + maxStepperDigipotVoltage/2)/maxStepperDigipotVoltage);
		if (driver < 4)
		{
			//mcpDuet.setNonVolatileWiper(potWipes[driver], pot);
			//mcpDuet.setVolatileWiper(potWipes[driver], pot);
		}
		else
		{
# ifndef DUET_NG
			if (board == BoardType::Duet_085)
			{
# endif
				// Extruder 0 is on DAC channel 0
				if (driver == 4)
				{
					const float dacVoltage = max<float>(current * 0.008 * senseResistor + stepperDacVoltageOffset, 0.0);	// the voltage we want from the DAC relative to its minimum
					const float dac = dacVoltage/stepperDacVoltageRange;
# ifdef DUET_NG
					AnalogOut(DAC1, dac);
# else
# endif
				}
				else
				{
					//mcpExpansion.setNonVolatileWiper(potWipes[driver-1], pot);
					//mcpExpansion.setVolatileWiper(potWipes[driver-1], pot);
				}
# ifndef DUET_NG
			}
			else if (driver < 8)		// on a Duet 0.6 we have a maximum of 8 drives
			{
				//mcpExpansion.setNonVolatileWiper(potWipes[driver], pot);
				//mcpExpansion.setVolatileWiper(potWipes[driver], pot);
			}
# endif
		}
#endif
	}
}

// Get the configured motor current for a drive.
// Currently we don't allow multiple motors on a single axis to have different currents, so we can just return the current for the first one.
float Platform::GetMotorCurrent(size_t drive, bool isPercent) const
{
	if (drive < DRIVES)
	{
		const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
		const uint8_t driver = (drive < numAxes) ? axisDrivers[drive].driverNumbers[0] : extruderDrivers[drive - numAxes];
		if (driver < DRIVES)
		{
			return (isPercent) ? motorCurrentFraction[driver] * 100.0 : motorCurrents[driver];
		}
	}
	return 0.0;
}

// Set the motor idle current factor
void Platform::SetIdleCurrentFactor(float f)
{
	idleCurrentFactor = f;
	for (size_t driver = 0; driver < DRIVES; ++driver)
	{
		if (driverState[driver] == DriverStatus::idle)
		{
			UpdateMotorCurrent(driver);
		}
	}
}

// Set the microstepping for a driver, returning true if successful
bool Platform::SetDriverMicrostepping(size_t driver, int microsteps, int mode)
{
	if (driver < DRIVES)
	{
#if defined(DUET_NG)
		if (driver < numTMC2660Drivers)
		{
			return TMC2660::SetMicrostepping(driver, microsteps, mode);
		}
		else
		{
# endif
			// Other drivers only support x16 microstepping.
			// We ignore the interpolation on/off parameter so that e.g. M350 I1 E16:128 won't give an error if E1 supports interpolation but E0 doesn't.
			return microsteps == 16;
#if defined(DUET_NG)
		}
#endif
	}
	return false;
}

// Set the microstepping, returning true if successful. All drivers for the same axis must use the same microstepping.
bool Platform::SetMicrostepping(size_t drive, int microsteps, int mode)
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		bool ok = true;
		for (size_t i = 0; i < axisDrivers[drive].numDrivers; ++i)
		{
			ok = SetDriverMicrostepping(axisDrivers[drive].driverNumbers[i], microsteps, mode) && ok;
		}
		return ok;
	}
	else if (drive < DRIVES)
	{
		return SetDriverMicrostepping(extruderDrivers[drive - numAxes], microsteps, mode);
	}
	return false;
}

// Get the microstepping for a driver
unsigned int Platform::GetDriverMicrostepping(size_t driver, bool& interpolation) const
{
#if defined(DUET_NG)
	if (driver < numTMC2660Drivers)
	{
		return TMC2660::GetMicrostepping(driver, interpolation);
	}
#endif
	// On-board drivers only support x16 microstepping without interpolation
	interpolation = false;
	return 16;
}

// Get the microstepping for an axis or extruder
unsigned int Platform::GetMicrostepping(size_t drive, bool& interpolation) const
{
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive < numAxes)
	{
		return GetDriverMicrostepping(axisDrivers[drive].driverNumbers[0], interpolation);
	}
	else if (drive < DRIVES)
	{
		return GetDriverMicrostepping(extruderDrivers[drive - numAxes], interpolation);
	}
	else
	{
		interpolation = false;
		return 16;
	}
}

void Platform::SetAxisDriversConfig(size_t drive, const AxisDriversConfig& config)
{
	axisDrivers[drive] = config;
	uint32_t bitmap = 0;
	for (size_t i = 0; i < config.numDrivers; ++i)
	{
		bitmap |= CalcDriverBitmap(config.driverNumbers[i]);
	}
	driveDriverBits[drive] = bitmap;
}

// Map an extruder to a driver
void Platform::SetExtruderDriver(size_t extruder, uint8_t driver)
{
	extruderDrivers[extruder] = driver;
	driveDriverBits[extruder + reprap.GetGCodes()->GetNumAxes()] = CalcDriverBitmap(driver);
}

void Platform::SetDriverStepTiming(size_t driver, float microseconds)
{
	if (microseconds < minStepPulseTiming)
	{
		slowDrivers &= ~CalcDriverBitmap(driver);						// this drive does not need extended timing
	}
	else
	{
		const uint32_t clocks = (uint32_t)(((float)DDA::stepClockRate * microseconds/1000000.0) + 0.99);	// convert microseconds to step clocks, rounding up
		if (clocks > slowDriverStepPulseClocks)
		{
			slowDriverStepPulseClocks = clocks;
		}
		slowDrivers |= CalcDriverBitmap(driver);						// this drive does need extended timing
	}
}

float Platform::GetDriverStepTiming(size_t driver) const
{
	return ((slowDrivers & CalcDriverBitmap(driver)) != 0)
			? (float)slowDriverStepPulseClocks * 1000000.0/(float)DDA::stepClockRate
			: 0.0;
}

// Get current cooling fan speed on a scale between 0 and 1
float Platform::GetFanValue(size_t fan) const
{
	return -1;
}

// This is a bit of a compromise - old RepRaps used fan speeds in the range
// [0, 255], which is very hardware dependent.  It makes much more sense
// to specify speeds in [0.0, 1.0].  This looks at the value supplied (which
// the G Code reader will get right for a float or an int) and attempts to
// do the right thing whichever the user has done.
void Platform::SetFanValue(size_t fan, float speed)
{
	if (fan < NUM_FANS)
	{
		//fans[fan].SetValue(speed);
	}
}

#if !defined(DUET_NG) && !defined(__RADDS__)

// Enable or disable the fan that shares its PWM pin with the last heater. Called when we disable or enable the last heater.
void Platform::EnableSharedFan(bool enable)
{
	const size_t sharedFanNumber = NUM_FANS - 1;
}

#endif


// Get current fan RPM
float Platform::GetFanRPM()
{
	// The ISR sets fanInterval to the number of microseconds it took to get fanMaxInterruptCount interrupts.
	// We get 2 tacho pulses per revolution, hence 2 interrupts per revolution.
	// However, if the fan stops then we get no interrupts and fanInterval stops getting updated.
	// We must recognise this and return zero.
	return (fanInterval != 0 && micros() - fanLastResetTime < 3000000U)		// if we have a reading and it is less than 3 second old
			? (float)((30000000U * fanMaxInterruptCount)/fanInterval)		// then calculate RPM assuming 2 interrupts per rev
			: 0.0;															// else assume fan is off or tacho not connected
}

bool Platform::FansHardwareInverted(size_t fanNumber) const
{
#if defined(DUET_NG) || defined(__RADDS__)
	return false;
#else
	// The cooling fan output pin gets inverted on a Duet 0.6 or 0.7.
	// We allow a second fan controlled by a mosfet on the PC4 pin, which is not inverted.
	return fanNumber == 0 && (board == BoardType::Duet_06 || board == BoardType::Duet_07);
#endif
}

void Platform::InitFans()
{
	for (size_t i = 0; i < NUM_FANS; ++i)
	{
	}

	if (NUM_FANS > 1)
	{
#ifdef DUET_NG
		// Set fan 1 to be thermostatic by default, monitoring all heaters except the default bed heater
		fans[1].SetHeatersMonitored(0xFFFF & ~(1 << DefaultBedHeater));
		fans[1].SetValue(1.0);												// set it full on
#else
		// Fan 1 on the Duet 0.8.5 shares its control pin with heater 6. Set it full on to make sure the heater (if present) is off.
		//fans[1].SetValue(1.0);												// set it full on
#endif
	}

	lastRpmResetTime = 0.0;
	if (coolingFanRpmPin != NoPin)
	{
	}
}

void Platform::SetMACAddress(uint8_t mac[])
{
	for (size_t i = 0; i < 6; i++)
	{
		macAddress[i] = mac[i];
	}
}

//-----------------------------------------------------------------------------------------------------

FileStore* Platform::GetFileStore(const char* directory, const char* fileName, bool write)
{
	if (!fileStructureInitialised)
	{
		return nullptr;
	}

	for (size_t i = 0; i < MAX_FILES; i++)
	{
		if (!files[i]->inUse)
		{
			if (files[i]->Open(directory, fileName, write))
			{
				files[i]->inUse = true;
				return files[i];
			}
			else
			{
				return nullptr;
			}
		}
	}
	Message(HOST_MESSAGE, "Max open file count exceeded.\n");
	return NULL;
}

void Platform::AppendAuxReply(const char *msg)
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (msg[0] != 0 && HaveAux())
	{
		if (msg[0] == '{')
		{
			// JSON responses are always sent directly to the AUX device
			OutputBuffer *buf;
			if (OutputBuffer::Allocate(buf))
			{
				buf->copy(msg);
				auxOutput->Push(buf);
			}
		}
		else
		{
			// Regular text-based responses for AUX are currently stored and processed by M105/M408
			if (auxGCodeReply != nullptr || OutputBuffer::Allocate(auxGCodeReply))
			{
				auxSeq++;
				auxGCodeReply->cat(msg);
			}
		}
	}
}

void Platform::AppendAuxReply(OutputBuffer *reply)
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (reply == nullptr || reply->Length() == 0 || !HaveAux())
	{
		OutputBuffer::ReleaseAll(reply);
	}
	else if ((*reply)[0] == '{')
	{
		// JSON responses are always sent directly to the AUX device
		// For big responses it makes sense to write big chunks of data in portions. Store this data here
		auxOutput->Push(reply);
	}
	else
	{
		// Other responses are stored for M105/M408
		auxSeq++;
		if (auxGCodeReply == nullptr)
		{
			auxGCodeReply = reply;
		}
		else
		{
			auxGCodeReply->Append(reply);
		}
	}
}


bool Platform::AtxPower() const
{
	return ReadPin(ATX_POWER_PIN);
}

void Platform::SetAtxPower(bool on)
{
	WriteDigital(ATX_POWER_PIN, on);
}


void Platform::SetPressureAdvance(size_t extruder, float factor)
{
	if (extruder < MaxExtruders)
	{
		pressureAdvance[extruder] = factor;
	}
}

float Platform::ActualInstantDv(size_t drive) const
{
	const float idv = instantDvs[drive];
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	if (drive >= numAxes)
	{
		const float eComp = pressureAdvance[drive - numAxes];
		// If we are using pressure advance then we need to limit the extruder instantDv to avoid velocity mismatches.
		// Assume that we want the extruder motor position to be accurate to within 0.01mm of extrusion.
		// TODO remove this l imit and add/remove steps to the previous and/or next move instead
		return (eComp <= 0.0) ? idv : min<float>(idv, 0.01/eComp);
	}
	else
	{
		return idv;
	}
}

void Platform::SetBaudRate(size_t chan, uint32_t br)
{
}

uint32_t Platform::GetBaudRate(size_t chan) const
{
	return 0;
}

void Platform::SetCommsProperties(size_t chan, uint32_t cp)
{
}

uint32_t Platform::GetCommsProperties(size_t chan) const
{
	return 0;
}

// Re-initialise a serial channel.
// Ideally, this would be part of the Line class. However, the Arduino core inexplicably fails to make the serial I/O begin() and end() members
// virtual functions of a base class, which makes that difficult to do.
void Platform::ResetChannel(size_t chan)
{
	switch(chan)
	{
	case 0:
		//SERIAL_MAIN_DEVICE.end();
		//SERIAL_MAIN_DEVICE.begin(baudRates[0]);
		break;
	case 1:
		//SERIAL_AUX_DEVICE.end();
		//SERIAL_AUX_DEVICE.begin(baudRates[1]);
		break;
	default:
		break;
	}
}

void Platform::SetBoardType(BoardType bt)
{
	if (bt == BoardType::Auto)
	{
#if defined(DUET_NG) && defined(DUET_WIFI)
		board = BoardType::DuetWiFi_10;
#elif defined(DUET_NG) && defined(DUET_ETHERNET)
		board = BoardType::DuetEthernet_10;
#elif defined(__RADDS__)
		board = BoardType::RADDS_15;
#else
		// Determine whether this is a Duet 0.6 or a Duet 0.8.5 board.
		// If it is a 0.85 board then DAC0 (AKA digital pin 67) is connected to ground via a diode and a 2.15K resistor.
		// So we enable the pullup (value 100K-150K) on pin 67 and read it, expecting a LOW on a 0.8.5 board and a HIGH on a 0.6 board.
		// This may fail if anyone connects a load to the DAC0 pin on a Duet 0.6, hence we implement board selection in M115 as well.
#endif
	}
	else
	{
	}

	if (active)
	{
		InitZProbe();							// select and initialise the Z probe modulation pin
		InitFans();								// select whether cooling is inverted or not
	}
}

// Get a string describing the electronics
const char* Platform::GetElectronicsString() const
{
	switch (board)
	{
#if defined(DUET_NG) && defined(DUET_WIFI)
	case BoardType::DuetWiFi_10:			return "Duet WiFi 1.0";
#elif defined(DUET_NG) && defined(DUET_ETHERNET)
	case BoardType::DuetEthernet_10:		return "Duet Ethernet 1.0";
#elif defined(__RADDS__)
	case BoardType::RADDS_15:				return "RADDS 1.5";
#else
	case BoardType::Duet_06:				return "Duet 0.6";
	case BoardType::Duet_07:				return "Duet 0.7";
	case BoardType::Duet_085:				return "Duet 0.85";
#endif
	default:								return "Unidentified";
	}
}

// Get the board string
const char* Platform::GetBoardString() const
{
	switch (board)
	{
#if defined(DUET_NG) && defined(DUET_WIFI)
	case BoardType::DuetWiFi_10:			return "duetwifi10";
#elif defined(DUET_NG) && defined(DUET_ETHERNET)
	case BoardType::DuetEthernet_10:		return "duetethernet10";
#elif defined(__RADDS__)
	case BoardType::RADDS_15:				return "radds15";
#else
	case BoardType::Duet_06:				return "duet06";
	case BoardType::Duet_07:				return "duet07";
	case BoardType::Duet_085:				return "duet085";
#endif
	default:								return "unknown";
	}
}

// User I/O and servo support
bool Platform::GetFirmwarePin(int logicalPin, PinAccess access, Pin& firmwarePin, bool& invert)
{
	firmwarePin = NoPin;										// assume failure
	invert = false;												// this is the common case
	if (logicalPin < 0 || logicalPin > HighestLogicalPin)
	{
		// Pin number out of range, so nothing to do here
	}
	else if (logicalPin >= Heater0LogicalPin && logicalPin < Heater0LogicalPin + HEATERS)		// pins 0-9 correspond to heater channels
	{
		// For safety, we don't allow a heater channel to be used for servos until the heater has been disabled
		if (!reprap.GetHeat()->IsHeaterEnabled(logicalPin - Heater0LogicalPin))
		{
			firmwarePin = heatOnPins[logicalPin - Heater0LogicalPin];
		}
	}
	else if (logicalPin >= Fan0LogicalPin && logicalPin < Fan0LogicalPin + (int)NUM_FANS)		// pins 20- correspond to fan channels
	{
	}
	else if (logicalPin >= EndstopXLogicalPin && logicalPin < EndstopXLogicalPin + (int)ARRAY_SIZE(endStopPins))	// pins 40-49 correspond to endstop pins
	{
		if (access == PinAccess::read
#ifdef DUET_NG
			// Endstop pins on the DueX2/DueX5 can be used as digital outputs too
			|| (access == PinAccess::write && logicalPin >= EndstopXLogicalPin + 5)
#endif
		   )
		{
			firmwarePin = endStopPins[logicalPin - EndstopXLogicalPin];
		}
	}
#ifdef DUET_NG
	else if (logicalPin >= DueX5Gpio0LogicalPin && logicalPin < DueX5Gpio0LogicalPin + (int)ARRAY_SIZE(DueX5GpioPinMap))	// Pins 100-103 are the GPIO pins on the DueX2/X5
	{
		if (access != PinAccess::servo)
		{
			firmwarePin = DueX5GpioPinMap[logicalPin - DueX5Gpio0LogicalPin];
		}
	}
#endif

	if (firmwarePin != NoPin)
	{
		// Check that the pin mode has been defined suitably
		PinMode desiredMode;
		if (access == PinAccess::write)
		{
			desiredMode = (invert) ? OUTPUT_HIGH : OUTPUT_LOW;
		}
		else if (access == PinAccess::pwm || access == PinAccess::servo)
		{
		}
		else
		{
			desiredMode = INPUT_PULLUP;
		}
		if (logicalPinModes[logicalPin] != (int8_t)desiredMode)
		{
			SetPinMode(firmwarePin, desiredMode);
			logicalPinModes[logicalPin] = (int8_t)desiredMode;
		}
		return true;
	}
	return false;
}

bool Platform::SetExtrusionAncilliaryPwmPin(int logicalPin)
{
	return GetFirmwarePin(logicalPin, PinAccess::pwm, extrusionAncilliaryPwmFirmwarePin, extrusionAncilliaryPwmInvert);
}

#if SUPPORT_INKJET

// Fire the inkjet (if any) in the given pattern
// If there is no inkjet, false is returned; if there is one this returns true
// So you can test for inkjet presence with if(platform->Inkjet(0))
bool Platform::Inkjet(int bitPattern)
{
	if (inkjetBits < 0)
		return false;
	if (!bitPattern)
		return true;

	for(int8_t i = 0; i < inkjetBits; i++)
	{
		if (bitPattern & 1)
		{
			digitalWrite(inkjetSerialOut, 1);			// Write data to shift register

			for(int8_t j = 0; j <= i; j++)
			{
				digitalWrite(inkjetShiftClock, HIGH);
				digitalWrite(inkjetShiftClock, LOW);
				digitalWrite(inkjetSerialOut, 0);
			}

			digitalWrite(inkjetStorageClock, HIGH);		// Transfers data from shift register to output register
			digitalWrite(inkjetStorageClock, LOW);

			digitalWrite(inkjetOutputEnable, LOW);		// Fire the droplet out
			delayMicroseconds(inkjetFireMicroseconds);
			digitalWrite(inkjetOutputEnable, HIGH);

			digitalWrite(inkjetClear, LOW);				// Clear to 0
			digitalWrite(inkjetClear, HIGH);

			delayMicroseconds(inkjetDelayMicroseconds); // Wait for the next bit
		}

		bitPattern >>= 1; // Put the next bit in the units column
	}

	return true;
}
#endif

bool Platform::GCodeAvailable(const SerialSource source) const
{
	return false;
}

char Platform::ReadFromSource(const SerialSource source)
{
	return 0;
}

#ifndef __RADDS__
// CPU temperature
void Platform::GetMcuTemperatures(float& minT, float& currT, float& maxT) const
{
	minT = AdcReadingToCpuTemperature(lowestMcuTemperature);
	currT = AdcReadingToCpuTemperature(currentMcuTemperature);
	maxT = AdcReadingToCpuTemperature(highestMcuTemperature);
}
#endif

#ifdef DUET_NG
// Power in voltage
void Platform::GetPowerVoltages(float& minV, float& currV, float& maxV) const
{
	minV = AdcReadingToPowerVoltage(lowestVin);
	currV = AdcReadingToPowerVoltage(currentVin);
	maxV = AdcReadingToPowerVoltage(highestVin);
}
#endif

// Real-time clock

bool Platform::IsDateTimeSet() const
{
	return realTime != 0;
}

time_t Platform::GetDateTime() const
{
	return realTime;
}

bool Platform::SetDateTime(time_t time)
{
	return true;
}

bool Platform::SetDate(time_t date)
{
	return true;
}

bool Platform::SetTime(time_t time)
{
	return true;
}

// Step pulse timer interrupt
void STEP_TC_HANDLER()
{
}

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// Must be called with interrupts disabled,
/*static*/ bool Platform::ScheduleInterrupt(uint32_t tim)
{
	return false;
}

// Make sure we get no step interrupts
/*static*/ void Platform::DisableStepInterrupt()
{
}

// Process a 1ms tick interrupt
// This function must be kept fast so as not to disturb the stepper timing, so don't do any floating point maths in here.
// This is what we need to do:
// 1.  Kick off a new ADC conversion.
// 2.  Fetch and process the result of the last ADC conversion.
// 3a. If the last ADC conversion was for the Z probe, toggle the modulation output if using a modulated IR sensor.
// 3b. If the last ADC reading was a thermistor reading, check for an over-temperature situation and turn off the heater if necessary.
//     We do this here because the usual polling loop sometimes gets stuck trying to send data to the USB port.

//#define TIME_TICK_ISR	1		// define this to store the tick ISR time in errorCodeBits

void Platform::Tick()
{
#ifdef TIME_TICK_ISR
	uint32_t now = micros();
#endif

	if (tickState != 0)
	{
#ifdef DUET_NG
		// Read the power input voltage
		currentVin = AnalogInReadChannel(vInMonitorAdcChannel);
		if (currentVin > highestVin)
		{
			highestVin = currentVin;
		}
		if (currentVin < lowestVin)
		{
			lowestVin = currentVin;
		}
		if (driversPowered && currentVin > driverOverVoltageAdcReading)
		{
			TMC2660::SetDriversPowered(false);
			// We deliberately do not clear driversPowered here or increase the over voltage event count - we let the spin loop handle that
		}
#endif
	}

	switch (tickState)
	{
	case 1:			// last conversion started was a thermistor
	case 3:
		if (IsThermistorChannel(currentHeater))
		{
			// Because we are in the tick ISR and no other ISR reads the averaging filter, we can cast away 'volatile' here
			ThermistorAveragingFilter& currentFilter = const_cast<ThermistorAveragingFilter&>(thermistorFilters[currentHeater]);
			currentFilter.ProcessReading(AnalogInReadChannel(thermistorAdcChannels[heaterTempChannels[currentHeater]]));
		}

		// Guard against overly long delays between successive calls of PID::Spin().
		// Do not call Time() here, it isn't safe. We use millis() instead.
		++currentHeater;
		if (currentHeater == HEATERS)
		{
			currentHeater = 0;
		}
		++tickState;
		break;

	case 2:			// last conversion started was the Z probe, with IR LED on
		const_cast<ZProbeAveragingFilter&>(zProbeOnFilter).ProcessReading(GetRawZProbeReading());
		if (zProbeType == 2)									// if using a modulated IR sensor
		{
		}

		// Read the MCU temperature as well (no need to do it in every state)
#ifndef __RADDS__
		currentMcuTemperature = AnalogInReadChannel(temperatureAdcChannel);
#endif

		++tickState;
		break;

	case 4:			// last conversion started was the Z probe, with IR LED off if modulation is enabled
		const_cast<ZProbeAveragingFilter&>(zProbeOffFilter).ProcessReading(GetRawZProbeReading());
		// no break
	case 0:			// this is the state after initialisation, no conversion has been started
	default:
		if (zProbeType == 2)									// if using a modulated IR sensor
		{
		}
		tickState = 1;
		break;
	}


#ifdef TIME_TICK_ISR
	uint32_t now2 = micros();
	if (now2 - now > errorCodeBits)
	{
		errorCodeBits = now2 - now;
	}
#endif
}

// Pragma pop_options is not supported on this platform
//#pragma GCC pop_options

// End
