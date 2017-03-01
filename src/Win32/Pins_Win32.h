#pragma once
#include <cmath>
#include <algorithm>
#include <iostream>
#include <inttypes.h>
using namespace std;

#define private public //most evil define

#pragma warning (disable:4305)
#pragma warning (disable:4244)
#pragma warning (disable:4838)
#pragma warning (disable:4838)

const double PI = 3.14159265359;

typedef int Pin;
typedef int PinMode;
const int NoPin = -1;

static int millis() { return 1; };
static int64_t micros() { return 1; };

static void pinMode(int, int) {};
static void setPullup(int, int) {};
static bool digitalRead(int) { return false; };
static void digitalWrite(int, int) {};
static void AnalogOut(int, float, int) {};
static int AnalogInReadChannel(int) { return 0; };
static void AnalogInEnableChannel(int, int) {};
static void watchdogEnable(int) {};
static void watchdogReset() {};
static void cpu_irq_disable() {};
static void cpu_irq_enable() {};
static int cpu_irq_save() { return 0; };
static void cpu_irq_restore(int) {};
static int PinToAdcChannel(int) { return 0; }
static int inInterrupt() { return  false; }

#define asm
#define volatile(x) 

typedef int MCP4461;
typedef int AnalogChannelNumber;
typedef int irqflags_t;
typedef uint32_t uint;
const int MaxSpiTempSensors = 1;
const int NUM_FANS = 1;
const int NumSdCards = 1;
const int HighestLogicalPin = 4;
const int VARIANT_MCK = 1024 * 1024;
#define FIRMWARE_NAME "Simulator"
#define DEFAULT_BOARD_TYPE BoardType::Duet_06
#define IAP_FIRMWARE_FILE	"DuetEthernetFirmware.bin"
#define IAP_UPDATE_FILE "abc"
const size_t NumFirmwareUpdateModules = 1;		// 1 module
#define ATX_POWER_PIN 0
#define OUTPUT_LOW 0
#define OUTPUT_HIGH 1
#define SENSE_RESISTOR 1
#define MAX_STEPPER_DIGIPOT_VOLTAGE 12
#define STEPPER_DAC_VOLTAGE_RANGE 12
#define STEPPER_DAC_VOLTAGE_OFFSET 12
#define Z_PROBE_PIN 1
#define INPUT_PULLUP 1

static int DIRECTION_PINS[30];
struct TCC {
	int TC_CV;
};
struct STC {
	TCC TC_CHANNEL[4];
};
static STC* STEP_TC;
const int STEP_TC_CHAN = 1;

struct PIO {
	int PIO_ODSR;
	int PIO_OWDR;
};
struct PinDescription {
	PIO* pPort;
	int ulPin;
};
static PIO pio;
#define PIOA (&pio)
#define PIOB (&pio)
#define PIOC (&pio)
#define PIOD (&pio)
static PinDescription g_APinDescription[30];
static int STEP_PINS[10];

template<class T>
static const T& constrain(const T& x, const T& a, const T& b) {if (x < a) {return a;}else if (b < x) {return b;} else return x;}
static float fsquare(float f) { return f*f; }
static int64_t isquare64(int64_t i) { return i*i; };
#define ARRAY_SIZE( array ) ( sizeof( array ) / sizeof( array[0] ))
char *strptime(const char * a, const char * b, struct tm * c);

#define pre(x)
#define post(x)
#define FLASH_DATA_LENGTH 1024

#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill

// The physical capabilities of the machine

const size_t DRIVES = 10;						// The number of drives in the machine, including X, Y, and Z plus extruder drives
#define DRIVES_(a,b,c,d,e,f,g,h,i,j) { a,b,c,d,e,f,g,h,i,j }

const int8_t HEATERS = 8;						// The number of heaters in the machine; 0 is the heated bed even if there isn't one
#define HEATERS_(a,b,c,d,e,f,g,h) { a,b,c,d,e,f,g,h }
const size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

const size_t MAX_AXES = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
const size_t MIN_AXES = 3;						// The minimum and default number of axes
const size_t MaxExtruders = DRIVES - MIN_AXES;	// The maximum number of extruders

const size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

const Pin ExpansionStart = 200;					// Pin numbers at/above this are on the I/O expander

												// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

												// DRIVES


static int END_STOP_PINS[DRIVES];
static int POT_WIPES[8];

