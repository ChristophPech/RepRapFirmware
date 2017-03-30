#include "RepRap.h"

#include "Network.h"
#include "Movement/Move.h"
#include "GCodes/GCodes.h"
//#include "Heating/Heat.h"
#include "Platform.h"
//#include "PrintMonitor.h"
#include "Tool.h"
#include "Webserver.h"
#include "Version.h"

#ifndef __RADDS__
//# include "sam/drivers/hsmci/hsmci.h"
#endif

// Callback function from the hsmci driver, called while it is waiting for an SD card operation to complete
extern "C" void hsmciIdle()
{
	if (reprap.GetSpinningModule() != moduleNetwork)	// I don't think this should ever be false because the Network module doesn't do file access, but just in case...
	{
		//reprap.GetNetwork()->Spin(false);
	}
}

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap() : toolList(nullptr), currentTool(nullptr), lastWarningMillis(0), activeExtruders(0),
	activeToolHeaters(0), ticksInSpinState(0), spinningModule(noModule), debug(0), stopped(false),
	active(false), resetting(false), processingConfig(true), beepFrequency(0), beepDuration(0)
{
	OutputBuffer::Init();
	platform = new Platform();
	//network = new Network(platform);
//	webserver = new Webserver(platform, network);
	gCodes = new GCodes(platform, webserver);
	move = new Move(platform, gCodes);
	//heat = new Heat(platform);

#if SUPPORT_ROLAND
	roland = new Roland(platform);
#endif

	//printMonitor = new PrintMonitor(platform, gCodes);

	SetPassword(DEFAULT_PASSWORD);
	SetName(DEFAULT_NAME);
	message[0] = 0;
}

void RepRap::Init()
{
	// All of the following init functions must execute reasonably quickly before the watchdog times us out
	platform->Init();
	gCodes->Init();
	//network->Init();
	//webserver->Init();
	move->Init();
	//heat->Init();
#if SUPPORT_ROLAND
	roland->Init();
#endif
	//printMonitor->Init();
	Platform::EnableWatchdog();		// do this after all init calls are made
	active = true;					// must do this before we start the network, else the watchdog may time out

	platform->MessageF(HOST_MESSAGE, "%s Version %s dated %s\n", FIRMWARE_NAME, VERSION, DATE);

	// Run the configuration file
	const char *configFile = platform->GetConfigFile();
	platform->Message(HOST_MESSAGE, "\nExecuting ");

	if (gCodes->RunConfigFile(configFile))
	{
		while (gCodes->IsDaemonBusy())
		{
			// GCodes::Spin will read the macro and ensure DoingFileMacro returns false when it's done
			Spin();
		}
		platform->Message(HOST_MESSAGE, "Done!\n");
	}
	else
	{
		platform->Message(HOST_MESSAGE, "Error, not found\n");
	}
	processingConfig = false;

	// Enable network (unless it's disabled)
#ifdef DUET_NG
	network->Activate();			// Need to do this here, as the configuration GCodes may set IP address etc.
	if (!network->IsEnabled())
	{
		platform->Message(HOST_MESSAGE, "Network disabled.\n");
	}
#else
#endif

#ifndef __RADDS__
	//hsmci_set_idle_func(hsmciIdle);
#endif
	platform->MessageF(HOST_MESSAGE, "%s is up and running.\n", FIRMWARE_NAME);
	fastLoop = FLT_MAX;
	slowLoop = 0.0;
	lastTime = platform->Time();
}

void RepRap::Exit()
{
	active = false;
	//heat->Exit();
	move->Exit();
	gCodes->Exit();
	//webserver->Exit();
	//network->Exit();
	platform->Message(GENERIC_MESSAGE, "RepRap class exited.\n");
	platform->Exit();
}

void RepRap::Spin()
{
	if(!active)
		return;

	spinningModule = modulePlatform;
	ticksInSpinState = 0;
	platform->Spin();

	spinningModule = moduleNetwork;
	ticksInSpinState = 0;
	//network->Spin(true);

	spinningModule = moduleWebserver;
	ticksInSpinState = 0;
	//webserver->Spin();

	spinningModule = moduleGcodes;
	ticksInSpinState = 0;
	gCodes->Spin();

	spinningModule = moduleMove;
	ticksInSpinState = 0;
	move->Spin();

	spinningModule = moduleHeat;
	ticksInSpinState = 0;
	//heat->Spin();

#if SUPPORT_ROLAND
	spinningModule = moduleRoland;
	ticksInSpinState = 0;
	roland->Spin();
#endif

	spinningModule = modulePrintMonitor;
	ticksInSpinState = 0;
	//printMonitor->Spin();

	spinningModule = noModule;
	ticksInSpinState = 0;

	// Check if we need to display a cold extrusion warning

	// Keep track of the loop time
	const float t = platform->Time();
	const float dt = t - lastTime;
	if (dt < fastLoop)
	{
		fastLoop = dt;
	}
	if (dt > slowLoop)
	{
		slowLoop = dt;
	}
	lastTime = t;
}

void RepRap::Timing(MessageType mtype)
{
	platform->MessageF(mtype, "Slowest main loop (seconds): %f; fastest: %f\n", slowLoop, fastLoop);
	fastLoop = FLT_MAX;
	slowLoop = 0.0;
}

void RepRap::Diagnostics(MessageType mtype)
{
	platform->Message(mtype, "=== Diagnostics ===\n");
	OutputBuffer::Diagnostics(mtype);
	platform->Diagnostics(mtype);				// this includes a call to our Timing() function
	move->Diagnostics(mtype);
	//heat->Diagnostics(mtype);
	gCodes->Diagnostics(mtype);
	//network->Diagnostics(mtype);
	//webserver->Diagnostics(mtype);
}

// Turn off the heaters, disable the motors, and deactivate the Heat and Move classes. Leave everything else working.
void RepRap::EmergencyStop()
{
	stopped = true;
}

void RepRap::SetDebug(Module m, bool enable)
{
	if (enable)
	{
		debug |= (1 << m);
	}
	else
	{
		debug &= ~(1 << m);
	}
	PrintDebug();
}

void RepRap::SetDebug(bool enable)
{
	debug = (enable) ? 0xFFFF : 0;
}

void RepRap::PrintDebug()
{
	if (debug != 0)
	{
		platform->Message(GENERIC_MESSAGE, "Debugging enabled for modules:");
		for (size_t i = 0; i < numModules; i++)
		{
			if ((debug & (1 << i)) != 0)
			{
				platform->MessageF(GENERIC_MESSAGE, " %s(%u)", moduleName[i], i);
			}
		}
		platform->Message(GENERIC_MESSAGE, "\nDebugging disabled for modules:");
		for (size_t i = 0; i < numModules; i++)
		{
			if ((debug & (1 << i)) == 0)
			{
				platform->MessageF(GENERIC_MESSAGE, " %s(%u)", moduleName[i], i);
			}
		}
		platform->Message(GENERIC_MESSAGE, "\n");
	}
	else
	{
		platform->Message(GENERIC_MESSAGE, "Debugging disabled\n");
	}
}

// Add a tool.
// Prior to calling this, delete any existing tool with the same number
// The tool list is maintained in tool number order.
void RepRap::AddTool(Tool* tool)
{
	Tool** t = &toolList;
	while(*t != nullptr && (*t)->Number() < tool->Number())
	{
		t = &((*t)->next);
	}
	tool->next = *t;
	*t = tool;
//	tool->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters);
	platform->UpdateConfiguredHeaters();
}

void RepRap::DeleteTool(Tool* tool)
{
	// Must have a valid tool...
	if (tool == nullptr)
	{
		return;
	}

	// Deselect it if necessary
	if (GetCurrentTool() == tool)
	{
		SelectTool(-1);
	}

	// Switch off any associated heater
	for (size_t i = 0; i < tool->HeaterCount(); i++)
	{
//		reprap.GetHeat()->SwitchOff(tool->Heater(i));
	}

	// Purge any references to this tool
	for (Tool **t = &toolList; *t != nullptr; t = &((*t)->next))
	{
		if (*t == tool)
		{
			*t = tool->next;
			break;
		}
	}

	// Delete it
	//	Tool::Delete(tool);

	// Update the number of active heaters and extruder drives
	activeExtruders = activeToolHeaters = 0;
	for (Tool *t = toolList; t != nullptr; t = t->Next())
	{
		//t->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters);
	}
	platform->UpdateConfiguredHeaters();
}

void RepRap::SelectTool(int toolNumber)
{
	Tool* tool = toolList;

	while(tool != nullptr)
	{
		if (tool->Number() == toolNumber)
		{
			tool->Activate(currentTool);
			currentTool = tool;
			return;
		}
		tool = tool->Next();
	}

	// Selecting a non-existent tool is valid.  It sets them all to standby.
	if (currentTool != nullptr)
	{
		StandbyTool(currentTool->Number());
	}
	currentTool = nullptr;
}

void RepRap::PrintTool(int toolNumber, StringRef& reply) const
{
	Tool* tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		tool->Print(reply);
	}
	else
	{
		reply.copy("Error: Attempt to print details of non-existent tool.\n");
	}
}

void RepRap::StandbyTool(int toolNumber)
{
	Tool* tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		tool->Standby();
		if (currentTool == tool)
		{
			currentTool = nullptr;
		}
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Attempt to standby a non-existent tool: %d.\n", toolNumber);
	}
}

Tool* RepRap::GetTool(int toolNumber) const
{
	Tool* tool = toolList;
	while(tool != nullptr)
	{
		if (tool->Number() == toolNumber)
		{
			return tool;
		}
		tool = tool->Next();
	}
	return nullptr; // Not an error
}

// Get the current tool, or failing that the default tool. May return nullptr if we can't
// Called when a M104 or M109 command doesn't specify a tool number.
Tool* RepRap::GetCurrentOrDefaultTool() const
{
	// If a tool is already selected, use that one, else use the lowest-numbered tool which is the one at the start of the tool list
	return (currentTool != nullptr) ? currentTool : toolList;
}

void RepRap::SetToolVariables(int toolNumber, const float* standbyTemperatures, const float* activeTemperatures)
{
	Tool* tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		tool->SetVariables(standbyTemperatures, activeTemperatures);
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Attempt to set variables for a non-existent tool: %d.\n", toolNumber);
	}
}

bool RepRap::IsHeaterAssignedToTool(int8_t heater) const
{
	for(Tool *tool = toolList; tool != nullptr; tool = tool->Next())
	{
		for(size_t i = 0; i < tool->HeaterCount(); i++)
		{
			if (tool->Heater(i) == heater)
			{
				// It's already in use by some tool
				return true;
			}
		}
	}

	return false;
}

unsigned int RepRap::GetNumberOfContiguousTools() const
{
	unsigned int numTools = 0;
	while (GetTool(numTools) != nullptr)
	{
		++numTools;
	}
	return numTools;
}

void RepRap::Tick()
{
	if (active && !resetting)
	{
		platform->Tick();
		++ticksInSpinState;
		if (ticksInSpinState >= 20000)	// if we stall for 20 seconds, save diagnostic data and reset
		{
			resetting = true;
			for(size_t i = 0; i < HEATERS; i++)
			{
				platform->SetHeater(i, 0.0);
			}
			for(size_t i = 0; i < DRIVES; i++)
			{
				platform->DisableDrive(i);
				// We can't set motor currents to 0 here because that requires interrupts to be working, and we are in an ISR
			}

			platform->SoftwareReset((uint16_t)SoftwareResetReason::stuckInSpin);
		}
	}
}

// Get the JSON status response for the web server (or later for the M105 command).
// Type 1 is the ordinary JSON status response.
// Type 2 is the same except that static parameters are also included.
// Type 3 is the same but instead of static parameters we report print estimation values.
OutputBuffer *RepRap::GetStatusResponse(uint8_t type, ResponseSource source)
{
	return nullptr;
}

OutputBuffer *RepRap::GetConfigResponse()
{
	// We need some resources to return a valid config response...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();

	// Axis minima
	response->copy("{\"axisMins\":");
	char ch = '[';
	for (size_t axis = 0; axis < numAxes; axis++)
	{
		response->catf("%c%.2f", ch, platform->AxisMinimum(axis));
		ch = ',';
	}

	// Axis maxima
	response->cat("],\"axisMaxes\":");
	ch = '[';
	for (size_t axis = 0; axis < numAxes; axis++)
	{
		response->catf("%c%.2f", ch, platform->AxisMaximum(axis));
		ch = ',';
	}

	// Accelerations
	response->cat("],\"accelerations\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->Acceleration(drive));
		ch = ',';
	}

	// Motor currents
	response->cat("],\"currents\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->GetMotorCurrent(drive, false));
		ch = ',';
	}

	// Firmware details
	response->catf("],\"firmwareElectronics\":\"%s", platform->GetElectronicsString());
#ifdef DUET_NG
	const char* expansionName = DuetExpansion::GetExpansionBoardName();
	if (expansionName != nullptr)
	{
		response->catf(" + %s", expansionName);
	}
#endif
	response->catf("\",\"firmwareName\":\"%s\"", FIRMWARE_NAME);
	response->catf(",\"firmwareVersion\":\"%s\"", VERSION);
#if defined(DUET_NG) && defined(DUET_WIFI)
	response->catf(",\"dwsVersion\":\"%s\"", network->GetWiFiServerVersion());
#endif
	response->catf(",\"firmwareDate\":\"%s\"", DATE);

	// Motor idle parameters
	response->catf(",\"idleCurrentFactor\":%.1f", platform->GetIdleCurrentFactor() * 100.0);
	response->catf(",\"idleTimeout\":%.1f", move->IdleTimeout());

	// Minimum feedrates
	response->cat(",\"minFeedrates\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->ConfiguredInstantDv(drive));
		ch = ',';
	}

	// Maximum feedrates
	response->cat("],\"maxFeedrates\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->MaxFeedrate(drive));
		ch = ',';
	}

	// Config file is no longer included, because we can use rr_configfile or M503 instead
	response->cat("]}");

	return response;
}

// Get the JSON status response for PanelDue or the old web server.
// Type 0 was the old-style webserver status response, but is no longer supported.
// Type 1 is the new-style webserver status response.
// Type 2 is the M105 S2 response, which is like the new-style status response but some fields are omitted.
// Type 3 is the M105 S3 response, which is like the M105 S2 response except that static values are also included.
// 'seq' is the response sequence number, if it is not -1 and we have a different sequence number then we send the gcode response
OutputBuffer *RepRap::GetLegacyStatusResponse(uint8_t type, int seq)
{
	return nullptr;
}

// Copy some parameter text, stopping at the first control character or when the destination buffer is full, and removing trailing spaces
void RepRap::CopyParameterText(const char* src, char *dst, size_t length)
{
	size_t i;
	for (i = 0; i + 1 < length && src[i] >= ' '; ++i)
	{
		dst[i] = src[i];
	}
	// Remove any trailing spaces
	while (i > 0 && dst[i - 1] == ' ')
	{
		--i;
	}
	dst[i] = 0;
}

// Get the list of files in the specified directory in JSON format.
// If flagDirs is true then we prefix each directory with a * character.
OutputBuffer *RepRap::GetFilesResponse(const char *dir, bool flagsDirs)
{
	return nullptr;
}

// Get a JSON-style filelist including file types and sizes
OutputBuffer *RepRap::GetFilelistResponse(const char *dir)
{
	return nullptr;

}

// Send a beep. We send it to both PanelDue and the web interface.
void RepRap::Beep(int freq, int ms)
{
	beepFrequency = freq;
	beepDuration = ms;

	if (platform->HaveAux())
	{
		// If there is an LCD device present, make it beep
		platform->Beep(freq, ms);
	}
}

// Send a short message. We send it to both PanelDue and the web interface.
void RepRap::SetMessage(const char *msg)
{
	strncpy(message, msg, MESSAGE_LENGTH);
	message[MESSAGE_LENGTH] = 0;

	if (platform->HaveAux())
	{
		platform->SendAuxMessage(msg);
	}
}

// Get the status character for the new-style status response
char RepRap::GetStatusCharacter() const
{
	return    (processingConfig)										? 'C'	// Reading the configuration file
			: (gCodes->IsFlashing())									? 'F'	// Flashing a new firmware binary
			: (IsStopped()) 											? 'H'	// Halted
			: (gCodes->IsPausing()) 									? 'D'	// Pausing / Decelerating
			: (gCodes->IsResuming()) 									? 'R'	// Resuming
			: (gCodes->IsDoingToolChange())								? 'T'	// Running tool change macros
			: (gCodes->IsPaused()) 										? 'S'	// Paused / Stopped
//			: (printMonitor->IsPrinting())								? 'P'	// Printing
			: (gCodes->DoingFileMacro() || !move->NoLiveMovement()) 	? 'B'	// Busy
			: 'I';																// Idle
}

bool RepRap::NoPasswordSet() const
{
	return (!password[0] || StringEquals(password, DEFAULT_PASSWORD));
}

bool RepRap::CheckPassword(const char *pw) const
{
	return StringEquals(pw, password);
}

void RepRap::SetPassword(const char* pw)
{
	// Users sometimes put a tab character between the password and the comment, so allow for this
	CopyParameterText(pw, password, ARRAY_SIZE(password));
}

const char *RepRap::GetName() const
{
	return myName;
}

void RepRap::SetName(const char* nm)
{

}

// Given that we want to extrude/retract the specified extruder drives, check if they are allowed.
// For each disallowed one, log an error to report later and return a bit in the bitmap.
// This may be called by an ISR!
unsigned int RepRap::GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions)
{
	
	return -1;
}

void RepRap::FlagTemperatureFault(int8_t dudHeater)
{

}

void RepRap::ClearTemperatureFault(int8_t wasDudHeater)
{

}

// Get the current axes used as X axes
uint32_t RepRap::GetCurrentXAxes() const
{
	return 0;
}

// Helper function for diagnostic tests in Platform.cpp, to cause a deliberate divide-by-zero
/*static*/ uint32_t RepRap::DoDivide(uint32_t a, uint32_t b)
{
	return a/b;
}

// Helper function for diagnostic tests in Platform.cpp, to cause a deliberate unaligned memory read
/*static*/ uint32_t RepRap::ReadDword(const char* p)
{
	return *reinterpret_cast<const uint32_t*>(p);
}

// End
