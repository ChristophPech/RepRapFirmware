// Sim.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "reprapfirmware.h"
#include "Reprap.h"
#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Duet/Webserver.h"

extern "C" void debugPrintf(const char* fmt, ...);
void debugPrintf(const char* fmt, ...)
{
	char sBuf[1024];
	va_list xArgList;
	va_start(xArgList, fmt);
	::StringCchVPrintfA(sBuf, 1024, fmt, xArgList);
	va_end(xArgList);
	//::OutputDebugStringA(sBuf);
	printf("%s", sBuf);
};


const char* gcInit =
"M111 S1\n"                       // Debug on
"M550 Testname\n"
"M208 X300 Y300 Z300\n"  // set axis maxima(adjust to suit your machine)
"M208 X0 Y0 Z0 S1\n" // set axis minimum(adjust to make X = 0 and Y = 0 the edge of the bed)
"M92 X80 Y80 Z2560\n"// Set axis steps / mm
"M92 E420:420\n"// Set extruder steps per mm
"G21\n"// Work in millimetres
"G90\n"// Send absolute coordinates...
"M83\n"// ...but relative extruder moves
"M563 P0 D0 H1\n"// Define tool 0 to use extruder drive 0 and heater 1
"T0\n"// select first hot end
"M37 S1\n"//simulation
;

const char* gcTest =

"G1 X65.300 Y67.544 E0.0789\n"//
"G1 X66.573 Y66.203 E0.0797\n"//
"G1 X67.890 Y64.889 E0.0802\n"//
//; inner perimeter
"G1 X67.491 Y64.474 F6000\n"//
"G1 X70.170 Y61.999 E0.1572 F1395\n"//
"G1 X73.037 Y59.606 E0.1610\n"//
"G1 X73.307 Y59.386 E0.0150\n"//
;

void SpinAll() {
	for (int j = 0; j < 256; j++) {
		reprap.Spin();
	}
	SleepEx(1, true);
}

const char* gCurCG = NULL;
int iCurCG = 0;
bool Webserver::GCodeAvailable(const WebSource source) const
{
	switch (source)
	{
	case WebSource::Telnet:
		if (gCurCG&&iCurCG < strlen(gCurCG)) return true;
	}
	return false;
}

char Webserver::ReadGCode(const WebSource source)
{
	switch (source)
	{
	case WebSource::Telnet:
		return gCurCG[iCurCG++];
	}

	return 0;
}


void RunGCode(const char* gcode)
{
	gCurCG = gcode;
	iCurCG = 0;

	SpinAll();
}


int main()
{
	reprap.Init();
	reprap.Spin();

	GCodes& gc=*reprap.GetGCodes();
	GCodeBuffer& gb = *gc.httpGCode;

	gc.axesHomed = -1;
	RunGCode(gcInit);
	RunGCode(gcTest);

	SpinAll();

    return 0;
}

