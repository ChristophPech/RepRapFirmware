// Sim.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "reprapfirmware.h"
#include "Reprap.h"
#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Duet/Webserver.h"
#include "Movement/Grid.h"

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

"M201 X2000  Y2000    Z15 E2000\n"// Accelerations(mm / s ^ 2)
"M203 X15000 Y15000   Z360 E3600\n"// Maximum speeds(mm / min) Marlin had Z at 360 mm / s
"M566 X600   Y600     Z60 E600\n"// Maximum instant speed changes mm / minute(sometime miscalled 'jerk')

"M572  D0 S0.05\n" //pressure advance

"M92 E420:420\n"// Set extruder steps per mm
"G21\n"// Work in millimetres
"G90\n"// Send absolute coordinates...
"M83\n"// ...but relative extruder moves
"M563 P0 D0 H1\n"// Define tool 0 to use extruder drive 0 and heater 1
"T0\n"// select first hot end
"M37 S1\n"//simulation
;

const char* gcTestX =
"G1 X65.300 Y67.544 E0.0789\n"//
"G1 X66.573 Y66.203 E0.0797\n"//
"G1 X67.890 Y64.889 E0.0802\n"//
//; inner perimeter
"G1 X67.491 Y64.474 F6000\n"//
"G1 X70.170 Y61.999 E0.1572 F1395\n"//
"G1 X73.037 Y59.606 E0.1610\n"//
"G1 X73.307 Y59.386 E0.0150\n"//
;

const char* gcTest =
"G1 X60 Y50 F6000\n"//
"G1 X65 Y50 E1 F2000\n"//
"G1 X70 Y50 E1\n"//
//; inner perimeter
"G1 X75 Y50 F6000\n"//
"G1 X80 Y50 E1 F2000\n"//
"G1 X85 Y50 E1\n"//
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
	//RunGCode(gcTest);
	SpinAll();

	FILE* pF=fopen("d:/_out.csv","w");

	HeightMap xHM(new float[265]);
	xHM.ClearGridHeights();
	xHM.useMap = true;

	const char* sDef = "26.00,299.00,12.00,188.00,-1.00,32.00,9,6";
	xHM.def.ReadParameters(*(StringRef*)(&sDef));

	/*
	const char* sDef2 = "0,300,0,200,-1.00,100.00,4,3";
	xHM.def.ReadParameters(*(StringRef*)(&sDef2));
	float afH[256] = {
		0, 0, 0, 0,
		0, 0.001, 1, 0,
		0, 1, 2, 0,
	};/**/

	//*
	const char* sDef2 = "26.00,299.00,12.00,299.00,-1.00,32.00,9,9";
	xHM.def.ReadParameters(*(StringRef*)(&sDef2));
	float afH[256] = {
		//26      58     90     122    154    186    218     250    282
		0, 0, 0, 0, 0.095, 0, 0, 0, 0,
		0, 0, -0.003, 0.110, 0.007, 0.112, 0.117, 0, 0,
		0, -0.043, -0.003, 0.110, 0.007, 0.112, 0.117, 0, 0,
		0.085, -0.025, 0.120, 0.137, 0.097, 0.152, 0.115, 0, 0,
		0.095, 0.082, 0.082, 0.175, 0.205, 0.207, 0.320, 0, 0,
		0.085, 0.057, 0.125, 0.150, 0.205, 0.210, 0.107, 0, 0,
		0, 0.033, 0.148, 0.180, 0.115, 0.232, 0.162, 0, 0,
		0, 0, -0.008, 0.027, 0.095, -0.005, -0.005, 0, 0,
		0, 0, 0, 0, 0.095, 0, 0, 0, 0,
	};/**/

	/*
	float afH[256] = {
		//26      58     90     122    154    186    218     250    282
		0.080, -0.043, -0.003, 0.110, 0.007, 0.112, 0.117, 0, 0,
		0.085, -0.025, 0.120, 0.137, 0.097, 0.152, 0.115, 0, 0,
		0.095, 0.082, 0.082, 0.175, 0.205, 0.207, 0.320, 0, 0,
		0.085, 0.057, 0.125, 0.150, 0.205, 0.210, 0.107, 0, 0,
		0.092, 0.033, 0.148, 0.180, 0.115, 0.232, 0.162, 0, 0,
		- 0.053, 0.010, -0.008, 0.027, 0.095, -0.005, -0.005, 0, 0,
	};/**/

	/*
	float afH[256] = {
		//26      58     90     122    154    186    218     250    282
		0.080, -0.043, -0.003, 0.110, 0.007, 0.112, 0.117, -0.068, 0.015,
		0.085, -0.025, 0.120, 0.137, 0.097, 0.152, 0.115, 0.075, 0.127,
		0.095, 0.082, 0.082, 0.175, 0.205, 0.207, 0.320, 0.135, 0.062,
		0.085, 0.057, 0.125, 0.150, 0.205, 0.210, 0.107, 0.225, 0.212,
		0.092, 0.033, 0.148, 0.180, 0.115, 0.232, 0.162, 0.045, 0.170,
		-0.053, 0.010, -0.008, 0.027, 0.095, -0.005, -0.005, 0.060, -0.078,
	};/**/


	for (int iY = 0; iY < xHM.def.numY; iY++)
	{
		for (int iX = 0; iX < xHM.def.numX; iX++)
		{
			float val = afH[iX + iY*xHM.def.numX];
			if (val) {
				xHM.SetGridHeight(iX, iY, val);
			}
		}
	}
	xHM.ExtrapolateMissing();


	float fStepX = 2.0f;
	float fStepY = 2.0f;
	printf("\n\n");
	for (float fY = -1; fY < 200; fY += fStepX) {
		if (fY < 0) {
			fprintf(pF, ",");
		}
		else {
			fprintf(pF, "y:%g,", fY);
		}
		for (float fX = 0; fX < 300; fX += fStepY) {
			if (fY < 0) {
				fprintf(pF, "x:%g,", fX);
				continue;
			}

			if (fX >= 282) {
				int iDbg = 0;
			}
			float fE = xHM.GetInterpolatedHeightError(fX, fY);
			fprintf(pF, "%.3f,", fE);
		}
		fprintf(pF, "\n");
	}
	//xHM.gridHeights
	
	/*xmin, xmax, ymin, ymax, radius, spacing, xnum, ynum
		10.00, 165.00, 10.00, 165.00, -1.00, 25.00, 7, 7
		0, 0, 0, 0, 0, 0, 0
		0, 0, -0.502, -0.720, -0.785, -0.793, -0.440
		0, 0, -0.795, -0.850, -0.725, -1.155, -0.545
		0, 0, -0.880, -0.660, -0.955, -0.737, -0.488
		0, 0, -0.957, -0.803, -1.023, -0.930, -0.665
		0, 0, -0.940, -1.065, -1.082, -0.825, -0.400
		0, 0, -1.123, -1.103, -1.135, -1.062, -0.605*/

    return 0;
}

