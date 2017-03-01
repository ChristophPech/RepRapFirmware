// Sim.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "reprapfirmware.h"
#include "Reprap.h"
#include "strsafe.h"

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


int main()
{
	reprap.Init();

    return 0;
}

