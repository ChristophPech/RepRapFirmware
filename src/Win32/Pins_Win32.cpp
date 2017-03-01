#include "stdafx.h"
#include "Pins_Win32.h"

#include "Reprap.h"

char *strptime(const char * a, const char * b, struct tm * c)
{
	return (char*)a;
}

#include "Platform.h"

void Platform::Message(const MessageType type, OutputBuffer *buffer)
{
	debugPrintf("Message(%i): %s", type, buffer->Data());
	OutputBuffer::ReleaseAll(buffer);
}
void Platform::Message(const MessageType type, const char *buffer)
{
	debugPrintf("Message(%i): %s", type, buffer);
}

void Platform::MessageVA(MessageType type, const char *fmt, va_list vargs)
{
	char formatBuffer[1024*4];
	StringRef formatString(formatBuffer, ARRAY_SIZE(formatBuffer));
	formatString.vprintf(fmt, vargs);

	Message(type, formatBuffer);
}

void Platform::MessageF(MessageType type, const char *fmt, ...)
{
	char formatBuffer[1024 * 4];
	StringRef formatString(formatBuffer, ARRAY_SIZE(formatBuffer));

	va_list vargs;
	va_start(vargs, fmt);
	formatString.vprintf(fmt, vargs);
	va_end(vargs);

	Message(type, formatBuffer);
}





