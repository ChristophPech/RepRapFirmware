#include "stdafx.h"
#include "Pins_Win32.h"

#include "Reprap.h"
#include "PrintMonitor.h"
#include "Platform.h"
#include "Network.h"
#include "Fan.h"

char *strptime(const char * a, const char * b, struct tm * c)
{
	return (char*)a;
}



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





bool MassStorage::Delete(const char* directory, const char* fileName) { return false; };
bool MassStorage::MakeDirectory(const char *parentDir, const char *dirName) { return false; };
bool MassStorage::DirectoryExists(const char *path) const { return false; };
bool MassStorage::MakeDirectory(const char *directory) { return false; };
const char* MassStorage::CombineName(const char* directory, const char* fileName) { return ""; };
bool MassStorage::FindFirst(const char *directory, FileInfo &file_info) { return false; };
bool MassStorage::FindNext(FileInfo &file_info) { return false; };
bool MassStorage::Mount(size_t card, StringRef& reply, bool reportSuccess) { return false; };
bool MassStorage::Unmount(size_t card, StringRef& reply) { return false; };


FRESULT f_sync(FIL*) { return FR_OK; };
FRESULT f_open(FIL*, const TCHAR*, BYTE) { return FR_OK; };
FRESULT f_read(FIL*, void*, UINT, UINT*) { return FR_OK; };
FRESULT f_lseek(FIL*, DWORD) { return FR_OK; };
FRESULT f_close(FIL*) { return FR_OK; };
FRESULT f_write(FIL*, const void*, UINT, UINT*) { return FR_OK; };

void PrintMonitor::StartingPrint(const char *filename) {};
void PrintMonitor::StartedPrint() {};
void PrintMonitor::StoppedPrint() {};
bool PrintMonitor::GetFileInfoResponse(const char *filename, OutputBuffer *&response) { return false; };

void Network::EnableProtocol(int protocol, int port, int secure, StringRef& reply) {};
void Network::DisableProtocol(int protocol, StringRef& reply) {};
void Network::ReportProtocols(StringRef& reply) const {};
void Network::Enable() {};
void Network::Disable() {};
const uint8_t *Network::GetIPAddress() const { static uint i = 0; return (uint8_t*)&i; };


