//------------------------------------------------------------------------------------------------

#include "RepRapFirmware.h"
#include "FileStore.h"
#include "MassStorage.h"
#include "Platform.h"
#include "RepRap.h"

uint32_t FileStore::longestWriteTime = 0;

FileStore::FileStore(Platform* p) : platform(p)
{
}

void FileStore::Init()
{
	bufferPointer = 0;
	inUse = false;
	writing = false;
	lastBufferEntry = 0;
	openCount = 0;
	closeRequested = false;
}

// Invalidate the file if it uses the specified FATFS object
void FileStore::Invalidate(const FATFS *fs)
{
	if (file.fs == fs)
	{
		Init();
		file.fs = nullptr;
	}
}

// Return true if the file is open on the specified file system
bool FileStore::IsOpenOn(const FATFS *fs) const
{
	return openCount != 0 && file.fs == fs;
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.
bool FileStore::Open(const char* directory, const char* fileName, bool write)
{
	return true;
}

void FileStore::Duplicate()
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to dup a non-open file.\n");
		return;
	}
	irqflags_t flags = cpu_irq_save();
	++openCount;
	cpu_irq_restore(flags);
}

// This may be called from an ISR, in which case we need to defer the close
bool FileStore::Close()
{
	if (inInterrupt())
	{
		if (!inUse)
		{
			return false;
		}

		irqflags_t flags = cpu_irq_save();
		if (openCount > 1)
		{
			--openCount;
		}
		else
		{
			closeRequested = true;
		}
		cpu_irq_restore(flags);
		return true;
	}

	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to close a non-open file.\n");
		return false;
	}

	irqflags_t flags = cpu_irq_save();
	--openCount;
	bool leaveOpen = (openCount != 0);
	cpu_irq_restore(flags);

	if (leaveOpen)
	{
		return true;
	}

	bool ok = true;
	if (writing)
	{
		ok = Flush();
	}

	inUse = false;
	writing = false;
	lastBufferEntry = 0;
	closeRequested = false;
	return ok ;
}

bool FileStore::Seek(FilePosition pos)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to seek on a non-open file.\n");
		return false;
	}
	if (writing)
	{
		WriteBuffer();
	}
	return true;
}

FilePosition FileStore::Position() const
{
	FilePosition pos = file.fptr;
	if (writing)
	{
		pos += bufferPointer;
	}
	else if (bufferPointer < lastBufferEntry)
	{
		pos -= (lastBufferEntry - bufferPointer);
	}
	return pos;
}

#if 0	// not currently used
bool FileStore::GoToEnd()
{
	return Seek(Length());
}
#endif

FilePosition FileStore::Length() const
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to size non-open file.\n");
		return 0;
	}
	return file.fsize;
}

float FileStore::FractionRead() const
{
	FilePosition len = Length();
	if (len == 0)
	{
		return 0.0;
	}

	return (float)Position() / (float)len;
}

uint8_t FileStore::Status()
{
	if (!inUse)
		return (uint8_t)IOStatus::nothing;

	if (lastBufferEntry == FileBufLen)
		return (uint8_t)IOStatus::byteAvailable;

	if (bufferPointer < lastBufferEntry)
		return (uint8_t)IOStatus::byteAvailable;

	return (uint8_t)IOStatus::nothing;
}

bool FileStore::ReadBuffer()
{
	bufferPointer = 0;
	return true;
}

// Single character read via the buffer
bool FileStore::Read(char& b)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to read from a non-open file.\n");
		return false;
	}

	if (bufferPointer >= FileBufLen)
	{
		bool ok = ReadBuffer();
		if (!ok)
		{
			return false;
		}
	}

	if (bufferPointer >= lastBufferEntry)
	{
		b = 0;  // Good idea?
		return false;
	}

	b = (char) GetBuffer()[bufferPointer];
	bufferPointer++;

	return true;
}

// Block read, doesn't use the buffer
// Returns the number of bytes read or -1 if the read process failed
int FileStore::Read(char* extBuf, size_t nBytes)
{
	return 0;
}

// As Read but stop after '\n' or '\r\n' and null-terminate the string.
// If the next line is too long to fit in the buffer then the line will be split.
int FileStore::ReadLine(char* buf, size_t nBytes)
{
	const FilePosition lineStart = Position();
	const int r = Read(buf, nBytes);
	if (r < 0)
	{
		return r;
	}

	int i = 0;
	while (i < r && buf[i] != '\r' && buf[i] != '\n')
	{
		++i;
	}

	if (i + 1 < r && buf[i] == '\r' && buf[i + 1] == '\n')	// if stopped at CRLF (Windows-style line end)
	{
		Seek(lineStart + i + 2);							// seek to just after the CRLF
	}
	else if (i < r)											// if stopped at CR or LF
	{
		Seek(lineStart + i + 1);							// seek to just after the CR or LF
	}
	else if (i == (int)nBytes)
	{
		--i;												// make room for the null terminator
		Seek(lineStart + i);
	}
	buf[i] = 0;
	return i;
}

bool FileStore::WriteBuffer()
{
	if (bufferPointer != 0)
	{
		if (!InternalWriteBlock((const char*)GetBuffer(), bufferPointer))
		{
			return false;
		}
		bufferPointer = 0;
	}
	return true;
}

bool FileStore::Write(char b)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to write byte to a non-open file.\n");
		return false;
	}
	GetBuffer()[bufferPointer] = b;
	bufferPointer++;
	if (bufferPointer >= FileBufLen)
	{
		return WriteBuffer();
	}
	return true;
}

bool FileStore::Write(const char* b)
{
	return Write(b, strlen(b));
}

// Direct block write that bypasses the buffer. Used when uploading files.
bool FileStore::Write(const char *s, size_t len)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to write block to a non-open file.\n");
		return false;
	}

	if (!WriteBuffer())
	{
		return false;
	}
	return InternalWriteBlock(s, len);
}

bool FileStore::InternalWriteBlock(const char *s, size_t len)
{
	return true;
}

bool FileStore::Flush()
{
	return true;
}

float FileStore::GetAndClearLongestWriteTime()
{
	float ret = (float)longestWriteTime/1000.0;
	longestWriteTime = 0;
	return ret;
}

#if 0	// not currently used

// Provide a cluster map for fast seeking. Needs _USE_FASTSEEK defined as 1 in conf_fatfs to make any difference.
// The first element of the table must be set to the total number of 32-bit entries in the table before calling this.
bool FileStore::SetClusterMap(uint32_t tbl[])
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Error: Attempt to set cluster map for a non-open file.\n");
		return false;
	}

	file.cltbl = tbl;
	FRESULT ret = f_lseek(&file, CREATE_LINKMAP);
//	debugPrintf("ret %d need %u\n", (int)ret, tbl[0]);
	return ret == FR_OK;
}

#endif

// End
