// CircularBuffer.cpp: implementation of the CCircularBuffer class.
//
//////////////////////////////////////////////////////////////////////
#include "MobileEye.h"
#include "ASLCircularBuffer.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCircularBuffer::CCircularBuffer()
{
	Initialize();
}

CCircularBuffer::~CCircularBuffer()
{
}

//////////////////////////////////////////////////////////////////////
// Private helper functions
//////////////////////////////////////////////////////////////////////

void CCircularBuffer::WriteByte(unsigned char val)
{
	buffer[end] = val;

	end++;
	if (end >= CIRC_BUFFER_SIZE) end = 0;

	// check if we have overfilled the buffer
	if (end == begin)
	{
		begin++;
		if (begin >= CIRC_BUFFER_SIZE) begin=0;
	}
}

bool CCircularBuffer::ReadByte(unsigned char *val)
{
	if (begin == end) // buffer empty
		return false;

	*val = buffer[begin];
	
	begin++;
		if (begin >= CIRC_BUFFER_SIZE) begin=0;

	return true;
}

int CCircularBuffer::GetByteCount()
{
	// Note: max byte count is CIRC_BUFFER_SIZE -1.
	if (begin <= end)
		return (end - begin);
	else
		return (CIRC_BUFFER_SIZE - begin + end);

}

//////////////////////////////////////////////////////////////////////
// Operation
//////////////////////////////////////////////////////////////////////

void CCircularBuffer::Initialize()
{
	begin = end = 0;
}

//bool CCircularBuffer::ReadSerialPort(CSerialComm* pComm)
//{
//	unsigned char buf[MAX_MESSAGE_LENGTH];
//	unsigned long actualRead;
//	
//	bool res = pComm->ReadBuffer(buf, &actualRead);
//
//	if ((res == false) || (actualRead == 0) || (actualRead >= MAX_MESSAGE_LENGTH))
//		return false;
//
//	for (unsigned long i=0; i < actualRead; i++)
//		WriteByte(buf[i]);
//
//	if (GetByteCount() >= CIRC_BUFFER_SIZE-1)
//	{
//		static DWORD time = 0;
//		Trace("SerialOut DLL: Buffer overfill\n", &time);
//	}
//
//	return true;
//}


bool CCircularBuffer::Write(int size, unsigned char* src)
{
	unsigned char buf[MAX_MESSAGE_LENGTH];
	unsigned long actualRead;
	
	if ((size == 0) || (size >= MAX_MESSAGE_LENGTH))
		return false;

	for (unsigned long i=0; i < size; i++)
		WriteByte(src[i]);

	//if (GetByteCount() >= CIRC_BUFFER_SIZE-1)
	//{
	//	static DWORD time = 0;
	//	Trace("SerialOut DLL: Buffer overfill\n", &time);
	//}

	return true;
}

bool CCircularBuffer::ExtractMessage(int size, unsigned char* destination)
{
	if (GetByteCount() < size) // not enough bytes in the buffer
		return false;

	// find message start byte (should contain 1 in the MSB)
	unsigned char byte;
	bool found = false;
	while ((!found) && ReadByte(&byte))
	{
		if (byte & 0x80)
			found = true;
	}
	
	// check if we have enough bytes left in the buffer
	if (GetByteCount() < size - 1)
	{
		if (found) // return start byte back to try next time
		{
			begin--;
			if (begin < 0) begin = CIRC_BUFFER_SIZE -1;
		}
		return false;
	}
	
	// We found the first byte and there is enough data in the buffer
	*destination = byte;
	destination++;

	for (int i=0; i < size-1; i++)
	{
		ReadByte(destination);
		destination++;
	}
	return true;
}

