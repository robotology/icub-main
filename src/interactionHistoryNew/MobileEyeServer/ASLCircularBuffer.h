// CircularBuffer.h: interface for the CCircularBuffer class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __CIRCULARBUFFER_H__
#define __CIRCULARBUFFER_H__


const int CIRC_BUFFER_SIZE = 150;

class CCircularBuffer  
{
public:
	CCircularBuffer();
	virtual ~CCircularBuffer();

	void Initialize();
	//bool ReadSerialPort(CSerialComm* pComm);
	bool Write(int size, unsigned char* src);
	bool ExtractMessage(int size, unsigned char* destination);

private:
	// members
	unsigned char buffer[CIRC_BUFFER_SIZE];
	int begin;
	int end;

	// helper functions
	bool ReadByte(unsigned char* val);
	void WriteByte(unsigned char val);
	int GetByteCount();
};

#endif
