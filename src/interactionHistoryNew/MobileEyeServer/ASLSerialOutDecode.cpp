//---------------------------------------------------------------------------
// Helper class providing decoding of Serial Out port packets
//---------------------------------------------------------------------------
#include "MobileEye.h"
#include "ASLSerialOutDecode.h"
//---------------------------------------------------------------------------

// This program removes special formatting added to messages in order to
// send them as stream
// cnt - number of data bytes in the message (not counting overflow bytes)
int SerialOutDecode::decode_buffer( unsigned char *inp_buf, unsigned char *out_buf, int cnt)
{
  unsigned char byte;
  unsigned char overflow_byte;
  int rem;
  int i;
  int j;
  int m;
  int n;

  int Result = 0;
  m = ( ( cnt - 1 ) / 7 ) + 1;
  for (j=0; j<m; j=j+1) {
    if ( j == ( m - 1 ) )
	{
        rem = cnt % 7;
    	n = rem ? rem : 7;
	}
    else
            n = 7;
    overflow_byte = *( inp_buf + n );
    for ( i = 0; i < n; i = i + 1 )
            {
            unsigned char inp_byte = *inp_buf++;
            if (i==0) inp_byte &= 0x7F; // remove "message start" indicator
            byte = inp_byte | ( ( overflow_byte << ( 7 - i ) ) & 0x80 );
            *out_buf++ = byte;
            Result++;
            }
    inp_buf++;  // skip overflow byte
  }
  return Result;
}

long SerialOutDecode::parse_generic_buffer(
    unsigned char* buffer, // (input) buffer containing the message
    long bufferSize,       // (input) buffer length in bytes
    const vector<long>* itemTypes, // (input) array containg data types
                                   //  of items included in the message
    const vector<float>* itemScales, // scaling factors
    vector<int>* output) //struct containing output
{

    unsigned char* currentPos = buffer;
    for (unsigned int i=0; i < itemTypes->size(); i++)
    {
        if (currentPos > buffer + bufferSize)
            return -1;

        long type = itemTypes->at(i);
	float scale = 0;
	if (itemScales)
	  scale = itemScales->at(i);
	output->push_back(ConvertBytes2Value(currentPos, type, scale));
        currentPos += GetTypeSize(type);
    }
    return 0;
}

long SerialOutDecode::GetTypeSize(long type)
{
    switch (type)
    {
    case ASL_TYPE_BYTE:
        return (1);
    case ASL_TYPE_SHORT:
    case ASL_TYPE_UNSIGNED_SHORT:
        return (2);
    case ASL_TYPE_LONG:
    case ASL_TYPE_UNSIGNED_LONG:
    case ASL_TYPE_FLOAT:
        return (4);
    default:
        return (0);
    }
}

// Translate array of bytes into an integer
int SerialOutDecode::ConvertBytes2Value(
    const unsigned char* bytes, // (input) byte array representing the value
    long type,                  // (input) enumerated value type
    float scale)		// (input) scaling factor (0 means no scaling)
{
    union Translator
    {
        unsigned char   inpBytes[8];
        short           sVal;
        unsigned short  usVal;
        long            lVal;
        unsigned long   ulVal;
        float           fVal;
    };
    Translator t;
    int out;

    // Determine value size
    int size = GetTypeSize(type);
    //if (size <= 0)
    //   return -1;;

    // Copy bytes into union
    // Note: SerialOut port sends list significant byte first
    for (long i=0; i<size; i++)
        t.inpBytes[i] = bytes[size-i-1];

    // Perform the translation
    switch (type)
    {
    case ASL_TYPE_BYTE:
      out = (int) t.inpBytes[0];
        break;
    case ASL_TYPE_SHORT:
      out = (int) t.sVal;
        break;
    case ASL_TYPE_UNSIGNED_SHORT:
      out = (int) t.usVal;
        break;
    case ASL_TYPE_LONG:
      out = (int) t.lVal;
        break;
    case ASL_TYPE_UNSIGNED_LONG:
      out =  (int) t.ulVal;
        break;
    case ASL_TYPE_FLOAT:
      out =  (int) t.fVal;
        break;
    default:
      out = 0;
    }

    if (scale)
      {
	out = (int) (out*scale);
      }
    
    return out;
}

