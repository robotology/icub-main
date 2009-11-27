//---------------------------------------------------------------------------
#ifndef SerialOutDecodeH
#define SerialOutDecodeH

#include <vector>
using namespace std;

//---------------------------------------------------------------------------
class SerialOutDecode
{
public:
    static int decode_buffer( unsigned char *inp_buf, unsigned char *out_buf, int cnt );
    static long parse_generic_buffer(unsigned char* buffer, long bufferSize,
        const vector<long>* itemTypes, const vector<float>* itemScales, vector<int>* output);

    static long GetTypeSize(long type);
    static int ConvertBytes2Value(const unsigned char* bytes, long type, float scale);
};
#endif
