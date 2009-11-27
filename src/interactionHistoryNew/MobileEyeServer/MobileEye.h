#ifndef __MobileEyeh__
#define __MobileEyeh__
#include <string>
#include <vector>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>

using namespace std;
using namespace yarp::os;

class SerialThread;

const int MAX_MESSAGE_LENGTH = 80;

enum ASLSerialOutPort_Type
  {	ASL_TYPE_BYTE	= 0,
	ASL_TYPE_SHORT	= 1,
	ASL_TYPE_UNSIGNED_SHORT	= 2,
	ASL_TYPE_LONG	= 3,
	ASL_TYPE_UNSIGNED_LONG	= 4,
	ASL_TYPE_FLOAT	= 5,
	ASL_TYPE_NO_VALUE	= 6
  };

class MobileEye{
  
 public:
  MobileEye();
  virtual ~MobileEye();
  bool start();
  bool receive(Bottle &msg);
  virtual bool stop();
  
  Semaphore _mutex;
  
  typedef unsigned char BYTE;
  
  typedef struct
  {
    int stat; //not used for mobile eye
    int plane; //not used for mobile eye (except with EyeHead Int option)
    int pd; //pupil diameter
    int pog[2]; //pog[0] is horizontal point of gaze; pog[1] is vertical point of gaze
  } EYEDATA;
  
  // must be public to be available to the serial thread
  //bool            m_streamBufferReady;
  long            m_msgLength;
  long            m_dataLength;
  long		  m_updateRate;
  vector<long>    m_itemTypes;
  vector<float>   m_itemScales;
  unsigned char	m_streamBuffer[MAX_MESSAGE_LENGTH];
  vector<string> m_names;
  
  
 private:
  SerialThread *_serialThread;
  
};


#endif
