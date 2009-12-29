#ifndef DATALOGGER_H_
#define DATALOGGER_H_

class DataLogger
{
  void*   mBuffers[2];
  void*   mCurrentBuffer;  
  int     mCurrentBufferIndex;
  
};

#endif /*DATALOGGER_H_*/
