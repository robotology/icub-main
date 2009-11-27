#include <yarp/os/Time.h>
#include "ASLSerialOutDecode.h"
#include "ASLCircularBuffer.h"
#include "MobileEye.h"
#include "SerialThread.h"

SerialThread::SerialThread(MobileEye *mob) {
  owner = mob;
}

SerialThread::SerialThread() {

}

SerialThread::~SerialThread() {

}

void SerialThread::run() {

  static CCircularBuffer buffer;
  unsigned char serialBuf[MAX_MESSAGE_LENGTH];
  unsigned char streamBuf[MAX_MESSAGE_LENGTH];
  
  while(1) {

    double before, now;
    before = yarp::os::Time::now();
    
    //this function call blocks
    ssize_t bytes_read = _serial_dev.recv ((void *) serialBuf, MAX_MESSAGE_LENGTH);
  
    if (bytes_read == -1)
      ACE_ERROR((LM_ERROR, ACE_TEXT ("Error on MobileEye : receive \n")));
  
    //ACE_OS::printf("Data received in MobileEye receive: %d bytes, max %d \n",bytes_read, MAX_MESSAGE_LENGTH); 

    if (bytes_read > 0) {  
      if(buffer.Write(bytes_read,serialBuf))
	{
	  // Extract all messages
	  while (buffer.ExtractMessage(owner->m_msgLength, streamBuf))
	    {
	      owner->_mutex.wait();
	      memcpy(owner->m_streamBuffer, streamBuf, owner->m_msgLength);
	      //owner->m_streamBufferReady = true;
	      owner->_mutex.post();
	    }
	}
      //else {
      // Sleep(10);
      //}
      /// wait 50 ms.
      //now = yarp::os::Time::now();
      //if ((now - before)*1000 < 50) {
      //double k = 0.05-(now-before);
      //}

    }
    //yarp::os::Time::delay(1000);
  } 
}

bool SerialThread::threadInit() {
  //These settings are required by the MobileEye software and cannot be modified
  strcpy(CommChannel,"/dev/ttyUSB0");
  SerialParams.baudrate = 57600;
  SerialParams.xonlim = 0;
  SerialParams.xofflim = 0;
  SerialParams.readmincharacters = 1;
  SerialParams.readtimeoutmsec = 100;
  SerialParams.parityenb = 0;
  SerialParams.paritymode = "NONE";
  SerialParams.ctsenb = 0;
  SerialParams.rtsenb = 0;
  SerialParams.xinenb = 0;
  SerialParams.xoutenb = 0;
  SerialParams.modem = 0;
  SerialParams.rcvenb = 0;
  SerialParams.dsrenb = 0;
  SerialParams.dtrdisable = 0;
  SerialParams.databits = 8;
  SerialParams.stopbits = 1;
  
  ACE_TRACE("SerialHandler::initialize");
  ACE_OS::printf("Starting Serial Port in %s \n", CommChannel);
  
  // Initialize serial port
  if(_serialConnector.connect(_serial_dev, ACE_DEV_Addr(CommChannel)) == -1)
    { 
      ACE_OS::printf("Invalid communications port in %s \n", CommChannel);
      return false;
    } 
  
  
  // Set TTY_IO parameter into the ACE_TTY_IO device(_serial_dev)
  if (_serial_dev.control (ACE_TTY_IO::SETPARAMS, &SerialParams) == -1)
    {
      ACE_OS::printf("Can not control communications port %s \n", CommChannel);
      return false;
    }
  
  return true;    
}

void SerialThread::threadRelease() {

}

void SerialThread::onStop() {

}
