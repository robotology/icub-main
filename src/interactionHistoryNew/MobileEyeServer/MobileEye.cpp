#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "SerialThread.h"
#include "ASLSerialOutDecode.h"
#include "MobileEye.h"

MobileEye::MobileEye() {

  // fill types array with typical types
  // if necessary, we change it on case-by-case basis
  m_itemTypes.push_back(ASL_TYPE_BYTE);
  m_itemTypes.push_back(ASL_TYPE_SHORT);
  m_itemTypes.push_back(ASL_TYPE_BYTE);
  for (int i=3; i<5; i++)
    m_itemTypes.push_back(ASL_TYPE_SHORT);
  
  // Set scales to default (no scaling)
  for (int i=0; i<5; i++)
    m_itemScales.push_back(0.0);
  
  m_names.push_back("status");			// 0
  m_names.push_back("pupil_diam");		// 1
  m_names.push_back("spare_byte");		// 2
  m_names.push_back("horz_gaze_coord");   	// 3
  m_names.push_back("vert_gaze_coord");	        // 4
  

  _serialThread=new SerialThread(this);
  
}

MobileEye::~MobileEye() {
  stop();
}


bool MobileEye::start() {

  // Calculate total data size
  m_dataLength = 0;
  for (vector<long>::iterator it = m_itemTypes.begin(); it != m_itemTypes.end(); ++it)
    m_dataLength += SerialOutDecode::GetTypeSize(*it);
  
  // Calculate message length based on data length and trasfer mode.
  // In streaming mode additional bytes are added to the message
  m_msgLength = m_dataLength + (m_dataLength-1)/7 + 1;
  
  // Check message length for limits
  //if (m_msgLength <= 0)
  // {
  //  m_error = "Zero or negative message length";
  //  return E_INVALIDARG;
  // }
  //if (m_msgLength > MAX_MESSAGE_LENGTH)
  // {
  //  m_error = "Message length exceeds maximum";
  //  return E_INVALIDARG;
  //}
  
  // Initialize internal flags
  //m_continuousReading = false;
  //m_streamBufferReady = false;
  //m_requestData = false;
  //m_connected = true;
  
  fprintf(stderr,"Starting serial thread \n");
  return _serialThread->start();
}



bool MobileEye::stop(void) {
  _serialThread->stop();
  return true;
}


bool MobileEye::receive(Bottle& msg)
{
  timeval timev;
  static double lasttime = 0.0;
  
  gettimeofday(&timev, NULL);
  double currtime=timev.tv_sec+(timev.tv_usec/1000000.0);
  //fprintf(stderr,"%.6lf seconds elapsed\n", currtime-lasttime);
  lasttime=currtime; 
  
  // Get data from the thread
  unsigned char decodedBuf[MAX_MESSAGE_LENGTH];
  // Prevent data access from thread
  _mutex.wait();
  SerialOutDecode::decode_buffer(m_streamBuffer, decodedBuf, m_dataLength);
  _mutex.post();
  
  // parse the buffer
  vector<int> output;
  if (SerialOutDecode::parse_generic_buffer(decodedBuf, m_dataLength, &m_itemTypes, &m_itemScales, &output) < 0)
    {
      //m_error = "Error parsing a message in streaming mode";
      //return E_FAIL;
    }
  

  EYEDATA ed;
  ed.stat = output[0];
  ed.pd = output[1];
  ed.pog[0] = output[3];
  ed.pog[1] = output[4];
  
  //fprintf(stderr,"pupil %d h %d v %d \n",ed.pd, ed.pog[0],ed.pog[1]);      
  
  // Put message in the bottle
  msg.addString("time");
  msg.addDouble(currtime);
  msg.addString("pupil dia");
  msg.addInt(ed.pd);
  msg.addString("horiz gaze");
  msg.addInt(ed.pog[0]);
  msg.addString("vert gaze");
  msg.addInt(ed.pog[1]);
  
  return true;
}


// bool MobileEye::receive(Bottle& msg)
// {
//   static BYTE msg_buf[150];
//   static int buflen = 0;
//   static int start = 0;
//   BYTE message[150];
//   BYTE decoded_message[150];
//   timeval timev;
//   static double lasttime = 0.0;
  
//   //this function call blocks
//   ssize_t bytes_read = _serial_dev.recv ((void *) message, 150);
  
//   if (bytes_read == -1)
//     ACE_ERROR((LM_ERROR, ACE_TEXT ("Error on MobileEye : receive \n")));
  
//   //ACE_OS::printf("Data received in MobileEye receive: %d bytes, %d in buffer, %d start \n",bytes_read,buflen,start); 

//   if (bytes_read == 0) {  //nothing there
//     return true;

//   } 
//   //add message to buffer and see if it is full
//   for(int i = 0; i < bytes_read; i++) {
//     msg_buf[start+buflen+i] = message[i];
//   }
//   buflen+=bytes_read;
  
//   if(buflen < MSG_LEN) {
//     //didn't get a full message
//     return true;
//   //message[bytes_read] = 0;
//   }
  
//   //find the start of a well-formed 10 byte message
//   for(int i=0; i < buflen; i++) {
//     if(msg_buf[i] & 0x80) {
//       start = i;
//       //fprintf(stderr,"byte %d is start of message \n",i);
//       break;
//     }
//   }
  
//   if((buflen-start) < MSG_LEN) {
//     //there still isn't a full well-formed message in the buffer
//     buflen = buflen - start;
//     return true;
//   }

//   gettimeofday(&timev, NULL);
//   double currtime=timev.tv_sec+(timev.tv_usec/1000000.0);
//   //fprintf(stderr,"%.6lf seconds elapsed\n", currtime-lasttime);
//   lasttime=currtime; 
  
//   //parse the message into a sensible msg format here
//   decode_buffer(&(message[start]), decoded_message,bytes_read-start);
  
  
//   //put each byte of data received this time into the bottle
//   //for(int i; i < bytes_read; i++) {
//   EYEDATA ed = ParseEyeBuf(decoded_message);
  
//   //fprintf(stderr,"pupil %d h %d v %d \n",ed.pd, ed.pog[0],ed.pog[1]);      
  
//   // Put message in the bottle
//   msg.addString("time");
//   msg.addDouble(currtime);
//   msg.addString("pupil dia");
//   msg.addInt(ed.pd);
//   msg.addString("horiz gaze");
//   msg.addInt(ed.pog[0]);
//   msg.addString("vert gaze");
//   msg.addInt(ed.pog[1]);

//   buflen = 0;
//   start = 0;
  
//   //}
//   return true;
// }
