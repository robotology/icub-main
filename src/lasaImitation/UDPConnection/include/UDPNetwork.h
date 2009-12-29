#ifndef __UDPNetwork_H__
#define __UDPNetwork_H__


#include "StdTools/Timer.h"
#include "UDPConnection.h"



class UDPNetwork{

public:
  UDPConnection *Server;
  UDPConnection *Client;

  Timer  UDPTimer;
  Chrono UDPPingChrono;
  Chrono UDPPongChrono;

  bool clientHasConnected;
  bool serverHasConnected;
  bool connectionError;
  bool bIsPinging;

  int DeltaT;
  int PingDeltaT;
  int PingTimeout;
  
  bool bIsClient;
  
  char remotePCName[256];
  int  port;
  
  char msgToSend[MAX_BUFFER_SIZE];
  char recvMsg[MAX_BUFFER_SIZE];
  bool bMsgToSend;
  bool bMsgRecevied;
  int  recvMsgSize;
  
  bool bAutoFlush;
  bool bSingleShootFlush;

public:
  UDPNetwork();
  ~UDPNetwork();
  
  
  void  Free();
  
  void  Reset();
  
  void  Init(const char *remotePC, int portNo, bool server=true);
  void  SetTimers(int poolTimeMs, int testTimeoutMs);
  int   Step();
  bool  IsConnected();
  void  Disconnect();
  
  void  SetAutoFlush(bool bAuto);
  void  Flush();
  
  int   GetMessage(char *msg, int maxSize);
  void  SendMessage(const char *msg, int size);

};

typedef UDPNetwork *pUDPNetwork;

#endif
