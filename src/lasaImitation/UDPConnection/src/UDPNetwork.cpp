#include "UDPNetwork.h"

#ifdef WIN32
#pragma warning( disable : 4996)
#endif

UDPNetwork::UDPNetwork(){
  Server = NULL;
  Client = NULL;

  clientHasConnected  = false;
  serverHasConnected  = false;
  connectionError     = false;
  bIsPinging          = false;
  bIsClient           = false;
  bMsgToSend          = false;
  bMsgRecevied        = false;
  bAutoFlush          = true;
  bSingleShootFlush   = false;

  recvMsgSize = 0;
  DeltaT      = 100;
  PingDeltaT  = 1000;
  PingTimeout = 1000;
  
  sprintf(remotePCName,"localhost");
  port  = 1207;
  
}

UDPNetwork::~UDPNetwork(){
  Free();
}


void  UDPNetwork::Free(){
  if(Server !=NULL) delete Server; Server = NULL;
  if(Client !=NULL) delete Client; Client = NULL;
}

void  UDPNetwork::Reset(){
  if(Server != NULL) delete Server;
  if(Client != NULL) delete Client;
  Server = new UDPConnection();
  Client = new UDPConnection();
  
  if(bIsClient)
    Server->InitServer(port+1);
  else
    Server->InitServer(port);
  //Server->AddAllowedClient(127,0,0,1);
  Server->AddAllowedClient(remotePCName);


  if(bIsClient)
    Client->InitClient(remotePCName,port);
  else
    Client->InitClient(remotePCName,port+1);  

  UDPTimer.Start(DeltaT);

  clientHasConnected  = false;
  serverHasConnected  = false;
  connectionError     = false;
  bIsPinging          = false;
  bMsgToSend          = false;
  bMsgRecevied        = false;
  bAutoFlush          = true;  
  recvMsgSize         = 0;
}

void  UDPNetwork::Init(const char *remotePC, int portNo, bool server){
  port = portNo;
  strcpy(remotePCName,remotePC);
  bIsClient = !server;
  Reset();
}

void  UDPNetwork::SetTimers(int poolTimeMs, int testTimeoutMs){
  if(poolTimeMs<0)    poolTimeMs    = 100;
  if(testTimeoutMs<0) testTimeoutMs = 1000;
  
  DeltaT      = poolTimeMs;
  PingDeltaT  = testTimeoutMs;
  PingTimeout = testTimeoutMs;  
}

void  UDPNetwork::SetAutoFlush(bool bAuto){
  bAutoFlush = bAuto;
}

int   UDPNetwork::Step(){
  
  bMsgRecevied = false;
  
  if(UDPTimer.IsDone()){

    if(connectionError){
      fprintf(stderr,"UDPNetwork: Connection Error: Reseting...\n");
      Reset();
    }

    if((!clientHasConnected)||(!serverHasConnected)){
      //Connection phase  
      if(!serverHasConnected){
        Server->WaitForClient(0);        
        if(Server->IsConnected()){
          serverHasConnected = true;
          if(clientHasConnected)
            fprintf(stderr,"UDPNetwork: Connected...\n");
        }
      }
      if(!clientHasConnected){          
        Client->ConnectToServer(0);
        if(Client->IsConnected()){
          clientHasConnected = true;
          if(serverHasConnected)
            fprintf(stderr,"UDPNetwork: Connected...\n");
        }
      }        
    }else{
          
      if(Server->IsConnected()){
        if(Client->HasPing()){
          //printf("Pong\n");;
          Server->Pong(); 
        }else{
          if(bMsgToSend){
            bMsgToSend = false;
            if(!(Server->SendData(msgToSend,MAX_BUFFER_SIZE))){
              connectionError = true;           
            }
          }
          if(!Server->IsConnected()){
            connectionError = true;
          } 
        }
      }else{  
        connectionError = true;
      }
      
      if(Client->IsConnected()){
        if(Client->HasPong()){
          bIsPinging = false;
        }
        if((bIsPinging)&&((UDPPongChrono.ElapsedTimeMs()-PingTimeout)>0)){
          connectionError = true;
        }
        int mSize;
        if((mSize = Client->GetData(recvMsg,MAX_BUFFER_SIZE,0))>0){
          recvMsgSize = mSize;
          if((bAutoFlush)||(bSingleShootFlush)){
            bSingleShootFlush = false;
            while((mSize = Client->GetData(recvMsg,MAX_BUFFER_SIZE,0))>0){
              recvMsgSize = mSize;
            }
          }
          bMsgRecevied = true;
          UDPPingChrono.Start();
        }else{
          if((UDPPingChrono.ElapsedTimeMs()-PingDeltaT)>0){
            if(!bIsPinging){
              //printf("Ping\n");;
              Server->Ping();
              bIsPinging = true;
              UDPPongChrono.Start();
            }
          }            
        }
        if(!Client->IsConnected()){
          connectionError = true;
        }     
      }else{  
        connectionError = true;
      }
    }
    UDPTimer.Start(DeltaT);    
  }
  return IsConnected();
}

bool  UDPNetwork::IsConnected(){
  return (!(connectionError||(!clientHasConnected)||(!serverHasConnected)));
}

void  UDPNetwork::Flush(){
  bSingleShootFlush = true;
}


int   UDPNetwork::GetMessage(char *msg, int maxSize){
  if(bMsgRecevied){
    if(msg !=NULL){
      int size = (maxSize>MAX_BUFFER_SIZE?MAX_BUFFER_SIZE:maxSize);
      memcpy(msg,recvMsg,size);
    }    
    return recvMsgSize;
  }
  return 0;  
}

void  UDPNetwork::SendMessage(const char *msg, int size){
  if(msg !=NULL){
    int _size = (size>MAX_BUFFER_SIZE?MAX_BUFFER_SIZE:size);
    memcpy(msgToSend,msg,_size);
    bMsgToSend = true;
  }
}
