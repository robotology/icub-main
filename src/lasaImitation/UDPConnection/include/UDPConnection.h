#ifndef __UDPConnection_H__
#define __UDPConnection_H__

// WINDOWS PLATFORM
#ifdef WIN32

#include <winsock.h>
#include <io.h>
#include <sys/types.h>

//LINUX PLATFORM
#else 

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#endif


#include <stdio.h>
#include <memory.h>

#define MAX_ALLOWED_CLIENT    8
#define MAX_BUFFER_SIZE     16384


class UDPConnection
{
 private:

#ifdef WIN32
  static void UDPConnectionInit();
  static void UDPConnectionFree();
  static  int UDPConnectionCount;
#endif


 protected:
  enum MsgType{
    MSGTYPE_NONE = 0,
    QUERY_CONNECTION,
    ACCEPT_CONNECTION,
    REFUSE_CONNECTION,
    CLOSE_CONNECTION,
    SEND_DATA,
    PING_CONNECTION,
    PONG_CONNECTION,
    MSGTYPE_LENGTH
  }; 

  enum ConMode{
    CONMODE_NONE = 0,
    CONMODE_SERVER,
    CONMODE_CLIENT
  };

 protected:  
  // Port
  int                  m_port;
  // Socket
  int                  m_socket;
  // Connection Mode
  ConMode              m_ConMode;
  // IsConnected
  bool                 m_Connected;
#ifdef WIN32
  // Current blocking mode
  int                  m_blockMode;
#endif
  // Socket address
  struct sockaddr_in   m_server;
  struct sockaddr_in   m_client;
  // host
  struct hostent      *m_host;

  char                 m_buffer[MAX_BUFFER_SIZE];

  int                  m_nb_allowed_client;
  long int             m_allowed_client[MAX_ALLOWED_CLIENT];
  int                  bHasPing;
  int                  bHasPong;
 public:

  UDPConnection();

  ~UDPConnection();

  // Add an allowed client
  int AddAllowedClient(int ip1, int ip2, int ip3, int ip4);
  int AddAllowedClient(const char * name);

  // Initialize the server socket at a given port
  int InitServer(int port);

  // Wait for an allowed client to connect to the server  
  //  if(block) then wait until a right client is connected
  //  else return immediately
  int WaitForClient(int block);

  int InitClient(const char *serverName, int port); 
  int ConnectToServer(int block);

  int CloseConnection();

  int SendData(const void *data, int data_len);
  int GetData(void * data, int max_data_len, int block);

  int Flush();

  int Ping();
  int Pong(); 
  int HasPing();
  int HasPong(); 

  bool IsConnected();

 protected:
  
  void Clear();
  void Free();

  int SendMessage(MsgType type, const void *data, int data_len);
  int GetMessage(MsgType *type, void *data, int *max_data_len, int block);


  int EqualIP(struct sockaddr_in *adr1, struct sockaddr_in *adr2);
  int EqualIP(struct sockaddr_in *adr1, long int *adr2);
       
  void PrintIP(long int *adr);
  void PrintIP(struct sockaddr_in *sadr);


#ifdef WIN32
 protected:
  void SetBlockMode(int block);
#endif
};

#endif
