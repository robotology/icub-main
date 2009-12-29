#include "UDPConnection.h"

//#define  __DEBUG__
//#define __SHOW_ERROR__


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef WIN32
#define usleep(X)     Sleep((X) / 1000)
#define sleep(X)      Sleep((X) * 1000)

#define	MSG_DONTWAIT	0
#define socklen_t     int FAR
#endif

#define DEBUG_MSG(x)         
//printf(x);
#define DEBUG_MSG1(x,p)      
//printf(x,p);
#define DEBUG_MSG2(x,p1,p2)  
//printf(x,p1,p2);
#define DEBUG_CMD(x)         
#define ERROR_MSG(x)         
//printf("Error: ");printf(x);
#define ERROR_MSG1(x,p)      
//printf("Error: ");printf(x,p);
#define ERROR_MSG2(x,p1,p2)  
//printf("Error: ");printf(x,p1,p2);

#define PERROR(x)            
//printf("Error: ");perror(x);

#define ERROR_CMD(x)         
//x;



#ifdef WIN32

int UDPConnection::UDPConnectionCount = 0;

void UDPConnection::UDPConnectionInit(){
  if(UDPConnectionCount==0){
    DEBUG_MSG("Init WinSock.dll\n"); 
	  WORD    wVersionRequested;
	  WSADATA wsaData;
	  int err; 
	  wVersionRequested = MAKEWORD( 2, 2 ); 
	  err = WSAStartup( wVersionRequested, &wsaData );
	  if ( err != 0 ) {
      ERROR_MSG("Cannot find usabe WinSock.dll\n");
	    return;
    }
  }
  UDPConnectionCount++;
}

void UDPConnection::UDPConnectionFree(){
  UDPConnectionCount--;
  if(UDPConnectionCount==0){
    DEBUG_MSG("Free WinSock.dll\n"); 
    WSACleanup( );
  }
}

#endif




UDPConnection::UDPConnection(){

#ifdef WIN32
  UDPConnectionInit();
#endif

  Clear();

}

UDPConnection::~UDPConnection(){
  
  CloseConnection();
  Free();

#ifdef WIN32
  UDPConnectionFree();
#endif
}

void UDPConnection::Clear(){

  m_socket            = 0;  
  m_port              = 0;
  m_host              = NULL;
  m_ConMode           = CONMODE_NONE;
  m_Connected         = false;
#ifdef WIN32
  m_blockMode         = TRUE;
#endif
  bHasPing            = FALSE;
  bHasPong            = FALSE;
  
  m_nb_allowed_client = 0;

  memset(m_allowed_client, 0, MAX_ALLOWED_CLIENT * sizeof(long int));
  memset(m_buffer,         0, MAX_BUFFER_SIZE    * sizeof(unsigned char));
}

void UDPConnection::Free(){
  if(m_socket!=0){
#ifdef WIN32    
    closesocket(m_socket);
#else
    close(m_socket);
#endif    
    m_socket = 0;
  }
  Clear();
}


#ifdef WIN32
void UDPConnection::SetBlockMode(int block){

  if(block==0){
    if(m_blockMode!=0){
      u_long argp = 1;
      int err;
      err = ioctlsocket(m_socket, FIONBIO, &argp);
      m_blockMode = 0;
    }
  }else{
    if(m_blockMode==0){
      u_long argp = 0;
      int err;
      err = ioctlsocket(m_socket, FIONBIO, &argp);
      m_blockMode = 1;
    }
  }
}

#endif




// Initialize the server socket at a given port
int UDPConnection::InitServer(int port){

  if(m_ConMode!=CONMODE_NONE){
    ERROR_MSG("Connection already initialized\n");
    return FALSE;
  }

  DEBUG_MSG1("Server Initialization, port: %d\n",port);

  // Checking port number 
  m_port = port;
  if((port<1024)||(port>65000)){
    ERROR_MSG1("Port number: %d must be 1024<port<65000\n",port);      
    return FALSE;
  }

  // Creating socket
  m_socket = socket(PF_INET,SOCK_DGRAM,0);
  if(m_socket<0){
    ERROR_MSG("Socket initialization error\n");
    return FALSE;
  }

  // Binding socket
  m_server.sin_family      = AF_INET; 
  m_server.sin_addr.s_addr = INADDR_ANY; 
  m_server.sin_port        = htons(m_port);
    
  if(bind(m_socket,(struct sockaddr *)&m_server, sizeof(m_server))<0){
    PERROR("Socket bind error");
    return FALSE;
  }

  m_ConMode = CONMODE_SERVER;

  // Done
  return TRUE;
}


// Initialize the client socket at a given port
int UDPConnection::InitClient(const char *serverName, int port){

  if(m_ConMode!=CONMODE_NONE){
    ERROR_MSG("Connection already initialized\n");
    return FALSE;
  }
  
  m_port = port;
  if((port<1024)||(port>65000)){
    fprintf(stderr,"port doit etre 1024<port<65000\n");      
    return FALSE;
  }

  m_socket = socket(PF_INET,SOCK_DGRAM,0);
  if(m_socket<0){
    fprintf(stderr,"Erreur de creation de socket !!!\n");
    return FALSE;
  }

  m_server.sin_family = AF_INET;
  m_server.sin_port   = htons(m_port);
  
  m_host = gethostbyname(serverName);
  
  if ( m_host == (struct hostent *) 0 ) {
    fprintf(stderr,"Nom d'hote inconnu !!!\n");
    return FALSE;
  }
  memcpy( (char *) &m_server.sin_addr.s_addr,
	        (char *) m_host->h_addr,
	        m_host->h_length);

  DEBUG_MSG2("Client Initialization, server: %s port: %d\n",serverName, port);

  m_ConMode = CONMODE_CLIENT;

  return TRUE;
}


// Wait for an allowed client to connect to the server  
//  if(block) then wait until a right client is connected
//  else return immediately
int UDPConnection::WaitForClient(int block){

  if(m_ConMode!=CONMODE_SERVER){
    ERROR_MSG("Server not initialized\n");
    return FALSE;
  }
  DEBUG_MSG("Waiting for client\n");

#ifdef WIN32
  SetBlockMode(block);
#endif

  // Recieve data
  int first = 1;
  while((block==1)||(first==1)){

    if(first==0)
      sleep(1);

    DEBUG_MSG("Waiting for client\n"); 
    struct sockaddr_in client;
    int client_ln = sizeof(client);
    int n_read = recvfrom(m_socket,
                          m_buffer,
			                    MAX_BUFFER_SIZE,
			                    (block==0? MSG_DONTWAIT: 0),
			                    (struct sockaddr *)&client,
			                    (socklen_t*)&client_ln);
    if(n_read<0){
      ERROR_MSG((block!=0?"Error while fetching data\n":"No client present\n"));
      return FALSE;
    }
    
    int i;
    for(i=0;i<m_nb_allowed_client;i++){
      if(EqualIP(&client,&(m_allowed_client[i]))){
        memcpy(&m_client,&client,client_ln);
    
        m_Connected = true;
	      
	      DEBUG_MSG("Sending accept connection: ");
	      DEBUG_CMD(PrintIP(&m_client));

        if(!SendMessage(ACCEPT_CONNECTION,NULL,0)){
	        ERROR_MSG("Sending accpeting connection\n");
          m_Connected = false;
	      }

	      DEBUG_MSG("Client found: "); 
	      DEBUG_CMD(PrintIP(&client)); 
	      DEBUG_MSG("\n");
        return TRUE;
      }
    }
    
    if(!SendMessage(REFUSE_CONNECTION,NULL,0)){
      ERROR_MSG("Sending refusing connection\n");
    }

    ERROR_MSG("Unallowed client was found: ");
    ERROR_CMD(PrintIP(&client));
    first = 0;
  }

  return FALSE;
}


int UDPConnection::CloseConnection(){
  if(m_ConMode!=CONMODE_SERVER){
    m_Connected = false;
    Free();
    return TRUE;
  }

  if(!SendMessage(CLOSE_CONNECTION,NULL,0))
    return FALSE;

  m_Connected = false;

  return TRUE;
}

int UDPConnection::ConnectToServer(int block){

  if(m_ConMode!=CONMODE_CLIENT){
    ERROR_MSG("Client not initialized\n");
    return FALSE;
  }

  int first = 1;
  while((block==1)||(first==1)){
    if(first==1){
      DEBUG_MSG("Connecting to server\n");      
    }
    if(first==0)
      sleep(1);

    m_Connected = true;
    if(!SendMessage(QUERY_CONNECTION,NULL,0)){
      ERROR_MSG("Sending connection query failed\n");
      m_Connected = false;
      return FALSE;
    }
      
    if(first==1){
      DEBUG_MSG("Waiting for connection\n");
    }


    MsgType type;
    if(GetMessage(&type,NULL,NULL,0)){
      if((type)==ACCEPT_CONNECTION){
        DEBUG_MSG1("Connection accepted: %s\n",(char*)((((int*)m_buffer)+1)));
        m_Connected = true;
        return TRUE;
      }
      m_Connected = false;
      if((type)==REFUSE_CONNECTION){
        DEBUG_MSG1("Connection refused: %s\n",(char*)((((int*)m_buffer)+1)));
        return FALSE;
      }
    }
    m_Connected = false;
    first = 0;
  }
  return FALSE;
}


int UDPConnection::SendMessage(MsgType type, const void *data, int data_len){

  if(!m_Connected)
    return FALSE;

  if(data_len<0)
    return FALSE;

  // Setting the type of message
  ((long int*)m_buffer)[0] = ((long int)type);

  int tmp_data_len = data_len + sizeof(long int);

  if(data_len>0){
    tmp_data_len = (tmp_data_len > MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE : tmp_data_len);
    memcpy(m_buffer+sizeof(long int),data,tmp_data_len);
  }

  int n_send = sendto(m_socket,
            		      m_buffer,
                      tmp_data_len,
		                  0,
                      (m_ConMode==CONMODE_CLIENT ? (struct sockaddr *)&m_server:
                                                   (struct sockaddr *)&m_client),
                      (m_ConMode==CONMODE_CLIENT ? sizeof(m_server):
                                                   sizeof(m_client)));
  if(n_send<0){
    ERROR_MSG("Sending data failed\n");
    return FALSE;
  }

  return TRUE;
}



int UDPConnection::GetMessage(MsgType *type, void *data, int *max_data_len, int block){

  if(!m_Connected)
    return FALSE;

#ifdef WIN32
  SetBlockMode(block);
#endif

  struct sockaddr_in source;
  int source_ln = sizeof(source);
  int n_read = recvfrom(m_socket,
			                  m_buffer,
			                  MAX_BUFFER_SIZE,
			                  (block==0? MSG_DONTWAIT: 0),
			                  (struct sockaddr *)&source,
			                  (socklen_t*)&source_ln);
  if(n_read<0){
    if(block==1){
      ERROR_MSG("Error while fetching data\n");
    }else{
      DEBUG_MSG("No data present\n");
    }
    return FALSE;
  }

  // Check of IP address
  if(EqualIP(&source,(m_ConMode==CONMODE_CLIENT ? &m_server:
                                                   &m_client))){
    MsgType tmpType;
    // Check of message size
    if(n_read>=(int)sizeof(long int)){
      // Retreiving the message header
      tmpType = (MsgType)(((long int*)m_buffer)[0]);
      if(type!=NULL){
        *type = tmpType;
      }

        // Checks it
      if(tmpType >= MSGTYPE_LENGTH){
        ERROR_MSG("Not a valid header\n");
        return FALSE;
      }
      
      if((tmpType == PING_CONNECTION)){
        bHasPing = TRUE;
        return TRUE;  
      }

      if((tmpType == PONG_CONNECTION)){        
        bHasPong = TRUE;
        return TRUE;
      }
        
      
      if((tmpType == CLOSE_CONNECTION)){//||((n_read>1)&&(((char*)m_buffer)[sizeof(long int)]=='q'))){
        //printf("Closing connection\n");
        m_Connected = false;
        return FALSE;
      }

      // Copy data
      if(max_data_len!=NULL){
        if((*max_data_len)>0){
          *max_data_len = (n_read-(int)sizeof(long int)<(*max_data_len)?n_read-(int)sizeof(long int):(*max_data_len));
	        memcpy(data,((long int*)m_buffer)+1,(*max_data_len));
        }
      }

      return TRUE;
    }else{
      ERROR_MSG("Not a valid message (too short)\n");
    }
  }else{
    ERROR_MSG("Bad source IP address\n");
  }
  return FALSE;
}

int UDPConnection::Ping(){
  
  if(!SendMessage(PING_CONNECTION,NULL,0))
    return FALSE;

  return TRUE;
}

int UDPConnection::Pong(){
  if(!SendMessage(PONG_CONNECTION,NULL,0))
    return FALSE;

  return TRUE;
} 

int UDPConnection::HasPing(){
  if(bHasPing){
    bHasPing = FALSE;
    return TRUE;
  }
  return FALSE;
}

int UDPConnection::HasPong(){
  if(bHasPong){
    bHasPong = FALSE;
    return TRUE;
  }
  return FALSE;
} 



int UDPConnection::SendData(const void *data, int data_len){

  return SendMessage(SEND_DATA,data,data_len);
}


int UDPConnection::Flush(){
  if(!m_Connected)
    return FALSE;

  MsgType type;
  while(GetMessage(&type,NULL,NULL,0)){}  

  return TRUE;
}

int UDPConnection::GetData(void * data, int max_data_len, int block){

  MsgType type;
  int data_len = max_data_len;

  if(GetMessage(&type,data,&data_len,block)){
    if(type != SEND_DATA){
      ERROR_MSG("Not a data msg\n");
      return FALSE;
    }
    
    return data_len;
  }

  return FALSE;
}

  
bool UDPConnection::IsConnected(){
  return m_Connected;
}




//////////////////////////////////////////////////////////////////////////////////////////






int UDPConnection::EqualIP(struct sockaddr_in *adr1, struct sockaddr_in *adr2){
  return ((long int)adr1->sin_addr.s_addr) == ((long int)adr2->sin_addr.s_addr);
}

int UDPConnection::EqualIP(struct sockaddr_in *adr1, long int *adr2){
  return ((long int)adr1->sin_addr.s_addr) == ((long int)*adr2);
}
       
void UDPConnection::PrintIP(long int *adr){
  printf("%d.%d.%d.%d\n",((unsigned char*)adr)[0],((unsigned char*)adr)[1],((unsigned char*)adr)[2],((unsigned char*)adr)[3]);
}

void UDPConnection::PrintIP(struct sockaddr_in *sadr){
  long int adr = (long int)sadr->sin_addr.s_addr;
  printf("%d.%d.%d.%d\n",((unsigned char*)&adr)[0],((unsigned char*)&adr)[1],((unsigned char*)&adr)[2],((unsigned char*)&adr)[3]);
}


int UDPConnection::AddAllowedClient(int ip1, int ip2, int ip3, int ip4){
  if(m_nb_allowed_client>=MAX_ALLOWED_CLIENT)
    return FALSE;

  ((char*)&m_allowed_client[m_nb_allowed_client])[0] = ip1;
  ((char*)&m_allowed_client[m_nb_allowed_client])[1] = ip2;
  ((char*)&m_allowed_client[m_nb_allowed_client])[2] = ip3;
  ((char*)&m_allowed_client[m_nb_allowed_client])[3] = ip4;

  m_nb_allowed_client++;
  return TRUE;
}

int UDPConnection::AddAllowedClient(const char * name){
  if(m_nb_allowed_client>=MAX_ALLOWED_CLIENT)
    return FALSE;

  struct hostent *host;

  host = gethostbyname(name);
  if ( host == (struct hostent *) 0 ) {
    fprintf(stderr,"Nom d'hote inconnu !!!\n");
    return FALSE;
  }

  memcpy( (char *) &m_allowed_client[m_nb_allowed_client],
	        (char *) host->h_addr,
	        host->h_length);
  m_nb_allowed_client++;
  return TRUE;
}
