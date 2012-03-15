#include "udp.h"

Udp::Udp()
{
  state = UDP_STATE_NOINIT;
  error = UDP_ERROR_NOERROR;
  connected_clients = 0;
  bounded = FALSE;
  remote_port = -1;
  local_port = -1;
  newData = 0;
  lBuffer = 0;
}

Udp::~Udp()
{
  this->disconnect();

  if (this->lBuffer > 0)
    delete [] this->buffer;
}

/*
int Udp::setDestAddress(struct sockaddr_in *addr)
{
  strcpy((char*)&(udp_addr[client_id]), (char*) addr);
  return UDP_ERROR_NOERROR;
}
*/

int Udp::initLocal( int recv_port)
{
	int broadcastPermission = 1;

	if(bounded)
	{
		printf("socket already bounded (port %d)!!\n", recv_port);
		return -1;
	}
	else
	{
		printf("Bounding socket ...");
	}

	  // Create the UDP socket
	  this->udp_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	  if (this->udp_s <= 0)
	    return 0;

	  // Configurando local
	  struct sockaddr_in addrL;
	  addrL.sin_family = AF_INET;
	  addrL.sin_port = htons(recv_port);
	  addrL.sin_addr.s_addr = INADDR_ANY;			// any interfaccia!! (eth0, eth1, wlan0 ecc...)

	  /* Set socket to allow broadcast */
	  if (setsockopt(this->udp_s, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof(broadcastPermission)) < 0)
		  printf("setsockopt() broadcast failed\n");

	  if (bind(udp_s, (struct sockaddr*)&addrL, sizeof(addrL)) == -1)
		  return 0;

	bounded = TRUE;
	printf("done\n");
	return 1;
}

int Udp::connect(char *address, int send_port, int recv_port)
{
	int broadcastPermission = 1;
	//this->disconnect();

  this->state = UDP_STATE_DISCONNECTED;
  this->error = UDP_ERROR_NOERROR;

  /*
  // Checking the ports
  if(send_port < 0 || send_port > 32767)
    return 0;


  if(recv_port < 0 || recv_port > 32767)
    return 0;
*/

  if(!bounded)
  {
	  // Create the UDP socket
	  this->udp_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	  if (this->udp_s <= 0)
		  return 0;

	  // If local port is undefined, use the same as send port
	  if(recv_port == -1)
		  recv_port = send_port;

	  // Configurando local
	  struct sockaddr_in addrL;
	  addrL.sin_family = AF_INET;
	  addrL.sin_port = htons(recv_port);
	  addrL.sin_addr.s_addr = INADDR_ANY;

	  if (bind(udp_s, (struct sockaddr*)&addrL, sizeof(addrL)) == -1)		// ERROR
		return 0;

	 bounded = TRUE;
  }

  // Configurando remoto
  udp_addr[connected_clients].sin_family = AF_INET;
  udp_addr[connected_clients].sin_port = htons(send_port);
  udp_addr[connected_clients].sin_addr.s_addr = inet_addr(address);
  //printf("\nudp_addr[connected_clients].sin_addr.s_addr: 0x%08X\n", udp_addr[connected_clients].sin_addr.s_addr);

  /* Set socket to allow broadcast */
  if (setsockopt(this->udp_s, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof(broadcastPermission)) < 0)
      printf("setsockopt() broadcast failed\n");

  // Resuelto del nombre de la direccion remota
  struct hostent *host;

  if (udp_addr[connected_clients].sin_addr.s_addr == INADDR_NONE)
  {
	  host = NULL;
	  host = gethostbyname(address);
	  if (host == NULL)
		  return 0;

	  memcpy(&(udp_addr[connected_clients]).sin_addr, host->h_addr_list[0], host->h_length);
  }

  this->state = UDP_STATE_CONNECTED;
  connected_clients++;

  // Lancio il thread
  printf("\n> > >UDP connect done!");
  pthread_mutex_init(&this->mutex, NULL);
  //pthread_create(&this->thread, NULL, this->udp_thread, (void*)this);


  return 1;
}

int Udp::disconnect()
{
  if (state == UDP_STATE_CONNECTED)
  {
    state = UDP_STATE_DISCONNECTED;
    int ret = close(udp_s);

    // Espero a que se cierren los threads
    usleep(100 *1000);

    pthread_mutex_destroy(&mutex);

    return (ret == 0);
  }

  return 1;
}

int Udp::send(char *data, int client_id)
{
  int ret = sendto(udp_s, (char*)data, lBuffer, 0, (struct sockaddr*)&this->udp_addr[client_id], sizeof(udp_addr[client_id]));
  //printf("udp_addr[client_id].sin_addr.s_addr: 0x%06X\n", udp_addr[client_id].sin_addr.s_addr);

  if (ret <= 0)
  {
    this->error = UDP_ERROR_SEND;
    return 0;
  }

  return ret;
}

int Udp::receive(char *data)
{
  if (!newData)
    return 0;

  memcpy(data, buffer, lBuffer);
  newData = 0;

  return 1;
}

int Udp::recv(char *data)
{
  return recvfrom(this->udp_s, (char*)data, this->lBuffer, 0, NULL, NULL);
}

int Udp::setBufferSize(int size)
{
  if (size <= 0)
    return 0;

  if (this->lBuffer != 0)
    delete [] this->buffer;

  this->lBuffer = size;
  this->buffer = new char[this->lBuffer];

  return this->lBuffer;
}

int Udp::getState()
{
  return this->state;
}

int Udp::getfd()
{
	return this->udp_s;
}

int Udp::getError()
{
  return this->error;
}

void *Udp::udp_thread(void *arg)
{
  Udp *udp = (Udp*)arg;
  printf("Thread started\n");

  if (udp->lBuffer <= 0)
  {
    pthread_exit(NULL);
    return NULL;
  }

  int ret;

  while (udp->getState() == UDP_STATE_CONNECTED)
  {
    sleep(0);

    ret = recvfrom(udp->udp_s, udp->buffer, udp->lBuffer, 0, NULL, NULL);

    if (ret > 0)
    {
      udp->newData = 1;
      printf("New packet\n");
    }
  }

  pthread_exit(NULL);
  return NULL;
}
