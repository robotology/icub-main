#ifndef _UDP_H_
#define _UDP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <pthread.h>

#include <sys/time.h>

#define TRUE                			(1)
#define FALSE               			(0)

#define MAX_CONNECTIONS		64

// States
#define UDP_STATE_NOINIT		    0
#define UDP_STATE_DISCONNECTED	1
#define UDP_STATE_CONNECTED		  2

// Errors
#define UDP_ERROR_NOERROR		  0
#define UDP_ERROR_GENERIC		  1
#define UDP_ERROR_SEND			  2
#define UDP_ERROR_RECEIVE		  3
#define UDP_ERROR_OVERFLOW		4

class Udp
{
private:
	// State and error
	int state;
	int error;

	// UDP conection
	int     udp_s;
	struct sockaddr_in udp_addr[MAX_CONNECTIONS];
	int	  connected_clients;
	int 	  bounded;
	int     remote_port;
	int     local_port;

	// Pthreads
	pthread_mutex_t mutex;
	pthread_t thread;
	
	// Read buffer
	char *buffer;
	int lBuffer;
	
	// Receive
	int newData;

public:
	Udp();
	~Udp();

	int initLocal( int recv_port);
	int connect(char *address, int send_port, int recv_port = -1);
	int disconnect();
	
	int setBufferSize(int size);
//	int setDestAddress(struct sockaddr_in *addr);

	int send(char *data,  int client_id = 0);
	int receive(char *data);
	int recv(char *data);

	int getState();
	int getfd();
	int getError();

private:
	static void *udp_thread(void *arg);
};

#endif
