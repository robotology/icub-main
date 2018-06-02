/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/* @file       main.c
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/23/2013
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include <string>
#include <signal.h>
#include <iostream>

using namespace std;
// Ace stuff
#include <ace/ACE.h>
#include "ace/SOCK_Dgram.h"
#include "ace/Addr.h"
#include "ace/Thread.h"
#include "ace/Logging_Strategy.h"	// for logging stuff


/*#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>
*/


//#include "EoCommon.h"

#include "template_buttons.hpp"

#include "OPCprotocolManager.h"
#include "OPCprotocolManager_Cfg.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
#define DEFAULT_LOCAL_IP 		"10.0.1.104"
#define DEFAULT_PORT 			4444
#define PAYLOAD_MAX_SIZE		128


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
OPCprotocolManager *opcMan_ptr = NULL;



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
typedef struct
{
	string	 		address_string;
	ACE_INET_Addr	addr;
} hostAddr_t;



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
static void sighandler(int _signum);
void usage(void);
void *recvThread(void * arg);

#ifdef __cplusplus
extern "C" {
#endif
	extern opcprotman_cfg_t* get_OPCprotocolManager_cfg(void);
#ifdef __cplusplus
}
#endif

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialization) of static variables
// --------------------------------------------------------------------------------------------------------------------
ACE_SOCK_Dgram	*ACE_socket;
int keepGoingOn   = 1;
uint8_t board = 0;

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------
static void s_check_seqNum(ACE_INET_Addr src_addr, uint32_t recseqnum);

static void sighandler(int _signum)
{
  printf("\n Received signal to quit %d\n", _signum);
  keepGoingOn = 0;
  printf("you shoukd write <quit>\n", keepGoingOn);
}


int main(int argc, char *argv[])
{

	ACE_UINT16	    port;
	ACE_INT8	    flags = 0;
	hostAddr_t 	    local, remote;
	ACE_UINT32 	    ip1,ip2,ip3,ip4;

	uint8_t payload_buffer[PAYLOAD_MAX_SIZE];
	uint16_t payload_size = 0;

	ACE_thread_t id_recvThread; //thread manages rx pkt

	uint32_t byte2send = 0;



	//	Register handler to catch CTRL+C
//	if(signal(SIGINT, sighandler) == SIG_IGN)
//		signal(SIGINT, SIG_IGN);
	signal (SIGQUIT, sighandler);
	signal (SIGINT, sighandler);

	// Set default connection data
	local.address_string=string(DEFAULT_LOCAL_IP);
	port = DEFAULT_PORT;



	// parse command line input argument
	if(argc > 1)
	{
		for(int i = 1; i < argc; i++)
		{

			if(strcmp("--loc-ip", argv[i]) == 0 )
			{
				if(i < (argc - 1))
					local.address_string = string(argv[++i]);
				continue;
			}

			if(strcmp("--port", argv[i]) == 0 )
			{
				if(i < (argc - 1))
					port = atoi(argv[++i]);
				continue;
			}

			if((strcmp("-h", argv[i]) == 0) || (strcmp("--help", argv[i]) == 0))
			{
				usage();
				return 0;
			}

			usage();
			return -1;
		}
	}

	usage();

	printf("\nConfiguration is : \n\n");

	// start the udp socket using ace / winsock / psocks
	// Set my own address_string
	sscanf(local.address_string.c_str(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
	local.addr.set(port, (ip1<<24)|(ip2<<16)|(ip3<<8)|ip4 );

	printf("local.address: %s\n", local.address_string.c_str());

	ACE_socket = new ACE_SOCK_Dgram();
	if (-1 == ACE_socket->open(local.addr) )
	{
		printf("Error opening socket!!!\n");
		return -1;
	}
	int n;
	int m = sizeof(n);
	ACE_socket->get_option(SOL_SOCKET,SO_RCVBUF,(void *)&n, &m);
	printf("SO_RCVBUF %d\n", n);
	ACE_socket->get_option(SOL_SOCKET,SO_SNDBUF,(void *)&n, &m);
	printf("SO_SNDBUF %d\n", n);
	ACE_socket->get_option(SOL_SOCKET,SO_RCVLOWAT,(void *)&n, &m);
	printf("SO_RCVLOWAT %d\n", n);


//	// Set destination address_string
//	sscanf(remote01.address_string.c_str(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
//	remote01.addr.set(port, (ip1<<24)|(ip2<<16)|(ip3<<8)|ip4 );
//	remote02.addr.set(3333, (ip1<<24)|(ip2<<16)|(ip3<<8)|ip4 );
//	remoteAddr = eo_common_ipv4addr(ip1,ip2,ip3,ip4);
//
//	printf("remote01.address: %s\n", remote01.address_string.c_str());
//	printf("port is : %d\n\n", port);
//	//check boardN
//	remoteAddr = eo_common_ipv4addr(ip1,ip2,ip3,ip4); // marco (10, 255, 39, 151)
//	eOport = port;



	// Start receiver thread
	printf("Launching recvThread\n");
	if(ACE_Thread::spawn((ACE_THR_FUNC)recvThread, NULL, THR_CANCEL_ENABLE, &id_recvThread)==-1)
	{
		printf(("Error in spawning recvThread\n"));
	}


	//init opc manager
	opcMan_ptr = opcprotman_New(OPCprotocolManager_Cfg_getconfig());
	if(NULL ==opcMan_ptr)
	{
		printf("Error in creating opcprotman\n!");
		return(0);
	}

	while(keepGoingOn)
	{
		//reset pkt
		memset(&payload_buffer[0], 0x00, PAYLOAD_MAX_SIZE);
		char in_cmd[10], aux[2];
		int n_cmd, ret;
		commands();

		ret = scanf("%s", in_cmd);

		if((strcmp("quit", in_cmd) == 0 ))
		{
			keepGoingOn = 0;
			break;
		}
		else
		{
			n_cmd = atoi(in_cmd);
			switch(n_cmd)
			{
				case 0:
					byte2send = callback_button_0(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 1:	//	send one ask rop
					byte2send = callback_button_1(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 2:	// config/reset regular rops
					byte2send = callback_button_2(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 3:	//	send empty rop
					byte2send = callback_button_3(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 4:	//	send a set pid rop
					byte2send = callback_button_4(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 5:	//	send a status sig
					byte2send = callback_button_5(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 6:	//	send a status sig
					byte2send = callback_button_6(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 7:	//	send a status sig
					byte2send = callback_button_7(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 8:	//	send a status sig
					byte2send = callback_button_8(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 9:	//	send a status sig
					byte2send = callback_button_9(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				case 10:
					byte2send = callback_button_10(&payload_buffer[0], PAYLOAD_MAX_SIZE, &remote.addr);
					break;

				default:
					byte2send = false;
					printf("Command not known!\n");
					break;
				}

				if(byte2send > 0)
				{
					ssize_t sentBytes = ACE_socket->send(&payload_buffer[0], byte2send, remote.addr, 0/*flags*/);
					ACE_TCHAR     address[64];					
					remote.addr.addr_to_string(address, 64);
					printf("payload of size %d is sent to %s!! (sentbytes=%ld)\n",  byte2send, address, sentBytes);
				}
				else
				{
					printf("nothing to send");

				}

			}//else

	}


	printf("mainThraed: exit from while..going to sleep\n");
	sleep(2);

	printf("mainThraed: exit from sleep\n");
	//pthread_cancel(thread);
	ACE_Thread::cancel(id_recvThread);


	return(0);
}


// --------------------------------------------------------------------------------------------------------------------
// - functions with internal scope
// --------------------------------------------------------------------------------------------------------------------

void *recvThread(void * arg)
{
	printf("recvThread started\n");

	uint8_t rec_payload_buffer[PAYLOAD_MAX_SIZE];
	uint8_t replay_payload_buffer[PAYLOAD_MAX_SIZE];
	size_t rec_payload_size = 0;
	uint16_t replysize = 0;
	ACE_INET_Addr src_addr;
        ACE_TCHAR     address[64];
	opcprotman_res_t res;


	sockaddr					sender_addr;

	while(keepGoingOn)
	{
		memset(&rec_payload_buffer[0], 0, PAYLOAD_MAX_SIZE);
		rec_payload_size = ACE_socket->recv(&rec_payload_buffer[0], PAYLOAD_MAX_SIZE, src_addr, 0/*flags*/);
		src_addr.addr_to_string(address, 64);
		printf("\n\nReceived pkt of size = %ld from %s\n",  rec_payload_size, address);
		s_check_seqNum(src_addr, opcprotman_getSeqNum(opcMan_ptr, (opcprotman_message_t*)&rec_payload_buffer[0]));
		/*chiama qui funzione parser*/
/*		for(int i =0; i<rec_payload_size; i++)
		{		
			printf("%x", rec_payload_buffer[i]);
		}
*/

		res = opcprotman_Parse(opcMan_ptr, (opcprotman_message_t*)&rec_payload_buffer[0], (opcprotman_message_t*) &replay_payload_buffer[0], &replysize);
		if(opcprotman_OK  != res)
		{
			printf("erron in parser\n");
		}

	}

	printf("recThraed: exit from while\n");

	pthread_exit(NULL);
	return NULL;
}




void usage(void)
{
	printf("usage: \t\t--help		print this help\n");
	printf("\t\t--loc-ip  <xx>	set the ip address of the local machine - needed by yarp - by default %s\n", DEFAULT_LOCAL_IP);
	printf("\t\t--port    <xx>	set the socket port - by default %d\n", DEFAULT_PORT);
}


static uint32_t seqnumList[10] = {0};
static uint8_t isfirstList[10] = {0};
static void s_check_seqNum(ACE_INET_Addr src_addr, uint32_t recseqnum)
{
	uint32_t addr = src_addr.get_ip_address();

	board = addr- 0xa000100;
	if(board >=10)
	{
		printf("error in get board: num=%d, addr=%d\n", board, addr );
	}

	if(isfirstList[board] == 0)
	{
		seqnumList[board] = recseqnum;
		printf("seqnum first board %d num %d\n", board, recseqnum);
		isfirstList[board] = 1;
		return;
	}

	if((seqnumList[board] +1) != recseqnum)
	{
		printf("error in SEQ NUM form board %d: rec %d expected = %d\n",board, recseqnum, seqnumList[board] +1);
		seqnumList[board] = recseqnum;
	}
	else
	{
		printf("received SEQ NUM from board %d:  %d\n",board, recseqnum);
		seqnumList[board]++;
	}



}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------
