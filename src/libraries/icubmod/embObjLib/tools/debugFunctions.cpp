/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "debugFunctions.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

//typedef struct
//{
//	uint32_t size;
//	uint32_t progNum;
//	struct timeval time;
//} DeepDebugData;
//
//DeepDebugData 		DDData[MAX_ACQUISITION] = {0};
//uint32_t 			errors[MAX_ACQUISITION] = {0};


uint32_t 			nErr[BOARD_NUM] 		= {0};
static uint32_t		idx[BOARD_NUM]			= {0};

struct timeval 		recvTime[BOARD_NUM];
struct timeval 		prevTime[BOARD_NUM];
struct timeval 		diffTime[BOARD_NUM];

FILE 				*outFile				= stdout;

size_t				maxBytes2Read 			= 1500;

#define eb2_ip "10.0.1.2"
#define eb4_ip "10.0.1.4"

#define eb2_ip "192.168.202.1"
#define eb4_ip "10.255.72.208"

//#ifdef _LINUX_UDP_SOCKET_
#define eb2_addr inet_addr(eb2_ip)
#define eb4_addr inet_addr(eb4_ip)

//#else
ACE_INET_Addr		eb2(eb2_ip":3333");
ACE_INET_Addr		eb4(eb4_ip":3333");
//#endif

uint8_t				board;
uint32_t			prevNum[BOARD_NUM]		= {0};
uint32_t			progNum[BOARD_NUM]		= {0};

eOabstime_t			txtime;
ACE_UINT16 			size_ini[BOARD_NUM] 	= {0};
EOropframe			*ropFrame				= NULL;

bool check_received_pkt(ACE_INET_Addr *sender_addr, void * pkt, ACE_UINT16 pkt_size)
{
	char  str[64];
	// init ropframe
	if(NULL == ropFrame)
	{
		printf("|\n|\tStarting check on eth packets...\n|\n");
		ropFrame = eo_ropframe_New();
	}

	// detect board
	if( *sender_addr == eb2)
	{
		board = 2;
	}
	else if( *sender_addr == eb4)
	{
		board = 4;
	}
	else
	{
		sender_addr->addr_to_string(str, 64, 1);
		printf(" no board %s\n", str);
		board = 1;
		return false;
	}
	return do_real_check(board, pkt, pkt_size);
}


bool check_received_pkt(sockaddr *sender_addr, void * pkt, ACE_UINT16 pkt_size)
{
	char  str[64];
	// init ropframe
	if(NULL == ropFrame)
	{
		printf("|\n|\tStarting check on eth packets...\n|\n");
		ropFrame = eo_ropframe_New();
	}

	sockaddr_in *p = (sockaddr_in *) sender_addr;
	sockaddr_in a;
	a.sin_addr.s_addr = eb2_addr;

	// detect board
	if( p->sin_addr.s_addr == a.sin_addr.s_addr )
	{
		board = 2;
	}
	else
	{
		a.sin_addr.s_addr = eb4_addr;
		if( p->sin_addr.s_addr == a.sin_addr.s_addr )
		{
			board = 4;
		}
		else
		{
			//sender_addr->addr_to_string(str, 64, 1);
			printf(" no board %04X\n", p->sin_addr.s_addr);
			board = 1;
			return false;
		}
	}

	return do_real_check(board, pkt, pkt_size);
}

bool do_real_check(int board, void * pkt, ACE_UINT16 pkt_size)
{
	// Get data from the packet
	eo_ropframe_Load(ropFrame, (uint8_t*) pkt, pkt_size, maxBytes2Read);
//	txtime = eo_ropframe_age_Get(ropFrame);

	if(idx[board] == 0)
	{
		printf("received packet size %d\n", pkt_size);

		size_ini[board] = pkt_size;
		prevNum[board] = (txtime >> 32);
		gettimeofday(&prevTime[board],NULL);
		idx[board]++;
		printf("case 0=%d\n", board);
	}

	else //if(idx[board] < max_idx)
	{
		if(size_ini[board] != pkt_size)
			fprintf(outFile, "size of packet %d differs from %d\n", size_ini[board], pkt_size);

		gettimeofday(&recvTime[board],NULL);
		timeval_subtract(&diffTime[board], &recvTime[board], &prevTime[board]);
		if(diffTime[board].tv_usec >= SOGLIA)
			fprintf(outFile, "time between packets %06d.%06d\n", diffTime[board].tv_sec, diffTime[board].tv_usec);

		progNum[board] = (txtime >> 32);
		if( (progNum[board] - prevNum[board]) > 1)
		{
			fprintf(outFile,"missing packet(board %d) - old = %d, new = %d\n ", board, prevNum[board], progNum[board]);
			nErr[board]++;
		}
		prevTime[board] = recvTime[board];
		prevNum[board] = progNum[board];
		idx[board]++;
	}
	return true;
}

void print_data(void)
{
	for(board = 0; board <BOARD_NUM; board++)
		fprintf(outFile,"errors board[%d] = %d over %d\n",board, nErr[board], idx[board]);
}
