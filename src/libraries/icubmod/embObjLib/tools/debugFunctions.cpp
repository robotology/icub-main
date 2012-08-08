/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#include "StdAfx.h"
#endif
#include "debugFunctions.h"
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <netdb.h>
//#include <unistd.h>

//typedef struct
//{
//	uint32_t size;
//	uint32_t progNum;
//	struct timeval time;
//} DeepDebugData;
//
//DeepDebugData 		DDData[MAX_ACQUISITION] = {0};
//uint32_t 			errors[MAX_ACQUISITION] = {0};


uint32_t 			missErr[BOARD_NUM] 		= {0};
uint32_t 			lateErr[BOARD_NUM] 		= {0};
uint32_t 			sizeErr[BOARD_NUM] 		= {0};

static uint32_t		idx[BOARD_NUM]			= {0};

struct timeval 		recvTime[BOARD_NUM];
struct timeval 		prevTime[BOARD_NUM];
struct timeval 		diffTime[BOARD_NUM];

FILE 				*outFile				= stdout;

size_t				maxBytes2Read 			= 1500;


#define eb1_addr inet_addr(eb1_ip)
#define eb2_addr inet_addr(eb2_ip)
#define eb3_addr inet_addr(eb3_ip)
#define eb4_addr inet_addr(eb4_ip)
#define eb5_addr inet_addr(eb5_ip)
#define eb6_addr inet_addr(eb6_ip)

ACE_INET_Addr		eb1(eb1_ip":3333");
ACE_INET_Addr		eb2(eb2_ip":3333");
ACE_INET_Addr		eb3(eb3_ip":3333");
ACE_INET_Addr		eb4(eb4_ip":3333");
ACE_INET_Addr		eb5(eb5_ip":3333");
ACE_INET_Addr		eb6(eb6_ip":3333");


uint8_t				board;
uint32_t			prevNum[BOARD_NUM]		= {0};
uint32_t			progNum[BOARD_NUM]		= {0};

//eOabstime_t		txtime;
ACE_UINT64			txtime;
ACE_UINT16 			size_ini[BOARD_NUM] 	= {0};
//EOropframe		*ropFrame				= NULL;

bool error = false;
int n = 0;

bool check_received_pkt(ACE_INET_Addr *sender_addr, void * pkt, ACE_UINT16 pkt_size)
{
	char  str[64];

	// detect board
	if( *sender_addr == eb1)
	{
		board = 1;
	}
	else if( *sender_addr == eb2)
	{
		board = 2;
	}
	else if( *sender_addr == eb3)
	{
		board = 3;
	}
	else if( *sender_addr == eb4)
	{
		board = 4;
	}
	else if( *sender_addr == eb5)
	{
		board = 5;
	}
	else if( *sender_addr == eb6)
	{
		board = 6;
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
		//a.sin_addr.s_addr = eb4_addr;
		if( p->sin_addr.s_addr == eb4_addr )
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
	error = false;
//	// init ropframe
//	if(NULL == ropFrame)
//	{
//		printf("|\n|\tStarting check on eth packets...\n|\n");
//		ropFrame = eo_ropframe_New();
//	}
//
//	// Get data from the packet
//	if( eores_OK != eo_ropframe_Load(ropFrame, (uint8_t*) pkt, pkt_size, maxBytes2Read))
//	{
//		printf("Error in the ropFrame!!");
//		return false;
//	}
//
//	txtime = eo_ropframe_age_Get(ropFrame);

	char	*tmp = ((char*) pkt);
	memcpy(&txtime,  (&tmp[8]), 8);

	if(idx[board] == 0)
	{
		printf("received packet size %d\n", pkt_size);

		size_ini[board] = pkt_size;
		prevNum[board] = (txtime >> 32);
		gettimeofday(&prevTime[board],NULL);
		idx[board]++;
		//AfxMessageBox("Fisrt packet received");
		printf("Fisrt packet from board=%d\n", board);
		progNum[board] = (txtime >> 32);
	}

	else //if(idx[board] < max_idx)
	{
		if(size_ini[board] != pkt_size)
		{
//			fprintf(outFile, "size of packet %d differs from %d\n", size_ini[board], pkt_size);
			sizeErr[board]++;
		}

		progNum[board] = (txtime >> 32);

		if( (progNum[board] - prevNum[board]) != 1)
		{
			fprintf(outFile,"missing packet: old = %d, new = %d(0x%04X)  - board [%d] \n", prevNum[board], progNum[board], progNum[board], board);
			missErr[board]++;
		}

		gettimeofday(&recvTime[board],NULL);
		timeval_subtract(&diffTime[board], &recvTime[board], &prevTime[board]);
		if(diffTime[board].tv_usec >= SOGLIA)
		{
//			fprintf(outFile, "time between packets %06d.%06d - board [%d] - pkt %d\n", diffTime[board].tv_sec, diffTime[board].tv_usec, board, progNum[board]);
			lateErr[board]++;
		}

//		if (error)
//		{
//			nErr[board]++;
////			printf("\n");
//		}


		if( (0 == (progNum[board] % 100000)) && (progNum[board] != 0) )
		{
			printf(">periodic check:\n\tpkt_size = %d\n" \
					"\tprogNum[%d] = %d\n", pkt_size, board, progNum[board]);
			printf("\ttime between packets %06d.%06d - board [%d] - pkt %d\n\n", diffTime[board].tv_sec, diffTime[board].tv_usec, board, progNum[board]);
			print_data();
		}

		prevTime[board] = recvTime[board];
		prevNum[board] = progNum[board];
		idx[board]++;
	}
	return true;
}

void print_data(void)
{
	char	output[2048], tmp[64];
	sprintf(output,"Printing data...\n");
	for(board = 0; board <BOARD_NUM; board++)
	{
		sprintf(tmp,"errors board[%d] \n",board);
		strcat(output, tmp);
		sprintf(tmp,"missing 	= %d over %d\n", missErr[board], idx[board]);
		strcat(output, tmp);
		sprintf(tmp,"late 		= %d over %d\n", lateErr[board], idx[board]);
		strcat(output, tmp);
		sprintf(tmp,"size 		= %d over %d\n", sizeErr[board], idx[board]);
		strcat(output, tmp);
	}
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
	AfxMessageBox(output);
#else
	printf(output);
#endif
}


#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
// Definition of a gettimeofday function

int gettimeofday(struct timeval *tv, struct timezone *tz)
{
// Define a structure to receive the current Windows filetime
  FILETIME ft;

// Initialize the present time to 0 and the timezone to UTC
  unsigned __int64 tmpres = 0;
  static int tzflag = 0;

  if (NULL != tv)
  {
    GetSystemTimeAsFileTime(&ft);

// The GetSystemTimeAsFileTime returns the number of 100 nanosecond
// intervals since Jan 1, 1601 in a structure. Copy the high bits to
// the 64 bit tmpres, shift it left by 32 then or in the low 32 bits.
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

// Convert to microseconds by dividing by 10
    tmpres /= 10;

// The Unix epoch starts on Jan 1 1970.  Need to subtract the difference
// in seconds from Jan 1 1601.
    tmpres -= DELTA_EPOCH_IN_MICROSECS;

// Finally change microseconds to seconds and place in the seconds value.
// The modulus picks up the microseconds.
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }

  if (NULL != tz)
  {
    if (!tzflag)
    {
      _tzset();
      tzflag++;
    }

// Adjust for the timezone west of Greenwich
      tz->tz_minuteswest = _timezone / 60;
    tz->tz_dsttime = _daylight;
  }

  return 0;
}
#endif
