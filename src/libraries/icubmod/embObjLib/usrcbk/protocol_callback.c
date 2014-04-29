/*
 * protocol_callback.c
 *
 *  Created on: Jun 6, 2013
 *  Author: Cardellino Alberto
 */


#if 0
// marco.accame on 29 apr 2014: we use a cpp_protocol_callback_incaseoferror_in_sequencenumberReceived() function defined inside
//                              hostTransceiver.cpp
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "EoCommon.h"

extern void protocol_callback_incaseoferror_in_sequencenumberReceived(uint32_t remipv4addr, uint64_t rec_seqnum, uint64_t expected_seqnum)
{  
    long long unsigned int exp = expected_seqnum;
    long long unsigned int rec = rec_seqnum;
    printf("Error in sequence number from 0x%x!!!! \t Expected %llu, received %llu\n", remipv4addr, exp, rec);
};

#endif


