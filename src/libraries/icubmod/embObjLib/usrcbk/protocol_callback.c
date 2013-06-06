/*
 * protocol_callback.c
 *
 *  Created on: Jun 6, 2013
 *  Author: Cardellino Alberto
 */

#include <stdint.h>
#include <string.h>

extern void eo_receiver_callback_incaseoferror_in_sequencenumberReceived(uint64_t rec_seqnum, uint64_t expected_seqnum)
{
#warning "Check of sequence number active"
    printf("Error in sequecnce number!!!! \n Expected %lu, received %lu\n", expected_seqnum, rec_seqnum);
}



