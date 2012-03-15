
/* @file       eupdater-parser.c
	@brief      This file keeps the vcececew
	@author     marco.accame@iit.it
    @date       01/11/2012
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "eEcommon.h"
#include "eEmemorymap.h"
#include "EOVtheSystem.h"

#include "stdlib.h"

#include "updater-core.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eupdater_parser.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


typedef struct
{
    eOnanotime_t    client_tx_time;
    eOnanotime_t    server_tx_time;
    uint32_t        client_id;
} pkt_payload_t;

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eObool_t s_eupdater_parser_process_rop_dummy(EOpacket *rxpkt, EOpacket *txpkt);
static eObool_t s_eupdater_parser_process_rop_alessandro(EOpacket *rxpkt, EOpacket *txpkt);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eupdater_parser_init(void)
{

}



// return true if there is a pkt to transmit back
extern eObool_t eupdater_parser_process_rop(EOpacket *rxpkt, EOpacket *txpkt)
{
    //return(s_eupdater_parser_process_rop_dummy(rxpkt, txpkt));
    return(s_eupdater_parser_process_rop_alessandro(rxpkt, txpkt));
}


extern eObool_t eupdater_parser_process_data(EOpacket *rxpkt, EOpacket *txpkt)
{

    return(eobool_false);
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eObool_t s_eupdater_parser_process_rop_alessandro(EOpacket *rxpkt, EOpacket *txpkt)
{
    uint8_t *datarx;
    uint8_t *datatx;
    uint16_t sizerx;
    uint16_t sizetx;

    eOipv4addr_t remaddr = 0;
    eOipv4port_t remport = 0;

    eo_packet_Payload_Get(rxpkt, &datarx, &sizerx);
    eo_packet_Payload_Get(txpkt, &datatx, &sizetx);

    if(1 == upd_core_manage_cmd(datarx, datatx, &sizetx))
    {
        eo_packet_Destination_Get(rxpkt, &remaddr, &remport);

        eo_packet_Payload_Set(txpkt, (uint8_t*)datatx, sizetx);
        //eo_packet_Destination_Set(txpkt, remaddr, remport);
        eo_packet_Destination_Set(txpkt, remaddr, 3333);

        return(eobool_true);
    }

    return(eobool_false);

}

static eObool_t s_eupdater_parser_process_rop_dummy(EOpacket *rxpkt, EOpacket *txpkt)
{
    pkt_payload_t *pktdata = NULL;
    uint8_t *data;
    uint16_t size;
    eOnanotime_t nano = 0;

    eOipv4addr_t remaddr = 0;
    eOipv4port_t remport = 0;

    eo_packet_Destination_Get(rxpkt, &remaddr, &remport);


    eo_packet_Payload_Get(rxpkt, &data, &size);
    //ipaddr = ((uint8_t*)&remaddr);
    pktdata = (pkt_payload_t*)data;


    // prepare tx packet
    pktdata->client_id = 0x10101010;
    eov_sys_NanoTimeGet(eov_sys_GetHandle(), &nano); 

    pktdata->client_tx_time = nano;

    eo_packet_Payload_Set(txpkt, (uint8_t*)pktdata, sizeof(pkt_payload_t));

    eo_packet_Destination_Set(txpkt, remaddr, 3333);

    return(eobool_true);
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



