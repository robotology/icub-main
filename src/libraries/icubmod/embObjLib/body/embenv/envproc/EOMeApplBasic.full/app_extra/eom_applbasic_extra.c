
/* @file       eom_applbasic_extra.c
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
#include "EOMtask.h"
#include "EOpacket.h"
#include "EOsocketDatagram.h"
#include "EOMmutex.h"
#include "EOaction_hid.h"
#include "EOtheErrorManager.h"

#include "osal.h"
#include "hal.h"
#include "ipal.h"

#include "stdlib.h"

#include "eom_applbasic_specialise.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eom_applbasic_extra.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eom_applbasic_extra_echoer_receive(EOpacket* rxpkt);

static void s_eom_applbasic_extra_asimm_receive(EOpacket* rxpkt);

static void s_eom_applbasic_extra_transceiver_receive(EOpacket* rxpkt);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

EOpacket* s_mytxpkt = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void eom_applbasic_extra_transceiver_init(void)
{
    
    // init transceiver

    s_mytxpkt = eo_packet_New(512);

    eo_packet_Size_Set(s_mytxpkt, 512);
    
    
    
    // impose that on rx pkt the tasl udp calls ...
    
    eom_applbasic_specialise_onpktreceived_set(s_eom_applbasic_extra_echoer_receive);
    //eom_applbasic_specialise_onpktreceived_set(s_eom_applbasic_extra_asimm_receive);
       
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_eom_applbasic_extra_echoer_receive(EOpacket* rxpkt)
{
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    // at first attempt to connect.

    if(eobool_false == eom_applbasic_specialise_connect(remaddr))
    {
        return;
    }


    // then send back teh very same pkt

    eom_applbasic_specialise_transmit(rxpkt);

}


static void s_eom_applbasic_extra_asimm_receive(EOpacket* rxpkt)
{
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    // at first attempt to connect.

    if(eobool_false == eom_applbasic_specialise_connect(remaddr))
    {
        return;
    }


    // then send back teh 260 bytes pkt

    eom_applbasic_specialise_transmit(s_mytxpkt);

}

static void s_eom_applbasic_extra_transceiver_receive(EOpacket* rxpkt)
{

}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



