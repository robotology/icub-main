
/* @file       eom_applprotoc_extra.c
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

#include "eom_applprotoc_specialise.h"


// -- nvs part

// all that is enough for the local board
#include "eOcfg_EPs_loc_board.h"
#include "EOtheBOARDtransceiver.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eom_applprotoc_extra.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define     USE_PROTOCOLTRANSCEIVER
#undef      USE_ECHOER
#undef      USE_ASIMM


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

#if     defined(USE_ECHOER) || defined(USE_ASIMM)
static void s_eom_applprotoc_extra_echoer_asimm_init(void);
#endif

#if     defined(USE_ECHOER)
static void s_eom_applprotoc_extra_echoer_receive(EOpacket* rxpkt);
#endif

#if     defined(USE_ASIMM)
static void s_eom_applprotoc_extra_asimm_receive(EOpacket* rxpkt);
#endif


#if     defined(USE_PROTOCOLTRANSCEIVER)
static void s_eom_applprotoc_extra_protocoltransceiver_init(void);
static void s_eom_applprotoc_extra_protocoltransceiver_receive(EOpacket* rxpkt);
static void s_eom_applprotoc_extra_protocoltransceiver_addoccasionalrop(void);
#endif

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

EOpacket* s_mytxpkt = NULL;


// -- nvs part

static EOtransceiver *ems00txrx = NULL;

static const eOipv4addr_t nvs_pc104_ipaddress       = EO_COMMON_IPV4ADDR(10, 255, 39, 152);

static const eOipv4port_t nvs_base_endpoint_iport   = 33333;

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void eom_applprotoc_extra_transceiver_init(void)
{    
    // init transceiver
#if     defined(USE_ECHOER) || defined(USE_ASIMM)

    s_eom_applprotoc_extra_echoer_asimm_init();

#elif   defined(USE_PROTOCOLTRANSCEIVER)

    s_eom_applprotoc_extra_protocoltransceiver_init();

#endif


    // impose that on rx pkt the tasl udp calls ...

#if     defined(USE_ECHOER) 

    eom_applprotoc_specialise_onpktreceived_set(s_eom_applprotoc_extra_echoer_receive);

#elif    defined(USE_ASIMM)

    eom_applprotoc_specialise_onpktreceived_set(s_eom_applprotoc_extra_asimm_receive);

#elif   defined(USE_PROTOCOLTRANSCEIVER)

    eom_applprotoc_specialise_onpktreceived_set(s_eom_applprotoc_extra_protocoltransceiver_receive);

#endif    

    
       
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

#if     defined(USE_ECHOER) || defined(USE_ASIMM)
static void s_eom_applprotoc_extra_echoer_asimm_init(void)
{
    s_mytxpkt = eo_packet_New(512);
    eo_packet_Size_Set(s_mytxpkt, 512);
}
#endif

#if     defined(USE_ASIMM)
static void s_eom_applprotoc_extra_echoer_receive(EOpacket* rxpkt)
{
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    // at first attempt to connect.

    if(eobool_false == eom_applprotoc_specialise_connect(remaddr))
    {
        return;
    }


    // then send back teh very same pkt

    eom_applprotoc_specialise_transmit(rxpkt);

}
#endif


#if     defined(USE_ASIMM)
static void s_eom_applprotoc_extra_asimm_receive(EOpacket* rxpkt)
{
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    // at first attempt to connect.

    if(eobool_false == eom_applprotoc_specialise_connect(remaddr))
    {
        return;
    }


    // then send back teh 260 bytes pkt

    eom_applprotoc_specialise_transmit(s_mytxpkt);

}
#endif


#if     defined(USE_PROTOCOLTRANSCEIVER)

static void s_eom_applprotoc_extra_protocoltransceiver_init(void)
{
    eOboardtransceiver_cfg_t boardtxrxcfg = 
    {
        .vectorof_endpoint_cfg          = eo_cfg_EPs_vectorof_loc_board,
        .hashfunction_ep2index          = eo_cfg_nvsEP_loc_board_fptr_hashfunction_ep2index,
        .remotehostipv4addr             = nvs_pc104_ipaddress,
        .remotehostipv4port             = nvs_base_endpoint_iport
    };    

    ems00txrx = eo_boardtransceiver_Initialise(&boardtxrxcfg);

}

static void s_eom_applprotoc_extra_protocoltransceiver_receive(EOpacket* rxpkt)
{
    uint16_t numofrops_ems00_rx;
    uint16_t numofrops_ems00_tx;
    eOabstime_t txtime;
    EOpacket *txpkt = NULL;
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    // get addressing of received packet
    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);


    // -- process a received packet: rxpkt
    eo_transceiver_Receive(ems00txrx, rxpkt, &numofrops_ems00_rx, &txtime);

    // -- add an occasional packet
    s_eom_applprotoc_extra_protocoltransceiver_addoccasionalrop();


    // -- retrieve the packet to transmit: txpkt
    eo_transceiver_Transmit(ems00txrx, &txpkt, &numofrops_ems00_tx);


    // -- set the remote address and port 
    eo_packet_Addressing_Set(txpkt, remaddr, remport);

    // -- attempt to connect.
    if(eobool_false == eom_applprotoc_specialise_connect(remaddr))
    {
        return;
    }

    // -- assign the packet to the transmitter
    eom_applprotoc_specialise_transmit(txpkt);

}

static void s_eom_applprotoc_extra_protocoltransceiver_addoccasionalrop(void)
{
    static uint32_t step = 1;

    return;

    // i add an extra rop only once every 5 transmissions

    if(3==(step%10))
    {
        eo_transceiver_ropinfo_t ropinfo;

        ropinfo.ropcfg      = eok_ropconfig_basic;
        ropinfo.ropcode     = eo_ropcode_sig;
        ropinfo.nvep        = EOK_cfg_nvsEP_base_endpoint;
        ropinfo.nvid        = EOK_cfg_nvsEP_base_NVID__bootprocess;
        eo_transceiver_rop_occasional_Load(ems00txrx, &ropinfo);

//            snprintf(str, sizeof(str)-1, "               - added a sig<__boardinfo>");
//            hal_trace_puts(str); 
    }



    step += 2;
}


#endif



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



