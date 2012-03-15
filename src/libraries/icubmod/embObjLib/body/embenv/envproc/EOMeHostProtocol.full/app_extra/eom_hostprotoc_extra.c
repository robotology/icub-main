
/* @file       eom_hostprotoc_extra.c
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

#include "eom_hostprotoc_specialise.h"



// -- gpio part

#include "EOMtheCallbackManager.h"

#include "EOtheGPIO.h"
#include "eOcfg_GPIO_MCBSTM32c.h"
#include "EOMtheGPIOManager.h"

#include "EOioPinOutputManaged.h"
#include "EOioPinInputManaged.h"
#include "EOioPinOutput.h"


// -- nvs part

// all that is enough for the host board
#include "eOcfg_EPs_rem_board.h"
#include "EOhostTransceiver.h"







// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eom_hostprotoc_extra.h"

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
static void s_eom_hostprotoc_extra_echoer_asimm_init(void);
#endif

#if     defined(USE_ECHOER)
static void s_eom_hostprotoc_extra_echoer_receive(EOpacket* rxpkt);
#endif

#if     defined(USE_ASIMM)
static void s_eom_hostprotoc_extra_asimm_receive(EOpacket* rxpkt);
#endif


#if     defined(USE_PROTOCOLTRANSCEIVER)
static void s_eom_hostprotoc_extra_protocoltransceiver_init(void);
static void s_eom_hostprotoc_extra_protocoltransceiver_receive(EOpacket* rxpkt);
static void s_eom_hostprotoc_extra_protocoltransceiver_addoccasionalrop(void);

static void s_eom_hostprotoc_extra_protocoltransceiver_ask_the_board(void);
static void s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board(void);
static void s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid);
static void s_eom_hostprotoc_extra_protocoltransceiver_transmit_ropframe(void);
#endif



static void s_callback_button_user(void *arg);
static void s_callback_button_tamp(void *arg);
static void s_callback_button_wkup(void *arg);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

EOpacket* s_mytxpkt = NULL;


// -- nvs part

static EOtransceiver *pc104txrx = NULL;

static const eOipv4addr_t nvs_ems00_ipaddress       = EO_COMMON_IPV4ADDR(10, 255, 39, 151);
//static const eOipv4addr_t nvs_ems00_ipaddress       = EO_COMMON_IPV4ADDR(10, 255, 36, 127);


static const eOipv4port_t nvs_base_endpoint_iport   = 33333;

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern void eom_hostprotoc_extra_environment_init(void)
{
    EOaction s_fullaction;

    // start gpio handler
    eom_gpioman_Initialise(eo_gpio_Initialise(eo_cfg_gpio_mcbstm32c_Get()), NULL);

    // init three callbacks on USER, TAMP, and WKUP
    eo_action_SetCallback(&s_fullaction, s_callback_button_user, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    eo_iopininpman_ActionOn_Register(eo_iopininpman_GetHandle(iopinID_Iman_mcbstm32c_BUTTON_USER), 
                                     &s_fullaction, eo_iopinTrig_OnRiseStay, 3*eok_reltime100ms);

    eo_action_SetCallback(&s_fullaction, s_callback_button_tamp, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    eo_iopininpman_ActionOn_Register(eo_iopininpman_GetHandle(iopinID_Iman_mcbstm32c_BUTTON_TAMP), 
                                     &s_fullaction, eo_iopinTrig_OnRiseStay, 3*eok_reltime100ms);

    eo_action_SetCallback(&s_fullaction, s_callback_button_wkup, NULL, eom_callbackman_GetTask(eom_callbackman_GetHandle()));
    eo_iopininpman_ActionOn_Register(eo_iopininpman_GetHandle(iopinID_Iman_mcbstm32c_BUTTON_WKUP), 
                                     &s_fullaction, eo_iopinTrig_OnRiseStay, 3*eok_reltime100ms);

}

extern void eom_hostprotoc_extra_transceiver_init(void)
{    
    // init transceiver
#if     defined(USE_ECHOER) || defined(USE_ASIMM)

    s_eom_hostprotoc_extra_echoer_asimm_init();

#elif   defined(USE_PROTOCOLTRANSCEIVER)

    s_eom_hostprotoc_extra_protocoltransceiver_init();

#endif


    // impose that on rx pkt the tasl udp calls ...

#if     defined(USE_ECHOER) 

    eom_hostprotoc_specialise_onpktreceived_set(s_eom_hostprotoc_extra_echoer_receive);

#elif    defined(USE_ASIMM)

    eom_hostprotoc_specialise_onpktreceived_set(s_eom_hostprotoc_extra_asimm_receive);

#elif   defined(USE_PROTOCOLTRANSCEIVER)

    eom_hostprotoc_specialise_onpktreceived_set(s_eom_hostprotoc_extra_protocoltransceiver_receive);

#endif    

    
       
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

#if     defined(USE_ECHOER) || defined(USE_ASIMM)
static void s_eom_hostprotoc_extra_echoer_asimm_init(void)
{
    s_mytxpkt = eo_packet_New(512);
    eo_packet_Size_Set(s_mytxpkt, 512);
}
#endif

#if     defined(USE_ASIMM)
static void s_eom_hostprotoc_extra_echoer_receive(EOpacket* rxpkt)
{
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    // at first attempt to connect.

    if(eobool_false == eom_hostprotoc_specialise_connect(remaddr))
    {
        return;
    }


    // then send back teh very same pkt

    eom_hostprotoc_specialise_transmit(rxpkt);

}
#endif


#if     defined(USE_ASIMM)
static void s_eom_hostprotoc_extra_asimm_receive(EOpacket* rxpkt)
{
    eOipv4addr_t remaddr;
    eOipv4port_t remport;

    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    // at first attempt to connect.

    if(eobool_false == eom_hostprotoc_specialise_connect(remaddr))
    {
        return;
    }


    // then send back teh 260 bytes pkt

    eom_hostprotoc_specialise_transmit(s_mytxpkt);

}
#endif


#if     defined(USE_PROTOCOLTRANSCEIVER)

static void s_eom_hostprotoc_extra_protocoltransceiver_init(void)
{
    eOhosttransceiver_cfg_t hosttxrxcfg = 
    {
        .vectorof_endpoint_cfg          = eo_cfg_EPs_vectorof_rem_board,
        .hashfunction_ep2index          = eo_cfg_nvsEP_rem_board_fptr_hashfunction_ep2index,
        .remoteboardipv4addr            = nvs_ems00_ipaddress,
        .remoteboardipv4port            = nvs_base_endpoint_iport
    };  

    EOhostTransceiver *hosttxrx = NULL;

    hosttxrx = eo_hosttransceiver_New(&hosttxrxcfg);
    pc104txrx = eo_hosttransceiver_Transceiver(hosttxrx);

}

static void s_eom_hostprotoc_extra_protocoltransceiver_receive(EOpacket* rxpkt)
{
    uint16_t numofrops_ems00_rx;
    static uint32_t count = 0;
//    uint16_t numofrops_ems00_tx;
    eOabstime_t txtime;
//    EOpacket *txpkt = NULL;
    eOipv4addr_t remaddr;
    eOipv4port_t remport;
    char str[128];
    uint8_t *ipv4array;

    // get addressing of received packet
    eo_packet_Addressing_Get(rxpkt, &remaddr, &remport);

    ipv4array = (uint8_t*)&remaddr;

    snprintf(str, sizeof(str)-1, "xxx: cont = %d --> received from ip = %d.%d.%d.%d", count++, ipv4array[0], ipv4array[1], ipv4array[2], ipv4array[3]); 
    hal_trace_puts(str);


    // -- process a received packet: rxpkt
    eo_transceiver_Receive(pc104txrx, rxpkt, &numofrops_ems00_rx, &txtime);

    snprintf(str, sizeof(str)-1, "zzz - a total of %d rops", numofrops_ems00_rx); 
    hal_trace_puts(str);


//    // -- add an occasional packet
//    s_eom_hostprotoc_extra_protocoltransceiver_addoccasionalrop();
//
//
//    // -- retrieve the packet to transmit: txpkt
//    eo_transceiver_Transmit(ems00txrx, &txpkt, &numofrops_ems00_tx);
//
//
//    // -- set the remote address and port 
//    eo_packet_Addressing_Set(txpkt, remaddr, remport);

//    // -- attempt to connect.
//    if(eobool_false == eom_hostprotoc_specialise_connect(remaddr))
//    {
//        return;
//    }
//
//    // -- assign the packet to the transmitter
//    eom_hostprotoc_specialise_transmit(txpkt);

}

static void s_eom_hostprotoc_extra_protocoltransceiver_addoccasionalrop(void)
{
    static uint32_t step = 1;
    char str[128];


    if(4==(step%10))
    {

        eo_transceiver_ropinfo_t ropinfo;

        ropinfo.ropcfg      = eok_ropconfig_basic;
        ropinfo.ropcode     = eo_ropcode_ask;
        ropinfo.nvep        = EOK_cfg_nvsEP_base_endpoint;
        ropinfo.nvid        = EOK_cfg_nvsEP_base_NVID__gotoprocess;
        
        eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);

        snprintf(str, sizeof(str)-1, "               - added a ask<__gotoprocess>");
        hal_trace_puts(str); 

    }
    else if(8==(step%10))
    {
        eo_transceiver_ropinfo_t ropinfo;

        ropinfo.ropcfg      = eok_ropconfig_basic;
        ropinfo.ropcode     = eo_ropcode_ask;
        ropinfo.nvep        = EOK_cfg_nvsEP_base_endpoint;
        ropinfo.nvid        = EOK_cfg_nvsEP_base_NVID__localise;

        eo_cfg_nvsEP_base_usr_rem_anydev_mem_local->localise = 0;
        
        eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);

        snprintf(str, sizeof(str)-1, "               - added a set<__localise, %d>", 0);
        hal_trace_puts(str); 

    }



    if(3==(step%10))
    {
        eo_transceiver_ropinfo_t ropinfo;

        ropinfo.ropcfg      = eok_ropconfig_basic;
        ropinfo.ropcode     = eo_ropcode_ask;
        ropinfo.nvep        = EOK_cfg_nvsEP_base_endpoint;
        ropinfo.nvid        = EOK_cfg_nvsEP_base_NVID__bootprocess;
//        eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);

//            snprintf(str, sizeof(str)-1, "               - added a ask<__boardinfo>");
//            hal_trace_puts(str); 
    }



    step += 2;
}


#endif



static void s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board(void)
{
    EOarray *upto10 = (EOarray*) & eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_local->upto10rop2signal;
    eOropSIGcfg_t sigcfg;
    char str[128];
    static uint8_t reset = 0;

    eo_array_Reset(upto10);

    if(0 == reset)
    {

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork;
        sigcfg.plustime = 1;
        eo_array_PushBack(upto10, &sigcfg);
    
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__bootprocess;
        sigcfg.plustime = 1;
        eo_array_PushBack(upto10, &sigcfg);
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__applicationinfo;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__boardinfo;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__remoteipaddress;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);
    
        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__remoteipport;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);    
#if 0
#endif

    }

    s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eo_ropcode_set, EOK_cfg_nvsEP_mngmnt_endpoint, EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal);  

    if(1 == reset)
    {
        snprintf(str, sizeof(str)-1, "added a set<__upto10rop2signal, empty-list>");
        hal_trace_puts(str);     
    }
    else
    {
        snprintf(str, sizeof(str)-1, "added a set<__upto10rop2signal, list>");
        hal_trace_puts(str);        
    }    


    reset = (0==reset) ? (1) : (0);
}

static void s_eom_hostprotoc_extra_protocoltransceiver_ask_the_board(void)
{
    char str[128];

    s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__applicationinfo);  
    
    snprintf(str, sizeof(str)-1, "added a ask<__applicationinfo>");
    hal_trace_puts(str); 
}

static void s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid)
{
    eo_transceiver_ropinfo_t ropinfo;

    ropinfo.ropcfg      = eok_ropconfig_basic;
    ropinfo.ropcode     = opc;
    ropinfo.nvep        = ep;

    ropinfo.nvid = nvid;
    eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);
}

static void s_eom_hostprotoc_extra_protocoltransceiver_transmit_ropframe(void)
{
    EOpacket* txpkt;
    
    eOipv4addr_t remaddr;
    eOipv4port_t remport;
    uint16_t numofrops_tx;

    // -- retrieve the packet to transmit: txpkt
    eo_transceiver_Transmit(pc104txrx, &txpkt, &numofrops_tx);

    eo_packet_Addressing_Get(txpkt, &remaddr, &remport);

    // -- attempt to connect.
    if(eobool_false == eom_hostprotoc_specialise_connect(remaddr))
    {
        return;
    }

    // -- assign the packet to the transmitter
    eom_hostprotoc_specialise_transmit(txpkt);
}



static void s_callback_button_user(void *arg)
{
    char str[128];

    snprintf(str, sizeof(str)-1, "called callback on BUTTON_USER: tx a ask<>");
    hal_trace_puts(str);

    s_eom_hostprotoc_extra_protocoltransceiver_ask_the_board();

    s_eom_hostprotoc_extra_protocoltransceiver_transmit_ropframe();
}

static void s_callback_button_tamp(void *arg)
{
    char str[128];

    snprintf(str, sizeof(str)-1, "called callback on BUTTON_TAMP: tx a set<regulars>");
    hal_trace_puts(str);

    s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board();

    s_eom_hostprotoc_extra_protocoltransceiver_transmit_ropframe();
}

static void s_callback_button_wkup(void *arg)
{
    char str[128];

    snprintf(str, sizeof(str)-1, "called callback on BUTTON_WKUP: tx a ropframe");
    hal_trace_puts(str);

    s_eom_hostprotoc_extra_protocoltransceiver_transmit_ropframe();
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



