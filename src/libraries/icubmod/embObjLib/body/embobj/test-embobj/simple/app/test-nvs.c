
/* @file       test-eonetvar.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       09/08/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

// embobj
#include "EoCommon.h"
#include "EOnetvar.h"
#include "EOnetvar_hid.h"  // only for advanced operations


#include "EOrop_hid.h"

#include "EOtheNVsCfgDevice.h"


#include "EOVtheNVsCfg.h"
#include "EOtheNVs.h"
#include "EOtheParser.h"
#include "EOtheAgent.h"
#include "EOtheFormer.h"

#include "EOarray.h"

#include "EOvport.h"
#include "EOnetvarNode.h"






// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-nvs.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_initialise_all(void);

static void s_smartnode_pktmake_set_vportRO(eOipv4addr_t toipaddr, uint8_t *pktdata);

static eObool_t s_simplenode_pktproc_from(eOipv4addr_t fromipaddr, uint8_t *pktdata, uint16_t pktsize);

static eObool_t s_smartnode_pktproc_from(eOipv4addr_t fromipaddr, uint8_t *pktdata, uint16_t pktsize);

static void s_simplenode_change_some_nvs(void);

static void s_smartnode_pktmake_ask_value(eOipv4addr_t toipaddr, eOnetvarID_t nvid, uint8_t *pktdata);

static void s_smartnode_asks(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr, eOnetvarID_t nvid);

static void s_simplenode_sigs(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr, eOnetvarID_t nvid);

static void s_simplenode_pktmake_sig_value(eOipv4addr_t toipaddr, eOnetvarID_t nvid, uint8_t *pktdata);

static void s_smartnode_configure_vportRO(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr);


static void s_smartnode_configure_its_vportWOof(eOipv4addr_t ipaddr);

static void s_smartnode_sets(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr, eOnetvarID_t nvid);

static void s_smartnode_pktmake_set_value(eOipv4addr_t toipaddr, eOnetvarID_t nvid, uint8_t *pktdata);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static EOrop *rop = NULL;
static EOrop *rop0 = NULL;
static EOrop *reply = NULL;

static uint8_t  pkt2tx_data[128] =  {0};
static uint16_t pkt2tx_size = 0;

static volatile uint32_t aaa32;
static volatile uint16_t aaa16;
static volatile uint8_t  aaa08;

static EOnetvar *nvInp08 = NULL;
static EOnetvar *nvInp16 = NULL;

static uint8_t  *pInp08 = NULL;
static uint16_t *pInp16 = NULL;


static EOnetvar *nvTMP = NULL;

static EOvportCfg *vportcfg = NULL;

static eOropconfig_t ropcfg = {.confrqst = eobool_true, .timerqst = eobool_true, .plussign = eobool_true, .plustime = eobool_true};
static uint32_t sign = 0x21000012;
static uint64_t time = 0xaaaabbbbccccdddd;



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void test_nvs_Init(void)
{

    const eOipv4addr_t ipaddr_smartnode = eo_common_ipv4addr(10, 10, 1, 100);
    const eOipv4addr_t ipaddr_simplenode00 = rem00_device_ipaddr; // or loc_device_ipaddr


    s_initialise_all(); 
 

    s_smartnode_configure_vportRO(ipaddr_smartnode, ipaddr_simplenode00);


    // i am a normal node, and a peripheral changes the values of input08 and input16

    s_simplenode_change_some_nvs();



    // the smart node asks teh value of ...
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_input08__input08value);

    // the smart node asks teh value of ...
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_input08);
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_input08__acquisitiontime);
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_input16_inputconfiguration);
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev__fixedarray);
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev__varsizearray);
    
    




    // i am a smart node which wants to know the value of the ouput inside teh simple node
    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_output__outputvalue);


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // the smart node wants to know the vportROnly.dat of simple node 0

    s_smartnode_asks(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_vportROnly__dat);


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // the simple node 0 signals the value of the vportROnly.dat to the smart node

    s_simplenode_sigs(ipaddr_simplenode00, ipaddr_smartnode, nvIDlocdev_vportROnly__dat);



    // the smart node configures its vportWO relative to simplenode 0.
    s_smartnode_configure_its_vportWOof(ipaddr_simplenode00);

    // the smartnode sends a set-rop to the simplenode 0

    s_smartnode_sets(ipaddr_smartnode, ipaddr_simplenode00, nvIDrem00dev_vportWOnly__dat);



    for(;;)
    {
        // try to understand how a smart node process a reception of a say ...
        // answer: the agent copies the received data into ram of remote NVs and a callback function on_say() shoudl be
        // defined and called. the function is responsible to operate the reply.
        ;
    }


}



extern void test_nvs_Do(void)
{

}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_initialise_all(void)
{
    // the network variables ...
    //eo_nvs_Initialise(eov_nvscfg_Initialise(eo_nvscfg_device_GetHandle()));
    eo_nvs_Initialise(eo_nvscfg_device_GetHandle());
    
    // the parser
    eo_parser_Initialise();

    // the agent
    eo_agent_Initialise(NULL);
    
    // the former
    eo_former_Initialise();

    // some rops
    rop0 = eo_rop_New(128);
    rop = eo_rop_New(128);
    reply = eo_rop_New(128);

    // a vportcfg
    vportcfg = eo_vportcfg_New();

}

static void s_smartnode_pktmake_set_vportRO(eOipv4addr_t toipaddr, uint8_t *pktdata)
{
    uint8_t *vdata = NULL;
    uint16_t tmp16 = 0;
    eOipv4addr_t ipaddr = 0;

    eo_vportcfg_Clear(vportcfg);
    // fill inside a local vportcfg all the NVs i want to be in the remote node
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev_input08__input08value); 
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev_input16__input16value); 
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev_output__outputvalue); 
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev_output__applicationtime); 
    

    // i get the memory and its size
    vdata = eo_vportcfg_GetMemory(vportcfg, &tmp16);

    // now i get the nv whcih contains the cfg of the ronly vport in device with ip address toipaddr
    nvTMP = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDrem00dev_vportROnly__cfg, eo_nv_ownership_remote, toipaddr));

    // now i write the value of local vportcfg inside the nv aand i dont update ... 
    eo_netvar_Set(nvTMP, vdata, eobool_true, eo_nv_upd_dontdo); // i force the write

    // now i initialise the rop to send out 
    eo_agent_OutROPinit(eo_agent_GetHandle(), toipaddr, 
                        eo_ropcode_set, nvIDrem00dev_vportROnly__cfg, ropcfg, rop0);
    
    // i fill it with data
    eo_agent_OutROPfill(eo_agent_GetHandle(), rop0, NULL, NULL);

    // i get a bitstream to send
    eo_former_GetStream(eo_former_GetHandle(), rop0, pktdata, &tmp16, &ipaddr);
}

static eObool_t s_simplenode_pktproc_from(eOipv4addr_t fromipaddr, uint8_t *pktdata, uint16_t pktsize)
{
    uint16_t tmp16;
    eOipv4addr_t ipaddr;

    eObool_t ret = eobool_false;

    eo_parser_GetROP(eo_parser_GetHandle(), pktdata, pktsize, fromipaddr, rop, &tmp16);

    eo_agent_InpROPprocess(eo_agent_GetHandle(), rop, reply);

    // if i have a packet to send back i send it.
    if(eo_ropcode_none != reply->head.ropc)
    {
        eo_former_GetStream(eo_former_GetHandle(), reply, pkt2tx_data, &pkt2tx_size, &ipaddr);
        ret = eobool_true;
    }

    return(ret);

}

static eObool_t s_smartnode_pktproc_from(eOipv4addr_t fromipaddr, uint8_t *pktdata, uint16_t pktsize)
{
    uint16_t tmp16;
    eOipv4addr_t ipaddr;

    eObool_t ret = eobool_false;

    eo_parser_GetROP(eo_parser_GetHandle(), pktdata, pktsize, fromipaddr, rop, &tmp16);

    eo_agent_InpROPprocess(eo_agent_GetHandle(), rop, reply);

    // if i have a packet to send back i send it.
    if(eo_ropcode_none != reply->head.ropc)
    {
        eo_former_GetStream(eo_former_GetHandle(), reply, pkt2tx_data, &pkt2tx_size, &ipaddr);
        ret = eobool_true;
    }

    return(ret);

}



static void s_simplenode_change_some_nvs(void)
{
    uint8_t tmp08 = 0;
    uint16_t tmp16 = 0;
    uint32_t tmp32 = 77777;
    uint64_t tmp64 = 9999999;

    EOnetvar *nv = NULL;

    nvInp08 = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDlocdev_input08__input08value, eo_nv_ownership_local, 0));
    nvInp16 = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDlocdev_input16__input16value, eo_nv_ownership_local, 0));

    tmp08 = 55;
    tmp16 = 666;

    // the peripheral change that, thus ... we force the change. also, it is likely that the change is done inside
    // the function update(), thus we MUST inhnibit the update.
    eo_netvar_Set(nvInp08, &tmp08, eobool_true, eo_nv_upd_dontdo);
    eo_netvar_Set(nvInp16, &tmp16, eobool_true, eo_nv_upd_dontdo);

    nv = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDlocdev_output__outputvalue, eo_nv_ownership_local, 0));
    eo_netvar_Set(nv, &tmp32, eobool_true, eo_nv_upd_dontdo);

    nv = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDlocdev_output__applicationtime, eo_nv_ownership_local, 0));
    eo_netvar_Set(nv, &tmp64, eobool_true, eo_nv_upd_dontdo);
}


static void s_smartnode_pktmake_ask_value(eOipv4addr_t toipaddr, eOnetvarID_t nvid, uint8_t *pktdata)
{
    eOipv4addr_t ipaddr;
    uint16_t tmp16 = 0;

    eo_agent_OutROPinit(eo_agent_GetHandle(), toipaddr,
                        eo_ropcode_ask, nvid, ropcfg, rop0);

    eo_agent_OutROPfill(eo_agent_GetHandle(), rop0, &sign, &time);

    // get the pkt data and send it to ipaddr 10.1.1.0
    eo_former_GetStream(eo_former_GetHandle(), rop0, pktdata, &tmp16, &ipaddr);

    // now the smart nod should send the pkt  to ipaddr
    // tx ....
}


static void s_simplenode_pktmake_sig_value(eOipv4addr_t toipaddr, eOnetvarID_t nvid, uint8_t *pktdata)
{
    eOipv4addr_t ipaddr;
    uint16_t tmp16 = 0;

    eo_agent_OutROPinit(eo_agent_GetHandle(), toipaddr,
                        eo_ropcode_sig, nvid, ropcfg, rop0);

    eo_agent_OutROPfill(eo_agent_GetHandle(), rop0, &sign, &time);

    // get the pkt data and send it to toipaddr
    eo_former_GetStream(eo_former_GetHandle(), rop0, pktdata, &tmp16, &ipaddr);

    // now the node should send the pkt 
    // tx ....
}

static void s_smartnode_asks(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr, eOnetvarID_t nvid)
{
    eObool_t reply_avail = eobool_false;
    // i am a smart node which wants to know the value of the input08 inside teh simple node

    s_smartnode_pktmake_ask_value(toipaddr, nvid, pkt2tx_data);



    // now i act as the normal node whcih receives the pkt from smart node

    reply_avail = s_simplenode_pktproc_from(fromipaddr, pkt2tx_data, sizeof(pkt2tx_data));
     
    // the smart node processes the reply.
    if(eobool_true == reply_avail)
    {
        // to do ...  
        s_smartnode_pktproc_from(toipaddr, pkt2tx_data, sizeof(pkt2tx_data)); 
    }
}



static void s_simplenode_sigs(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr, eOnetvarID_t nvid)
{
//    eObool_t reply_avail = eobool_false;
    // i am a simple node which sigs the ....

    s_simplenode_pktmake_sig_value(toipaddr, nvid, pkt2tx_data);

     
    // the smart node processes the packet.
    s_smartnode_pktproc_from(fromipaddr, pkt2tx_data, sizeof(pkt2tx_data)); 

}


static void s_smartnode_configure_vportRO(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr)
{

    eObool_t reply_avail;

    // now i am the smart node 10.10.1.100 and i configure the remote vportROnly.cfg of simple node 0 to contain 
    // some nv-ids which i want to know: for instance the input08 and the input16.

    s_smartnode_pktmake_set_vportRO(toipaddr, pkt2tx_data);

    // now i act as the simple node: i parse the packet received from the smart node, process the rop and 
    // put back in pkt2tx_data a possible reply.

    reply_avail = s_simplenode_pktproc_from(fromipaddr, pkt2tx_data, sizeof(pkt2tx_data));
     
    // the smart node processes a packet from .
    if(eobool_true == reply_avail)
    {
        s_smartnode_pktproc_from(toipaddr, pkt2tx_data, sizeof(pkt2tx_data)); 
    }
}


static void s_smartnode_configure_its_vportWOof(eOipv4addr_t ipaddr)
{
    uint32_t uu32;
    uint8_t bbb[64] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21}; 
    // use the vportcfg and puts some NV IDs inside (ony writeable)

    eo_vportcfg_Clear(vportcfg);
    // fill inside a local vportcfg all the NVs i want to be able to write in the remote node
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev_output__outputvalue); 
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev_output__applicationtime); 
    eo_vportcfg_PushBack(vportcfg, nvIDrem00dev__fixedarray);

    nvTMP = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDrem00dev_vportWOnly__cfg, eo_nv_ownership_remote, ipaddr));
    eo_netvar_Set(nvTMP, vportcfg, eobool_true, eo_nv_upd_always);

    
    // pass the vportcfg to the remote copy of vportWO relative to ipaddr
//    eo_vport_hid_LoadCfg(&rem00_device_vol.vportWOnly, vportcfg, eo_nv_ownership_remote, ipaddr);


    // at this point any write into the NVs shall be written inside the vport.
    uu32 = 0x12345678;
    nvTMP = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDrem00dev_output__outputvalue, eo_nv_ownership_remote, ipaddr));
    eo_netvar_Set(nvTMP, &uu32, eobool_true, eo_nv_upd_dontdo); // i force the write

    nvTMP = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvIDrem00dev__fixedarray, eo_nv_ownership_remote, ipaddr));
    eo_netvar_Set(nvTMP, bbb, eobool_true, eo_nv_upd_dontdo); // i force the write
     
}


static void s_smartnode_sets(eOipv4addr_t fromipaddr, eOipv4addr_t toipaddr, eOnetvarID_t nvid)
{
    eObool_t reply_avail = eobool_false;
    // i am a smart node which wants to know the value of the input08 inside teh simple node

    s_smartnode_pktmake_set_value(toipaddr, nvid, pkt2tx_data);



    // now i act as the normal node whcih receives the pkt from smart node

    reply_avail = s_simplenode_pktproc_from(fromipaddr, pkt2tx_data, sizeof(pkt2tx_data));
     
    // the smart node processes the reply.
    if(eobool_true == reply_avail)
    {
        // to do ...  
        s_smartnode_pktproc_from(toipaddr, pkt2tx_data, sizeof(pkt2tx_data)); 
    }
}

static void s_smartnode_pktmake_set_value(eOipv4addr_t toipaddr, eOnetvarID_t nvid, uint8_t *pktdata)
{
    eOipv4addr_t ipaddr;
    uint16_t tmp16 = 0;

    eo_agent_OutROPinit(eo_agent_GetHandle(), toipaddr,
                        eo_ropcode_set, nvid, ropcfg, rop0);

    eo_agent_OutROPfill(eo_agent_GetHandle(), rop0, &sign, &time);

    // get the pkt data and send it to ipaddr 10.1.1.0
    eo_former_GetStream(eo_former_GetHandle(), rop0, pktdata, &tmp16, &ipaddr);

    // now the smart nod should send the pkt  to ipaddr
    // tx ....
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



