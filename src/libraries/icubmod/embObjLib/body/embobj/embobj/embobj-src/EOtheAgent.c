

/* @file       EOtheAgent.c
    @brief      This file implements internal implementation of the parser singleton.
    @author     marco.accame@iit.it
    @date       09/03/2010
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "stdio.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"
#include "EOnv_hid.h"
#include "EOrop_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheAgent.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheAgent_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_agent_on_rop_confirmation_received(eOipv4addr_t ipaddr, eOropcode_t ropc, eOnvID_t nvid, 
                                                    uint32_t sign, eOabstime_t time, eOropconfinfo_t confinfo);


static void s_eo_agent_on_rop_confirmation_requested(eOipv4addr_t ipaddr, eOropcode_t ropc, eOnvID_t nvid,
                                                     uint32_t sign, eOabstime_t time);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtheAgent";
 
static EOtheAgent eo_theagent = 
{
    EO_INIT(.initted)           0,
    EO_INIT(.cfg)               NULL
};


extern const eOagent_cfg_t eo_agent_cfg_default =
{
    EO_INIT(.on_rop_conf_received)      NULL, 
    EO_INIT(.on_rop_conf_requested)     NULL
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOtheAgent * eo_agent_Initialise(const eOagent_cfg_t *cfg) 
{
    
    if(1 == eo_theagent.initted)
    {
        return(&eo_theagent);
    }


    if(NULL == cfg)
    {
        cfg = &eo_agent_cfg_default;
    }

    eo_theagent.cfg     = cfg;
    eo_theagent.initted = 1;
    
    return(&eo_theagent);        
}    


extern EOtheAgent * eo_agent_GetHandle(void) 
{
    return( (1 == eo_theagent.initted) ? (&eo_theagent) : (eo_agent_Initialise(NULL)) );
}


extern eOresult_t eo_agent_InpROPprocess(EOtheAgent *p, EOnvsCfg* nvscfg, EOrop *r, EOrop *replyrop, eOipv4addr_t fromipaddr)
{
    uint8_t ropc = eo_ropcode_none;
    eOresult_t res;

    if((NULL == p) || (NULL == nvscfg) || (NULL == r) || (NULL == replyrop))
    {
        return(eores_NOK_nullpointer);
    }


    // reset a possible replyrop. responsibility of full preparation is on other functions: eo_rop_Process() and called ones
    eo_rop_Reset(replyrop);

    // init the ropc of the input rop
    ropc = r->head.ropc;

    r->aboutip.ipaddr = fromipaddr;

    // normal commands
    // a simple node which only knows about its own netvars must use eo_nv_ownership_local
    // when receives ask<>, set<>, rst<>, upd<>.
    // a smart node who receives say<> and sig<> must search into eo_nv_ownership_remote.
    
    // ack/nak commands
    // the simple node just process: ack-nak-sig<>. 
    // the smart node can also process: nak-ask<>, ack-nak-set<>, ack-nak-rst<>, ack-nak-upd<>
    

    // can process only valid commands
    if( (eo_ropcode_none == ropc) || (eo_ropcode_usr == ropc) )
    {
        return(eores_NOK_generic); 
    }


    // if it is a confirmation, then ... call the confirmation engine
    if(eo_ropconf_none != r->head.ctrl.confinfo)
    {
        // received a confirmation ack/nak: execute the callback
        if(NULL != eo_theagent.cfg->on_rop_conf_received)
        {
            eo_theagent.cfg->on_rop_conf_received(r->aboutip.ipaddr, ropc, r->head.nvid, r->sign, r->time, (eOropconfinfo_t)r->head.ctrl.confinfo);
        }

        return(eores_OK); 
    }
    else
    {
        // otherwise we have a normal rop to be processed.
        
        r->aboutnvs.nvscfg = nvscfg;
        r->aboutnvs.nvownership = eo_rop_hid_GetOwnership(ropc, eo_ropconf_none, eo_rop_dir_received);
 
        // retrieve the indices inside the nvscfg given the triple (ip, ep, id) 
        res = eo_nvscfg_GetIndices( r->aboutnvs.nvscfg,  
                                    (eo_nv_ownership_local == r->aboutnvs.nvownership) ? (eok_ipv4addr_localhost) : (r->aboutip.ipaddr),
                                    r->head.endp, r->head.nvid, 
                                    &r->aboutnvs.ondevindex,
                                    &r->aboutnvs.onendpointindex,
                                    &r->aboutnvs.onidindex); 

        // if the nvscfg does not have the triple (ip, ep, id) then we return an error
        if(eores_OK != res)
        {   
            r->aboutnvs.nvtreenoderoot  = NULL;
            eo_nv_Clear(&r->aboutnvs.nvroot);
        }
        else
        {
            // we need a treenode of the nv
            r->aboutnvs.nvtreenoderoot = eo_nvscfg_GetTreeNode(r->aboutnvs.nvscfg, r->aboutnvs.ondevindex, r->aboutnvs.onendpointindex, r->aboutnvs.onidindex);
            // but also the handle to the nv.
            eo_nvscfg_GetNV(r->aboutnvs.nvscfg, r->aboutnvs.ondevindex, r->aboutnvs.onendpointindex, r->aboutnvs.onidindex, r->aboutnvs.nvtreenoderoot, &r->aboutnvs.nvroot);
        }

 
        // process the rop on that nvtreenoderoot even if it is NULL ... because we may need to send back a nack.
 
        eo_rop_Process(r, replyrop);

        return(eores_OK);
    }

}


extern eOresult_t eo_agent_OutROPinit(EOtheAgent *p, EOnvsCfg* nvscfg, eOipv4addr_t toipaddr, eOipv4port_t toport, eOropcode_t ropc, eOnvEP_t endpoint, eOnvID_t nvid, eOropconfig_t ropcfg, EOrop *rop, uint16_t *requiredbytes)
{
#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
    eOnvType_t typ = eo_nv_TYP_NO4;
    eOnvFunc_t fun = eo_nv_FUN_NO0;
#endif
    eOrophead_t rophead;
    eOresult_t res;

    if((NULL == p) || (NULL == rop))
    {
        return(eores_NOK_nullpointer);
    } 

    // reset the rop
    eo_rop_Reset(rop);  
    
    // put in rophead all the options
    rophead.ctrl.ffu        = 0;
    rophead.ctrl.confinfo   = eo_ropconf_none; // cannot do a ack/ack
    rophead.ctrl.plustime   = (eobool_true == ropcfg.plustime) ? (1) : (0);
    rophead.ctrl.plussign   = (eobool_true == ropcfg.plussign) ? (1) : (0);
    rophead.ctrl.rqsttime   = (eobool_true == ropcfg.timerqst) ? (1) : (0);
    rophead.ctrl.rqstconf   = (eobool_true == ropcfg.confrqst) ? (1) : (0);
    rophead.ctrl.userdefn   = 0;
    rophead.ropc            = ropc;
    rophead.endp            = endpoint;
    rophead.nvid            = nvid;
    rophead.dsiz            = 0;


    // check validity of ropc 
    if((ropc >= (uint8_t)eo_ropcodevalues_numberofthem) || (ropc == (uint8_t)eo_ropcode_none))
    {
        return(eores_NOK_generic);
    }
    else if((ropc == (uint8_t)eo_ropcode_usr))
    {
        // not managed yet
        return(eores_NOK_generic);
    }

#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
    // check validity of nvid
    fun = eo_nv_hid_fromIDtoFUN(nvid);
    typ = eo_nv_hid_fromIDtoTYP(nvid);

    if((eo_nv_FUN_NO0 == fun) || (eo_nv_FUN_NO1 == fun) || (eo_nv_TYP_NO4 == typ) ||(eo_nv_TYP_NO5 == typ))
    {
        // incorrect nvid
        return(eores_NOK_generic);
    }
#endif

    rop->aboutip.ipaddr = toipaddr;

    // get the ownership    
    rop->aboutnvs.nvscfg = nvscfg;
    rop->aboutnvs.nvownership = eo_rop_hid_GetOwnership(ropc, eo_ropconf_none, eo_rop_dir_outgoing);

    // retrieve the indices inside the nvscfg given the triple (ip, ep, id)
     res = eo_nvscfg_GetIndices( rop->aboutnvs.nvscfg,  
                                (eo_nv_ownership_local == rop->aboutnvs.nvownership) ? (eok_ipv4addr_localhost) : (toipaddr), 
                                endpoint, nvid, 
                                &rop->aboutnvs.ondevindex,
                                &rop->aboutnvs.onendpointindex,
                                &rop->aboutnvs.onidindex); 

    // if the nvscfg does not have the triple (ip, ep, id) then we return an error because we cannot form the rop
    if(eores_OK != res)
    {
        return(eores_NOK_generic);
    }

    // we need a treenode of the nv
    rop->aboutnvs.nvtreenoderoot = eo_nvscfg_GetTreeNode(rop->aboutnvs.nvscfg, rop->aboutnvs.ondevindex, rop->aboutnvs.onendpointindex, rop->aboutnvs.onidindex);

   
    // --- ok: now we are ready to prepare the rop
    
    // head
    memcpy(&rop->head, &rophead, sizeof(eOrophead_t));
    
    // data
    if(eobool_true == eo_rop_hid_DataField_is_Required(&rophead))
    {
        // we also need the nv
        uint16_t sss;
        eo_nvscfg_GetNV(rop->aboutnvs.nvscfg, rop->aboutnvs.ondevindex, rop->aboutnvs.onendpointindex, rop->aboutnvs.onidindex, rop->aboutnvs.nvtreenoderoot, &rop->aboutnvs.nvroot);
        
        eo_nv_Get(&rop->aboutnvs.nvroot, eo_nv_strg_volatile, rop->data, &sss);
        rop->head.dsiz = sss;
    }
    
    // reset sign
    rop->sign = EOK_uint32dummy;

    // reset time
    rop->time = EOK_uint64dummy;

    if(NULL != requiredbytes)
    {
        *requiredbytes = sizeof(eOrophead_t) + rop->head.dsiz + (4*rop->head.ctrl.plussign) + (8*rop->head.ctrl.plustime);
    }


    return(eores_OK);
}



extern eOresult_t eo_agent_OutROPfill(EOtheAgent *p, EOrop *rop, uint32_t *sign, eOabstime_t *time)
{

    if((NULL == p) || (NULL == rop))
    {
        return(eores_NOK_nullpointer);
    } 

    // if not found, then error because must have been searched before
    if(NULL == rop->aboutnvs.nvtreenoderoot)
    {
        return(eores_NOK_generic);
    }

    
    // ok, now we are ready to update the rop
 
    // datainfo
    if(eobool_true == eo_rop_hid_DataField_is_Present(&rop->head))
    {   // update also rop->datasize
        uint16_t sss;
        eo_nv_Get(&rop->aboutnvs.nvroot, eo_nv_strg_volatile, rop->data, &sss);
        rop->head.dsiz = sss;
    }
    
    // sign
    if(1 == rop->head.ctrl.plussign)
    {
        rop->sign = (NULL != sign) ? (*sign) : (EOK_uint32dummy);
    }

    // time
    if(1 == rop->head.ctrl.plustime)
    {
        rop->time = (NULL != time) ? (*time) : (EOK_uint64dummy);
    }


    return(eores_OK);

}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern eOresult_t eo_agent_hid_OutROPonTransmission(EOtheAgent *p, EOrop *rop)
{
    if(1 == rop->head.ctrl.rqstconf)
    {
        if(NULL !=  eo_theagent.cfg->on_rop_conf_requested)
        {
            eo_theagent.cfg->on_rop_conf_requested(rop->aboutip.ipaddr, rop->head.ropc, rop->head.nvid, rop->sign, rop->time);
        }
        return(eores_OK);
    }

    return(eores_NOK_generic);
   
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eo_agent_on_rop_confirmation_received(eOipv4addr_t ipaddr, eOropcode_t ropc, eOnvID_t nvid, 
                                                    uint32_t sign, eOabstime_t time, eOropconfinfo_t confinfo)
{
    char str[64];
    snprintf(str, sizeof(str)-1, "rx %s for opc %d and nvid 0x%xd from ip %d", (eo_ropconf_ack == confinfo) ? ("ack") : ("nak"), ropc, nvid, ipaddr);
    eo_errman_Error(eo_errman_GetHandle(), eo_errortype_info, s_eobj_ownname, str);    
}


static void s_eo_agent_on_rop_confirmation_requested(eOipv4addr_t ipaddr, eOropcode_t ropc, eOnvID_t nvid,
                                                     uint32_t sign, eOabstime_t time)
{
    char str[64];
    snprintf(str, sizeof(str)-1, "rqst conf for opc %d and nvid 0x%x from ip %d", ropc, nvid, ipaddr);
    eo_errman_Error(eo_errman_GetHandle(), eo_errortype_info, s_eobj_ownname, str);        
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




