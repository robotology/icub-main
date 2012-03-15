

/* @file       EOhostTransceiver.c
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

#include "EOhostTransceiver.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOhostTransceiver_hid.h" 


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
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static EOnvsCfg* s_eo_hosttransceiver_nvscfg_get(const eOhosttransceiver_cfg_t *cfg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOhostTransceiver";
 

extern const eOhosttransceiver_cfg_t eo_hosttransceiver_cfg_default = 
{
    EO_INIT(.vectorof_endpoint_cfg)     NULL,
    EO_INIT(.remoteboardipv4addr)       0,
    EO_INIT(.remoteboardipv4port)       0,
    EO_INIT(.tobedefined)               0
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOhostTransceiver * eo_hosttransceiver_New(const eOhosttransceiver_cfg_t *cfg) 
{
    EOhostTransceiver* retptr = NULL;
    eo_transceiver_cfg_t txrxcfg = eo_transceiver_cfg_default;


    if(NULL == cfg)
    {
        cfg = &eo_hosttransceiver_cfg_default;
    }

    if(NULL == cfg->vectorof_endpoint_cfg)
    {
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "need a vector of endpoints");
    }
    
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOhostTransceiver), 1);

    // 1. init the proper transceiver cfg

    retptr->nvscfg = s_eo_hosttransceiver_nvscfg_get(cfg);
    

    txrxcfg.capacityofpacket               = EOK_HOSTTRANSCEIVER_capacityofpacket;
    txrxcfg.capacityofrop                  = EOK_HOSTTRANSCEIVER_capacityofrop;
    txrxcfg.capacityofropframeregulars     = EOK_HOSTTRANSCEIVER_capacityofropframeregulars;
    txrxcfg.capacityofropframeoccasionals  = EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals;
    txrxcfg.capacityofropframereplies      = EOK_HOSTTRANSCEIVER_capacityofropframereplies;
    txrxcfg.maxnumberofregularrops         = EOK_HOSTTRANSCEIVER_maxnumberofregularrops;
    txrxcfg.remipv4addr                    = cfg->remoteboardipv4addr;
    txrxcfg.remipv4port                    = cfg->remoteboardipv4port;
    txrxcfg.nvscfg                         = retptr->nvscfg;
    
    retptr->transceiver = eo_transceiver_New(&txrxcfg);
    
    
    return(retptr);        
}    


extern EOtransceiver* eo_hosttransceiver_Transceiver(EOhostTransceiver *p)
{
    if(NULL == p)
    {
        return(NULL);
    }
    
    return(p->transceiver);
}

extern EOnvsCfg* eo_hosttransceiver_NVsCfg(EOhostTransceiver *p)
{
    if(NULL == p)
    {
        return(NULL);
    }
    
    return(p->nvscfg);
}





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static EOnvsCfg* s_eo_hosttransceiver_nvscfg_get(const eOhosttransceiver_cfg_t *cfg)
{
    EOnvsCfg* nvscfg;
    eOnvscfg_EP_t *epcfg;
    const EOconstvector* theepcfgs = NULL;
    uint16_t nendpoints = 0;
    uint16_t i;

    theepcfgs = cfg->vectorof_endpoint_cfg;

    // the HOSTtransceiver does not use any storage.
    nvscfg = eo_nvscfg_New(1, NULL);

    nendpoints = eo_constvector_Size(theepcfgs);
    
    // use local-host address as we dont need to put the actual address of board in nvscfg.
    eo_nvscfg_PushBackDevice(nvscfg, eo_nvscfg_ownership_remote, cfg->remoteboardipv4addr, cfg->hashfunction_ep2index, nendpoints);
    
    for(i=0; i<nendpoints; i++)
    {
        epcfg = (eOnvscfg_EP_t*) eo_constvector_At(theepcfgs, i);

        eo_nvscfg_ondevice_PushBackEndpoint(nvscfg, 0, epcfg->endpoint,
                                        epcfg->hashfunction_id2index,
                                        epcfg->constvector_of_treenodes_EOnv_con,
                                        epcfg->constvector_of_EOnv_usr,
                                        epcfg->sizeof_endpoint_data, 
                                        epcfg->endpoint_data_init,
                                        NULL);
    }
    
    eo_nvscfg_data_Initialise(nvscfg);

    return(nvscfg);
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




