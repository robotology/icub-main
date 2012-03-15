

/* @file       EOtheBOARDtransceiver.c
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

#include "EOtheBOARDtransceiver.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheBOARDtransceiver_hid.h" 


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

static EOnvsCfg* s_eo_boardtransceiver_nvscfg_get(const eOboardtransceiver_cfg_t *cfg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtheBOARDtransceiver";
 
static EOtheBOARDtransceiver s_eo_theboardtrans = 
{
    .transceiver        = NULL,
    .nvscfg             = NULL
};


extern const eOboardtransceiver_cfg_t eo_boardtransceiver_cfg_default = 
{
    .vectorof_endpoint_cfg      = NULL,
    .remotehostipv4addr         = 0,
    .remotehostipv4port         = 0,
    .tobedefined = 0
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOtransceiver * eo_boardtransceiver_Initialise(const eOboardtransceiver_cfg_t *cfg) 
{
    eo_transceiver_cfg_t txrxcfg = eo_transceiver_cfg_default;
    
    if(NULL != s_eo_theboardtrans.transceiver)
    {
        return(s_eo_theboardtrans.transceiver);
    }


    if(NULL == cfg)
    {
        cfg = &eo_boardtransceiver_cfg_default;
    }

    if(NULL == cfg->vectorof_endpoint_cfg)
    {
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "need a vector of endpoints");
    }

    // 1. init the proper transceiver cfg

    s_eo_theboardtrans.nvscfg = s_eo_boardtransceiver_nvscfg_get(cfg);
    

    txrxcfg.capacityofpacket               = EOK_BOARDTRANSCEIVER_capacityofpacket;
    txrxcfg.capacityofrop                  = EOK_BOARDTRANSCEIVER_capacityofrop;
    txrxcfg.capacityofropframeregulars     = EOK_BOARDTRANSCEIVER_capacityofropframeregulars;
    txrxcfg.capacityofropframeoccasionals  = EOK_BOARDTRANSCEIVER_capacityofropframeoccasionals;
    txrxcfg.capacityofropframereplies      = EOK_BOARDTRANSCEIVER_capacityofropframereplies;
    txrxcfg.maxnumberofregularrops         = EOK_BOARDTRANSCEIVER_maxnumberofregularrops;
    txrxcfg.remipv4addr                    = cfg->remotehostipv4addr;
    txrxcfg.remipv4port                    = cfg->remotehostipv4port;
    txrxcfg.nvscfg                         = s_eo_theboardtrans.nvscfg;
    
    s_eo_theboardtrans.transceiver = eo_transceiver_New(&txrxcfg);
    
    
    return(s_eo_theboardtrans.transceiver);        
}    


extern EOtransceiver * eo_boardtransceiver_GetHandle(void) 
{
    return(s_eo_theboardtrans.transceiver);
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static EOnvsCfg* s_eo_boardtransceiver_nvscfg_get(const eOboardtransceiver_cfg_t *cfg)
{
    EOnvsCfg* nvscfg = s_eo_theboardtrans.nvscfg;
    eOnvscfg_EP_t *epcfg;
    const EOconstvector* theepcfgs = NULL;
    uint16_t nendpoints = 0;
    uint16_t i;

    if(NULL != nvscfg)
    {
        return(NULL);
    }

    theepcfgs = cfg->vectorof_endpoint_cfg;

    #warning --> so far the BOARDtransceiver does not use any storage. if needed ... change the NULL into a ...
    nvscfg = eo_nvscfg_New(1, NULL);

    nendpoints = eo_constvector_Size(theepcfgs);
    
    // use local-host address as we dont need to put the actual address of board in nvscfg.
    eo_nvscfg_PushBackDevice(nvscfg, eo_nvscfg_ownership_local, EO_COMMON_IPV4ADDR_LOCALHOST, cfg->hashfunction_ep2index, nendpoints);
    
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




