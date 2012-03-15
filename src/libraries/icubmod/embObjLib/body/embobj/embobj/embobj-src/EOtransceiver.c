
/* @file       EOtransceiver.c
    @brief      This file implements internal implementation of a datagram socket object.
    @author     marco.accame@iit.it
    @date       12/24/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheParser.h"
#include "EOtheFormer.h"
#include "EOropframe_hid.h"
#include "EOnv_hid.h"
#include "EOrop_hid.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtransceiver.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtransceiver_hid.h" 


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
// empty-section    


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOtransceiver";

extern const eo_transceiver_cfg_t eo_transceiver_cfg_default = 
{
    EO_INIT(.capacityofpacket)              512, 
    EO_INIT(.capacityofrop)                 128, 
    EO_INIT(.capacityofropframeregulars)    256,
    EO_INIT(.capacityofropframeoccasionals) 128,
    EO_INIT(.capacityofropframereplies)     128, 
    EO_INIT(.maxnumberofregularrops)        16,
    EO_INIT(.remipv4addr)                   EO_COMMON_IPV4ADDR_LOCALHOST,
    EO_INIT(.remipv4port)                   10001,
    EO_INIT(.nvscfg)                        NULL
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOtransceiver* eo_transceiver_New(const eo_transceiver_cfg_t *cfg)
{
    EOtransceiver *retptr = NULL;  
    eo_receiver_cfg_t rec_cfg;
    eo_transmitter_cfg_t tra_cfg;


    if(NULL == cfg)
    {    
        cfg = &eo_transceiver_cfg_default;
    }
    
    memcpy(&rec_cfg, &eo_receiver_cfg_default, sizeof(eo_receiver_cfg_t));
    rec_cfg.capacityofropframereply     = cfg->capacityofropframereplies;
    rec_cfg.capacityofropinput          = cfg->capacityofrop;
    rec_cfg.capacityofropreply          = cfg->capacityofrop;
    rec_cfg.nvscfg                      = cfg->nvscfg;

    
    memcpy(&tra_cfg, &eo_transmitter_cfg_default, sizeof(eo_transmitter_cfg_t));
    tra_cfg.capacityoftxpacket          = cfg->capacityofpacket;
    tra_cfg.capacityofropframepermanent = cfg->capacityofropframeregulars;
    tra_cfg.capacityofropframetemporary = cfg->capacityofropframeoccasionals;
    tra_cfg.capacityofropframereplies   = cfg->capacityofropframereplies;
    tra_cfg.capacityofrop               = cfg->capacityofrop;
    tra_cfg.maxnumberofpermanentrops    = cfg->maxnumberofregularrops;
    tra_cfg.ipv4addr                    = cfg->remipv4addr;
    tra_cfg.ipv4port                    = cfg->remipv4port; 
    tra_cfg.nvscfg                      = cfg->nvscfg;
    
    
    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOtransceiver), 1);
    
    
    memcpy(&retptr->cfg, cfg, sizeof(eo_transceiver_cfg_t)); 
    
    retptr->receiver = eo_receiver_New(&rec_cfg);
    
    retptr->transmitter = eo_transmitter_New(&tra_cfg);
    
    return(retptr);
}


extern eOresult_t eo_transceiver_Receive(EOtransceiver *p, EOpacket *pkt, uint16_t *numberofrops, eOabstime_t* txtime)
{
    eObool_t thereisareply = eobool_false;  
    eOresult_t res;
    
    if((NULL == p) || (NULL == pkt))
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eores_OK != (res = eo_receiver_Process(p->receiver, pkt, p->cfg.nvscfg, numberofrops, &thereisareply, txtime)))
    {
        return(res);
    }  

    if(eobool_true == thereisareply)
    {
        // must put the reply inside the transmitter
        EOropframe* ropframereply = NULL;
        eOipv4addr_t toipv4addr;
        eOipv4port_t toipv4port;
        
        // if in here, i am sure that there is a reply and that return value will be eores_OK
        res = eo_receiver_GetReply(p->receiver, &ropframereply, &toipv4addr, &toipv4port);
        // however, it may be that we must reply to a pair addr-port which is different from what we expect.
        if((p->cfg.remipv4addr == toipv4addr) && (p->cfg.remipv4port == toipv4port))
        {
            res = eo_transmitter_ropframereplies_Append(p->transmitter, ropframereply);        
        }
    }    
    
    return(res);
}



extern eOresult_t eo_transceiver_Transmit(EOtransceiver *p, EOpacket **pkt, uint16_t *numberofrops)
{
    eOresult_t res;
    
    if((NULL == p) || (NULL == pkt))
    {
        return(eores_NOK_nullpointer);
    }
    
    // refresh regulars ...    
    eo_transmitter_permanentrops_Refresh(p->transmitter);
    
    // finally retrieve the packet from teh transmitter. it will be formed by replies, regulars, occasionals.
    res = eo_transmitter_outpacket_Get(p->transmitter, pkt, numberofrops);
    
    
    return(res);    
}


extern eOresult_t eo_transceiver_rop_regular_Clear(EOtransceiver *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eo_transmitter_permanentrops_Clear(p->transmitter));
}

extern eOresult_t eo_transceiver_rop_regular_Load(EOtransceiver *p, eo_transceiver_ropinfo_t *ropinfo)
{
    if((NULL == p) || (NULL == ropinfo))
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eo_transmitter_permanentrops_Load(p->transmitter, ropinfo->ropcode, ropinfo->nvep, ropinfo->nvid, ropinfo->ropcfg));
}

extern eOresult_t eo_transceiver_rop_regular_Unload(EOtransceiver *p, eo_transceiver_ropinfo_t *ropinfo)
{
    if((NULL == p) || (NULL == ropinfo))
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eo_transmitter_permanentrops_Unload(p->transmitter, ropinfo->ropcode, ropinfo->nvid));
}

extern eOresult_t eo_transceiver_rop_occasional_Load(EOtransceiver *p, eo_transceiver_ropinfo_t *ropinfo)
{
    if((NULL == p) || (NULL == ropinfo))
    {
        return(eores_NOK_nullpointer);
    }

    return(eo_transmitter_temporaryrop_Load(p->transmitter, ropinfo->ropcode, ropinfo->nvep, ropinfo->nvid, ropinfo->ropcfg));
}    


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




