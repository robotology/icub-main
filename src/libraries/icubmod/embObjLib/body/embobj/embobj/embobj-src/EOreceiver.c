
/* @file       EOreceiver.c
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





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOreceiver.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOreceiver_hid.h" 


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

//static const char s_eobj_ownname[] = "EOreceiver";

extern const eo_receiver_cfg_t eo_receiver_cfg_default = 
{
    EO_INIT(.capacityofropframereply)   256, 
    EO_INIT(.capacityofropinput)        128, 
    EO_INIT(.capacityofropreply)        128, 
    EO_INIT(.nvscfg)                    NULL
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOreceiver* eo_receiver_New(const eo_receiver_cfg_t *cfg)
{
    EOreceiver *retptr = NULL;   

    if(NULL == cfg)
    {    
        cfg = &eo_receiver_cfg_default;
    }
    
    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOreceiver), 1);
    
    retptr->ropframeinput       = eo_ropframe_New();
    retptr->ropframereply       = eo_ropframe_New();
    retptr->ropinput            = eo_rop_New(cfg->capacityofropinput);
    retptr->ropreply            = eo_rop_New(cfg->capacityofropreply);
    retptr->nvscfg              = cfg->nvscfg;
    retptr->theagent            = eo_agent_Initialise(NULL);
    retptr->ipv4addr            = 0;
    retptr->ipv4port            = 0;
    retptr->bufferropframereply = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, cfg->capacityofropframereply, 1);
 
    // now we need to allocate the buffer for the ropframereply
    
    eo_ropframe_Load(retptr->ropframereply, retptr->bufferropframereply, 0, cfg->capacityofropframereply);
    
    return(retptr);
}


extern eOresult_t eo_receiver_Process(EOreceiver *p, EOpacket *packet, EOnvsCfg *nvscfg, uint16_t *numberofrops, eObool_t *thereisareply, eOabstime_t *transmittedtime)
{
    uint16_t rxremainingbytes = 0;
    uint16_t txremainingbytes = 0;
    uint8_t* payload;
    uint16_t size;
    uint16_t capacity;
    uint16_t nrops;
    uint16_t i;
    eOresult_t res;
    EOnvsCfg *nvs2use = NULL;

    
    if((NULL == p) || (NULL == packet)) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(NULL != nvscfg)
    {
        nvs2use = nvscfg;
    }
    else
    {
        nvs2use = p->nvscfg;
    }

    if(NULL == nvs2use)
    {
        return(eores_NOK_nullpointer);
    }
    
    // clear the ropframereply w/ eo_ropframe_Clear(). the clear operation also makes it safe to manipulate p->ropframereplay with *_quickversion
    
    eo_ropframe_Clear(p->ropframereply);
    
    // pass the packet to the ropframeinput w/ eo_ropframe_Load() 
    
    eo_packet_Addressing_Get(packet, &p->ipv4addr, &p->ipv4port);
    eo_packet_Payload_Get(packet, &payload, &size);
    eo_packet_Capacity_Get(packet, &capacity);
    
    eo_ropframe_Load(p->ropframeinput, payload, size, capacity);
    
    // verify if the ropframeinput is valid w/ eo_ropframe_IsValid()
    if(eobool_false == eo_ropframe_IsValid(p->ropframeinput))
    {
        if(NULL != thereisareply)
        {
            *thereisareply = eobool_false;
        }
        return(eores_NOK_generic);
    }
    
    // for every rop inside with eo_ropframe_ROP_NumberOf() :
    //nrops = eo_ropframe_ROP_NumberOf(p->ropframeinput);
    // i have already verified validity of p->ropframeinput
    nrops = eo_ropframe_ROP_NumberOf_quickversion(p->ropframeinput);
    
    for(i=0; i<nrops; i++)
    {
        // - get teh rop w/ eo_ropframe_ROP_Get()
        
        res = eo_ropframe_ROP_Get(p->ropframeinput, p->ropinput, &rxremainingbytes);
        
        if(eores_OK != res)
        {
            break;
        }        
        
        // - use teh agent w/ eo_agent_InpROPprocess() and retrieve the ropreply
        
        eo_agent_InpROPprocess(p->theagent, nvs2use, p->ropinput, p->ropreply, p->ipv4addr);
        
        // - if ropreply is ok w/ eo_rop_GetROPcode() then add it to ropframereply w/ eo_ropframe_ROP_Set()
        
        if(eo_ropcode_none != eo_rop_GetROPcode(p->ropreply))
        {
            res = eo_ropframe_ROP_Set(p->ropframereply, p->ropreply, NULL, NULL, &txremainingbytes);
        }
        
        // we keep on decoding eve if we cannot put a reply into teh ropframe 
        //if(0 == rxremainingbytes)
        //{
        //    break;
        //}
        
    }

    
    if(NULL != numberofrops)
    {
        *numberofrops = nrops;
    }

    // if any rop inside ropframereply w/ eo_ropframe_ROP_NumberOf() then sets thereisareply  
    if( NULL != thereisareply )
    {
        //*thereisareply = (0 == eo_ropframe_ROP_NumberOf(p->ropframereply)) ? (eobool_false) : (eobool_true);
        *thereisareply = (0 == eo_ropframe_ROP_NumberOf_quickversion(p->ropframereply)) ? (eobool_false) : (eobool_true);
    } 
    
    if(NULL != transmittedtime)
    {
        *transmittedtime = eo_ropframe_age_Get(p->ropframeinput);
    }   
    
    return(eores_OK);   
}


extern eOresult_t eo_receiver_GetReply(EOreceiver *p, EOropframe **ropframereply, eOipv4addr_t *ipv4addr, eOipv4port_t *ipv4port)
{
    if((NULL == p) || (NULL == ropframereply) || (NULL == ipv4addr) || (NULL == ipv4port)) 
    {
        return(eores_NOK_nullpointer);
    }
    
//    if(0 == eo_ropframe_ROP_NumberOf(p->ropframereply))
    if(0 == eo_ropframe_ROP_NumberOf_quickversion(p->ropframereply))
    {
        *ropframereply  = p->ropframereply;
        *ipv4addr       = p->ipv4addr;
        *ipv4port       = p->ipv4port;
        return(eores_NOK_generic);
    }
    
    *ropframereply  = p->ropframereply;
    *ipv4addr       = p->ipv4addr;
    *ipv4port       = p->ipv4port;
    
    return(eores_OK);
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




