
/* @file       EOtransmitter.c
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
#include "EOVtheSystem.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtransmitter.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtransmitter_hid.h" 


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

static eOresult_t s_eo_transmitter_listmatching_rule(void *item, void *param);

static void s_eo_transmitter_list_updaterop_in_ropframe(void *item, void *param);

static void s_eo_transmitter_list_shiftdownropinfo(void *item, void *param);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOtransmitter";

extern const eo_transmitter_cfg_t eo_transmitter_cfg_default = 
{
    EO_INIT(.capacityoftxpacket)            512, 
    EO_INIT(.capacityofropframepermanent)   256, 
    EO_INIT(.capacityofropframetemporary)   256,
    EO_INIT(.capacityofropframereplies)     256,
    EO_INIT(.capacityofrop)                 128, 
    EO_INIT(.maxnumberofpermanentrops)      16,
    EO_INIT(.nvscfg)                        NULL,
    EO_INIT(.ipv4addr)                      EO_COMMON_IPV4ADDR_LOCALHOST,
    EO_INIT(.ipv4port)                      10001
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOtransmitter* eo_transmitter_New(const eo_transmitter_cfg_t *cfg)
{
    EOtransmitter *retptr = NULL;   

    if(NULL == cfg)
    {    
        cfg = &eo_transmitter_cfg_default;
    }
    
    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOtransmitter), 1);
    
    retptr->txpacket                = eo_packet_New(cfg->capacityoftxpacket);
    retptr->ropframereadytotx       = eo_ropframe_New();
    retptr->ropframepermanent       = eo_ropframe_New();
    retptr->ropframetemporary       = eo_ropframe_New();
    retptr->ropframereplies         = eo_ropframe_New();
    retptr->roptmp                  = eo_rop_New(cfg->capacityofrop);
    retptr->nvscfg                  = cfg->nvscfg;
    retptr->theagent                = eo_agent_Initialise(NULL);
    retptr->ipv4addr                = cfg->ipv4addr;
    retptr->ipv4port                = cfg->ipv4port;
    retptr->bufferropframepermanent = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, cfg->capacityofropframepermanent, 1);
    retptr->bufferropframetemporary = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, cfg->capacityofropframetemporary, 1);
    retptr->bufferropframereplies = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, cfg->capacityofropframereplies, 1);
    retptr->listofpermropsinfo      = eo_list_New(sizeof(eo_transm_permrops_info_t), cfg->maxnumberofpermanentrops, NULL, 0, NULL, NULL);
    retptr->currenttime             = 0;

    eo_ropframe_Load(retptr->ropframepermanent, retptr->bufferropframepermanent, 0, cfg->capacityofropframepermanent);
    eo_ropframe_Clear(retptr->ropframepermanent);
    eo_ropframe_Load(retptr->ropframetemporary, retptr->bufferropframetemporary, 0, cfg->capacityofropframetemporary);
    eo_ropframe_Clear(retptr->ropframetemporary);
    eo_ropframe_Load(retptr->ropframereplies, retptr->bufferropframereplies, 0, cfg->capacityofropframereplies);
    eo_ropframe_Clear(retptr->ropframereplies);


    {   // to be put in constructor of EOtransmitter
        uint8_t *data;
        uint16_t size;
        uint16_t capacity;
        
        eo_packet_Payload_Get(retptr->txpacket, &data, &size);
        eo_packet_Capacity_Get(retptr->txpacket, &capacity);
    
        eo_ropframe_Load(retptr->ropframereadytotx, data, size, capacity);
        eo_ropframe_Clear(retptr->ropframereadytotx);

        eo_packet_Addressing_Set(retptr->txpacket, retptr->ipv4addr, retptr->ipv4port);
    }    
    
    return(retptr);
}






extern eOresult_t eo_transmitter_permanentrops_Load(EOtransmitter *p, eOropcode_t ropcode, eOnvEP_t nvep, eOnvID_t nvid, eOropconfig_t ropcfg)
{
    eo_transm_permrops_info_t permropinfo;
    eOresult_t res;
    uint16_t usedbytes;
    uint16_t remainingbytes;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }  

    // work on the list ... 
    
    if(eobool_true == eo_list_Full(p->listofpermropsinfo))
    {
        return(eores_NOK_generic);
    }
    
    permropinfo.ropcode = ropcode;
    permropinfo.nvid    = nvid;
    
    // search for ropcode+nvid if found, then ... return OK and dont do anything.
    if(NULL != eo_list_Find(p->listofpermropsinfo, s_eo_transmitter_listmatching_rule, &permropinfo))
    {   // it is already inside ...
        return(eores_NOK_generic);
    }
    
    // else ... prepare a temporary variable eo_transm_permrops_info_t to be put inside the list and wait success of rop + insetrtion in frame
    
    res = eo_agent_OutROPinit(p->theagent, p->nvscfg, 
                              p->ipv4addr, p->ipv4port, 
                              ropcode, nvep, nvid, ropcfg, 
                              p->roptmp, &usedbytes);
                              
    if(eores_OK != res)
    {
        return(res);
    }

    // put the rop inside the ropframe
    res = eo_ropframe_ROP_Set(p->ropframepermanent, p->roptmp, &permropinfo.ropstarthere, &permropinfo.ropsize, &remainingbytes);
    
    if(eores_OK != res)
    {
        return(res);
    }
    
    
    {
        permropinfo.nv = eo_rop_hid_NV_Get(p->roptmp);

        if(NULL != permropinfo.nv->loc)
        {
            // it has payload
            permropinfo.nvloc       = permropinfo.nv->loc;
            permropinfo.capacity    = permropinfo.nv->con->capacity;

        }
        else
        {
            permropinfo.nvloc       = NULL;
            permropinfo.capacity    = 0;
        }

        if(0 == p->roptmp->head.ctrl.plustime)
        {
            permropinfo.timeoffsetinsiderop = EOK_uint16dummy;
        }
        else
        {   // time is on the last 8 bytes
            permropinfo.timeoffsetinsiderop = permropinfo.ropsize - 8;
        }


    }
    
    // push back permropinfo inside the list.
    eo_list_PushBack(p->listofpermropsinfo, &permropinfo);
    
    
    return(eores_OK);   
}

extern eOresult_t eo_transmitter_permanentrops_Unload(EOtransmitter *p, eOropcode_t ropcode, eOnvID_t nvid)
{
    eo_transm_permrops_info_t permropinfo;
    EOlistIter *li = NULL;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }  

    // work on the list ... 
    
    if(eobool_true == eo_list_Empty(p->listofpermropsinfo))
    {
        return(eores_NOK_generic);
    }
    
    permropinfo.ropcode = ropcode;
    permropinfo.nvid    = nvid;
    
    // search for ropcode+nvid. if not found, then ... return OK and dont do anything.
    li = eo_list_Find(p->listofpermropsinfo, s_eo_transmitter_listmatching_rule, &permropinfo);
    if(NULL == li)
    {   // it is not inside ...
        return(eores_NOK_generic);
    }
    
    // copy what is insde the list in temporary variable
    memcpy(&permropinfo, eo_list_At(p->listofpermropsinfo, li), sizeof(eo_transm_permrops_info_t));
    
    // for each element after li: (name is afterli) retrieve it and modify its content so that ropstarthere is decremented by permropinfo.ropsize ...

    eo_list_FromIterForEach(p->listofpermropsinfo, eo_list_Next(p->listofpermropsinfo, li), s_eo_transmitter_list_shiftdownropinfo, &permropinfo);
    
    // remove the element indexedby li
    eo_list_Erase(p->listofpermropsinfo, li);
    
    // inside the p->ropframepermanent: remove a rop of permropinfo.ropsize whcih starts at permropinfo.ropstartshere. use a _friend method in here defined.
    //                                  you must: decrement the nrops by 1, decrement the size by permropinfo.ropsize, ... else in header and private variable ...
    //                                            and finally make a memmove down by permropinfo.ropsize.
    
    eo_ropframe_hid_rop_rem(p->ropframepermanent, permropinfo.ropstarthere, permropinfo.ropsize);
  

    
    return(eores_OK);   
}

extern eOresult_t eo_transmitter_permanentrops_Clear(EOtransmitter *p)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }  
    
    if(eobool_true == eo_list_Empty(p->listofpermropsinfo))
    {
        return(eores_OK);
    } 
    
    eo_list_Clear(p->listofpermropsinfo);
    
    eo_ropframe_Clear(p->ropframepermanent);

    return(eores_OK);   
}

extern eOresult_t eo_transmitter_permanentrops_Refresh(EOtransmitter *p)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }  
    
    if(eobool_true == eo_list_Empty(p->listofpermropsinfo))
    {
        return(eores_OK);
    } 
    
    p->currenttime = eov_sys_LifeTimeGet(eov_sys_GetHandle());
    
    // for each element in the list ... i do: ... see function
    eo_list_ForEach(p->listofpermropsinfo, s_eo_transmitter_list_updaterop_in_ropframe, p);

    return(eores_OK);   
}

extern eOresult_t eo_transmitter_outpacket_Get(EOtransmitter *p, EOpacket **outpkt, uint16_t *numberofrops)
{
    uint16_t remainingbytes;
    uint16_t size;

    if((NULL == p) || (NULL == outpkt)) 
    {
        return(eores_NOK_nullpointer);
    }

    
    // clear the content of the ropframe to transmit which uses the same storage of the packet ...
    eo_ropframe_Clear(p->ropframereadytotx);
    
    // add to it the permanent ropframe.
    eo_ropframe_Append(p->ropframereadytotx, p->ropframepermanent, &remainingbytes);

    // add the temporary ropframe and then clear it
    eo_ropframe_Append(p->ropframereadytotx, p->ropframetemporary, &remainingbytes);
    eo_ropframe_Clear(p->ropframetemporary);

    // add the replies ropframe and then clear it
    eo_ropframe_Append(p->ropframereadytotx, p->ropframereplies, &remainingbytes);
    eo_ropframe_Clear(p->ropframereplies);

    // now add the age of the frame
    eo_ropframe_age_Set(p->ropframereadytotx, eov_sys_LifeTimeGet(eov_sys_GetHandle()));
    
    // now set the size of the packet according to what is inside the ropframe.
    eo_ropframe_Size_Get(p->ropframereadytotx, &size);
    eo_packet_Size_Set(p->txpacket, size);
    
    // finally gives back the packet
    *outpkt = p->txpacket;

    // and the number of contained rops
    if(NULL != numberofrops)
    {
       *numberofrops = eo_ropframe_ROP_NumberOf(p->ropframereadytotx);
    }
    
    return(eores_OK);   
}


extern eOresult_t eo_transmitter_temporaryrop_Load(EOtransmitter *p, eOropcode_t ropcode, eOnvEP_t nvep, eOnvID_t nvid, eOropconfig_t ropcfg)
{
//    eo_transm_permrops_info_t permropinfo;
    eOresult_t res;
    uint16_t usedbytes;
    uint16_t ropsize;
    uint16_t remainingbytes;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }  

   
    // prepare the rop in p->roptmp
    
    res = eo_agent_OutROPinit(p->theagent, p->nvscfg, 
                              p->ipv4addr, p->ipv4port, 
                              ropcode, nvep, nvid, ropcfg, 
                              p->roptmp, &usedbytes);
                              
    if(eores_OK != res)
    {
        return(res);
    }

    // put the rop inside the ropframe
    res = eo_ropframe_ROP_Set(p->ropframetemporary, p->roptmp, NULL, &ropsize, &remainingbytes);
    
    
    return(res);   
}
 


extern eOresult_t eo_transmitter_ropframereplies_Append(EOtransmitter *p, EOropframe* ropframe)
{
    eOresult_t res;
    uint16_t remainingbytes;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }  

    res = eo_ropframe_Append(p->ropframereplies, ropframe, &remainingbytes);

    return(res);     
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static eOresult_t s_eo_transmitter_listmatching_rule(void *item, void *param)
{
    eo_transm_permrops_info_t *inside = (eo_transm_permrops_info_t*)item;
    eo_transm_permrops_info_t *target = (eo_transm_permrops_info_t*)param;
    
    if((inside->nvid == target->nvid) && (inside->ropcode == target->ropcode))
    {
        return(eores_OK);
    }
    else
    {
        return(eores_NOK_generic);
    }
}


static void s_eo_transmitter_list_updaterop_in_ropframe(void *item, void *param)
{
    eo_transm_permrops_info_t *inside = (eo_transm_permrops_info_t*)item;
    EOtransmitter *p = (EOtransmitter*)param;
    
    uint8_t *origofrop;
    uint8_t *dest;
    
    // retrieve the beginning of the ropstream inside the ropframe
    origofrop = eo_ropframe_hid_get_pointer_offset(p->ropframepermanent, inside->ropstarthere);
    dest = origofrop + sizeof(eOrophead_t);

    // if it has a data field ... copy from the nv to the ropstream
    if(NULL != inside->nvloc)
    {
        memcpy(dest, inside->nvloc, inside->capacity);
        // for debug: memset(dest, 0xaa, inside->capacity); 
    }
    
    // if it has a time field ... copy from the current time to the ropstream
    if(EOK_uint16dummy != inside->timeoffsetinsiderop)
    {
        eOabstime_t *time = (eOabstime_t*) ((uint32_t)((uint32_t)origofrop+inside->timeoffsetinsiderop));
        //*time = p->currenttime;
        memcpy(time, &p->currenttime, sizeof(eOabstime_t));
    }

}


static void s_eo_transmitter_list_shiftdownropinfo(void *item, void *param)
{
    eo_transm_permrops_info_t *after = (eo_transm_permrops_info_t*)item;
    eo_transm_permrops_info_t *theone2beremoved = (eo_transm_permrops_info_t*)param;

    // for each element after: modify its content so that ropstarthere is decremented by theone2beremoved.ropsize

    after->ropstarthere -= theone2beremoved->ropsize;
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




