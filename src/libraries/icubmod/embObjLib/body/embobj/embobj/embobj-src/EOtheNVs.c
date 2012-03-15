

/* @file       EOtheNVs.c
    @brief      This file implements internal implementation of the gpio singleton.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOnv.h" 





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVs.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheNVs_hid.h" 


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

//static eOresult_t s_eo_nvs_VerifyCfg(const eOnvs_cfg_t * const p, eObool_t forceit);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtheNVs";
 
static EOtheNVs s_thenvs = 
{
    .mutex          = NULL,
    .nvscfg         = NULL
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

 
extern EOtheNVs * eo_nvs_Initialise(EOnvsCfg* cfg, EOVmutexDerived *mtx) 
{
 
    if(NULL != s_thenvs.nvscfg)
    {
        return(&s_thenvs);
    }
    
    // verify cfg is not NULL
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != cfg), s_eobj_ownname, "cfg is NULL");

    // ok ... use the cfg
    s_thenvs.nvscfg = cfg;
 
    s_thenvs.mutex = mtx;   

    eo_nvscfg_data_Initialise(cfg);    


    return(&s_thenvs);        
} 

extern EOtheNVs * eo_nvs_GetHandle(void) 
{
    return( (NULL != s_thenvs.nvcfg) ? (&s_thenvs) : (NULL) );
} 


extern eOresult_t eo_nvs_Take(EOtheNVs *nvs, eOreltime_t tout)
{
    if((NULL == nvs) || (NULL == nvs->mutex))
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eov_mutex_Take(nvs->mutex, tout));
}


extern eOresult_t eo_nvs_Release(EOtheNVs *nvs)
{
    if((NULL == nvs) || (NULL == nvs->mutex))
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eov_mutex_Release(nvs->mutex));
}


// null if not found, a pointer to the sub-tree if found.
extern EOtreenode * eo_nvs_TreeNodeNVcon_GetByID(EOtheNVs *p, eOnetvarOwnership_t ownership, eOipv4addr_t ipaddr, eOipv4port_t port, eOnetvarID_t id)
{
    uint16_t i = 0;
    uint8_t ipindex = 0;
    uint8_t portindex = 0;
    EOtreenode *ret = NULL;

    uint16_t num = 0;

    if((NULL == p) || (NULL == p->nvscfg))
    {
        return(NULL);
    }

    if(eo_nv_ownership_local == ownership)
    {
        ipindex = p->nvscfg->indexoflocaldevice;
        if(EOK_uint08dummy == ipindex)
        {
            return(NULL);
        }
        
        portindex = eo_nvscfg_ondevice_fromport2index(p->nvscfg, ipindex, port);
        if(EOK_uint08dummy == portindex)
        {
            return(NULL);
        }
        
        ret = eo_nvscfg_ondevice_onport_withID_GetTreeNode(p->nvscfg, ipindex, portindex, id); 
        
        // in case of local ownership the ipindex shall be in p->nvscfg->indexoflocaldevice. we need to find the portindex.
        ipindex = 0;
        portindex = p->nvcfg->fn_from_localport_to_index(port); 
        if(0xffff == portindex)
        {
            return(NULL);
        }

        arrayip     = p->nvcfg->nvsloc_port_id; //= (EOconstarray*) eo_constarray_At(p->nvcfg->nvsloc_port_id, ipindex);
        arraypo     = (EOconstarray*) eo_constarray_At(arrayip, portindex);
        num         = eo_constarray_Size(arraypo);
    }
    else
    {
//        i = p->nvcfg->fn_from_ipv4_to_remotedev_index(ipaddr);
//        if(0xffff == i)
//        {
            return(NULL);
//        }
//        num     = p->nvcfg->remotenvs[i].nvnum;
//        nvhead  = p->nvcfg->remotenvs[i].nvars;
    }

    for(i=0; i<num; i++)
    {
        EOnetvarNode *nvnode = eo_constarray_At(arraypo, i);

        if(id == eo_netvar_GetID(eo_netvarnode_GetNV(nvnode)))
        {
            return(nvnode);
        }
    }
    
    // else ... not found
    return(NULL);
}


extern EOnetvarNode * eo_nvs_GetNVnodeByIndex(EOtheNVs *p, eOnetvarOwnership_t ownership, eOipv4addr_t ipaddr, eOipv4port_t port, uint16_t index)
{
    uint16_t i = 0;
    uint8_t ipindex = 0;
    uint8_t portindex = 0;

    EOconstarray *arrayip;
    EOconstarray *arraypo;

    uint16_t num = 0;
//    const EOnetvarNode *nvhead = NULL;

    if((NULL == p) || (NULL == p->nvcfg))
    {
        return(NULL);
    }

    if(eo_nv_ownership_local == ownership)
    {
       // in case of local ownership the ipindex shall be 0. we need to find the portindex.
        ipindex = 0;
        portindex = p->nvcfg->fn_from_localport_to_index(port); 
        if(0xffff == portindex)
        {
            return(NULL);
        }

        arrayip     = p->nvcfg->nvsloc_port_id; //(EOconstarray*) eo_constarray_At(p->nvcfg->nvsloc_port_id, ipindex);
        arraypo     = (EOconstarray*) eo_constarray_At(arrayip, portindex);
        num         = eo_constarray_Size(arraypo);
    }
    else
    {
//        i       = p->nvcfg->fn_from_ipv4_to_remotedev_index(ipaddr);
        if(0xffff == i)
        {
            return(NULL);
        }
//        num     = p->nvcfg->remotenvs[i].nvnum;
//        nvhead  = p->nvcfg->remotenvs[i].nvars;
    }

    if(index >= num)
    {
        return(NULL);
    }
    
    return((EOnetvarNode *)eo_constarray_At(arraypo, index));

}


extern EOVstorage * eo_nvs_GetLocalStorage(EOtheNVs *p)
{
    if((NULL == p) || (NULL == p->nvcfg))
    {
        return(NULL);
    }
    return(p->nvcfg->fn_get_loc_storage());
}
  

//extern eOresult_t eo_nvs_InitNVs(EOtheNVs *p, eOnetvarOwnership_t ownership)
//{
//    uint16_t i = 0;
//    uint16_t j = 0;
//
//    uint16_t num = 0;
//    const EOnetvarNode *nvhead = NULL;
//
//    if(eo_nv_ownership_local == ownership)
//    {
//        num     = p->nvcfg->localnvs.nvnum;
//        nvhead  = p->nvcfg->localnvs.nvars;
//
//        for(i=0; i<num; i++)
//        {
//            eo_netvar_Init(nvhead[i].netvar);
//        }
//    }
//    else
//    {
//
//        for(j=0; j<p->nvcfg->remotenum; j++)
//        {
//            num     = p->nvcfg->remotenvs[j].nvnum;
//            nvhead  = p->nvcfg->remotenvs[j].nvars;
//    
//            for(i=0; i<num; i++)
//            {
//                eo_netvar_Init(nvhead[i].netvar);
//            }
//        }
//
//    }
//
//    
//    return(eores_OK);
//}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

// c must hve been previously initialised with eov_nvscfg_hid_ForcedInitialisation(const EOVtheNVsCfgDerived * const ptr)
extern void eo_nvs_hid_Load(EOtheNVs *p, EOnvsCfg* c) 
{
    // verify cfg is not NULL
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != c), s_eobj_ownname, "c is NULL");
    s_thenvs.nvcfg = c;
    eo_nvscfg_data_Initialise(c);
} 





// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eo_nvs_VerifyCfg(const eOnvs_cfg_t * const p, eObool_t forceit)
{

    if(eobool_false == forceit)
    {
        s_thenvs.verified = eobool_false;
    }
    
    if(eobool_true == s_thenvs.verified)
    {
        return(eores_OK);
    }
    
    #warning -> put in s_eo_nvs_VerifyCfg() a verification of the tree structure. think of it !
    // put in here a verification of the ...  tree structure for instance and if something is wrong call the error manager
    //eo_errman_Assert(eo_errman_GetHandle(), eo_dummy08 == p->inp_map[p->ninp].id, s_eobj_ownname, "incorrect pin mapping");
	
   
    // ok, verified
    s_thenvs.verified = eobool_true;

    return(eores_OK);

}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




