
/* @file       EOnvsCfg.c
    @brief      This file implements internal implementation of a netvar object.
    @author     marco.accame@iit.it
    @date       09/03/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOnv_hid.h" 
#include "EOmatrix3d.h"

#include "EOtreenode_hid.h"





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOnvsCfg.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOnvsCfg_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define EO_NVSCFG_INIT_EVERY_NV

#define EO_NVSCFG_USE_CACHED_NVS

#undef  EO_NVSCFG_USE_HASHTABLE


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eo_nvscfg_devicesowneship_change(EOnvsCfg *p,  eOnvscfgOwnership_t ownership);


static uint8_t s_eo_nvscfg_ondevice_endpoint2index(EOnvsCfg* p, uint8_t ondevindex, eOnvEP_t endpoint);
static uint8_t s_eo_nvscfg_ondevice_onendpoint_id2index(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, eOnvID_t id);

#if defined(EO_NVSCFG_USE_HASHTABLE)
static uint16_t s_nvscfg_hashing(uint16_t ep, uint16_t sizeofhashtable);
#endif

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOnvsCfg";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOnvsCfg* eo_nvscfg_New(uint16_t ndevices, EOVstorageDerived* stg)
{
    EOnvsCfg *p = NULL;  

    eo_errman_Assert(eo_errman_GetHandle(), (0 != ndevices), s_eobj_ownname, "ndevices is zero");    

    // i get the memory for the object
    p = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOnvsCfg), 1);
    
    p->thedevices           = eo_vector_New(sizeof(EOnvsCfg_device_t*), ndevices, NULL, 0, NULL, NULL);
    p->ip2index             = eo_vector_New(sizeof(eOipv4addr_t), ndevices, NULL, 0, NULL, NULL);
    p->indexoflocaldevice   = EOK_uint08dummy;
    p->devicesowneship      = eo_nvscfg_devicesownership_none;
    p->storage              = stg;
    p->allnvs               = NULL;

    return(p);
}

extern eOnvscfgDevicesOwnership_t eo_nvscfg_GetDevicesOwnership(EOnvsCfg* p)
{
 	if(NULL == p) 
	{
		return(eo_nvscfg_devicesownership_none); 
	}

    return(p->devicesowneship);
}


extern uint8_t eo_nvscfg_GetIndexOfLocalDevice(EOnvsCfg* p)
{
 	if(NULL == p) 
	{
		return(EOK_uint08dummy); 
	}

    return(p->indexoflocaldevice);
}

extern eOresult_t eo_nvscfg_PushBackDevice(EOnvsCfg* p, eOnvscfgOwnership_t ownership, eOipv4addr_t ipaddress, eOuint16_fp_uint16_t hashfn_ep2index, uint16_t nendpoints)
{
    EOnvsCfg_device_t *dev = NULL;
 	if(NULL == p) 
	{
		return(eores_NOK_nullpointer); 
	}
    
    eo_errman_Assert(eo_errman_GetHandle(), (0 != nendpoints), s_eobj_ownname, "nendpoints is zero");
    
    eo_errman_Assert(eo_errman_GetHandle(), (eobool_false == eo_vector_Full(p->thedevices)), s_eobj_ownname, "->thedevices is full");

    eo_errman_Assert(eo_errman_GetHandle(), (eobool_true != eo_vector_Find(p->ip2index, &ipaddress, NULL)), s_eobj_ownname, "ip already inside");

    s_eo_nvscfg_devicesowneship_change(p, ownership);

    if(eo_nvscfg_ownership_local == ownership)
    {
        p->indexoflocaldevice   = (uint8_t)eo_vector_Size(p->thedevices);
    }
    
    dev = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOnvsCfg_device_t), 1);
    
    dev->ipaddress              = ipaddress;
    dev->theendpoints           = eo_vector_New(sizeof(EOnvsCfg_ep_t*), nendpoints, NULL, 0, NULL, NULL);
    dev->ownership              = ownership;
    dev->theendpoints_numberof  = nendpoints;
    dev->hashfn_ep2index        = hashfn_ep2index;
#if defined(EO_NVSCFG_USE_HASHTABLE)
    dev->ephashtable            = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(eOnvsCfgEPhash_t), nendpoints);
    // teh entries of teh hastable are all {0xffff, 0xffff} to tell that they are still invalid.
    memset(dev->ephashtable, 0xff,  sizeof(eOnvsCfgEPhash_t)*nendpoints); 
#endif

    eo_vector_PushBack(p->thedevices, &dev);

    eo_vector_PushBack(p->ip2index, &ipaddress);

    return(eores_OK);
}



extern eOresult_t eo_nvscfg_ondevice_PushBackEndpoint(EOnvsCfg* p, uint8_t ondevindex, eOnvEP_t endpoint, eOuint16_fp_uint16_t hashfn_id2index, const EOconstvector* treeofnvs_con, const EOconstvector* datanvs_usr, uint32_t datanvs_size, eOvoid_fp_voidp_voidp_t datanvs_init, EOVmutexDerived* mtx)
{
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t *theendpoint = NULL;
    uint8_t nnvs = 0;
 
    
 	if(NULL == p) 
	{
		return(eores_NOK_nullpointer); 
	}

    nnvs = (uint8_t)eo_constvector_Size(datanvs_usr);

    
    eo_errman_Assert(eo_errman_GetHandle(), (0 != nnvs), s_eobj_ownname, "nnvs is zero");
    eo_errman_Assert(eo_errman_GetHandle(), (eo_constvector_Size(treeofnvs_con) == nnvs), s_eobj_ownname, "_con and _usr are of different sizes");
    
    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != thedev), s_eobj_ownname, "->thedevices is indexed in wrong pos");

    
    eo_errman_Assert(eo_errman_GetHandle(), (eobool_false == eo_vector_Full((*thedev)->theendpoints)), s_eobj_ownname, "one of ->theendpoints is full");
 
    
    theendpoint = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOnvsCfg_ep_t), 1);
    
    theendpoint->endpoint           = endpoint;
    theendpoint->initted            = eobool_false;
    theendpoint->thenvs_numberof    = nnvs;
    // the constvector thetreeofnvs_con contains the EOtreenodes with field ->data which points to the EOnv_con 
    theendpoint->thetreeofnvs_con   = (EOconstvector*)treeofnvs_con; //eo_constvector_New(sizeof(EOtreenode), nnvs, treeofnvs_con);
    theendpoint->thenvs_usr         = (EOconstvector*)datanvs_usr; //eo_constvector_New(sizeof(EOnv_usr_t), nnvs, datanvs_cfg);    
    theendpoint->thenvs_vol         = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, datanvs_size, 1);
    if(eo_nvscfg_ownership_remote == (*thedev)->ownership)
    {
        theendpoint->thenvs_rem     = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, datanvs_size, 1);    
    }
    else
    {
        theendpoint->thenvs_rem     = NULL;
    }
    theendpoint->thenvs_initialise  = datanvs_init;
    theendpoint->thenvs_sizeof      = datanvs_size;
    theendpoint->hashfn_id2index    = hashfn_id2index;
    theendpoint->mtx_endpoint       = mtx;

#if defined(EO_NVSCFG_USE_HASHTABLE)
    {   // ok ... i must hash this endpoint with the used index.
        eOnvsCfgEPhash_t *hashentry;

        uint16_t hashindex = s_nvscfg_hashing(endpoint, (*thedev)->theendpoints_numberof);
        if(EOK_uint16dummy != hashindex)
        {
            hashentry = &((*thedev)->ephashtable[hashindex]);
            hashentry->index    = eo_vector_Size((*thedev)->theendpoints);
            hashentry->ephashed = endpoint;
        }
        else
        {
            // cannot fill in hashtable --> need to do an exhaustive search.
        }
    }
#endif    
    
    eo_vector_PushBack((*thedev)->theendpoints, &theendpoint);

    return(eores_OK);
}



extern eOresult_t eo_nvscfg_data_Initialise(EOnvsCfg* p)
{
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t **theendpoint = NULL;
    EOtreenode* treenode = NULL;
    uint16_t i, j, k;
    uint16_t ndev;
    uint16_t nendpoints;
    uint8_t nvars;
    eOvoid_fp_voidp_voidp_t initialise = NULL;
    EOnv tmpnv;

 	if(NULL == p) 
	{
		return(eores_NOK_nullpointer); 
	}

    ndev = eo_vector_Size(p->thedevices);



#if defined(EO_NVSCFG_USE_CACHED_NVS)
    p->allnvs  = eo_matrix3d_New(sizeof(EOnv), ndev);
#endif


    for(i=0; i<ndev; i++)
    {
        thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, i);
        nendpoints = eo_vector_Size((*thedev)->theendpoints);

#if defined(EO_NVSCFG_USE_CACHED_NVS)
        eo_matrix3d_Level1_PushBack(p->allnvs, nendpoints);
#endif

        for(j=0; j<nendpoints; j++)
        {
            theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, j);

            if(eobool_false == ((*theendpoint)->initted))
            {
                initialise = (*theendpoint)->thenvs_initialise;
                (*theendpoint)->initted = eobool_true;
                
                if((NULL != initialise))
                {
                    initialise((*theendpoint)->thenvs_vol, (*theendpoint)->thenvs_rem);
                }
            }

#if defined(EO_NVSCFG_INIT_EVERY_NV) || defined(EO_NVSCFG_USE_CACHED_NVS)

            nvars = (*theendpoint)->thenvs_numberof;

#if defined(EO_NVSCFG_USE_CACHED_NVS)
            eo_matrix3d_Level2_PushBack(p->allnvs, i, nvars);
#endif
            for(k=0; k<nvars; k++)
            {
                treenode = (EOtreenode*) eo_constvector_At((*theendpoint)->thetreeofnvs_con, k);

                tmpnv.con = (EOnv_con_t*) eo_treenode_GetData(treenode);
                tmpnv.usr = (EOnv_usr_t*) eo_constvector_At((*theendpoint)->thenvs_usr, k);
                tmpnv.loc = (void*) ((uint32_t)((*theendpoint)->thenvs_vol) + tmpnv.con->offset);
                if(eo_nvscfg_ownership_remote == (*thedev)->ownership)
                {
                    tmpnv.rem = (void*) ((uint32_t)((*theendpoint)->thenvs_rem) + tmpnv.con->offset);   
                }
                else
                {
                    tmpnv.rem = NULL;
                }
                tmpnv.mtx = (*theendpoint)->mtx_endpoint;
                tmpnv.stg = p->storage;

#if defined(EO_NVSCFG_INIT_EVERY_NV)
                eo_nv_Init(&tmpnv); 
#endif  
                
#if defined(EO_NVSCFG_USE_CACHED_NVS)
                eo_matrix3d_Level3_PushBack(p->allnvs, i, j, &tmpnv);
#endif
                             
            }
#endif//EO_NVSCFG_INIT_EVERY_NV           
              
        }
            
    }

    return(eores_OK);

}

#if 0
extern void * eo_nvscfg_localdev_endpoint_GetRAM(EOnvsCfg* p, eOnvEP_t endpoint)
{
    uint8_t ondevindex, onendpointindex;
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t **theendpoint = NULL;
 

    if((NULL == p) || (EOK_uint08dummy == p->indexoflocaldevice)) 
	{
		return(NULL); 
	}

    ondevindex = p->indexoflocaldevice;

    onendpointindex = eo_nvscfg_hid_ondevice_endpoint2index(p, ondevindex, endpoint);

    if(EOK_uint08dummy == onendpointindex)
    {
        return(NULL);
    }

    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, onendpointindex);

    return((*theendpoint)->thenvs_vol);
}
#endif



extern eOresult_t eo_nvscfg_GetIndices(EOnvsCfg* p, 
                                       eOipv4addr_t ip, eOnvEP_t ep, eOnvID_t id, 
                                       uint8_t *ipindex, uint8_t *epindex, uint8_t *idindex)
{
    if((NULL == p) || (NULL == ipindex) || (NULL == epindex) || (NULL == idindex))
    {
        return(eores_NOK_nullpointer);
    }

    // --- search for the index of the ip

    if(eok_ipv4addr_localhost == ip)
    {
        *ipindex = p->indexoflocaldevice;
    }
    else
    {
        *ipindex = eo_nvscfg_hid_ip2index(p, ip);
    }

    if(EOK_uint08dummy == *ipindex)
    {
        return(eores_NOK_generic);
    }

    // --- search for the index of the ep

    *epindex = s_eo_nvscfg_ondevice_endpoint2index(p, *ipindex, ep);
    if(EOK_uint08dummy == *epindex)
    {
        return(eores_NOK_generic);
    }

    // --- search for the index of the id

    *idindex = s_eo_nvscfg_ondevice_onendpoint_id2index(p, *ipindex, *epindex, id);
    if(EOK_uint08dummy == *idindex)
    {
        return(eores_NOK_generic);
    }

    return(eores_OK);
}



extern EOtreenode* eo_nvscfg_GetTreeNode(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, uint8_t onidindex)
{
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t **theendpoint = NULL;


    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    if(NULL == thedev)
    {
        return(NULL);
    }
    theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, onendpointindex);
    if(NULL == theendpoint)
    {
        return(NULL);
    }

    return((EOtreenode*) eo_constvector_At((*theendpoint)->thetreeofnvs_con, onidindex));
}




extern EOnv* eo_nvscfg_GetNV(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, uint8_t onidindex, EOtreenode* treenode, EOnv* nvtarget)
{

#if defined(EO_NVSCFG_USE_CACHED_NVS)

    // --- we use just the indices

    EOnv *nv;

    if(NULL == p) 
	{
		return(NULL); 
	}
    
    nv = (EOnv*) eo_matrix3d_At(p->allnvs, ondevindex, onendpointindex, onidindex);
    
    if((NULL != nv) && (NULL != nvtarget))
    {
        memcpy(nvtarget, nv, sizeof(EOnv));
    }

    return(nv);
    
#else

    // -- we use also the treenode

    EOnv* nv;
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t **theendpoint = NULL;
    uint16_t k = 0;
 
    if((NULL == p) || (NULL == nvtarget)) 
	{
		return(NULL); 
	}

    if(NULL == treenode)
    {
        treenode = eo_nvscfg_GetTreeNode(p, ondevindex, onendpointindex, onidindex);
        if(NULL == treenode)
        {
            return(NULL);
        }
    }

    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    if(NULL == thedev)
    {
        return(NULL);
    }
    theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, onendpointindex);
    if(NULL == theendpoint)
    {
        return(NULL);
    }

    k = onidindex; // or eo_treenode_GetIndex(treenode);
    nv = nvtarget;

    nv->con = (EOnv_con_t*) eo_treenode_GetData(treenode);
    nv->usr = (EOnv_usr_t*) eo_constvector_At((*theendpoint)->thenvs_usr, k);
    nv->loc = (void*) ((uint32_t)((*theendpoint)->thenvs_vol) + nv->con->offset);

    if(eo_nvscfg_ownership_remote == (*thedev)->ownership)
    {
        nv->rem = (void*) ((uint32_t)((*theendpoint)->thenvs_rem) + nv->con->offset);   
    }
    else
    {
        nv->rem = NULL;
    }

    nv->mtx = (*theendpoint)->mtx_endpoint;
    nv->stg = p->storage;


    return(nv);

#endif

}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



extern uint8_t eo_nvscfg_hid_ip2index(EOnvsCfg* p, eOipv4addr_t ipaddress)
{
    uint16_t index = 0;

    if(NULL == p)
    {
        return(EOK_uint08dummy);
    }

    if(eobool_true == eo_vector_Find(p->ip2index, &ipaddress, &index))
    {
        return((uint8_t)index);  
    }

    return(EOK_uint08dummy);
}

extern uint8_t eo_nvscfg_hid_ondevice_endpoint2index(EOnvsCfg* p, uint8_t ondevindex, eOnvEP_t endpoint)
{
    if(NULL == p)
    {
        return(EOK_uint08dummy);
    }

    return(s_eo_nvscfg_ondevice_endpoint2index(p, ondevindex, endpoint));
}


extern uint8_t eo_nvscfg_hid_ondevice_onendpoint_id2index(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, eOnvID_t id)
{
    if(NULL == p)
    {
        return(EOK_uint08dummy);
    }

    return(s_eo_nvscfg_ondevice_onendpoint_id2index(p, ondevindex, onendpointindex, id));
}


extern EOtreenode* eo_nvscfg_hid_ondevice_onendpoint_withID_GetTreeNode(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, eOnvID_t id)
{
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t **theendpoint = NULL;


    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    if(NULL == thedev)
    {
        return(NULL);
    }
    theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, onendpointindex);
    if(NULL == theendpoint)
    {
        return(NULL);
    }

    if((NULL != (*theendpoint)->hashfn_id2index))
    {
        uint16_t index = (*theendpoint)->hashfn_id2index(id);
        if(EOK_uint16dummy != index)
        {
            return((EOtreenode*) eo_constvector_At((*theendpoint)->thetreeofnvs_con, index));
        }        
    }


    if(1)
    {   // cannot find w/ hash function thus i use exhaustive search   
        EOnv_con_t *nvcon = NULL;
        uint16_t nvars = 0;
        uint16_t k = 0; 
        EOtreenode *treenode = NULL; 

        nvars = (*theendpoint)->thenvs_numberof;
        for(k=0; k<nvars; k++)
        {
            treenode = (EOtreenode*) eo_constvector_At((*theendpoint)->thetreeofnvs_con, k);
            nvcon = (EOnv_con_t*) eo_treenode_GetData(treenode);
    
            if(id == nvcon->id)
            {
                return(treenode);
            }
        }
    }


    return(NULL);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eo_nvscfg_devicesowneship_change(EOnvsCfg *p,  eOnvscfgOwnership_t ownership)
{
    switch(p->devicesowneship)
    {
        case eo_nvscfg_devicesownership_none:
        {
            if(eo_nvscfg_ownership_local == ownership)
            {
                p->devicesowneship = eo_nvscfg_devicesownership_onelocal;    
            }
            else
            {
                p->devicesowneship = eo_nvscfg_devicesownership_someremote;
            }
        } break;

        case eo_nvscfg_devicesownership_onelocal:
        {
            if(eo_nvscfg_ownership_local == ownership)
            {
                eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "at most one local");
            }
            else
            {
                p->devicesowneship = eo_nvscfg_devicesownership_onelocalsomeremote;
            }
        } break;

        case eo_nvscfg_devicesownership_onelocalsomeremote:
        {
            if(eo_nvscfg_ownership_local == ownership)
            {
                eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "at most one local");
            }
            else
            {
                p->devicesowneship = eo_nvscfg_devicesownership_onelocalsomeremote;
            }
        } break;

        case eo_nvscfg_devicesownership_someremote:
        {
            if(eo_nvscfg_ownership_local == ownership)
            {
                 p->devicesowneship = eo_nvscfg_devicesownership_onelocalsomeremote;    
            }
            else
            {
                p->devicesowneship = eo_nvscfg_devicesownership_someremote;
            }
        } break;

        default:
        {
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "verify your code");
        } break;

    }

}

#if defined(EO_NVSCFG_USE_HASHTABLE) 
static uint16_t s_nvscfg_hashing(uint16_t ep, uint16_t sizeofhashtable)
{
    uint16_t ret = (ep < sizeofhashtable) ? (ep) : (EOK_uint16dummy);
    return(ret);
}
#endif

static uint8_t s_eo_nvscfg_ondevice_endpoint2index(EOnvsCfg* p, uint8_t ondevindex, eOnvEP_t endpoint)
{
    EOnvsCfg_device_t** thedev = NULL;

    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    if(NULL == thedev)
    {
        return(EOK_uint08dummy);
    }

#if defined(EO_NVSCFG_USE_HASHTABLE)
    {
        eOnvsCfgEPhash_t *hashentry;
        uint16_t hashindex;

        hashindex = s_nvscfg_hashing(endpoint, (*thedev)->theendpoints_numberof);
    
        if((EOK_uint16dummy != hashindex))
        {
            hashentry = &((*thedev)->ephashtable[hashindex]);
            if(endpoint == hashentry->ephashed)
            {
                return(hashentry->index);
            }
        }
    }
#else

    if((NULL != (*thedev)->hashfn_ep2index))
    {
        uint16_t index = (*thedev)->hashfn_ep2index(endpoint);
        if(EOK_uint16dummy != index)
        {
            return(index);
        }        
    }

#endif


    if(1)
    {   // cannot fill w/ hash --> need to do an exhaustive search.
        uint8_t j = 0;
        uint8_t nendpoints = (uint8_t)eo_vector_Size((*thedev)->theendpoints);
        for(j=0; j<nendpoints; j++)
        {
            EOnvsCfg_ep_t **theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, j);
    
            if(endpoint == (*theendpoint)->endpoint)
            {
                return(j);
            }
        }

    }

    return(EOK_uint08dummy);
}


static uint8_t s_eo_nvscfg_ondevice_onendpoint_id2index(EOnvsCfg* p, uint8_t ondevindex, uint8_t onendpointindex, eOnvID_t id)
{
    EOnvsCfg_device_t** thedev = NULL;
    EOnvsCfg_ep_t **theendpoint = NULL;

    thedev = (EOnvsCfg_device_t**) eo_vector_At(p->thedevices, ondevindex);
    if(NULL == thedev)
    {
        return(EOK_uint08dummy);
    }
    theendpoint = (EOnvsCfg_ep_t**) eo_vector_At((*thedev)->theendpoints, onendpointindex);
    if(NULL == theendpoint)
    {
        return(EOK_uint08dummy);
    }

 
    if((NULL != (*theendpoint)->hashfn_id2index))
    {
        uint16_t index = (*theendpoint)->hashfn_id2index(id);
        if(EOK_uint16dummy != index)
        {
            return(index);
        }        
    }

    if(1)
    {   // cannot find with hash function, thus use exhaustive search
        uint8_t k = 0;
        uint8_t nvars = (*theendpoint)->thenvs_numberof;
    
        for(k=0; k<nvars; k++)
        {
            EOnv_con_t* nvcon = (EOnv_con_t*) eo_treenode_GetData((EOtreenode*) eo_constvector_At((*theendpoint)->thetreeofnvs_con, k));
    
            if(id == nvcon->id)
            {
                return(k);
            }
        }

    }

    return(EOK_uint08dummy);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




