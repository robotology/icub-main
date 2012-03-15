
/* @file       EOvport.c
    @brief      This file implements internal implementation of a netvar object.
    @author     marco.accame@iit.it
    @date       09/03/2010
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"

#include "EOtheNVs.h"

#include "EOrop_hid.h" 

#include "EOnetvar_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOvport.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOvport_hid.h" 


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


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOvport";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOvportCfg * eo_vportcfg_New(void)
{
    EOvportCfg *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOvportCfg), 1);

    eo_vportcfg_Clear(retptr);
    
    return(retptr);
}

extern eOresult_t eo_vportcfg_Clear(EOvportCfg *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    p->sizeofarray    = 2;
	p->n            = 0;
	memset((void*)p->nvids, 0, EOVPORT_MAX_NVS*sizeof(uint16_t));

    return(eores_OK);
}

extern eOresult_t eo_vportcfg_PushBack(EOvportCfg *p, eOnetvarID_t nv)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    if(p->n >= EOVPORT_MAX_NVS)
    {
        return(eores_NOK_generic);
    }

    p->nvids[p->n]  = nv;
    p->n ++;

    p->sizeofarray    = 2 + 2*p->n;

    return(eores_OK);
}

extern void * eo_vportcfg_GetMemory(EOvportCfg *p, uint16_t *size)
{
    if(NULL == p)
    {
        *size = 0;
        return(NULL);
    }

    *size = p->sizeofarray;
    return((void*)&(p->sizeofarray));
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOvport * eo_vport_hid_New(void)
{
    EOvport *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOvport), 1);
    
    return(retptr);
}


// used to initialise the vport w/ a given cfg.
extern void eo_vport_hid_LoadCfg(EOvport *vp, EOvportCfg *cfg, eOnetvarOwnership_t ownership, uint16_t port, eOipv4addr_t ipaddr)
{
    uint16_t i = 0;
    uint16_t num = 0;
    uint16_t maxNVs = cfg->n;
    uint16_t nvidtmp = 0;
    uint16_t effective_size = 0;
    uint16_t dat_buffer_offset = 0;
    EOnetvar *nv = NULL;
    eOvport_dat_nvidsizeinfo_t *vpdataelem = NULL;

    // process vp->cfg
    if(&(vp->cfg) != cfg)
    {
        memcpy(&(vp->cfg), cfg, sizeof(EOvportCfg));
    }

    // process vp->dat
    vp->dat.sizeofarray = 2;

    // piglio i dati in cfg e per ognuno elaboro un nv
    for(i=0; i<maxNVs; i++)
    {
        nvidtmp = cfg->nvids[i];

        // get the local nv described by the nvidtmp
        nv = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), ownership, ipaddr, port, nvidtmp));

        if(NULL != nv)
        {
            effective_size = ((2+eo_netvar_Capacity(nv)+3) >> 2) << 2;

            if(((2+effective_size)+dat_buffer_offset) > EOVPORT_MAXSIZE_NVIDSIZEINFO)
            {   // there is no space enough in the buffer for more NVs, thus ... i quit the for() loop
                break;
            }
    
            // only if we have the nv and if it has a mirror, then we copy that into the vport.dat
            if(eobool_true == eo_netvar_hid_isMirrorable(nv))
            {
                num++;
                // get the corresponding data element of vport.dat
                vpdataelem = (eOvport_dat_nvidsizeinfo_t*)(&vp->dat.buffer_of_nvidsizeinfo[dat_buffer_offset]);
                // set the nvid of nv 
                vpdataelem->nvid = nvidtmp; 
                // acemor - very important note: if the nv is an arr-m we decide to put inside the vport.dat the whole
                //                               of it, thus the full capacity. the reason is that after we place the variable
                //                               in vport.dat, the device could change its size in runtime and we want properly 
                //                               reserved space
                // set the size ...
                vpdataelem->size = eo_netvar_Capacity(nv);
                // oad the address of this info in the mirror
                // note that: nv->data.mirror must be configured to have a value pointing to vport.mirrors
                eo_netvar_hid_loadMirror(nv, &(vpdataelem->info[0]));
     
                // and finally update the mirror
                eo_netvar_hid_updateMirror(nv);
            }
    
            // incremento dat_buffer_offset di 2+effective_size
            // +2 because we go forward by a nvid. +effective_size because we go forward by data (size+info) which is multiple of four
            dat_buffer_offset += (2+effective_size);
        }

    }

    vp->dat.n = num;
    vp->dat.sizeofarray = 2 + dat_buffer_offset;

}

// this function is used by a smart node to propagate the content of the received input vport into the network variables
// and then to call the function update()


extern void eo_vport_hid_MoveDat2NVs(EOvport *vp, eOnetvarOwnership_t ownership, uint16_t port, eOipv4addr_t ipaddr)
{
    uint16_t i = 0;
    uint16_t maxNVs = vp->dat.n;
    uint16_t effective_size = 0;
    uint16_t dat_buffer_offset = 0;
    EOnetvar *nv = NULL;
    eOvport_dat_nvidsizeinfo_t *vpdataelem = NULL;

    // prendo ciascun data element in vport1.dat e per ognuno scrivo la relativa netvar
    for(i=0; i<maxNVs; i++)
    {
        vpdataelem = (eOvport_dat_nvidsizeinfo_t*)(&vp->dat.buffer_of_nvidsizeinfo[dat_buffer_offset]);
         
        nv = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), ownership, ipaddr, port, vpdataelem->nvid));

        if(NULL != nv)
        {
            #warning -> uso sempre remoteSet  ... acemor: che vuol dire ???
            if(eo_nv_ownership_local == ownership)
            {
                eo_netvar_Set(nv, vpdataelem->info, eobool_true, eo_nv_upd_always); //forceset
            }
            else
            {
                eo_netvar_remoteSet(nv, vpdataelem->info, eo_nv_upd_always);
            }
        }

        // incremento dat_buffer_offset di 2+effective_size
        // +2 because we go forward by a nvid. +effective_size because we go forward by data (size+info) which is multiple of four
         effective_size = ((2+vpdataelem->size+3) >> 2) << 2;
        dat_buffer_offset += (2+effective_size);  
    }

}

//// this function is used to initialise an input vport so that the mirrors of variables points to data inside
//// the vport.dat
//// the device MUST update the input nvs using the method eo_netvar_Set() or eo_netvar_Reset() which take
//// care of copying the value of the nvs in its data section but also in their mirror.
// 
//extern void eo_vport_hid_SynchroniseCfg2Dat(EOvport *vp)
//{
//    uint16_t i = 0;
//    uint16_t num = 0;
//    uint16_t max = vp->cfg.n;
//    uint16_t nvidtmp = 0;
//    uint16_t offset = 0;
//    EOnetvar *nv = NULL;
//    eOvport_dat_nvidsizeinfo_t *vpdataelem = NULL;
//
//    vp->dat.sizeofarray = 2;
//
//    // piglio i dati in vport1.cfg e per ognuno elaboro un nv
//    for(i=0; i<max; i++)
//    {
//        nvidtmp = vp->cfg.nvids[i];
//
//        nv = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), nvidtmp, eo_nv_ownership_local, 0));
//
//        if((NULL != nv) && (NULL != nv->data.mirror))
//        {
//            num++;
//            // get the corresponding data element of vport.dat
//            vpdataelem = (eOvport_dat_nvidsizeinfo_t*)(&vp->dat.buffer_of_nvidsizeinfo[offset]);
//            vpdataelem->nvid = nvidtmp; 
//            // acemor - very important note: if the nv is an arr-m we decide to put inside the vport.dat the whole
//            //                               of it, thus the full capacity. the reason is that after we place the variable
//            //                               in vport.dat, the device could change its size in runtime and we want properly 
//            //                               reserved space
//            vpdataelem->size = nv->data.capacity;
//            *nv->data.mirror = &(vpdataelem->info[0]);
//            memcpy(*nv->data.mirror, nv->data.valuevol, nv->data.capacity); 
//
//            #warning meglio incapsulare ogni get/set/rst di netvar all'interno di EOnetvar per evitare frammentazione del codice e quindi errori
//            // idem per il mirror 
//        }
//
//        nvidtmp = ((2+nv->data.capacity+3) >> 2) << 2;
//        offset += (2+nvidtmp);
//
//
//        // il puntatore alla nvid: nv.
//
//        // se ha nv->data.mirror not null, then it is shadowable.
//        // allora:
//        // in &mydevice_vol.vport1.dat[offset] ci metto il nvid,
//        // in &mydevice_vol.vport1.dat[offset+2] ci metto la capacity che trovo in nv->data.capacity
//        // calcolo l'effective_size come il primo multiplo di quattro dopo il numero (2+size)
//        // metto in *nv->data.mirror l'indirizzo di mydevice_vol.vport1.dat[offset+2+2]
//        // copio quanto e' in nv->data.valuevol dentro *nv->data.mirror.
//        // incremento offset di 2+effective_size
//
//    }
//
//    vp->dat.n = num;
//    vp->dat.sizeofarray = 2 + offset;
//
//}

// this function is used by a smart node to propagate the content of the received input vport into the network variables
// and then to call the function after_rop
// in such a way, in the callback functions in after_rop the smart node can operate on each variable one after the other.

//extern void eo_vport_hid_SynchroniseDat2NVs(EOvport *vp, eOnetvarOwnership_t ownership, eOipv4addr_t ipaddr, eOropcode_t ropc)
//{
//    uint16_t i = 0;
//    uint16_t num = 0;
//    uint16_t max = vp->dat.n;
//    uint16_t tmp = 0;
//    uint16_t offset = 0;
//    EOnetvar *nv = NULL;
//    eOvport_dat_nvidsizeinfo_t *vpdataelem = NULL;
//
//
//    // prendo ciascun data element in vport1.dat e per ognuno scrivo la relativa netvar
//    for(i=0; i<max; i++)
//    {
//        vpdataelem = (eOvport_dat_nvidsizeinfo_t*)(&vp->dat.buffer_of_nvidsizeinfo[offset]);
//         
//        nv = eo_netvarnode_GetNV(eo_nvs_GetNVnodeByID(eo_nvs_GetHandle(), vpdataelem->nvid, ownership, ipaddr));
//
//        if(NULL != nv)
//        {
//            // copy vpdataelem->info inside the memory. 
//            // acemor - very important note: i use nv->data.capacity and not vpdataelem->size even for teh case of
//            //                               an arr-m variable because by convention inside the vport.dat the arr-m are contained
//            //                               in full.
//            memcpy(nv->data.valuevol, vpdataelem->info, nv->data.capacity); 
//
//            // i now call the callback
//            eo_netvar_hid_OnAfter_ROP(nv, ropc);
//        }
//
//        // now increment offset to point to next vpdataelem. use vpdataelem->size as it is the legal fiels to use 
//        tmp = ((2+vpdataelem->size+3) >> 2) << 2;
//        offset += (2+tmp);  // +2 because we go forward by a nvid. +tmp because we go forward by data (size+info) which multiple of four
//
//
//        // il puntatore alla nvid: nv.
//
//        // se ha nv->data.mirror not null, then it is shadowable.
//        // allora:
//        // in &mydevice_vol.vport1.dat[offset] ci metto il nvid,
//        // in &mydevice_vol.vport1.dat[offset+2] ci metto la capacity che trovo in nv->data.capacity
//        // calcolo l'effective_size come il primo multiplo di quattro dopo il numero (2+size)
//        // metto in *nv->data.mirror l'indirizzo di mydevice_vol.vport1.dat[offset+2+2]
//        // copio quanto e' in nv->data.valuevol dentro *nv->data.mirror.
//        // incremento offset di 2+effective_size
//
//    }
//
//
//
//}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




