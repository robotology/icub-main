
/* @file       EOnv.c
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

#include "EOVmutex.h"
#include "EOVstorage.h"

#include "EOrop.h" 
#include "EOarray.h" 





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOnv.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOnv_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define eov_mutex_Take(a, b)   

#define eov_mutex_Release(a)

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eo_nv_Set(const EOnv *netvar, const void *dat, void *dst, eOnvUpdate_t upd);
static eOresult_t s_eo_nv_SetTS(const EOnv *nv, const void *dat, void *dst, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign);

EO_static_inline uint16_t s_eo_nv_array_get_size(void* data)
{
    // 4 bytes are for teh capacity and the size fields, whcih are always present. head->size are the othres
    return(eo_array_UsedBytes((EOarray*)data));    
}

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOnv";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOnv * eo_nv_New(void)
{
    EOnv *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOnv), 1);

    eo_nv_Clear(retptr);
      
    return(retptr);
}


extern eOresult_t eo_nv_Clear(EOnv *nv)
{
    if(NULL == nv)
    {
        return(eores_NOK_nullpointer);
    }
    
    nv->con     = NULL;       
    nv->usr     = NULL;
    nv->loc     = NULL;  
    nv->rem     = NULL; 
    nv->mtx     = NULL;
    nv->stg     = NULL;
      
    return(eores_OK);
}


extern eOresult_t eo_nv_Set(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd)
{
    if(NULL == nv)
    {
        return(eores_NOK_nullpointer);
    }


    if(eobool_true == forceset)
    {
        // ok. go on.
    }
    else if(eobool_false == eo_nv_hid_isWritable(nv))
    {
        return(eores_NOK_generic);
    }

   return(s_eo_nv_Set(nv, dat, nv->loc, upd));
}

extern eOresult_t eo_nv_SetTS(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign)
{
    if(NULL == nv)
    {
        return(eores_NOK_nullpointer);
    }


    if(eobool_true == forceset)
    {
        // ok. go on.
    }
    else if(eobool_false == eo_nv_hid_isWritable(nv))
    {
        return(eores_NOK_generic);
    }

   return(s_eo_nv_SetTS(nv, dat, nv->loc, upd, time, sign));
}



extern eOresult_t eo_nv_Reset(const EOnv *nv, eObool_t forcerst, eOnvUpdate_t upd)
{
    if(NULL == nv)
    {
        return(eores_NOK_nullpointer);
    }

    if(eobool_true == forcerst)
    {
        // ok. go on.
    }
    else if(eobool_false == eo_nv_hid_isWritable(nv))
    {
        return(eores_NOK_generic);
    }

    return(s_eo_nv_Set(nv, nv->con->resetval, nv->loc, upd));

}

extern eOresult_t eo_nv_ResetTS(const EOnv *nv, eObool_t forcerst, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign)
{
    if(NULL == nv)
    {
        return(eores_NOK_nullpointer);
    }

    if(eobool_true == forcerst)
    {
        // ok. go on.
    }
    else if(eobool_false == eo_nv_hid_isWritable(nv))
    {
        return(eores_NOK_generic);
    }

    return(s_eo_nv_SetTS(nv, nv->con->resetval, nv->loc, upd, time, sign));

}


extern uint16_t eo_nv_Capacity(const EOnv *nv)
{
    if(NULL == nv)
    {
        return(0);
    }
    
    return(nv->con->capacity);  
}


extern uint16_t eo_nv_Size(const EOnv *nv, const void *data)
{
    uint16_t size = 0;
    eObool_t typisarray = eobool_false;
    void *dat = NULL;

    if(NULL == nv)
    {
        return(0);
    }

    typisarray = (eo_nv_TYP_arr == nv->con->typ) ? (eobool_true) : (eobool_false);
    dat = (NULL != data) ? ((void*)data) : (nv->loc);

    size = (eobool_false == typisarray) ? (nv->con->capacity) : (s_eo_nv_array_get_size(dat));
       
    return(size);  
}

// fills data with capacity bytes, ... but if an array typ then size is not capacity ...
extern eOresult_t eo_nv_Get(const EOnv *nv, eOnvStorage_t strg, void *data, uint16_t *size)
{
    eOresult_t res = eores_NOK_generic;
    void *source = NULL;
    eObool_t typisarray = eobool_false;
    
    if((NULL == data) || (NULL == size) || (NULL == nv))
    {
        return(eores_NOK_nullpointer);
    }


    typisarray = (eo_nv_TYP_arr == nv->con->typ) ? (eobool_true) : (eobool_false);    


    switch(strg)
    {
        case eo_nv_strg_volatile:
        {   // better to protect so that the copy is atomic and not interrupted by other tasks which write 
            eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
            source = nv->loc;
            //*size = (eobool_false == typisarray) ? (nv->con->capacity) : (2 + *((uint16_t*)source));
            *size = (eobool_false == typisarray) ? (nv->con->capacity) : (s_eo_nv_array_get_size(source));
            memcpy(data, source, *size); 
            eov_mutex_Release(nv->mtx);
            res = eores_OK;
        } break;

        case eo_nv_strg_default:
        {   // no need to protect as the data is read only
            source = (void*)nv->con->resetval;
            //*size = (eobool_false == typisarray) ? (nv->con->capacity) : (2 + *((uint16_t*)source));
            *size = (eobool_false == typisarray) ? (nv->con->capacity) : (s_eo_nv_array_get_size(source));            
            memcpy(data, source, *size);
            res = eores_OK;
        } break;

        case eo_nv_strg_permanent:
        {   // protection is done inside eo_nv_hid_GetPERMANENT()
            res = eo_nv_hid_GetPERMANENT(nv, data, size);
        } break;
    }

    
    return(res);  
}

extern eOresult_t eo_nv_Init(const EOnv *nv)
{
    eOresult_t res = eores_NOK_generic;
    // call the init function if existing
    if((NULL != nv->usr->peripheralinterface) && (NULL != nv->usr->peripheralinterface->init))
    {   // protect ...
        eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
        nv->usr->peripheralinterface->init(nv);
        eov_mutex_Release(nv->mtx);
        res = eores_OK;
    }

    return(res);
}

extern eOresult_t eo_nv_Update(const EOnv *nv)
{
    eOresult_t res = eores_NOK_generic;
    // call the update function if necessary, to propagate the change of data to the peripheral (out)
    if(eobool_true == eo_nv_hid_isUpdateable(nv)) 
    {
        if((NULL != nv->usr->peripheralinterface) && (NULL != nv->usr->peripheralinterface->update))
        {   // protect
            eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
            nv->usr->peripheralinterface->update(nv, eok_uint64dummy, eok_uint32dummy);
            eov_mutex_Release(nv->mtx);
            res = eores_OK;
        }
    }

    return(res);
}

extern eOresult_t eo_nv_UpdateTS(const EOnv *nv, eOabstime_t roptime, uint32_t ropsign)
{
    eOresult_t res = eores_NOK_generic;
    // call the update function if necessary, to propagate the change of data to the peripheral (out)
    if(eobool_true == eo_nv_hid_isUpdateable(nv)) 
    {
        if((NULL != nv->usr->peripheralinterface) && (NULL != nv->usr->peripheralinterface->update))
        {   // protect
            eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
            nv->usr->peripheralinterface->update(nv, roptime, ropsign);
            eov_mutex_Release(nv->mtx);
            res = eores_OK;
        }
    }

    return(res);
}

extern eOnvID_t eo_nv_GetID(const EOnv *nv)
{
    if(NULL == nv)
    {
        return(0);
    }
    return(nv->con->id);

}


extern eOresult_t eo_nv_remoteSet(const EOnv *nv, const void *dat, eOnvUpdate_t upd)
{
    if((NULL == nv) || (NULL == dat))
    {
        return(eores_NOK_nullpointer);
    }

    if(eobool_true == eo_nv_hid_isLocal(nv))
    {
        return(eores_NOK_generic);
    }

    return(s_eo_nv_Set(nv, dat, nv->rem, upd));
}


extern eOresult_t eo_nv_remoteSetTS(const EOnv *nv, const void *dat, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign)
{
    if((NULL == nv) || (NULL == dat))
    {
        return(eores_NOK_nullpointer);
    }

    if(eobool_true == eo_nv_hid_isLocal(nv))
    {
        return(eores_NOK_generic);
    }

    return(s_eo_nv_SetTS(nv, dat, nv->rem, upd, time, sign));
}

extern eOresult_t eo_nv_remoteGet(const EOnv *nv, void *data, uint16_t *size)
{
    void *source = NULL;
    eObool_t typisarray = eobool_false;
    
    if((NULL == data) || (NULL == size) || (NULL == nv))
    {
        return(eores_NOK_nullpointer);
    }

    if(eobool_true == eo_nv_hid_isLocal(nv))
    {
        return(eores_NOK_generic);
    }


    typisarray = (eo_nv_TYP_arr == nv->con->typ) ? (eobool_true) : (eobool_false);    

    eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
    source = nv->rem;
    *size = (eobool_false == typisarray) ? (nv->con->capacity) : (s_eo_nv_array_get_size(source));
    memcpy(data, source, *size);
    eov_mutex_Release(nv->mtx);

    
    return(eores_OK);  
}

extern eOnvFunc_t eo_nv_GetFUN(const EOnv *netvar)
{
    if((NULL == netvar) || (NULL == netvar->con))
    {
        return(eo_nv_FUN_NO0);
    }

    return((eOnvFunc_t)netvar->con->fun);
}


extern eOnvType_t eo_nv_GetTYP(const EOnv *netvar)
{
    if((NULL == netvar) || (NULL == netvar->con))
    {
        return(eo_nv_TYP_NO4);
    }

    return((eOnvType_t)netvar->con->typ);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------




extern eObool_t eo_nv_hid_OnBefore_ROP(const EOnv *nv, eOropcode_t ropcode, eOabstime_t roptime, uint32_t ropsign)
{
    eObool_t ret = eobool_false;
    eOvoid_fp_cnvp_cabstime_cuint32_t fn_before_rop = NULL;

    if((NULL == nv) || (NULL == nv->usr->on_rop_reception))
    {
        return(ret);
    }

    switch(ropcode)
    {
        case eo_ropcode_ask:
        {
            fn_before_rop = nv->usr->on_rop_reception->loc.ask.bef;
        } break;

        case eo_ropcode_say:
        {
            fn_before_rop = nv->usr->on_rop_reception->rem.say.bef;
        } break;

        case eo_ropcode_set:
        {
            fn_before_rop = nv->usr->on_rop_reception->loc.set.bef;
        } break;

        case eo_ropcode_sig:
        {
            fn_before_rop = nv->usr->on_rop_reception->rem.sig.bef;
        } break;

        case eo_ropcode_rst:
        {
            fn_before_rop = nv->usr->on_rop_reception->loc.rst.bef;
        } break;

        case eo_ropcode_upd:
        {
            fn_before_rop = nv->usr->on_rop_reception->loc.upd.bef;

        } break;
    }

    if(NULL != fn_before_rop)
    {
        eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
        fn_before_rop(nv, roptime, ropsign);
        eov_mutex_Release(nv->mtx);
        ret = eobool_true;
    }

    return(ret);
   
}

extern eObool_t eo_nv_hid_OnAfter_ROP(const EOnv *nv, eOropcode_t ropcode, eOabstime_t roptime, uint32_t ropsign)
{
    eObool_t ret = eobool_false;
    eOvoid_fp_cnvp_cabstime_cuint32_t fn_after_rop = NULL;

    if((NULL == nv) || (NULL == nv->usr->on_rop_reception))
    {
        return(ret);
    }

    switch(ropcode)
    {
        case eo_ropcode_ask:
        {
            fn_after_rop = nv->usr->on_rop_reception->loc.ask.aft;
        } break;

        case eo_ropcode_say:
        {
            fn_after_rop = nv->usr->on_rop_reception->rem.say.aft;
        } break;

        case eo_ropcode_set:
        {
            fn_after_rop = nv->usr->on_rop_reception->loc.set.aft;
        } break;

        case eo_ropcode_sig:
        {
            fn_after_rop = nv->usr->on_rop_reception->rem.sig.aft;
        } break;

        case eo_ropcode_rst:
        {
            fn_after_rop = nv->usr->on_rop_reception->loc.rst.aft;
        } break;

        case eo_ropcode_upd:
        {
            fn_after_rop = nv->usr->on_rop_reception->loc.upd.aft;
        } break;
    }

    if(NULL != fn_after_rop)
    {
        eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);
        fn_after_rop(nv, roptime, ropsign);
        eov_mutex_Release(nv->mtx);
        ret = eobool_true;
    }
        
    return(ret);
}



extern eObool_t eo_nv_hid_isWritable(const EOnv *nv)
{
    eOnvFunc_t fun = (eOnvFunc_t)nv->con->fun;

    if((fun == eo_nv_FUN_out) || (fun == eo_nv_FUN_cfg) || (fun == eo_nv_FUN_beh))
    {
        return(eobool_true);
    }
    else
    {
        return(eobool_false);
    }
}


extern eObool_t eo_nv_hid_isLocal(const EOnv *nv)
{
    return((NULL == nv->rem) ? (eobool_true) : (eobool_false));
} 

extern eObool_t eo_nv_hid_isPermanent(const EOnv *nv)
{
    eOnvFunc_t fun = (eOnvFunc_t)nv->con->fun;

    if(fun == eo_nv_FUN_cfg)
    {
        return(eobool_true);
    }
    else
    {
        return(eobool_false);
    }
} 

extern eObool_t eo_nv_hid_isUpdateable(const EOnv *nv)
{
    eOnvFunc_t fun = (eOnvFunc_t)nv->con->fun;

    if((fun == eo_nv_FUN_inp) || (fun == eo_nv_FUN_out) || (fun == eo_nv_FUN_cfg) || (fun == eo_nv_FUN_beh))
    {
        return(eobool_true);
    }
    else
    {
        return(eobool_false);
    }
} 



extern const void* eo_nv_hih_GetDEFAULT(const EOnv *nv)
{
    if(NULL == nv)
    {
        return(NULL);
    }

    return(nv->con->resetval);
}

extern void* eo_nv_hid_GetVOLATILE(const EOnv *nv)
{
    if(NULL == nv)
    {
        return(NULL);
    }

    return(nv->loc);
}


extern eOresult_t eo_nv_hid_GetPERMANENT(const EOnv *nv, void *dat, uint16_t *size)
{
    eOresult_t res = eores_NOK_generic;
    eObool_t typisarray = eobool_false;

    if((NULL == nv) || (NULL == dat))
    {
        return(eores_NOK_nullpointer);
    }

    typisarray = (eo_nv_TYP_arr == nv->con->typ) ? eobool_true : eobool_false;

    *size = 0;

    if(eobool_true == eo_nv_hid_isPermanent(nv))
    {
        if((EOK_uint32dummy != nv->usr->stg_address) && (NULL != nv->stg))
        {   // protection is inside EOVstorage
            uint16_t capacity = eo_nv_hid_GetCAPACITY(nv);
            eov_strg_Get(nv->stg, nv->usr->stg_address, capacity, dat);
            *size = (eobool_false == typisarray) ? (capacity) : (2 + *((uint16_t*)dat));
            res = eores_OK;
        }
    }

    return(res);
}



extern uint16_t eo_nv_hid_GetCAPACITY(const EOnv *nv)
{
    if(NULL == nv)
    {
        return(0);
    }

    return(nv->con->capacity);
}


#if defined(EO_NV_EMBED_FUNTYP_IN_ID) 
extern eOnvFunc_t eo_nv_hid_fromIDtoFUN(eOnvID_t id)
{
    return((eOnvFunc_t)eo_nv_getFUNfromID(id));
}  
extern eOnvType_t eo_nv_hid_fromIDtoTYP(eOnvID_t id)
{
    return((eOnvType_t)eo_nv_getTYPfromID(id));
} 
//extern uint16_t eo_nv_fromIDtoOFF(eOnvID_t id)
//{
//    return(eo_nv_getOFF(id));
//}
#endif



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eo_nv_Set(const EOnv *nv, const void *dat, void *dst, eOnvUpdate_t upd)
{
    uint16_t size = 0;
    eObool_t typisarray = eobool_false;

    eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);

    typisarray = (eo_nv_TYP_arr == nv->con->typ) ? (eobool_true) : (eobool_false);

    size = (eobool_false == typisarray) ? (eo_nv_hid_GetCAPACITY(nv)) : (s_eo_nv_array_get_size((void*)dat));


    memcpy(dst, dat, size);


    if(eobool_true == eo_nv_hid_isPermanent(nv)) 
    {
        if((EOK_uint32dummy != nv->usr->stg_address) && (NULL != nv->stg))
        {
            eov_strg_Set(nv->stg, nv->usr->stg_address, size, dat);
        }
    }


    // call the update function if necessary, to propagate the change of data to the peripheral
    if(eo_nv_upd_dontdo != upd)
    {
        if((eo_nv_upd_always == upd) || (eobool_true == eo_nv_hid_isUpdateable(nv))) 
        {
            if((NULL != nv->usr->peripheralinterface) && (NULL != nv->usr->peripheralinterface->update))
            {
                nv->usr->peripheralinterface->update(nv, eok_uint64dummy, eok_uint32dummy);
            }
        }
    }

    eov_mutex_Release(nv->mtx);

    return(eores_OK);
}


static eOresult_t s_eo_nv_SetTS(const EOnv *nv, const void *dat, void *dst, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign)
{
    uint16_t size = 0;
    eObool_t typisarray = eobool_false;

    eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);

    typisarray = (eo_nv_TYP_arr == nv->con->typ) ? (eobool_true) : (eobool_false);

    size = (eobool_false == typisarray) ? (eo_nv_hid_GetCAPACITY(nv)) : (s_eo_nv_array_get_size((void*)dat));


    memcpy(dst, dat, size);

    if(eobool_true == eo_nv_hid_isPermanent(nv)) 
    {
        if((EOK_uint32dummy != nv->usr->stg_address) && (NULL != nv->stg))
        {
            eov_strg_Set(nv->stg, nv->usr->stg_address, size, dat);
        }
    }

    // call the update function if necessary, to propagate the change of data to the peripheral
    if(eo_nv_upd_dontdo != upd)
    {
        if((eo_nv_upd_always == upd) || (eobool_true == eo_nv_hid_isUpdateable(nv))) 
        {
            if((NULL != nv->usr->peripheralinterface) && (NULL != nv->usr->peripheralinterface->update))
            {
                //nv->usr->peripheralinterface->update(nv, time, sign);
                nv->usr->peripheralinterface->update(nv, time, sign);
            }
        }
    }

    eov_mutex_Release(nv->mtx);

    return(eores_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




