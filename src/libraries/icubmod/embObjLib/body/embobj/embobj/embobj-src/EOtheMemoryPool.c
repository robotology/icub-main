
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "stdio.h"
#include "EoCommon.h"
#include "EOtheErrorManager.h"
#include "EOVmutex.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheMemoryPool.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheMemoryPool_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 // --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOmempool_cfg_t eom_mempool_DefaultCfg = 
{
    EO_INIT(.mode)          eo_mempool_alloc_dynamic,
    EO_INIT(.size08)        0,
    EO_INIT(.data08)        NULL,
    EO_INIT(.size16)        0,
    EO_INIT(.data16)        NULL,    
    EO_INIT(.size32)        0,
    EO_INIT(.data32)        NULL,    
    EO_INIT(.size64)        0,
    EO_INIT(.data64)        NULL
};



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void * s_eo_mempool_get_static(eOmempool_alignment_t alignmode, uint16_t size, uint16_t number);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtheMemoryPool";

static EOtheMemoryPool s_the_mempool = 
{ 
    EO_INIT(.cfg)           NULL, 
    EO_INIT(.mutex)         NULL, 
    EO_INIT(.tout)          0, 
    EO_INIT(.allocmode)     eo_mempool_alloc_dynamic, 
    EO_INIT(.staticmask)    0, 
    EO_INIT(.initted)       0, 
    EO_INIT(.uint08s_num)   0, 
    EO_INIT(.uint16s_num)   0, 
    EO_INIT(.uint32s_num)   0, 
    EO_INIT(.uint64s_num)   0,
    EO_INIT(.usedbytes)     0
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOtheMemoryPool * eo_mempool_Initialise(const eOmempool_cfg_t *cfg)
{
    
    if(0 == s_the_mempool.initted)
    {
        s_the_mempool.initted = 1;
        s_the_mempool.usedbytes = 0;

        // allow initialisation with null cfg .....
        if(NULL == cfg)
        {
            s_the_mempool.cfg = &eom_mempool_DefaultCfg;
            s_the_mempool.allocmode = eo_mempool_alloc_dynamic;
        }
        else
        {
            s_the_mempool.cfg = cfg;
            s_the_mempool.allocmode = cfg->mode; 
    
            if(eo_mempool_alloc_dynamic != s_the_mempool.allocmode)
            {
                if(0 != cfg->size08)
                {
                    s_the_mempool.staticmask |= 1;
                    s_the_mempool.uint08s_num = cfg->size08;
                }
                
                if(0 != cfg->size16)
                {
                    s_the_mempool.staticmask |= 2;
                    s_the_mempool.uint16s_num = cfg->size16/2;
                }   
                
                if(0 != cfg->size32)
                {
                    s_the_mempool.staticmask |= 4;
                    s_the_mempool.uint32s_num = cfg->size32/4;
                } 
               
                if(0 != cfg->size64)
                {
                    s_the_mempool.staticmask |= 8;
                    s_the_mempool.uint64s_num = cfg->size64/8;
                }            
            }
        }
    }

    return(&s_the_mempool);
}
 
    
extern EOtheMemoryPool* eo_mempool_GetHandle(void) 
{
    return((1==s_the_mempool.initted) ? (&s_the_mempool) : (NULL));
}


extern eOresult_t eo_mempool_SetMutex(EOtheMemoryPool *p, EOVmutex *mutex, eOreltime_t tout) 
{ 
    // avoid assigning more than one mutex to the mempool
    if(NULL != s_the_mempool.mutex) 
    {
        return(eores_NOK_generic);
    }
   
    s_the_mempool.mutex = mutex;
    s_the_mempool.tout = tout;
    
    return(eores_OK);
}    


extern void * eo_mempool_GetMemory(EOtheMemoryPool *p, eOmempool_alignment_t alignmode, uint16_t size, uint16_t number)
{
    void *ret = NULL;
    
       
    if((0 == size) || (0 == number)) 
    {
        // manage the ... warning 
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_warning, s_eobj_ownname, "requested zero memory");
        
        return(NULL);
    }

 

    
    // adjust size so that it is a multiple of 1, 2, 4, 8 depending on the alignment
//    size = (size + (uint8_t)alignmode - 1) / ((uint8_t)alignmode);
//    size *= ((uint8_t)alignmode);

    switch(alignmode)
    {   
        case eo_mempool_align_08bit: {} break;
        case eo_mempool_align_16bit: { size++;  size>>=1; size<<=1;} break;
        case eo_mempool_align_32bit: { size+=3; size>>=2; size<<=2;} break;
        case eo_mempool_align_64bit: { size+=7; size>>=3; size<<=3;} break;
    }



    if(NULL == p)
    {
        // manage the ... warning 
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_warning, s_eobj_ownname, "not initialised yet. using calloc()");

        ret = calloc(number, size);

        if(NULL == ret)
        {
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "no anymore memory from heap");
        }

        s_the_mempool.usedbytes += (number*size);
        return(ret);
    }


    // ok, using the singleton
        
    switch(s_the_mempool.allocmode)
    {
        case eo_mempool_alloc_dynamic:
        {
            ret = calloc(number, size);
        } break;
        
        case eo_mempool_alloc_mixed:
        {
        
            if(0 != (p->staticmask & (uint8_t)alignmode))
            {
                // i have a static allocation
                ret = s_eo_mempool_get_static(alignmode, size, number);
            }
            else
            {
                ret = calloc(number, size);
            }

        } break;
        
        case eo_mempool_alloc_static:
        {
            ret = s_eo_mempool_get_static(alignmode, size, number);
        } break;
    
    }
    
    if(NULL == ret)
    {
        // manage the fatal error
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "no memory anymore from heap");
    }
    
    s_the_mempool.usedbytes += (number*size); 
    return(ret);   
}

extern uint32_t eo_mempool_SizeOfAllocated(EOtheMemoryPool *p)
{
    return(s_the_mempool.usedbytes);
}





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void * s_eo_mempool_get_static(eOmempool_alignment_t alignmode, uint16_t size, uint16_t number)
{
    uint16_t numentries = 0;
    void *ret = NULL;

    // attempt to lock mutex if it is null it will return nok_nullpointer, else ok or nok_timeout.
    if(eores_NOK_timeout == eov_mutex_Take(s_the_mempool.mutex, s_the_mempool.tout))
    {
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "mutex not taken from static after ?? micro-sec");
    }
     
    
    switch(alignmode) 
    {
    
        case eo_mempool_align_08bit:
        {  
            numentries = number * size;
            if((uint32_t)(s_the_mempool.uint08s_num + numentries) <= s_the_mempool.cfg->size08) 
            {
                ret = (void*) &s_the_mempool.cfg->data08[s_the_mempool.uint08s_num];
                s_the_mempool.uint08s_num += numentries;
            }

        } break;

        case eo_mempool_align_16bit:
        {
            numentries = number * ((size+1)/2);
            if((uint32_t)(s_the_mempool.uint16s_num + numentries) <= s_the_mempool.cfg->size16) 
            {
                ret = (void*) &s_the_mempool.cfg->data16[s_the_mempool.uint16s_num];
                s_the_mempool.uint16s_num += numentries;
            }

        } break;
        
        case eo_mempool_align_32bit:
        {
            numentries = number * ((size+3)/4);
            if((uint32_t)(s_the_mempool.uint32s_num + numentries) <= s_the_mempool.cfg->size32) 
            {
                ret = (void*) &s_the_mempool.cfg->data32[s_the_mempool.uint32s_num];
                s_the_mempool.uint32s_num += numentries;
            }

        } break;

        case eo_mempool_align_64bit:
        {
            numentries = number * ((size+7)/8);
            if((uint32_t)(s_the_mempool.uint64s_num + numentries) <= s_the_mempool.cfg->size64) 
            {
                ret = (void*) &s_the_mempool.cfg->data64[s_the_mempool.uint64s_num];
                s_the_mempool.uint64s_num += numentries;
            }

        } break;
       
        default:
        {
            ret = NULL;
        } break;    
        
    }
    

    if(NULL == ret) 
    {
        // manage error
        eo_errman_Error(eo_errman_GetHandle(), eo_errortype_fatal, s_eobj_ownname, "no memory anymore from static");
    }
    
    
    // unlock mutex. it is safe even if mutex is null.
    eov_mutex_Release(s_the_mempool.mutex);

    return(ret);
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





