
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEMEMORYPOOL_H_
#define _EOTHEMEMORYPOOL_H_

/** @file       EOtheMemoryPool.h
	@brief      This header file contains public interfaces for the memory pool singleton object.
	@author     marco.accame@iit.it
	@date       08/03/2011
**/


/** @defgroup eo_thememorypool Object EOtheMemoryPool
    The EOtheMemoryPool is a memory manager for the embOBJ. It is a singleton which can be initialised to work
    with the heap (calloc, then), with user-defined static memory, or with a mixture of them.
    If not initialised, the EOtheMemoryPool gives memory using the heap. It is responsibility of the object EOVtheSystem
    (via its derived object) to initialise the EOtheMemoryPool. 

    As on the the MDK ARM environment, as malloc is thread safe, the function eo_mempool_GetMemory() is  
    guaranteed to be thread-safe when the EOtheMemoryPool uses the heap. Instead, for static or mixed allocation
    modes, the EOtheMemoryPool requires a mutex EOVmutex to protect vs concurrent access. 

    
    @{		
 */
 

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/**	@typedef    EOtheMemoryPool_hid EOtheMemoryPool
 	@brief 		EOtheMemoryPool is an opaque struct. It is used to implement data abstraction for the memory pool
                object so that the user cannot see its private fields so that he/she is forced to manipulate the 
                object only with the proper public functions. 
 **/  
typedef struct EOtheMemoryPool_hid EOtheMemoryPool;


/**	@typedef    typedef enum eOmempool_alignment_t 
 	@brief      Contains the alignment types for the memory. 
 **/  
typedef enum 
{
    eo_mempool_align_08bit  = 1,             
    eo_mempool_align_16bit  = 2,
    eo_mempool_align_32bit  = 4,
    eo_mempool_align_64bit  = 8
} eOmempool_alignment_t;


/**	@typedef    typedef enum eOmempool_allocmode_t 
 	@brief      Contains the allocation mode for the memory. 
 **/ 
typedef enum
{
    eo_mempool_alloc_static     = 0,
    eo_mempool_alloc_dynamic    = 1,
    eo_mempool_alloc_mixed      = 2
} eOmempool_allocmode_t;


/**	@typedef    typedef struct eOmempool_cfg_t 
 	@brief      Contains the configuration for the EOtheMemoryPool. 
 **/ 
typedef struct
{
    eOmempool_allocmode_t       mode;
    uint32_t                    size08;
    uint8_t                     *data08;
    uint32_t                    size16;
    uint16_t                    *data16;    
    uint32_t                    size32;
    uint32_t                    *data32;    
    uint32_t                    size64;
    uint64_t                    *data64;    
} eOmempool_cfg_t;
   
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOmempool_cfg_t eom_mempool_DefaultCfg; // = {eo_mempool_alloc_dynamic, 0, NULL, 0, NULL, 0, NULL, 0, NULL};


// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern EOtheMemoryPool * eo_mempool_Initialise(const eOmempool_cfg_t *cfg)
    @brief      Initialise the singleton EOtheMemoryPool. 
    @param      cfg             It specifies the working mode of the allocator. In case of cfg->mode equal to
                                eo_mempool_alloc_static the memory is assigned using user-defined memory pools
                                aligned at 1, 2, 4, and 8 bytes. In such a case it is necessary to fill
                                the pointers and the size of the four user-defined memory pools.
                                In case of cfg->mode equal to eo_mempool_alloc_mixed the memory is assigned
                                both from the memory pools but it is also used the heap for those memory pools with NULL
                                pointer and zero size.
                                In case of cfg->mode equal to eo_mempool_alloc_dynamic the memory is assigned
                                only from the heap, thus other fields of cfg are not considered.
                                A NULL value for cfg is a shortcut for the mode eo_mempool_alloc_dynamic.
    @return     Pointer to the required EOtheMemoryPool singleton (or NULL upon un-initialised singleton).
 **/
extern EOtheMemoryPool * eo_mempool_Initialise(const eOmempool_cfg_t *cfg);


/** @fn         extern EOtheMemoryPool* eo_mempool_GetHandle(void)
    @brief      Returns a handle to the singleton EOtheMemoryPool. The singleton must have been initialised
                with mempool_Initialise(), otherwise this function call will return NULL.
    @return     Pointer to the required EOtheMemoryPool singleton (or NULL upon un-initialised singleton).
 **/
extern EOtheMemoryPool * eo_mempool_GetHandle(void);


/** @fn         extern eOresult_t eo_mempool_SetMutex(EOtheMemoryPool *p, EOVmutex *mutex, eOreltime_t tout
    @brief      Sets an internal mutex to protect concurrent access to the memory pool. In the multi-task execution
                environment there is no need to use the mutex if we use the heap on the MDK-ARM environment. 
                In a single-task execution environment if the EOtheMemoryPool is not used
                within an ISR (sic!), there is no need to pass a mutex.
    @param      mutex           The mutex.
    @param      tout            The waiting timeout
    @return     eores_OK upon succesful assignment of the passed mutex, eores_NOK_generic if a non-NULL mutex 
                was previously assigned.
    @warning    It is better to call this function before the start of the MEE.
 **/
extern eOresult_t eo_mempool_SetMutex(EOtheMemoryPool *p, EOVmutex *mutex, eOreltime_t tout);

 
/** @fn         extern void * eo_mempool_GetMemory(EOtheMemoryPool *p, eOmempool_alignment_t alignmode, 
                                                   uint16_t size, uint16_t number)
    @brief      Gives back memory for @e number consecutive objects each of @e size bytes and with an alignment
                given by @e alignmmode. If no memory is available the function directly calls the error manager.
    @param      p               The mempool singleton                
    @param      alignmode       The alignment required for the object (08-, 16-, 32-, 64-bit).
    @param      size            The size of the object in bytes. Use sizeof(EoMyObject).
    @param      number          The number of consecutive objects.     
    @return     The required memory if available. NULL if the requedsted memory was zero but with a warning given
                to the EOtheErrorManager. Issues a fatal error to the EOtheErrorManager if there was not memory anymore. 
    @warning    This function is thread-safe in static or mixed mode only if the EOtheMemoryPool has been protected 
                by a proper mutex.
 **/ 
extern void * eo_mempool_GetMemory(EOtheMemoryPool *p, eOmempool_alignment_t alignmode, uint16_t size, uint16_t number);


extern uint32_t eo_mempool_SizeOfAllocated(EOtheMemoryPool *p);

/** @}            
    end of group eo_thememorypool  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

