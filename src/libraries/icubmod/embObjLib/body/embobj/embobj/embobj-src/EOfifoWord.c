// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOfifo_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOfifoWord.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOfifoWord_hid.h" 


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

static const char s_eobj_ownname[] = "EOfifoWord";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOfifoWord* eo_fifoword_New(eOsizecntnr_t capacity, EOVmutexDerived *mutex) 
{
    EOfifoWord *retptr = NULL; 
    
    // i get memory for a fifobyte. it can never be NULL 
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOfifoWord), 1);

    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity is zero");

    // now i create a mutexfifo made of word (no ctor or dtor!) and i fill it into the fifo of 
    // the fifoword. it can never be NULL 
    retptr->fifo = eo_fifo_New(4, capacity, NULL, 0, NULL, NULL, mutex);

    // ok, done
    return(retptr);
}


extern eOresult_t eo_fifoword_Capacity(EOfifoWord *fifoword, eOsizecntnr_t *capacity, eOreltime_t tout) 
{
    if(NULL == fifoword)
    {
        return(eores_NOK_nullpointer);
    }
    return(eo_fifo_Capacity(fifoword->fifo, capacity, tout));
}


extern eOresult_t eo_fifoword_Size(EOfifoWord *fifoword, eOsizecntnr_t *size, eOreltime_t tout) 
{
    if(NULL == fifoword)
    {
        return(eores_NOK_nullpointer);
    }
    return(eo_fifo_Size(fifoword->fifo, size, tout));
}


extern eOresult_t eo_fifoword_Put(EOfifoWord *fifoword, uint32_t item, eOreltime_t tout) 
{
    // marco.accame: i dont like using the address of a function parameter which is stored in a register.
    // even if it is perfectly safe to do so, i prefer using the address of a variable on the stack.
    uint32_t myitem = item;

    if(NULL == fifoword)
    {
        return(eores_NOK_nullpointer);
    }
    return(eo_fifo_Put(fifoword->fifo, &myitem, tout));
}


extern eOresult_t eo_fifoword_Get(EOfifoWord *fifoword, uint32_t *item, eOreltime_t tout) 
{
    const void *fitem = NULL;
    eOresult_t res = eores_NOK_generic;

    if(NULL == fifoword)
    {
        return(eores_NOK_nullpointer);
    }

    res = eo_fifo_Get(fifoword->fifo, &fitem, tout);

    if(fitem != NULL) 
    {
        *item = *((uint32_t *)fitem); 
    }
    
    return(res);    
}


extern eOresult_t eo_fifoword_Rem(EOfifoWord *fifoword, eOreltime_t tout) 
{
    if(NULL == fifoword)
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eo_fifo_Rem(fifoword->fifo, tout));
}


extern eOresult_t eo_fifoword_Clear(EOfifoWord *fifoword, eOreltime_t tout) 
{
    if(NULL == fifoword)
    {
        return(eores_NOK_nullpointer);
    }
    return(eo_fifo_Clear(fifoword->fifo, tout));
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

