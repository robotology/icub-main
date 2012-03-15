
/* @file       EOropframe.c
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

#include "EOropframe.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOropframe_hid.h" 


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


EO_static_inline EOropframeHeader_t* s_eo_ropframe_header_get(EOropframe *p)
{
    return( (EOropframeHeader_t *)&p->headropsfooter->header );
}

EO_static_inline uint8_t* s_eo_ropframe_rops_get(EOropframe *p)
{
    return( (uint8_t *)(&p->headropsfooter->ropsfooter[0]) );
}

EO_static_inline uint16_t s_eo_ropframe_sizeofrops_get(EOropframe *p)
{
    return( p->headropsfooter->header.ropssizeof );
}

EO_static_inline uint16_t s_eo_ropframe_numberofrops_get(EOropframe *p)
{
    return( p->headropsfooter->header.ropsnumberof );
}

EO_static_inline EOropframeFooter_t* s_eo_ropframe_footer_get(EOropframe *p)
{
    return( (EOropframeFooter_t *)(&p->headropsfooter->ropsfooter[s_eo_ropframe_sizeofrops_get(p)]) );
}


static void s_eo_ropframe_header_addrop(EOropframe *p, uint16_t sizeofrop);
static void s_eo_ropframe_header_remrop(EOropframe *p, uint16_t sizeofrop);
static void s_eo_ropframe_header_addrops(EOropframe *p, uint16_t numofrops, uint16_t sizeofrops);
static void s_eo_ropframe_header_clr(EOropframe *p);
static void s_eo_ropframe_footer_adjust(EOropframe *p);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

//static const char s_eobj_ownname[] = "EOropframe";

static const uint16_t s_eo_ropframe_minimum_framesize = (sizeof(EOropframeHeader_t)+sizeof(EOropframeFooter_t));


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOropframe* eo_ropframe_New(void)
{
    EOropframe *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOropframe), 1);
    
    retptr->capacity            = 0;
    retptr->size                = 0;
    retptr->index               = 0;
    retptr->headropsfooter      = NULL;

    return(retptr);
}


extern eOresult_t eo_ropframe_Load(EOropframe *p, uint8_t *framedata, uint16_t framesize, uint16_t framecapacity)
{
    if((NULL == p) || (NULL == framedata)) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if((framecapacity < s_eo_ropframe_minimum_framesize) || (framesize > framecapacity))
    {
        return(eores_NOK_generic);
    }

    p->capacity     = framecapacity;
    p->size         = framesize;
    p->index        = 0;
    p->headropsfooter      = (EOropframeHeaderRopsFooter_t*)framedata;
    
    return(eores_OK);
}

extern eOresult_t eo_ropframe_Unload(EOropframe *p)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }

    p->capacity            = 0;
    p->size                = 0;
    p->index               = 0;
    p->headropsfooter      = NULL;

    return(eores_OK);
}

extern eOresult_t eo_ropframe_Get(EOropframe *p, uint8_t **framedata, uint16_t* framesize, uint16_t* framecapacity)  
{
    if((NULL == p) || (NULL == framedata) || (NULL == framesize) || (NULL == framecapacity)) 
    {
        return(eores_NOK_nullpointer);
    }

    *framedata          = (uint8_t*)p->headropsfooter;
    *framesize          = p->size;
    *framecapacity      = p->capacity;

    return(eores_OK);
}

extern eOresult_t eo_ropframe_Size_Get(EOropframe *p, uint16_t* framesize)
{
    if((NULL == p) || (NULL == framesize)) 
    {
        return(eores_NOK_nullpointer);
    }

    *framesize          = p->size;

    return(eores_OK);
}


extern eOresult_t eo_ropframe_Clear(EOropframe *p)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }

    
    p->size         = (0 == p->capacity) ? (0) : (s_eo_ropframe_minimum_framesize);
    p->index        = 0;
//    p->currop       = 0;
    
    if(NULL != p->headropsfooter)
    {
        s_eo_ropframe_header_clr(p);    
        s_eo_ropframe_footer_adjust(p);
    }

    return(eores_OK);
}


extern eOresult_t eo_ropframe_Append(EOropframe *p, EOropframe *rfr, uint16_t *remainingbytes)  
{
    uint16_t rfr_sizeofrops;
    uint16_t p_sizeofrops;

// removed because the control vs NULL is done inside _Isvalid() method
//    if((NULL == p) || (NULL == rfr)) 
//    {
//        return(eores_NOK_nullpointer);
//    }

    // both must be valid
    if((eobool_false == eo_ropframe_IsValid(p)) || (eobool_false == eo_ropframe_IsValid(rfr)))
    {
        return(eores_NOK_generic);
    }

    // get the ropstream starting from the end of rops. call the parser

    rfr_sizeofrops = s_eo_ropframe_sizeofrops_get(rfr);
    
    if(0 == rfr_sizeofrops)
    {   // the second ropframe is empty
        return(eores_OK);
    }

    p_sizeofrops = s_eo_ropframe_sizeofrops_get(p);

    // first must be able to accept the rops contained in teh second
    if(p->capacity < (s_eo_ropframe_minimum_framesize+p_sizeofrops+rfr_sizeofrops))
    {
        return(eores_NOK_generic);
    }

    // ok, now i can concatenate ...

    // copy
    memcpy(s_eo_ropframe_rops_get(p)+p_sizeofrops, s_eo_ropframe_rops_get(rfr), rfr_sizeofrops);

    // advance the internal index
    p->index += rfr_sizeofrops;

    // advance the size
    p->size  += rfr_sizeofrops;
    
    // adjust the header
    s_eo_ropframe_header_addrops(p, s_eo_ropframe_numberofrops_get(rfr), rfr_sizeofrops);

    // adjust the footer
    s_eo_ropframe_footer_adjust(p);
    
    // fill the retrun value
    if(NULL != remainingbytes)
    {
        *remainingbytes = p->capacity - s_eo_ropframe_minimum_framesize - s_eo_ropframe_sizeofrops_get(p);
    }


    return(eores_OK);
}

extern eObool_t eo_ropframe_IsValid(EOropframe *p)
{
    EOropframeHeader_t *header;
    EOropframeFooter_t *footer;
    
    if((NULL == p) || (NULL == p->headropsfooter)) 
    {
        return(eobool_false);
    }

    header = s_eo_ropframe_header_get(p);
    footer = s_eo_ropframe_footer_get(p);
    
    if(EOFRAME_START != header->startofframe)
    {
        return(eobool_false);
    }
       
    if(EOFRAME_END != footer->endoframe)
    {
        return(eobool_false);
    }
    
    return(eobool_true);
}

extern uint16_t eo_ropframe_ROP_NumberOf(EOropframe *p)
{
    if(eobool_false == eo_ropframe_IsValid(p))
    {
        return(0);
    }
    else
    {
        return(s_eo_ropframe_numberofrops_get(p));
    }
}

extern uint16_t eo_ropframe_ROP_NumberOf_quickversion(EOropframe *p)
{
    return( p->headropsfooter->header.ropsnumberof );
}

extern eOresult_t eo_ropframe_ROP_Get(EOropframe *p, EOrop *rop, uint16_t *unparsedbytes)
{
    uint16_t consumedbytes = 0;
    eOresult_t res = eores_NOK_generic;
    uint16_t unparsed = 0;
    uint8_t * ropstream = NULL;
    
    if((NULL == p) || (NULL == rop)) 
    {
        return(eores_NOK_nullpointer);
    }
    
    // dont go on if the rops are all retrieved.
    unparsed = s_eo_ropframe_sizeofrops_get(p) - p->index;
    
    if(0 == unparsed)
    {
        // cannot parse anymore ...
        eo_rop_Reset(rop);
        return(eores_NOK_generic);
    }
    
    // get the ropstream starting from p->index. call the parser
    
    ropstream = s_eo_ropframe_rops_get(p);
    ropstream += p->index;
   
    // this function fills the rop only if everything is ok. it returns error if teh packet is not big enough or if keeps non-valid data
    res = eo_parser_GetROP(eo_parser_GetHandle(), ropstream, unparsed, p->fromipaddr, rop, &consumedbytes);
    
    if(eores_OK != res)
    {
        eo_rop_Reset(rop);
        p->index = s_eo_ropframe_sizeofrops_get(p);
        if(NULL != unparsedbytes)
        {
            *unparsedbytes = 0;
        }
        
        return(eores_NOK_generic);
    }
    
    // advance the internal index
    p->index += consumedbytes;
  
    // returns the unspersedbytes number
    if(NULL != unparsedbytes)
    {
        *unparsedbytes = s_eo_ropframe_sizeofrops_get(p) - p->index;
    }

    // then ... returns ok.

    return(eores_OK);
}

extern eOresult_t eo_ropframe_ROP_Set(EOropframe *p, const EOrop *rop, uint16_t* addedinpos, uint16_t* consumedbytes, uint16_t *remainingbytes)
{
    uint16_t remaining = 0;
    uint8_t* ropstream = NULL;
    uint16_t streamsize = 0;
    uint16_t streamindex = 0;
    eOipv4addr_t toipaddr;
    eOresult_t res = eores_NOK_generic;
    
    if((NULL == p) || (NULL == p->headropsfooter) || (NULL == rop)) 
    {
        return(eores_NOK_nullpointer);
    }
     
    // verify that the rop has a valid ropcode ...
    if(eo_ropcode_none == eo_rop_GetROPcode((EOrop*)rop))
    {
        return(eores_NOK_generic);
    }
    
    // verify that we have bytes enough to convert the rop to stream 
  
    streamsize = eo_former_GetSizeOfStream(eo_former_GetHandle(), rop);
    remaining = p->capacity - s_eo_ropframe_minimum_framesize - s_eo_ropframe_sizeofrops_get(p);
    if(remaining < streamsize)
    {   // not enough space in ...
        return(eores_NOK_generic);
    }
  
    // get the ropstream starting from the end of rops. call the parser
    ropstream = s_eo_ropframe_rops_get(p);
    streamindex = s_eo_ropframe_sizeofrops_get(p);
    ropstream += streamindex;
    
    // convert the rop and put it inside the stream;    
    res = eo_former_GetStream(eo_former_GetHandle(), rop, remaining, ropstream, &streamsize, &toipaddr);
 
    if(eores_OK != res)
    {
        // the rop is incorrect or it simply contains too much data to be fit inside the stream ...
        return(eores_NOK_generic);
    }
    
    // advance the internal index
    p->index += streamsize;

    // advance the size
    p->size  += streamsize;
    
    // adjust the header
    s_eo_ropframe_header_addrop(p, streamsize);

    // adjust the footer
    s_eo_ropframe_footer_adjust(p);


    if(NULL != addedinpos)
    {
        *addedinpos = streamindex;
    }

    if(NULL != consumedbytes)
    {
        *consumedbytes = streamsize;
    }
        
    if(NULL != remainingbytes)
    {
        *remainingbytes = p->capacity - s_eo_ropframe_minimum_framesize - s_eo_ropframe_sizeofrops_get(p);
    }
    
    // ... returns ok

    return(eores_OK);
}


extern eOresult_t eo_ropframe_age_Set(EOropframe *p, eOabstime_t age)
{
    EOropframeHeader_t* header = NULL;
    
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }

    header = s_eo_ropframe_header_get(p);

    header->ageofframe = age;
 
    return(eores_OK);
}

extern eOabstime_t eo_ropframe_age_Get(EOropframe *p)
{
    EOropframeHeader_t* header = NULL;
    
    if(NULL == p) 
    {
        return(eok_abstimeNOW);
    }

    header = s_eo_ropframe_header_get(p);

    return(header->ageofframe);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

uint8_t* eo_ropframe_hid_get_pointer_offset(EOropframe *p, uint16_t offset)
{
    if(NULL == p)
    {
        return(NULL);
    }

    return(s_eo_ropframe_rops_get(p) + offset);
}


extern eOresult_t eo_ropframe_hid_rop_rem(EOropframe *p, uint16_t startat, uint16_t size)
{
    int16_t tmp = 0;

    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }


    // move memory
    tmp = s_eo_ropframe_sizeofrops_get(p)-startat-size; // howmanymove
    if(tmp > 0)
    {
        memmove(s_eo_ropframe_rops_get(p)+startat, s_eo_ropframe_rops_get(p)+startat+size, tmp);
    }

    // decrement the internal index
    p->index -= size;

    // decrement the size
    p->size  -= size;
    
    // adjust the header
    s_eo_ropframe_header_remrop(p, size);

    // adjust the footer
    s_eo_ropframe_footer_adjust(p);

    // clear what stays beyond footer: instead or remaining i clear only what was non-zero (size)
    //tmp = p->capacity - s_eo_ropframe_minimum_framesize - s_eo_ropframe_sizeofrops_get(p);  // remaining
    memset(((uint8_t*)s_eo_ropframe_footer_get(p))+sizeof(EOropframeFooter_t), 0, size);

    return(eores_OK);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eo_ropframe_header_addrop(EOropframe *p, uint16_t sizeofrop)
{
    EOropframeHeader_t* header = s_eo_ropframe_header_get(p);
    
    header->ropssizeof              += sizeofrop;
    header->ropsnumberof            += 1;
}


static void s_eo_ropframe_header_remrop(EOropframe *p, uint16_t sizeofrop)
{
    EOropframeHeader_t* header = s_eo_ropframe_header_get(p);
    
    header->ropssizeof              -= sizeofrop;
    header->ropsnumberof            -= 1;
}

static void s_eo_ropframe_header_addrops(EOropframe *p, uint16_t numofrops, uint16_t sizeofrops)
{
    EOropframeHeader_t* header = s_eo_ropframe_header_get(p);
    
    header->ropssizeof              += sizeofrops;
    header->ropsnumberof            += numofrops;
}

static void s_eo_ropframe_header_clr(EOropframe *p)
{
    EOropframeHeader_t* header = s_eo_ropframe_header_get(p);
    
    header->startofframe            = EOFRAME_START;
    header->ropssizeof              = 0;
    header->ropsnumberof            = 0;
    header->ageofframe              = 0;
}
    
static void s_eo_ropframe_footer_adjust(EOropframe *p)
{
    EOropframeFooter_t* footer = s_eo_ropframe_footer_get(p);
    
    footer->endoframe               = EOFRAME_END;
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




