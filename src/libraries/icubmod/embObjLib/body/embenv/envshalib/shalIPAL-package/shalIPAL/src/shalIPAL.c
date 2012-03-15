
/* @file       shalIPAL.c
    @brief      This header file implements the shalIPAL library.
    @author     marco.accame@iit.it
    @date       05/07/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "ipal.h"
#include "string.h"

extern void ipal_hid_entrypoint(void);



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalIPAL.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct                                      // 8B 
{
    eEmoduleType_t          type;                   // 1B
    uint8_t                 signature;              // 1B
    eEversion_t             version;                // 2B
    eEbuilddate_t           builddate;              // 4B
} ipalSignature_t;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALIPAL_RAMFOR_RWDATA      (EENV_MEMMAP_SHALIPAL_RAMFOR_RWDATA)

// the ram size to be used with __attribute__((at(SHALIPAL_RAMADDR))), which is sizeof(ipalSignature_t)
#define SHALIPAL_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALIPAL_RAMFOR_ZIDATA) //(8)

// and its control
typedef int dummy1[sizeof(ipalSignature_t)     <= ((SHALIPAL_RAMSIZE-SHALIPAL_RAMFOR_RWDATA)) ? 1 : -1];
typedef int dummy2[SHALIPAL_RAMFOR_ZIDATA <= ((SHALIPAL_RAMSIZE-SHALIPAL_RAMFOR_RWDATA)) ? 1 : -1];

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile ipalSignature_t s_shalipal_ram_signature      __attribute__((at(SHALIPAL_RAMADDR)));


static const eEmoduleInfo_t s_shalipal_moduleinfo       __attribute__((at(SHALIPAL_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .type       = ee_sharlib,
    .signature  = SHALIPAL_SIGNATURE,
    .version    = 
    { 
        .major = SHALIPAL_MAJOR, 
        .minor = SHALIPAL_MINOR
    },  
    .builddate  = 
    {
        .byear  = SHALIPAL_BUILDDATE_YEAR,
        .bmonth = SHALIPAL_BUILDDATE_MONTH,
        .bday   = SHALIPAL_BUILDDATE_DAY,
        .bhour  = SHALIPAL_BUILDDATE_HOUR,
        .bmin   = SHALIPAL_BUILDDATE_MIN
    },
    .name       = SHALIPAL_NAME,                  
    .rom        = 
    {   
        .addr   = SHALIPAL_ROMADDR,
        .size   = SHALIPAL_ROMSIZE
    },
    .ram        = 
    {   
        .addr   = SHALIPAL_RAMADDR,
        .size   = SHALIPAL_RAMSIZE
    },
    .storage    = 
    {
        .type   = SHALIPAL_STGTYPE,
        .size   = SHALIPAL_STGSIZE,
        .addr   = SHALIPAL_STGADDR
    }
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eEresult_t shalipal_init(uint8_t forceinit)
{
    // this function can be called multiple times by any e-process or e-sharlib and should be executed only once,
    // thus i use this guard ...   
    if(0 == memcmp((void*)&s_shalipal_ram_signature, (void*)&s_shalipal_moduleinfo, sizeof(ipalSignature_t)))
    {   // the base signature is ok, thus we have already initted the shalIPAL
        if(0 == forceinit)
        {
            return(ee_res_OK);
        }
    }
    
    // set the base signature
    memcpy((void*)&s_shalipal_ram_signature, (void*)&s_shalipal_moduleinfo, sizeof(ipalSignature_t));

    // set to zero the ram beyond the signature
    memset((void*)(SHALIPAL_RAMADDR+SHALIPAL_RAMFOR_ZIDATA), 0, (SHALIPAL_RAMSIZE-SHALIPAL_RAMFOR_ZIDATA));

    // set to proper values the ram that is required to be correct until the call of ipal_initialise(), which will
    // set any variable to its proper value
    
    


    // ok, return
    return(ee_res_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void shalipal_entrypoint(void)
{
    shalipal_init(0);
    ipal_hid_entrypoint();
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

