
/* @file       shalOSAL.c
    @brief      This header file implements the shalOSAL library.
    @author     marco.accame@iit.it
    @date       05/07/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "osal.h"
#include "string.h"

extern void osal_hid_reset_static_ram(void);
extern void osal_hid_entrypoint(void);



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalOSAL.h"


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
} osalSignature_t;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALOSAL_RAMFOR_RWDATA      (EENV_MEMMAP_SHALOSAL_RAMFOR_RWDATA)

// the ram size to be used with __attribute__((at(SHALOSAL_RAMADDR))), which is sizeof(osalSignature_t)
#define SHALOSAL_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALOSAL_RAMFOR_ZIDATA) //(8)

// and its control
typedef int dummy1[sizeof(osalSignature_t)     <= ((SHALOSAL_RAMSIZE-SHALOSAL_RAMFOR_RWDATA)) ? 1 : -1];
typedef int dummy2[SHALOSAL_RAMFOR_ZIDATA <= ((SHALOSAL_RAMSIZE-SHALOSAL_RAMFOR_RWDATA)) ? 1 : -1];

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile osalSignature_t s_shalosal_ram_signature      __attribute__((at(SHALOSAL_RAMADDR)));


static const eEmoduleInfo_t s_shalosal_moduleinfo       __attribute__((at(SHALOSAL_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .type       = ee_sharlib,
    .signature  = SHALOSAL_SIGNATURE,
    .version    = 
    { 
        .major = SHALOSAL_MAJOR, 
        .minor = SHALOSAL_MINOR
    },  
    .builddate  = 
    {
        .byear  = SHALOSAL_BUILDDATE_YEAR,
        .bmonth = SHALOSAL_BUILDDATE_MONTH,
        .bday   = SHALOSAL_BUILDDATE_DAY,
        .bhour  = SHALOSAL_BUILDDATE_HOUR,
        .bmin   = SHALOSAL_BUILDDATE_MIN
    },
    .name       = SHALOSAL_NAME,                  
    .rom        = 
    {   
        .addr   = SHALOSAL_ROMADDR,
        .size   = SHALOSAL_ROMSIZE
    },
    .ram        = 
    {   
        .addr   = SHALOSAL_RAMADDR,
        .size   = SHALOSAL_RAMSIZE
    },
    .storage    = 
    {
        .type   = SHALOSAL_STGTYPE,
        .size   = SHALOSAL_STGSIZE,
        .addr   = SHALOSAL_STGADDR
    }
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eEresult_t shalosal_init(uint8_t forceinit)
{
    // this function can be called multiple times by any e-process or e-sharlib and should be executed only once,
    // thus i use this guard ...   
    if(0 == memcmp((void*)&s_shalosal_ram_signature, (void*)&s_shalosal_moduleinfo, sizeof(osalSignature_t)))
    {   // the base signature is ok, thus we have already initted the shalHAL
        if(0 == forceinit)
        {
            return(ee_res_OK);
        }
    }
    
    // set the base signature
    memcpy((void*)&s_shalosal_ram_signature, (void*)&s_shalosal_moduleinfo, sizeof(osalSignature_t));

    // set to zero the ram beyond the signature
    memset((void*)(SHALOSAL_RAMADDR+SHALOSAL_RAMFOR_ZIDATA), 0, (SHALOSAL_RAMSIZE-SHALOSAL_RAMFOR_ZIDATA));

    // set to proper values the ram that is required to be correct until the call of osal_initialise(), which will
    // set any variable to its proper value
    osal_hid_reset_static_ram();


    // ok, return
    return(ee_res_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void shalosal_entrypoint(void)
{
    shalosal_init(0);
    osal_hid_entrypoint();
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

