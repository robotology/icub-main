
/* @file       shalHAL.c
    @brief      This header file implements the shalHAL library.
    @author     marco.accame@iit.it
    @date       05/07/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "hal.h"
#include "string.h"

extern void hal_hid_link_to_all_files(void);
extern void hal_hid_ram_basic_init(void);



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalHAL.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

#warning removed HAL_USE_DISPLAY to fit shalHAL into 16K



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct                                      // 8B 
{
    eEmoduleType_t          type;                   // 1B
    uint8_t                 signature;              // 1B
    eEversion_t             version;                // 2B
    eEbuilddate_t           builddate;              // 4B
} halSignature_t;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALHAL_RAMFOR_RWDATA      (EENV_MEMMAP_SHALHAL_RAMFOR_RWDATA) //(256-8)

// the ram size to be used with __attribute__((at(SHALHAL_RAMADDR))), which is sizeof(halSignature_t)
#define SHALHAL_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALHAL_RAMFOR_ZIDATA) //(8)

// and its control
typedef int dummy1[sizeof(halSignature_t)     <= ((SHALHAL_RAMSIZE-SHALHAL_RAMFOR_RWDATA)) ? 1 : -1];
typedef int dummy2[SHALHAL_RAMFOR_ZIDATA <= ((SHALHAL_RAMSIZE-SHALHAL_RAMFOR_RWDATA)) ? 1 : -1];

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile halSignature_t s_shalhal_ram_signature      __attribute__((at(SHALHAL_RAMADDR)));


static const eEmoduleInfo_t s_shalhal_moduleinfo       __attribute__((at(SHALHAL_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .type       = ee_sharlib,
    .signature  = SHALHAL_SIGNATURE,
    .version    = 
    { 
        .major = SHALHAL_MAJOR, 
        .minor = SHALHAL_MINOR
    },  
    .builddate  = 
    {
        .byear  = SHALHAL_BUILDDATE_YEAR,
        .bmonth = SHALHAL_BUILDDATE_MONTH,
        .bday   = SHALHAL_BUILDDATE_DAY,
        .bhour  = SHALHAL_BUILDDATE_HOUR,
        .bmin   = SHALHAL_BUILDDATE_MIN
    },
    .name       = SHALHAL_NAME,                  
    .rom        = 
    {   
        .addr   = SHALHAL_ROMADDR,
        .size   = SHALHAL_ROMSIZE
    },
    .ram        = 
    {   
        .addr   = SHALHAL_RAMADDR,
        .size   = SHALHAL_RAMSIZE
    },
    .storage    = 
    {
        .type   = SHALHAL_STGTYPE,
        .size   = SHALHAL_STGSIZE,
        .addr   = SHALHAL_STGADDR
    }
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eEresult_t shalhal_init(uint8_t forceinit)
{
    // this function can be called multiple times by any e-process or e-sharlib and should be executed only once,
    // thus i use this guard ...   
    if(0 == memcmp((void*)&s_shalhal_ram_signature, (void*)&s_shalhal_moduleinfo, sizeof(halSignature_t)))
    {   // the base signature is ok, thus we have already initted the shalHAL
        if(0 == forceinit)
        {
            return(ee_res_OK);
        }
    }
    
    // set the base signature
    memcpy((void*)&s_shalhal_ram_signature, (void*)&s_shalhal_moduleinfo, sizeof(halSignature_t));

    // set to zero the ram beyond the signature
    memset((void*)(SHALHAL_RAMADDR+SHALHAL_RAMFOR_ZIDATA), 0, (SHALHAL_RAMSIZE-SHALHAL_RAMFOR_ZIDATA));

    // set to proper values the ram that is required to be correct until the call of hal_initialise(), which will
    // set any variable to its proper value
    hal_hid_ram_basic_init();

    // ok, return
    return(ee_res_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void shalhal_entrypoint(void)
{
    shalhal_init(0);
    hal_hid_link_to_all_files();
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

