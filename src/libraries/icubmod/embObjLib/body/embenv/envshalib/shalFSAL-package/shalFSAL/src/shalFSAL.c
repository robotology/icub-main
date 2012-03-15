
/* @file       shalFSAL.c
    @brief      This header file implements the shalFSAL library.
    @author     marco.accame@iit.it
    @date       05/07/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "fsal.h"
#include "string.h"

extern void fsal_hid_entrypoint(void);



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "shalFSAL.h"


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
} fsalSignature_t;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

// the ram size to be used in scatter-file and the one used by the program for static ram
#define SHALFSAL_RAMFOR_RWDATA      (EENV_MEMMAP_SHALFSAL_RAMFOR_RWDATA)

// the ram size to be used with __attribute__((at(SHALFSAL_RAMADDR))), which is sizeof(fsalSignature_t)
#define SHALFSAL_RAMFOR_ZIDATA      (EENV_MEMMAP_SHALFSAL_RAMFOR_ZIDATA) //(8)

// and its control
typedef int dummy1[sizeof(fsalSignature_t)     <= ((SHALFSAL_RAMSIZE-SHALFSAL_RAMFOR_RWDATA)) ? 1 : -1];
typedef int dummy2[SHALFSAL_RAMFOR_ZIDATA <= ((SHALFSAL_RAMSIZE-SHALFSAL_RAMFOR_RWDATA)) ? 1 : -1];

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile fsalSignature_t s_shalfsal_ram_signature      __attribute__((at(SHALFSAL_RAMADDR)));


static const eEmoduleInfo_t s_shalfsal_moduleinfo       __attribute__((at(SHALFSAL_ROMADDR+EENV_MODULEINFO_OFFSET))) = 
{
    .type       = ee_sharlib,
    .signature  = SHALFSAL_SIGNATURE,
    .version    = 
    { 
        .major = SHALFSAL_MAJOR, 
        .minor = SHALFSAL_MINOR
    },  
    .builddate  = 
    {
        .byear  = SHALFSAL_BUILDDATE_YEAR,
        .bmonth = SHALFSAL_BUILDDATE_MONTH,
        .bday   = SHALFSAL_BUILDDATE_DAY,
        .bhour  = SHALFSAL_BUILDDATE_HOUR,
        .bmin   = SHALFSAL_BUILDDATE_MIN
    },
    .name       = SHALFSAL_NAME,                  
    .rom        = 
    {   
        .addr   = SHALFSAL_ROMADDR,
        .size   = SHALFSAL_ROMSIZE
    },
    .ram        = 
    {   
        .addr   = SHALFSAL_RAMADDR,
        .size   = SHALFSAL_RAMSIZE
    },
    .storage    = 
    {
        .type   = SHALFSAL_STGTYPE,
        .size   = SHALFSAL_STGSIZE,
        .addr   = SHALFSAL_STGADDR
    }
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern eEresult_t shalfsal_init(uint8_t forceinit)
{
    // this function can be called multiple times by any e-process or e-sharlib and should be executed only once,
    // thus i use this guard ...   
    if(0 == memcmp((void*)&s_shalfsal_ram_signature, (void*)&s_shalfsal_moduleinfo, sizeof(fsalSignature_t)))
    {   // the base signature is ok, thus we have already initted the shalFSAL
        if(0 == forceinit)
        {
            return(ee_res_OK);
        }
    }
    
    // set the base signature
    memcpy((void*)&s_shalfsal_ram_signature, (void*)&s_shalfsal_moduleinfo, sizeof(fsalSignature_t));

    // set to zero the ram beyond the signature
    memset((void*)(SHALFSAL_RAMADDR+SHALFSAL_RAMFOR_ZIDATA), 0, (SHALFSAL_RAMSIZE-SHALFSAL_RAMFOR_ZIDATA));

    // set to proper values the ram that is required to be correct until the call of osal_initialise(), which will
    // set any variable to its proper value


    // ok, return
    return(ee_res_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern void shalfsal_entrypoint(void)
{
    shalfsal_init(0);
    fsal_hid_entrypoint();
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

