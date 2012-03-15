
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVPORT_H_
#define _EOVPORT_H_


/** @file       EOvport.h
    @brief      This header file implements public interface to a remote oepration rop.
    @author     marco.accame@iit.it
    @date       09/03/2010
**/

/** @defgroup eo_vport Object EOvport
    The EOvport object is used to .....
     
    @{        
 **/

#error --> dont use it yet

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------

#define EOVPORT_MAX_NVS			            (8)			// or how many you want
#define EOVPORT_MAXSIZE_NVIDSIZEINFO		(96)        // or how many you want


// - declaration of public user-defined types -------------------------------------------------------------------------    



/** @typedef    typedef struct EOvport_hid EOvport
    @brief      EOaction is an opaque struct. It is used to implement data abstraction for the 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef __packed struct EOvport_hid EOvport;
typedef __packed struct EOvportCfg_hid EOvportCfg;
typedef __packed struct EOvportDat_hid EOvportDat;


__packed struct EOvportCfg_hid 
{
    uint16_t        sizeofarray;                    // number of bytes: equal to 2 (for n) + 2*n
	uint16_t		n;			                    // actual number inside nvids[]
	uint16_t		nvids[EOVPORT_MAX_NVS];		    // container for nvids
};


__packed struct EOvportDat_hid 
{
    uint16_t        sizeofarray;                    // number of bytes: equal to 2 (for n) + 2*n
    uint16_t		n;			                    // actual number of {nvid, size, info}
	uint8_t		    buffer_of_nvidsizeinfo[EOVPORT_MAXSIZE_NVIDSIZEINFO];	// container for {nvid, size, info}
};

/** @struct     EOvport_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/
__packed struct EOvport_hid 
{
    EOvportCfg      cfg;
    EOvportDat      dat;
    void*           mirrors[EOVPORT_MAX_NVS];
};   


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 

/** @fn         extern EOvportCfg * eo_vportcfg_New(void)
    @brief      Creates a new vport cfg object. 
    @return     The pointer to the required object.
 **/
extern EOvportCfg * eo_vportcfg_New(void);

extern eOresult_t eo_vportcfg_Clear(EOvportCfg *p);

extern eOresult_t eo_vportcfg_PushBack(EOvportCfg *p, eOnetvarID_t nv);

extern void * eo_vportcfg_GetMemory(EOvportCfg *p, uint16_t *size);



/** @}            
    end of group eo_vport 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

