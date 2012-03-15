
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EONV_H_
#define _EONV_H_



/** @file       EOnv.h
    @brief      This header file implements public interface to a netvar.
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_n Objevct EOnv
    Offers methods for reading and writing some data on RAM, ROM and storage and for doing some
    actions upon such operations. It is to be manipulated only via its public methods.
    It is not created but loaded as a constant object from a configuration mapped in ROM
      
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------

// if it is undefined, then the id does not contain anymore information about functionality and type of the netvar,
// pros: there is complete freedom in giving the IDs as the space is a full 2^16
// cons: there is less control upon validity of the rop and it may happen that a badly formed rop which has a wrong size
//       field incoherent with the type ... may cause a crash. but if one uses the EOtheFormer there is no danger
#define EO_NV_EMBED_FUNTYP_IN_ID
//#undef EO_NV_EMBED_FUNTYP_IN_ID

#define EO_nv_FUNTYP(fun, typ)      ((uint8_t)( (((fun)&0x07)<<5) | (((typ)&0x07)<<2) ))

#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
    #define EO_nv_ID(funtyp, off)   ((uint16_t)( ((((uint16_t)(funtyp))<<8)&0xfc00) | (((uint16_t)(off)&0x03ff)<<0) ))
#else
    #define EO_nv_ID(funtyp, off)   ((uint16_t)( ((uint16_t)(off)&0xffff) ))
#endif  

// - declaration of public user-defined types -------------------------------------------------------------------------    


/** @typedef    typedef struct EOnv_hid EOnv
    @brief      EOaction is an opaque struct. It is used to implement data abstraction for the datagram 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOnv_hid EOnv;



typedef uint16_t eOnvID_t;

typedef uint16_t eOnvEP_t;

typedef struct      // 04 bytes
{
    eOnvEP_t        ep;
    eOnvID_t        id;
} eOnvEPID_t;       EO_VERIFYsizeof(eOnvEPID_t, 4);


typedef enum
{
    eo_nv_ownership_local       = 0,
    eo_nv_ownership_remote      = 1
} eOnvOwnership_t; 



typedef enum
{
    eo_nv_strg_volatile     = 0,
    eo_nv_strg_default      = 1,
    eo_nv_strg_permanent    = 2
} eOnvStorage_t;


typedef enum
{
    eo_nv_FUN_NO0               = 0,
    eo_nv_FUN_NO1               = 1,
    eo_nv_FUN_mix               = 2,
    eo_nv_FUN_con               = 3,
    eo_nv_FUN_cfg               = 4,
    eo_nv_FUN_beh               = 5,
    eo_nv_FUN_inp               = 6,
    eo_nv_FUN_out               = 7
} eOnvFunc_t; 


typedef enum
{
    eo_nv_TYP_u08                = 0,
    eo_nv_TYP_u16                = 1,
    eo_nv_TYP_u32                = 2,
    eo_nv_TYP_u64                = 3,
    eo_nv_TYP_NO4                = 4,
    eo_nv_TYP_NO5                = 5,
    eo_nv_TYP_arr                = 6,   // packed as a EOarray object: 2B+1B+1B+[data]
    eo_nv_TYP_pkd                = 7
} eOnvType_t; 



typedef enum
{
    eo_nv_upd_dontdo        = 0,
    eo_nv_upd_ifneeded      = 1,
    eo_nv_upd_always        = 2
} eOnvUpdate_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------





// - declaration of extern public functions ---------------------------------------------------------------------------



extern EOnv* eo_nv_New(void);

extern eOresult_t eo_nv_Clear(EOnv *nv);



extern eOresult_t eo_nv_Set(const EOnv *netvar, const void *dat, eObool_t forceset, eOnvUpdate_t upd);
extern eOresult_t eo_nv_SetTS(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign);

extern eOresult_t eo_nv_remoteSet(const EOnv *netvar, const void *dat, eOnvUpdate_t upd);
extern eOresult_t eo_nv_remoteSetTS(const EOnv *netvar, const void *dat, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign);

extern eOresult_t eo_nv_Reset(const EOnv *netvar, eObool_t forcerst, eOnvUpdate_t upd);
extern eOresult_t eo_nv_ResetTS(const EOnv *netvar, eObool_t forcerst, eOnvUpdate_t upd, eOabstime_t time, uint32_t sign);

extern eOresult_t eo_nv_Get(const EOnv *netvar, eOnvStorage_t strg, void *data, uint16_t *size);

extern eOresult_t eo_nv_remoteGet(const EOnv *netvar, void *data, uint16_t *size);

extern uint16_t eo_nv_Size(const EOnv *netvar, const void *data);

extern uint16_t eo_nv_Capacity(const EOnv *netvar);

extern eOresult_t eo_nv_Init(const EOnv *netvar);

extern eOresult_t eo_nv_Update(const EOnv *netvar);

extern eOresult_t eo_nv_UpdateTS(const EOnv *nv, eOabstime_t roptime, uint32_t ropsign);

extern eOnvID_t eo_nv_GetID(const EOnv *netvar);

extern eOnvFunc_t eo_nv_GetFUN(const EOnv *netvar);

extern eOnvType_t eo_nv_GetTYP(const EOnv *netvar);




/** @}            
    end of group eo_nv 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

