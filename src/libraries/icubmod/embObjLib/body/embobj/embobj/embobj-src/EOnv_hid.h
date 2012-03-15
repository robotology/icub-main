
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EONV_HID_H_
#define _EONV_HID_H_


/* @file       EOnv_hid.h
    @brief      This header file implements hidden interface to a netvar object.
    @author     marco.accame@iit.it
    @date       09/06/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOrop.h"
#include "EOVmutex.h"
#include "EOVstorage.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOnv.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

#define EO_nv_FUN(funtyp)           ((uint8_t)(  (((uint8_t)(funtyp))>>5) & 0x07) )
#define EO_nv_TYP(funtyp)           ((uint8_t)(  (((uint8_t)(funtyp))>>2) & 0x07) )



#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
    #define eo_nv_getFUNfromID(id)        ((uint8_t) (((id)>>13)&0x0007) )
    #define eo_nv_getTYPfromID(id)        ((uint8_t) (((id)>>10)&0x0007) )
    #define eo_nv_getOFFfromID(id)        ((uint16_t) (((id)>>0)&0x03ff) )
#else
    #define eo_nv_getOFFfromID(id)        ((uint16_t) (((id)>>0)&0xffff) )
#endif

#define EONV_NOVALUEPER     EOK_uint32dummy
#define EONV_NOPTRREC       NULL



// - definition of the hidden struct implementing the object ----------------------------------------------------------



typedef     void        (*eOvoid_fp_cnvp_t)                         (const EOnv *);
typedef     void        (*eOvoid_fp_cnvp_cabstime_t)                 (const EOnv *, const eOabstime_t);
typedef     void        (*eOvoid_fp_cnvp_cabstime_cuint32_t)         (const EOnv *, const eOabstime_t, const uint32_t);

typedef struct                     
{
    eOvoid_fp_cnvp_cabstime_cuint32_t   bef;
    eOvoid_fp_cnvp_cabstime_cuint32_t   aft;
} eOnv_fn_onrop_2fn_t;

typedef struct                      
{
    eOnv_fn_onrop_2fn_t             ask;        // used by a simple node on reception of a query sent by a smart node
    eOnv_fn_onrop_2fn_t             set;        // used by a simple node on reception of a set from a smart node
    eOnv_fn_onrop_2fn_t             rst;        // used by a simple node on reception of a rst from a smart node
    eOnv_fn_onrop_2fn_t             upd;        // used by a simple node on reception of a upd from a smart node
} eOnv_fn_onrop_rx_loc_t;

typedef struct                      
{
    eOnv_fn_onrop_2fn_t             say;        // used by a smart node on reception of a reply of a query previously sent
    eOnv_fn_onrop_2fn_t             sig;        // used by a smart node on reception of a spontaneous signal from a node
} eOnv_fn_onrop_rx_rem_t;

typedef union                      
{   // either local or remote
    eOnv_fn_onrop_rx_loc_t          loc;        // set of functions used by a node on rop receptions operated on it local variables
    eOnv_fn_onrop_rx_rem_t          rem;        // set of functions used by a node rop receptions operated on it remote variables
} eOnv_fn_onrop_rx_t;



typedef struct                      
{
    eOvoid_fp_cnvp_t                    init;       // called at startup to init the link between the input or output netvar and the peripheral
    eOvoid_fp_cnvp_cabstime_cuint32_t   update;     // used to propagate the value of the netvar towards the peripheral (if out)
                                                    // or to place the value of the peripheral into the netvar (if inp)
} eOnv_fn_peripheral_t;


typedef const struct                // 12 bytes on arm
{
    eOnvID_t                        id;
    uint16_t                        capacity;
    const void*                     resetval; 
    uint16_t                        offset;   
    uint8_t                         typ;
    uint8_t                         fun;
 } EOnv_con_t;                       EO_VERIFYsizeof(EOnv_con_t, 12); 

typedef const struct
{
    const eOnv_fn_peripheral_t*     peripheralinterface;
    const eOnv_fn_onrop_rx_t*       on_rop_reception;
    const uint32_t                  stg_address;           
} EOnv_usr_t; 


/** @struct     EOnv_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/
struct EOnv_hid 
{
    EOnv_con_t*                     con;        // pointer to the constant part common to every device which uses this nv
    EOnv_usr_t*                     usr;        // pointer to the configurable part specific to each device which uses this nv
    void*                           loc;        // the volatile part which keeps LOCAL value of nv 
    void*                           rem;        // the volatile part which keeps REMOTE value of nv, when signalled or said
    EOVmutexDerived*                mtx;        // the mutex which protects concurrent access to this nv 
    EOVstorageDerived*              stg;
};   
 



// - declaration of extern hidden functions ---------------------------------------------------------------------------

///** @fn         extern EOnv * eo_nv_hid_New(uint8_t fun, uint8_t typ, uint32_t otherthingsmaybe)
//    @brief      Creates a new netvar object. 
//    @return     The pointer to the required object.
// **/
//extern EOnv * eo_nv_hid_New(uint8_t fun, uint8_t typ, uint32_t otherthingsmaybe);



extern eObool_t eo_nv_hid_OnBefore_ROP(const EOnv *nv, eOropcode_t ropcode, eOabstime_t roptime, uint32_t ropsign);

extern eObool_t eo_nv_hid_OnAfter_ROP(const EOnv *nv, eOropcode_t ropcode, eOabstime_t roptime, uint32_t ropsign);


extern eObool_t eo_nv_hid_isWritable(const EOnv *netvar);

extern eObool_t eo_nv_hid_isLocal(const EOnv *netvar);

extern eObool_t eo_nv_hid_isPermanent(const EOnv *netvar);

extern eObool_t eo_nv_hid_isUpdateable(const EOnv *netvar);


extern const void* eo_nv_hid_GetDEFAULT(const EOnv *netvar);

extern void* eo_nv_hid_GetVOLATILE(const EOnv *netvar);

extern void * eo_nv_hid_GetMIRROR(const EOnv *netvar);



extern eOresult_t eo_nv_hid_GetPERMANENT(const EOnv *netvar, void *dat, uint16_t *size);

extern uint16_t eo_nv_hid_GetCAPACITY(const EOnv *netvar);



#if defined(EO_NV_EMBED_FUNTYP_IN_ID) 
    extern eOnvFunc_t eo_nv_hid_fromIDtoFUN(eOnvID_t id);
    extern eOnvType_t eo_nv_hid_fromIDtoTYP(eOnvID_t id);
    //extern uint16_t eo_nv_hid_fromIDtoOFF(eOnvID_t id);
#endif

 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




