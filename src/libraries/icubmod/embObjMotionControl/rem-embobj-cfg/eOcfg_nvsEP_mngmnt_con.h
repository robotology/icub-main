
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVSEP_MNGMNT_CON_H_
#define _EOCFG_NVSEP_MNGMNT_CON_H_




/** @file       eOcfg_nvsEP_mngmnt_con.h
	@brief      This header file gives the constant configuration for the NVs in the base endpoint port
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_uilsdede Configuation of the NVs for management of the ems board
    Tcecece 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOconstvector.h"

#include "EOarray.h"
#include "EOnv.h"
#include "EOrop.h"


// - public #define  --------------------------------------------------------------------------------------------------

#define EOK_cfg_nvsEP_mngmnt                                            (64*1)

#define EOK_cfg_nvsEP_mngmnt_endpoint                                   (1)


// -- the fun and typ of all the nv in the endpoint

#define EOK_cfg_nvsEP_mngmnt_NVFUNTYP__upto10rop2signal                 EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_arr)
#define EOK_cfg_nvsEP_mngmnt_NVFUNTYP__workingmode                      EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_u08)


// -- the identifiers of all the nv in the endpoint

#define EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal                     EO_nv_ID(EOK_cfg_nvsEP_mngmnt_NVFUNTYP__upto10rop2signal,   0+EOK_cfg_nvsEP_mngmnt)
#define EOK_cfg_nvsEP_mngmnt_NVID__workingmode                          EO_nv_ID(EOK_cfg_nvsEP_mngmnt_NVFUNTYP__workingmode,        1+EOK_cfg_nvsEP_mngmnt)

//#define EOK_cfg_nvsEP_mngmnt_NVID__upto15epid2signal                    EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_arr, 0+EOK_cfg_nvsEP_mngmnt)
//#define EOK_cfg_nvsEP_mngmnt_NVID__workingmode                          EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_u08, 1+EOK_cfg_nvsEP_mngmnt)



// - declaration of public user-defined types ------------------------------------------------------------------------- 

//typedef struct              // size is 4+15*4=64 bytes
//{
//    eOarray_head_t          head;
//    uint8_t                 data[15*sizeof(eOnvEPID_t)];
//} EOarray_of_15eOnvEPID;    EO_VERIFYsizeof(EOarray_of_15eOnvEPID, 64);

typedef struct              // size is 4+10*6=64 bytes
{
    eOarray_head_t          head;
    uint8_t                 data[10*sizeof(eOropSIGcfg_t)];
} EOarray_of_10eOropSIGcfg; EO_VERIFYsizeof(EOarray_of_10eOropSIGcfg, 64);

//typedef struct              // size is 4+14*6=88 bytes
//{
//    eOarray_head_t          head;
//    uint8_t                 data[14*sizeof(eOropSIGcfg_t)];
//} EOarray_of_14eOropSIGcfg; EO_VERIFYsizeof(EOarray_of_14eOropSIGcfg, 88);

typedef enum                
{
    mngmnt_workingmode_idle         = 0,
    mngmnt_workingmode_running      = 1
} ep_mngmnt_workingmode_t;

// - very important information: on ARM alignment is done using "#pragma pack(8)", thus ... there are rules to follow

typedef struct              // size is 64+1+7 = 72 bytes
{
    EOarray_of_10eOropSIGcfg    upto10rop2signal;
    uint8_t                     workingmode;        
    uint8_t                     notanv_filler0[7];
} eo_cfg_nvsEP_mngmnt_t;        EO_VERIFYsizeof(eo_cfg_nvsEP_mngmnt_t, 72)

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eo_cfg_nvsEP_mngmnt_t eo_cfg_nvsEP_mngmnt_default;

// EOconstvector where each element is a EOtreenode whose data field is a EOnv_con_t object (id, capacity, valuedef, offset)
extern const EOconstvector* const eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con;

// if not NULL it contains a mapping from IDs to index inside eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con
extern const eOuint16_fp_uint16_t eo_cfg_nvsEP_mngmnt_fnptr_hashfunction_id2index;



// - declaration of extern public functions ---------------------------------------------------------------------------

// same as the eo_cfg_nvsEP_mngmnt_fnptr_hashfunction_id2index pointer  
extern uint16_t eo_cfg_nvsEP_mngmnt_hashfunction_id2index(uint16_t nvid);



/** @}            
    end of group eo_uilsdede  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




