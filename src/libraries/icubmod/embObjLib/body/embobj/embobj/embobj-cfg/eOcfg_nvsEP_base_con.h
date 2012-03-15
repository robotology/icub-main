
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVSEP_BASE_CON_H_
#define _EOCFG_NVSEP_BASE_CON_H_




/** @file       eOcfg_nvsEP_base_con.h
	@brief      This header file gives the constant configuration for the NVs in the base endpoint port
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_asfdgr Configuation of the NVs for the updater
    Tcecece 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOconstvector.h"

#include "eEcommon.h"
#include "shalINFO.h"
#include "EOarray.h"
#include "EOnv.h"



// - public #define  --------------------------------------------------------------------------------------------------

#define EOK_cfg_nvsEP_base                                          (64*0)

#define EOK_cfg_nvsEP_base_endpoint                                 (0)


// -- the fun and typ of all the nv in the endpoint

#define EOK_cfg_nvsEP_base_NVFUNTYP__boardinfo                      EO_nv_FUNTYP(eo_nv_FUN_con, eo_nv_TYP_pkd)
#define EOK_cfg_nvsEP_base_NVFUNTYP__applicationinfo                EO_nv_FUNTYP(eo_nv_FUN_con, eo_nv_TYP_pkd)
#define EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork                       EO_nv_FUNTYP(eo_nv_FUN_cfg, eo_nv_TYP_pkd)
#define EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__macaddress           EO_nv_FUNTYP(eo_nv_FUN_cfg, eo_nv_TYP_u64)
#define EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipaddress            EO_nv_FUNTYP(eo_nv_FUN_cfg, eo_nv_TYP_u32)
#define EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipnetmask            EO_nv_FUNTYP(eo_nv_FUN_cfg, eo_nv_TYP_u32)
#define EOK_cfg_nvsEP_base_NVFUNTYP__bootprocess                    EO_nv_FUNTYP(eo_nv_FUN_cfg, eo_nv_TYP_u08)
#define EOK_cfg_nvsEP_base_NVFUNTYP__gotoprocess                    EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_u08)
#define EOK_cfg_nvsEP_base_NVFUNTYP__forcerestart                   EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_u08)
#define EOK_cfg_nvsEP_base_NVFUNTYP__localise                       EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_u08)
#define EOK_cfg_nvsEP_base_NVFUNTYP__remoteipaddress                EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_u32)
#define EOK_cfg_nvsEP_base_NVFUNTYP__remoteipport                   EO_nv_FUNTYP(eo_nv_FUN_beh, eo_nv_TYP_u16)

// -- the identifiers of all the nv in the endpoint

#define EOK_cfg_nvsEP_base_NVID__boardinfo                          EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__boardinfo,            0+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__applicationinfo                    EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__applicationinfo,      1+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID_ipnetwork                           EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork,             2+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID_ipnetwork__macaddress               EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__macaddress, 3+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress                EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipaddress,  4+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask                EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipnetmask,  5+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__bootprocess                        EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__bootprocess,          6+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__gotoprocess                        EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__gotoprocess,          7+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__forcerestart                       EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__forcerestart,         8+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__localise                           EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__localise,             9+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__remoteipaddress                    EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__remoteipaddress,      10+EOK_cfg_nvsEP_base)
#define EOK_cfg_nvsEP_base_NVID__remoteipport                       EO_nv_ID(EOK_cfg_nvsEP_base_NVFUNTYP__remoteipport,         11+EOK_cfg_nvsEP_base)


//#define EOK_cfg_nvsEP_base_NVID__boardinfo                          EO_nv_getID(eo_nv_FUN_con, eo_nv_TYP_pkd, 0+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__applicationinfo                    EO_nv_getID(eo_nv_FUN_con, eo_nv_TYP_pkd, 1+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID_ipnetwork                           EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_pkd, 2+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID_ipnetwork__macaddress               EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u64, 3+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress                EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 4+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask                EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u32, 5+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__bootprocess                        EO_nv_getID(eo_nv_FUN_cfg, eo_nv_TYP_u08, 6+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__gotoprocess                        EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_u08, 7+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__forcerestart                       EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_u08, 8+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__localise                           EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_u08, 9+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__remoteipaddress                    EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_u32, 10+EOK_cfg_nvsEP_base)
//#define EOK_cfg_nvsEP_base_NVID__remoteipport                       EO_nv_getID(eo_nv_FUN_beh, eo_nv_TYP_u16, 11+EOK_cfg_nvsEP_base)


// - declaration of public user-defined types ------------------------------------------------------------------------- 



// - very important information: on ARM alignment is done using "#pragma pack(8)", thus ... there are rules to follow

typedef struct                  // 128 = 64+64+16+1+1+1+1+4+2+6
{
	eEboardInfo_t               boardinfo;
	eEmoduleInfo_t              applicationinfo;
    eEipnetwork_t               ipnetwork;
    eEprocess_t                 bootprocess;
    eEprocess_t                 gotoprocess;
    eObool_t                    forcerestart;
    eObool_t                    localise;
    eOipv4addr_t                remoteipaddress;
    eOipv4port_t                remoteipport;
    uint8_t                     notanv_filler0[6];
} eo_cfg_nvsEP_base_t;          EO_VERIFYsizeof(eo_cfg_nvsEP_base_t, 160);

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eo_cfg_nvsEP_base_t eo_cfg_nvsEP_base_default;

// EOconstvector where each element is a EOtreenode whose data field is a EOnv_con_t object (id, capacity, valuedef, offset)
extern const EOconstvector* const eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con;

// if not NULL it contains a mapping from IDs to index inside eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con
extern const eOuint16_fp_uint16_t eo_cfg_nvsEP_base_fptr_hashfunction_id2index;



// - declaration of extern public functions ---------------------------------------------------------------------------

extern uint16_t eo_cfg_nvsEP_base_hashfunction_id2index(uint16_t id);





/** @}            
    end of group eo_asfdgr  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




