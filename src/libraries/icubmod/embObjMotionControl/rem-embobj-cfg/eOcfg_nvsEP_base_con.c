
/* @file       eOcfg_nvsEP_base_con.c
    @brief      This file keeps constant configuration for the NVs of the updater
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"

#include "EoCommon.h"


#include "EOnv_hid.h"
#include "EOtreenode_hid.h"


#include "EOconstvector_hid.h"

#include "eEcommon.h"
#include "shalPART.h"
#include "shalINFO.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_nvsEP_base_con.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

//#define EOK_cfg_nvsEP_base_numberof                                 10







// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static uint16_t s_hash(uint16_t id);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



// this struct contains the default values of the entities of the whole device

extern const eo_cfg_nvsEP_base_t eo_cfg_nvsEP_base_default =
{
    EO_INIT(.boardinfo)             {0},
    EO_INIT(.applicationinfo)       {0},
    EO_INIT(.ipnetwork)
    {

        EO_INIT(.macaddress)            EECOMMON_mac_oui_iit,
        EO_INIT(.ipaddress)             EECOMMON_ipaddr_from(10, 0, 0, 1),
        EO_INIT(.ipnetmask)             EECOMMON_ipmask_default_iit 
    },
    EO_INIT(.bootprocess)           ee_procUpdater,
    EO_INIT(.gotoprocess)           ee_procNone,
    EO_INIT(.forcerestart)          eobool_false,
    EO_INIT(.localise)              eobool_false,
    EO_INIT(.remoteipaddress)       EECOMMON_ipaddr_from(10, 0, 0, 0),
    EO_INIT(.remoteipport)          EOK_cfg_nvsEP_base_endpoint,
    EO_INIT(.notanv_filler0)        {0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1}
}; 

  



// now we have the const netvars


#define OFFSETof__boardinfo                         (0)
#define CAPACITY__boardinfo                         sizeof(eEboardInfo_t)
extern EOnv_con_t eo_cfg_nvsEP_base__boardinfo =
{   // pos =  0
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__boardinfo,
    EO_INIT(.capacity)  CAPACITY__boardinfo,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.boardinfo,
    EO_INIT(.offset)    OFFSETof__boardinfo,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__boardinfo),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__boardinfo)
};
#define OFFSETafter__boardinfo                      (OFFSETof__boardinfo+CAPACITY__boardinfo)


#define OFFSETof__applicationinfo                   OFFSETafter__boardinfo
#define CAPACITY__applicationinfo                   sizeof(eEmoduleInfo_t)
extern EOnv_con_t eo_cfg_nvsEP_base__applicationinfo =
{   // pos =  1
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__applicationinfo,
    EO_INIT(.capacity)  CAPACITY__applicationinfo,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.applicationinfo,
    EO_INIT(.offset)    OFFSETof__applicationinfo,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__applicationinfo),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__applicationinfo)
};
#define OFFSETafter__applicationinfo                (OFFSETof__applicationinfo+CAPACITY__applicationinfo)


#define OFFSETof_ipnetwork                      OFFSETafter__applicationinfo
#define CAPACITY_ipnetwork                      sizeof(eEipnetwork_t)
extern EOnv_con_t eo_cfg_nvsEP_base_ipnetwork =
{   // pos =  2
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID_ipnetwork,
    EO_INIT(.capacity)  CAPACITY_ipnetwork,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.ipnetwork,
    EO_INIT(.offset)    OFFSETof_ipnetwork,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork)
};
#define OFFSETafter_ipnetwork                   (OFFSETof_ipnetwork +CAPACITY_ipnetwork)


#define OFFSETof_ipnetwork__macaddress          OFFSETof_ipnetwork
#define CAPACITY_ipnetwork__macaddress          sizeof(eOmacaddr_t)
extern EOnv_con_t eo_cfg_nvsEP_base_ipnetwork__macaddress =
{   // pos =  3
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID_ipnetwork__macaddress,
    EO_INIT(.capacity)  CAPACITY_ipnetwork__macaddress,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.ipnetwork.macaddress,
    EO_INIT(.offset)    OFFSETof_ipnetwork__macaddress,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__macaddress),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__macaddress)
};
#define OFFSETafter_ipnetwork__macaddress      (OFFSETof_ipnetwork__macaddress + CAPACITY_ipnetwork__macaddress)


#define OFFSETof_ipnetwork__ipaddress           OFFSETafter_ipnetwork__macaddress
#define CAPACITY_ipnetwork__ipaddress           sizeof(eOipv4addr_t)
extern EOnv_con_t eo_cfg_nvsEP_base_ipnetwork__ipaddress =
{   // pos =  4
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress,
    EO_INIT(.capacity)  CAPACITY_ipnetwork__ipaddress,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.ipnetwork.ipaddress,
    EO_INIT(.offset)    OFFSETof_ipnetwork__ipaddress,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipaddress),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipaddress)
};
#define OFFSETafter_ipnetwork__ipaddress        (OFFSETof_ipnetwork__ipaddress + CAPACITY_ipnetwork__ipaddress)


#define OFFSETof_ipnetwork__ipnetmask           OFFSETafter_ipnetwork__ipaddress
#define CAPACITY_ipnetwork__ipnetmask           sizeof(eOipv4addr_t)
extern EOnv_con_t eo_cfg_nvsEP_base_ipnetwork__ipnetmask =
{   // pos =  9
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask,
    EO_INIT(.capacity)  CAPACITY_ipnetwork__ipnetmask,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.ipnetwork.ipnetmask,
    EO_INIT(.offset)    OFFSETof_ipnetwork__ipnetmask,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipnetmask),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP_ipnetwork__ipnetmask)
};
#define OFFSETafter_ipnetwork__ipnetmask        (OFFSETof_ipnetwork__ipnetmask + CAPACITY_ipnetwork__ipnetmask)


#define OFFSETof__bootprocess                   OFFSETafter_ipnetwork
#define CAPACITY__bootprocess                   sizeof(eEprocess_t)
extern EOnv_con_t eo_cfg_nvsEP_base__bootprocess =
{   // pos =  6
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__bootprocess,
    EO_INIT(.capacity)  CAPACITY__bootprocess,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.bootprocess,
    EO_INIT(.offset)    OFFSETof__bootprocess,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__bootprocess),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__bootprocess)
};
#define OFFSETafter__bootprocess                (OFFSETof__bootprocess + CAPACITY__bootprocess)


#define OFFSETof__gotoprocess                   OFFSETafter__bootprocess
#define CAPACITY__gotoprocess                   sizeof(eEprocess_t)
extern EOnv_con_t eo_cfg_nvsEP_base__gotoprocess =
{   // pos =  7
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__gotoprocess,
    EO_INIT(.capacity)  CAPACITY__gotoprocess,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.gotoprocess,
    EO_INIT(.offset)    OFFSETof__gotoprocess,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__gotoprocess),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__gotoprocess)
};
#define OFFSETafter__gotoprocess                (OFFSETof__gotoprocess + CAPACITY__gotoprocess)


#define OFFSETof__forcerestart                   OFFSETafter__gotoprocess
#define CAPACITY__forcerestart                   sizeof(eObool_t)
extern EOnv_con_t eo_cfg_nvsEP_base__forcerestart =
{   // pos =  8
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__forcerestart,
    EO_INIT(.capacity)  CAPACITY__forcerestart,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.forcerestart,
    EO_INIT(.offset)    OFFSETof__forcerestart,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__forcerestart),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__forcerestart)
};
#define OFFSETafter__forcerestart               (OFFSETof__forcerestart + CAPACITY__forcerestart)


#define OFFSETof__localise                      OFFSETafter__forcerestart
#define CAPACITY__localise                      sizeof(eObool_t)
extern EOnv_con_t eo_cfg_nvsEP_base__localise =
{   // pos =  9
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__localise,
    EO_INIT(.capacity)  CAPACITY__localise,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.localise,
    EO_INIT(.offset)    OFFSETof__localise,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__localise),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__localise)
};
#define OFFSETafter__localise                   (OFFSETof__localise + CAPACITY__localise)


#define OFFSETof__remoteipaddress                      OFFSETafter__localise
#define CAPACITY__remoteipaddress                      sizeof(eOipv4addr_t)
extern EOnv_con_t eo_cfg_nvsEP_base__remoteipaddress =
{   // pos =  10
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__remoteipaddress,
    EO_INIT(.capacity)  CAPACITY__remoteipaddress,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.remoteipaddress,
    EO_INIT(.offset)    OFFSETof__remoteipaddress,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__remoteipaddress),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__remoteipaddress)
};
#define OFFSETafter__remoteipaddress                    (OFFSETof__remoteipaddress + CAPACITY__remoteipaddress)


#define OFFSETof__remoteipport                      OFFSETafter__remoteipaddress
#define CAPACITY__remoteipport                      sizeof(eOipv4port_t)
extern EOnv_con_t eo_cfg_nvsEP_base__remoteipport =
{   // pos =  11
    EO_INIT(.id)        EOK_cfg_nvsEP_base_NVID__remoteipport,
    EO_INIT(.capacity)  CAPACITY__remoteipport,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_base_default.remoteipport,
    EO_INIT(.offset)    OFFSETof__remoteipport,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_base_NVFUNTYP__remoteipport),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_base_NVFUNTYP__remoteipport)
};
#define OFFSETafter__remoteipport                       (OFFSETof__remoteipport + CAPACITY__remoteipport)

#define OFFSETof__notanv_filler0                    OFFSETafter__remoteipport
#define CAPACITY__notanv_filler0                    sizeof(eo_cfg_nvsEP_base_default.notanv_filler0)
#define OFFSETafter__notanv_filler0                 (OFFSETof__notanv_filler0 + CAPACITY__notanv_filler0)


// guard on alignement of variables. if it doesnt compile then ... the compiler has surely inserted some holes
typedef uint8_t eo_cfg_nvsEP_base_t_GUARD[ ( (OFFSETafter__notanv_filler0) == sizeof(eo_cfg_nvsEP_base_t) ) ? (1) : (0)];

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

extern EOtreenode eo_cfg_nvsEP_base_tree_con[];

extern EOtreenode eo_cfg_nvsEP_base_tree_con[] =
{
    {   // 0
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__boardinfo,
        EO_INIT(.index)     0,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    },
    {   // 1
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__applicationinfo,
        EO_INIT(.index)     1,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    },
    {   // 2
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base_ipnetwork,
        EO_INIT(.index)     6,
        EO_INIT(.nchildren) 3,
        EO_INIT(.ichildren) {3, 4, 5},
        EO_INIT(.pchildren) {&eo_cfg_nvsEP_base_tree_con[3], &eo_cfg_nvsEP_base_tree_con[4], &eo_cfg_nvsEP_base_tree_con[5]}
    },
        {   // 3
            EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base_ipnetwork__macaddress,
            EO_INIT(.index)     3,
            EO_INIT(.nchildren) 0,
            EO_INIT(.ichildren) {0},
            EO_INIT(.pchildren) {NULL}
        },  
        {   // 4
            EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base_ipnetwork__ipaddress,
            EO_INIT(.index)     4,
            EO_INIT(.nchildren) 0,
            EO_INIT(.ichildren) {0},
            EO_INIT(.pchildren) {NULL}
        }, 
        {   // 5
            EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base_ipnetwork__ipnetmask,
            EO_INIT(.index)     5,
            EO_INIT(.nchildren) 0,
            EO_INIT(.ichildren) {0},
            EO_INIT(.pchildren) {NULL}
        },
    {   // 6
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__bootprocess,
        EO_INIT(.index)     6,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    }, 
    {   // 7
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__gotoprocess,
        EO_INIT(.index)     7,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    },                      
    {   // 8
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__forcerestart,
        EO_INIT(.index)     8,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    }, 
    {   // 9
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__localise,
        EO_INIT(.index)     9,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    }, 
    {   // 10
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__remoteipaddress,
        EO_INIT(.index)     10,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    },
    {   // 11
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_base__remoteipport,
        EO_INIT(.index)     11,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    }  
};

//typedef uint8_t eo_cfg_nvsEP_base_guard_t[ (EOK_cfg_nvsEP_base_numberof == (sizeof(eo_cfg_nvsEP_base_tree_con)/sizeof(const EOtreenode))) ? (1) : (0)];



const EOconstvector  s_eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con = 
{
    EO_INIT(.size)              sizeof(eo_cfg_nvsEP_base_tree_con)/sizeof(EOtreenode), //EOK_cfg_nvsEP_base_numberof,
    EO_INIT(.item_size)         sizeof(EOtreenode),
    EO_INIT(.item_array_data)   eo_cfg_nvsEP_base_tree_con
};


extern const EOconstvector* const eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con = &s_eo_cfg_nvsEP_base_constvector_of_treenodes_EOnv_con;


extern const eOuint16_fp_uint16_t eo_cfg_nvsEP_base_fptr_hashfunction_id2index = eo_cfg_nvsEP_base_hashfunction_id2index;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern uint16_t eo_cfg_nvsEP_base_hashfunction_id2index(uint16_t id)
{
    #define IDTABLESIZE     16

    // in order to always have a hit the table s_idtable[] it must be of size equal to max{ s_hash(id) }, thus if we
    // use an id of value 16 and s_hash() just keeps the lsb, then the size must be 17 
    // if there are holes, they shall have EOK_uint16dummy in other entries. for example, if we have ids = {0, 7, 16}
    // then the table shall be of size 17, shall contain 0xffff everywhere but in positions 0, 7, 16 where the values
    // are ... 0, 7, 16

    static const uint16_t s_idtable[IDTABLESIZE] = 
    { 
        EOK_cfg_nvsEP_base_NVID__boardinfo,                     EOK_cfg_nvsEP_base_NVID__applicationinfo,   
        EOK_cfg_nvsEP_base_NVID_ipnetwork,                      EOK_cfg_nvsEP_base_NVID_ipnetwork__macaddress,
        EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress,           EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask,
        EOK_cfg_nvsEP_base_NVID__bootprocess,                   EOK_cfg_nvsEP_base_NVID__gotoprocess,
        EOK_cfg_nvsEP_base_NVID__forcerestart,                  EOK_cfg_nvsEP_base_NVID__localise,
        EOK_cfg_nvsEP_base_NVID__remoteipaddress,               EOK_cfg_nvsEP_base_NVID__remoteipport,
        EOK_uint16dummy,                                        EOK_uint16dummy,
        EOK_uint16dummy,                                        EOK_uint16dummy  
    };
    
    uint16_t index = s_hash(id);
    
    if((index < IDTABLESIZE) && (id == s_idtable[index]) )
    {
        return(index);
    }
    else
    {
        return(EOK_uint16dummy);
    }
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static uint16_t s_hash(uint16_t id)
{
    return((id - EOK_cfg_nvsEP_base) & 0x03FF);
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



