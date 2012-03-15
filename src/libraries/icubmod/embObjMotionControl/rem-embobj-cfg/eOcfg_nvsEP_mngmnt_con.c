
/* @file       eOcfg_nvsEP_mngmnt_con.c
    @brief      This file keeps the constant configuration for the NVs in the base endpoint port
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h" 
#include "string.h"
#include "stdio.h"


#include "EOnv_hid.h"
#include "EOtreenode_hid.h"


#include "EOconstvector_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_nvsEP_mngmnt_con.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

// must be extern so that another file can use it as initialiser of a a const struct
extern uint16_t eo_cfg_nvsEP_mngmnt_hashfunction_id2index(uint16_t nvid);

static uint16_t s_hash(uint16_t id);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



// this struct contains the default values of the entities of the whole device

extern const eo_cfg_nvsEP_mngmnt_t eo_cfg_nvsEP_mngmnt_default =
{
    EO_INIT(.upto10rop2signal)
    {
        EO_INIT(.head)
        {
            EO_INIT(.capacity)      10,
            EO_INIT(.itemsize)      sizeof(eOropSIGcfg_t),
            EO_INIT(.size)          0     // it is the only one in the header that can change
        },
        EO_INIT(.data)              {0}           // data can also change
    },
    EO_INIT(.workingmode)           mngmnt_workingmode_idle,
    EO_INIT(.notanv_filler0)        { 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1 }
}; 

  


// now we have the const netvars


#define OFFSETof__upto10rop2signal                 (0) 
#define CAPACITY__upto10rop2signal                 sizeof(EOarray_of_10eOropSIGcfg)
extern EOnv_con_t eo_cfg_nvsEP_mngmnt__upto10rop2signal =
{   // pos =  00
    EO_INIT(.id)        EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal,
    EO_INIT(.capacity)  CAPACITY__upto10rop2signal,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mngmnt_default.upto10rop2signal,
    EO_INIT(.offset)    OFFSETof__upto10rop2signal,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mngmnt_NVFUNTYP__upto10rop2signal),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mngmnt_NVFUNTYP__upto10rop2signal)
};
#define OFFSETafter__upto10rop2signal              (OFFSETof__upto10rop2signal + CAPACITY__upto10rop2signal)



#define OFFSETof__workingmode                       OFFSETafter__upto10rop2signal
#define CAPACITY__workingmode                       sizeof(uint8_t)
extern EOnv_con_t eo_cfg_nvsEP_mngmnt__workingmode =
{   // pos =  01
    EO_INIT(.id)        EOK_cfg_nvsEP_mngmnt_NVID__workingmode,
    EO_INIT(.capacity)  CAPACITY__workingmode,
    EO_INIT(.resetval)  (const void*)&eo_cfg_nvsEP_mngmnt_default.workingmode,
    EO_INIT(.offset)    OFFSETof__workingmode,
    EO_INIT(.typ)       EO_nv_TYP(EOK_cfg_nvsEP_mngmnt_NVFUNTYP__workingmode),
    EO_INIT(.fun)       EO_nv_FUN(EOK_cfg_nvsEP_mngmnt_NVFUNTYP__workingmode)
};
#define OFFSETafter__workingmode                    (OFFSETof__workingmode + CAPACITY__workingmode)


#define OFFSETof__notanv_filler0                    OFFSETafter__workingmode
#define CAPACITY__notanv_filler0                    sizeof(eo_cfg_nvsEP_mngmnt_default.notanv_filler0)
#define OFFSETafter__notanv_filler0                 (OFFSETof__notanv_filler0 + CAPACITY__notanv_filler0)

// guard on alignement of variables. if it doesnt compile then ... the compiler has surely inserted some holes
typedef uint8_t eo_cfg_nvsEP_mngmnt_t_GUARD[ ( (OFFSETafter__notanv_filler0) == sizeof(eo_cfg_nvsEP_mngmnt_t) ) ? (1) : (0)];

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables
// --------------------------------------------------------------------------------------------------------------------

extern EOtreenode eo_cfg_nvsEP_mngmnt_tree_con[];

extern EOtreenode eo_cfg_nvsEP_mngmnt_tree_con[] =
{              
    {   // 00
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_mngmnt__upto10rop2signal,
        EO_INIT(.index)     0,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    },
    {   // 1
        EO_INIT(.data)      (void*)&eo_cfg_nvsEP_mngmnt__workingmode,
        EO_INIT(.index)     1,
        EO_INIT(.nchildren) 0,
        EO_INIT(.ichildren) {0},
        EO_INIT(.pchildren) {NULL}
    }    
};


const EOconstvector  s_eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con = 
{
    EO_INIT(.size)              sizeof(eo_cfg_nvsEP_mngmnt_tree_con)/sizeof(EOtreenode),
    EO_INIT(.item_size)         sizeof(EOtreenode),
    EO_INIT(.item_array_data)   eo_cfg_nvsEP_mngmnt_tree_con
};


extern const EOconstvector* const eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con = &s_eo_cfg_nvsEP_mngmnt_constvector_of_treenodes_EOnv_con;


extern const eOuint16_fp_uint16_t eo_cfg_nvsEP_mngmnt_fnptr_hashfunction_id2index = eo_cfg_nvsEP_mngmnt_hashfunction_id2index;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern uint16_t eo_cfg_nvsEP_mngmnt_hashfunction_id2index(uint16_t id)
{
    #define IDTABLESIZE     4

    // in order to always have a hit the table s_idtable[] it must be of size equal to max{ s_hash(id) }, thus if we
    // use an id of value 16 and s_hash() just keeps the lsb, then the size must be 17 
    // if there are holes, they shall have EOK_uint16dummy in other entries. for example, if we have ids = {0, 7, 16}
    // then the table shall be of size 17, shall contain 0xffff everywhere but in positions 0, 7, 16 where the values
    // are ... 0, 7, 16

    static const uint16_t s_idtable[IDTABLESIZE] = 
    { 
        EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal,           EOK_cfg_nvsEP_mngmnt_NVID__workingmode,
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
    return((id - EOK_cfg_nvsEP_mngmnt) & 0x03FF);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



