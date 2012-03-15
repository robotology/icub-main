
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMATRIX3D_HID_H_
#define _EOMATRIX3D_HID_H_


/* @file        EOmatrix3d_hid.h
    @brief      This header file implements hidden interface to a matrix3d object.
    @author     marco.accame@iit.it
    @date       08/03/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOmatrix3d.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef struct
{
    uint8_t         capacity;
    uint8_t         itemsize;
    uint8_t         size;
    uint8_t         dummy;
} eOmatrix3d_head_t;   EO_VERIFYsizeof(eOmatrix3d_head_t, 4);

typedef struct eOmatrix3d_node_T
{
    eOmatrix3d_head_t           head;
    struct eOmatrix3d_node_T*   node[1];
} eOmatrix3d_node_t;

typedef struct
{
    eOmatrix3d_head_t           head;
    uint8_t                     data[4];
} eOmatrix3d_end_t;


/** @struct     EOmatrix3d_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOmatrix3d_hid 
{
    eOmatrix3d_head_t           head;
    eOmatrix3d_node_t*          node[1];
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

//extern eOresult_t eo_matrix3d_hid_PushBackData(EOmatrix3d *p, const void *data, uint8_t sizedata);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




