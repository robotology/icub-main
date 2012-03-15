
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMATRIX3D_H_
#define _EOMATRIX3D_H_


/** @file       EOmatrix3d.h
    @brief      This header file implements public interface to a matrix3d object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eo_matrix3d Object EOmatrix3d
    The EOmatrix3d object is a container which is work-in-progress. It could be used as a slim vector
    in which the content of memory is exposed outsize via a number of data strcuctures such as eOmatrix3dofbytes_t
    or eOmatrix3dofhalves_t or eOmatrix3dofwords_t
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 





/** @typedef    typedef struct EOmatrix3d_hid EOmatrix3d
    @brief      EOmatrix3d is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOmatrix3d_hid EOmatrix3d;   

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOmatrix3d* eo_matrix3d_New(uint8_t sizeofitem, uint8_t capacity1)
    @brief      creates a 3d matrix able to accept items of @e sizeitems
 **/
extern EOmatrix3d* eo_matrix3d_New(uint8_t sizeofitem, uint8_t capacity1);


/** @fn         extern eOresult_t eo_matrix3d_Reset(EOmatrix3d *t)
    @brief      Resets the matrix3d
    @param      p               The pointer to the matrix3d object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL
 **/
extern eOresult_t eo_matrix3d_Level1_PushBack(EOmatrix3d *p, uint8_t capacity2);


extern eOresult_t eo_matrix3d_Level2_PushBack(EOmatrix3d *p, uint8_t onindex1, uint8_t capacity3);

extern eOresult_t eo_matrix3d_Level3_PushBack(EOmatrix3d *p, uint8_t onindex1, uint8_t onindex2, void *pitem);

extern void* eo_matrix3d_At(EOmatrix3d *p, uint8_t i1, uint8_t i2, uint8_t i3);

extern uint8_t eo_matrix3d_ItemSize(EOmatrix3d *p);

extern uint8_t eo_matrix3d_Level1_Capacity(EOmatrix3d *p);
extern uint8_t eo_matrix3d_Level2_Capacity(EOmatrix3d *p, uint8_t onindex1);
extern uint8_t eo_matrix3d_Level3_Capacity(EOmatrix3d *p, uint8_t onindex1, uint8_t onindex2);

extern uint8_t eo_matrix3d_Level1_Size(EOmatrix3d *p);
extern uint8_t eo_matrix3d_Level2_Size(EOmatrix3d *p, uint8_t onindex1);
extern uint8_t eo_matrix3d_Level3_Size(EOmatrix3d *p, uint8_t onindex1, uint8_t onindex2);



extern void* eo_matrix3d_At(EOmatrix3d *p, uint8_t i1, uint8_t i2, uint8_t i3);
 

 


/** @}            
    end of group eo_matrix3d  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

