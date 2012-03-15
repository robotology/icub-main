// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMDATACONTAINER_HID_H_
#define _EOMDATACONTAINER_HID_H_

// - doxy -------------------------------------------------------------------------------------------------------------

/* @file       EOMdataContainer_hid.h
    @brief      This header file implements hidden interface to an entity.
    @author     valentina.gaggero@iit.it
    @date       16/12/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------
#include "EOvector.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
#include "EOMdataContainer.h"


// - definition of the hidden struct implementing the object ----------------------------------------------------------


typedef struct 
{
    eOsizeitem_t  datasize;
    void          *dataptr;
}EOMdataContainer_itemStruct_hid_t;


/** @struct     Object_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/   
struct EOMdataContainer_hid 
{
    EOMmutex                *readers_mutex;     /**< mutex used by readers */
    EOMmutex                *obj_mutex;         /**< mutex for the obj */
    uint16_t                readers_num;        /**< num of readers that are reading data */
    EOvector                *vector;
//    eOsizeitem_t            datasize;           /**< size of data */
//    uint8_t                 *data;              /**< pointer to data */
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



