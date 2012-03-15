// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMDATACONTAINER_H_
#define _EOMDATACONTAINER_H_


// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       EOMdataContainer.h
    @brief      This header file implements public interface to Datacontainer object
    @author     valentina.gaggero@iit.it
    @date       16/12/2011
**/

/** @defgroup eom_datacontainer A service SW entity: object
    this object has been create with the aim of shared data beetwwen tasks, where there is one writer, and many readers.
    
    
    @todo put documentation proper to the entity.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef struct Object_hid Object
    @brief      Object is an opaque struct. It is used to implement data abstraction for the generic
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMdataContainer_hid     EOMdataContainer;

typedef uint16_t eOdataContainerIndex_t;

    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern Object* obj_New(float a, float b, float c)
    @brief      Creates a new object filter with response y(n) = a * y(n-1) + b * x(n) + c * x(n-1)
    @return     The pointer to the required object or NULL upon failure.
 **/
extern EOMdataContainer* eom_dataContainer_New(eOsizecntnr_t capacity);



/** @fn         
    @brief      Executes a filtering on @e x and retrieves result @e y. 
    @param      p               Pointer to the object.
    @param      x               The input signal to be filtered. It will not be changed by this function.
    @param      y               The filtered output.
    @return     The success or failure of the filtering: ob_Result_OK if OK, ob_Result_NOK_GENERIC or 
                ob_Result_NOK_NULLPOINTER elsewhere.
 **/
//extern eOresult_t eom_dataContainer_LinkTo(EOMdataContainer* dataCont, eOsizeitem_t datasize, uint8_t *data);
extern eOresult_t eom_dataContainer_AddItem(EOMdataContainer* p, eOsizeitem_t itemsize, void *itemdata, eOdataContainerIndex_t index);
 
/** @fn         
    @brief      Executes a filtering on @e x and retrieves result @e y. 
    @param      p               Pointer to the object.
    @param      x               The input signal to be filtered. It will not be changed by this function.
    @param      y               The filtered output.
    @return     The success or failure of the filtering: ob_Result_OK if OK, ob_Result_NOK_GENERIC or 
                ob_Result_NOK_NULLPOINTER elsewhere.
 **/
//extern eOresult_t eom_dataContainer_GetSize(EOMdataContainer *dataCont, eOsizeitem_t *datasize);



/** @fn         
    @brief      Executes a filtering on @e x and retrieves result @e y. 
    @param      p               Pointer to the object.
    @param      x               The input signal to be filtered. It will not be changed by this function.
    @param      y               The filtered output.
    @return     The success or failure of the filtering: ob_Result_OK if OK, ob_Result_NOK_GENERIC or 
                ob_Result_NOK_NULLPOINTER elsewhere.
 **/
//extern eOresult_t eom_dataContainer_Read(EOMdataContainer *dataCont, void *data);
extern eOresult_t eom_dataContainer_ReadItem(EOMdataContainer *p, eOdataContainerIndex_t itemIndex, void *data);

/** @fn         
    @brief      Executes a filtering on @e x and retrieves result @e y. 
    @param      p               Pointer to the object.
    @param      x               The input signal to be filtered. It will not be changed by this function.
    @param      y               The filtered output.
    @return     The success or failure of the filtering: ob_Result_OK if OK, ob_Result_NOK_GENERIC or 
                ob_Result_NOK_NULLPOINTER elsewhere.
 **/
//extern eOresult_t eom_dataContainer_Write(EOMdataContainer *dataCont, void *data);
extern eOresult_t eom_dataContainer_WriteItem(EOMdataContainer *p, eOdataContainerIndex_t itemIndex, void *data);





// - doxy end ---------------------------------------------------------------------------------------------------------

/** @}            
    end of group eom_datacontainer  
 **/

 
#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


