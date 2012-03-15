
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOFIFOBYTE_H_
#define _EOFIFOBYTE_H_

/** @file       EOfifoByte.h
	@brief      This header file implements public interface to a mutex-protected fifo object which 
                manupulates bytes.
	@author     marco.accame@iit.it
	@date       08/03/2011
**/

/** @defgroup eo_fifobyte Object EOfifoByte
    The EOfifoByte implements a fifo queue of bytes which is protected against concurrent
    access by a mutex. It can be used in single-task applications for bi-directional data exchange between
    the task and a ISR. For instance the USART ISR can fill a EOfifoByte with
    received data and send an event to the task which can retrieve buffered data and use it.
    Also, it can be used in multi-task applications to exchange data between a task and an ISR or amongst
    user tasks.
    The object EOfifoByte is "derived" from the EOfifo object.
    See @ref eo_mutexfifo.
     
    @{		
 **/

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef struct EOfifoByte_hid EOfifoByte
    @brief      EOfifoByte is an opaque struct. It is used to implement data abstraction for the fifo of bytes 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOfifoByte_hid EOfifoByte;


// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern EOfifoByte* eo_fifobyte_New(eOsizecntnr_t capacity, EOVmutexDerived *mutex)
    @brief      Creates a new EOfifoByte object. 
    @param      capacity        Maximum number of byte items that will be stored in the fifobyte queue
    @param      mutex           Pointer to a mutex-derived object which can offer the proper protection. If NULL,
                                the mutex is not used and there is no protection vs concurrent access to the object.
    @return     Pointer to the object. The function always returns a valid not NULL pointer.
 **/
extern EOfifoByte* eo_fifobyte_New(eOsizecntnr_t capacity, EOVmutexDerived *mutex);

 
/** @fn         extern eOresult_t eo_fifobyte_Capacity(EOfifoByte *fifobyte, eOsizecntnr_t *capacity, eOreltime_t tout)
    @brief      Returns the maximum number of byte items that the fifobyte queue can contain.
    @param      fifobyte        Pointer to the fifo object.
    @param      capacity        pointer to the max number of byte items that the fifobyte queue can contain. 
                                Note: the user must pass a &capacity.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifobyte is NULL, eores_NOK_timeout if the mutex was 
                busy within the specified timeout.
 **/ 
extern eOresult_t eo_fifobyte_Capacity(EOfifoByte *fifobyte, eOsizecntnr_t *capacity, eOreltime_t tout);


/** @fn         extern eOresult_t eo_fifobyte_Size(EOfifoByte *fifobyte, eOsizecntnr_t *size, eOreltime_t tout)
    @brief      Returns the current number of byte items in the fifobyte queue.
    @param      fifobyte        Pointer to the fifobyte object.
    @param      size            Pointer to the number of byte items in the fifobyte queue. 
                                Note: the user must pass a &size.
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifobyte is NULL, eores_NOK_timeout if the mutex was 
                busy within the specified timeout
 **/
extern eOresult_t eo_fifobyte_Size(EOfifoByte *fifobyte, eOsizecntnr_t *size, eOreltime_t tout);


/** @fn         eOresult_t eo_fifobyte_Put(EOfifoByte *fifobyte, uint8_t item, eOreltime_t tout)
    @brief      Copies the byte in @e item in the fifobyte queue.
    @param      fifobyte        Pointer to the fifobyte object.
    @param      item            The byte to be copied.  
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon successful copy, eores_NOK_busy if the queue is full, eores_NOK_nullpointer
                if fifobyte is NULL, eores_NOK_timeout if the mutex was busy within the specified timeout
 **/
extern eOresult_t eo_fifobyte_Put(EOfifoByte *fifobyte, uint8_t item, eOreltime_t tout);

/** @fn         extern eOresult_t eo_fifobyte_Get(EOfifoByte *fifobyte, uint8_t *item, eOreltime_t tout)
    @brief      Retrieves a pointer to the first-in byte object from the fifobyte queue without removing it.
    @param      fifobyte        Pointer to the fifobyte object.
    @param      item            The address in which the function will copy the retrieved byte. 
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nodata if fifobyte is empty, 
                eores_NOK_nullpointer if fifobyte is NULL, eores_NOK_timeout if the mutex was busy within the 
                specified timeout. 
 **/
extern eOresult_t eo_fifobyte_Get(EOfifoByte *fifobyte, uint8_t *item, eOreltime_t tout);

/** @fn         eOresult_t eo_fifobyte_Rem(EOfifoByte *fifobyte, eOreltime_t tout)
    @brief      Removes the first-in byte in the fifobyte queue.
    @param      fifobyte        Pointer to the fifobyte object.
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifobyte is NULL, eores_NOK_timeout if 
                the mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_fifobyte_Rem(EOfifoByte *fifobyte, eOreltime_t tout); 


/** @fn         eOresult_t eo_fifobyte_Clear(EOfifoByte *fifobyte, eOreltime_t tout)
    @brief      Removes every object in the fifobyte queue.
    @param      fifobyte        Pointer to the fifobyte object.
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifobyte is NULL, eores_NOK_timeout 
                if the mutex was busy within the specified timeout
 **/ 
extern eOresult_t eo_fifobyte_Clear(EOfifoByte *fifobyte, eOreltime_t tout);


/** @}            
    end of group eo_fifobyte  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



