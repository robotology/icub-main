
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOFIFOWORD_H_
#define _EOFIFOWORD_H_

/** @file       EOfifoWord.h
	@brief      This header file implements public interface to a mutex-protected fifo object which 
                manupulates words.
	@author     marco.accame@iit.it
	@date       08/03/2011
**/

/** @defgroup eo_fifoword Object EOfifoWord
    The EOfifoWord implements a fifo queue of words which is protected against concurrent
    access by a mutex. It can be used in single-task applications for bi-directional data exchange between
    the task and a ISR. For instance the USART ISR can fill a EOfifoWord with
    received data and send an event to the task which can retrieve buffered data and use it.
    Also, it can be used in multi-task applications to exchange data between a task ad an ISR or amongst
    user tasks.
    The object EOfifoWord is "derived" from the EOfifo.
     
    @{		
 **/

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOfifoWord_hid EOfifoWord
    @brief      EOfifoWord is an opaque struct. It is used to implement data abstraction for the fifo of bytes 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOfifoWord_hid EOfifoWord;


// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern EOfifoWord* eo_fifoword_New(eOsizecntnr_t capacity, EOVmutexDerived *mutex)
    @brief      Creates a new EOfifoWord object. 
    @param      capacity        Maximum number of byte items that will be stored in the fifo queue
    @param      mutex           Pointer to a mutex-derived object which can offer the proper protection. If NULL,
                                the mutex is not used and there is no protection vs concurrent access to the object.
    @return     Pointer to the object. The function always returns a valid not NULL pointer.
 **/
extern EOfifoWord* eo_fifoword_New(eOsizecntnr_t capacity, EOVmutexDerived *mutex);

 
/** @fn         extern eOresult_t eo_fifoword_Capacity(EOfifoWord *fifo, eOsizecntnr_t *capacity, eOreltime_t tout)
    @brief      Returns the maximum number of byte items that the fifo queue can contain.
    @param      fifo            Pointer to the fifo object.
    @param      capacity        pointer to the max number of byte items that the fifo queue can contain. 
                                Note: the user must pass a &capacity.
    @param      tout            Timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the mutex was 
                busy within the specified timeout.
 **/ 
extern eOresult_t eo_fifoword_Capacity(EOfifoWord *fifo, eOsizecntnr_t *capacity, eOreltime_t tout);


/** @fn         extern eOresult_t eo_fifoword_Size(EOfifoWord *fifo, eOsizecntnr_t *size, eOreltime_t tout)
    @brief      Returns the current number of byte items in the fifo queue.
    @param      fifo            Pointer to the fifo object.
    @param      size            Pointer to the number of byte items in the fifo queue. 
                                Note: the user must pass a &size.
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the mutex was 
                busy within the specified timeout
 **/
extern eOresult_t eo_fifoword_Size(EOfifoWord *fifo, eOsizecntnr_t *size, eOreltime_t tout);


/** @fn         eOresult_t eo_fifoword_Put(EOfifoWord *fifo, uint32_t item, eOreltime_t tout)
    @brief      Copies the byte in @e item in the fifo queue.
    @param      fifo            Pointer to the fifo object.
    @param      item            The byte to be copied. 
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon successful copy, eores_NOK_busy if the queue is full, eores_NOK_nullpointer
                if fifo is NULL, eores_NOK_timeout if the mutex was busy within the specified timeout
 **/
extern eOresult_t eo_fifoword_Put(EOfifoWord *fifo, uint32_t item, eOreltime_t tout);

/** @fn         extern eOresult_t eo_fifoword_Get(EOfifoWord *fifo, uint32_t *item, eOreltime_t tout)
    @brief      Retrieves a pointer to the first-in byte object from the fifo queue without removing it.
    @param      fifo            Pointer to the fifo object.
    @param      item            The address in which the function will copy the retrieved byte. 
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success (retrieval of valid data), eores_NOK_nodata if fifo is empty, 
                eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if the mutex was busy within the 
                specified timeout. 
 **/
extern eOresult_t eo_fifoword_Get(EOfifoWord *fifo, uint32_t *item, eOreltime_t tout);

/** @fn         eOresult_t eo_fifoword_Rem(EOfifoWord *fifo, eOreltime_t tout)
    @brief      Removes the first-in byte in the fifo queue.
    @param      fifo            Pointer to the fifo object.
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout if 
                the mutex was busy within the specified timeout.
 **/
extern eOresult_t eo_fifoword_Rem(EOfifoWord *fifo, eOreltime_t tout); 


/** @fn         eOresult_t eo_fifoword_Clear(EOfifoWord *fifo, eOreltime_t tout)
    @brief      Removes every object in the fifo queue.
    @param      fifo            Pointer to the fifo object.
    @param      tout            The timeout for the operation in micro-seconds.
    @return     eores_OK upon success, eores_NOK_nullpointer if fifo is NULL, eores_NOK_timeout 
                if the mutex was busy within the specified timeout
 **/ 
extern eOresult_t eo_fifoword_Clear(EOfifoWord *fifo, eOreltime_t tout);


/** @}            
    end of group eo_fifoword  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



