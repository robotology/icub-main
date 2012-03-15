
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_FLASH_H_
#define _HAL_FLASH_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_flash.h
    @brief      This header file implements public interface to the hal flash module.
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_flash HAL FLASH

    The HAL FLASH ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern hal_result_t hal_flash_lock(void)
    @brief      Locks the FLASH 
    @return     Always hal_res_OK.
 **/
extern hal_result_t hal_flash_lock(void);


/** @fn         extern hal_result_t hal_flash_unlock(void)
    @brief      Unlocks the FLASH 
    @return     Always hal_res_OK.
 **/
extern hal_result_t hal_flash_unlock(void);


/** @fn         extern hal_result_t hal_flash_erase(uint32_t addr, uint32_t size)
    @brief      Erases the pages of FLASH which contain the starting address @e addr and the following @e size bytes.  
    @param      addr        The starting address.
    @param      size        The number of bytes to be erased
    @return     hal_res_OK or hal_res_NOK_generic on failure or if the addr is out of range.
    @warning    The erase operation is done by pages, and in particular all the pages which contain [addr, addr+size].
 **/
extern hal_result_t hal_flash_erase(uint32_t addr, uint32_t size);


/** @fn         extern hal_result_t hal_flash_write(uint32_t addr, uint32_t size, void *data)
    @brief      Writes @e size bytes pointed by @e data at address @e addr.
    @param      addr        The address. It can be 4-bytes-aligned, 2-bytes-aligned and in some FLASH types also 1-bytes-aligned.
    @param      size        The size in bytes to be written.      
    @param      data        The data to be written. 
    @return     hal_res_OK or hal_res_NOK_generic on failure or @e data is NULL or if in some FLASHes the @e addr is not correctly 
                aligned, or if the @e addr is out of range 
    @warning    The write operation can be successfully done only on FLASH which was previously erased and never written. 
 **/
extern hal_result_t hal_flash_write(uint32_t addr, uint32_t size, void *data);


/** @fn         extern hal_result_t hal_flash_read(uint32_t addr, uint32_t size, void *data)
    @brief      Reads @e size bytes pointed by @e data from address @e addr.
    @param      addr        The address. It can be 4-bytes-aligned, 2-bytes-aligned and in some FLASHes types also 1-bytes-aligned.
    @param      size        The size in bytes to be read.      
    @param      data        The pointer where to copy the data to be read. 
    @return     hal_res_OK or hal_res_NOK_generic on failure or @e data is NULL or if in some FLASHes the @e addr is not correctly 
                aligned, or if the @e addr is out of range 
 **/
extern hal_result_t hal_flash_read(uint32_t addr, uint32_t size, void *data);


/** @fn         extern uint32_t hal_flash_get_baseaddress(void)
    @brief      Retrieves the base address of the FLASH.
    @return     0x0800000 on STM32F1x.
 **/
extern uint32_t hal_flash_get_baseaddress(void);


/** @fn         extern uint32_t hal_flash_get_totalsize(void)
    @brief      Retrieves the size of the FLASH.
    @return     256K on STM32F107.
 **/
extern uint32_t hal_flash_get_totalsize(void);


/** @fn         extern hal_boolval_t hal_flash_address_isvalid(uint32_t addr)
    @brief      tells if @addris in FLASH
    @param      addr        The address of the memory of which one wants to check
    @return     hal_false or hal_true
 **/
extern hal_boolval_t hal_flash_address_isvalid(uint32_t addr);


/** @fn         extern uint32_t hal_flash_get_pagesize(void)
    @brief      Retrieves the page size of the FLASH at address @addr
    @param      addr        The address of the memory of which one wants the size of page
    @return     Always 2048 on STM32F1x for a valid address, or hal_NA32 if addr is not on FLASH
 **/
extern uint32_t hal_flash_get_pagesize(uint32_t addr);


/** @fn         extern uint32_t hal_flash_get_pageaddr(uint32_t addr))
    @brief      Retrieves the address of the page containing the FLASH address @addr
    @param      addr        The address of the memory of which one wants the page address
    @return     Always 2048-aligned on STM32F1x for a valid address, or hal_NA32 if addr is not on FLASH
 **/
extern uint32_t hal_flash_get_pageaddr(uint32_t addr);


/** @fn         extern uint32_t hal_flash_get_unitsize(uint32_t addr))
    @brief      Retrieves the unit size of the FLASH at address @addr. The unit is the smallest writeable 
                piece of FLASH.
    @param      addr        The address of the memory of which one wants the unit size
    @return     Always 2 on STM32F1x for a valid address, or hal_NA32 if addr is not on FLASH
 **/
extern uint32_t hal_flash_get_unitsize(uint32_t addr);



/** @}            
    end of group arm_hal_flash  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



