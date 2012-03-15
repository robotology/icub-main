
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_CRC_H_
#define _HAL_CRC_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_crc.h
    @brief      This header file implements public interface to the hal crc module.
    @author     marco.accame@iit.it
    @date       09/16/2011
**/

/** @defgroup arm_hal_crc HAL CRC

    The HAL CRC ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef enum hal_crc_t 
    @brief      hal_crc_t contains every possible crc peripheral.
 **/ 
typedef enum  
{ 
    hal_crc0 = 0,         
    hal_crc1 = 1
} hal_crc_t;

enum { hal_crcs_num = 2 };


/** @typedef    typedef enum hal_crc_order_t 
    @brief      hal_crc_order_t contains the supported orders.
 **/ 
typedef enum
{
    hal_crc_order_16    = 16,
    hal_crc_order_32    = 32
} hal_crc_order_t;



/** @typedef    typedef struct hal_crc_cfg_t;
    @brief      contains configuration data of led peripheral.
 **/
typedef struct
{
    hal_crc_order_t order;              /**< the order of the polynomial to use */    
    uint32_t        polynomial;         /**< the polynomial to use specified in direct form. up to order 32.
                                             crc32 is 0x04C11DB7, crc16-ccitt is 0x00001021 */ 
    void*           crctblram;          /**< the ram used to compute the crc table when there is no hw support for
                                             a given polynomial. its size must be 256*2 in case of CRC-16 and 256*4 in
                                             case of CRC-32. */                                                     
} hal_crc_cfg_t;


/** @typedef    typedef enum hal_crc_compute_mode_t 
    @brief      hal_crc_mode_t contains the mode with which the crc is computed.
 **/ 
typedef enum
{
    hal_crc_mode_clear          = 0,    /**< the initial status is cleared to 0xfff / 0xffffffff */    
    hal_crc_mode_accumulate     = 1     /**< the initial status is kept from previous computation */  
} hal_crc_compute_mode_t;

 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const hal_crc_cfg_t hal_crc_cfg_default;     // = { .order =  hal_crc_order_32, .polynomial = 0x04C11DB7. .crctblram = NULL};

extern const uint32_t hal_crc_poly_crc32;           // = 0x04C11DB7; 
extern const uint32_t hal_crc_poly_crc16_ccitt;     // = 0x00001021; 

// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn			extern hal_result_t hal_crc_init(hal_crc_t crc, const hal_crc_cfg_t *cfg)
    @brief  	This function inits a crc peripheral.
    @details    The type of CRC to use is specified by the order and polymomial. It is possible to specify any polymomial
                of orders 16 and 32, as long as it is passed the ram required by the crc table.
                The hal_crc_init() function prepares a proper crc table and stores it in the externally passed ram.
                The crctblram field is not used (and thus can be NULL) in the following cases:
                - for polynomials 0x04C11DB7 or 0x00001021, as in STM32Fx the former is HW supported and the latter
                  uses a pre-calculated ROM-ed table, and on DSPIC the other way round (crc16 is HW supported and crc32 uses 
                  a ROM-ed table).
                - for any CRC-16 in DSPICx where the HW support is used.
    @param      crc             The crc to initialise. 
    @param      cfg             The configuration. In case is NULL, the default is used.
    @return 	hal_res_NOK_unsupported in case the led is not supported, else hal_res_OK
  */
extern hal_result_t hal_crc_init(hal_crc_t crc, const hal_crc_cfg_t *cfg);


/**
    @fn         extern hal_result_t hal_crc_compute(hal_crc_t crc, hal_crc_compute_mode_t mode, const void *data, uint32_t size, uint32_t *out)
    @brief      computes the crc on len bytes pointed by buf and places the result in out.
    @details    the crc is computed using the following conventions: initial value is 0xffff / 0xffffffff, final
                xor is not performed (the user can do it externally), 
    @param      crc         the crc peripheral.
    @param      mode        the compute mode.
    @param      data        The data as a stream of bytes.  
    @param      size        The number of bytes. If not a multiple of 4 then data is internally zero padded before computing crc.
    @param      out         Pointer to the resulting crc. For hal_crc_order_16 only the 16 LSB are used
    @return     hal_res_NOK_generic in case the crc peripheral wasn't initted or parameters are incorrect, else hal_res_OK
 **/
extern hal_result_t hal_crc_compute(hal_crc_t crc, hal_crc_compute_mode_t mode, const void *data, uint32_t size, uint32_t *out);



/** @}            
    end of group arm_hal_crc  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



