
// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _HAL_DISPLAY_H_
#define _HAL_DISPLAY_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       hal_display.h
    @brief      This header file implements public interface to the hal display module.
    @author     marco.accame@iit.it
    @date       09/09/2011
**/

/** @defgroup arm_hal_display HAL DISPLAY

    The HAL DISPLAY ...
 
    @todo acemor-facenda: review documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "hal_base.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/** @typedef    typedef enum hal_display_resolution_t 
    @brief      hal_display_resolution_t expresses resolution of the display in term of pixel width, height and bits
                per pixel
 **/ 
typedef enum
{
    hal_display_res_320x240x24      = 0
} hal_display_resolution_t;


/** @typedef    typedef enum hal_display_font_t 
    @brief      hal_display_font_t express format of text used by the display.
 **/ 
typedef enum
{
    hal_display_font_24x16      = 0
} hal_display_font_t;


/** @typedef    typedef enum hal_display_color_t 
    @brief      hal_display_color_t express some color used by the display.
    @warning    The values in this hal_display_color_t must not be changed to avoid incoherency with the MPU's vendor's
                library. So far they are tuned for STM32F1x.    
 **/ 
typedef enum
{
    hal_display_col_black       = 0x0000,
    hal_display_col_darkgrey    = 0x7BEF,
    hal_display_col_lightgrey   = 0xC618,
    hal_display_col_white       = 0xFFFF,
    hal_display_col_red         = 0xF800,
    hal_display_col_green       = 0x07E0,
    hal_display_col_blue        = 0x001F,
    hal_display_col_yellow      = 0xFFE0,
    hal_display_col_cyan        = 0x07FF,
    hal_display_col_magenta     = 0xF81F 
} hal_display_color_t;


/** @typedef    typedef struct hal_display_cfg_t;
    @brief      contains configuration data of display peripheral.
 **/
typedef struct
{
    hal_display_resolution_t    res;
} hal_display_cfg_t;


 
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const hal_display_cfg_t hal_display_cfg_default;   // = { .res = hal_display_res_320x240x24 };


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern hal_result_t hal_display_init(const hal_display_cfg_t *cfg)
    @brief      Initialise an external display. 
    @param      cfg         The configuration
    @return     hal_res_OK or hal_res_NOK_generic on failure or if the display is disabled by the base config.
    @warning    Use only for debug purposes on the MCBSTM32C board, where the display is a 320x240 SPI-mapped. 
 **/
extern hal_result_t hal_display_init(const hal_display_cfg_t *cfg);


/** @fn         extern hal_result_t hal_display_clear(hal_display_color_t color)
    @brief      Clear the whole display with color @e color.  
    @param      color       The color which teh display will be.
    @warning    Use only for debug purposes on the MCBSTM32C board
 **/
extern hal_result_t hal_display_clear(hal_display_color_t color);


/** @fn         extern hal_result_t hal_display_settext(hal_display_font_t fnt, hal_display_color_t backcolor, hal_display_color_t textcolor)
    @brief      Sets the display so that text uses a given font, a given background color and a given color.  
    @param      fnt         The font to be used.
    @param      backcolor   The background color of the text
    @param      textcolor   The color used for text.
    @warning    Use only for debug purposes on the MCBSTM32C board
 **/
extern hal_result_t hal_display_settext(hal_display_font_t fnt, hal_display_color_t backcolor, hal_display_color_t textcolor);


/** @fn         extern hal_result_t hal_display_clearline(uint32_t ln)
    @brief      Clear a complete line in teh dispalay.  
    @param      ln          The target line.
    @warning    Use only for debug purposes on the MCBSTM32C board
 **/
extern hal_result_t hal_display_clearline(uint32_t ln);


/** @fn         extern hal_result_t hal_display_putchar(uint32_t ln, uint8_t  c)
    @brief      Display a character on a given line 
    @param      ln          The target line.
    @param      c           The target character.
    @warning    Use only for debug purposes on the MCBSTM32C board
 **/
extern hal_result_t hal_display_putchar(uint32_t ln, uint8_t  c);


/** @fn         extern hal_result_t hal_display_putstring(uint32_t ln, uint8_t  *s)
    @brief      Display a null-terminated string on a given line 
    @param      ln          The target line.
    @param      s           The null-terminated string.
    @warning    Use only for debug purposes on the MCBSTM32C board
 **/
extern hal_result_t hal_display_putstring(uint32_t ln, uint8_t  *s);


/** @}            
    end of group arm_hal_display  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



