
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPIN_HID_H_
#define _EOIOPIN_HID_H_


/* @file       EOioPin_hid.h
    @brief      This header file implements hidden interface to a IO pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOioPin.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

#define EOIOPIN_VLOW            0
#define EOIOPIN_VHIGH           1
#define EOIOPIN_VNONE           3

#define EOIOPIN_DNONE           0
#define EOIOPIN_DINP            1
#define EOIOPIN_DOUT            2

#define EOIOPIN_SNOTMNG         0
#define EOIOPIN_SIDLE           1
#define EOIOPIN_SDRIVEN         2
#define EOIOPIN_SUNDEF          2


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EoIOPin_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOioPin_hid 
{
    // - vtable: must be on top of the struct
    // empty-section

    // other-stuff
    uint8_t         val:    2;      /**< value of the pin                                       **/
    uint8_t         dir:    2;      /**< direction of the pin                                   **/
    uint8_t         defval: 2;      /**< default value (not used ???)                           **/
    uint8_t         status: 2;      /**< status of the pin                                      **/
    uint8_t         halpin;         /**< hal pin                                                **/
    uint8_t         halport;        /**< hal port                                               **/
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


 
///** @fn         extern eOresult_t eo_iopin_hid_SetVal(EOioPin *const p, eOiopinVal_t val)
//    @brief      Sets the passed value to the EOioPin but only if the pin has the output direction
//    @param      p               The pointer to the EOioPin object.
//    @param      val             The target value
//    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the pin is not 
//                configured as output.
// **/
//extern eOresult_t eo_iopin_hid_SetVal(EOioPin *const p, eOiopinVal_t val);
//
//
///** @fn         extern eOresult_t eo_iopin_hid_ToggleVal(EOioPin *const p)
//    @brief      Toggles the value of the EOioPin only if the pin has the output direction and has value either
//                ioValHIGH or ioValLOW.
//    @param      p               The pointer to the EOioPin object.
//    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the pin is not 
//                configured as output or if its value is ioValNONE.
// **/
//extern eOresult_t eo_iopin_hid_ToggleVal(EOioPin *const p);
//
//
///**  @fn         extern eOiopinVal_t eo_iopin_hid_GetVal(EOioPin *const p)
//     @brief      Gets the value of the EOioPin. It works for both cases of input or output direction.
//     @param      p               The pointer to the EOioPin object.
//     @return     The current value or eo_iopinvalNONE if EOioPin is not yet configured or if p is NULL.
// **/
//extern eOiopinVal_t eo_iopin_hid_GetVal(EOioPin *const p);


///** @fn         extern eOresult_t eo_iopin_hid_SetCfg(EOioPin *const p, eOiopinPort_t port, eOiopinPos_t pos, eOiopinDir_t dir)
//    @brief      Configures the EOioPin with a given direction and maps it into a proper pin-port of the 
//                HAL GPIO device.
//    @param      p               The pointer to the EOioPin object.
//    @param      port            The HAL port.
//    @param      pos             The HAL pin.
//    @param      dir             The target direction.
//    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL.
// **/
//extern eOresult_t eo_iopin_hid_SetCfg(EOioPin *const p, eOiopinPort_t port, eOiopinPos_t pos, uint8_t val, eOiopinDir_t dir);
//
//
///** @fn         extern eOresult_t eo_iopin_hid_GetCfg(EOioPin *const p, eOiopinPort_t *pport, eOiopinPos_t *ppos, eOiopinDir_t *pdir)
//    @brief      Retrieves the configured direction and pin-port maps for the HAL gio device.
//    @param      p               The pointer to the EOioPin object.
//    @param      pdir            Pointer to the configured direction.
//    @param      ppin            Pointer to the HAL pin number.
//    @param      ppos            Pointer to the HAL port number.
//    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL.
// **/
//extern eOresult_t eo_iopin_hid_GetCfg(EOioPin *const p, uint8_t *pport, eOiopinPos_t *ppos, eOiopinDir_t *pdir);
//
//
///** @fn         extern eOiopinStatus_t eo_iopin_hid_GetStatus(EOioPin *const p)
//    @brief      Retrieves the status of the EOioPin object.
//    @param      p               The pointer to the EOioPin object.
//    @return     The current status of the pin.
// **/
//extern eOiopinStatus_t eo_iopin_hid_GetStatus(EOioPin *const p);
//
//
///** @fn         extern eOresult_t eo_iopin_hid_SetStatus(EOioPin *const p, eOiopinStatus_t st)
//    @brief      Sets the status of the EOioPin object.
//    @param      p               The pointer to the EOioPin object.
//    @param      st              The target status of the pin.
//    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL 
// **/
//extern eOresult_t eo_iopin_hid_SetStatus(EOioPin *const p, eOiopinStatus_t st);
//
//
///** @fn         extern eOresult_t eo_iopin_hid_Reset(EOioPin *const p)
//    @brief      Reset the status of the EOioPin object.
//    @param      p               The pointer to the EOioPin object.
//    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL 
// **/
//extern eOresult_t eo_iopin_hid_Reset(EOioPin *const p);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


