
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOIOPIN_H_
#define _EOIOPIN_H_


/** @file       EOioPin.h
    @brief      This header file implements public interface to a IO pin object.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/

/** @defgroup eo_iopin Object EOioPin
    The EOioPin object is the base representation of a GPIO pin and contains abstraction versus GPIO 
    hardware controller. To do that it uses the GPIO functions of the hal library which abstracts from the registers
    (or from the library offered by the silicon vendors).
    The EOioPin is used only as a base object for more specialised objects, such as simple input and output pins
    and such as managed input and output pins. 
    For this reason its creator eo_iopin_hid_New() is a hidden method. 
 
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


 

/** @typedef    typedef struct EOioPin_hid EOioPin
    @brief      EOioPin is an opaque struct. It is used to implement data abstraction for the io pin 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOioPin_hid EOioPin;

/** @typedef    typedef void EOioPinDerived
    @brief      EOioPinDerived used to is used to implement polymorphism in the objects derived from EOioPin
 **/
typedef void EOioPinDerived;


/** @typedef    typedef enum eOiopinPort_t 
    @brief      eOiopinPort_t contains the port values to which a pin can belong 
 **/
typedef enum  
{
    eo_iopinportA = 0,          
    eo_iopinportB, eo_iopinportC, eo_iopinportD, eo_iopinportE, eo_iopinportF, eo_iopinportG,
    eo_iopinportNONE = EOK_uint08dummy
} eOiopinPort_t;

/** @typedef    typedef enum eOiopinPos_t 
    @brief      eOiopinPos_t contains the position in the port which a pin can have 
 **/
typedef enum  
{
    eo_iopinpos0 = 0,          
    eo_iopinpos1, eo_iopinpos2, eo_iopinpos3, eo_iopinpos4, eo_iopinpos5, eo_iopinpos6, eo_iopinpos7, eo_iopinpos8, 
    eo_iopinpos9, eo_iopinpos10, eo_iopinpos11, eo_iopinpos12, eo_iopinpos13, eo_iopinpos14, eo_iopinpos15,
    eo_iopinposNONE = EOK_uint08dummy
} eOiopinPos_t;


/** @typedef    typedef enum eOiopinDir_t 
    @brief      eOiopinDir_t contains all possible directions that a pin GPIO can have.
 **/
typedef enum  
{
    eo_iopindirNONE = 0,        /**< Not a defined direction. It must be used very carefully and NEVER as function argument   */
    eo_iopindirINP  = 1,        /**< Input direction            */
    eo_iopindirOUT  = 2         /**< Output direction           */
} eOiopinDir_t;

 
/** @typedef    typedef enum eOiopinVal_t 
    @brief      eOiopinVal_t contains the values that a pin can have.
 **/
typedef enum  
{ 
    eo_iopinvalLOW  = 0,          /**< Logical 0 value        */
    eo_iopinvalHIGH = 1,          /**< Logical 1 value        */
    eo_iopinvalNONE = 3           /**< Not defined value. It must be used very carefully and NEVER as function argument   */
} eOiopinVal_t; 


/** @typedef    typedef enum eOiopinStatus_t 
    @brief      eOiopinStatus_t contains the status of a pin. 
 **/
typedef enum  
{
    eo_iopinStatNOTMNG = 0,   /**< Not managed                                    */
    eo_iopinStatIDLE   = 1,   /**< Manageable but idle (not currently driven)     */
    eo_iopinStatDRIVEN = 2,   /**< Manageable and driven                          */
    eo_iopinStatUNDEF  = 3    /**< For future use                                 */
} eOiopinStatus_t;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section



// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern EOioPin * eo_iopin_New(void)
    @brief      Creates a new EOioPin object. 
    @return     The pointer to the required object. The pointer is guaranteed to be always valid and never to be NULL, 
                because failure is managed by the memory pool.
 **/ 
extern EOioPin * eo_iopin_New(void);


/** @fn         extern eOresult_t eo_iopin_SetVal(EOioPin *p, eOiopinVal_t val)
    @brief      Sets the passed value to the EOioPin object but only if the pin has the output direction.
    @param      p               The pointer to the EOioPin object.
    @param      val             The target value
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the pin is not 
                configured as output.
 **/
extern eOresult_t eo_iopin_SetVal(EOioPin *p, eOiopinVal_t val);


/** @fn         extern eOresult_t eo_iopin_ToggleVal(EOioPin *p)
    @brief      Toggles the value of the EOioPin object only if the pin has the output direction and has 
                value either ioValHIGH or ioValLOW.
    @param      p               The pointer to the EOioPin object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the pin is not 
                configured as output or if its value is eo_iopinvalNONE.
 **/
extern eOresult_t eo_iopin_ToggleVal(EOioPin *p);


/** @fn         extern eOiopinVal_t eo_iopin_GetVal(EOioPin *p)
    @brief      Gets the value of the EOioPin object. It works for both cases of input or output direction.
    @param      p               The pointer to the EOioPin object.
    @return     The current value or eo_iopinvalNONE if EOioPin is not yet configured or if p is NULL.
 **/
extern eOiopinVal_t eo_iopin_GetVal(EOioPin *p);


/** @fn         extern eOresult_t eo_iopin_derived_SetVal(EOioPinDerived *p, eOiopinVal_t val)
    @brief      Sets the passed value to the EOioPin-derived object but only if the pin has the output direction.
    @param      p               The pointer to the EOioPin-derived object.
    @param      val             The target value
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the pin is not 
                configured as output.
 **/
extern eOresult_t eo_iopin_derived_SetVal(EOioPinDerived *p, eOiopinVal_t val);


/** @fn         extern eOresult_t eo_iopin_derived_ToggleVal(EOioPinDerived *p)
    @brief      Toggles the value of the EOioPin-derived object only if the pin has the output direction and has 
                value either ioValHIGH or ioValLOW.
    @param      p               The pointer to the EOioPin-derived object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the pin is not 
                configured as output or if its value is eo_iopinvalNONE.
 **/
extern eOresult_t eo_iopin_derived_ToggleVal(EOioPinDerived *p);


/** @fn         extern eOiopinVal_t eo_iopin_derived_GetVal(EOioPinDerived *p)
    @brief      Gets the value of the EOioPin-derived object. It works for both cases of input or output direction.
    @param      p               The pointer to the EOioPin-derived object.
    @return     The current value or eo_iopinvalNONE if EOioPin is not yet configured or if p is NULL.
 **/
extern eOiopinVal_t eo_iopin_derived_GetVal(EOioPinDerived *p);


/** @fn         extern eOresult_t eo_iopin_SetCfg(EOioPin *p, eOiopinPort_t port, eOiopinPos_t pos, eOiopinDir_t dir, eOiopinVal_t val)
    @brief      Configures the EOioPin with a given direction and maps it into a proper pin-port of the 
                HAL GPIO device.
    @param      p               The pointer to the EOioPin object.
    @param      port            The HAL port.
    @param      pos             The HAL pin.
    @param      dir             The target direction.
    @param      val             The target value
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL.
 **/
extern eOresult_t eo_iopin_SetCfg(EOioPin *p, eOiopinPort_t port, eOiopinPos_t pos, eOiopinDir_t dir, eOiopinVal_t val);


/** @fn         extern eOresult_t eo_iopin_GetCfg(EOioPin *const p, eOiopinPort_t *pport, eOiopinPos_t *ppos, eOiopinDir_t *pdir)
    @brief      Retrieves the configured direction and pin-port maps for the HAL gio device.
    @param      p               The pointer to the EOioPin object.
    @param      pdir            Pointer to the configured direction.
    @param      ppin            Pointer to the HAL pin number.
    @param      ppos            Pointer to the HAL port number.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL.
 **/
extern eOresult_t eo_iopin_GetCfg(EOioPin *p, uint8_t *pport, eOiopinPos_t *ppos, eOiopinDir_t *pdir);


/** @fn         extern eOiopinStatus_t eo_iopin_GetStatus(EOioPin *p)
    @brief      Retrieves the status of the EOioPin object.
    @param      p               The pointer to the EOioPin object.
    @return     The current status of the pin.
 **/
extern eOiopinStatus_t eo_iopin_GetStatus(EOioPin *p);


/** @fn         extern eOresult_t eo_iopin_SetStatus(EOioPin *p, eOiopinStatus_t st)
    @brief      Sets the status of the EOioPin object.
    @param      p               The pointer to the EOioPin object.
    @param      st              The target status of the pin.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL 
 **/
extern eOresult_t eo_iopin_hid_SetStatus(EOioPin *p, eOiopinStatus_t st);


/** @fn         extern eOresult_t eo_iopin_Reset(EOioPin *p)
    @brief      Reset the status of the EOioPin object.
    @param      p               The pointer to the EOioPin object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL 
 **/
extern eOresult_t eo_iopin_Reset(EOioPin *p);

 


/** @}            
    end of group eo_iopin  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




