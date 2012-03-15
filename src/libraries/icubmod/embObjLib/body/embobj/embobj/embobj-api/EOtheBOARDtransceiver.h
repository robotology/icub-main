

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEBOARDTRANSCEIVER_H_
#define _EOTHEBOARDTRANSCEIVER_H_


/** @file       EOtheBOARDtransceiver.h
    @brief      This header file implements public interface to board transceiver
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_ecvrevrebvtr2342r4 Object EOtheBOARDtransceiver
    The EOtheBOARDtransceiver is a singleton .....
      
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtransceiver.h"


// - public #define  --------------------------------------------------------------------------------------------------

#define EOK_BOARDTRANSCEIVER_capacityofpacket                   512
#define EOK_BOARDTRANSCEIVER_capacityofrop                      128
#define EOK_BOARDTRANSCEIVER_capacityofropframeregulars         256
#define EOK_BOARDTRANSCEIVER_capacityofropframeoccasionals      128
#define EOK_BOARDTRANSCEIVER_capacityofropframereplies          128 
#define EOK_BOARDTRANSCEIVER_maxnumberofregularrops             16 

// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef struct
{
    const EOconstvector* const  vectorof_endpoint_cfg;
    eOuint16_fp_uint16_t        hashfunction_ep2index;
    eOipv4addr_t                remotehostipv4addr;
    eOipv4port_t                remotehostipv4port;
    uint8_t                     tobedefined; // ip, port, numendpoints ... or simply: local ip is retrieved by eeprom, remote ip from
                             // first reception, port is fixed, numendpoints and an constvector of endpoint config
                             // is store in a file containing a variable pointer to const data. 
} eOboardtransceiver_cfg_t;


/** @typedef    typedef struct EOtheBOARDtransceiver_hid EOtheBOARDtransceiver
    @brief      EOtheBOARDtransceiver is an opaque struct. It is used to implement data abstraction for the Parser  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtheBOARDtransceiver_hid EOtheBOARDtransceiver;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOboardtransceiver_cfg_t eo_boardtransceiver_cfg_default; // = {NULL};


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
 
/** @fn         extern EOtheBOARDtransceiver * eo_boardtransceiver_Initialise(void)
    @brief      Initialise the singleton EOtheBOARDtransceiver. 
    @param      cfg         Contains actions to be done on reception or transmission which are specific to the application.
                            If NULL, then  is is issued a info by the EOtheErrorManager.
    @return     A valid and not-NULL pointer to the EOtheBOARDtransceiver singleton.
 **/
extern EOtransceiver * eo_boardtransceiver_Initialise(const eOboardtransceiver_cfg_t *cfg);


/** @fn         extern EOtheBOARDtransceiver * eo_boardtransceiver_GetHandle(void)
    @brief      Gets the handle of the EOtheBOARDtransceiver singleton 
    @return     Constant pointer to the singleton.
 **/
extern EOtransceiver * eo_boardtransceiver_GetHandle(void);



/** @}            
    end of group eo_ecvrevrebvtr2342r4  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



