

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEAGENT_H_
#define _EOTHEAGENT_H_


/** @file       EOtheAgent.h
    @brief      This header file implements public interface to the parser singleton used for communication protocol
    @author     marco.accame@iit.it
    @date       09/06/2011
**/

/** @defgroup eo_theagent Object EOtheAgent
    The EOtheAgent is a singleton which is only responsible to evaluate the content of a received EOrop
    and to execute it and to prepare a EOrop for transmission.
      
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOrop.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOtheAgent_hid EOtheAgent
    @brief      EOtheAgent is an opaque struct. It is used to implement data abstraction for the Parser  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtheAgent_hid EOtheAgent;



typedef struct
{
    void (*on_rop_conf_received)(eOipv4addr_t ipaddr, eOropcode_t ropc, eOnvID_t nvid, uint32_t sign, eOabstime_t time, eOropconfinfo_t confinfo);
    void (*on_rop_conf_requested)(eOipv4addr_t ipaddr, eOropcode_t ropc, eOnvID_t nvid, uint32_t sign, eOabstime_t time);
} eOagent_cfg_t;
 

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOagent_cfg_t eo_agent_cfg_default; // = {NULL, NULL};


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
 
/** @fn         extern EOtheAgent * eo_agent_Initialise(void)
    @brief      Initialise the singleton EOtheAgent. 
    @param      cfg         Contains actions to be done on reception or transmission which are specific to the application.
                            If NULL, then  is is issued a info by the EOtheErrorManager.
    @return     A valid and not-NULL pointer to the EOtheAgent singleton.
 **/
extern EOtheAgent * eo_agent_Initialise(const eOagent_cfg_t *cfg);



/** @fn         extern EOtheAgent * eo_agent_GetHandle(void)
    @brief      Gets the handle of the EOtheAgent singleton 
    @return     Constant pointer to the singleton.
 **/
extern EOtheAgent * eo_agent_GetHandle(void);



// receives a rop, it may call on_rop_confirmation_received(), it may execute some actions related to the rop,
// it may produce a rop in output. the rop in output must not be fed to eo_agent_OutROPinit() or eo_agent_OutROPfill()
extern eOresult_t eo_agent_InpROPprocess(EOtheAgent *p, EOnvsCfg* nvscfg, EOrop *r, EOrop *replyrop, eOipv4addr_t fromipaddr);


// prepare a rop by taking data from the proper nv (or from argument ...), it may call on_rop_confirmation_requested()  
extern eOresult_t eo_agent_OutROPinit(EOtheAgent *p, EOnvsCfg* nvscfg, eOipv4addr_t toipaddr, eOipv4port_t toport, eOropcode_t ropc, eOnvEP_t endpoint, eOnvID_t nvid, eOropconfig_t ropcfg, EOrop *rop, uint16_t *requiredbytes);

// updates an existing rop
extern eOresult_t eo_agent_OutROPfill(EOtheAgent *p, EOrop *rop, uint32_t *sign, eOabstime_t *time);



/** @}            
    end of group eo_theagent  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



