
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHEIPNET_HID_H_
#define _EOMTHEIPNET_HID_H_


/* @file       EOMtheIPnet_hid.h
    @brief      This header file implements hidden interface to the rtos IP net singleton.
    @author     marco.accame@iit.it
    @date       08/24/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtheIPnet.h"
#include "EOMtask.h"
#include "EOaction.h"

#include "osal.h"
#include "ipal.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOMtheIPnet.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef enum
{   // contains all possible command types that can be sent to the EOMtask tskproc
    cmdDoNONE       = 0,
    cmdDoARP        = 1,
    cmdAttachDTG    = 2,
    cmdDetachDTG    = 3
} EOMtheIPnetCmdCode;

typedef struct  
{   // is the command sent to the task tskprocl
    osal_mutex_t            *mtxcaller;     // mutex used to stop other tasks to use the same command      
    EOMtheIPnetCmdCode      opcode;         // the opcode of the command
    uint8_t                 repeatcmd;      // flag used to tell the receiver of the command to process it againg
    uint8_t                 result;         // the result of the command: 1 is OK
    uint16_t                par16b;         // first parameter
    uint32_t                par32b;         // second parameter
    uint32_t                tout;           // the timeout of the command 
    osal_semaphore_t        *semaphore;     // used by the caller to block until the end of the command execution.
    EOtimer                 *stoptmr;       // used to drop the command after tout microsec even if the executer hasnt finished it
    EOaction                *stopact;       // the action to be done at expiry of the stop timer
} EOMtheIPnetCommand; 

/* @struct     EOMtheIPnet_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMtheIPnet_hid 
{ 
    // base object
    EOVtheIPnet                     *ipnet;                 /*< the base object */
    
    // other stuff
    EOMtask                         *tskproc;               /*< the main task whcih process tcp/ip traffic and commands from other tasks */
    EOMtask                         *tsktick;               /*< the task which ticks the tcp/ip stack for updating its internal timing */
    EOMtheIPnetCommand              cmd;                    /*< the command used by other tasks to issue request to the EOMtheIPnet */
    EOpacket                        *rxpacket;              /*< contains the temporary packet received by the network */
    eOreltime_t                     maxwaittime;            /*< a generic wait time */
    osal_messagequeue_t             *dgramsocketready2tx;   /*< a fifo containing pointers of the datagram sockets which need to tx a packet  */
    ipal_cfg_t               		ipcfg;                  /*< the ipal configuration */
    eObool_t                        taskwakeuponrxframe;    /*< tells to send an evt to tskproc when there is a received eth frame */
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

// name of the tasks as it is shown in uvision

void eom_ipnettick(void *p);

void eom_ipnetproc(void *p);

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




