
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHEIPNET_HID_H_
#define _EOVTHEIPNET_HID_H_


/* @file       EOVtheIPNet_hid.h
    @brief      This header file implements hidden interface to the base IP net singleton.
    @author     marco.accame@iit.it
    @date       08/24/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOlist.h"
#include "EOVmutex.h"
#include "EOVtask.h"
#include "EOsocketDatagram.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOVtheIPnet.h"

// - #define used with hidden struct ----------------------------------------------------------------------------------

#define VF00_attach                     0
#define VF01_detach                     1
#define VF02_alert                      2
#define VF03_arp                        3
#define VF04_waitpacket                 4
#define VTABLESIZE_ipnet                5

//#define VF05_gettask                    5
//#define VTABLESIZE_ipnet                6

// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef     eOresult_t  (*eOres_fp_ipnetp_voidp_t)          (EOVtheIPnet*, void *);
typedef     eOresult_t  (*eOres_fp_ipnetp_voidp_uint32_t)   (EOVtheIPnet*, void *, uint32_t);
typedef     eOresult_t  (*eOres_fp_ipnetp_voidp_evt_t)      (EOVtheIPnet*, void *, eOevent_t);
typedef     eOresult_t  (*eOres_fp_ipnetp_uint32_uint32_t)  (EOVtheIPnet*, uint32_t, uint32_t);
//typedef     void*       (*eOvoidp_fp_ipnetp_t)              (EOVtheIPnet*);

/* @struct     EOVtheIPnet_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOVtheIPnet_hid 
{
    // - vtable: must be on top of the struct
    void * vtable[VTABLESIZE_ipnet];

    // other stuff
    EOVtaskDerived                  *tsk;
    EOlist                          *activedgramsocksptrlist;   /*< list of pointers to active datagram sockets */     
    EOVmutexDerived                 *mutexactivedgram;          /*< mutex which guarantees exclusive access to the list */

}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOVtheIPnet * eov_ipnet_hid_Initialise(uint8_t maxdgramsocks, EoMutexDerived *mutex, 
                                                         eOres_fp_ipnetp_dsktp_t attach_fn, 
                                                         eOres_fp_ipnetp_dsktp_t detach_fn,
                                                         eOres_fp_ipnetp_voidp_evt_t alert_fn)
    @brief      Initialise the singleton. The function is hidden because this singleton can be used only
                by a derived object.
    @param      maxdgramsocks   The maximum number of datagram sockets that can be managed at the same time.
    @param      mutex           The mutex to be used.
    @param      attach_fn       The specific attach function
    @param      detach_fn       The specific detach function
    @param      altert_fn       The specific alert function
 
 **/

extern EOVtheIPnet * eov_ipnet_hid_Initialise(uint8_t maxdgramsocks, EOVmutexDerived *mutex,
                                              eOresult_t (*attach_fn)(EOVtheIPnet *n, EOsocketDerived *skt),
                                              eOresult_t (*detach_fn)(EOVtheIPnet *n, EOsocketDerived *skt),
                                              eOresult_t (*alert_fn)(EOVtheIPnet *n, void *eobjcaller, eOevent_t e),
                                              eOresult_t (*arp_fn)(EOVtheIPnet *n, uint32_t ipaddr, eOreltime_t tout),
                                              eOresult_t (*waitpacket_fn)(EOVtheIPnet *n, EOsocketDerived *skt, eOreltime_t tout)
                                              //void* (gettask_fn)(EOVtheIPnet *n)
                                              );



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

