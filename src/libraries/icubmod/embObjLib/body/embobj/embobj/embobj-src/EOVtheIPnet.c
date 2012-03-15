
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"

#include "EOsocketDatagram.h"

#include "EOtheErrorManager.h"
#include "EOtheMemoryPool.h"

#include "ipal.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheIPnet.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheIPnet_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

extern const eOevent_t  eok_ipnet_evt_RXipframe         = 0x00000001;  
extern const eOevent_t  eok_ipnet_evt_CMD2process       = 0x00000002;  
extern const eOevent_t  eok_ipnet_evt_CMD2stop          = 0x00000004;  
extern const eOevent_t  eok_ipnet_evt_TXdatagram        = 0x00000008; 
extern const eOevent_t  eok_ipnet_evt_evalRXipframe     = 0x00000010;  
//extern const eOevent_t  eok_ipnet_evt_TXstream          = 0x00000010;  





// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOVtheIPnet";

static EOVtheIPnet s_ipnet = 
{
    .vtable                     = {NULL},         // vtable: attach, detach, alert, arp, waitpacket
    .tsk                        = NULL,           // tsk
    .activedgramsocksptrlist    = NULL,           // activedgramsocksptrlist
    .mutexactivedgram           = NULL            // mutexactivedgram
}; 


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOVtheIPnet* eov_ipnet_GetHandle(void) 
{
    if(NULL == s_ipnet.vtable[0])
    {    
        return(NULL);
    }
    
    return(&s_ipnet);
}




extern eOresult_t  eov_ipnet_AttachSocket(EOVtheIPnet* p, EOsocketDerived *s)
{
    eOres_fp_ipnetp_voidp_t fptr;

    if((NULL == p) || (NULL == s)) 
    {
        return(eores_NOK_nullpointer);    
    }

    fptr = (eOres_fp_ipnetp_voidp_t)p->vtable[VF00_attach];

    // if p is not NULL, ftpr cant be NULL because we have verified that in eov_ipnet_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, s));
}


extern eOresult_t eov_ipnet_DetachSocket(EOVtheIPnet* p, EOsocketDerived *s)
{
    eOres_fp_ipnetp_voidp_t fptr;

    if((NULL == p) || (NULL == s)) 
    {
        return(eores_NOK_nullpointer);    
    }

    fptr = (eOres_fp_ipnetp_voidp_t)p->vtable[VF01_detach];

    // if p is not NULL, ftpr cant be NULL because we have verified that in eov_ipnet_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, s));
}


extern eOresult_t eov_ipnet_Alert(EOVtheIPnet* p, void *eobjcaller, eOevent_t evt)
{
    eOres_fp_ipnetp_voidp_evt_t fptr;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }

    fptr = (eOres_fp_ipnetp_voidp_evt_t)p->vtable[VF02_alert];

    // alert can be NULL
    if(NULL == fptr) 
    {
         return(eores_NOK_generic);
    }

    return(fptr(p, eobjcaller, evt));
}


extern eOresult_t eov_ipnet_ResolveIP(EOVtheIPnet* p, eOipv4addr_t ipaddr, eOreltime_t tout)
{
    eOres_fp_ipnetp_uint32_uint32_t fptr;

    if(NULL == p)  
    {
        return(eores_NOK_nullpointer);    
    }

    fptr = (eOres_fp_ipnetp_uint32_uint32_t)p->vtable[VF03_arp];

    // if p is not NULL, ftpr cant be NULL because we have verified that in eov_ipnet_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, ipaddr, tout));  
}

extern eOpurevirtual eOresult_t eov_ipnet_WaitPacket(EOVtheIPnet* p, EOsocketDerived *s, eOreltime_t tout)
{
   eOres_fp_ipnetp_voidp_uint32_t fptr;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }

    fptr = (eOres_fp_ipnetp_voidp_uint32_t)p->vtable[VF04_waitpacket];

    // wait packet can be NULL
    if(NULL == fptr) 
    {
         return(eores_NOK_generic);
    }

    return(fptr(p, s, tout));
}


//extern eOpurevirtual EOVtaskDerived* eov_ipnet_GetTask(EOVtheIPnet* p)
//{
//    eOvoidp_fp_ipnetp_t fptr;
//
//    if(NULL == p) 
//    {
//        return(NULL);    
//    }
//    
//    // if p is not NULL, ftpr can be NULL because we have verified that in eov_ipnet_hid_Initialise(), thus ...
//    // just call the method initialised by the derived object
//    fptr = (eOvoidp_fp_ipnetp_t)p->vtable[VF05_gettask];
//
//    return(fptr(p));
//}

extern eOresult_t eov_ipnet_IGMPgroupJoin(EOVtheIPnet* p, eOipv4addr_t igmpgroup)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    

    // no need to ask the ipnet to send a message, no need to manage tasks, just a direct call to
    // the ipal that can be made also by the virtual object. thus i do it immediately. just 
    // check validity of the igmp.

    // acemor-facenda: put a control on highest four bits. they must be 1110 (0xE).
    // also: make 224.0.0.0 and 224.0.0.1 not available.

    if((0x000000E0 & igmpgroup) != 0x000000E0)
    {
        return(eores_NOK_generic);
    }

    if((0x000000E0 == igmpgroup) || (0x010000E0 == igmpgroup))
    {
        return(eores_NOK_generic);
    }

    return((eOresult_t)ipal_igmp_join(igmpgroup));
   
}

extern eOresult_t eov_ipnet_IGMPgroupLeave(EOVtheIPnet* p, eOipv4addr_t igmpgroup)
{
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    

    // no need to ask the ipnet to send a message, no need to manage tasks, just a direct call to
    // the ipal that can be made also by the virtual object. thus i do it immediately. just 
    // check validity of the igmp.

    // acemor-facenda: put a control on highest four bits. they must be 1110 (0xE).
    // also: make 224.0.0.0 and 224.0.0.1 not available.


    if((0x000000E0 & igmpgroup) != 0x000000E0)
    {
        return(eores_NOK_generic);
    }

    if((0x000000E0 == igmpgroup) || (0x010000E0 == igmpgroup))
    {
        return(eores_NOK_generic);
    }

    return((eOresult_t)ipal_igmp_leave(igmpgroup));
   
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



extern EOVtheIPnet * eov_ipnet_hid_Initialise(uint8_t maxdgramsocks, EOVmutexDerived *mutex,
                                              eOresult_t (*attach_fn)(EOVtheIPnet *n, EOsocketDerived *skt),
                                              eOresult_t (*detach_fn)(EOVtheIPnet *n, EOsocketDerived *skt),
                                              eOresult_t (*alert_fn)(EOVtheIPnet *n, void *eobjcaller, eOevent_t e),
                                              eOresult_t (*arp_fn)(EOVtheIPnet *n, uint32_t ipaddr, eOreltime_t tout),
                                              eOresult_t (*waitpacket_fn)(EOVtheIPnet *n, EOsocketDerived *skt, eOreltime_t tout)
//                                              void* (gettask_fn)(EOVtheIPnet *n)
                                              )
{
    if(NULL != s_ipnet.vtable[0]) 
    {
        // already initialised
        return(&s_ipnet);
    }
    
    // trying to initialise with no sockets ? we can do it ... but better to inform someone.
    if(0 == maxdgramsocks)
    {
        eo_errman_Info(eo_errman_GetHandle(), s_eobj_ownname, "... ZERO EOsocketDatagram are initted"); 
    }
    
    // of the vtable alert can be NULL.
    eo_errman_Assert(eo_errman_GetHandle(), NULL != attach_fn, s_eobj_ownname, "attach_fn() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != attach_fn, s_eobj_ownname, "detach_fn() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != arp_fn, s_eobj_ownname, "arp_fn() is NULL");
//    eo_errman_Assert(eo_errman_GetHandle(), NULL != gettask_fn, s_eobj_ownname, "gettask_fn() is NULL");

    // initialise vtable
    s_ipnet.vtable[VF00_attach]              = attach_fn;
    s_ipnet.vtable[VF01_detach]              = detach_fn;
    s_ipnet.vtable[VF02_alert]               = alert_fn;
    s_ipnet.vtable[VF03_arp]                 = arp_fn;
    s_ipnet.vtable[VF04_waitpacket]          = waitpacket_fn;
//    s_ipnet.vtable[VF05_gettask]             = gettask_fn;
 
    // i get activedgramsocksptrlist list. it keeps pointers to socket objects.
    s_ipnet.activedgramsocksptrlist = (0 == maxdgramsocks) ? (NULL) : (eo_list_New(sizeof(EOsocketDatagram *), maxdgramsocks, NULL, 0, NULL, NULL));

    // i copy the mutex
    s_ipnet.mutexactivedgram = mutex;


    return(&s_ipnet);
}    


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




