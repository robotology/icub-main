
/* @file       EOsocket.c
    @brief      This file implements internal implementation of a base socket object.
    @author     marco.accame@iit.it
    @date       08/25/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOaction.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOsocket.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOsocket_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



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

//static const char s_eobj_ownname[] = "EOsocket";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOsocket* eo_socket_New(void)
{
    EOsocket *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOsocket), 1);

 
    // i create the two actions even if we may not use them
    retptr->onreception                     = eo_action_New();
    retptr->ontransmission                  = eo_action_New();
    eo_action_Clear(retptr->onreception);
    eo_action_Clear(retptr->ontransmission);

    retptr->type                            = eo_skttyp_none;
    retptr->localport                       = 0;
    retptr->skthandle                       = NULL;
    retptr->status                          = STATUS_SOCK_NONE;

    retptr->dir                             = eo_sktdir_TXRX;
    retptr->block2wait4packet               = eobool_false;
    retptr->blkgethandle                    = NULL;

    return(retptr);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



extern eOresult_t eo_socket_hid_derived_Prepare(EOsocketDerived *s, eOsocketType_t type, eOipv4port_t port, EOaction *onrec, EOaction *ontra, eOsocketDirection_t dir, eObool_t block2wait4packet)
{
    EOsocket *bs = (EOsocket*)eo_common_getbaseobject(s);
    
    if(NULL == bs)
    {
        return(eores_NOK_nullpointer);
    }
    
    bs->type                    = type;
    bs->localport               = port;
    bs->skthandle               = NULL;                 // reset it ...
    bs->status                  = STATUS_SOCK_NONE;     // reset it
    bs->dir                     = dir;
    bs->block2wait4packet       = block2wait4packet;
    // bs->blkgethandle MUST not be set to NULL, as this function is called at every open() of teh socket, whcih can be called
    // many times. and we dont want to have memory leaks ....
    // bs->blkgethandle            = bs->blkgethandle; // not NULL. it is set to NULL in the _New()

    // copy rx action into socket.
    if(NULL != onrec)
    {
        eo_action_Copy(bs->onreception, onrec);
    }
    else
    {
        eo_action_Clear(bs->onreception); 
    }


    // copy tx actions into socket.
    if(NULL != ontra)
    {
        eo_action_Copy(bs->ontransmission, ontra);
    }
    else
    {
        eo_action_Clear(bs->ontransmission); 
    }    
    
    
    return(eores_OK);
}




extern void * eo_socket_hid_derived_Get_Handle(EOsocketDerived *s)
{
    EOsocket *bs = (EOsocket*)eo_common_getbaseobject(s);
        
    if(NULL == bs)
    {
        return(NULL);
    }
    else
    {
        return(bs->skthandle);
    }     
}


extern eOsocketType_t eo_socket_hid_derived_Get_Type(EOsocketDerived *s)
{
    EOsocket *bs = (EOsocket*)eo_common_getbaseobject(s);
        
    if(NULL == bs)
    {
        return(eo_skttyp_none);
    }
    else
    {
        return(bs->type);
    }     
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




