
/* @file       EOtheGPIOManager.c
    @brief      This file implements the GPIO manager.
    @author     marco.accame@iit.it
    @date       08/24/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOlist.h"
#include "EOiopin.h"
#include "EOioPinInputManaged.h"
#include "EOioPinOutputManaged.h"

#include "EOtheGPIO_hid.h"

#include "EOiopin_hid.h"
#include "EOioPinInputManaged_hid.h"
#include "EOioPinOutputManaged_hid.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheGPIOManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheGPIOManager_hid.h" 


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

static eOresult_t s_eo_gpioman_manage_outputfor(EOioPinOutputManaged *mo, eOreltime_t delta);
static void s_eo_gpioman_manage_inputfor(EOioPinInputManaged *mi, eOreltime_t delta);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtheGPIOManager";

static EOtheGPIOManager s_gpiomanager = 
{
    .activegpioout      = NULL,           // activegpioout
    .activegpioinp      = NULL,           // activegpioinp
    .mutex              = NULL,           // mutex
    .initted            = 0               // initted
}; 



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOtheGPIOManager * eo_gpioman_Initialise(EOtheGPIO *gpio, EOVmutexDerived *mutex) 
                                             
{
    uint8_t maxinp = 0;
    uint8_t maxout = 0;
    
    // trying to initialise the manager without the gpios ??
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != gpio), s_eobj_ownname, "EOtheGPIO is NULL");
     
    // already initted
    if(0 != s_gpiomanager.initted) 
    {
        // already initialised
        return(&s_gpiomanager);
    }
    
    // get number of input and output pin to manage
    maxinp = eo_gpio_hid_GetNumMngInp(gpio);
    maxout = eo_gpio_hid_GetNumMngOut(gpio);
    
    if(0 == (maxinp+maxout)) 
    {
        // no pin to manage 
        return(NULL);
    }
    
    // i get active input and output pin lists
    s_gpiomanager.activegpioout = (0 == maxout) ? (NULL) : eo_list_New(sizeof(EOioPinOutputManaged *), maxout, NULL, 0, NULL, NULL);
    s_gpiomanager.activegpioinp = (0 == maxinp) ? (NULL) : eo_list_New(sizeof(EOioPinInputManaged *), maxinp, NULL, 0, NULL, NULL);

    // i copy the mutex
    s_gpiomanager.mutex = mutex;
     
    // set initted to 1
    s_gpiomanager.initted = 1;
    
    // return the singleton handler
    return(&s_gpiomanager);
}    


extern EOtheGPIOManager * eo_gpioman_GetHandle(void) 
{ 
    return( (1 == s_gpiomanager.initted) ? (&s_gpiomanager) : (NULL) );   
}

  
extern eOresult_t eo_gpioman_Tick(EOtheGPIOManager *p, eOreltime_t delta) 
{
    eOresult_t res = eores_NOK_generic;

    EOlist *activepins = NULL;
    EOlistIter *li = NULL;
    EOlistIter *next = NULL;

    EOioPinOutputManaged *mo = NULL;
    EOioPinInputManaged *mi = NULL;
 

    // not yet initialised
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }

    if(0 == delta) 
    {
        // if called with delta equal to zero i return
         return(eores_NOK_generic);
    }

    // lock the gpio manager
    if(eores_NOK_timeout ==  eov_mutex_Take(p->mutex, eok_reltimeINFINITE)) 
    {
        return(eores_NOK_generic);
    }

    // manage the output pins
    activepins = p->activegpioout;

    if(NULL != activepins)
    {
    
        // i navigate the list from beginning to end until i find a NULL pointer
        li = eo_list_Begin(activepins);
        
        while(NULL != li) 
        {
            // get now the next list iterator in case we need to remove it inside this iteration
            next = eo_list_Next(activepins, li);
    
            // get the managed output pin
            mo = *((EOioPinOutputManaged**) eo_list_At(activepins, li));
            
            // extra safety. does it really need ???
            eo_errman_Assert(eo_errman_GetHandle(), eo_iopinStatDRIVEN == mo->iopin->status, s_eobj_ownname, "output pin is not a driven one");
    
            // make necessary computations upon values in mo->iopin->val etc...
            // if expired, remove item from list ...
            res = s_eo_gpioman_manage_outputfor(mo, delta);
            
            if(eores_NOK_generic == res) 
            {
                // need to remove current item because the managed output pin has reached maximum number of actions
                eo_list_Erase(activepins, li); 
            }
    
            // ok, go on with next list item
            li = next;
        }

    }

    // manage the input pins
    activepins = p->activegpioinp;

    if(NULL != activepins)
    {
        
        // i navigate the list from beginning to end until i find a NULL pointer
        li = eo_list_Begin(activepins);
    
        while(NULL != li) 
        {
            // get now the next list iterator in case we need to remove it
            next = eo_list_Next(activepins, li);
    
           // get the managed input pin
            mi = *((EOioPinInputManaged**) eo_list_At(activepins, li));
            
            // extra safety. does it really need ???
            eo_errman_Assert(eo_errman_GetHandle(), (eo_iopinStatDRIVEN == mi->iopin->status), s_eobj_ownname, "input pin is not a driven one");
    
            
            // make necessary computations for the input. the input never expires. can only
            // be removed by the user
            s_eo_gpioman_manage_inputfor(mi, delta);
    
            // ok, go on with next list item
            li = next;
        }

    }


    // unlock the gpio manager
    eov_mutex_Release(p->mutex);

    // return result
    return(eores_OK);
}



/* ------------------------------------------------------------------------------------
   definition of extern protected functions
   ------------------------------------------------------------------------------------
*/

extern eOresult_t eo_gpioman_hid_CanManagePin(EOtheGPIOManager* p, eOiopinDir_t dir)
{
    eOresult_t res = eores_NOK_generic;
    EOlist *activepins = NULL;
    
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eo_iopindirNONE == dir)
    {
        return(res);
    }    
    
    // lock the gpio manager
    if(eores_NOK_timeout ==  eov_mutex_Take(p->mutex, eok_reltimeINFINITE)) 
    {
        return(res);
    }
    
    activepins = (eo_iopindirINP == dir) ? (p->activegpioinp) : (p->activegpioout);
    
    if(eobool_false == eo_list_Full(activepins)) 
    {
        // the gpio manager can handle the pin ....
        res = eores_OK;
    }
    
    // unlock the gpio manager
    eov_mutex_Release(p->mutex);

    
    return(res);
}

extern eOresult_t eo_gpioman_hid_ManagePin(EOtheGPIOManager* p, EOioPinDerived *mp, eOiopinDir_t dir)
{
    eOresult_t res = eores_NOK_generic;
    EOlist *activepins = NULL;
    
    if((NULL == p) || (NULL == mp)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    if(eo_iopindirNONE == dir)
    {
        return(res);
    }     

    if(eores_OK != eo_gpioman_hid_CanManagePin(p, dir)) 
    {
        return(eores_NOK_busy);
    }

    // lock the gpio manager.    
    if(eores_NOK_timeout ==  eov_mutex_Take(p->mutex, eok_reltimeINFINITE)) 
    {
        return(res);
    }

    // i get the proper list
    activepins = (eo_iopindirINP == dir) ? (p->activegpioinp) : (p->activegpioout);
    
    // i add the pin to the list, but only if it is not already inside.
    if(NULL == eo_list_FindItem(activepins, &mp))
    {
        eo_list_PushFront(activepins, &mp);
    }

    // important: if output pin i set first value as the default value.
    if(eo_iopindirOUT == dir)
    {
        eo_iopin_SetVal(((EOioPinOutputManaged*)mp)->iopin, (eOiopinVal_t)((EOioPinOutputManaged*)mp)->iopin->defval);
    }
    
    // unlock the gpio manager
    eov_mutex_Release(p->mutex);
    

    return(eores_OK);    
}


extern eOresult_t eo_gpioman_hid_IsPinManaged(EOtheGPIOManager* p, EOioPinDerived *mp, eOiopinDir_t dir)
{
    eOresult_t res = eores_NOK_generic;
    EOlist *activepins = NULL;
    
    if((NULL == p) ||(NULL == mp))
    {
        return(eores_NOK_nullpointer);
    }

    if(eo_iopindirNONE == dir)
    {
        return(res);
    }        
    
    // lock the gpio manager
    if(eores_NOK_timeout ==  eov_mutex_Take(p->mutex, eok_reltimeINFINITE)) 
    {
        return(res);
    }

    
    // i get the proper list
    activepins = (eo_iopindirINP == dir) ? (p->activegpioinp) : (p->activegpioout);
    
    // find the iterator to the list item which contains the pin
    if(NULL != eo_list_FindItem(activepins, &mp))
    {   
        // the gpio manager is currently managing the pin 
        res = eores_OK;
    }

     
    // unlock the gpio manager
    eov_mutex_Release(p->mutex);

    
    return(res);
}



extern eOresult_t eo_gpioman_hid_UnManagePin(EOtheGPIOManager* p, EOioPinDerived *mp, eOiopinDir_t dir, uint8_t trigger)
{
    eOresult_t res = eores_NOK_generic;
    EOlist *activepins = NULL;
    EOlistIter *li = NULL;
    
    if((NULL == p) || (NULL == mp)) 
    {
        return(eores_NOK_nullpointer);    
    }

    // lock the gpio manager.   
    if(eores_NOK_timeout ==  eov_mutex_Take(p->mutex, eok_reltimeINFINITE)) 
    {
        return(res);
    }
    
    // i get the proper list
    activepins = (eo_iopindirINP == dir) ? (p->activegpioinp) : (p->activegpioout);
    
    // find the iterator to the list item which contains the pin
    li = eo_list_FindItem(activepins, &mp);

    if(NULL != li) 
    {
        // the pin still exists in the active list, thus ....
    
        // remove it from the active list.
        eo_list_Erase(activepins, li); 
        
        // the calling function will reset the pin
        
        // result value
        res = eores_OK;
    }
    else 
    {
        // this case should never happen if the calling function checks before.
        // however, we keep it for extra safety.
        
        // the calling function will reset the pin
        
        // result value
        res = eores_OK;  
    }
    
    
    // unlock the gpio manager
    eov_mutex_Release(p->mutex);

    return(res);    
}

extern eOresult_t eo_gpioman_hid_Take(EOtheGPIOManager *p, eOreltime_t tout)
{
    if(NULL == p)
    {
        return(eores_NOK_timeout);
    }
    
    return(eov_mutex_Take(p->mutex, tout));
}


extern eOresult_t eo_gpioman_hid_Release(EOtheGPIOManager *p)
{
    if(NULL == p)
    {
        return(eores_NOK_timeout);
    }
    
    return(eov_mutex_Release(p->mutex));
}    


// #warning "keep teh function because it can help in rmeoving only one action
#if 0
extern eOresult_t pro_eo_gpioman_RemoveActionOn(EOtheGPIOManager* p, gpioInpObj_t *gi, gpioTrigger_t trigger) {

    eOresult_t res = eores_NOK_generic;
    uint8_t index = 255;
    listItemObj_t *li = NULL;
    
    if((NULL == p) || (NULL == gi)) {
        return(eores_NOK_nullpointer);    
    }


    if(list_max_size(p->activegpioinp) == list_size(p->activegpioinp)) {
        // the gpio manager cannot handle any more gpios....
        return(res);
    }


     note very well in case you want to use this function.
    // the timer manager does not do teh initialisation of the object (in its case the timer), so, we should change later on.

    switch(trigger) {
        case gpeo_iopinTrig_OnRise:        index = 0; break; 
        case gpeo_iopinTrig_OnFall:        index = 1; break; 
        case gpeo_iopinTrig_AfterRise:    index = 2; break; 
        case gpeo_iopinTrig_AfterFall:    index = 3; break;
        default:                    return(res);
    } 

    if(255 == index) {
         return(res);
    }

    atomic_criticalsection_enter(); 

//    gi->pin.active                 = 1;
//    gi->pin.dir                    = gpioDirINP;
//    gi->pin.currval                = 2; // none
//    gi->counter                 = 0;
//    gi->after                    = after; // nota moltyo bene: attenzione. bisognerebbe modificarlo solo se abbiamo un low or high
//    gi->counting                = 0;
    gi->trigger                 &= (~trigger);

    gi->action[index].actiontype     = eo_actypeNONE;
    gi->action[index].event      = 0;
    gi->action[index].message     = 0;
    gi->action[index].callback    = NULL;
    gi->action[index].totask    = NULL;

    if(0 == gi->trigger) {
        // ok i remove it
        gi->pin.active = 0;
        li = list_iter_find(p->activegpioinp, gi);
        list_erase(p->activegpioinp, li);
    }

    
    atomic_criticalsection_exit();    
    

    return(eores_OK);    



}


#endif



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

 
static eOresult_t s_eo_gpioman_manage_outputfor(EOioPinOutputManaged *mo, eOreltime_t delta) 
{
    eOresult_t res = eores_OK;
    
    uint8_t firsthalf = 0;
    uint8_t keepon = 0;
    uint32_t currhalftime = 0;
    
    // see if i am in the first semi-wave or in the second one
    firsthalf       = (mo->iopin->val == mo->iopin->defval) ? (1) : (0);
    currhalftime    = (1 == firsthalf) ? (mo->ushalf01) : (mo->ushalf02-1); // -1 to keep dcewcdew
    
    // increment counter with current delta. 
    mo->counter += delta;
    
    // see if it is time to change
    if(mo->counter > currhalftime) 
    {
        // ok, we change value.
        eo_iopin_ToggleVal(mo->iopin); 
        // we reset counter 
        mo->counter = 0;
        // if in first half, we must do the other half as well
        if(1 == firsthalf) 
        {
            keepon = 1;
        }
        else 
        {
            // if we are in second half we decrement counter if not infinite repetions, and we check if we go on
            if(eok_reltimeINFINITE != mo->numwaves) 
            {
                mo->numwaves --;
            }

            keepon = (mo->numwaves > 0) ? (1) : (0);
            
        }
        
        if(0 == keepon) 
        {
            mo->iopin->status = eo_iopinStatIDLE;
            // tell the caller to remove the output pin
            res = eores_NOK_generic;
        }
        
    }

    
    return(res);
   
}




static void s_eo_gpioman_manage_inputfor(EOioPinInputManaged *mi, eOreltime_t delta) 
{

    eOiopinVal_t oldval = eo_iopinvalNONE;
    eOiopinVal_t newval = eo_iopinvalNONE;
    EOaction *acton_hit = NULL;
    EOaction *acton_stay = NULL;

    // 1. get values. use only current and previous value.
    oldval = (eOiopinVal_t)mi->iopin->defval; 
    newval = eo_iopin_GetVal(mi->iopin);            
    mi->iopin->defval = newval;


    // processing a possible change of level in the input pin
    if(oldval != newval) 
    {
    // ok, we have a change in the value. it can be a rise or a fall.

        // reset actions ...
        mi->counting    = 0;  // no count
        mi->counter     = 0;


        // ok, see what we must do.

        if(eo_iopinvalHIGH == newval) 
        {
        // it is a rise

            if(1 == mi->trigonrise_hit) 
            {
                 // ok... send a message for rising
                acton_hit = &mi->actionon[0];
            }
            
            if(1 == mi->trigonrise_stay)
            {
                 // start the timer for high stay
                mi->counting = 1; 
            } 

        }
        else 
        {
        // it is a fall

            if(1 == mi->trigonfall_hit) 
            {
                 // ok... send a message for falling
                acton_hit = &mi->actionon[1];
            }
            
            if(1 == mi->trigonfall_stay) 
            {
                 // start timer for low staty
                mi->counting = 1;  
            } 
        }

    }
    
    
    // processing possible counting
    if(1 == mi->counting) 
    {
        mi->counter += delta;

        if(eo_iopinvalHIGH == newval)
        {
        // i am high
            if(mi->counter > mi->afterrise) 
            {
                mi->counting = 0;
                mi->counter = 0;
                // message on high after ...
                acton_stay = &mi->actionon[2];
            }
        }
        else 
        {
        // i am low
            if(mi->counter > mi->afterfall) 
            {
                mi->counting = 0;
                mi->counter = 0;
                // message on low after ...
                acton_stay = &mi->actionon[3];
            }
        }
    }


    // processing possible actions: there can be at most one on hit and one on stay
    if(NULL != acton_hit)
    {
        eo_action_Execute(acton_hit, eok_reltimeZERO);

    }
    if(NULL != acton_stay)
    {
        eo_action_Execute(acton_stay, eok_reltimeZERO);
    }
    
}
 


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




