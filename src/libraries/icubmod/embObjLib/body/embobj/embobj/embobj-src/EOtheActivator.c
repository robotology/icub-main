
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "stdio.h"
#include "EoCommon.h"
#include "EOtheErrorManager.h"
#include "EOVmutex.h"
#include "EOvectorMutex.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheActivator.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheActivator_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

 // --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOactivator_cfg_t eo_activator_DefaultCfg = 
{
    EO_INIT(.mutex)         NULL,
    EO_INIT(.capacity)      4
};



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

//static const char s_eobj_ownname[] = "EOtheActivator";

static EOtheActivator s_the_activator = 
{ 
    EO_INIT(.vector)        NULL,
    EO_INIT(.tout)          0,
    EO_INIT(.act)           EOACTION_DUMMY,
    EO_INIT(.onnumitems)    0,
    EO_INIT(.activated)     eobool_false
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOtheActivator * eo_activator_Initialise(const eOactivator_cfg_t *cfg)
{
    
    if(NULL == s_the_activator.vector)
    {
        // allow initialisation with null cfg .....
        if(NULL == cfg)
        {
            cfg = &eo_activator_DefaultCfg;
        }
        
        s_the_activator.vector = eo_vectorMutex_New(sizeof(eOactivator_objectinfo_t), cfg->capacity, NULL, NULL, NULL, NULL, cfg->mutex);
        
        s_the_activator.tout = eok_reltimeINFINITE;
        s_the_activator.onnumitems = 0;
        eo_action_Clear(&s_the_activator.act);
        s_the_activator.activated = eobool_false;

    }

    return(&s_the_activator);
}
 
    
extern EOtheActivator* eo_activator_GetHandle(void) 
{
    return((NULL!=s_the_activator.vector) ? (&s_the_activator) : (NULL));
}


extern eOresult_t eo_activator_SetActionOnNumItems(EOtheActivator *p, EOaction *act, eOsizecntnr_t onnumitems)
{
    eOresult_t res = eores_OK;
    if((NULL == p) || (NULL == act)) 
    {
        return(eores_NOK_nullpointer);
    }
    
    p->onnumitems = onnumitems;
    eo_action_Copy(&p->act, act);
            
    
    return(res);   
}


extern eOresult_t eo_activator_Register(EOtheActivator *p, eOactivator_objectinfo_t* objinfo)
{
    eOresult_t res = eores_OK;
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    res = eo_vectorMutex_PushBack(p->vector, objinfo, p->tout); 
    
    if((eores_OK == res) && ((eo_actypeNONE != eo_action_GetType(&p->act)))) 
    {
        eOsizecntnr_t size;
        eo_vectorMutex_Size(p->vector, &size, p->tout);
        
        if(size >= p->onnumitems)
        {
            eo_action_Execute(&p->act, eok_reltimeINFINITE);
        }     
    }    
    
    return(res);
}

extern eOresult_t eo_activator_ActivateAll(EOtheActivator *p)
{
    eOsizecntnr_t size, i;
    eOresult_t res = eores_OK;
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eores_OK != eo_vectorMutex_Size(p->vector, &size, p->tout))
    {
        return(eores_NOK_generic);
    }
    
    p->activated =  eobool_true;
    
    for(i=0; i<size; i++)
    {
        eOactivator_objectinfo_t *pitem = NULL;
        eo_vectorMutex_At(p->vector, i, (void**)&pitem, p->tout);
        if((NULL != pitem) && (NULL != pitem->activate))
        {
            pitem->activate(pitem->objptr);
        }
        else 
        {
            res = eores_NOK_generic;
        }
    } 

    return(res);    
}

extern eOresult_t eo_activator_DeactivateAll(EOtheActivator *p)
{
    eOsizecntnr_t size, i;
    eOresult_t res = eores_OK;
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eores_OK != eo_vectorMutex_Size(p->vector, &size, p->tout))
    {
        return(eores_NOK_generic);
    }
    
    p->activated =  eobool_false;
    
    for(i=0; i<size; i++)
    {
        eOactivator_objectinfo_t *pitem = NULL;
        eo_vectorMutex_At(p->vector, i, (void**)&pitem, p->tout);
        if((NULL != pitem) && (NULL != pitem->deactivate))
        {
            pitem->deactivate(pitem->objptr);
        }
        else 
        {
            res = eores_NOK_generic;
        }
    } 

    return(res);    
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





