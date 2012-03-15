

/* @file       EOtheGPIO.c
    @brief      This file implements internal implementation of the gpio singleton.
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOioPin_hid.h" 
#include "EOioPinInput_hid.h" 
#include "EOioPinOutput_hid.h" 
#include "EOioPinInputManaged_hid.h" 
#include "EOioPinOutputManaged_hid.h" 





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheGPIO.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheGPIO_hid.h" 


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

static eOresult_t s_eo_gpio_VerifyCfg(const eOgpio_cfg_t * const p);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtheGPIO";

static EOtheGPIO s_thegpio = 
{    
    .cfg        = NULL,      
    .inp        = NULL,       // inp 
    .out        = NULL,       // out 
    .mnginp     = NULL,       // mnginp
    .mngout     = NULL,       // mngout 
};



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

 
extern EOtheGPIO * eo_gpio_Initialise(const eOgpio_cfg_t * const cfg) 
{
    uint8_t i = 0;
    
    if(NULL != s_thegpio.cfg)
    {
        return(&s_thegpio);
    }
    
    // verify cfg is not NULL
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != cfg), s_eobj_ownname, "cfg is NULL");

    
    // verify consistency of the user-defined cfg data structure. humans can fail!
    s_eo_gpio_VerifyCfg(cfg);
     
     
    s_thegpio.cfg = cfg;

    s_thegpio.inp           = (0 == s_thegpio.cfg->ninp) ? (NULL) : eo_iopininp_hid_NewArray(s_thegpio.cfg->ninp); 

    for(i=0; i<s_thegpio.cfg->ninp; i++) 
    {
        eo_iopin_SetCfg(s_thegpio.inp[i].iopin, 
                              s_thegpio.cfg->inp_map[i].port, s_thegpio.cfg->inp_map[i].pos, eo_iopindirINP, s_thegpio.cfg->inp_map[i].val);
    }


    s_thegpio.out           = (0 == s_thegpio.cfg->nout) ? (NULL) : eo_iopinout_hid_NewArray(s_thegpio.cfg->nout); 

    for(i=0; i<s_thegpio.cfg->nout; i++) 
    {
        eo_iopin_SetCfg(s_thegpio.out[i].iopin, 
                               s_thegpio.cfg->out_map[i].port, s_thegpio.cfg->out_map[i].pos, eo_iopindirOUT, s_thegpio.cfg->out_map[i].val);
    }


    s_thegpio.mnginp        = (0 == s_thegpio.cfg->nmnginp) ? (NULL) : eo_iopininpman_hid_NewArray(s_thegpio.cfg->nmnginp); 

    for(i=0; i<s_thegpio.cfg->nmnginp; i++) 
    {
        eo_iopin_SetCfg(s_thegpio.mnginp[i].iopin, 
                               s_thegpio.cfg->mnginp_map[i].port, s_thegpio.cfg->mnginp_map[i].pos, eo_iopindirINP, s_thegpio.cfg->mnginp_map[i].val);
    }

    s_thegpio.mngout        = (0 == s_thegpio.cfg->nmngout) ? (NULL) : eo_iopinoutman_hid_NewArray(s_thegpio.cfg->nmngout); 

    for(i=0; i<s_thegpio.cfg->nmngout; i++) 
    {
        eo_iopin_SetCfg(s_thegpio.mngout[i].iopin, 
                               s_thegpio.cfg->mngout_map[i].port,  s_thegpio.cfg->mngout_map[i].pos, eo_iopindirOUT, s_thegpio.cfg->mngout_map[i].val);
    }

    
    return(&s_thegpio);        
} 


extern EOtheGPIO * eo_gpio_GetHandle(void) 
{
    return( (NULL != s_thegpio.cfg) ? (&s_thegpio) : (NULL) );
}   


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------




extern uint8_t eo_gpio_hid_GetNumMngInp(EOtheGPIO *const p) 
{
    return( (NULL == p) ? (0) : (p->cfg->nmnginp) );
}

extern uint8_t eo_gpio_hid_GetNumMngOut(EOtheGPIO *const p) 
{
    return( (NULL == p) ? (0) : (p->cfg->nmngout) );
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eo_gpio_VerifyCfg(const eOgpio_cfg_t * const p)
{
    uint8_t i = 0;
 

	for(i=0; i<p->ninp; i++)
	{
		eo_errman_Assert(eo_errman_GetHandle(), i == p->inp_map[i].id, s_eobj_ownname, "incorrect pin mapping");	
	}
    eo_errman_Assert(eo_errman_GetHandle(), eok_uint08dummy == p->inp_map[p->ninp].id, s_eobj_ownname, "incorrect pin mapping");
	
	for(i=0; i<p->nout; i++)
	{
		eo_errman_Assert(eo_errman_GetHandle(), i == p->out_map[i].id, s_eobj_ownname, "incorrect pin mapping");	
	}
    eo_errman_Assert(eo_errman_GetHandle(), eok_uint08dummy == p->out_map[p->nout].id, s_eobj_ownname, "incorrect pin mapping");    
    
	for(i=0; i<p->nmnginp; i++)
	{
		eo_errman_Assert(eo_errman_GetHandle(), i == p->mnginp_map[i].id, s_eobj_ownname, "incorrect pin mapping");	
	}
    eo_errman_Assert(eo_errman_GetHandle(), eok_uint08dummy == p->mnginp_map[p->nmnginp].id, s_eobj_ownname, "incorrect pin mapping");    
    
	for(i=0; i<p->nmngout; i++)
	{
		eo_errman_Assert(eo_errman_GetHandle(), i == p->mngout_map[i].id, s_eobj_ownname, "incorrect pin mapping");	
	}
    eo_errman_Assert(eo_errman_GetHandle(), eok_uint08dummy == p->mngout_map[p->nmngout].id, s_eobj_ownname, "incorrect pin mapping");        
    
 
    return(eores_OK);
}




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




