
/* @file       dspal_cfg.c
	@brief      This file keeps internal implementation of the fsal.
	@author     marco.accame@iit.it
    @date       02/25/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "dspal.h"
#include "string.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "dspal_cfg.h"


// defines

// static values

static const dspal_params_cfg_t s_cfg = 
{   
    .dsplibtype             = DSPAL_DSPLIBTYPE,
    .cpufam                 = DSPAL_CPUFAM,
    .extfn                  =
    {
        .usr_on_fatal_error     = NULL
    }      
};


extern const dspal_params_cfg_t *dspal_params_cfgMINE = &s_cfg;








// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



