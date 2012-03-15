
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_NVSEP_MNGMNT_USR_LOC_BOARD_H_
#define _EOCFG_NVSEP_MNGMNT_USR_LOC_BOARD_H_




/** @file       eOcfg_nvsEP_mngmnt_usr_loc_board.h
	@brief      This header file gives the local configuration for the NVs in the management port of any board
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_sdfwervcer Local configuation of the management NVs for the board (an ems or mcbstm2x)
    Tcecece 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOconstvector.h"
#include "eOcfg_nvsEP_mngmnt_con.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section


// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const EOconstvector* const eo_cfg_nvsEP_mngmnt_usr_loc_board_constvector_of_EOnv_usr;

extern eo_cfg_nvsEP_mngmnt_t* eo_cfg_nvsEP_mngmnt_usr_loc_board_mem_local;



// - declaration of extern public functions ---------------------------------------------------------------------------

extern void eo_cfg_nvsEP_mngmnt_usr_loc_board_initialise(void* loc, void* rem);


/** @}            
    end of group eo_sdfwervcer  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




