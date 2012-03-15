
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_EPS_REM_BOARD_H_
#define _EOCFG_EPS_REM_BOARD_H_




/** @file       eOcfg_EPs_rem_board.h
	@brief      This header file contains a const configuration for the local endpoints managed by a given board for remote use
	@author     marco.accame@iit.it
	@date       09/06/2011
**/

/** @defgroup eo_1234frr Configuation of the NVs for management of the ems board for use of teh remote host pc104
    Tcecece 
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOconstvector.h"
#include "EOnvsCfg.h"



// - public #define  --------------------------------------------------------------------------------------------------

// - in here there are the nvids plus ... 
#include "eOcfg_nvsEP_base_con.h"
#include "eOcfg_nvsEP_mngmnt_con.h"

// - in here there is access to ram of the endpoints
#include "eOcfg_nvsEP_base_usr_rem_anydev.h"
#include "eOcfg_nvsEP_mngmnt_usr_rem_board.h"


// - declaration of public user-defined types ------------------------------------------------------------------------- 


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

// EOconstvector where each element is a eOnvscfg_EP_t
extern const EOconstvector* const eo_cfg_EPs_vectorof_rem_board;

// if not NULL it contains a mapping from EPs to index inside eo_cfg_EPs_vectorof_rem_board
extern const eOuint16_fp_uint16_t eo_cfg_nvsEP_rem_board_fptr_hashfunction_ep2index;


// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section




/** @}            
    end of group eo_1234frr  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




