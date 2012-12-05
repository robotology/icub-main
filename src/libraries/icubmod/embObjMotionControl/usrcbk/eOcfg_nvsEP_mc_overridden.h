/*
 * pc104_mc_ovverride.h
 *
 *  Created on: Sep 17, 2012
 *      Author: icub
 */

#ifndef PC104_MC_OVVERRIDE_H_
#define PC104_MC_OVVERRIDE_H_


#ifdef EOCFG_NVSEP_MC_OVERRIDE
#define OVERRIDE_eo_cfg_nvsEP_mc_hid_UPDT_Mxx_mconfig
#define OVERRIDE_eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__pidposition
#define OVERRIDE_eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jstatus__basic
#define OVERRIDE_eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jconfig__controlmode
#define OVERRIDE_eo_cfg_nvsEP_mc_hid_UPDT_Jxx_jcmmnds__setpoint
#endif
#endif /* PC104_MC_OVVERRIDE_H_ */
