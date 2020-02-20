/*
 * protocol_callback.c
 *
 *  Created on: Jun 6, 2013
 *  Author: Cardellino Alberto
 */


#include "FeatureInterface.h"

#include "EOYtheSystem.h"
#include "EoCommon.h"
#include "EOnv.h"
#include "EOnv_hid.h"
#include "EOrop.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "EoProtocolSK.h"

extern EOnvSet* arrayofnvsets[];

void boardtransceiver_fun_UPDT_mn_appl_cmmnds_go2state(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOmn_appl_state_t *newstate_ptr = (eOmn_appl_state_t *)rd->data;

    switch(*newstate_ptr)
    {
        case applstate_running:
        case applstate_config:
        case applstate_error:
        {
            //applstate = *newstate_ptr;
        } break;
    }
}

void boardtransceiver_fun_UPDT_mc_joint_cmmnds_interactionmode(const EOnv* nv, const eOropdescriptor_t* rd)
{
    EOnv_hid aNV = {0};
    eOnvBRD_t brd = eo_nv_GetBRD(nv);

    EOnvSet* mynvset = arrayofnvsets[brd];

    eOprotIndex_t index = eoprot_ID2index(rd->id32);
    eOenum08_t* pmode = (eOenum08_t*) rd->data;

    eOnvID32_t id32status = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, index, eoprot_tag_mc_joint_status);
    eo_nvset_NV_Get(mynvset, eok_ipv4addr_localhost, id32status, &aNV);

    eOmc_joint_status_t jointstatus = {0};
    uint16_t size = 0;

    eOresult_t res = eo_nv_Get(&aNV, eo_nv_strg_volatile, &jointstatus, &size);
    jointstatus.interactionmodestatus = *pmode;
    eo_nv_Set(&aNV, &jointstatus, eobool_true, eo_nv_upd_dontdo);

}



