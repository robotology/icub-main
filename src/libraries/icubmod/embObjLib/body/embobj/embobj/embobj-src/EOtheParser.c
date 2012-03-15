

/* @file       EOtheParser.c
    @brief      This file implements internal implementation of the parser singleton.
    @author     marco.accame@iit.it
    @date       09/06/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheErrorManager.h"
#include "EOnv.h"
#include "EOrop_hid.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheParser.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheParser_hid.h" 


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



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
static const char s_eobj_ownname[] = "EOtheParser";
#endif 

static EOtheParser eo_theparser = 
{
    EO_INIT(.initted)       0
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

 
extern EOtheParser * eo_parser_Initialise(void) 
{
    if(1 == eo_theparser.initted)
    {
        return(&eo_theparser);
    }
 
    eo_theparser.initted = 1;

    return(&eo_theparser);        
}    


extern EOtheParser * eo_parser_GetHandle(void) 
{
    return( (1 == eo_theparser.initted) ? (&eo_theparser) : (eo_parser_Initialise()) );
}


extern eOresult_t eo_parser_GetROP(EOtheParser *p, const uint8_t *streamdata, const uint16_t streamsize, const eOipv4addr_t ipaddr, EOrop *rop, uint16_t *consumedbytes)
{
    eOrophead_t *rophead            = NULL;
    uint8_t     *ropdata            = NULL;
    uint8_t     *roptail            = NULL;
#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
    eOnvType_t typ                  = eo_nv_TYP_NO4;
    eOnvFunc_t fun                  = eo_nv_FUN_NO0;
#endif
    uint16_t    dataeffectivesize   = 0; // multiple of four
    uint16_t    signeffectivesize   = 0;
    uint16_t    timeeffectivesize   = 0;

    if((NULL == p) || (NULL == streamdata) || (NULL == rop) || (NULL == consumedbytes))
    {
        return(eores_NOK_nullpointer);
    }

    // reset return data: rop and consumed bytes
    eo_rop_Reset(rop);
    *consumedbytes = 0;

    if(streamsize < sizeof(eOrophead_t))
    {
        return(eores_NOK_generic);
    }

    // get the head of the rop with ctrl, ropc, endp, nvid, dsiz. 
    // for now the roptail is just after the four bytes of the head
    rophead = (eOrophead_t*)(&streamdata[0]);
    roptail = (uint8_t*)(&streamdata[4]);
    roptail = roptail;  // there is this instruction to force roptail to have its correct value in debugger

    // check validity of ctrl
    if(1 == rophead->ctrl.userdefn)
    {
       // not managed yet
        return(eores_NOK_generic);
    }

    // check validity of ropc 
    if((rophead->ropc >= (uint8_t)eo_ropcodevalues_numberofthem) || (rophead->ropc == (uint8_t)eo_ropcode_none))
    {
        return(eores_NOK_generic);
    }
    else if((rophead->ropc == (uint8_t)eo_ropcode_usr))
    {
        // not managed yet
        return(eores_NOK_generic);
    }

#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
    // check validity of nvid
    fun = eo_nv_hid_fromIDtoFUN(rophead->nvid);
    typ = eo_nv_hid_fromIDtoTYP(rophead->nvid);
    if((eo_nv_FUN_NO0 == fun) || (eo_nv_FUN_NO1 == fun) || (eo_nv_TYP_NO4 == typ) ||(eo_nv_TYP_NO5 == typ))
    {
        // incorrect nvid
        return(eores_NOK_generic);
    }
#endif

    // some ropcodes also have a data field
    if(eobool_true == eo_rop_hid_DataField_is_Present(rophead))
    {
        ropdata = (uint8_t*)(&streamdata[sizeof(eOrophead_t)]);
    }

    // in case there is data field, verify it, and sets its effective length 
    // remember that size and info must occupy 4, 8, 12, etc bytes.
    if(NULL != ropdata)
    {

#if defined(EO_NV_EMBED_FUNTYP_IN_ID)
        // at first we verify that the type of nv and the dsiz fields are consistent
        if((typ < eo_nv_TYP_arr) && ((1<<typ) != rophead->dsiz))
        {
            // incorrect nvid and data.size fields
            eo_errman_Error(eo_errman_GetHandle(), eo_errortype_warning, s_eobj_ownname, "incorrect nvid and head.dsiz fields");
            return(eores_NOK_generic);
        }
#endif
        // then we compute the effective size of data as the next multiple of four of dsiz
        dataeffectivesize = eo_rop_hid_DataField_EffectiveSize(rophead->dsiz);

        // the roptail is now after the bytes of the head and the bytes of the data
        roptail = (uint8_t*)(&streamdata[sizeof(eOrophead_t) + dataeffectivesize]);
    
    }

    // in case there is sign and/or time presence, then sets the correct sizes

    if(1 == rophead->ctrl.plussign)
    {
        signeffectivesize = 4;
    }

    if(1 == rophead->ctrl.plustime)
    {
        timeeffectivesize = 8;
    }

    // if we dont have enough bytes in the packet ... leave with an error
    if(streamsize < (sizeof(eOrophead_t) + dataeffectivesize + signeffectivesize + timeeffectivesize))
    {
        //  not enough bytes in the passed packet to keep the data sugegsted by the dsiz
        return(eores_NOK_generic);
    }

    // else ... fill the rop w/ the acquired info

    *consumedbytes = sizeof(eOrophead_t) + dataeffectivesize + signeffectivesize + timeeffectivesize;

    // copy head
    memcpy(&rop->head, rophead, sizeof(eOrophead_t));

    // copy data
    if(NULL != ropdata)
    {
        rop->head.dsiz = rophead->dsiz;
        memcpy(rop->data, ropdata, rophead->dsiz);
    }
    
    // copy the signature
    if(0 != signeffectivesize)
    {
        rop->sign = *( (uint32_t*) &roptail[0] );
    }

    // copy the time
    if(0 != timeeffectivesize)
    {
        rop->time = *( (uint64_t*) &roptail[signeffectivesize] );
    }  
    
    // sets the ipaddr
    rop->aboutip.ipaddr = ipaddr; 

    // sets the ownership
    rop->aboutnvs.nvownership = eo_rop_hid_GetOwnership(rophead->ropc, (eOropconfinfo_t)rophead->ctrl.confinfo, eo_rop_dir_received);

    // do ???
    rop->aboutdata.index    = 0;
    
    return(eores_OK);
}





// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




