
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOROP_HID_H_
#define _EOROP_HID_H_


/* @file       EOrop_hid.h
    @brief      This header file implements hidden interface to a rop object.
    @author     marco.accame@iit.it
    @date       09/06/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOnv_hid.h"
#include "EOtreenode.h"
#include "EOnvsCfg.h"

//typedef EOtreenode EOnvnodeTMP;

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOrop.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------





typedef struct
{
    uint8_t         ffu         : 1;
    uint8_t         confinfo    : 2;       
    uint8_t         plustime    : 1;
    uint8_t         plussign    : 1;
    uint8_t         rqsttime    : 1;
    uint8_t         rqstconf    : 1;
    uint8_t         userdefn    : 1; 
} eOropctrl_t;      EO_VERIFYsizeof(eOropctrl_t, 1);


typedef struct
{
    eOropctrl_t     ctrl;
    eOropcode_t     ropc;
    eOnvEP_t        endp;
    eOnvID_t        nvid;
    uint16_t        dsiz;
} eOrophead_t;      EO_VERIFYsizeof(eOrophead_t, 8);

typedef enum
{
    eo_rop_dir_received     = 0,
    eo_rop_dir_outgoing     = 1
} eOropDirection;


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef struct
{
    uint16_t            capacity;
    uint16_t            index;
} EOrop_aboutdata_hid;


typedef struct
{
    EOnvsCfg*           nvscfg;                 // the used nvscfg
    eOnvOwnership_t     nvownership;
    uint8_t             ondevindex;
    uint8_t             onendpointindex;
    uint8_t             onidindex;
    EOtreenode*         nvtreenoderoot;         // points to the relevant nv in the used nvscfg using ondevindx and onendpointindex
    EOnv                nvroot;
//    EOnv                nvleaf;               // dont need it
} EOrop_aboutnvs_hid;

typedef struct
{
    eOipv4addr_t        ipaddr;
//    eOipv4port_t        ipport;               // dont need it
} EOrop_aboutip_hid;


/** @struct     EOrop_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/
struct EOrop_hid 
{
    EOrop_aboutdata_hid aboutdata;
    EOrop_aboutnvs_hid  aboutnvs;
    EOrop_aboutip_hid   aboutip;
    eOrophead_t         head;                    
    uint8_t*            data;       
    uint32_t            sign;
    uint64_t            time;
};    



// - declaration of extern hidden functions ---------------------------------------------------------------------------

extern EOnv* eo_rop_hid_NV_Get(EOrop *p);

extern eOnvOwnership_t eo_rop_hid_GetOwnership(eOropcode_t ropc, eOropconfinfo_t confinfo, eOropDirection direction);

extern eObool_t eo_rop_hid_DataField_is_Present(const eOrophead_t *head);

extern eObool_t eo_rop_hid_DataField_is_Required(const eOrophead_t *head);

//__inline extern
static inline uint16_t eo_rop_hid_DataField_EffectiveSize(uint16_t ropdatasize)
{
    return(((ropdatasize + 3) >> 2) << 2);
}

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




