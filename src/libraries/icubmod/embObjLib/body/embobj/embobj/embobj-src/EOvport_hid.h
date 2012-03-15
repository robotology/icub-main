
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVPORT_HID_H_
#define _EOVPORT_HID_H_


/* @file       EOvport_hid.h
    @brief      This header file implements hidden interface to a rop object.
    @author     marco.accame@iit.it
    @date       09/03/2010
 **/

#error --> dont use it yet

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheNVs.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOvport.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

//#define EOVPORT_MAX_NVS			            (8)			// or how many you want
#define EOVPORTMAXSIZEVAR                   (8)
//#define EOVPORT_MAXSIZE_NVIDSIZEINFO		(96)


#define EOVPORT_GET_SizeOfArray(n)          (2+2*(n))

#define EOVPORT_GET_Mirror(vp, n)          ((void**)&((vp).mirrors[(n)]))

			
// - definition of the hidden struct implementing the object ----------------------------------------------------------


//__packed struct EOvportCfg_hid 
//{
//    uint16_t        sizeofarray;                  // number of bytes: equal to 2 (for n) + 2*n
//	uint16_t		n;			                // actual number inside nvids[]
//	uint16_t		nvids[EOVPORT_MAX_NVS];		    // container for nvids
//};
//
//
//__packed struct EOvportDat_hid 
//{
//    uint16_t        sizeofarray;                  // number of bytes: equal to 2 (for n) + 2*n
//    uint16_t		n;			                // actual number of {nvid, size, info}
//	uint8_t		    buffer_of_nvidsizeinfo[EOVPORT_MAXSIZE_NVIDSIZEINFO];	// container for {nvid, size, info}
//};


typedef __packed struct
{
	uint16_t		nvid;	        // the nvid
	uint16_t		size;		    // the size
	uint8_t		    info[1];		// the info
} eOvport_dat_nvidsizeinfo_t;


///** @struct     EOvport_hid
//    @brief      Hidden definition. Implements private data used only internally by the 
//                public or private (static) functions of the object and protected data
//                used also by its derived objects.
// **/
//__packed struct EOvport_hid 
//{
//    EOvportCfg      cfg;
//    EOvportDat      dat;
//    void*           mirrors[EOVPORT_MAX_NVS];
//};   
 



// - declaration of extern hidden functions ---------------------------------------------------------------------------

/** @fn         extern EOvport * eo_vport_hid_New(void)
    @brief      Creates a new vport object. 
    @return     The pointer to the required object.
 **/
extern EOvport * eo_vport_hid_New(void);


extern void eo_vport_hid_LoadCfg(EOvport *vp, EOvportCfg *cfg, eOnetvarOwnership_t ownership, uint16_t port, eOipv4addr_t ipaddr);

extern void eo_vport_hid_MoveDat2NVs(EOvport *vp, eOnetvarOwnership_t ownership, uint16_t port, eOipv4addr_t ipaddr);


// this function prepares the content of vport.dat on the basis of what is in vport.cfg.
// so far it is meaningful only for a local vport
//extern void eo_vport_hid_SynchroniseCfg2Dat(EOvport *vp);


// this function propagates the content of vport.dat into the NVs contained in it.
// so far it is used only for a remote vport, so that a receiver can update the netvars 
// on the vport and call the related callbacks.
//extern void eo_vport_hid_SynchroniseDat2NVs(EOvport *vp, eOnetvarOwnership_t ownership, eOipv4addr_t ipaddr, eOropcode_t ropc); 

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




