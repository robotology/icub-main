
/* @file       EoCommon.c
	@brief      This file implements internal implementation of the embobj common module
	@author     marco.accame@iit.it
    @date       07/12/2010
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EoCommon.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOversion_t eok_version               = {
                                                EO_INIT(.major)     EOK_VER_MAJ, 
                                                EO_INIT(.minor)     EOK_VER_MIN, 
                                                EO_INIT(.release)   EOK_VER_REL
                                              };            

const uint32_t  eok_u32version              = EOK_U32VERSION;

const uint8_t   eok_uint04dummy             = EOK_uint04dummy;
const uint8_t   eok_uint08dummy             = EOK_uint08dummy;
const uint16_t  eok_uint16dummy             = EOK_uint16dummy;
const uint32_t  eok_uint32dummy             = EOK_uint32dummy;
const uint64_t  eok_uint64dummy             = EOK_uint64dummy;

const eOreltime_t eok_reltimeZERO           = 0;
const eOreltime_t eok_reltime1ms            = 1000;
const eOreltime_t eok_reltime10ms           = 10000;
const eOreltime_t eok_reltime100ms          = 100000;
const eOreltime_t eok_reltime1sec           = 1000000;
const eOreltime_t eok_reltimeMAX            = 0xfffffffe;
const eOreltime_t eok_reltimeINFINITE       = 0xffffffff;

const eOabstime_t eok_abstimeNOW            = 0xffffffffffffffffLL;

const eOipv4addr_t eok_ipv4addr_localhost   = EO_COMMON_IPV4ADDR_LOCALHOST;

//const uint8_t  eo_dummy08            = 255;



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

// used to retrieve the base object when it is the first pointer in the struct
// the idea is to use: typedef struct { base_t *name; } derived_t;
extern void * eo_common_getbaseobject(eOderived_t *derived) 
{
	void **pp_base = (void **)derived;
	
	if(NULL == pp_base) // pp_base is equal to derived. 
	{
		return(NULL);	
	}
	
	return(*pp_base);
}

// used to retrieve teh base object from a safely derived object.
// the idea is to use: typedef struct { uint32_t tag; base_t *name; } safelyderived_t;
extern void * eo_common_getsafebaseobject(eOsafelyderived_t *derived)
{
    uint32_t* peoidentifier = (uint32_t*)derived;
    void **pp_base = NULL;

    pp_base = (void **) (peoidentifier+1);
	
	return(*pp_base);
}

// used to verify that the derived object is effectively derived from the base with the passed tag.
// the idea is to use: typedef struct { uint32_t tag; base_t *name; } safelyderived_t;
extern eOresult_t eo_common_verifysafebaseobject(eOsafelyderived_t *derived, const uint32_t tag)
{
    const uint32_t* peoidentifier = (uint32_t*)derived;
    return((tag == *peoidentifier) ? (eores_OK) : (eores_NOK_generic));
}


//// used to manipulate an array
//extern eOresult_t eo_common_array_reset(eOarray_t *array)
//{
//    if(NULL == array)
//    {
//        return(eores_NOK_nullpointer);
//    }
//
//    memset(array->data, 0, array->capacity);
//    array->size = 0;
//
//    return(eores_OK);
//}
//
//// used to manipulate an array
//extern eOresult_t eo_common_array_puskback(eOarray_t *array, const void *data, const uint16_t size)
//{
//    if((NULL == array) || (NULL == data))
//    {
//        return(eores_NOK_nullpointer);
//    }
//
//    if(array->size + size > array->capacity)
//    {
//         return(eores_NOK_generic);
//    }
//
//    memcpy(&array->data[array->size], data, size);
//    array->size += size;
//
//    return(eores_OK);
//}
//
//extern void* eo_common_array_get_sizedata(eOarray_t *array, uint16_t *size)
//{
//    if(NULL == array)
//    {
//        return(NULL);
//    }
//
//    *size = array->size;
//
//    return(&(array->size));
//}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

