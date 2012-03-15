
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCOMMON_H_
#define _EOCOMMON_H_


/** @file       EoCommon.h
    @brief      This header file implements public interface to the common types used by the EmbObj
    @author     marco.accame@iit.it
    @date       10/26/2010
**/

/** @defgroup eo_common Common types in embOBJ
    The embOBJ uses some common types and constants for its objects. Those types all follow the eOtypename_t
    naming conventions, whereas the constants use the eo_constantname naming convention.

    Moreover, the objects use one of the following naming conventions:  EOobjName, EOVobjName, EOMobjName, 
    and EOSobjName.  Singleton also use the keyword "the" to express uniqueness (e.g., EOtheObjName).
    The EOobjName name is used when the object objName can be used in any ececution environment, be it
    singletask or multitask.
    The EOVobjName name is used when the object objName has one or more virtual members, and can thus be used 
    only if another object is derived from it.
    The EOMobjName name is used when the object is suited for a multitask execution environment only. Some objects
    of this kind may have some internal tasks used to offer some services to the caller.
    The EOSobjName name is used when the object is suited for the singletask execution environment only.
    
    In general an object objName can be instantiated as many times as there is RAM memory available with the
    _New() method, which gives back an opaque pointer to the object. Other member functions operate on a particular
    object by using the opaque pointer passed as first argument.
    
    However, the object can also be a singleton and as such it can be instantiated only once. The singleton must be 
    explicitely initialised by calling a _Initialise() method which returns an opaque pointer to the correct object.  
    The _Initialise() function is always protected vs multiple calls.
    The same opaque pointer can be retrieved at any time with the _GetHandle() method. 
    Other member functions operate on the singleton by using the opaque pointer passed as first argument.
    
    
    List of objects is the following.
    The embOBJ runs by starting a 
    basic:      EOVmutex, EOtheErrorManager, EOtheMemoryPool, EOV
    Containers: EOdeque, EOlist, EOvector, EOarray
    
    
    
    @todo doxy documentation
    
   @{        
 */
 
  

// - external dependencies --------------------------------------------------------------------------------------------

#include "emBODYporting.h" 

#include "stdint.h"   

//#include "stdbool.h"        // contains true, false, bool. you must use pragma -c99. 


// - public #define  --------------------------------------------------------------------------------------------------


#define EOK_VER_MAJ                         (0x01)
#define EOK_VER_MIN                         (0x00)
#define EOK_VER_REL                         (0x0000)

// issues a compiler error if the sizeof the struct is not what in second argument
#define EO_VERIFYsizeof(sname, ssize)       __emBODYportingVERIFYsizeof(sname, ssize)

#define EO_U8toU32(a)                       ((uint32_t)(a)&0xff)
#define EO_U16toU32(a)                      ((uint32_t)(a)&0xffff)
#define EO_U8toU64(a)                       ((uint64_t)(a)&0xff)
#define EO_4BtoU32(a, b, c, d)              ((EO_U8toU32(a))|(EO_U8toU32(b)<<8)|(EO_U8toU32(c)<<16)|(EO_U8toU32(d)<<24)) 
#define EO_8BtoU64(a, b, c, d, e, f, g, h)  (((uint64_t)EO_4BtoU32(a, b, c, d))|((uint64_t)EO_4BtoU32(e, f, g, h)<<32))


#define EOK_U32VERSION                      ((EO_U8toU32(EOK_VER_MAJ))|((EO_U8toU32(EOK_VER_MIN)<<8))|((EO_U16toU32(EOK_VER_REL))<<16))

#define EOK_uint04dummy                     (0xf)
#define EOK_uint08dummy                     (0xff)
#define EOK_uint16dummy                     (0xffff)
#define EOK_uint32dummy                     (0xffffffffL)
#define EOK_uint64dummy                     (0xffffffffffffffffLL)

#define EOK_reltimeZERO                     (0)
#define EOK_reltime1ms                      (1000)
#define EOK_reltime10ms                     (10*1000)
#define EOK_reltime100ms                    (100*1000)
#define EOK_reltime1sec                     (1000*1000)
#define EOK_reltimeMAX                      (0xFFFFFFFE)
#define EOK_reltimeINFINITE                 (0xFFFFFFFE)

#define EOK_abstimeNOW                      (0xffffffffffffffffLL)


// - declaration of public user-defined types -------------------------------------------------------------------------


/** @typedef    typedef enum eOresult_t
    @brief      eOresult_t contains the results that a function or an operation can have. It is safe to verify the 
                correct result by checking its value being >= 0 (or eores_OK).
    @warning    If you change it, please be sure that the osal, ipal, hal, fsal all have a similar type w/ equal values,
                as sometimes the functions just use a cast of the return values of the abstraction layer. 
 **/
typedef enum  
{
    eores_OK                =  0,       /**< correct result: generic */
    eores_NOK_generic       = -1,       /**< error: generic */
    eores_NOK_nullpointer   = -2,       /**< error: a null pointer was used */
    eores_NOK_timeout       = -3,       /**< error: the caller (task) did not get the osal resource after it waited (even for zero time) */
    eores_NOK_nowait        = -4,       /**< error: the calling isr did not get the osal resource when called the not waiting version of the function */ 
    eores_NOK_nodata        = -5,       /**< error: there was not data to be retrieved */
    eores_NOK_busy          = -6,       /**< error: the service was busy and request has not been served */
    eores_NOK_unsupported   = -15       /**< error: unsupported feature */
} eOresult_t;


/** @typedef    typedef enum eOboolvalues_t
    @brief      eOboolvalue_t contains the two boolean values.
    @warning    C99 contains bool, true, and false. To use C99 include "stdbool.h" and --c99 compiler option.
                At this point please redefine eobool_false and eobool_true to be equal to false and true.
                Also, eObool_t must be typedeffed as bool.
 **/
typedef enum 
{
    eobool_false = 0,                       /**< false */
    eobool_true  = 1                        /**< true */
} eOboolvalues_t;


/** @typedef    typedef uint8_t eObool_t
    @brief      eObool_t contains the boolean type of size 1 byte.
 **/
typedef uint8_t eObool_t;


/** @typedef    typedef struct eOversion_t
    @brief      eOversion_t contains the version of a embOBJ object or library
 **/
typedef struct  
{
    uint8_t     major;                  /**< the major number */
    uint8_t     minor;                  /**< the minor number */
    uint16_t    release;                /**< the release */
} eOversion_t;


/** @typedef    typedef uint32_t eOreltime_t
    @brief      eOreltime_t contains the relative time expressed in micro-seconds. It is used for relative timing
                operations because its maximum value is about 4294 seconds. 
    @warning    It is strongly advised to use the associated eok_reltime* constants (e.g., eok_reltime1sec etc.).
 **/
typedef uint32_t    eOreltime_t; 


/** @typedef    typedef uint64_t eOabstime_t
    @brief      eOabstime_t contains the time expressed in micro-seconds in a much longer range. It is 
                used for absolute timing, usually measured from bootstrap of the device (or from start
                of the singletask or multitask exution environment). A eOabstime_t variable keeps time
                up to 584 thousand years ...
 **/
typedef uint64_t    eOabstime_t; 


/** @typedef    typedef struct eOnanotime_t
    @brief      eOnanotime_t contains the time expressed at its finest resolution: nanoseconds.
                It is used for both relative and absolute time when a finer resultion is needed.
                A eOnanotime_t variable keeps time at the nano-second resolution up to 136 years ...
 **/
typedef uint64_t    eOnanotime_t; 


/** @typedef    typedef uint16_t    eOsizeitem_t
    @brief      eOsizeitem_t contains the size of an object (or item) as used by the container objects
                EOdeque, EOlist, EOvector and its derived objects.
 **/
typedef uint16_t    eOsizeitem_t;


/** @typedef    typedef uint16_t    eOsizecntnr_t
    @brief      eOsizecntnr_t contains the size of internal buffer as used by the container objects
                EOdeque, EOlist, EOvector and its derived objects.
 **/
typedef uint16_t    eOsizecntnr_t;    


/** @typedef    typedef uint32_t    eOevent_t
    @brief      eOevent_t contains an event expressed as a single bit flag. It is used by some objects to
                exchange signalling in a not enqueued manner, for instance by EOaction or by EOVtask and its derivates.
 **/
typedef uint32_t    eOevent_t;


/** @typedef    typedef uint32_t    eOmessage_t
    @brief      eOmessage_t contains a message. It is used by some objects to exchange signalling in an enqueued manner, 
                for instance by EOaction or by EOVtask and its derivates. 
 **/
typedef uint32_t    eOmessage_t;


/** @typedef    typedef void (*eOcallback_t) (void* p)
    @brief      eOcallback_t contains a pointer to a function which can be executed as a result of an action object.
 **/
typedef void (*eOcallback_t) (void* p);


/** @typedef    typedef uint8_t     eOid08_t
    @brief      eOid08_t is a type which contains an ID which can assume up to 256 values. It can be used to retrieve the handle 
                of a ... multiton, which is an object that is instantiated only once and which is uniquely defined by an ID.
 **/
typedef uint8_t     eOid08_t;


/** @typedef    typedef void        eOderived_t
    @brief      eOderived_t is the generic type used for polymorphism.  It is used in the virtual methods of the objects.
 **/
typedef void        eOderived_t;


/** @typedef    typedef void        eOsafelyderived_t
    @brief      It is a derived type with a safety guard. 
 **/ 
typedef void        eOsafelyderived_t;


/** @typedef    typedef uint32_t eOipv4addr_t
    @brief      The address is stored in four consecutive bytes as it is in ipal_ipv4addr_t and as
                specified by macro EO_COMMON_IPV4ADDR().
                As an example, the address ip1.ip2.ip3.ip4 = 10.255.37.204 is stored in the 32-bit word with
                ip1 in least significant byte, ip2 in next etc, thus 0xCC25FF0A. 
                In little endian architectures, the memory layout is the same as in  uint8_t addr[] = {10, 255, 37, 204}. 
 **/  
typedef uint32_t eOipv4addr_t;


/** @typedef    typedef uint32_t eOipv4addr_t
    @brief      The address is stored in sizx consecutive bytes as specified by macro EO_COMMON_IPV4ADDR().
                As an example, the address 03-0E-01-00-01-F0 is stored in the 6 least significant bytes
                03 in least significant byte, 0E in next etc, thus 0x0000F00100010E03. 
                In little endian architectures, the memory layout is the same as in  uint8_t addr[] = {0x03, etc}. 
 **/
typedef uint64_t eOmacaddr_t;


/** @typedef    typedef uint16_t eOipv4port_t
    @brief      The port used for IP v4. 
 **/ 
typedef uint16_t eOipv4port_t;



typedef struct 
{
    eOipv4addr_t    addr;
    eOipv4port_t    port;
} eOipv4addressing_t; 


/** @define     eOvirtual
    @brief      eOvirtual is a qualifier used to identify the virtual methods of an object.
 **/
#define eOvirtual


/** @define     epureOvirtual
    @brief      eOpurevirtual is a qualifier used to identify the pure virtual methods of an abstract object
 **/
#define eOpurevirtual  


typedef     uint16_t    (*eOuint16_fp_uint16_t)                     (uint16_t);
typedef     uint16_t    (*eOuint16_fp_uint32_t)                     (uint32_t);
typedef     uint8_t     (*eOuint8_fp_voidp_t)                       (void *);
typedef     uint8_t     (*eOuint8_fp_uint16_t)                      (uint16_t);
typedef     uint8_t     (*eOuint8_fp_cvoidp_t)                      (const void *);
typedef     void        (*eOvoid_fp_void_t)                         (void);
typedef     void*       (*eOvoidp_fp_void_t)                        (void);
typedef     void        (*eOvoid_fp_voidp_uint32_t)                 (void *, uint32_t);
typedef     void        (*eOvoid_fp_vuint32p_uint32_t)              (volatile uint32_t *, uint32_t);
typedef     void        (*eOvoid_fp_voidp_t)                        (void *);
typedef     void        (*eOvoid_fp_voidp_voidp_t)                  (void *, void *);
typedef     void        (*eOvoid_fp_cvoidp_t)                       (const void *);
typedef     eObool_t    (*eObool_fp_cvoidp_t)                       (const void *);
typedef     eOresult_t  (*eOres_fp_voidp_uint32_uint32_t)           (void *, uint32_t, uint32_t);
typedef     void        (*eOvoid_fp_uint32_uint32_t)                (uint32_t, uint32_t);
typedef     eOresult_t  (*eOres_fp_voidp_evt_t)                     (void *, eOevent_t);
typedef     eOresult_t  (*eOres_fp_voidp_msg_tim_t)                 (void *, eOmessage_t, eOreltime_t);
typedef     eOresult_t  (*eOres_fp_voidp_cbk_voidp_t)               (void *, eOcallback_t, void *);
typedef     eOresult_t  (*eOres_fp_voidp_cbk_voidp_tim_t)           (void *, eOcallback_t, void *p, eOreltime_t);
typedef     eOresult_t  (*eOres_fp_voidp_msg_t)                     (void *, eOmessage_t);
typedef     eOresult_t  (*eOres_fp_voidp_uint32_t)                  (void *, uint32_t);
typedef     eOresult_t  (*eOres_fp_voidp_t)                         (void *);
typedef     eOresult_t  (*eOres_fp_voidp_uint32p_t)                 (void *, uint32_t *);
typedef     eOresult_t  (*eOres_fp_voidp_uint8p_t)                  (void *, uint8_t *);
typedef     eOresult_t  (*eOres_fp_voidp_cvoidpp_t)                 (void *, const void **);
typedef     eOresult_t  (*eOres_fp_voidp_cvoidpp_uint8p_t)          (void *, const void **, uint8_t *);
typedef     eOresult_t  (*eOres_fp_voidp_voidp_t)                   (void *, void *);
typedef     void        (*eOvoid_fp_uint64_t)                       (uint64_t);
typedef     void        (*eOvoid_fp_uint8_evt_t)                    (uint8_t, eOevent_t);
typedef     void        (*eOvoid_fp_msg_t)                          (eOmessage_t);
typedef     void        (*eOvoid_fp_uint16_t)                       (uint16_t);
typedef     void        (*eOvoid_fp_uint32_t)                       (uint32_t);
typedef     uint64_t    (*eOuint64_fp_void_t)                       (void);
typedef     uint8_t     (*eOuint8_fp_void_t)                        (void);
typedef     eOresult_t  (*eOres_fp_voidfpvoid_t)                    (void (*)(void));
typedef     eOresult_t  (*eOres_fp_voidp_uint32_uint32_cvoidp_t)    (void *, uint32_t, uint32_t, const void *);
typedef     eOresult_t  (*eOres_fp_voidp_uint32_uint32_voidp_t)     (void *, uint32_t, uint32_t, void *);
typedef     void        (*eOvoid_fp_voidfpvoiduint32_t)             (void (*)(void), uint32_t);
typedef     void        (*eOvoid_fp_voidfpvoiduint32uint8_t)       (void (*)(void), uint32_t, uint8_t);




/** @typedef    typedef struct eObasicabstraction_hal_sys_fn_t
    @brief      eObasicabstr_hal_sys_fn_t contains the basic system functions typically given by a HAL which can be used for instance
                by the objects of the single-task execution environment (or SEE) to run. They have similar name and APIs as the
                ones in the HAL.
 **/  
typedef struct
{
    eOvoid_fp_void_t                    hal_base_init;                  /**< initialise the hal **/
    eOvoid_fp_void_t                    hal_sys_systeminit;             /**< initialise the system */
    eOvoid_fp_voidfpvoiduint32uint8_t   hal_sys_systick_sethandler;     /**< starts a tick function w/ a given period at a given priority**/
    eOvoid_fp_vuint32p_uint32_t         hal_sys_atomic_bitwiseAND;      /**< performs atomic bitwise AND on a uint32_t passed as volatile pointer **/
    eOvoid_fp_vuint32p_uint32_t         hal_sys_atomic_bitwiseOR;       /**< performs atomic bitwise OR on a uint32_t passed as volatile pointer **/
    eOres_fp_voidp_uint32_t             hal_sys_criticalsection_take;   /**< takes a critical section on a dummy object (also for ISRs) with a dummy timeout**/
    eOres_fp_voidp_t                    hal_sys_criticalsection_release;/**< releases the critical section on the dummy object **/
    eOvoid_fp_void_t                    hal_sys_irq_disable;            /**< disable all irqs **/
    eOvoid_fp_void_t                    hal_sys_irq_enable;             /**< enable all irqs **/
} eObasicabstr_hal_sys_fn_t;


/** @typedef    typedef struct eObasicabstr_fsal_fn_t
    @brief      eObasicabstr_fsal_fn_t contains the basic functions sued to initialise a FSAL.
 **/  
typedef struct
{
    eOvoid_fp_void_t                fsal_init;                       /**< initialise the fsal **/
} eObasicabstr_fsal_fn_t;



// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOversion_t    eok_version;            /**< = major, minor, release */
extern const uint32_t       eok_u32version;         /**< = LSB->MSB: major (1B), minor (1B), release (2B), w/ same layout as eok_version */

extern const uint8_t        eok_uint04dummy;        /**< = EOK_uint04dummy = 0xf;                   // dummy for a nibble */
extern const uint8_t        eok_uint08dummy;        /**< = EOK_uint08dummy = 0xff;                  // dummy for a uint8_t */
extern const uint16_t       eok_uint16dummy;        /**< = EOK_uint16dummy = 0xffff;                // dummy for a uint16_t */
extern const uint32_t       eok_uint32dummy;        /**< = EOK_uint32dummy = 0xffffffffL;           // dummy for a uint32_t */
extern const uint64_t       eok_uint64dummy;        /**< = EOK_uint64dummy = 0xffffffffffffffffLL;  // dummy for a uint64_t */

extern const eOreltime_t    eok_reltimeZERO;        /**< = 0;                                       // Zero time */
extern const eOreltime_t    eok_reltime1ms;         /**< = 1000;                                    // 1 milli-sec */
extern const eOreltime_t    eok_reltime10ms;        /**< = 10*1000;                                 // 10 milli-sec */
extern const eOreltime_t    eok_reltime100ms;       /**< = 100*1000;                                // 100 milli-sec */
extern const eOreltime_t    eok_reltime1sec;        /**< = 1000*1000;                               // 1 sec */
extern const eOreltime_t    eok_reltimeMAX;         /**< = 0xFFFFFFFE;                              // equals 4294 seconds */
extern const eOreltime_t    eok_reltimeINFINITE;    /**< = 0xFFFFFFFE;                              // means that the wait must be infinite */

extern const eOabstime_t    eok_abstimeNOW;         /**< = 0xffffffffffffffffLL; */


extern const eOipv4addr_t   eok_ipv4addr_localhost;



// - declaration of extern public functions ---------------------------------------------------------------------------

extern void * eo_common_getbaseobject(eOderived_t *derived);

extern void * eo_common_getsafebaseobject(eOsafelyderived_t *derived);

extern eOresult_t eo_common_verifysafebaseobject(eOsafelyderived_t *derived, const uint32_t tag);


// - definition of extern public macros ------------------------------------------------------------------------------

#define EO_COMMON_IPV4ADDR(ip1, ip2, ip3, ip4)      ((eOipv4addr_t)EO_4BtoU32(ip1, ip2, ip3, ip4))
#define EO_COMMON_MACADDR(m1, m2, m3, m4, m5, m6)   ((eOmacaddr_t)EO_8BtoU64(m1, m2, m3, m4, m5, m6, 0, 0))

#define EO_COMMON_IPV4ADDR_LOCALHOST                EO_COMMON_IPV4ADDR(127, 0, 0, 1)

// - definition of extern public inlined functions --------------------------------------------------------------------

EO_extern_inline eOipv4addr_t eo_common_ipv4addr(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4)
{
    return(EO_COMMON_IPV4ADDR(ip1, ip2, ip3, ip4));
}

EO_extern_inline eOmacaddr_t eo_common_macaddr(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t m5, uint8_t m6)
{
    return(EO_COMMON_MACADDR(m1, m2, m3, m4, m5, m6));
}





/** @}            
    end of group eo_common  
 **/
 
 
#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



