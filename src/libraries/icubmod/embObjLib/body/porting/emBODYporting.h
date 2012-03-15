
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EMBODYPORTING_H_
#define _EMBODYPORTING_H_

// - doxy begin -------------------------------------------------------------------------------------------------------

/** @file       emBODYporting.h
    @brief      This header file cevcervcwer.
    @author     marco.accame@iit.it
    @date       11/03/2011
**/

/** @defgroup cecedcedc complier specifics for emBODY environment
    The embENV allows ...... 
 
    @todo acemor-facenda: do documentation.
    

    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------


// - public #define  --------------------------------------------------------------------------------------------------

#if defined(_MSC_VER)
    // msc does not support c99, thus inline must be redefined as __inline
    #define EO_extern_inline       extern __inline
    #define EO_static_inline       static __inline
    //#define inline          __inline
    // msc does not support c99, thus designated initializers in structs (i.e., .item = val) must be moved to the old way
    #define EO_INIT(f) 
    #pragma pack(8) 
    // or pack(4) ???
    #define snprintf        sprintf_s
//#pragma message(a)
#elif defined(__GNUC__)
    // gcc-linux
    //#define inline          inline
    #define EO_extern_inline       static inline
    #define EO_static_inline       static inline
    #define EO_INIT(f)
    #pragma pack(8)
    #define snprintf        snprintf    
#else
    // armcc
    #define EO_extern_inline        extern inline
    #define EO_static_inline        static inline
    //#define inline accidenti
    // other compilers which support c99 can keep the designated initializers in structs
    #define EO_INIT(f)      f =
    #pragma pack(8)
    // or pack(4) ???
    #define snprintf        snprintf   
#endif



#define __emBODYportingVERIFYsizeof(sname, ssize)    typedef uint8_t GUARD##sname[ ( ssize == sizeof(sname) ) ? (1) : (0)];

// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section 

// - declaration of extern public functions ---------------------------------------------------------------------------

 

/** @}            
    end of group cecedcedc 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



