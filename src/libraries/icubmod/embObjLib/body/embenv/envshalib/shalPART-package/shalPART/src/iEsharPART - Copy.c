
/* @file       iSharPART.c
    @brief      This header file implements the iEsharPART library.
    @author     marco.accame@iit.it
    @date       05/03/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------



#include "stm32f10x.h"
#include "stm32f10x_flash.h"

#include "string.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "iEsharPART.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------

typedef struct
{
    uint64_t                signature;
    iEsharPARTversion_t     version;
    uint32_t                defflag;
} romHeader_t;


#define STRSIZE             IESHARPART_STRINGSIZE
#define NPROCMAX            10
#define NSHARMAX            10


typedef struct
{
    iEsharPARTversion_t     procversion;
    uint32_t                procaddress;
    uint32_t                procsizemax;
    uint8_t                 procname[STRSIZE];
} iEsharPARTprocess_t;

typedef struct
{
    iEsharPARTversion_t     slibversion;
    uint32_t                slibaddress;
    uint32_t                slibsizemax;
    iEsharPARTversion_t     sdatversion;
    uint32_t                sdataddress;
    uint32_t                sdatsizemax;
    uint8_t                 sharname[STRSIZE];
} iEsharPARTsharlib_t;

typedef struct
{
    uint8_t                 NprocMax;               // constant and fixed to 10 ...  as it is size of array inside
    uint8_t                 NsharMax;               // constant and fixed to 10 ...  as it is size of array inside 
    uint8_t                 Nproc;                  // can be changed in runtime if a new partition w/ a pro is added
    iEsharPARTproc_t        procs[NPROCMAX];        // contains list of Nproc procs
    uint8_t                 Nshar;                  // can be changed in runtime if a new partition w/ a sha is added
    iEsharPARTshar_t        shars[NSHARMAX];        // contains list of Nshar shars
    iEsharPARTproc_t        defProc2run;            // it is the default iProcess the iLoader runs (if no other info)
    iEsharPARTprocess_t     procTable[NPROCMAX];    // the process table
    iEsharPARTsharlib_t     sharTable[NSHARMAX];    // the sharlib table
} romData_t;

typedef struct 
{
    romHeader_t             head;
    romData_t               data;
} romPart_t;


typedef int dummy[sizeof(romPart_t) <= (1024-32) ? 1 : -1];



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define DEFFLAG_TRUE                          0x0001
#define DEFFLAG_FALSE                         0x0000

#define INVALID_ADDR                          0x00000000
#define EMPTY_STR                             0

// - signature  -------------------------------------------------------------------------------------------------------
#define ISHARPARTSIGNATURE                  0xACE0ACE01E000003

// - version of iEsharPART --------------------------------------------------------------------------------------------
#define DEF_VERS                            {IESHARPART_MAJOR, IESHARPART_MINOR, IESHARPART_BUILD}

// - defflag  ---------------------------------------------------------------------------------------------------------
#define DEF_DEFFLAG                          DEFFLAG_TRUE

// - def-proc2run -----------------------------------------------------------------------------------------------------
#define DEFPROC2RUN                         iEsharPARTprocApplication

// - null-proc --------------------------------------------------------------------------------------------------------
#define NULL_PROC                           {{0, 0, 0}, INVALID_ADDR, 0, {EMPTY_STR}}

// - null-shar --------------------------------------------------------------------------------------------------------
#define NULL_SHAR                           {{0, 0, 0}, INVALID_ADDR, 0, {0, 0, 0}, INVALID_ADDR, 0, {EMPTY_STR}}


// - loader-proc ------------------------------------------------------------------------------------------------------
#define LOADER_PROC                         {{0, 0, 0}, 0x08000000, 0x01000, {"iLoader-def"}}

// - appl-proc --------------------------------------------------------------------------------------------------------
#define APPLIC_PROC                         {{0, 0, 0}, 0x08010000, 0x30000, {"iProcess-def"}}


// - part-shar --------------------------------------------------------------------------------------------------------
#define PARTIT_SHAR                         {DEF_VERS,  0x08002000, 0x00800, DEF_VERS,  0x08002800, 0x00800, {"iSharPART"}}


// - ipc-shar ---------------------------------------------------------------------------------------------------------
#define IPCOMM_SHAR                         {{0, 0, 0}, 0x08001000, 0x00800, {0, 0, 0}, 0,          0,       {"iSharIPC-def"}}



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_program_page(void);
static iEsharPARTresult_t s_itisinside_proctable(iEsharPARTproc_t proc);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


// - default values ---------------------------------------------------------------------------------------------------



static const iEsharPARTversion_t s_sharpart_version __attribute__((at(0x08002000))) = DEF_VERS;


static const romPart_t s_sharpart_ro __attribute__((at(0x08002800))) = 
{
    {   // head 
        ISHARPARTSIGNATURE,
        DEF_VERS,
        DEF_DEFFLAG
    },
    {   // data
        NPROCMAX,                           // NprocMax   
        NSHARMAX,                           // NsharMax
        2,                                  // Nproc
        {                                   // list of ordered procs
            iEsharPARTprocLoader, 
            iEsharPARTprocApplication, 
            iEsharPARTprocNone, iEsharPARTprocNone, iEsharPARTprocNone, iEsharPARTprocNone, // filler 
            iEsharPARTprocNone, iEsharPARTprocNone, iEsharPARTprocNone, iEsharPARTprocNone  // filler
        },  
        2,                                  // Nshar
        {                                   // list of ordered shars
            iEsharPARTsharPART, 
            iEsharPARTsharIPC,           
            iEsharPARTsharNone, iEsharPARTsharNone, iEsharPARTsharNone, iEsharPARTsharNone, // filler 
            iEsharPARTsharNone, iEsharPARTsharNone, iEsharPARTsharNone, iEsharPARTsharNone  // filler
        }, 
        DEFPROC2RUN,                        // defProc2run
        {                                   // procTable[NPROCMAX]
            LOADER_PROC,                        // 0 loader
            NULL_PROC,                          // 1 updater
            APPLIC_PROC,                        // 2 application
            NULL_PROC,                          // 3 userapp
            NULL_PROC,                          // 4 userapp
            NULL_PROC,                          // 5 userapp
            NULL_PROC,                          // 6 userapp
            NULL_PROC,                          // 7 userapp
            NULL_PROC,                          // 8 userapp
            NULL_PROC                           // 9 userapp
        },
        {                                   // sharTable[NSHARMAX]
            PARTIT_SHAR,                        // 0 part
            IPCOMM_SHAR,                        // 1 ipc
            NULL_SHAR,                          // 2 info
            NULL_SHAR,                          // 3 fs
            NULL_SHAR,                          // 4 userpar
            NULL_SHAR,                          // 5 userpar
            NULL_SHAR,                          // 6 userpar
            NULL_SHAR,                          // 7 userpar
            NULL_SHAR,                          // 8 userpar
            NULL_SHAR,                          // 9 userpar
        }
    }
}; 


static volatile romPart_t s_sharpart_rw __attribute__((at(0x20000020))); 
                                                                             
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern iEsharPARTresult_t sharpart_init(void)
{

    if(IESHARPART_MAJOR != s_sharpart_ro.head.version.major)
    {
        // the rom data has surely been updated by the updater and is not synchronised with
        // the manipulating code .... what to do? decide it.
    }


    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_deinit(void)
{
    // so far we dont need to undo anything. whenever we support eeprom (on i2c) we need to ....
    return(iEsharPARTresOK);
}


extern iEsharPARTresult_t sharpart_proc2run_get(iEsharPARTproc_t *proc)
{
    *proc = (iEsharPARTproc_t)s_sharpart_ro.data.defProc2run;
    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_proc2run_set(iEsharPARTproc_t proc)
{
    if(s_sharpart_ro.data.defProc2run != proc)
    {
        memcpy((void*)&s_sharpart_rw, (void*)&s_sharpart_ro, sizeof(romPart_t));
        s_sharpart_rw.data.defProc2run = proc;
        s_sharpart_rw.head.defflag = DEFFLAG_FALSE;
        s_program_page();
    }

    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_proc_table_get(const iEsharPARTproc_t **table, uint8_t *size)
{
    *table = s_sharpart_ro.data.procs;
    *size = s_sharpart_ro.data.Nproc;
    return(iEsharPARTresOK);
}


extern iEsharPARTresult_t sharpart_proc_add(iEsharPARTproc_t proc, iEsharPARTprocessInfo_t *procinfo) 
{

    const uint8_t *name;
    iEsharPARTversion_t *vers;
    uint32_t addr;
    uint32_t msize;

    uint8_t i, n;

    

    if((proc >= NPROCMAX) || (NULL == procinfo))
    {
        return(iEsharPARTresNOK_generic);     
    }

    name  = procinfo->name;
    vers  = &procinfo->version;
    addr  = procinfo->startaddress;
    msize = procinfo->maxsize;

    // 1. copy rom into ram and then copy ....
    memcpy((void*)&s_sharpart_rw, (void*)&s_sharpart_ro, sizeof(romPart_t));
    // name
    strncpy((char *)s_sharpart_rw.data.procTable[proc].procname, (const char *)name, STRSIZE-1);
    s_sharpart_rw.data.procTable[proc].procname[STRSIZE-1] = '\0';
    // version
    memcpy((void*)&(s_sharpart_rw.data.procTable[proc].procversion), vers, sizeof(iEsharPARTversion_t));        
    // start addr
    s_sharpart_rw.data.procTable[proc].procaddress = addr;
    // max size
    s_sharpart_rw.data.procTable[proc].procsizemax = msize;
    
    // 2. update the table
    n = 0;
    for(i=0; i<NPROCMAX; i++)
    {
        s_sharpart_rw.data.procs[i] = iEsharPARTprocNone;
    }
    for(i=0; i<NPROCMAX; i++)
    {
        if(INVALID_ADDR != s_sharpart_rw.data.procTable[i].procaddress)
        {
            // found one
            s_sharpart_rw.data.procs[n] = (iEsharPARTproc_t)i;
            n++; 
        }
    }
    // 3. update the number in teh table
    s_sharpart_rw.data.Nproc = n;
    

    // 4. touch the default flag
    s_sharpart_rw.head.defflag = DEFFLAG_FALSE;

    // 5. copy into flash
    s_program_page();


    return(iEsharPARTresOK);
}


extern iEsharPARTresult_t sharpart_proc_rem(iEsharPARTproc_t proc)
{

    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }

}



extern iEsharPARTresult_t sharpart_proc_name_get(iEsharPARTproc_t proc, const uint8_t **name)
{

    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }

    *name = s_sharpart_ro.data.procTable[proc].procname;

    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_proc_name_set(iEsharPARTproc_t proc, const uint8_t *name)
{

    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }

    if(0 != strcmp((const char*)s_sharpart_ro.data.procTable[proc].procname, (const char*)name))
    {
        memcpy((void*)&s_sharpart_rw, (void*)&s_sharpart_ro, sizeof(romPart_t));
        strncpy((char *)s_sharpart_rw.data.procTable[proc].procname, (const char *)name, STRSIZE-1);
        s_sharpart_rw.data.procTable[proc].procname[STRSIZE-1] = '\0';
        s_sharpart_rw.head.defflag = DEFFLAG_FALSE;
        s_program_page();
    }

    return(iEsharPARTresOK);
    
}


extern iEsharPARTresult_t sharpart_proc_addr_get(iEsharPARTproc_t proc, uint32_t *addr)
{
    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }
    
    *addr = s_sharpart_ro.data.procTable[proc].procaddress;

    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_proc_addr_set(iEsharPARTproc_t proc, uint32_t addr)
{
    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }

    if(s_sharpart_ro.data.procTable[proc].procaddress != addr)
    {
        memcpy((void*)&s_sharpart_rw, (void*)&s_sharpart_ro, sizeof(romPart_t));
        s_sharpart_rw.data.procTable[proc].procaddress = addr;
        s_sharpart_rw.head.defflag = DEFFLAG_FALSE;
        s_program_page();
    }

    return(iEsharPARTresOK);
}


extern iEsharPARTresult_t sharpart_proc_maxsize_get(iEsharPARTproc_t proc, uint32_t *msize)
{
    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }

    *msize = s_sharpart_ro.data.procTable[proc].procsizemax;

    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_proc_maxsize_set(iEsharPARTproc_t proc, uint32_t msize)
{
    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }
    
    if(s_sharpart_ro.data.procTable[proc].procsizemax != msize)
    {
            memcpy((void*)&s_sharpart_rw, (void*)&s_sharpart_ro, sizeof(romPart_t));
            s_sharpart_rw.data.procTable[proc].procsizemax = msize;
            s_sharpart_rw.head.defflag = DEFFLAG_FALSE;
            s_program_page();
    }

    return(iEsharPARTresOK);
}


extern iEsharPARTresult_t sharpart_proc_vers_get(iEsharPARTproc_t proc, const iEsharPARTversion_t **vers)
{
    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }
    
    *vers = &(s_sharpart_ro.data.procTable[proc].procversion);

    return(iEsharPARTresOK);
}

extern iEsharPARTresult_t sharpart_proc_vers_set(iEsharPARTproc_t proc, iEsharPARTversion_t *vers)
{
    if(iEsharPARTresNOK_generic == s_itisinside_proctable(proc))
    {
        return(iEsharPARTresNOK_generic);
    }
    
    if(0 != memcmp(&(s_sharpart_ro.data.procTable[proc].procversion), vers, sizeof(iEsharPARTversion_t)))
    {
            memcpy((void*)&s_sharpart_rw, (void*)&s_sharpart_ro, sizeof(romPart_t));
            memcpy((void*)&(s_sharpart_rw.data.procTable[proc].procversion), vers, sizeof(iEsharPARTversion_t));
            s_sharpart_rw.head.defflag = DEFFLAG_FALSE;
            s_program_page();
    }

    return(iEsharPARTresOK);
}















// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void s_program_page(void)
{
    uint32_t addr = (uint32_t)&s_sharpart_ro; 
    uint32_t EndAddr = addr + sizeof(romPart_t);
    uint32_t *data = (uint32_t *) (&s_sharpart_rw);
    volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

    // unlock
    FLASH_Unlock();

    // erase
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(addr);

    while((addr < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
    {
        FLASHStatus = FLASH_ProgramWord(addr, *data);
        addr = addr + 4;
        data++;
    }

}


static iEsharPARTresult_t s_itisinside_proctable(iEsharPARTproc_t proc)
{
    uint8_t i;

    if(proc >= NPROCMAX)
    {
        return(iEsharPARTresNOK_generic);
    }

    for(i=0; i<s_sharpart_ro.data.Nproc; i++)
    {
        if(s_sharpart_ro.data.procs[i] == proc)
        {
            return(iEsharPARTresOK);
        }
    }

    return(iEsharPARTresNOK_generic);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

