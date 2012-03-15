
/* @file       app-telnet.c
	@brief      This file implements a test for abslayer with a telnet server, an ftp server (hal + ipal + fsal), a 
                blinking led facility (hal) done w/ a sw timer (osal). wow.
	@author     marco.accame@iit.it
    @date       06/16/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include <string.h>

#include "hal.h"
#include "osal.h"
#include "ipal.h"
#include "fsal.h"

#include "shalPART.h"
#include "shalBASE.h"
#include "shalINFO.h"

#include "hexParser.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "app-telnet.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


#define ADR_DEF_APP_LOW         (EENV_MEMMAP_EAPPLICATION_ROMADDR)
#define ADR_DEF_APP_HIGH        (EENV_MEMMAP_EAPPLICATION_ROMADDR+EENV_MEMMAP_EAPPLICATION_ROMSIZE)


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------



extern const char apptelnet_msg_login[] = 
{
    "                                           \r\n"
    "       eUpdater                            \r\n"
    "                                           \r\n"
};

extern const char apptelnet_msg_welcome[] = 
{
    "       Welcome to eUpdater                 \r\n"
    "       Type help for supported commands    \r\n"
    "                                           \r\n"
};

extern const char apptelnet_prompt[] = "\r\nupdt> ";

static const shalinfo_deviceinfo_t* s_deviceinfo = NULL;

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_restart(void *tmr, void *arg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static uint32_t s_acemor_sig[] = { 0, 0, 7 };

static char const s_help_table[] = {
  "\r\n\r\n"
  "  available commands:\r\n"
  "\r\n"
  "  help           - prints this help  \r\n"
  "  quit           - leave the telnet session  \r\n"
  "  defrag         - defrag the embedded file system.  \r\n"
  "  format         - format the embedded file system.  \r\n"
  "  shals          - prints info on shared libraries   \r\n"
  "  procs          - prints info on processes  \r\n"
  "  syncprocs      - searches for e-procs in flash and syncronise the part table  \r\n"
  "  syncshals      - searches for e-shals in flash and syncronise the part table  \r\n"
  "  defproc n      - makes proc n on the list the default one  \r\n"
  "  verify f.ext   - verify if f.ext contains a valid e-proc   \r\n"
  "                   fname at max 12 char and ext = [bin, hex] \r\n"
  "  remove         - removes eApplication from FLASH.  \r\n"
  "                   default process will become the eUpdater  \r\n"
  "  load f.ext     - loads the fname.ext in flash and makes it the default \r\n"
  "                   fname at max 12 char and ext = [bin, hex] \r\n"
  "  ipset a.b.c.d  - w/out argument it tells the ip addr, else it sets addr    \r\n"
  "                   the new address is effective at next bootstrap.   \r\n"
  "  mskset a.b.c.d - w/out argument it tells the ip mask, else it sets msk \r\n"
  "                   the new mask is effective at next bootstrap.  \r\n"
  "  restart        - restart the system and boostrap the default process   \r\n"
  "\r\n\r\n"
};          

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern uint16_t apptelnet_onexecupdater(const char *cmd, char *rep, uint8_t *quitflag)
{
    uint16_t cmdlen =0;
    uint16_t retlen = 0;

    uint8_t s_size08 = 0;
    uint16_t s_size16 = 0;
    uint8_t *s_data = NULL;
    uint32_t s_addrdata = 0;
    uint8_t i = 0;
    char *strtnet = NULL;
    uint32_t s_lower = 0;
    uint32_t s_upper = 0;
    uint8_t s_eof_flag = 0;


    cmdlen = strlen(cmd);
    *quitflag = 0;

    // ## help -
    if(0 == strncmp(cmd, "help", strlen("help")))
    {
        retlen += sprintf ((char *)rep + retlen,"\r\n %s", s_help_table);    
    }


    // ## quit -
    else if(0 == strncmp(cmd, "quit", strlen("quit")))
    {
        retlen += sprintf ((char *)rep + retlen,"\r\n quitting telnet... bye\r\n"); 
        *quitflag = 1;
    }


    // ## defrag -
    else if(0 == strncmp(cmd, "defrag", strlen("defrag")))
    {
        hal_sys_irq_disable();
        fsal_defrag(fsal_drive_default);
        hal_sys_irq_enable();
        retlen += sprintf ((char *)rep + retlen,"\r\n defrag finished\r\n"); 
    }


    // ## format -
    else if(0 == strncmp(cmd, "format", strlen("format")))
    {
        FILE *fp = NULL;
        fsal_result_t res = fsal_res_NOK_generic;
        
        
        hal_sys_irq_disable();
        res = fsal_format(fsal_drive_default);
        fp = fopen("acemor.was.here", "w");
        if(NULL != fp)
        {
            fwrite(s_acemor_sig, 1, sizeof(s_acemor_sig), fp);
            fclose(fp);
        }
        hal_sys_irq_enable();
        retlen += sprintf ((char *)rep + retlen,"\r\n format finished and added acemor.was.here %d\r\n", res); 
    }


    // ## procs -
    else if(0 == strncmp(cmd, "procs", strlen("procs")))
    {

        const eEprocess_t *s_proctable = NULL;
        const eEmoduleInfo_t *s_modinfo = NULL;
        eEprocess_t defproc;

        shalpart_proc_def2run_get(&defproc);

        if(ee_res_OK == shalpart_proc_allavailable_get(&s_proctable, &s_size08))
        {
            retlen +=  sprintf ((char *)rep + retlen,"\r\n there are %d e-procs :\r\n", s_size08);
                
            for(i=0; i<s_size08; i++)
            {
                shalpart_proc_get(s_proctable[i], &s_modinfo);
  
                retlen += sprintf ((char *)rep + retlen," e-proc %d: %s\r\n", i, (i == (uint8_t)defproc)?(" default"):("") );
                retlen += sprintf ((char *)rep + retlen,"   name     = %s\r\n", s_modinfo->info.name);
                retlen += sprintf ((char *)rep + retlen,"   version  = %d.%d. w/ build done on %d.%d.%d at hour %d.%d\r\n", 
                                                                                    s_modinfo->info.entity.version.major, 
                                                                                    s_modinfo->info.entity.version.minor,
                                                                                    s_modinfo->info.entity.builddate.year,
                                                                                    s_modinfo->info.entity.builddate.month,
                                                                                    s_modinfo->info.entity.builddate.day,
                                                                                    s_modinfo->info.entity.builddate.hour,
                                                                                    s_modinfo->info.entity.builddate.min
                                                                                    );
                retlen += sprintf ((char *)rep + retlen,"   rom.addr  = 0x%0.8X\r\n", s_modinfo->info.rom.addr);
                retlen += sprintf ((char *)rep + retlen,"   rom.size  = 0x%0.8X\r\n", s_modinfo->info.rom.size);
                retlen += sprintf ((char *)rep + retlen,"   ram.addr  = 0x%0.8X\r\n", s_modinfo->info.ram.addr);
                retlen += sprintf ((char *)rep + retlen,"   ram.size  = 0x%0.8X\r\n", s_modinfo->info.ram.size);

                retlen += sprintf ((char *)rep + retlen,"   stg.type  = %s\r\n", (ee_strg_none == s_modinfo->info.storage.type) ? ("none") 
                                                                                  : ( (ee_strg_eflash==s_modinfo->info.storage.type) ? ("flash") : ("eeprom") ) );
                retlen += sprintf ((char *)rep + retlen,"   stg.addr  = 0x%0.8X\r\n", s_modinfo->info.storage.addr);
                retlen += sprintf ((char *)rep + retlen,"   stg.size  = 0x%0.8X\r\n", s_modinfo->info.storage.size);
                retlen += sprintf ((char *)rep + retlen,"   com.msk   = 0x%0.8X\r\n", s_modinfo->info.communication);
            }
    
        }
        else
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n cannot get info from shal-part");
        }
    
       
    }


    // ## shals -
    else if(0 == strncmp(cmd, "shals", strlen("shals")))
    {

 
        eEresult_t shalres;
        const eEsharlib_t s_shartable[10] = {ee_shalBASE, ee_shalPART, ee_shalINFO, 
                                             ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone, ee_shalNone}; 
        const eEmoduleInfo_t *s_modinfo = NULL;            

        s_size08 = 3;
   

        if(1)
        {
    
            retlen += sprintf ((char *)rep + retlen,"\r\n there are %d shals :\r\n", s_size08);
            
    
            for(i=0; i<s_size08; i++)
            {
                shalres = shalpart_shal_get(s_shartable[i], &s_modinfo);
                retlen += sprintf ((char *)rep + retlen," shal %d:\r\n", i);
    
                if(ee_res_OK != shalres)
                {
                    retlen += sprintf ((char *)rep + retlen,"   not present in shared partition table\r\n");  
                    continue;  
                }

                retlen += sprintf ((char *)rep + retlen," e-shal %d:\r\n", i);

                retlen += sprintf ((char *)rep + retlen,"   name     = %s\r\n", s_modinfo->info.name);
                retlen += sprintf ((char *)rep + retlen,"   version  = %d.%d. w/ build done on %d.%d.%d at hour %d.%d\r\n", 
                                                                                    s_modinfo->info.entity.version.major, 
                                                                                    s_modinfo->info.entity.version.minor,
                                                                                    s_modinfo->info.entity.builddate.year,
                                                                                    s_modinfo->info.entity.builddate.month,
                                                                                    s_modinfo->info.entity.builddate.day,
                                                                                    s_modinfo->info.entity.builddate.hour,
                                                                                    s_modinfo->info.entity.builddate.min
                                                                                    );
                retlen += sprintf ((char *)rep + retlen,"   rom.addr  = 0x%0.8X\r\n", s_modinfo->info.rom.addr);
                retlen += sprintf ((char *)rep + retlen,"   rom.size  = 0x%0.8X\r\n", s_modinfo->info.rom.size);
                retlen += sprintf ((char *)rep + retlen,"   ram.addr  = 0x%0.8X\r\n", s_modinfo->info.ram.addr);
                retlen += sprintf ((char *)rep + retlen,"   ram.size  = 0x%0.8X\r\n", s_modinfo->info.ram.size);

                retlen += sprintf ((char *)rep + retlen,"   stg.type  = %s\r\n", (ee_strg_none == s_modinfo->info.storage.type) ? ("none") 
                                                                                  : ( (ee_strg_eflash==s_modinfo->info.storage.type) ? ("flash") : ("eeprom") ) );
                retlen += sprintf ((char *)rep + retlen,"   stg.addr  = 0x%0.8X\r\n", s_modinfo->info.storage.addr);
                retlen += sprintf ((char *)rep + retlen,"   stg.size  = 0x%0.8X\r\n", s_modinfo->info.storage.size);
                retlen += sprintf ((char *)rep + retlen,"   com.msk   = 0x%0.8X\r\n", s_modinfo->info.communication);

            }
    
        }
        else
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n cannot get info from shal-part");
        }

    }

    // ## syncprocs
    else if(0 == strncmp(cmd, "syncprocs", strlen("syncprocs")))
    {
        eEmoduleInfo_t * volatile s_modinfo = NULL;
        uint8_t i;
        eEresult_t res = ee_res_NOK_generic;
        const eEmoduleInfo_t *mi = NULL;

        retlen += sprintf ((char *)rep + retlen,"\r\n");

        for(i=0; i<ee_procMaxNum; i++)
        {
            uint32_t romaddr = 0;
            switch(i)
            {
                case 0:     romaddr = EENV_MEMMAP_ELOADER_ROMADDR + EENV_MODULEINFO_OFFSET;         break;
                case 1:     romaddr = EENV_MEMMAP_EUPDATER_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                case 2:     romaddr = EENV_MEMMAP_EAPPLICATION_ROMADDR + EENV_MODULEINFO_OFFSET;    break;
                default:    romaddr = 0;
            }
            
            if(0 != romaddr)
            {
                mi = (const eEmoduleInfo_t *)romaddr;
                if((ee_entity_process == mi->info.entity.type) && (i == mi->info.entity.signature))
                {
                    shalpart_proc_synchronise((eEprocess_t)i, mi);
                    retlen += sprintf ((char *)rep + retlen,"sync-ed proc %d: %s\r\n", i, mi->info.name);
                }
                else
                {
                    // remove it
                    shalpart_proc_rem((eEprocess_t)i);
                    retlen += sprintf ((char *)rep + retlen,"removed proc %d\r\n", i);
                }
            }


        }

    }

    // ## syncshals
    else if(0 == strncmp(cmd, "syncshals", strlen("syncshals")))
    {
        eEmoduleInfo_t * volatile s_modinfo = NULL;
        uint8_t i;
        eEresult_t res = ee_res_NOK_generic;
        const eEmoduleInfo_t *mi = NULL;

        retlen += sprintf ((char *)rep + retlen,"\r\n");

        for(i=0; i<ee_shalMaxNum; i++)
        {
            uint32_t romaddr = 0;
            switch(i)
            {
                case 0:     romaddr = EENV_MEMMAP_SHALBASE_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                case 1:     romaddr = EENV_MEMMAP_SHALPART_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                case 2:     romaddr = EENV_MEMMAP_SHALINFO_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                case 3:     romaddr = EENV_MEMMAP_SHALHAL_ROMADDR + EENV_MODULEINFO_OFFSET;         break;
                case 4:     romaddr = EENV_MEMMAP_SHALOSAL_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                case 5:     romaddr = EENV_MEMMAP_SHALIPAL_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                case 6:     romaddr = EENV_MEMMAP_SHALFSAL_ROMADDR + EENV_MODULEINFO_OFFSET;        break;
                default:    romaddr = 0;
            }
            
            if(0 != romaddr)
            {
                mi = (const eEmoduleInfo_t *)romaddr;
                if((ee_entity_sharlib == mi->info.entity.type) && (i == mi->info.entity.signature))
                {
                    shalpart_shal_synchronise((eEsharlib_t)i, mi);
                    retlen += sprintf ((char *)rep + retlen,"sync-ed shal %d: %s\r\n", i, mi->info.name);
                }
            }


        }

    }

    // ## defproc n
    else if(0 == strncmp(cmd, "defproc", strlen("defproc")))
    {
        eEmoduleInfo_t * volatile s_modinfo = NULL;
        uint8_t arg;
        eEresult_t res = ee_res_NOK_generic;

        // check file name
        if(cmdlen > (strlen("defproc")+1))
        {
            sscanf((const char*)(cmd+strlen("defproc")+1), "%d", &arg);

        }
        else
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n error: missing a valid eproc number usage: defproc n (n from procs command)");
            return(retlen);
        }

        if(arg <= (eEprocess_t)(ee_procMaxNum-1))
        {
            res = shalpart_proc_def2run_set((eEprocess_t)arg);

        }
        else
        {
            res = ee_res_NOK_generic;
        }

        if(ee_res_OK == res)
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n proc %d is now teh default one", arg);
            return(retlen);
        }
        else
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n error: could not set proc %d as default", arg);
            return(retlen);
        }



    }


    // ## verify -
    else if(0 == strncmp(cmd, "verify", strlen("verify")))
    {
        eEmoduleInfo_t * volatile s_modinfo = NULL;
        char filename[17] = {0};
        uint8_t kindoffile = 0; // 1 is bin, 2 is hex

        // check file name
        if(cmdlen > (strlen("verify")+1))
        {
            sscanf((const char*)(cmd+strlen("verify")+1), "%16s", filename);

            if(0 == strncmp(filename+strlen(filename)-4, ".bin", strlen(".bin")))
            {
                kindoffile = 1;
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is recognised as a .bin file", filename);
            }
            else if(0 == strncmp(filename+strlen(filename)-4, ".hex", strlen(".hex")))
            {
                kindoffile = 2;
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is recognised as a .hex file", filename);
            }
            else
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not recognised as a .hex or .bin file", filename);
                return(retlen);
            }
        }
        else
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n error: missing a file.[bin, hex] usage: load file.[bin, hex]");
            return(retlen);
        }

        if(1 == kindoffile)
        {
            // bin
            if(hexparres_OK != hexpar_init(filename))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not present in internal file system. load it w/ ftp", filename);
            }
            else if(hexparres_OK != hexparVerifyBin(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH, (eEmoduleInfo_t **)&s_modinfo))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not a valid .bin file or does not contain a valid e-proc in range [0x%0.8X, 0x%0.8X)", filename, ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
            }
            else
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s contains a valid e-proc in range [0x%0.8X, 0x%0.8X):\r\n", filename, ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);

                retlen += sprintf ((char *)rep + retlen,"   name     = %s\r\n", s_modinfo->info.name);
                retlen += sprintf ((char *)rep + retlen,"   version  = %d.%d. w/ build done on %d.%d.%d at hour %d.%d\r\n", 
                                                                                    s_modinfo->info.entity.version.major, 
                                                                                    s_modinfo->info.entity.version.minor,
                                                                                    s_modinfo->info.entity.builddate.year,
                                                                                    s_modinfo->info.entity.builddate.month,
                                                                                    s_modinfo->info.entity.builddate.day,
                                                                                    s_modinfo->info.entity.builddate.hour,
                                                                                    s_modinfo->info.entity.builddate.min
                                                                                    );
                retlen += sprintf ((char *)rep + retlen,"   rom.addr  = 0x%0.8X\r\n", s_modinfo->info.rom.addr);
                retlen += sprintf ((char *)rep + retlen,"   rom.size  = 0x%0.8X\r\n", s_modinfo->info.rom.size);
                retlen += sprintf ((char *)rep + retlen,"   ram.addr  = 0x%0.8X\r\n", s_modinfo->info.ram.addr);
                retlen += sprintf ((char *)rep + retlen,"   ram.size  = 0x%0.8X\r\n", s_modinfo->info.ram.size);

                retlen += sprintf ((char *)rep + retlen,"   stg.type  = %s\r\n", (ee_strg_none == s_modinfo->info.storage.type) ? ("none") 
                                                                                  : ( (ee_strg_eflash==s_modinfo->info.storage.type) ? ("flash") : ("eeprom") ) );
                retlen += sprintf ((char *)rep + retlen,"   stg.addr  = 0x%0.8X\r\n", s_modinfo->info.storage.addr);
                retlen += sprintf ((char *)rep + retlen,"   stg.size  = 0x%0.8X\r\n", s_modinfo->info.storage.size);
            }

            hexpar_deinit();

        }
        else if(2 == kindoffile)
        {

            if(hexparres_OK != hexpar_init(filename))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not present in internal file system. load it w/ ftp", filename);
            }
            else if(hexparres_OK != hexparVerify(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH, &s_lower, &s_upper))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s does not contain a valid e-proc in range [0x%0.8X, 0x%0.8X)", filename, ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
            }
            else
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s contains a valid e-proc in range [0x%0.8X, 0x%0.8X)", filename, ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
            }
    
            hexpar_deinit();
        }

    }


    // ## remove -
    else if(0 == strncmp(cmd, "remove", strlen("remove")))
    {
        hexparErasePages(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
 
        shalpart_proc_rem(ee_procApplication);
        shalpart_proc_def2run_set(ee_procUpdater);
        retlen += sprintf ((char *)rep + retlen,"\r\n e-appl removed from flash and from partition table. Made e-updater the default e-process");

    }


    // ## load -
    else if(0 == strncmp(cmd, "load", strlen("load")))
    {
        eEmoduleInfo_t * volatile s_modinfo = NULL;
        char filename[17] = {0};
        uint8_t kindoffile = 0; // 1 is bin, 2 is hex

        // check file name
        if(cmdlen > (strlen("load")+1))
        {
            sscanf((const char*)(cmd+strlen("load")+1), "%16s", filename);

            if(0 == strncmp(filename+strlen(filename)-4, ".bin", strlen(".bin")))
            {
                kindoffile = 1;
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is recognised as a .bin file", filename);
            }
            else if(0 == strncmp(filename+strlen(filename)-4, ".hex", strlen(".hex")))
            {
                kindoffile = 2;
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is recognised as a .hex file", filename);
            }
            else
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not recognised as a .hex or .bin file", filename);
                return(retlen);
            }
        }
        else
        {
            retlen += sprintf ((char *)rep + retlen,"\r\n error: missing a file.[bin, hex] usage: load file.[bin, hex]");
            return(retlen);
        }

        if(1 == kindoffile)
        {

           // manage a bin file

            // 1. binpar_init(filename);                                   // will open file
            // 2. binpar_verify(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH, &procinfo);      // will verify that size is not allowed flash size. 
            //                                                             // maybe also that at location +0x200 there is a valid data structure.
            //                                                             // it may also return back the full eEmoduleInfo_t 
            // 3. binpar_erase(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
            // 4. get a pointer (hopefully of 2k bytes) w/ its size. write it to flash
            // 5. print info about the flashed proc.


            if(hexparres_OK != hexpar_init(filename))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not present in internal file system. load it w/ ftp", filename);
            }
            else if(hexparres_OK != hexparVerifyBin(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH, (eEmoduleInfo_t **)&s_modinfo))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not a valid .bin file or does not contain a valid e-proc in range [0x%0.8X, 0x%0.8X)", filename, ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
            }
            else
            {
                hexparErasePages(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
        
                s_eof_flag = 0;
        
                for(;;) 
                {
                    hexparGetBinData(&s_data, &s_size16, &s_addrdata, &s_eof_flag);
        
                    if(1 == s_eof_flag)
                    {
                        break;    
                    }
        
                    if(0 != s_size16)
                    {
                        hexparWriteData(s_addrdata, s_data, s_size16);
                    }
                }
    
         
                // and now add the proc info diretcly. we know that the eEmoduleInfo_t is located at
                // start address + EENV_MODULEINFO_OFFSET, thus ....
                s_modinfo = (eEmoduleInfo_t*)(ADR_DEF_APP_LOW + EENV_MODULEINFO_OFFSET);
                shalpart_proc_add(ee_procApplication, s_modinfo); 
                shalpart_proc_def2run_set(ee_procApplication);  
        
               
            
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s contains the e-proc ""%s"": now is burned to flash, added to part table, made it def on boot", filename, s_modinfo->info.name );
             }
    
            hexpar_deinit();
        

        }
        else if(2 == kindoffile)
        {
            // manage an hex file
            if(hexparres_OK != hexpar_init(filename))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not present in internal file system. load it w/ ftp", filename);
            }
            else if(hexparres_OK != hexparVerify(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH, &s_lower, &s_upper))
            {
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s is not a valid .hex file or does not contain a valid e-proc in range [0x%0.8X, 0x%0.8X)", filename, ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
            }
            else
            {
                hexparErasePages(ADR_DEF_APP_LOW, ADR_DEF_APP_HIGH);
        
                s_eof_flag = 0;
        
                for(;;) 
                {
                    hexparGetData(&s_data, &s_size08, &s_addrdata, &s_eof_flag);
        
                    if(1 == s_eof_flag)
                    {
                        break;    
                    }
        
                    if(0 != s_size08)
                    {
                        hexparWriteData(s_addrdata, s_data, s_size08);
                    }
                }
    
         
                // and now add the proc info diretcly. we know that the eEmoduleInfo_t is located at
                // start address + 0x200, thus ....
                s_modinfo = (eEmoduleInfo_t*)(ADR_DEF_APP_LOW + EENV_MODULEINFO_OFFSET);
                shalpart_proc_add(ee_procApplication, s_modinfo); 
                shalpart_proc_def2run_set(ee_procApplication);  
        
               
            
                retlen += sprintf ((char *)rep + retlen,"\r\n file %s contains the e-proc ""%s"": now is burned to flash, added to part table, made it def on boot", filename, s_modinfo->info.name );
             }
    
            hexpar_deinit();
        }
    }


    // ## ipset -
    else if(0 == strncmp(cmd, "ipset", strlen("ipset")))
    {
        int ip1, ip2, ip3, ip4;
        eEipnetwork_t ipnetwork;
        const eEipnetwork_t *ipnetworkstrg;



        if(cmdlen > (strlen("ipset")+1))
        {
            sscanf((const char*)(cmd+strlen("ipset")+1), "%d.%d.%d.%d", 
                                                         (int*)&ip1, (int*)&ip2, 
                                                         (int*)&ip3, (int*)&ip4);

            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);
            memcpy(&ipnetwork, ipnetworkstrg, sizeof(eEipnetwork_t));

            ipnetwork.ipaddress = EECOMMON_ipaddr_from(ip1, ip2, ip3, ip4);
            shalinfo_deviceinfo_part_set(shalinfo_ipnet, (const void*)&ipnetwork);

            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);

            retlen += sprintf ((char *)rep + retlen,"\r\n ip set to: %d.%d.%d.%d, and effective from next bootstrap", ip1, ip2, ip3, ip4);
            ip1 = (ipnetworkstrg->ipaddress >> 24) & 0xff; 
            ip2 = (ipnetworkstrg->ipaddress >> 16) & 0xff;
            ip3 = (ipnetworkstrg->ipaddress >> 8) & 0xff;
            ip4 = (ipnetworkstrg->ipaddress >> 0) & 0xff;
            retlen += sprintf ((char *)rep + retlen,"\r\n stored ip: %d.%d.%d.%d", ip1, ip2, ip3, ip4);

        }
        else
        {
            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);
            ip1 = (ipnetworkstrg->ipaddress >> 24) & 0xff; 
            ip2 = (ipnetworkstrg->ipaddress >> 16) & 0xff;
            ip3 = (ipnetworkstrg->ipaddress >> 8) & 0xff;
            ip4 = (ipnetworkstrg->ipaddress >> 0) & 0xff;
            retlen += sprintf ((char *)rep + retlen,"\r\n curr ip is: %d.%d.%d.%d", ip1, ip2, ip3, ip4);
            retlen += sprintf ((char *)rep + retlen,"\r\n you need to digit an ip ad.dr.e.ss if you want to change it");
        } 

    }


    // ## mskset -
    else if(0 == strncmp(cmd, "mskset", strlen("mskset")))
    {
        int ip1, ip2, ip3, ip4;
        eEipnetwork_t ipnetwork;
        const eEipnetwork_t *ipnetworkstrg;

        

        if(cmdlen > (strlen("mskset")+1))
        {
            sscanf((const char*)(cmd+strlen("mskset")+1), "%d.%d.%d.%d", 
                                                         (int*)&ip1, (int*)&ip2, 
                                                         (int*)&ip3, (int*)&ip4);

            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);
            memcpy(&ipnetwork, ipnetworkstrg, sizeof(eEipnetwork_t));

            ipnetwork.ipnetmask = EECOMMON_ipaddr_from(ip1, ip2, ip3, ip4);
            shalinfo_deviceinfo_part_set(shalinfo_ipnet, (const void*)&ipnetwork);

            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);

            retlen += sprintf ((char *)rep + retlen,"\r\n ip set to: %d.%d.%d.%d, and effective from next bootstrap", ip1, ip2, ip3, ip4);
            ip1 = (ipnetworkstrg->ipnetmask >> 24) & 0xff; 
            ip2 = (ipnetworkstrg->ipnetmask >> 16) & 0xff;
            ip3 = (ipnetworkstrg->ipnetmask >> 8) & 0xff;
            ip4 = (ipnetworkstrg->ipnetmask >> 0) & 0xff;
            retlen += sprintf ((char *)rep + retlen,"\r\n stord msk: %d.%d.%d.%d", ip1, ip2, ip3, ip4);

        }
        else
        {
            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);
            ip1 = (ipnetworkstrg->ipnetmask >> 24) & 0xff; 
            ip2 = (ipnetworkstrg->ipnetmask >> 16) & 0xff;
            ip3 = (ipnetworkstrg->ipnetmask >> 8) & 0xff;
            ip4 = (ipnetworkstrg->ipnetmask >> 0) & 0xff;
            retlen += sprintf ((char *)rep + retlen,"\r\n curr msk is: %d.%d.%d.%d", ip1, ip2, ip3, ip4);
            retlen += sprintf ((char *)rep + retlen,"\r\n you need to digit an ip m.a.s.k if you want to change it");
        } 


    }
    // ## restart -
    else if(0 == strncmp(cmd, "restart", strlen("restart")))
    {
        osal_timer_timing_t timing = { .startat = osal_abstimeNONE, .count = 3*1000*1000, .mode = osal_tmrmodeONESHOT };
        osal_timer_onexpiry_t onexpiry = { .cbk = s_restart, .par = NULL};
        osal_timer_t *tmr = NULL;
        retlen += sprintf ((char *)rep + retlen,"\r\n closing telnet session immediately and restarting system in three seconds ...");
        // better ... calls a timer which calls the restart in 2 seconds.
        //shalbase_system_restart();

        tmr = osal_timer_new();
        if(NULL == tmr)
        {
            shalbase_system_restart();
        }
 
        osal_timer_start(tmr, &timing, &onexpiry, osal_callerTSK);

        *quitflag = 1;
    }


    // ## end -
    else
    {
        retlen += sprintf ((char *)rep + retlen,"\r\n command not found. type help for supported commands");
    }



    return(retlen);

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern uint16_t apptelnet_onexecsafe(const char *cmd, char *rep, uint8_t *quitflag)
{
    uint16_t cmdlen =0;
    static uint16_t cnt = 0;
    uint16_t retlen = 0;
    int arg1 = 0;

    cmdlen = strlen(cmd);
    *quitflag = 0;

    if(0 == strncmp(cmd, "help", strlen("help")))
    {
        if(cmdlen > 5)
        {
            sscanf((const char*)(cmd+5), "%d", &arg1);
            ipal_ftp_stop();
        } 
        
        retlen += sprintf ((char *)rep + retlen,"\r\n help found aaa # %d w/ arg %d", cnt++, arg1);    

    }
    else if(0 == strncmp(cmd, "quit", strlen("quit")))
    {
        retlen += sprintf ((char *)rep + retlen,"\r\n quitting telnet... bye\r\n"); 
        *quitflag = 1;
        ipal_ftp_restart();
    }
    else
    {
        retlen += sprintf ((char *)rep + retlen,"\r\n command not found. type help for supported commands");
    }

    return(retlen);
}



  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_restart(void *tmr, void *arg)
{
    shalbase_system_restart();
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



