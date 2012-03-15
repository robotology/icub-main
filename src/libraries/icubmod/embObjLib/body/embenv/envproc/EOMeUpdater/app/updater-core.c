#include "updater-core.h"

#include "stdlib.h"
#include "string.h"

#include "hal.h"
#include "osal.h"
#include "ipal.h"

#include "shalBASE.h"
#include "shalPART.h"
#include "shalINFO.h"
 

typedef enum {
    CMD_SCAN    =0xFF,
    CMD_START   =0x01,
    CMD_DATA    =0x02,
    CMD_JUMP    =0x03,
    CMD_END     =0x04,
    CMD_BOOT    =0x05,
    CMD_RESET   =0x06,
    CMD_IPSET   =0x07,
    CMD_MASKSET =0x08,
    CMD_PROCS   =0x09
} canloader_opcode_t;

enum {
    UPD_OK    = 0,
    UPD_PROT  = 1
};


typedef struct
{
    ipal_ipv4addr_t     ipaddr;
    ipal_port_t         port;
    uint16_t            size;
} datagram_header_t;

typedef struct
{
    datagram_header_t   head;
    uint8_t             data[1];
} datagram_t;

#define BOARD_TYPE_EMS 0x0A

//static osal_messagequeue_t *s_txpktqueue = NULL;
//static osal_task_t *s_tskid_ip = NULL;

static uint32_t s_protected_mem_start = EENV_MEMMAP_EUPDATER_ROMADDR;
static uint32_t s_protected_mem_end   = EENV_MEMMAP_EUPDATER_ROMADDR + EENV_MEMMAP_EUPDATER_ROMSIZE - 1;

void upd_core_init(void)
{
 
}

uint8_t upd_core_manage_cmd(uint8_t *pktin, uint8_t *pktout, uint16_t *sizeout)
{
    static uint8_t s_download_state = 0;      
    static uint8_t s_eeprom_flag = 0;

    datagram_t *rep = NULL;
    datagram_t *rxdgram = (datagram_t*)pktin;

    hal_trace_init();


    *sizeout = 8;
    rep = (datagram_t*)pktout;
    rep->head.ipaddr = 0x12345678;

    return(1);


    switch(rxdgram->data[0]) // opcode
    {
        case CMD_SCAN:
        {
            hal_trace_puts("CMD_SCAN");

            eEmoduleInfo_t* module=(eEmoduleInfo_t*)(EENV_MEMMAP_EUPDATER_ROMADDR+EENV_MODULEINFO_OFFSET);

            //rep = malloc(sizeof(datagram_header_t) + 8);
            *sizeout = sizeof(datagram_header_t) + 8;
            rep = (datagram_t*)pktout;
            rep->head = rxdgram->head;
            rep->head.size = 8;
            rep->data[0] = CMD_SCAN;
            rep->data[1] = BOARD_TYPE_EMS;
            rep->data[2] = module->info.entity.version.major;
            rep->data[3] = module->info.entity.version.minor;
            
            const eEipnetwork_t *ipnetworkstrg;
            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);

            rep->data[4] = (ipnetworkstrg->ipnetmask>>24) & 0xFF;
            rep->data[5] = (ipnetworkstrg->ipnetmask>>16) & 0xFF;
            rep->data[6] = (ipnetworkstrg->ipnetmask>>8)  & 0xFF;
            rep->data[7] =  ipnetworkstrg->ipnetmask      & 0xFF;

            break;
        }

        case CMD_START:
        {
            hal_trace_puts("CMD_START");

            s_eeprom_flag = rxdgram->data[1];
            s_download_state = 1;

            //rep = malloc(sizeof(datagram_header_t) + 1);
            *sizeout = sizeof(datagram_header_t) + 1;
            rep = (datagram_t*)pktout;
            rep->head = rxdgram->head;
            rep->head.size = 1;
            rep->data[0] = CMD_START;
         
            if (!s_eeprom_flag)
            {
                osal_system_scheduling_suspend();
                
                if (hal_res_OK != hal_flash_erase(EENV_MEMMAP_EAPPLICATION_ROMADDR, EENV_MEMMAP_EAPPLICATION_ROMSIZE))
                {
                    s_download_state = 0;
                    hal_trace_puts("ERASE FAILED");
                    
                }
                else
                {
                    hal_trace_puts("ERASE DONE");
                }

                osal_system_scheduling_restart();
            }

            break;
        }
            
        case CMD_DATA:
        {
            hal_trace_puts("CMD_DATA");

            if (s_download_state)
            {
                //rep = malloc(sizeof(datagram_header_t) + 2);
                *sizeout = sizeof(datagram_header_t) + 2;
                rep = (datagram_t*)pktout;
                rep->head = rxdgram->head;
                rep->head.size = 2;
                rep->data[0] = CMD_DATA;
                rep->data[1] = UPD_OK;

                uint32_t address = rxdgram->data[4]<<24 |
                                   rxdgram->data[3]<<16 |
                                   rxdgram->data[2]<<8  |
                                   rxdgram->data[1];

                uint32_t size = rxdgram->head.size - 5;

                void *data = rxdgram->data + 5;
                  
                if (s_eeprom_flag)
                {
                    hal_eeprom_write(hal_eeprom_i2c_01, address, size, data);
                }
                else
                {
                    if ((address > s_protected_mem_end) || (address + size < s_protected_mem_start))
                    { 
                        hal_sys_irq_disable();
                        hal_flash_unlock();
                        hal_flash_write(address, size, data);
                        hal_sys_irq_enable();
                    }
                    else
                    {
                        rep->data[1] = UPD_PROT;
                    }
                }
            }
               
            break;
        }
            
        case CMD_END:
        {
            hal_trace_puts("CMD_END");
   
            if (s_download_state)
            {
                s_download_state = 0;

                //rep = malloc(sizeof(datagram_header_t) + 1);
                *sizeout = sizeof(datagram_header_t) + 1;
                rep = (datagram_t*)pktout;
                rep->head = rxdgram->head;
                rep->head.size = 1;
                rep->data[0] = CMD_END;

                shalpart_proc_synchronise(ee_procApplication,(eEmoduleInfo_t*)(EENV_MEMMAP_EAPPLICATION_ROMADDR+EENV_MODULEINFO_OFFSET)); 
                shalpart_proc_def2run_set(ee_procUpdater);
            }

            break;
        }

        case CMD_BOOT:
        {
            hal_trace_puts("CMD_BOOT");

            eEprocess_t active_part = (eEprocess_t)rxdgram->data[1];

            if ((active_part >= ee_procUpdater) && (active_part <= ee_procApplUser04))
            {
                shalpart_proc_def2run_set(active_part);
            }

            break;
        }

        case CMD_RESET:
        {
            hal_trace_puts("CMD_RESET");
            shalbase_system_restart();
            break;
        }

        case CMD_IPSET:
        {
            hal_trace_puts("CMD_IPSET");

            uint8_t *ip = rxdgram->data;
            
            eEipnetwork_t ipnetwork;
            const eEipnetwork_t *ipnetworkstrg;

            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);
            memcpy(&ipnetwork, ipnetworkstrg, sizeof(eEipnetwork_t));
            ipnetwork.ipaddress = EECOMMON_ipaddr_from(ip[1], ip[2], ip[3], ip[4]);
            
            shalinfo_deviceinfo_part_set(shalinfo_ipnet, (const void*)&ipnetwork);

            break;
        }

        case CMD_MASKSET:
        {
            hal_trace_puts("CMD_MASKSET");
            
            uint8_t *ip = rxdgram->data;
            
            eEipnetwork_t ipnetwork;
            const eEipnetwork_t *ipnetworkstrg;

            shalinfo_deviceinfo_part_get(shalinfo_ipnet, (const void**)&ipnetworkstrg);
            memcpy(&ipnetwork, ipnetworkstrg, sizeof(eEipnetwork_t));
            ipnetwork.ipnetmask = EECOMMON_ipaddr_from(ip[1], ip[2], ip[3], ip[4]);
            shalinfo_deviceinfo_part_set(shalinfo_ipnet, (const void*)&ipnetwork);

            break;
        }

        case CMD_PROCS:
        {
            uint8_t s_size08 = 0;
            const eEprocess_t *s_proctable = NULL;
            const eEmoduleInfo_t *s_modinfo = NULL;
            eEprocess_t defproc;

            shalpart_proc_def2run_get(&defproc);

            if (ee_res_OK == shalpart_proc_allavailable_get(&s_proctable, &s_size08))
            {
                uint16_t data_size =  s_size08 * sizeof(eEinfo_t) + 2;

                //uint8_t *data = malloc(sizeof(datagram_header_t) + data_size);
                uint8_t *data = pktout;
                *sizeout = sizeof(datagram_header_t) + data_size;
                
                rep = (datagram_t*)data;
                rep = (datagram_t*)pktout;
                rep->head = rxdgram->head;
                rep->head.size = data_size;
                rep->data[0] = CMD_PROCS;
                rep->data[1] = s_size08;
                
                eEinfo_t *procs = (eEinfo_t*)(data + sizeof(datagram_header_t) + 2);

                for (uint8_t i=0; i<s_size08; i++)
                {
                    shalpart_proc_get(s_proctable[i], &s_modinfo);
  
                    procs[i] = s_modinfo->info;
                }
            }

            break;
        }

        default:
            hal_trace_puts("DEFAULT");
    }
    
    if(NULL == rep)
    {
        *sizeout = 0;
        return(0);
    }
    else
    {
        return(1);
    }

}
