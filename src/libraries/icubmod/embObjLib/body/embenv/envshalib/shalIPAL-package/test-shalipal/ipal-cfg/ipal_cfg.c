
/* @file       ipal_cfg.c
	@brief      This file keeps internal implementation of the ipal.
	@author     marco.accame@iit.it
    @date       06/07/2010
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "ipal.h"

#include "hal.h"
#include "fsal.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "ipal_cfg.h"

extern uint16_t tnet_onexec(const char *cmd, char *rep, uint8_t *quitflag);

static void onethframerx(void);

static const char s_tel_msg_login[] = 
{
    "                                           \r\n"
    "       chl embedded telnet                 \r\n"
    "                                           \r\n"
};

static const char s_tel_msg_welcome[] = 
{
    "                                           \r\n"
    "       type help for supported commands    \r\n"
    "                                           \r\n"
};

static const char s_tel_prompt[] = "\r\ntnet> ";

static void s_ipal_cfg_on_fatal_error(uint32_t errorcode, const char * errormsg);

extern uint64_t macnn = IPAL_mac48addr(0x1e, 0x30, 0x6c, 0xa2, 0x45, 0x5e);  

static const ipal_params_cfg_t s_cfg = 
{ 

    // sys
    .arch_tcpiptype         = IPAL_TCPIPTYPE,
    
    .sys_timetick           = IPAL_TIMETICK,          // uint32_t        sys_timetick;       // in usec, but multiple of 10 ms. upto 200 ms.  

    .sys_mempoolsize        = IPAL_MEMPOOLSIZE,       // uint16_t        sys_mempoolsize;
    
    .eth_mac                = IPAL_mac48addr(0xFF&(IPAL_MACOUI>>16), 0xFF&(IPAL_MACOUI>>8), 0xFF&(IPAL_MACOUI), 0xFF&(IPAL_MAC3OCT>>16), 0xFF&(IPAL_MAC3OCT>>8), 0xFF&(IPAL_MAC3OCT)),

    .eth_ip                 = IPAL_ipv4addr(IPAL_IP0, IPAL_IP1, IPAL_IP2, IPAL_IP3),    //uint8_t         eth_ip[4];
    .eth_mask               = IPAL_ipv4addr(IPAL_MSK0, IPAL_MSK1, IPAL_MSK2, IPAL_MSK3),        //uint8_t         eth_mask[4];
    
    // arp
    .arp_cachesize          = IPAL_ARP_CACHESIZE,     // uint8_t         arp_cachesize;
    .arp_cachetimeout       = IPAL_ARP_CACHETIMEOUT,  // uint32_t        arp_cachetimeout;   // in usec, but multiple of seconds
    .arp_retrymaxnum        = IPAL_ARP_RETRYMAXNUM,   //uint8_t         arp_retrymaxnum;
    .arp_retrytimeout       = IPAL_ARP_RETRYTIMEOUT,  //uint32_t        arp_retrytimeout;   // in usec, but multiple of seconds
    .arp_autonotify         = IPAL_ARP_AUTONOTIFY,    //uint8_t         arp_autonotify;
    
    // igmp
    .igmp_enable            = IPAL_IGMP_ENABLE,       //uint8_t         igmp_enable;
    .igmp_groupsnum         = IPAL_IGMP_GROUPSNUM,    // uint8_t         igmp_groupsnum;
    

    // dhcp
    .dhcp_enable            = IPAL_DHCP_ENABLE,   // uint8_t         dhcp_enable;
    
    // udp
    .udp_enable             = IPAL_UDP_ENABLE,    //uint8_t         udp_enable;
    .udp_socketnum          = IPAL_UDP_SOCKETNUM, //uint8_t         udp_socketnum;
    
    // tcp sockets
    .tcp_enable             = IPAL_TCP_ENABLE,            // uint8_t         tcp_enable;                     
    .tcp_socketnum          = IPAL_TCP_SOCKETNUM,         //uint8_t         tcp_socketnum;                   
    .tcp_retrymaxnum        = IPAL_TCP_RETRYMAXNUM,       //uint8_t         tcp_retrymaxnum;                   
    .tcp_retrytimeout       = IPAL_TCP_RETRYTOUT,         // uint32_t        tcp_retrytimeout;           // in usec, but multiple of seconds                  
    .tcp_connectiontimeout  = IPAL_TCP_CONNECTIONTIMEOUT, //uint32_t        tcp_connectiontimeout;      // in usec, but multiple of seconds, upto 64 secs

  
    // tftp
    .tftp_enable            = IPAL_TFTP_ENABLE, 
    .tftp_port              = IPAL_TFTP_PORT,                       
    .tftp_retrymaxnum       = IPAL_TFTP_RETRYMAXNUM,         
    .tftp_timeout           = IPAL_TFTP_TIMEOUT,       // in usec but multiple of seconds, upto 60 secs     

    // ftp
    .ftp_enable             = IPAL_FTP_ENABLE,            // uint8_t         ftp_enable;                     
    .ftp_port               = IPAL_FTP_PORT,              // uint16_t        ftp_port;                       
    .ftp_authenable         = IPAL_FTP_AUTHENABLE,        // uint8_t         ftp_authenable;                 
    .ftp_user               = IPAL_FTP_USER,              // uint8_t         ftp_user[8];                
    .ftp_pass               = IPAL_FTP_PASS,              // uint8_t         ftp_pass[8]; 

    // telnet
    .tel_enable             = IPAL_TEL_ENABLE,            //uint8_t         tel_enable;                    
    .tel_port               = IPAL_TEL_PORT,              //uint16_t        tel_port;                      
    .tel_authenable         = IPAL_TEL_AUTHENABLE,        // uint8_t         tel_authenable;                
    .tel_user               = IPAL_TEL_USER,              // uint8_t         tel_user[8];               
    .tel_pass               = IPAL_TEL_PASS,              // uint8_t         tel_pass[8]; 

    .extfn                  = 
    { 
        .usr_on_fatal_error         = s_ipal_cfg_on_fatal_error,

        .osal_mutex_new             = NULL,
        .osal_mutex_take            = NULL,
        .osal_mutex_release         = NULL,
        .osal_param_tout_forever    = 0,

        .hal_eth_init               = (void (*)(void*, void*)) hal_eth_init,
        .hal_eth_enable             = (void (*)(void)) hal_eth_enable,
        .hal_eth_disable            = (void (*)(void)) hal_eth_disable,
        .hal_eth_sendframe          = (void (*)(void*)) hal_eth_sendframe,

        .usr_on_ethframe_received   = onethframerx,

        .fopen                      = fopen,
        .fclose                     = fclose,
        .fwrite                     = fwrite,
        .fread                      = fread,
        .ftell                      = ftell,
        .fseek                      = fseek,

        .fsal_delete                = (int (*)(const char *)) fsal_delete, 
        .fsal_rename                = (int (*)(const char *, const char *)) fsal_rename,
        .fsal_find                  = (int (*)(const char *, void *)) fsal_find,

        .usr_tnet_exec              = tnet_onexec,
        .usr_tnet_login_msg         = s_tel_msg_login,              
        .usr_tnet_welcome_msg       = s_tel_msg_welcome,
        .usr_tnet_prompt            = s_tel_prompt

    } 
};


extern const ipal_params_cfg_t *ipal_params_cfgMINE = &s_cfg;


static void s_ipal_cfg_on_fatal_error(uint32_t errorcode, const char * errormsg)
{
    static volatile uint8_t a = 0;
    for(;;)
    {
        a++;
        a = a;
    }
}

static void onethframerx(void)
{
    static volatile uint8_t b = 0;
    b++;
    b = b;
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



