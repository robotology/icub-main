
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _IPAL_CFG_H_
#define _IPAL_CFG_H_

// --------------------------------------------------------------------------------------------------------------------
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------


// <h> Configuration of IPAL
// <i> It holds configuration for objects used in IPAl


// <h> Porting specifics 
// <i> sssssssss

//   <o> Used TCP/IP stack         <0=>   IIT modified TCPNET
//   <i> Only IIT modified TCPNET is now supported.
#ifndef IPAL_TCPIPTYPE
 #define IPAL_TCPIPTYPE      0
#endif


//   <o> Memory model         <0=>   static
//   <i> Only static model is now supported.
#ifndef IPAL_MEMMODEL
 #define IPAL_MEMMODEL      0
#endif

// </h>Porting

// <h> System 
// <i> sssssssss

//   <o> Time tick for internal counters     <10000=> 10 ms <20000=> 20 ms <25000=> 25 ms
//                          <50000=> 50 ms <100000=> 100 ms  <200000=> 200 ms <250000=> 250 ms <500000=> 500 ms
//   <i> The IPAL must be ticked every time interval in here defined
#ifndef IPAL_TIMETICK
 #define IPAL_TIMETICK      200000
#endif

//   <o> Memory Pool size [Bytes]      <1500-32000:4>
//   <i> It keeps buffers for IP transmission.
#ifndef IPAL_MEMPOOLSIZE
 #define IPAL_MEMPOOLSIZE      6144
#endif

// </h>System


// <h> Network interface 
// <i> Defines network interface 


//   <e> Ethernet 
//   <i> Use Ethernet as network interface
#ifndef IPAL_ETH_ENABLE
 #define IPAL_ETH_ENABLE      1
#endif

//   <o> ETH ISR priority     <0-15>
//   <i> The IPAL needs an ETH ISR teh priority of which can be specified (lower number, higher priority)
#ifndef IPAL_ETH_ISRPRIO
 #define IPAL_ETH_ISRPRIO      1
#endif

//  <h> MAC address

//   <o> MAC OUI <0x00000000-0x00FFFFFF> 
//   <i> IEEE-assigned part of the MAC (first three octets).
//   <i> Default: 0x010103
#ifndef IPAL_MACOUI
 #define IPAL_MACOUI        1978476
#endif

//   <o> MAC final 3 octets <0x00000000-0x00FFFFFF> 
//   <i> Free-assigned part of the MAC (last three octets).
//   <i> Default: 0x010103
#ifndef IPAL_MAC3OCT
 #define IPAL_MAC3OCT        10634152
#endif

//  </h>MAC address



//  <h> Address Resolution Protocol
//  <i> Address Resolution Protocol

//     <o> Cache table size <4-100>
//     <i> Number of cached hardware/IP addresses
//     <i> Default: 10
#define IPAL_ARP_CACHESIZE    8

//     <o> Cache Timeout [sec] <5-255><#*1000000>
//     <i> A timeout for a cached hardware/IP addresses
//     <i> Default: 150
#define IPAL_ARP_CACHETIMEOUT    255000000

//     <o> Number of retries <0-20>
//     <i> Number of retries to resolve an IP address
//     <i> before ARP module gives up
//     <i> Default: 4
#define IPAL_ARP_RETRYMAXNUM   5

//     <o> Retry Timeout [sec] <1-10><#*1000000>
//     <i> A timeout to resend the ARP Request
//     <i> Default: 1
#define IPAL_ARP_RETRYTIMEOUT     1000000

//     <q> Autonotify enabled
//     <i> When this option is enabled, the embedded host
//     <i> will send a gratuitous ARP notification at startup,
//     <i> or when the device IP address has changed.
//     <i> Default: Enabled
#define IPAL_ARP_AUTONOTIFY     1


//  </h>Address Resolution Protocol


//   </e>Ethernet 

// </h>Network interface 


//  <h> IP Network

//  <h> Fixed IP address
//  <i> Used if DHCP is disabled.
//  <i> Default: 10.255.39.151

//   <o> IP0 <0-255> 
#ifndef IPAL_IP0
 #define IPAL_IP0        10
#endif
//   <o> IP1 <0-255> 
#ifndef IPAL_IP1
 #define IPAL_IP1        255
#endif
//   <o> IP2 <0-255> 
#ifndef IPAL_IP2
 #define IPAL_IP2        39
#endif
//   <o> IP3 <0-255> 
#ifndef IPAL_IP3
 #define IPAL_IP3        152
#endif

//  </h>Fixed IP address


//  <h> Subnet mask
//  <i> Default: 255.255.252.0

//   <o> MSK0 <0-255> 
#ifndef IPAL_MSK0
 #define IPAL_MSK0        255
#endif
//   <o> MSK1 <0-255> 
#ifndef IPAL_MSK1
 #define IPAL_MSK1        255
#endif
//   <o> MSK2 <0-255> 
#ifndef IPAL_MSK2
 #define IPAL_MSK2        252
#endif
//   <o> MSK3 <0-255> 
#ifndef IPAL_MSK3
 #define IPAL_MSK3        0
#endif

//  </h>Subnet mask


//  </h>IP Network


// <h> IP Protocols


// <e> DHCP client
// <i> If enabled, the IP address is achieved using DHCP. It needs one dedicated UDP socket
#define IPAL_DHCP_ENABLE     0
// </e>DHCP

// <e> UDP
// <i> If enabled, you can use UDP sockets and services whcih rely on them (DHCP client, NBNS, IGMP?, DNS client?)
#define IPAL_UDP_ENABLE     1

//     <o> Number of UDP sockets <1-20>
//     <i> Number of available UDP sockets (DHCP uses one)
//     <i> Default: 3
#define IPAL_UDP_SOCKETNUM     5

// </e>UDP



// <e> IGMP
// <i> The Internet Group Management Protocol (IGMP) is a communications protocol used 
// <i> to manage the membership of Internet Protocol multicast groups.
#define IPAL_IGMP_ENABLE     1

//     <o> Max number of IGMP groups <2-10>
//     <i> Number of groups this host can join
//     <i> Default: 3
#define IPAL_IGMP_GROUPSNUM     4

// </e>IGMP




// <e> TCP
// <i> If enabled, you can use TCP sockets and services based on them (Telnet, FTP) 
#define IPAL_TCP_ENABLE     0

//     <o> Number of TCP sockets <1-20>
//     <i> Number of available TCP sockets (Telnet uses one per session, FTP uses two per session)
//     <i> Default: 5
#define IPAL_TCP_SOCKETNUM     3

//   <o> Number of retries <0-20>
//   <i> How many times TCP module will try to retransmit data
//   <i> before giving up. Increase this value for high-latency
//   <i> and low_throughput networks.
//   <i> Default: 5
#define IPAL_TCP_RETRYMAXNUM   5

//   <o> Retry timeout [sec] <1-10><#*1000000>
//   <i> If data frame not acknowledged within this time frame,
//   <i> TCP module will try to resend data again
//   <i> Default: 4
#define IPAL_TCP_RETRYTOUT  2000000

//   <o> Default connection timeout [sec] <1-1800><#*1000000>
//   <i> Default TCP Socket Keep Alive timeout. When it expires
//   <i> with no TCP data frame send, TCP Connection is closed.
//   <i> Default: 120
#define IPAL_TCP_CONNECTIONTIMEOUT    1800000000

// </e>TCP



// <e> TFTP server
// <i> Enable or disable TFTP Server (single session only)
#define IPAL_TFTP_ENABLE     0

//   <o> Port Number <1-65535>
//   <i> Listening port number.
//   <i> Default: 69
#define IPAL_TFTP_PORT    69

//   <o> Number of Retries <1-10>
//   <i> How many times TFTP Server will try to retransmit data
//   <i> before giving up.
//   <i> Default: 4
#define IPAL_TFTP_RETRYMAXNUM  4

//   <o> Inactive session timeout [sec] <5-120><#*1000000>
//   <i> When timeout expires TFTP Session is closed. This timeout
//   <i> is used when the UDP connection is broken because of error.
//   <i> Default: 15
#define IPAL_TFTP_TIMEOUT   15000000

// </e>TFTP server



// <e> FTP server
// <i> Enable or disable FTP Server (single session only: each connection uses 2 TCP sockets)
#define IPAL_FTP_ENABLE     0


//   <o> Port Number <1-65535>
//   <i> Listening port number.
//   <i> Default: 21
#define IPAL_FTP_PORT    21

//   <e> Enable user authentication
//   <i> When enabled, the user will have to authenticate
//   <i> himself by username and password before access
//   <i> to the system is allowed.
#define IPAL_FTP_AUTHENABLE     1

//     <s.15> Authentication username
//     <i> Default: "admin"
#define IPAL_FTP_USER   "admin"

//     <s.15>Authentication Password
//     <i> Default: "admin"
#define IPAL_FTP_PASS  "admin"

//   </e>Enable user authentication

// </e>FTP server



// <e> Telnet server

// <i> Enable or disable Telnet Server
#define IPAL_TEL_ENABLE    0

//   <o> Number of Telnet Connections <1-2>
//   <i> Number of simultaneously active Telnet Connections.
//   <i> Modify also the number of TCP Sockets because
//   <i> each Telnet connection uses it's own TCP socket
//   <i> Default: 1
#define IPAL_TEL_CONNSNUM   1

//   <o> Port Number <1-65535>
//   <i> Listening port number.
//   <i> Default: 23
#define IPAL_TEL_PORT   23

//   <e> Enable user authentication
//   <i> When enabled, the user will have to authenticate
//   <i> himself by username and password before access
//   <i> to the system is allowed.
#define IPAL_TEL_AUTHENABLE    1

//     <s.15>Authentication Username
//     <i> Default: "admin"
#define IPAL_TEL_USER  "admin"

//     <s.15> Authentication Password
//     <i> Default: "admin"
#define IPAL_TEL_PASS "admin"

//   </e>Enable user authentication

// </e>Telnet server



// </h>IP Protocols



// </h>



// --------------------------------------------------------------------------------------------------------------------
//------------- <<< end of configuration section >>> ------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------


// - some controls ----------------------------------------------------------------------------------------------------

#if(0 != IPAL_TCPIPTYPE)
    #error only tcpnet is supported so far
#endif

#if(0 == IPAL_ETH_ENABLE)
    #error must have ethernet network interface enabled
#endif

#if(((IPAL_FTP_ENABLE*2)+IPAL_TEL_ENABLE) > IPAL_TCP_SOCKETNUM)
    #error need 2 tcp sockets for ftp and 1 for telnet
#endif


#if(((IPAL_DHCP_ENABLE)+IPAL_TFTP_ENABLE) > IPAL_UDP_SOCKETNUM)
    #error need 1 upd socket for tftp and 1 for dchp
#endif



// - end of controls --------------------------------------------------------------------------------------------------


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


