/*
    Boards_iface.cpp

    Copyright (C) 2012 Italian Institute of Technology

    Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/


#include <Boards_iface.h>

#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <sys/mman.h>
#include <assert.h>
#include <string.h>
#include <bits/local_lim.h>
#include <linux/sched.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <stdexcept>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>

#include <CommProtocol.hpp>

#ifdef __XENO__
    #include <rtdk.h>
    #include <rtnet.h>
    #define DPRINTF rt_printf
#else
    #define DPRINTF printf
#endif

#define RX_UDP_BUFSIZE 1024

#define MC_BOARD    0x02
#define FT_BOARD    0x03

#define BOARD_ID_BYTE_POS    3


const char th_name[] = "udp_rx";

// MOVE in utils or something else
static int getIPv4(const char * dev, char * ipv4)
{
    struct ifreq ifc;
    int res;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0)
        return -1;
    strcpy(ifc.ifr_name, dev);
    res = ioctl(sockfd, SIOCGIFADDR, &ifc);
    close(sockfd);
    if (res < 0)
        return -1;
    strcpy(ipv4, inet_ntoa(((struct sockaddr_in*)&ifc.ifr_addr)->sin_addr));
    return 0;
}

/**
 * Boards_ctrl constructor
 *
 * parse yaml configuration file and set Boards_ctrl::iface and
 * Boards_ctrl::expected_num_boards
 *
 * create udp socket and bind to Boards_ctrl::local_addr
 *
 * @param config    yaml configuration file
 *
 */
Boards_ctrl::Boards_ctrl(const char *config) {

    int     broadcastOn = 1;
    struct timeval tv;

    std::ifstream fin(config);
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
    const YAML::Node& board_ctrl = doc["board_ctrl"];
    std::string iface;

    board_ctrl["eth_iface"] >> iface;
    board_ctrl["boards_num"] >> expected_num_boards;

#ifndef __XENO__
    char    ip[16];
    if ( getIPv4(iface.c_str(), ip) ) {
        printf("\n\ncannot find iface_name %s\n\n\n", iface.c_str());
        assert(0);
    }
#endif

    // create udp socket
    if ( (udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        perror("socket cannot be created");
        assert(0);
    }

    // set socket broadcast option
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcastOn, sizeof(broadcastOn));
    // set socket timeout option
#ifdef __XENO__
    // This socket control option is used to specify the time-out on the socket before it returns.
    // It is used typically to wait for data on a Read.
    // The time-out specifies the amount of time the function will wait for data before it returns.
    int64_t timeout_ns = 250000000;
    if (ioctl(udp_sock, RTNET_RTIOC_TIMEOUT, &timeout_ns) < 0)
        DPRINTF("ioctl RTNET_RTIOC_TIMEOUT failed\n");
#else
    struct timeval timeout;
    timeout.tv_sec =  0;
    timeout.tv_usec = 250000;
    if (setsockopt (udp_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                sizeof(timeout)) < 0)
        DPRINTF("setsockopt SO_RCVTIMEO failed\n");

    if (setsockopt (udp_sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                sizeof(timeout)) < 0)
        DPRINTF("setsockopt SO_SNDTIMEO failed\n");
#endif

    // bind the socket to local_addr
    local_addr.sin_family       = AF_INET;
    local_addr.sin_port         = htons(0);
#ifdef __XENO__
    local_addr.sin_addr.s_addr  = INADDR_ANY;
#else
    local_addr.sin_addr.s_addr  = inet_addr(ip);
#endif
    if (bind(udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        perror("cannot bind to local ip/port");
        close(udp_sock);
        assert(0);
    }

    //
    memset((void*)&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family        = AF_INET;
    dest_addr.sin_port          = htons(23);
    dest_addr.sin_addr.s_addr   = INADDR_BROADCAST;


}

/**
 * Boards_ctrl deconstructor
 *
 * delete recorded DSP_board instances and close udp socket
 *
 */
Boards_ctrl::~Boards_ctrl() {

    for (dsp_map_t::iterator it = _boards.begin(); it != _boards.end(); it++) {
        delete it->second;
    }
    DPRINTF("Delete Dsp_Board\n");

    pthread_mutex_destroy(&data_sync_mutex);
    pthread_cond_destroy(&data_sync_cond);

    close(udp_sock);
}

/**
 * create Boards_ctrl::rx_udp(void *_) thread and initialize
 * mutex and condition variable
 *
 * @return 0
 */
int Boards_ctrl::init(void) {

    pthread_attr_t      attr;
    int                 policy;
    cpu_set_t           cpu_set;
    struct sched_param  schedparam;

    CPU_ZERO(&cpu_set);
    CPU_SET(1,&cpu_set);

    pthread_mutex_init(&data_sync_mutex, NULL);
    pthread_cond_init(&data_sync_cond, NULL);

#ifdef __XENO__
    policy = SCHED_FIFO;
#else
    //policy = SCHED_FIFO;
    policy = SCHED_OTHER;
#endif

    // thread configuration and creation
    pthread_attr_init(&attr);
/*    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, policy);
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_attr_setschedparam(&attr, &schedparam);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
*/
    if ( pthread_create(&rx_upd_thread, &attr, rx_udp, (void*)this) ) {
        perror("pthread_create fail ");
        exit(1);
    }
    pthread_attr_destroy(&attr);

    return 0;
}

/**
 * create instances of derived DSP_board class depending on
 * bType
 *
 * @param buff  payload of REPLY_ACTIVE_BOARDS packet
 */
void Boards_ctrl::factory_board(uint8_t * buff) {

    uint8_t bId     = buff[4];
    uint8_t bType   = buff[3];

    if ( (*this)[bId] != NULL ) {
        //DPRINTF("Board %d already recorded\n", bId);
        return;
    }

    switch (bType) {
    case MC_BOARD :
        _mcs[bId] = new McBoard(buff);
        _boards[bId] = _mcs[bId];
        break;
    case FT_BOARD :
        _fts[bId] = new FtBoard(buff);
        _boards[bId] = _fts[bId];
        break;
    default:
        // unknown boards
        assert(0);
    }

    bcScanMask.set(bId-1);

}

/**
 * stop Boards_ctrl::rx_udp(void *_) thread
 *
 */
void Boards_ctrl::stop_rx_udp(){

    pthread_cancel(rx_upd_thread);
    pthread_join(rx_upd_thread, NULL);
    DPRINTF("stop rx_udp\n");

}

/**
 * static thread routine receive all the UDP traffic
 *
 * call Boards_ctrl::factory_board() and disptch broadcast data
 * received from boards
 *
 * @param _
 *
 * @return void* 0
 */
void * Boards_ctrl::rx_udp(void *_) {

    Boards_ctrl     * kls = (Boards_ctrl*)_;

    socklen_t           fromLength;
    struct sockaddr_in  fromAddr;
    int                 size;
    uint8_t             buff[RX_UDP_BUFSIZE];


#ifdef __XENO__
    pthread_set_mode_np(0, PTHREAD_WARNSW);
    pthread_set_name_np(pthread_self(), "rx_udp");
#endif

    for (;;) {

        // I do not use UDPCommPacket because it needs to be modified to receive generic bc data
        // in this thread we JUST receive udp pkt, we just need to verify the checksum ...

        //size = recvfrom(kls->udp_sock, &buff, sizeof(buff), 0, (struct sockaddr *)&fromAddr, &fromLength);
        size = recvfrom(kls->udp_sock, &buff, sizeof(buff), 0, 0, 0);

        if (size < 0) {
            //DPRINTF("udp recvfrom() %s\n", strerror(errno) );
            continue;
        }
#if 0
        DPRINTF("recv %d bytes\n", size);
        for (int i=0; i<size; i++) {
            DPRINTF("0x%02X ", buff[i]);
        }
        DPRINTF("\n");
#endif

        // verify checksum
        // TODO

        // switch on command byte
        switch (buff[2]) {

        case 0x82 :
            // REPLY_ACTIVE_BOARDS ... same packet for ALL DSP boards !!!
            // create board instance that handle tcp commands
            // check buffer overflow and board copies ?!?
            //DPRINTF("ID %d\n", buff[4]);
            kls->factory_board(buff);
            break;

        case 0xBB :
            // BCAST_DATA_PACKETS MotorController
        case 0xBC :
            // BCAST_DATA_PACKETS ForceTorqueSens
        case 0xBD :
            // BCAST_DATA_PACKETS MultiAxis ?!?

            // process bc_data at Boards_ctrl level
            kls->on_bc_data(buff);
            break;

        default:
            break;
        }
    }

    return 0;
}



void Boards_ctrl::on_bc_data(uint8_t *bc_packet) {

    Dsp_Board   * pBoard = NULL;
    uint8_t bId = bc_packet[BOARD_ID_BYTE_POS];

    /////////////////////////////////////////////////////////////////

    //pBoard = get_board(bId);
    pBoard = (*this)[bId];

    if ( pBoard ) {
        // process bc_data at single board level
        pBoard->on_bc_data(bc_packet);
    } else {
        // board is broadcasting but I DO NOT register it ....
        // raise error .... ?!
        ;
    }

    /////////////////////////////////////////////////////////////////
    try {
        if ( bcMask.test(bId-1) ) {
            // --- we assume all boards TX at same bc_freq ---
            // oversampling .... or at least some boards are in late ?!?
            //DPRINTF("#*#*#*  %d\t#*#*#*\t%s\n", bId, bcMask.to_string().c_str());
        }

        //assert( !(bcMask & (1 << bId)) );
        bcMask.set(bId-1);
#if 0
        DPRINTF("scan   \t%s\n", bcScanMask.to_string().c_str());
        DPRINTF("set %d \t%s\n\n", bId, bcMask.to_string().c_str());
#endif
        if ( bcMask.to_ulong() == bcScanMask.to_ulong() ) {
            // got bc_data from all scanned board
            // could trig here to get all boards sample ....
            //DPRINTF("****** \t******\n");
            pthread_mutex_lock(&data_sync_mutex);
            pthread_cond_signal(&data_sync_cond);
            pthread_mutex_unlock(&data_sync_mutex);

            bcMask.reset();
        }
    } catch (std::out_of_range &e) {
        //
    }

}

void Boards_ctrl::get_sync_data(void * dst) {

    pthread_mutex_lock(&data_sync_mutex);
    pthread_cond_wait(&data_sync_cond, &data_sync_mutex);
    //
    pthread_mutex_unlock(&data_sync_mutex);

}

/**
 * get a bc_data snapshot from dsp boards
 *
 */

void Boards_ctrl::get_bc_data(ts_bc_data_t * ts_bc_data) {

    for (dsp_map_t::iterator it = _boards.begin(); it != _boards.end(); it++) {
        it->second->get_bc_data(ts_bc_data[it->second->bId-1]);
    }

}


// ----------------------------------------------
// Boards protocol stuff
// ----------------------------------------------

/**
 * send a GET_ACTIVE_BOARDS broadcast UDP packet .
 *
 * each board reply with a REPLY_ACTIVE_BOARDS UDP packet, the
 * payload contain information about board type, board ID and IP
 * address.
 *
 * the method Boards_ctrl::factory_board(uint8_t * buff) use
 * this information the create proper DSP_board instance
 *
 * @retval number of active boards;
 *
 **/
int Boards_ctrl::scan4active(void) {

    //int wait4scan = 5;
    int last_active = 0;
    UDPCommPacket pkt(GET_ACTIVE_BOARDS);

    assert( ! pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr)));
    //while( pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr))) {}

    //while ( getActiveNum() < expected_num_boards && wait4scan-- ) {
    do {
        last_active = getActiveNum();
        // wait ... if in RT do a context switch ... I regret nothing
        sleep(1);

    } while ( last_active < getActiveNum() );

    //assert( getActiveNum() == expected_num_boards );
    if ( getActiveNum() != expected_num_boards ) {
        DPRINTF("****** WARN : expected %d boards got %d*****\n", expected_num_boards, getActiveNum());
    }

    return getActiveNum();

}

/**
 * configure each DSP board calling Dsp_Board::configure(const
 * YAML::Node&)
 *
 */
void Boards_ctrl::configure_boards(void) {

    for (dsp_map_t::iterator it = _boards.begin(); it != _boards.end(); it++) {
        it->second->configure(doc);
        it->second->print_me();

    }
}

/**
 * start/stop broadcast of all scanned board
 * for each board set bc_rate via TCP packet
 *
 * @param start_stop    true/false;
 *
 */
void Boards_ctrl::start_stop_bc_boards(uint8_t start_stop) {

    for (dsp_map_t::iterator it = _boards.begin(); it != _boards.end(); it++) {
        it->second->start_stop_bc(start_stop);
    }
}

/**
 * clear Mc-boards faults
 * send a CLEAR_BOARD_FAULT TCP packet
 *
 */
void Boards_ctrl::clear_mcs_faults(void) {

    for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
        it->second->setItem(CLEAR_BOARD_FAULT, 0, 0);
    }
}

/**
 * start/stop control of all scanned board
 * send one broadcast UDP packet .
 *
 * @param start_stop    true/false;
 * @param ctrl_type     default POSITION_MOVE, VELOCITY_MOVE,
 *                      TORQUE_MOVE;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::start_stop_control(uint8_t start_stop, uint8_t ctrl_type) {

    uint8_t cmd[MAX_DSP_BOARDS];
    UDPCommPacket pkt(ctrl_type);

    memset((void*)cmd, 0, sizeof(cmd));
    uint8_t start_stop_value = start_stop ? 0x03 : 0x01;

    for (dsp_map_t::iterator it = _boards.begin(); it != _boards.end(); it++) {
        cmd[it->second->bId-1] = start_stop_value;
    }

    pkt.appendData(cmd, sizeof(cmd));

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * start/stop control of a single scanned board
 * send one broadcast UDP packet .
 *
 * @param bId           board ID;
 * @param start_stop    true/false;
 * @param ctrl_type     default POSITION_MOVE, VELOCITY_MOVE,
 *                      TORQUE_MOVE;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::start_stop_single_control(uint8_t bId, uint8_t start, uint8_t ctrl_type) {

    uint8_t cmd[MAX_DSP_BOARDS];
    UDPCommPacket pkt(ctrl_type);

    memset((void*)cmd, 0, sizeof(cmd));
    uint8_t start_stop_value = start ? 0x03 : 0x01;

    dsp_map_t::iterator it = _boards.find(bId);
    if (it != _boards.end()) {
        cmd[it->second->bId-1] = start_stop_value;
    } else {
        ;//DPRINTF("****** ERROR START STOP BOARD %d******\n", bId);
    }

    pkt.appendData(cmd, sizeof(cmd));

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * start/stop control of a set of scanned boards send one
 * broadcast UDP packet .
 *
 * @param boards_set    board ID vector;
 * @param start_stop    true/false;
 * @param ctrl_type     default POSITION_MOVE, VELOCITY_MOVE,
 *                      TORQUE_MOVE;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::start_stop_set_control(std::vector<int> boards_set, uint8_t start_stop, uint8_t ctrl_type) {

    int res;
    for (int j=0; j<boards_set.size(); j++) {
        res |= start_stop_single_control((uint8_t)boards_set[j], start_stop, ctrl_type);
    }
    return (res?-1:0);
}

int Boards_ctrl::test(void) {

    for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
        it->second->test_setting();
    }
}

/**
 * send a SET_DESIRED_POSITION broadcast UDP packet .
 *
 * @param des_pos   array of reference positions;
 * @param nbytes    size in bytes of array;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_position(int *des_pos, int nbytes) {

    UDPCommPacket pkt(SET_DESIRED_POSITION);
    pkt.appendData((uint8_t*)des_pos, nbytes);

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * send a SET_DESIRED_VELOCITY broadcast UDP packet .
 *
 * @param des_vel   array of reference velocities;
 * @param nbytes    size in bytes of array;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_velocity(short *des_vel, int nbytes) {

    UDPCommPacket pkt(SET_DESIRED_VELOCITY);
    pkt.appendData((uint8_t*)des_vel, nbytes);

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * send a SET_DESIRED_TORQUE broadcast UDP packet .
 *
 * @param des_tor   array of reference torques;
 * @param nbytes    size in bytes of array;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_torque(short *des_tor, int nbytes) {

    UDPCommPacket pkt(SET_DESIRED_TORQUE);
    pkt.appendData((uint8_t*)des_tor, nbytes);

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * send a SET_DESIRED_POS_VEL broadcast UDP packet .
 *
 * @param des_pos   array of reference positions;
 * @param des_vel   array of reference velocities;
 * @param nElem     number of reference in each array, nElem =
 *                  len(des_pos) and len(des_pos) ==
 *                  len(des_vel);
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_position_velocity(int *des_pos, short *des_vel, int nElem)
{
    UDPCommPacket pkt(SET_DESIRED_POS_VEL);
    for (int i = 0; i < nElem ; i++) {
        pkt.appendData((uint8_t*)&des_pos[i], sizeof(int));
        pkt.appendData((uint8_t*)&des_vel[i], sizeof(short));
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));
}

/**
 * send a SET_GRAVITY_COMPENSATION broadcast UDP packet .
 *
 * @param des_gc   array of reference;
 * @param nbytes   size in bytes of array;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_gravity_compensation(int *des_gc, int nbytes) {

    UDPCommPacket pkt(SET_GRAVITY_COMPENSATION);
    pkt.appendData((uint8_t*)des_gc, nbytes);

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * send a SET_STIFFNESS_DAMPING broadcast UDP packet .
 *
 * @param des_stiff     array of reference positions;
 * @param des_damp      array of reference velocities;
 * @param nElem         number of reference in each array,
 *                  nElem = len(des_stiff) and len(des_stiff) ==
 *                  len(des_damp);
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_stiffness_damping(int *des_stiff, int *des_damp, int nElem) {

    UDPCommPacket pkt(SET_STIFFNESS_DAMPING);
    for (int i = 0; i < nElem ; i++) {
        pkt.appendData((uint8_t*)&des_stiff[i], 3);
        pkt.appendData((uint8_t*)&des_damp[i],  3);
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

/**
 * send a SET_PID_OFFSET broadcast UDP packet .
 *
 * @param des_pid_offset   array of reference;
 * @param nbytes    size in bytes of array;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_pid_offset(short *des_pid_offset, int nbytes) {

    UDPCommPacket pkt(SET_PID_OFFSET);
    pkt.appendData((uint8_t*)des_pid_offset, nbytes);

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));

}

Boards_ctrl::mcs_map_t Boards_ctrl::get_mcs_map()
{
    return _mcs;
}

Boards_ctrl::fts_map_t Boards_ctrl::get_fts_map()
{
    return _fts;
}


/**
 * send a SET_DESIRED_POSITION_GROUP broadcast UDP packet .
 *
 * @param pos_group   map< bId, pos > of reference positions;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_position_group(const uint8_t * bIds, const int * des_pos, const int nElem) {

    group_ref_t pos_group;
    for (int i = 0; i < nElem ; i++) {
        pos_group[*bIds++] = *des_pos++; 
    }

    return set_position_group(pos_group);
}

/**
 * send a SET_DESIRED_POSITION_GROUP broadcast UDP packet .
 *
 * @param 
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_position_group(const group_ref_t &pos_group) {

    UDPCommPacket pkt(SET_DESIRED_POSITION_GROUP);
    for (group_ref_t::const_iterator it = pos_group.begin(); it != pos_group.end(); it++) {
        pkt.appendData((uint8_t*)&it->first, sizeof(uint8_t));
        pkt.appendData((uint8_t*)&it->second, sizeof(int));
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));
}



/**
 * send a SET_DESIRED_VELOCITY_GROUP broadcast UDP packet .
 *
 * @param vel_group   map< bId, vel > of velocity references;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_velocity_group(const uint8_t * bIds, const short * des_vel, const int nElem) {

    group_ref_t vel_group;
    for (int i = 0; i < nElem ; i++) {
        vel_group[*bIds++] = *des_vel++; 
    }

    return set_velocity_group(vel_group);
}

/**
 * send a SET_DESIRED_VELOCITY_GROUP broadcast UDP packet .
 *
 * @param 
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_velocity_group(const group_ref_t &vel_group) {

    UDPCommPacket pkt(SET_DESIRED_VELOCITY_GROUP);
    for (group_ref_t::const_iterator it = vel_group.begin(); it != vel_group.end(); it++) {
        pkt.appendData((uint8_t*)&it->first, sizeof(uint8_t));
        pkt.appendData((uint8_t*)&it->second, sizeof(short));
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));
}



/**
 * send a SET_DESIRED_TORQUE_GROUP broadcast UDP packet .
 *
 * @param 
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_torque_group(const uint8_t * bIds, const short * des_tor, const int nElem) {

    group_ref_t tor_group;
    for (int i = 0; i < nElem ; i++) {
        tor_group[*bIds++] = *des_tor++; 
    }

    return set_torque_group(tor_group);
}

/**
 * send a SET_DESIRED_TORQUE_GROUP broadcast UDP packet .
 *
 * @param tor_group   map< bId, tor > of reference;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_torque_group(const group_ref_t &tor_group) {

    UDPCommPacket pkt(SET_DESIRED_TORQUE_GROUP);
    for (group_ref_t::const_iterator it = tor_group.begin(); it != tor_group.end(); it++) {
        pkt.appendData((uint8_t*)&it->first, sizeof(uint8_t));
        pkt.appendData((uint8_t*)&it->second, sizeof(short));
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));
}



/**
 * send a SET_DESIRED_POS_VEL_GROUP broadcast UDP packet .
 *
 * @param 
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_position_velocity_group(const uint8_t *bIds, const int * des_pos, const short * des_vel, const int nElem) {

    group_ref_comp_t pos_vel_group;
    for (int i = 0; i < nElem ; i++) {
        pos_vel_group[*bIds++] = std::make_pair(*des_pos++, *des_vel++); 
    }

    return set_position_velocity_group(pos_vel_group);
}

/**
 * send a SET_DESIRED_POS_VEL_GROUP broadcast UDP packet .
 *
 * @param pos_vel_group   map< bId, pair<pos,vel> > of 
 *                        reference;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_position_velocity_group(const group_ref_comp_t &pos_vel_group) {

    UDPCommPacket pkt(SET_DESIRED_POS_VEL_GROUP);
    for (group_ref_comp_t::const_iterator it = pos_vel_group.begin(); it != pos_vel_group.end(); it++) {
        pkt.appendData((uint8_t*)&it->first, sizeof(uint8_t));
        pkt.appendData((uint8_t*)&it->second.first, sizeof(int));
        pkt.appendData((uint8_t*)&it->second.second, sizeof(short));
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));
}



/**
 * send a SET_STIFFNESS_DAMPING_GROUP broadcast UDP packet .
 *
 * @param 
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_stiffness_damping_group(const uint8_t *bIds, const int * des_stiff, const int * des_damp, const int nElem) {

    group_ref_comp_t stiff_damp_group;
    for (int i = 0; i < nElem ; i++) {
        stiff_damp_group[*bIds++] = std::make_pair(*des_stiff++, *des_damp++); 
    }

    return set_stiffness_damping_group(stiff_damp_group);
}

/**
 * send a SET_STIFFNESS_DAMPING_GROUP broadcast UDP packet .
 *
 * @param stiff_damp_group   map< bId, pair<stiff,damp> > of 
 *                           reference;
 *
 * @retval 0 on success;
 * @retval -1 on fail;
 *
 */
int Boards_ctrl::set_stiffness_damping_group(const group_ref_comp_t &stiff_damp_group) {

    UDPCommPacket pkt(SET_STIFFNESS_DAMPING_GROUP);
    for (group_ref_comp_t::const_iterator it = stiff_damp_group.begin(); it != stiff_damp_group.end(); it++) {
        pkt.appendData((uint8_t*)&it->first, sizeof(uint8_t));
        pkt.appendData((uint8_t*)&it->second.first, 3);
        pkt.appendData((uint8_t*)&it->second.second,3);
    }

    return pkt.sendToUDPSocket(udp_sock, (sockaddr *)&dest_addr, sizeof(dest_addr));
}

