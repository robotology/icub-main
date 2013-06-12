/*
	DSP_board.cpp

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/

#include <DSP_board.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <fstream>
#include <boost/format.hpp>

#include <utils.h>
// #include <signals.h>
#include <CommProtocol.hpp>


#ifdef __XENO__
    #include <rtdk.h>
    #include <rtnet.h>
    #define DPRINTF rt_printf
#else
    #define DPRINTF printf
#endif

static unsigned long long __get_time_ns(void)
{
    unsigned long long time_ns;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    return time_ns;
}


Dsp_Board::Dsp_Board(uint8_t  *replyScan4Active) {

    bType   = replyScan4Active[3];
    bId     = replyScan4Active[4];
    memset(ip_addr, 0, 16);
    sprintf(ip_addr, "%d.%d.%d.%d", replyScan4Active[8], replyScan4Active[7], replyScan4Active[6], replyScan4Active[5]);

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);

    // set socket timeout
#ifdef __XENO__
    // This socket control option is used to specify the time-out on the socket before it returns.
    // It is used typically to wait for data on a Read.
    // The time-out specifies the amount of time the function will wait for data before it returns.
    int64_t timeout_ns = 250000000;
    if (ioctl(sock_fd, RTNET_RTIOC_TIMEOUT, &timeout_ns) < 0)
        DPRINTF("ioctl RTNET_RTIOC_TIMEOUT failed\n");
#else
    struct timeval timeout;
    timeout.tv_sec =  0;
    timeout.tv_usec = 250000;
    if (setsockopt (sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                sizeof(timeout)) < 0)
        DPRINTF("setsockopt SO_RCVTIMEO failed\n");

    if (setsockopt (sock_fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                sizeof(timeout)) < 0)
        DPRINTF("setsockopt SO_SNDTIMEO failed\n");
#endif

    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = inet_addr(ip_addr);
    sock_addr.sin_port = htons(23);

    if ( connect(sock_fd, (sockaddr *)&sock_addr, sizeof(sockaddr_in)) < 0 ) {
        perror("connect");
        assert(0);
    }

    pthread_mutex_init(&dsp_mutex, NULL);

    dsp_log.set_capacity(LOG_SIZE);
    _rx_bc_prec = 0;
}


Dsp_Board::~Dsp_Board() {

    print_stat();
    dump_log();
    close(sock_fd);
    pthread_mutex_destroy(&dsp_mutex);
}

/**
 * send a set command
 *
 *
 * @param reqCmd
 * @param src
 * @param srcBytes
 *
 * @return int
 */
int Dsp_Board::setItem(int reqCmd, void *src, int srcBytes)
{
    TCPCommPacket req(reqCmd);

    if (srcBytes > 0 && src != NULL) {
        req.appendData((uint8_t*)src, srcBytes);
    }

    // Send request
    if (req.sendToTCPSocket(sock_fd)) {
        DPRINTF("[TCP]{%d} Fail sendTo\n", bId);
        return 2;
    }

    return 0;
}

/**
 * send a request command and wait for reply
 *
 * @param reqCmd
 * @param src
 * @param srcBytes
 * @param resCmd
 * @param dst
 * @param dstBytes
 *
 * @return int
 */
int Dsp_Board::getItem(int reqCmd, void *src, int srcBytes,
                       int resCmd, void *dst, int dstBytes)
{
    TCPCommPacket req(reqCmd), rep(resCmd);

    if (srcBytes > 0 && src != NULL) {
        req.appendData((uint8_t*)src, srcBytes);
    }

    // Send request
    if (!req.sendToTCPSocket(sock_fd)) {
        // Receive response
        if (!rep.recvFromTCPSocket(sock_fd))
            rep.readData((uint8_t *)dst, dstBytes);
        else {
            DPRINTF("[TCP]{%d} Fail recvFrom reply 0x%02X\n", bId, cmdsInfo[resCmd].cmdId);
            return 1;
        }
    } else {
        DPRINTF("[TCP]{%d} Fail sendTo\n", bId);
        return 2;
    }

    return 0;
}

/**
 * start/stop broadcast, set bc_rate via TCP packet
 *
 * @param start_stop    true/false;
 *
 */
void Dsp_Board::start_stop_bc(uint8_t start_stop) {

    // start_stop = true --> start bc
    // start_stop = false --> stop bc
    struct timespec ts;
    int try_count = 0;
    TCPCommPacket   bc_rate_pkt(SET_BCAST_RATE);
    uint8_t         bc_rate_cmd[] = {bc_rate, !!start_stop};

    bc_rate_pkt.appendData(bc_rate_cmd, sizeof(bc_rate_cmd));
    while (bc_rate_pkt.sendToTCPSocket(sock_fd)) {
        DPRINTF("[TCP]{%d} Fail send bcast rate to start/stop bc\n", bId);
        ts.tv_sec = 0;
        ts.tv_nsec = 100*1e6; // 100 ms
        clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        if (++try_count > 10) {
            break;
        }
    }

    if (start_stop) {
        dsp_log.clear();
        _bc_tStart = get_time_ns();
    }
    stopped = !start_stop;
}

void Dsp_Board::print_me(void) {

    DPRINTF("ID %d type %d addr %s\n",
              bId,
              bType,
              ip_addr);
}

void Dsp_Board::measure_bc_freq(void)
{
    uint64_t bc_loop , tNow = get_time_ns();

    if (_rx_bc_prec > 0) {
        bc_loop = tNow - _rx_bc_prec;
        bc_freq(bc_loop);
    }
    _rx_bc_prec = tNow;

}

void Dsp_Board::print_stat(void) {

    DPRINTF("ID %d %s\n", bId, ip_addr);
    DPRINTF("\t bcast freq us : min %.3f max %.3f avg %.3f ", min(bc_freq)/1e3, max(bc_freq)/1e3, mean(bc_freq)/1e3);
    DPRINTF("rx %lu\n", count(bc_freq));
}


void Dsp_Board::on_bc_data(uint8_t *raw_bc_buff) {

    if ( !stopped ) {
        measure_bc_freq();
    }

    pthread_mutex_lock(&dsp_mutex);
    ts_bc_data.ts_rx = get_time_ns() - _bc_tStart;
    memcpy((void*)&ts_bc_data.raw_bc_data, raw_bc_buff, get_bc_data_size());
    pthread_mutex_unlock(&dsp_mutex);

    if ( !stopped ) {
        dsp_log.push_back(ts_bc_data);
    }
}

void Dsp_Board::get_bc_data(ts_bc_data_t &dest_data) {

    pthread_mutex_lock(&dsp_mutex);
    memcpy(&dest_data, &ts_bc_data, sizeof(ts_bc_data_t));
    pthread_mutex_unlock(&dsp_mutex);
}



/**
 * write log file in /tmp/log_bId_<...>.txt, see LOG_SIZE define
 * for circular buffer dimension
 *
 */
void Dsp_Board::dump_log(void) {

    char buffer[1024];
    unsigned char bc_data_board_type = 0;

    std::string filename = str(boost::format("/tmp/log_bId_%1%.txt") % (int)bId);
    std::ofstream log_file(filename.c_str());

    for (boost::circular_buffer<ts_bc_data_t>::iterator it=dsp_log.begin(); it!=dsp_log.end(); it++) {
    //while ( ! dsp_log.empty() ) {
        log_file << boost::format("%1%\t") % (*it).ts_rx;
        bc_data_board_type = (*it).raw_bc_data.bc_header._command;
        switch ( bc_data_board_type ) {
            case BCAST_MC_DATA_PACKETS :
                (*it).raw_bc_data.mc_bc_data.sprint(buffer, sizeof(buffer));
                break;
            case BCAST_FT_DATA_PACKETS :
                (*it).raw_bc_data.ft_bc_data.sprint(buffer, sizeof(buffer));
                break;
            default:
                DPRINTF("dump_log unknown bc_data\n");
                sprintf(buffer, "%s\n", __FILE__);
                break;
        }
        log_file << std::string(buffer);
    }
    log_file << std::flush;
    log_file.close();
}


///////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////

/**
 * @enum GainSet
 * @ingroup RoboMotorController
 * Set of gains of different controller types.
 */

// _AC_  come è usato??? nel .h c'è un char. la define è attiva o meno?
// viene usato un enum confrontato con un char??
// #ifdef C_WRAPPER
//     typedef char GainSet;
//     #define	VELOCITY_GAINS 0
//     #define	POSITION_GAINS 1
//     #define	TORQUE_GAINS   2
// #else
//     enum GainSet
//     {
//     	VELOCITY_GAINS = 0,
//     	POSITION_GAINS = 1,
//     	TORQUE_GAINS   = 2
//     };
// #endif

/**
 * configure DSP board and get some parameter using TCP/IP
 * packets via Dsp_Board::setItem() and Dsp_Board::getItem()
 *
 * @param doc    YAML::Node
 */
void McBoard::configure(const YAML::Node &doc) {

    const YAML::Node * mc_board_node = doc.FindValue("mc_board");
    const YAML::Node * board_node;

    // look for board_<id> in config
    std::string board("board_");
    std::ostringstream oss;
    oss << board << (int)bId;
    board_node = doc.FindValue(oss.str());
    if (! board_node) {
        DPRINTF("%s NOT FOUND... using only mc_board node\n", oss.str().c_str());
    }

    // at least set bc_policy and bc_freq
    assert(mc_board_node);

    unsigned short tmp;
    // yaml operator ">>" with uint8_t ?!@?#!@
    (*mc_board_node)["bc_rate"] >> tmp;
    bc_rate = tmp;
    (*mc_board_node)["policy"] >> policy;
    (*mc_board_node)["extra_policy"] >> extra_policy;

    ts_bc_data.raw_bc_data.mc_bc_data.check_policy(policy, extra_policy);

    setItem(CLEAR_BOARD_FAULT, 0, 0);

    setItem(SET_BCAST_POLICY, &policy, 2);
    setItem(SET_EXTRA_BCAST_POLICY, &extra_policy, 2);
    
    //
    uint8_t torque_on_off = 0;
    setItem(SET_TORQUE_ON_OFF, &torque_on_off, 1);

    getItem(GET_MIN_POSITION,   NULL, 0, REPLY_MIN_POSITION, &_min_pos, sizeof(_min_pos));
    getItem(GET_MAX_POSITION,   NULL, 0, REPLY_MAX_POSITION, &_max_pos, sizeof(_max_pos));

    printf("Max position %d\n", _max_pos);
    printf("Min position %d\n", _min_pos);

    getItem(GET_MIN_VELOCITY,   NULL, 0, REPLY_MIN_VELOCITY, &_min_vel, sizeof(_min_vel));
    getItem(GET_MAX_VELOCITY,   NULL, 0, REPLY_MAX_VELOCITY, &_max_vel, sizeof(_max_vel));

    printf("Max velocity %d\n", _max_vel);
    printf("Min velocity %d\n", _min_vel);

    getItem(GET_MAX_TORQUE,     NULL, 0, REPLY_MAX_TORQUE,   &_max_tor, sizeof(_max_tor));

    printf("Max torque %d\n", _max_tor);

    getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &_des_pos, sizeof(_des_pos));
    getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &_des_vel, sizeof(_des_vel));
    getItem(GET_DESIRED_TORQUE,     NULL, 0, REPLY_DESIRED_TORQUE,   &_des_tor, sizeof(_des_tor));

    printf("Des position %d\n", _des_pos);
    printf("Des velocity %d\n", _des_vel);
    printf("Des Torque   %d\n", _des_tor);

    // set specific board attribute
    if ( board_node ) {

        std::vector<int> pid(3);
        pid_gains_t      p_i_d;
        const YAML::Node *node = board_node->FindValue("pid");
        if (node) {
            for(YAML::Iterator it=node->begin(); it!=node->end(); ++it) {
                std::string key;
                it.first() >> key;
                if ( key == "position" ) {
                    p_i_d.gain_set = POSITION_GAINS;
                } else if ( key == "velocity" ) {
                    p_i_d.gain_set = VELOCITY_GAINS;
                } else if ( key == "torque" ) {
                    p_i_d.gain_set = TORQUE_GAINS;
                } else {
                    continue;
                }
                it.second() >> pid;
                p_i_d.p = pid[0];
                p_i_d.i = pid[1];
                p_i_d.d = pid[2];
                setItem(SET_PID_GAINS,      &p_i_d.gain_set, sizeof(p_i_d));
            }
        }

        node = board_node->FindValue("current_lim_mA");
        if (node) {
            (*board_node)["current_lim_mA"] >> _current_lim;
            setItem(SET_CURRENT_LIMIT, &_current_lim, 2);            
        }

        node = board_node->FindValue("max_torque_mNm");
        if (node) {
            (*board_node)["max_torque_mNm"] >> _max_tor;
            setItem(SET_MAX_TORQUE, &_max_tor, 2);
        }

        getItem(GET_CURRENT_LIMIT,     NULL, 0, REPLY_CURRENT_LIMIT,   &_current_lim, sizeof(_current_lim));

        node = board_node->FindValue("impedance_control");
        if (node) {
            uint16_t motor_config_mask = 0;
            uint16_t motor_config2_mask = 0;
            //
            tmp = 1;  //1 for torque control    0 for position control
            setItem(SET_TORQUE_ON_OFF, &tmp, 1);

            (*node)["motor_config_mask"] >> motor_config_mask;
            getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &tmp, 2);
            DPRINTF("get %d MOTOR_CONFIG 0x%04X\n", bId, tmp);
            tmp |= motor_config_mask;
            setItem(SET_MOTOR_CONFIG, &tmp, 2);
            DPRINTF("set %d MOTOR_CONFIG 0x%04X\n", bId, tmp);

            (*node)["motor_config2_mask"] >> motor_config2_mask;
            getItem(GET_MOTOR_CONFIG2, NULL, 0, REPLY_MOTOR_CONFIG2, &tmp, 2);
            DPRINTF("get %d MOTOR_CONFIG2 0x%04X\n", bId, tmp);
            tmp |= motor_config2_mask;
            setItem(SET_MOTOR_CONFIG2, &tmp, 2);
            DPRINTF("set %d MOTOR_CONFIG2 0x%04X\n", bId, tmp);

            //tmp = 0x1; //0 Moving Average 1 ButterWorth 2 Least Square 3 Jerry Pratt
            //setItem(SET_MOTOR_CONFIG2, &tmp, 2);
        }

        node = board_node->FindValue("filter_samples");
        if (node) {
            DPRINTF("set filter samples\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

            // 0 Torque sample
            _filter_setup[0] = 0;
            _filter_setup[1] = 50;
            _filter_setup[2] = 0;
            setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
            getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
            //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

            // 1 motor velocity sample
            _filter_setup[0] = 1;
            _filter_setup[1] = 50; //4
            _filter_setup[2] = 0;
            setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
            getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
            //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

            // 2 Link velocity sample
            _filter_setup[0] = 2;
            _filter_setup[1] = 10;
            _filter_setup[2] = 0;
            setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
            getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
            //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

            // 3 voltage sample
            _filter_setup[0] = 3;
            _filter_setup[1] = 1;
            _filter_setup[2] = 0;
            setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
            getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
            //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

            // 4 Current sample
            _filter_setup[0] = 4;
            _filter_setup[1] = 1;
            _filter_setup[2] = 0;
            setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
            getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
            //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

        }

    }
    int ret;
    // read back PIDs anyway
    V_pid.gain_set = VELOCITY_GAINS;
    getItem(GET_PID_GAINS,      &V_pid.gain_set, 1, REPLY_PID_GAINS, &V_pid, sizeof(V_pid));
    printf("Vel PID Kp %d Ki %d Kd %d (ret %d)\n", V_pid.p, V_pid.i, V_pid.d, ret);

    printf("\ngoing to ask os pid!!\n");
//    sleep(10);
    P_pid.gain_set = POSITION_GAINS;
    getItem(GET_PID_GAINS,      &P_pid.gain_set, 1, REPLY_PID_GAINS, &P_pid, sizeof(P_pid));
    printf("Pos PID Kp %d Ki %d Kd %d (ret %d)\n", P_pid.p, P_pid.i, P_pid.d, ret);

//    sleep(10);

    T_pid.gain_set = TORQUE_GAINS;
    getItem(GET_PID_GAINS,      &T_pid.gain_set, 1, REPLY_PID_GAINS, &T_pid, sizeof(T_pid));
    printf("Tor PID Kp %d Ki %d Kd %d (ret %d)\n", T_pid.p, T_pid.i, T_pid.d, ret);

    getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &_motor_config, sizeof(_motor_config));
    getItem(GET_MOTOR_CONFIG2, NULL, 0, REPLY_MOTOR_CONFIG2, &_motor_config2, sizeof(_motor_config2));

    getItem(GET_TORQUE_FACTORS, NULL, 0, REPLY_TORQUE_FACTORS,  &_torque_factor,    sizeof(_torque_factor));
    getItem(GET_CURRENT_LIMIT,  NULL, 0, REPLY_CURRENT_LIMIT,   &_current_lim,      sizeof(_current_lim));
    getItem(GET_MAX_TORQUE,     NULL, 0, REPLY_MAX_TORQUE,      &_max_tor,          sizeof(_max_tor));



}

void McBoard::set_PID(int gain_set, int p, int i, int d) {

    pid_gains_t      p_i_d;

    switch (gain_set) {
        case POSITION_GAINS:
            p_i_d = P_pid;
            break;
        case TORQUE_GAINS:
            p_i_d = T_pid;
            break;
        case VELOCITY_GAINS:
            p_i_d = V_pid;
            break;
        default:
            assert(0);
    }

    p_i_d.p = p;
    p_i_d.i = i;
    p_i_d.d = d;

    setItem(SET_PID_GAINS,      &p_i_d.gain_set, sizeof(p_i_d));

    switch (gain_set) {
        case POSITION_GAINS:
            getItem(GET_PID_GAINS,      &P_pid.gain_set, 1, REPLY_PID_GAINS, &P_pid, sizeof(P_pid));
            DPRINTF("\tPID gains %d : %d %d %d\n",P_pid.gain_set, P_pid.p, P_pid.i, P_pid.d);
            break;
        case TORQUE_GAINS:
            getItem(GET_PID_GAINS,      &T_pid.gain_set, 1, REPLY_PID_GAINS, &T_pid, sizeof(T_pid));
            DPRINTF("\tPID gains %d : %d %d %d\n",T_pid.gain_set, T_pid.p, T_pid.i, T_pid.d);
            break;
        case VELOCITY_GAINS:
            getItem(GET_PID_GAINS,      &V_pid.gain_set, 1, REPLY_PID_GAINS, &V_pid, sizeof(V_pid));
            DPRINTF("\tPID gains %d : %d %d %d\n",V_pid.gain_set, V_pid.p, V_pid.i, V_pid.d);
            break;
        default:
            break;
    }

}

void McBoard::set_PID_increment(int gain_set, int p_incr, int i_incr, int d_incr) {

    pid_gains_t      p_i_d;

    switch (gain_set) {
        case POSITION_GAINS:
            p_i_d = P_pid;
            break;
        case TORQUE_GAINS:
            p_i_d = T_pid;
            break;
        case VELOCITY_GAINS:
            p_i_d = V_pid;
            break;
        default:
            assert(0);
    }

    p_i_d.p += p_incr;
    p_i_d.i += i_incr;
    p_i_d.d += d_incr;

    set_PID(p_i_d.gain_set, p_i_d.p, p_i_d.i, p_i_d.d);

}

void McBoard::get_PID(int gain_set, int &p, int &i, int &d) {

    pid_gains_t      p_i_d;

    switch (gain_set) {
        case POSITION_GAINS:
            p_i_d = P_pid;
            break;
        case TORQUE_GAINS:
            p_i_d = T_pid;
            break;
        case VELOCITY_GAINS:
            p_i_d = V_pid;
            break;
        default:
            assert(0);
    }

    p = p_i_d.p;
    i = p_i_d.i;
    d = p_i_d.d;

    return;
}

void McBoard::test_setting(void) {

    getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &_des_pos, sizeof(_des_pos));
    getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &_des_vel, sizeof(_des_vel));
    getItem(GET_DESIRED_TORQUE,     NULL, 0, REPLY_DESIRED_TORQUE,   &_des_tor, sizeof(_des_tor));
    DPRINTF("Des[%d] {pos:%d vel:%d tor:%d} \n",bId,_des_pos,_des_vel,_des_tor);
}


void McBoard::print_me(void) {

    Dsp_Board::print_me();
    DPRINTF("\tbc_policy 0x%04X\n",policy);
    DPRINTF("\tbc_extra_policy 0x%04X\n",extra_policy);
    DPRINTF("\tbc_freq %.1f ms\n", (float)bc_rate/2);
    DPRINTF("\t%s [%d:%d] mRAD\n","Position",_min_pos,_max_pos);
    DPRINTF("\t%s [%d:%d] mRAD/s\n","Velocity",_min_vel,_max_vel);
    DPRINTF("\tTorque %d Nm\n",_max_tor);
    DPRINTF("\tPID gains %d : %d %d %d\n",V_pid.gain_set, V_pid.p, V_pid.i, V_pid.d);
    DPRINTF("\tPID gains %d : %d %d %d\n",P_pid.gain_set, P_pid.p, P_pid.i, P_pid.d);
    DPRINTF("\tPID gains %d : %d %d %d\n",T_pid.gain_set, T_pid.p, T_pid.i, T_pid.d);
    DPRINTF("\ttorque factors %d %d\n",_torque_factor.multiplier, _torque_factor.offset);
    DPRINTF("\tcurrent limit  %d mA\n",_current_lim);
    DPRINTF("\tmotor config  0x%04X\n",_motor_config);
    DPRINTF("\tmotor config2 0x%04X\n",_motor_config2);
    DPRINTF("\tDes {pos:%d vel:%d tor:%d} \n",_des_pos,_des_vel,_des_tor);

}


///////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////

/**
 * configure DSP board and get some parameter using TCP/IP
 * packets via Dsp_Board::setItem() and Dsp_Board::getItem()
 *
 * @param doc    YAML::Node
 */
void FtBoard::configure(const YAML::Node &doc) {


    const YAML::Node * ft_board_node = doc.FindValue("ft_board");
    bool calibrate_offset;

    // at least set bc_policy and bc_freq
    assert(ft_board_node);

    // yaml operator ">>" with uint8_t ?!@?#!@
    unsigned short tmp;
    (*ft_board_node)["bc_rate"] >> tmp;
    bc_rate = tmp;
    (*ft_board_node)["policy"] >> policy;

    ts_bc_data.raw_bc_data.ft_bc_data.check_policy(policy, 0);

    setItem(SET_BCAST_POLICY, &policy, 2);
    try {
        (*ft_board_node)["calibrate_offset"] >> calibrate_offset;
        if ( calibrate_offset ) {
            setItem(CALIBRATE_OFFSETS, NULL, 0);
            DPRINTF("FtSensor calibrate offsets\n");
        }
    } catch (YAML::Exception &e) {
            DPRINTF("%s\n", e.what());
    }
}

void FtBoard::print_me(void) {

    Dsp_Board::print_me();
    DPRINTF("\tbc_policy 0x%04X\n",policy);
    DPRINTF("\tbc_freq %.1f ms\n", (float)bc_rate/2);

}


