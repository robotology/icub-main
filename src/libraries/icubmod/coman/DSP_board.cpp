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

#include <CommProtocol.hpp>

#ifdef RT_ENV
    #include <rtdk.h>
    #include <rtnet.h>
    #define DPRINTF rt_printf
#else
    #define DPRINTF printf
#endif

/*extern*/ unsigned long long g_tStart;

static unsigned long long get_time_ns(void)
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
        req.appendData((char*)src, srcBytes);
    }

    // Send request
    if (req.sendToTCPSocket(sock_fd)) {
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
        req.appendData((char*)src, srcBytes);
    }

    // Send request
    if (!req.sendToTCPSocket(sock_fd)) {
        // Receive response
        if (!rep.recvFromTCPSocket(sock_fd))
            rep.readData((char *)dst, dstBytes);
        else return 1;
    } else return 2;

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

    TCPCommPacket   bc_rate_pkt(SET_BCAST_RATE);
    uint8_t         bc_rate_cmd[] = {bc_rate, !!start_stop};

    bc_rate_pkt.appendData((char *)bc_rate_cmd, sizeof(bc_rate_cmd));
    if (bc_rate_pkt.sendToTCPSocket(sock_fd))
        perror("Fail send bcast rate");

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
    DPRINTF("\t bcast : avg %llu ns ", (uint64_t)mean(bc_freq));
    DPRINTF("rx %llu\n", count(bc_freq));
}

void Dsp_Board::on_bc_data(uint8_t *buff) {

    if ( !stopped ) {
        measure_bc_freq();
    }

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

    for (boost::circular_buffer<log_t>::iterator it=dsp_log.begin(); it!=dsp_log.end(); it++) {
        log_file << boost::format("%1%\t") % (*it).ts;
        bc_data_board_type = (*it).bc_data.bc_header._command;
        switch ( bc_data_board_type ) {
            case BCAST_MC_DATA_PACKETS :
                (*it).bc_data.mc_bc_data.sprint(buffer);
                break;
            case BCAST_FT_DATA_PACKETS :
                (*it).bc_data.ft_bc_data.sprint(buffer);
                break;
            default:
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
#ifdef C_WRAPPER
    typedef char GainSet;
    #define	VELOCITY_GAINS 0
    #define	POSITION_GAINS 1
    #define	TORQUE_GAINS   2
#else
    enum GainSet
    {
    	VELOCITY_GAINS = 0,
    	POSITION_GAINS = 1,
    	TORQUE_GAINS   = 2
    };
#endif

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
        ;//DPRINTF("%s NOT FOUND... using only mc_board node\n", oss.str().c_str());
    }

    // at least set bc_policy and bc_freq
    assert(mc_board_node);

    // yaml operator ">>" with uint8_t ?!@?#!@
    unsigned short tmp;
    (*mc_board_node)["bc_rate"] >> tmp;
    bc_rate = tmp;
    (*mc_board_node)["policy"] >> policy;
    (*mc_board_node)["extra_policy"] >> extra_policy;

    setItem(SET_BCAST_POLICY, &policy, 2);
    setItem(SET_EXTRA_BCAST_POLICY, &extra_policy, 2);
    //
    tmp = 0;
    setItem(SET_TORQUE_ON_OFF, &tmp, 1);

    getItem(GET_MIN_POSITION,   NULL, 0, REPLY_MIN_POSITION, &_min_pos, sizeof(_min_pos));
    getItem(GET_MAX_POSITION,   NULL, 0, REPLY_MAX_POSITION, &_max_pos, sizeof(_max_pos));

    getItem(GET_MIN_VELOCITY,   NULL, 0, REPLY_MIN_VELOCITY, &_min_vel, sizeof(_min_vel));
    getItem(GET_MAX_VELOCITY,   NULL, 0, REPLY_MAX_VELOCITY, &_max_vel, sizeof(_max_vel));

    getItem(GET_MAX_TORQUE,     NULL, 0, REPLY_MAX_TORQUE,   &_max_tor, sizeof(_max_tor));

    getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &_des_pos, sizeof(_des_pos));
    getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &_des_vel, sizeof(_des_vel));
    getItem(GET_DESIRED_TORQUE,     NULL, 0, REPLY_DESIRED_TORQUE,   &_des_tor, sizeof(_des_tor));


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

        node = board_node->FindValue("impedance_control");
        if (node) {
            uint16_t motor_config_mask = 0;
            //
            tmp = 1;  //1 for torque control    0 for position control
            setItem(SET_TORQUE_ON_OFF, &tmp, 1);

            (*node)["motor_config_mask"] >> motor_config_mask;
            getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &tmp, 2);
            DPRINTF("get %d MOTOR_CONFIG 0x%04X\n", bId, tmp);
            tmp |= motor_config_mask;
            setItem(SET_MOTOR_CONFIG, &tmp, 2);
            DPRINTF("set %d MOTOR_CONFIG 0x%04X\n", bId, tmp);

            tmp = 0x1; //0 Moving Average 1 ButterWorth 2 Least Square 3 Jerry Pratt
            setItem(SET_MOTOR_CONFIG2, &tmp, 2);
        }

    }

    // read back PIDs anyway
    V_pid.gain_set = VELOCITY_GAINS;
    getItem(GET_PID_GAINS,      &V_pid.gain_set, 1, REPLY_PID_GAINS, &V_pid, sizeof(V_pid));
    P_pid.gain_set = POSITION_GAINS;
    getItem(GET_PID_GAINS,      &P_pid.gain_set, 1, REPLY_PID_GAINS, &P_pid, sizeof(P_pid));
    T_pid.gain_set = TORQUE_GAINS;
    getItem(GET_PID_GAINS,      &T_pid.gain_set, 1, REPLY_PID_GAINS, &T_pid, sizeof(T_pid));

    getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &_motor_config, 2);

    getItem(GET_TORQUE_FACTORS, NULL, 0, REPLY_TORQUE_FACTORS, &_torque_factor, sizeof(_torque_factor));

    unsigned char filter_setup[3]; 
    //0 Torque sample 
    filter_setup[0] = 0;
    filter_setup[1] = 1;
    filter_setup[2] = 0;
    setItem(SET_FILTER_SAMPLES, &filter_setup, 3);
    getItem(GET_FILTER_SAMPLES, &filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &filter_setup, 3);
    //DPRINTF("%d %d %d\n", filter_setup[0], filter_setup[1], filter_setup[2]);
     
    // 1 motor velocity sample
#if 0 // !!! ASK PHIL
    filter_setup[0] = 1;
    filter_setup[1] = 4;
    filter_setup[2] = 0;
    setItem(SET_FILTER_SAMPLES, &filter_setup, 3);
    getItem(GET_FILTER_SAMPLES, &filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &filter_setup, 3);
    //DPRINTF("%d %d %d\n", filter_setup[0], filter_setup[1], filter_setup[2]);
#endif
    // 2 Link velocity sample
    filter_setup[0] = 2;
    filter_setup[1] = 10;
    filter_setup[2] = 0;
    setItem(SET_FILTER_SAMPLES, &filter_setup, 3);
    getItem(GET_FILTER_SAMPLES, &filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &filter_setup, 3);
    //DPRINTF("%d %d %d\n", filter_setup[0], filter_setup[1], filter_setup[2]);

    // 3 voltage sample
    filter_setup[0] = 3;
    filter_setup[1] = 1;
    filter_setup[2] = 0;
    setItem(SET_FILTER_SAMPLES, &filter_setup, 3);
    getItem(GET_FILTER_SAMPLES, &filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &filter_setup, 3);
    //DPRINTF("%d %d %d\n", filter_setup[0], filter_setup[1], filter_setup[2]);

    // 4 Current sample
    filter_setup[0] = 4;
    filter_setup[1] = 1;
    filter_setup[2] = 0;
    setItem(SET_FILTER_SAMPLES, &filter_setup, 3);
    getItem(GET_FILTER_SAMPLES, &filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &filter_setup, 3);
    //DPRINTF("%d %d %d\n", filter_setup[0], filter_setup[1], filter_setup[2]);


}

void McBoard::test_setting(void) {

    getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &_des_pos, sizeof(_des_pos));
    getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &_des_vel, sizeof(_des_vel));
    getItem(GET_DESIRED_TORQUE,     NULL, 0, REPLY_DESIRED_TORQUE,   &_des_tor, sizeof(_des_tor));
    DPRINTF("\tDes[%d] {pos:%d vel:%d tor:%d} \n",bId,_des_pos,_des_vel,_des_tor);
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
    DPRINTF("\tmotor config 0x%04X\n",_motor_config);
    DPRINTF("\tDes {pos:%d vel:%d tor:%d} \n",_des_pos,_des_vel,_des_tor);

}

void McBoard::on_bc_data(uint8_t *buff) {

    Dsp_Board::on_bc_data(buff);

    pthread_mutex_lock(&dsp_mutex);
    memcpy((void*)&bc_data, buff, sizeof(bc_data));
    pthread_mutex_unlock(&dsp_mutex);

    if ( !stopped ) {
        do_log(buff);
    }
}

void McBoard::get_bc_data(void * dest_buff) {

    pthread_mutex_lock(&dsp_mutex);
    memcpy(dest_buff, &bc_data, sizeof(bc_data));
    pthread_mutex_unlock(&dsp_mutex);
}


void McBoard::do_log(uint8_t *buff) {

    log_t log_elem;
    log_elem.ts = get_time_ns() - g_tStart;
    memcpy((void*)&log_elem.bc_data, buff, sizeof(mc_bc_data_t));
    dsp_log.push_back(log_elem);
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

void FtBoard::on_bc_data(uint8_t *buff) {

    Dsp_Board::on_bc_data(buff);

    pthread_mutex_lock(&dsp_mutex);
    memcpy((void*)&bc_data, buff, sizeof(bc_data));
    pthread_mutex_unlock(&dsp_mutex);

    if ( !stopped ) {
        do_log(buff);
    }
}

void FtBoard::get_bc_data(void * dest_buff) {

    pthread_mutex_lock(&dsp_mutex);
    memcpy(dest_buff, &bc_data, sizeof(bc_data));
    pthread_mutex_unlock(&dsp_mutex);
}

void FtBoard::do_log(uint8_t *buff) {

    log_t log_elem;
    log_elem.ts = get_time_ns() - g_tStart;
    memcpy((void*)&log_elem.bc_data, buff, sizeof(ft_bc_data_t));
    dsp_log.push_back(log_elem);

}

