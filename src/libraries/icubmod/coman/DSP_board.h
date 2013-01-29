/*
	DSP_board.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)
    
*/

#ifndef __DSP_BOARDS_H__
#define __DSP_BOARDS_H__

#include <netinet/in.h>
#include <arpa/inet.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/circular_buffer.hpp>

#include "yaml-cpp/yaml.h"

#include <definitions.h>

//------------------------------------------------

//------------------------------------------------
using namespace boost::accumulators;


#warning "was under #ifndef C_WRAPPER... what does it means??"
enum GainSet
{
    VELOCITY_GAINS = 0,
    POSITION_GAINS = 1,
    TORQUE_GAINS   = 2
};

struct __pid_gains_t {
    GainSet    gain_set;
    int     p;
    int     i;
    int     d;
}  __attribute__((__packed__));
typedef __pid_gains_t pid_gains_t;

/**
 * @class Dsp_Board
 * @defgroup Dsp_Board Dsp board
 * @ingroup RoboLLI
 * @brief Dsp_Board base class to interface with DSP boards, 
 *        each single instance handle TCP/IP communication with
 *        the relative board and process its broadcast data
 *        dispatched by Boards_ctrl class
 */

class Dsp_Board {

public:
    Dsp_Board(uint8_t  *);
    virtual ~Dsp_Board();

    int setItem(int reqCmd, void *src, int srcBytes);
    int getItem(int reqCmd, void *src, int nSrcBytes, int resCmd, void *dst, int nDstBytes);

    virtual void configure(const YAML::Node&) = 0;
    virtual void get_bc_data(void *) = 0;
    virtual void print_me(void);

    void start_stop_bc(uint8_t);

    virtual void on_bc_data(uint8_t *);

    void measure_bc_freq(void);
    void print_stat(void);

    uint8_t     stopped;
    uint8_t     bId;
    uint8_t     bType;

protected:

    char        ip_addr[16];
    //
    int         sock_fd;
    sockaddr_in sock_addr;
    //
    uint16_t    policy;
    uint16_t    extra_policy;
    uint8_t     bc_rate;
    //
    uint64_t _rx_bc_prec;
    accumulator_set<uint64_t, 
        features<
            tag::count
            ,tag::mean
            ,tag::min
            ,tag::max
            //,tag::variance(lazy)
            //,tag::error_of<tag::mean>
        >
    > bc_freq;

    pthread_mutex_t dsp_mutex;

    boost::circular_buffer<log_t> dsp_log;
    void dump_log(void);

};
     

/**
 * @class McBoard
 * @defgroup McBoard Motor controller board
 * @ingroup Dsp_Board
 * @brief McBoard class derived from Dsp_Board, implement 
 *        derived virtual method to carry out motor controller
 *        protocol
 */
class McBoard : public Dsp_Board{

public:
    McBoard(uint8_t *_):Dsp_Board(_) { }
    virtual ~McBoard() { dump_log(); }

    virtual void configure(const YAML::Node&);
    virtual void print_me(void);
    virtual void on_bc_data(uint8_t *);
    virtual void get_bc_data(void *);

    void test_setting(void);

private:

    mc_bc_data_t bc_data;
    void do_log(uint8_t *);

    int         _min_pos, _max_pos;
    short int   _min_vel, _max_vel;
    short int   _max_tor;
    short int   _des_pos, _des_vel, _des_tor;
    pid_gains_t V_pid, P_pid, T_pid;

    torque_factor_t _torque_factor;
    short int   _motor_config;
};


/**
 * @class FtBoard
 * @defgroup FtBoard Force torque sensor board
 * @ingroup Dsp_Board
 * @ingroup Dsp_Board
 * @brief FtBoard class derived from Dsp_Board, implement 
 *        derived virtual method to carry out force torque
 *        sensor protocol
 */
class FtBoard : public Dsp_Board {
public:
    FtBoard(uint8_t *_):Dsp_Board(_) { }
    virtual ~FtBoard() { dump_log(); }

    virtual void configure(const YAML::Node&);
    virtual void print_me(void);
    virtual void on_bc_data(uint8_t *);
    virtual void get_bc_data(void *);

private:

    ft_bc_data_t bc_data;

    void do_log(uint8_t *);


};

#endif
