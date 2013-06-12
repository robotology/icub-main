/*
   Boards_iface.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2012-, alessio.margan@iit.it)
   
*/

/**
 * @defgroup RoboLLI RoboLLI
 *
 * @brief Real-time library for Low-Level Interaction with cCub motor controllers and F/T sensors.
 *
 * %RoboLLI is a real-time library that allows a GNU/Linux/Xenomai/RTnet PC to interact
 * with cCub motor controllers and F/T sensors by means of an Ethernet cable.
 * All you need on your PC is an RTnet-compliant network adapter, i.e. supported for hard real-time communication:
 * for a list of all supported adapters, please see <a href="http://rtnet.org/"> RTnet homepage </a>.
 * Each joint is controlled by means of a DSP board.
 *
 * %RoboLLI handles two kinds of connections:
 * @li UDP, established to send/receive broadcast data to/from all active joints/sensors (i.e. all powered on joints/sensors);
 * @li TCP, established with each active joint/sensor.
 *  
 * %The protocol is implemented by Boards_ctrl class, DSP_board
 * class and its derived classes, McBoard and FtBoard 
 * @li instance of Boards_ctrl class handle UDP braodcast 
 *     communication and record reference of "ALIVE" boards that
 *     at start up time reply to the controller
 * @li instance og DSP_Board class handle TCP communication with
 *     the associated joint/sensor
  
 * %RoboLLI library should be used according to the following steps:
 * @li instantiate the Boards_ctrl class;
 * @li call Boards_ctrl::init() method; 
 * @li call Boards_ctrl::scan4active() method; 
 * @li call Boards_ctrl::configure_boards(); 
 * @li set initial position and velocity for each active McBoard 
 *     inatances;
 * @li call Boards_ctrl::start_stop_control() true to start the 
 *     dsp controller;
 * @li call Boards_ctrl::start_stop_bc_boards() true to start 
 *     broadcast data transmission from boards;
 * @li feed DSPs controller by sending position/velocity/torque
 *     references calling :
 *     - Boards_ctrl::set_position();
 *     - Boards_ctrl::set_velocity();
 *     - Boards_ctrl::set_torque();
 * @li on shutdown call :
 *     - Boards_ctrl::start_stop_control() false to stop
 *       controller;
 *     - Boards_ctrl::stop_rx_udp();
 *     - Boards_ctrl::start_stop_bc_boards() false to stop
 *       broadcast data transmission;
 *
 * Before starting your own applications making use of %RoboLLI library, you must load all the necessary Xenomai modules
 * and configure RTnet network which PC and remote boards belong to .
 *
 * @author Alessio Margan (2012-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_IFACE_H__
#define __BOARDS_IFACE_H__

#include <netinet/in.h>
#include <arpa/inet.h>
#include <bitset>
#include <pthread.h>
#include <map>

#include <CommProtocol.hpp>

#include "yaml-cpp/yaml.h"

#include <definitions.h>
#include <DSP_board.h>


/**
 * @class Boards_ctrl
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl class
 */

class Boards_ctrl {

    /** @ingroup Boards_controller
     *  @brief maps of boards <bId, instance ref>
     *  @{
     */
public:
    typedef std::map<int, Dsp_Board*>   dsp_map_t;
    typedef std::map<int, McBoard*>     mcs_map_t;
    typedef std::map<int, FtBoard*>     fts_map_t;
    /** @}
     */

public:
    Boards_ctrl(const char * config);
    ~Boards_ctrl();

    Dsp_Board * get_board(uint8_t bId) { return(_boards.find(bId) != _boards.end()) ? _boards[bId] : NULL;}
    Dsp_Board * operator[](int bId) { return(_boards.find(bId) != _boards.end()) ? _boards[bId] : NULL;} 

    void stop_rx_udp();

    void get_sync_data(void *);
    void get_bc_data(ts_bc_data_t *);

    int init(void);
    int test(void);

    int getActiveNum(void) { return bcScanMask.count();}

    void configure_boards(void);
    void start_stop_bc_boards(uint8_t);
    void clear_mcs_faults(void);

    /** @defgroup UDP_cmd udp commands
     *  @ingroup Boards_controller
     *  @brief List of UDP commands exchanged with all the active
     *  boards, by broadcasting them.
     *  @{
     */
    int scan4active(void);
    int start_stop_control(uint8_t start, uint8_t ctrl_type = POSITION_MOVE);
    int start_stop_single_control(uint8_t bId, uint8_t start, uint8_t ctrl_type = POSITION_MOVE);
    int start_stop_set_control(std::vector<int> boards_set, uint8_t start, uint8_t ctrl_type = POSITION_MOVE);

    int set_position(int *, int);
    int set_velocity(short *, int);
    int set_torque(short *, int);
    int set_position_velocity(int *, short *, int);
    int set_gravity_compensation(int *des_gc, int nbytes);
    int set_stiffness_damping(int *des_stiff, int *des_damp, int nElem);
    int set_pid_offset(short *, int);

    int set_position_group(const uint8_t *, const int *, const int nElem);
    int set_velocity_group(const uint8_t *, const short *, const int nElem);
    int set_torque_group(const uint8_t *, const short *, const int nElem);
    int set_position_velocity_group(const uint8_t *, const int *, const short *, const int nElem);
    int set_stiffness_damping_group(const uint8_t *, const int *, const int *, const int nElem);

    int set_position_group(const group_ref_t &);
    int set_velocity_group(const group_ref_t &);
    int set_torque_group(const group_ref_t &);
    int set_position_velocity_group(const group_ref_comp_t &);
    int set_stiffness_damping_group(const group_ref_comp_t &);

    /** @} */ // end of UDP_cmd


    // _AC_  added to get a reference to the internal maps of boards.
    mcs_map_t get_mcs_map();
    fts_map_t get_fts_map();

protected:

    dsp_map_t _boards;
    mcs_map_t _mcs;
    fts_map_t _fts;

    void factory_board(uint8_t *);
    // thread
    static void * rx_udp(void *);
    void on_bc_data(uint8_t *);

private:

    int         udp_sock;
    pthread_t   rx_upd_thread;
    int         expected_num_boards;

    struct sockaddr_in local_addr;
    struct sockaddr_in  dest_addr;

    YAML::Node doc;

    pthread_mutex_t data_sync_mutex;
    pthread_cond_t  data_sync_cond;

    std::bitset<32>   bcMask, bcScanMask;
};

#endif
