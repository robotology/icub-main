/*
   Boards_ctrl_ext.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifdef __XENO__
    #include <rt_ipc.h>
#endif
#include <Boards_ctrl_ext.h>

Boards_ctrl_ext::Boards_ctrl_ext(const char * config): Boards_ctrl(config) {

    std::string pipe;
    pipes_name[0] = std::string("boards_ctrl");
    pipes_name[1] = std::string("boards_bc_data");
#ifdef __XENO__
    fd_info = xddp_bind(pipes_name[0].c_str());
    fd_data = xddp_bind(pipes_name[1].c_str());
#else
    pipe = pipe_prefix + pipes_name[0];
    mkfifo(pipe.c_str(), S_IRWXU);
    fd_info = open(pipe.c_str(), O_RDWR | O_NONBLOCK);

    pipe = pipe_prefix + pipes_name[1];
    mkfifo(pipe.c_str(), S_IRWXU);
    fd_data = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
#endif
    assert(fd_info && fd_data);

}


Boards_ctrl_ext::~Boards_ctrl_ext() {

    struct timespec ts = {3,0};
    clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);

    DPRINTF("Stop DSPs\n");
    start_stop_control(false);
    stop_rx_udp();
    start_stop_bc_boards(false);

    close(fd_info);
    close(fd_data);

#ifndef __XENO__
    std::string pipe = pipe_prefix + pipes_name[0];
    unlink(pipe.c_str());
    pipe = pipe_prefix + pipes_name[1];
    unlink(pipe.c_str());
#endif
}


void Boards_ctrl_ext::init() {

    int dummy, bId = 0;

    Boards_ctrl::init();

    DPRINTF("Scan for active boards ....\n");
    int numActive = scan4active();
    DPRINTF("Found %d boards\n", numActive);

    // stop motor controllers [udp]
    start_stop_control(false);
    // try to stop broadcast if still actice [tcp]
    start_stop_bc_boards(false);
    // use config.yaml settings to set boards parameters [tcp]
    configure_boards();
    // tell to ALL dps to start broadcast data [tcp]
    start_stop_bc_boards(true);
    // warm up rx_udp thread ...
    sleep(1);

}

void Boards_ctrl_ext::homing(void) {

    int         bId = 0;

    get_bc_data(_ts_bc_data);

    for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        _pos[bId-1] = _ts_bc_data[bId-1].raw_bc_data.mc_bc_data.Position;
        _home[bId-1] = _pos[bId-1];
        _vel[bId-1] = DEG2RAD(25)*1000;
        _tor[bId-1] = 0;
        _mVolt[bId-1] = 0;
    }

    move();
}


void Boards_ctrl_ext::sense(void) {

    //get_sync_data((void*)0);
    get_bc_data(_ts_bc_data);

    // idx --> bId-1
    //(*boards_ctrl)[0]->
    //_ts_bc_data[0].mc_bc_data.Position;

    // write to pipe or cross domain socket ALL bc data
    ssize_t nbytes = write(fd_data, (void*)&_ts_bc_data, sizeof(_ts_bc_data));
    if (nbytes < 0) {
        //perror(">> write to xddp/pipe fail ");
    }
}

int Boards_ctrl_ext::user_input(void *buffer, ssize_t buff_size) {

    int     nbytes;

    /////////////////////////////////////////////////////////
    // NON-BLOCKING read char from pipe or cross domain socket
#if __XENO__
    nbytes = recvfrom(fd_info, buffer, buff_size, MSG_DONTWAIT, NULL, 0);
#else
    // NON-BLOCKING
    nbytes = read(fd_info, buffer, buff_size);
#endif

    return nbytes;
}


void Boards_ctrl_ext::move(void) {

    set_position(_pos, sizeof(_pos));
    set_velocity(_vel, sizeof(_vel));
    set_torque(_tor, sizeof(_tor));

}

