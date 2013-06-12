/*
   Boards_ctrl_ext.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_CTRL_EXT_H__
#define __BOARDS_CTRL_EXT_H__

#include <utils.h>
#include <Boards_iface.h>


/**
 * @class Boards_ctrl_ext
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_ext class
 */

class Boards_ctrl_ext : public Boards_ctrl {

public:
    Boards_ctrl_ext(const char * config);
    virtual ~Boards_ctrl_ext();

    virtual void init(void);
    virtual void homing(void);
    virtual void sense(void);
    virtual void move(void);
    virtual int user_input(void *buffer, ssize_t buff_size);

protected:

    //std::string pipes_name[2] = { "boards_ctrl", "boards_bc_data" };
    std::string pipes_name[2];
    int         fd_info, fd_data;

    int     _home[MAX_DSP_BOARDS];
    int     _pos[MAX_DSP_BOARDS];
    short   _vel[MAX_DSP_BOARDS];
    short   _tor[MAX_DSP_BOARDS];
    short   _mVolt[MAX_DSP_BOARDS];
    int     _stiff[MAX_DSP_BOARDS];
    int     _damp[MAX_DSP_BOARDS];

    ts_bc_data_t   _ts_bc_data[MAX_DSP_BOARDS];

};


#endif
