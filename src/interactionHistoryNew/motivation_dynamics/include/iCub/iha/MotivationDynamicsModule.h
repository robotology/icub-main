// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __MOTIVATIONDYNAMICSMODULE__
#define __MOTIVATIONDYNAMICSMODULE__

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class MotivationDynamicsModule;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

/**
 *
 * MotivationDynamicsModule class
 *
 * \brief See \ref icub_iha_Dynamics
 */
class iCub::contrib::MotivationDynamicsModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::BufferedPort<yarp::os::Bottle> dataPort; // for reading data from the SMI
    yarp::os::BufferedPort<yarp::os::Bottle> memPort; // for score from short term memory
    yarp::os::Port outPort; //for writing the data with reward added
    yarp::os::Port expressionRawPort; // for writing facial expressions


    // parameters read from ini file/command line
    int face_response_attack;      // time over which response will rise to maximum
    int face_response_level;       // time for which response stays at maximum
    int face_response_decay;       // time over which response will decay to nearly zero
    double sound_catch_threshold;  // ignore quite sounds
    int face_lost_count;           // debounce - face has to be absent for this long for it to be lost
    double reward_contrib_face;    // contribution of face to reward
    double reward_contrib_sound;   // contribution of sound to reward
    double reward_contrib_gaze;   // contribution of gaze to reward
    double reward_contrib_drum;   // contribution of drum to reward
    double reward_contrib_hide;   // contribution of hide to reward

    // -- for emotion actions
    bool reward_display;
    int current_eout;
    int new_eout;

    int action_ehi;
    int action_elo;
    int action_emid;
    double th_ehi;
    double th_elo;

    //indices used to access sensor data
    int ts_offset; //the first data element is the current timestep
    int num_encoders;
    int face_index;
    int sound_index;
    int gaze_index;  
    int beat_index;  
    int action_index;                
    int reward_index;
    int insert_index;

    double *last_enc; //pointer for array to store previous timestep encoder values

    // operational variables lasting more than a single update
    int resetcount;   // counting lost face timesteps
    int acc;          // accumulating consecutive timesteps seeing face

    double faceReward;
    double gazeReward;
    double gazeAlpha;
    void sendExpression(int expr);


public:

    MotivationDynamicsModule();
    virtual ~MotivationDynamicsModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
