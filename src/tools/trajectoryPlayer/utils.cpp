/* 
 * Copyright (C)2013  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Thread.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

#include "utils.h"

// ******************** ACTION CLASS

int action_struct::get_n_joints()
{ 
    return N_JOINTS;
}

action_struct::action_struct(int n)
{
    N_JOINTS = n;
    q_joints = new double [N_JOINTS];
    for (int i=0; i<N_JOINTS; i++) q_joints[i]=0.0;
    tag = "UNKNOWN";
}

action_struct::action_struct(const action_struct& as)
{
    N_JOINTS = as.N_JOINTS;
    counter = as.counter;
    time = as.time;
    tag = as.tag;
    q_joints = new double [N_JOINTS];
    for (int i=0; i< N_JOINTS; i++)
    {
        q_joints[i]=as.q_joints[i];
    }
}

action_struct & action_struct::operator=(const action_struct & as)
{
    if(this == &as)
        return *this;
    N_JOINTS = as.N_JOINTS;
    counter = as.counter;
    time = as.time;
    tag = as.tag;
    q_joints = new double [N_JOINTS];
    for (int i=0; i< N_JOINTS; i++)
    {
        q_joints[i]=as.q_joints[i];
    }
    return *this;
}

action_struct::~action_struct()
{
    if (q_joints)
    { 
        delete []q_joints;
        q_joints = 0;
    }
}

void action_class::clear()
{
    forever = false;
    current_action = 0;
    current_status = ACTION_IDLE;
    action_vector.clear();
}

action_class::action_class()
{
    clear();
}

void action_class::print()
{
    for (action_it=action_vector.begin(); action_it<action_vector.end(); action_it++)
    {
        yInfo ("%d %f ",action_it->counter,action_it->time);
        for (int i=0; i< action_it->get_n_joints(); i++)
            yInfo("%f ", action_it->q_joints[i]);
        yInfo ("\n");
    }
}

bool action_class::openFile(string filename, int n_joints)
{
    bool ret = true;
    FILE* data_file = 0;
    data_file = fopen(filename.c_str(),"r");
    if (data_file!=NULL)
    {
        char* bb = 0;
        int   line =0;
        do
        {
            char command_line[1024];
            bb = fgets (command_line, 1024, data_file);
            if (bb == 0) break;
            //if(!parseCommandLine(command_line, line++))
            //  if(!parseCommandLineFixTime(command_line, line++, 1.0/25.0)) //25hz
            if(!parseCommandLineFixTime(command_line, line++, 1.0/50.0, n_joints)) //50hz
                {
                    yError ("error parsing file, line %d\n", line);
                    ret = false;
                    break;
                };
        }
        while (1);

        fclose (data_file);
    }
    else 
    {
        yError("unable to find file\n");
        ret = false;
    }
    return ret;
}

bool action_class::parseCommandLineFixTime(const char* command_line, int line, double fixTime, int n_joints)
{
    static int count = 0;
    static double time =0.0;
    action_struct tmp_action(n_joints);

    tmp_action.counter = count; count = count + 1;
    tmp_action.time    = time;  time = time+fixTime;
        
    char* tok;
    size_t ss = strlen(command_line);
    char* cline = new char [ss];
    strcpy(cline,command_line);
    tok = strtok (cline," ");
    int i=0;
    int joints_count = 0;
    while (tok != 0 && joints_count < n_joints)
    {
        sscanf(tok, "%lf", &tmp_action.q_joints[i++]);
        tok = strtok (NULL," ");
        joints_count++;
    }

    //insertion in the vector based on the timestamp
    /*for (action_it=action_vector.begin(); action_it<action_vector.end(); action_it++)
    {
        if (tmp_action.time > action_it->time) break;
    }
    action_vector.insert(action_it,tmp_action);*/

    if (action_vector.size()==0) 
    {
        action_vector.push_back(tmp_action);
        return true;
    }

    for (action_it=action_vector.end()-1; action_it>=action_vector.begin(); action_it--)
    {
        if (tmp_action.time > action_it->time) break;
    }
    action_vector.insert(action_it+1,tmp_action);
    return true;
}

bool action_class::parseCommandLine(const char* command_line, int line, int n_joints)
{
    action_struct tmp_action(n_joints);
    //use strtok for runtime-defined number of entries
    char command_line_format [1000];
    sprintf(command_line_format, "%%d %%lf    ");
    for (int i = 0; i < n_joints; i++)
    {
        size_t off = strlen(command_line_format);
        sprintf(command_line_format+off, "%%lf ");
    }

    int ret = sscanf(command_line, "%d %lf    %lf %lf %lf %lf %lf %lf", 
    &tmp_action.counter,
    &tmp_action.time,
            
    &tmp_action.q_joints[0],
    &tmp_action.q_joints[1],
    &tmp_action.q_joints[2],
    &tmp_action.q_joints[3],
    &tmp_action.q_joints[4],
    &tmp_action.q_joints[5]
    );

    if (ret == n_joints+2) 
    {
        //insertion in the vector based on the timestamp
        for (action_it=action_vector.begin(); action_it<action_vector.end(); action_it++)
            if (tmp_action.time > action_it->time) break;
        action_vector.insert(action_it,tmp_action);
        return true;
    }
            
    return false;
}


// ******************** ROBOT DRIVER CLASS
robotDriver::robotDriver()
{
    drvOptions_ll.clear();
    drv_ll  = 0;
    ipos_ll = 0;
    iposdir_ll = 0;
    ipid_ll = 0;
    icmd_ll = 0;
    ienc_ll = 0;
    n_joints = 0;

    verbose=1;
    drv_connected=false;
}

bool robotDriver::configure(const Property &copt)
{
    bool ret=true;
    Property &options=const_cast<Property &> (copt);

    drvOptions_ll.put("device","remote_controlboard");

    if (!options.check("robot")) { yError() << "Missing robot parameter"; return false; }
    if (!options.check("part")) { yError() << "Missing part parameter"; return false; }

    string remote_ll;
    string local_ll;
    remote_ll = string("/") + string(options.find("robot").asString()) + string("/") +string(options.find("part").asString());
    local_ll  = string("/") + string("trajectoryPlayer") + string("/") + string(options.find("part").asString());
    drvOptions_ll.put("remote",remote_ll.c_str());
    drvOptions_ll.put("local",local_ll.c_str());

    if (verbose)
    {
        yDebug() << "driver options:\n" << drvOptions_ll.toString().c_str();
    }

    return ret;
}

bool robotDriver::init()
{
    drv_ll=new PolyDriver(drvOptions_ll);

    if (drv_ll->isValid())
        drv_connected = drv_ll->view(ipos_ll) && drv_ll->view(iposdir_ll) && drv_ll->view(ienc_ll) && drv_ll->view(ipid_ll) && drv_ll->view(imotenc_ll) && drv_ll->view(icmd_ll);
    else
        drv_connected=false;

    if (!drv_connected)
    {  
        if (drv_ll)
        {
            delete drv_ll;
            drv_ll=0;
        }
        return false;
    }

    //get the number of the joints
    ienc_ll->getAxes(&n_joints);

    //set the initial reference speeds
    double* speeds = new double [n_joints];
    for (int i=0; i<n_joints; i++) speeds[i] = 20.0;
    ipos_ll->setTrajSpeeds(speeds);

    //dbg_connected is optional, so it has not to be returned
    return drv_connected;
}

robotDriver::~robotDriver()
{
    if (drv_ll)
    {
        delete drv_ll;
        drv_ll=0;
    }
}

bool robotDriver::setControlMode(const int j, const int mode)
{
    if (!icmd_ll) return false;
    return icmd_ll->setControlMode(joints_map[j], mode);
}

bool robotDriver::setPosition(int j, double ref)
{
    if (!iposdir_ll) return false;
    return iposdir_ll->setPosition(joints_map[j], ref);
}

bool robotDriver::getEncoder(int j, double *v)
{
    if (!ienc_ll) return false;
    return ienc_ll->getEncoder(joints_map[j], v);
}

bool robotDriver::positionMove(int j, double v)
{
    if (!ipos_ll) return false;
    return ipos_ll->positionMove(joints_map[j], v);
}