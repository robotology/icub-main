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
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <mutex>

#include "utils.h"

// ******************** THE THREAD
class BroadcastingThread: public PeriodicThread
{
private:
    action_class          *actions;
    robotDriver           *driver;
    BufferedPort<Bottle>  port_data_out;

public:
    BroadcastingThread(int period=1): PeriodicThread((double)period/1000.0)
    {
        port_data_out.open("/trajectoryPlayer/all_joints_data_out:o");
    }

    ~BroadcastingThread()
    {
        port_data_out.interrupt();
        port_data_out.close();
    }

    void attachRobotDriver(robotDriver *p)
    {
        if (p)  driver=p;
    }

    void attachActions(action_class *a)
    {
        if (a)  actions=a;
    }

    bool threadInit()
    {
        if (!driver)
            return false;
        return true;
    }

    void run()
    {
        //quick reads the current position
        double encs[50];
        if (driver && driver->ienc_ll)
        {
            driver->ienc_ll->getEncoders(encs);
        }
        else
        {
            //invalid driver
        }

        //quick reads the pid output
        double outs[50];
        if (driver && driver->ipid_ll)
        {
            driver->ipid_ll->getPidOutputs(VOCAB_PIDTYPE_POSITION,outs);
        }
        else
        {
            //invalid driver
        }

        //quick reads the pid error
        double errs[50];
        if (driver && driver->ipid_ll)
        {
            driver->ipid_ll->getPidErrors(VOCAB_PIDTYPE_POSITION,errs);
        }
        else
        {
            //invalid driver
        }

        //quick reads the optical encoders
        double opts[50];
        if (driver && driver->imotenc_ll)
        {
            driver->imotenc_ll->getMotorEncoders(opts);
        }
        else
        {
            //invalid driver
        }

        int j = actions->current_action;

        Bottle& bot2 = this->port_data_out.prepare();
        bot2.clear();
        bot2.addInt(actions->action_vector[j].counter);
        bot2.addDouble(actions->action_vector[j].time);

        int size = this->actions->action_vector[j].get_n_joints();
        double *ll = actions->action_vector[j].q_joints;

        bot2.addString("commands:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(ll[ix]);
        }
        bot2.addString("encoders:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(encs[ix]);
        }
        bot2.addString("outputs:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(outs[ix]);
        }
        bot2.addString("optical:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(opts[ix]);
        }
        bot2.addString("errors:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(errs[ix]);
        }
        bot2.addString("timestamp:");
        bot2.addDouble(yarp::os::Time::now());
        this->port_data_out.write();
    }
};

class WorkingThread: public PeriodicThread
{
private:

public:
    robotDriver           *driver;
    action_class          actions;
    mutex                 mtx;
    BufferedPort<Bottle>  port_command_out;
    BufferedPort<Bottle>  port_command_joints;
    bool                  enable_execute_joint_command;
    double                start_time;

    WorkingThread(double period=5): PeriodicThread((double)period/1000.0)
    {
        enable_execute_joint_command = true;
        //*** open the output port
        port_command_out.open("/trajectoryPlayer/port_command_out:o");
        port_command_joints.open("/trajectoryPlayer/port_joints:o");
    }

    ~WorkingThread()
    {
        port_command_out.interrupt();
        port_command_out.close();
        port_command_joints.interrupt();
        port_command_joints.close();
    }

    void attachRobotDriver(robotDriver *p)
    {
        if (p)  driver=p;
    }

    bool threadInit()
    {
        if (!driver)
            return false;
        return true;
    }
    
    bool execute_joint_command(int action_id)
    {
        if (!driver) return false;
        if (!enable_execute_joint_command) return true;

        double *ll = actions.action_vector[action_id].q_joints;
        int nj = actions.action_vector[action_id].get_n_joints();

        for (int j = 0; j < nj; j++)
        {
            driver->setPosition(j, ll[j]);
        }
        return true;
    }

    void compute_and_send_command(int action_id)
    {
        //prepare the output command
        Bottle& bot = port_command_out.prepare();
        bot.clear();
        bot.addInt(actions.action_vector[action_id].counter);
        bot.addDouble(actions.action_vector[action_id].time);
        bot.addString(actions.action_vector[action_id].tag.c_str());
        //@@@ you can add stuff here...

        //send the output command
        port_command_out.write();
        if (!execute_joint_command(action_id))
        {
            yError("failed to execute command");
        }

        //quick reads the current position
        double encs[50];
        if (driver)
        {
            int nj= actions.action_vector[action_id].get_n_joints();
            for (int j = 0; j < nj; j++)
            {
                driver->getEncoder(j, &encs[j]);
            }
        }
        else
        {
            //invalid driver
            yError("Critical error: invalid driver");
        }

        //send the joints angles on debug port
        double *ll = actions.action_vector[action_id].q_joints;
        Bottle& bot2 = this->port_command_joints.prepare();
        bot2.clear();
        bot2.addInt(actions.action_vector[action_id].counter);
        bot2.addDouble(actions.action_vector[action_id].time);
        int size = this->actions.action_vector[action_id].get_n_joints();
        bot2.addString("commands:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(ll[ix]);
        }
        bot2.addString("encoders:");
        for (int ix=0;ix<size;ix++)
        {
            bot2.addDouble(encs[ix]);
        }
        this->port_command_joints.write();
    }

    void run()
    {
        lock_guard<mutex> lck(mtx);
        double current_time = yarp::os::Time::now();
        if (actions.current_status==ACTION_IDLE)
        {
            // do nothing
        }
        else if (actions.current_status == ACTION_STOP)
        {
            int nj = actions.action_vector[0].get_n_joints();
            actions.current_status = ACTION_IDLE;
        }
        else if (actions.current_status == ACTION_RESET)
        {
            int nj = actions.action_vector[0].get_n_joints();
            for (int j = 0; j < nj; j++)
            {
                driver->setControlMode(j, VOCAB_CM_POSITION);
            }
            actions.current_status = ACTION_IDLE;
        }
        else if (actions.current_status == ACTION_RUNNING)
        {
            size_t last_action = actions.action_vector.size();
            if (last_action == 0)
            {
                yError("sequence empty!");
                actions.current_status=ACTION_RESET;
                return;
            }

            //if it's not the last action
            if (actions.current_action < last_action-1)
            {
                //if enough time is passed from the previous action
                //double duration = actions.action_vector[actions.current_action+1].time -
                //                  actions.action_vector[actions.current_action].time;
                double elapsed_time = actions.action_vector[actions.current_action].time;
                if (current_time-start_time > elapsed_time)
                {
                    //printf("%d %+6.6f \n", actions.current_action, current_time-start_time);
                    //last_time = current_time;
                    actions.current_action++;
                    compute_and_send_command(actions.current_action);
                    yDebug("Executing action: %4zd/%4zd", actions.current_action , last_action);
                    //printf("EXECUTING %d, elapsed_time:%.5f requested_time:%.5f\n", actions.current_action, current_time-last_time, duration);
                }
                else
                {
                    //printf("WAITING %d, elapsed_time:%.5f requested_time:%.5f\n", actions.current_action, current_time-last_time, duration);
                }
            }
            else
            {
                if (actions.forever==false)
                {
                    yInfo("sequence completed in: %f s",yarp::os::Time::now()-start_time);
                    actions.current_status=ACTION_RESET;
                }
                else
                {
                    yInfo("sequence completed in: %f s, restarting", yarp::os::Time::now() - start_time);
                    actions.current_action=0;
                    start_time = yarp::os::Time::now();
                }
            }
        }
        else if (actions.current_status==ACTION_START)
        {
            if (actions.action_vector.size()>0)
            {
                double *ll = actions.action_vector[0].q_joints;
                int nj = actions.action_vector[0].get_n_joints();
                for (int j = 0; j < nj; j++)
                {
                    driver->setControlMode(j, VOCAB_CM_POSITION);
                }
                yarp::os::Time::delay(0.1);
                for (int j = 0; j < nj; j++)
                {
                    driver->positionMove(j, ll[j]);
                }
                
                yInfo() << "going to home position";
                double enc[50];
                int loops = 100;
                bool check = true;
                do
                {
                    check = true;
                    for (int j = 0; j < nj; j++)
                    {
                        driver->getEncoder(j, &enc[j]);
                        double err = fabs(enc[j] - ll[j]);
                        check &= (err < 2.0);
                    }
                    yarp::os::Time::delay(0.1);
                    loops--;
                } while (!check && loops>0);

                if (check)
                {
                    yInfo() << "done";

                    for (int j = 0; j <nj; j++)
                    {
                        driver->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
                    }
                    yarp::os::Time::delay(0.1);
                    compute_and_send_command(0);

                    actions.current_status = ACTION_RUNNING;
                    start_time = yarp::os::Time::now();
                }
                else
                {
                    yError() << "unable to reach start position!";
                    if (0) 
                    {
                        //very strict behavior! if your are controlling fingers, you will probably end here
                        actions.current_status = ACTION_STOP;
                    }
                    else
                    {
                        for (int j = 0; j <nj; j++)
                        {
                            driver->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
                        }
                        yarp::os::Time::delay(0.1);
                        compute_and_send_command(0);

                        actions.current_status = ACTION_RUNNING;
                        start_time = yarp::os::Time::now();
                    }
                }
            }
            else
            {
                yWarning("no sequence in memory");
                actions.current_status=ACTION_STOP;
            }
        }
        else
        {
            yError() << "unknown current_status";
        }
    }
};

// ******************** THE MODULE
class scriptModule: public RFModule
{
protected:
    Port                rpcPort;
    string              name;
    bool                verbose;
    robotDriver         robot;
    WorkingThread       w_thread;
    BroadcastingThread  b_thread;

public:
    scriptModule() 
    {
        verbose=true;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        if (rf.check("name"))
            name=string("/")+rf.find("name").asString().c_str();
        else
            name="/trajectoryPlayer";

        rpcPort.open((name+"/rpc").c_str());
        attach(rpcPort);

        Property portProp;
        portProp.put("robot", rf.find("robot"));
        portProp.put("part", rf.find("part"));

        //*** start the robot driver
        if (!robot.configure(portProp))
        {
            yError() << "Error configuring position controller, check parameters";
            return false;
        }
        
        if (!robot.init())
        {
            yError() << "Error cannot connect to remote ports" ;
            return false;
        }

        //*** attach the robot driver to the thread and start it
        w_thread.attachRobotDriver(&robot);
        b_thread.attachRobotDriver(&robot);
        if (!w_thread.start())
        {
            yError() << "Working thread did not start, queue will not work";
        }
        else
        {
            yInfo() << "Working thread started";
        }

        if (rf.check("execute")==true)
        {
            yInfo() << "Enabling iPid->setReference() controller";
            w_thread.enable_execute_joint_command = true;
        }
        else
        {
            yInfo() << "Not using iPid->setReference() controller";
            w_thread.enable_execute_joint_command = false;
        }

        if (rf.check("period")==true)
        {
            int period = rf.find("period").asInt();
            yInfo() << "Thread period set to " << period << "ms";
            w_thread.setPeriod((double)period/1000.0);
        }
        yInfo() << "Using parameters:"  << rf.toString();

        //*** open the position file
        yInfo() << "opening file...";
        if (rf.check("filename")==true)
        {
            string filename = rf.find("filename").asString().c_str();
            int req_joints = 0; //default value
            if (rf.check("joints")) 
            {
                req_joints = rf.find("joints").asInt();}
            else
            {
                yError() << "Missing parameter 'joints' (number of joints to control)";
                return false;
            }

            if (req_joints > robot.n_joints)
            {
                yError() << "Requested number of joints exceeds the number of available joints on the robot!";
                return false;
            }

            if (req_joints < robot.n_joints)
            {
                yWarning() << "changing n joints from" << robot.n_joints << "to" << req_joints;
                robot.n_joints = req_joints;
            }

            if (rf.check("jointsMap"))
            {
                Bottle* b = rf.find("jointsMap").asList();
                if (!b) { yError() << "Error parsing parameter jointsMap";  return false; }
                if (b->size() != req_joints) { yError() << "invalid size of jointsMap parameter"; return false; }
                for (int i = 0; i < b->size(); i++)
                {
                    robot.joints_map[i] = b->get(i).asInt();
                }
            }
            else
            {
                //create a default joint map
                for (int i = 0; i < req_joints; i++)
                {
                    robot.joints_map[i] = i;
                }
            }

            if (!w_thread.actions.openFile(filename,robot.n_joints))
            {
                yError() << "Unable to parse file " << filename;
                return false;
            };
        }
        else
        {
            yWarning() << "no sequence files load.";
        }

        yInfo() << "module successfully configured. ready.";
        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        lock_guard<mutex> lck(this->w_thread.mtx);
        bool ret=true;

        if (command.size()!=0)
        {
            string cmdstring = command.get(0).asString().c_str();
            {
                if  (cmdstring == "help")
                    {                    
                        cout << "Available commands:"          << endl;
                        cout << "start" << endl;
                        cout << "stop"  << endl;
                        cout << "reset" << endl;
                        cout << "clear" << endl;
                        cout << "add" << endl;
                        cout << "load" << endl;
                        cout << "forever" << endl;
                        cout << "list" << endl;
                        reply.addVocab(Vocab::encode("many"));
                        reply.addVocab(Vocab::encode("ack"));
                        reply.addString("Available commands:");
                        reply.addString("start");
                        reply.addString("stop");
                        reply.addString("reset");
                        reply.addString("clear");
                        reply.addString("add");
                        reply.addString("load");
                        reply.addString("forever");
                        reply.addString("list");
                    }
                else if  (cmdstring == "start")
                    {
                        if (this->w_thread.actions.current_action == 0)
                            this->w_thread.actions.current_status = ACTION_START;
                        else
                            this->w_thread.actions.current_status = ACTION_RUNNING;
                        this->w_thread.actions.forever = false;

                        this->b_thread.attachActions(&w_thread.actions);
                        if (this->b_thread.isRunning()==false) b_thread.start();

                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "forever")
                    {
                        if (this->w_thread.actions.current_action == 0)
                            this->w_thread.actions.current_status = ACTION_START;
                        else
                            this->w_thread.actions.current_status = ACTION_RUNNING;
                        w_thread.actions.forever = true;
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "stop")
                    {
                        this->w_thread.actions.current_status = ACTION_STOP;
                        if (this->b_thread.isRunning()==true) b_thread.askToStop();

                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "clear")
                    {
                        this->w_thread.actions.clear();
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "add")
                    {
                        if(!w_thread.actions.parseCommandLine(command.get(1).asString().c_str(), -1, robot.n_joints))
                        {
                            yError() << "Unable to parse command";
                            reply.addVocab(Vocab::encode("ERROR Unable to parse file"));
                        }
                        else
                        {
                            yInfo() << "Command added";
                            reply.addVocab(Vocab::encode("ack"));
                        }
                    }
                else if  (cmdstring == "load")
                    {
                        string filename = command.get(1).asString().c_str();
                        if (!w_thread.actions.openFile(filename, robot.n_joints))
                        {
                            yError() << "Unable to parse file";
                            reply.addVocab(Vocab::encode("ERROR Unable to parse file"));
                        }
                        else
                        {
                            yInfo() << "File opened";
                            reply.addVocab(Vocab::encode("ack"));
                        }
                    }
                else if  (cmdstring == "reset")
                    {
                        this->w_thread.actions.current_status = ACTION_RESET;
                        this->w_thread.actions.current_action = 0;

                        if (this->b_thread.isRunning()==true) b_thread.askToStop();

                        reply.addVocab(Vocab::encode("ack"));
                    }
                else if  (cmdstring == "list")
                    {
                        this->w_thread.actions.print();
                        reply.addVocab(Vocab::encode("ack"));
                    }
                else
                    {
                        reply.addVocab(Vocab::encode("nack"));
                        ret = false;
                    }
            }
        }
        else
        {
            reply.addVocab(Vocab::encode("nack"));
            ret = false;
        }

        return ret;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setDefaultContext("ctpService");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:";
        yInfo() << "\t--joints       <n>:          number of joints, default 6";
        yInfo() << "\t--jointsMap    (1 2 3...)    map of the joints to be controller. Size must be equal to --joints";
        yInfo() << "\t--name         <moduleName>: set new module name";
        yInfo() << "\t--robot        <robotname>:  robot name";
        yInfo() << "\t--part         <robotname>:  part name";
        yInfo() << "\t--filename     <filename>:   the positions file";
        yInfo() << "\t--execute      activate the iPid->setReference() control";
        yInfo() << "\t--period       <period>: the period in ms of the internal thread (default 5)";
        yInfo() << "\t--verbose      to display additional infos";
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError() << "yarp.checkNetwork() failed.";
        return -1;
    }

    scriptModule mod;
 
    return mod.runModule(rf);
}



