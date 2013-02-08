// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Alberto Cardellino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "virtualAnalogServer.h"

#include <iostream>

using namespace std;

inline void appendTimeStamp(Bottle &bot, Stamp &st)
{
    int count=st.getCount();
    double time=st.getTime();
    bot.addVocab(VOCAB_TIMESTAMP);
    bot.addInt(count);
    bot.addDouble(time);
}

AnalogSubDevice::AnalogSubDevice()
{
    base=-1;
    top=-1;
    myChannels=0;

    device=0;
    sensor=0;
    iTimed= 0;

    configuredF=false;
    attachedF=false;
}

bool AnalogSubDevice::configure(int _base, int _top, int _channels, const std::string &key)
{
    configuredF=false;

    base = _base;
    top  = _top;
    myChannels = _channels;
    id = key;

    if (top < base)
    {
        cerr<<"check configuration file top<base."<<endl;
        return false;
    }

    if ((top-base+1) != _channels)
    {
        cerr<<"check configuration file, number of axes and top/base parameters do not match"<<endl;
        return false;
    }

    if (_channels <= 0)
    {
        cerr<<"check number of axes"<<endl;
        return false;
    }

    encoders.resize(_channels);
    encodersTimes.resize(_channels);

    configuredF=true;
    return true;
}

void AnalogSubDevice::detach()
{
    base=-1;
    top=-1;
    myChannels=0;

    device=0;
    sensor=0;
    iTimed= 0;

    configuredF=false;
    attachedF=false;
}

bool AnalogSubDevice::attach(yarp::dev::PolyDriver *_device, const std::string &k)
{
    if (id!=k)
    {
        cerr<<"Wrong device sorry."<<endl;
        return false;
    }

    //configure first
    if (!configuredF)
    {
        cerr<<"You need to call configure before you can attach any device"<<endl;
        return false;
    }

    if (_device==0)
    {
        cerr<<"Invalid device (null pointer)\n"<<endl;
        return false;
    }

    device = _device;

    if (device->isValid())
    {
        device->view(sensor);
        device->view(iTimed);
    }
    else
    {
        cerr<<"Invalid device (isValid() returned false"<<endl;
        return false;
    }


    if (iTimed!=0)
        std::cout << id << ":using IPreciselyTimed interface"<<endl;

    int deviceJoints=0;

    if (sensor != 0)
    {

        if (pos!=0)
        {
            if (!pos->getAxes(&deviceJoints))
            {
                std::cerr<< "Error: attached device has 0 axes\n";
                return false;
            }
        }
        if (vel!=0)
        {
            if (!vel->getAxes(&deviceJoints))
            {
                std::cerr<< "Error: attached device has 0 axes\n";
                return false;
            }
        }

        if (deviceJoints<axes)
        {
            std::cerr<<"check device configuration, number of joints of attached device less than the one specified during configuration.\n";
            return false;
        }

        attachedF=true;
        return true;
    }
    else
    {
        return false;
    }

    return false;
}


// d for the driver factory.
DriverCreator *createVirtualAnalogServer() {
    return new DriverCreatorOf<VirtualAnalogServer>("controlboard",
                                                     "controlboard",
                                                     "VirtualAnalogServer");
}

ImplementCallbackHelper2::ImplementCallbackHelper2(VirtualAnalogServer *x)
{
    IVAnalog = dynamic_cast<yarp::dev::IVirtualAnalogSensor *> (x);
}

#if 0

void CommandsHelper2::handleImpedanceMsg(const yarp::os::Bottle& cmd,
                                           yarp::os::Bottle& response, bool *rec, bool *ok)
{
    if (caller->verbose())
        fprintf(stderr, "Handling IImpedance message\n");
     if (!iImpedance)
        {
            fprintf(stderr, "Error I do not have a valid interface\n");
            *ok=false;
            return;
        }

    int code = cmd.get(0).asVocab();
    *ok=false;
    switch(code)
    {
    case VOCAB_SET:
        {
            if (caller->verbose())
                fprintf(stderr, "handleImpedanceMsg::VOCAB_SET command\n");
            switch (cmd.get(2).asVocab())
            {
                case VOCAB_IMP_PARAM:
                    {
                        Bottle *b = cmd.get(4).asList();
                        if (b!=NULL)
                        {
                            double stiff = b->get(0).asDouble();
                            double damp = b->get(1).asDouble();
                            *ok = iImpedance->setImpedance(cmd.get(3).asInt(),stiff,damp);
                            *rec=true;
                        }
                    }
                    break;
                case VOCAB_IMP_OFFSET:
                    {
                        Bottle *b = cmd.get(4).asList();
                        if (b!=NULL)
                        {
                            double offs = b->get(0).asDouble();
                            *ok = iImpedance->setImpedanceOffset(cmd.get(3).asInt(),offs);
                            *rec=true;
                        }
                    }
                    break;
            }
        }
        break;
    case VOCAB_GET:
        {
            double stiff = 0;
            double damp = 0;
            double offs = 0;
            if (caller->verbose())
                fprintf(stderr, "handleImpedanceMsg::VOCAB_GET command\n");

            response.addVocab(VOCAB_IS);
            response.add(cmd.get(1));
            switch (cmd.get(2).asVocab())
            {
                case VOCAB_IMP_PARAM:
                    {
                        *ok = iImpedance->getImpedance(cmd.get(3).asInt(),&stiff, &damp);
                        Bottle& b = response.addList();
                        b.addDouble(stiff);
                        b.addDouble(damp);
                        *rec=true;
                    }
                    break;
                case VOCAB_IMP_OFFSET:
                    {
                        *ok = iImpedance->getImpedanceOffset(cmd.get(3).asInt(),&offs);
                        Bottle& b = response.addList();
                        b.addDouble(offs);
                        *rec=true;
                    }
                    break;
                case VOCAB_LIMITS:
                    {
                        double min_stiff    = 0;
                        double max_stiff    = 0;
                        double min_damp     = 0;
                        double max_damp     = 0;
                        *ok = iImpedance->getCurrentImpedanceLimit(cmd.get(3).asInt(),&min_stiff, &max_stiff, &min_damp, &max_damp);
                        Bottle& b = response.addList();
                        b.addDouble(min_stiff);
                        b.addDouble(max_stiff);
                        b.addDouble(min_damp);
                        b.addDouble(max_damp);
                        *rec=true;
                    }
                    break;
            }
        }
        lastRpcStamp.update();
        appendTimeStamp(response, lastRpcStamp);
        break; // case VOCAB_GET
    default:
        {
            *rec=false;
        }
        break;
    }
}

void CommandsHelper2::handleControlModeMsg(const yarp::os::Bottle& cmd,
                                           yarp::os::Bottle& response, bool *rec, bool *ok)
{
    if (caller->verbose())
        fprintf(stderr, "Handling IControlMode message\n");
     if (!iMode)
        {
            fprintf(stderr, "Error I do not have a valid interface\n");
            *ok=false;
            return;
        }

    //TODO: handle here messages about  IControlMode interface
    int code = cmd.get(0).asVocab();
    *ok=true;

    switch(code)
    {
        case VOCAB_SET:
            {
                if (caller->verbose())
                    fprintf(stderr, "handleControlModeMsg::VOCAB_SET command\n");
                int p=-1;
                int axis = cmd.get(3).asInt();
                int mode=cmd.get(2).asVocab();
                switch (cmd.get(2).asInt())
                {
                    case VOCAB_CM_POSITION:
                        *ok = iMode->setPositionMode(axis);
                        break;
                    case VOCAB_CM_VELOCITY:
                        *ok = iMode->setVelocityMode(axis);
                        break;
                    case VOCAB_CM_TORQUE:
                        *ok = iMode->setTorqueMode(axis);
                        break;
                    case VOCAB_CM_IMPEDANCE_POS:
                        *ok = iMode->setImpedancePositionMode(axis);
                        break;
                    case VOCAB_CM_IMPEDANCE_VEL:
                        *ok = iMode->setImpedanceVelocityMode(axis);
                        break;
                    case VOCAB_CM_OPENLOOP:
                        *ok = iMode->setOpenLoopMode(axis);
                        break;
                    default:
                        *ok = false;
                        break;
                }
                *rec=true; //or false
            }
            break;
        case VOCAB_GET:
            {
                if (caller->verbose())
                    fprintf(stderr, "GET command\n");
                if (cmd.get(2).asVocab()==VOCAB_CM_CONTROL_MODES)
                {
                    if (caller->verbose())
                        fprintf(stderr, "getControlModes\n");
                    int *p = new int[controlledJoints];
                    *ok = iMode->getControlModes(p);

                    response.addVocab(VOCAB_IS);
                    response.addVocab(VOCAB_CM_CONTROL_MODES);

                    Bottle& b = response.addList();
                    int i;
                    for (i = 0; i < controlledJoints; i++)
                        b.addVocab(p[i]);
                    delete[] p;

                    *rec=true;
                }
                else if (cmd.get(2).asVocab()==VOCAB_CM_CONTROL_MODE)
                {
                    if (caller->verbose())
                        fprintf(stderr, "getControlMode\n");

                    int p=-1;
                    int axis = cmd.get(3).asInt();
                    *ok = iMode->getControlMode(axis, &p);

                    response.addVocab(VOCAB_IS);
                    response.addInt(axis);
                    response.addVocab(p);

                    //fprintf(stderr, "Returning %d\n", p);
                    *rec=true;
                }
            }
            lastRpcStamp.update();
            appendTimeStamp(response, lastRpcStamp);
            break; // case VOCAB_GET
        default:
            {
                *rec=false;
            }
            break;
    }
}


void CommandsHelper2::handleTorqueMsg(const yarp::os::Bottle& cmd,
                                      yarp::os::Bottle& response, bool *rec, bool *ok)
{

}

bool CommandsHelper2::respond(const yarp::os::Bottle& cmd,
                              yarp::os::Bottle& response)
{

    ACE_thread_t self=ACE_Thread::self();
    //    fprintf(stderr, "--> [%X] starting responder\n",self);

    bool ok = false;
    bool rec = false; // is the command recognized?
     if (caller->verbose())
        printf("command received: %s\n", cmd.toString().c_str());
    int code = cmd.get(0).asVocab();

    // If this is a torque message, pass it to the apprproate function
    // Ok, this is completely useless since we have only 1 mdg to handle ma fa figo!
    if ((cmd.size()>1) && (cmd.get(1).asVocab()==VOCAB_TORQUE))
    {
        handleTorqueMsg(cmd, response, &rec, &ok);
    }

    if (!rec)
    {
        //??? cosa fa?? chiede ad un qualche altro responder... if any???
        ok = DeviceResponder::respond(cmd,response);
    }

    if (!ok)
    {
        // failed thus send only a VOCAB back.
        response.clear();
        response.addVocab(VOCAB_FAILED);
    }
    else
        response.addVocab(VOCAB_OK);

    yDebug() << "Virtual AnalogServer responder --> thread["<< self << "] handled a msg with return value of" << ok;
    return ok;
}


bool CommandsHelper2::initialize()
{
    bool ok = false;
    if (pos)
        {
            ok = pos->getAxes(&controlledJoints);
        }

    DeviceResponder::makeUsage();
    addUsage("[set] [trq] $iAxisNumber", "set the torque value for an axis");
    addUsage("[set] [trqs] ", "set the torque values for all axes");
    return ok;
}

CommandsHelper2::CommandsHelper2(VirtualAnalogServer *x)
{
    caller = x;
    pid = dynamic_cast<yarp::dev::IPidControl *> (caller);
    pos = dynamic_cast<yarp::dev::IPositionControl *> (caller);
    vel = dynamic_cast<yarp::dev::IVelocityControl *> (caller);
    enc = dynamic_cast<yarp::dev::IEncodersTimed *> (caller);
    amp = dynamic_cast<yarp::dev::IAmplifierControl *> (caller);
    lim = dynamic_cast<yarp::dev::IControlLimits *> (caller);
    info = dynamic_cast<yarp::dev::IAxisInfo *> (caller);
    ical2= dynamic_cast<yarp::dev::IControlCalibration2 *> (caller);
    iOpenLoop=dynamic_cast<yarp::dev::IOpenLoopControl *> (caller);
    iDbg=dynamic_cast<yarp::dev::IDebugInterface *> (caller);
    iImpedance=dynamic_cast<yarp::dev::IImpedanceControl *> (caller);
    torque=dynamic_cast<yarp::dev::ITorqueControl *> (caller);
    iMode=dynamic_cast<yarp::dev::IControlMode *> (caller);
    controlledJoints = 0;
}

#endif

bool ImplementCallbackHelper2::initialize()
{
    controlledAxes=0;
    if (pos)
        pos->getAxes(&controlledAxes);

    return true;
}

void ImplementCallbackHelper2::onRead(CommandMessage& v)
{
////////////////////////////////////
// mine
////////////////////////////////////

 if (caller->verbose())
        fprintf(stderr, "Handling ITorqueControl message\n");

    if (!torque)
        {
            fprintf(stderr, "Error, I do not have a valid ITorque interface\n");
            *ok=false;
            return;
        }

    int code = cmd.get(0).asVocab();
    switch (code)
    {
        case VOCAB_SET:
            {
                *rec = true;
                if (caller->verbose())
                    printf("get command received\n");
                int tmp = 0;
                double dtmp  = 0.0;
                double dtmp2 = 0.0;
                response.addVocab(VOCAB_IS);
                response.add(cmd.get(1));

                switch(cmd.get(2).asVocab())
                {

                    case VOCAB_TRQ:
                        {
                            *ok = iVirtualSensor->setTorque(cmd.get(3).asInt(), cmd.get(4).asDouble());
                        }
                        break;

                    case VOCAB_TRQS:
                        {
                            Bottle *b = cmd.get(3).asList();
                            if (b==NULL)
                                break;

                            int i;
                            const int njs = b->size();
                            if (njs==controlledJoints)
                                {
                                    double *p = new double[njs];    // LATER: optimize to avoid allocation.
                                    for (i = 0; i < njs; i++)
                                        p[i] = b->get(i).asDouble();
                                    *ok = iVirtualSensor->setTorques (p);
                                    delete[] p;
                                }
                        }
                        break;
                }
            }
            lastRpcStamp.update();
            appendTimeStamp(response, lastRpcStamp);
            break; // case VOCAB_GET
    }

    ///////////////////////////////
    // prev
    ///////////////////////////////
    Bottle& b = v.head;
    Vector& cmdVector = v.body;

    // some consistency checks
    if (controlledAxes!=cmdVector.size())
    {
        yarp::os::ConstString str = yarp::os::Vocab::decode(b.get(0).asVocab());
        fprintf(stderr, "Received command vector with incorrect number of elements (cmd: %s requested jnts: %d received jnts: %d)\n",str.c_str(),controlledAxes,(int)cmdVector.size());
        return;
    }
    if (cmdVector.data()==0)
    {
         fprintf(stderr, "Received null command vector\n");
         return;
    }

    switch (b.get(0).asVocab())
        {
        case VOCAB_POSITION_MODE:
        case VOCAB_POSITION_MOVES:
            {
                //printf("Received a position command\n");
                //for (int i = 0; i < v.body.size(); i++)
                //    printf("%.2f ", v.body[i]);
                //printf("\n");

                if (pos)
                    {
                        bool ok = pos->positionMove(cmdVector.data());
                        if (!ok)
                            fprintf(stderr, "Issues while trying to start a position move\n");
                    }

            }
            break;

        case VOCAB_VELOCITY_MODE:
        case VOCAB_VELOCITY_MOVES:
            {
                //            printf("Received a velocity command\n");
                //            for (i = 0; i < v.body.size(); i++)
                //                printf("%.2f ", v.body[i]);
                //            printf("\n");
                if (vel)
                    {
                        bool ok = vel->velocityMove(cmdVector.data());
                        if (!ok)
                            fprintf(stderr, "Issues while trying to start a velocity move\n");
                    }
            }
            break;
        case VOCAB_OUTPUTS:
            {
                if (iOpenLoop)
                    {
                        bool ok=iOpenLoop->setOutputs(cmdVector.data());
                        if (!ok)
                            fprintf(stderr, "Issues while trying to command an open loop message\n");
                    }
            }
            break;
        default:
            {
                yarp::os::ConstString str = yarp::os::Vocab::decode(b.get(0).asVocab());
                fprintf(stderr, "Unrecognized message while receiving on command port (%s)\n",str.c_str());
            }
            break;
        }


    //    printf("v: ");
    //    int i <;
    //    for (i = 0; i < (int)v.size(); i++)
    //        printf("%.3f ", v[i]);
    //    printf("\n");
}

/**
* Constructor.
*/
VirtualAnalogServer::VirtualAnalogServer() : RateThread(20), callback_impl(this), command_reader(this)
{
    channels = 0;
    // thread_period = 20; // ms.

    verb = false;
}

VirtualAnalogServer::~VirtualAnalogServer()
{
    //YARP_TRACE(Logger::get(),"VirtualAnalogServer::~VirtualAnalogServer()", Logger::get().log_files.f3);
    closeMain();
}

bool VirtualAnalogServer::open(Searchable& prop)
{
    string str=prop.toString().c_str();

    cout << str << endl << endl;

    verb = (prop.check("verbose","if present, give detailed output"));
    if (verb)
        cout<<"running with verbose output\n";

//  thus thread period is useful for output port... this input port has callback so maybe can skip it (?)
//     thread_period = prop.check("threadrate", 20, "thread rate in ms. for streaming encoder data").asInt();

    int totalJ=0;

    std::cout<<"Using VirtualAnalogServer\n";

    if (!prop.check("networks", "list of networks merged by this wrapper"))
        return false;

    Bottle *nets=prop.find("networks").asList();

    if (!prop.check("channels", "number of channels "))
        return false;

    channels=prop.find("channels").asInt();

    int nsubdevices=nets->size();
    lut.resize(channels);
    subdevices.resize(nsubdevices);

    for(int k=0;k<nets->size();k++)
        {
            Bottle parameters=prop.findGroup(nets->get(k).asString().c_str());

            if (parameters.size()!=5)    // mapping joints using the paradigm: part from - to / network from - to
                {
                    cerr<<"Error: check network parameters in part description"<<endl;
                    cerr<<"--> I was expecting "<<nets->get(k).asString().c_str() << " followed by four integers"<<endl;
                    return false;
                }

            int wBase=parameters.get(1).asInt();
            int wTop=parameters.get(2).asInt();
            base=parameters.get(3).asInt();
            top=parameters.get(4).asInt();

            //cout<<"--> "<<wBase<<" "<<wTop<<" "<<base<<" "<<top<<endl;

            //TODO check consistenty
            int axes=top-base+1;

            AnalogSubDevice *tmpDevice=device.getSubdevice(k);

            if (!tmpDevice->configure(base, top, axes, nets->get(k).asString().c_str()))
                {
                    cerr<<"configure of subdevice ret false"<<endl;
                    return false;
                }

            for(int j=wBase;j<=wTop;j++)
                {
                    lut[j].deviceEntry=k;
                    lut[j].offset=j-wBase;
                }

            totalJ+=axes;
        }

    if (totalJ!=channels)
        {
            cerr<<"Error total number of mapped channels does not correspond to part channels"<<endl;
            return false;
        }

    // initialize callback
    if (!inputStreamingData_callback.initialize())
    {
        cerr<<"Error could not initialize callback object"<<endl;
        return false;
    }

    partName=prop.check("name",Value("controlboard"),
                        "prefix for port names").asString().c_str();
    std::string rootName="/";
    rootName+=(partName);

    // attach readers.
    //rpc_p.setReader(command_reader);
    // changed so that streaming input accepted if offered
    command_buffer.attach(rpc_p);
    command_reader.attach(command_buffer);

    // attach buffers.
    state_buffer.attach(state_p);
    control_buffer.attach(control_p);
    // attach callback.
    control_buffer.useCallback(callback_impl);

    rpc_p.open((rootName+"/rpc:i").c_str());
    control_p.open((rootName+"/command:i").c_str());
    state_p.open((rootName+"/state:o").c_str());

    return true;
}

bool VirtualAnalogServer::close()
{
    if (RateThread::isRunning())
            RateThread::stop();

    inputStreamingData_p.close();
}

bool VirtualAnalogServer::attachAll(const PolyDriverList &polylist)
{
    for(int p=0;p<polylist.size();p++)
        {
            // find appropriate entry in list of subdevices
            // and attach
            unsigned int k=0;
            for(k=0; k<subdevices.size(); k++)
                {
                    std::string tmpKey=polylist[p]->key.c_str();
                    if (subdevices[k].id==tmpKey)
                        {
                            if (!subdevices[k].attach(polylist[p]->poly, tmpKey))
                                return false;
                        }
                }
        }

    //check if all devices are attached to the driver
    bool ready=true;
    for(unsigned int k=0; k<subdevices.size(); k++)
        {
            if (!subdevices[k].isAttached())
                {
                    ready=false;
                }
        }

    if (!ready)
        return false;

    RateThread::setRate(thread_period);
    RateThread::start();

    return true;
}

void VirtualAnalogServer::run()
{
    std::string tmp(partName.c_str());

    yarp::sig::Vector& v = state_buffer.get();
    v.size(controlledJoints);

    //getEncoders for all subdevices
    double *encoders=v.data();
    double timeStamp=0.0;

    for(unsigned int k=0;k<device.subdevices.size();k++)
        {
            int axes=device.subdevices[k].axes;
            int base=device.subdevices[k].base;

            device.subdevices[k].refreshEncoders();

            for(int l=0;l<axes;l++)
            {
                encoders[l]=device.subdevices[k].encoders[l+base];
                timeStamp+=device.subdevices[k].encodersTimes[l+base];
            }

            encoders+=device.subdevices[k].axes; //jump to next group
        }

    timeMutex.wait();
    time.update(timeStamp/controlledJoints);
    timeMutex.post();

    state_p.setEnvelope(time);
    state_buffer.write();
}

inline AnalogSubDevice *VirtualAnalogServer::getSubdevice(unsigned int i)
{
    if (i>=subdevices.size())
        return 0;

    return &subdevices[i];
}
