
#include <RpcMsgHandler.h>
#include <DebugInterfaceWrapper.h>


using namespace iCub::Wrapper::iDebug;

RpcMsgHandler::RpcMsgHandler(DebugInterfaceWrapper *x)
{
    caller = x;
    //    iPos_rpcHandler  = dynamic_cast<yarp::dev::IPositionControl *> (caller);
    //    iDbg_rpcHandler  = dynamic_cast<yarp::dev::IDebugInterface *> (caller);
    controlledJoints = 0;
    tmpDoubleArray = NULL;
}


RpcMsgHandler::~RpcMsgHandler()
{
    caller = NULL;
    checkAndDestroyDebug<double> (tmpDoubleArray);
}


bool RpcMsgHandler::initialize()
{
    bool ok = false;
    //    if (iPos_rpcHandler)
    {
        ok = caller->getAxes(controlledJoints);
    }
    tmpDoubleArray = new double[controlledJoints];

    DeviceResponder::makeUsage();
    addUsage("[get] [axes]", " get the number of axes");
    addUsage("[get] [dbgp]  <j> <index> ", " get debug parameter <index> for joint <j>");
    addUsage("[get] [genp]  <j> <index> ", " get generic parameter <index> for joint <j>");
    addUsage("[get] [ddps]  <j> ", " get debug desired position for joint <j>");
    addUsage("[get] [drp]   <j> ", " get debug rotor   position for joint <j>");
    addUsage("[get] [drps]      ", " get debug rotor   position for all joints");
    addUsage("[get] [drv]   <j> ", " get debug rotor   velocity for joint <j>");
    addUsage("[get] [drvs]      ", " get debug rotor   velocity for all joints");
    addUsage("[get] [dra]   <j> ", " get debug rotor   acceleration for joint <j>");
    addUsage("[get] [dras]      ", " get debug rotor   acceleration for all joints");
    addUsage("[get] [djp]   <j> ", " get debug joint   position for joint <j>");
    addUsage("[get] [djps]      ", " get debug joint   position for all joints");

    addUsage("[set] [dbgp]  <j> <index> ", " set debug parameter <index> for joint <j>");
    addUsage("[set] [genp]  <j> <index> ", " set generic parameter <index> for joint <j>");
    addUsage("[set] [ddps]  <j> ", " set debug desired position for joint <j>");

//    std::string args;
//    for (int i=0; i<controlledJoints; i++) {
//        if (i>0) {
//            args += " ";
//        }
//    }
//    addUsage((std::string("[set] [poss] (")+args+")").c_str(),
//             "command the position of all axes");
//    addUsage((std::string("[set] [rels] (")+args+")").c_str(),
//             "command the relative position of all axes");
//    addUsage((std::string("[set] [vmos] (")+args+")").c_str(),
//             "command the velocity of all axes");

//    addUsage("[set] [aen] $iAxisNumber", "enable (amplifier for) the given axis");
//    addUsage("[set] [adi] $iAxisNumber", "disable (amplifier for) the given axis");
//    addUsage("[get] [acu] $iAxisNumber", "get current for the given axis");
//    addUsage("[get] [acus]", "get current for all axes");

    return ok;
}



// Callback handler for RPC commands (?)
bool RpcMsgHandler::respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply)
{
    bool ok = true;
    bool commandUnderstood = true; // is the command recognized?

    if (caller->verbose())
        printf("command received: %s\n", cmd.toString().c_str());

    int code = cmd.get(0).asVocab();

    switch (code)
    {
        case VOCAB_GET:
        {
            double dtmp  = 0.0;
            double dtmp2 = 0.0;
            reply.addVocab(VOCAB_IS);
            reply.add(cmd.get(1));

            switch(cmd.get(1).asVocab())
            {
                case VOCAB_AXES:
                {
                    int axes;
                    caller->getAxes(axes);
                    reply.addInt(axes);
                }
                break;

                case VOCAB_DEBUG_PARAMETER:
                {
                    int j     = cmd.get(2).asInt();
                    int index = cmd.get(3).asInt();
                    ok = caller->getDebugParameter(j, index, &dtmp);
                    reply.addInt(j);
                    reply.addInt(index);
                    reply.addDouble(dtmp);
                }
                break;

                case VOCAB_DEBUG_DESIRED_POS:
                {
                    int j = cmd.get(2).asInt();
                    ok = caller->getDebugReferencePosition(j, &dtmp);
                    reply.addInt(j);
                    reply.addDouble(dtmp);
                }
                break;

                case VOCAB_DEBUG_ROTOR_POS:
                {
                    int j = cmd.get(2).asInt();
                    ok = caller->getRotorPosition(j, &dtmp);
                    reply.addInt(j);
                    reply.addDouble(dtmp);
                }
                break;

                case VOCAB_DEBUG_ROTOR_POSS:
                {
                    ok = caller->getRotorPositions(tmpDoubleArray);
                    for(int j=0; j<controlledJoints; j++)
                    {
                        reply.addDouble(tmpDoubleArray[j]);
                    }
                }
                break;

                case VOCAB_DEBUG_ROTOR_SPEED:
                {
                    int j = cmd.get(2).asInt();
                    ok = caller->getRotorSpeed(j, &dtmp);
                    reply.addInt(j);
                    reply.addDouble(dtmp);
                }
                break;

                case VOCAB_DEBUG_ROTOR_SPEEDS:
                {
                    ok = caller->getRotorSpeeds(tmpDoubleArray);
                    for(int j=0; j<controlledJoints; j++)
                        reply.addDouble(tmpDoubleArray[j]);
                }
                break;

                case VOCAB_DEBUG_ROTOR_ACCEL:
                {
                    int j     = cmd.get(2).asInt();
                    ok = caller->getRotorAcceleration(j, &dtmp);
                    reply.addInt(j);
                    reply.addDouble(dtmp);
                }
                break;

                case VOCAB_DEBUG_ROTOR_ACCELS:
                {
                    ok = caller->getRotorAccelerations(tmpDoubleArray);
                    for(int j=0; j<controlledJoints; j++)
                        reply.addDouble(tmpDoubleArray[j]);
                }
                break;

                case VOCAB_DEBUG_JOINT_POS:
                {
                    int j     = cmd.get(2).asInt();
                    ok = caller->getJointPosition(j, &dtmp);
                    reply.addInt(j);
                    reply.addDouble(dtmp);
                }
                break;

                case VOCAB_DEBUG_JOINT_POSS:
                {
                    ok = caller->getJointPositions(tmpDoubleArray);
                    for(int j=0; j<controlledJoints; j++)
                        reply.addDouble(tmpDoubleArray[j]);
                }
                break;

                case VOCAB_GENERIC_PARAMETER:
                {
                    int j     = cmd.get(2).asInt();
                    int param = cmd.get(3).asInt();
                    ok = caller->getParameter(j, param, &dtmp);
                    reply.addInt(j);
                    reply.addInt(param);
                    reply.addDouble(dtmp);
                }
                break;

                default:
                {
                    commandUnderstood = false;
                    std::cout << "Debug Interface 1: command not understood! received " << cmd.toString().c_str() << std::endl;
                }
                break;
            }
        }
        break;      // case VOCAB_GET

        case VOCAB_SET:
        {
            switch(cmd.get(1).asVocab())
            {
                case VOCAB_GENERIC_PARAMETER:
                {
                    int j     = cmd.get(2).asInt();
                    int param = cmd.get(3).asInt();
                    double val   = cmd.get(4).asDouble();
                    ok = caller->setParameter(j, param, val);
                }
                break;

                case VOCAB_DEBUG_PARAMETER:
                {
                    int j     = cmd.get(2).asInt();
                    int index = cmd.get(3).asInt();
                    double val   = cmd.get(4).asDouble();
                    ok = caller->setDebugParameter(j, index, val);
                }
                break;

                case VOCAB_DEBUG_DESIRED_POS:
                {
                    std::cout << " set VOCAB_DEBUG_DESIRED_POS " << std::endl;

                    int j     = cmd.get(2).asInt();
                    double val   = cmd.get(3).asDouble();
                    ok = caller->setDebugReferencePosition(j, val);
                }
                break;

                default:
                {
                    commandUnderstood = false;
                    std::cout << "Debug Interface 2: command not understood! received " << cmd.toString().c_str() << std::endl;
                }
                break;
            }
        }
        break;      // case VOCAB_SET

        default:
        {
            commandUnderstood = false;
            std::cout << "Debug Interface 3: command not understood! received " << cmd.toString().c_str() << std::endl;
        }
        break;

    } //switch code

    if (!commandUnderstood)
    {
        ok = DeviceResponder::respond(cmd,reply);
    }


    if (!ok)
    {
        // failed thus send only a VOCAB back.
        reply.clear();
        reply.addVocab(VOCAB_FAILED);
    }
    else
        reply.addVocab(VOCAB_OK);

    return ok;
}

