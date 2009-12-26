
#include <ace/Auto_Event.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

#include <iCub/iKinVocabs.h>
#include <iCub/iKinHlp.h>
#include <iCub/iKinSlv.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;


// this class handles the incoming messages from the solver
class solutionPort : public BufferedPort<Bottle>
{
protected:
    ACE_Auto_Event ev;
    Bottle sol;

public:
    solutionPort()
    {
        useCallback();        
        ev.reset();
    }

    virtual void onRead(Bottle& b)
    {
        sol=b;
        ev.signal();
    }

    void wait(Bottle& b)
    {
        ev.wait();
        b=sol;
        ev.reset();
    }
};


int main()
{
    Bottle cmd, reply;
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;
    
    // declare the on-line arm solver called "solver"
    iCubArmCartesianSolver onlineSolver("solver");

    Property options;
    // it will operate on the simulator (which is supposed to be already running)
    options.put("robot","icubSim");
    // it will work with the right arm
    options.put("type","right");
    // it will achieve just the positional pose
    options.put("pose","xyz");
    // switch off verbosity
    options.put("verbosity","off");

    // launch the solver and let it connect to the simulator
    if (!onlineSolver.open(options))
        return -1;
    
    // prepare ports
    solutionPort in;
    Port out, rpc;
    in.open("/in"); out.open("/out"); rpc.open("/rpc");
    Network::connect("/solver/out",in.getName().c_str());
    Network::connect(out.getName().c_str(),"/solver/in");
    Network::connect(rpc.getName().c_str(),"/solver/rpc");

    // print status    
    cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_GET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_DOF);
    rpc.write(cmd,reply);
    cout<<"got dof: "<<reply.toString()<<endl;

    cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_GET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_POSE);
    rpc.write(cmd,reply);
    cout<<"got pose: "<<reply.toString()<<endl;

    cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_GET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_MODE);
    rpc.write(cmd,reply);
    cout<<"got mode: "<<reply.toString()<<endl;

    // change to tracking mode so that when we
    // reconfigure chain's dof or any movements
    // induced on unactuated joints are detected
    // the solver is able to react
    cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_SET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_MODE);
    cmd.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
    cout<<"switching to track mode...";
    rpc.write(cmd,reply);
    cout<<reply.toString()<<endl;    

    // ask to resolve for some xyz position
    cmd.clear();
    Vector xd(3);
    xd[0]=-0.3;
    xd[1]=0.0;
    xd[2]=0.1;
    iCubArmCartesianSolver::addTargetOption(cmd,xd);
    out.write(cmd);
    in.wait(reply);

    cout<<"xd      ="<<CartesianHelper::getTargetOption(reply)->toString()<<endl;
    cout<<"x       ="<<CartesianHelper::getEndEffectorPoseOption(reply)->toString()<<endl;
    cout<<"q [deg] ="<<CartesianHelper::getJointsOption(reply)->toString()<<endl;
    cout<<endl;

    // ask the same but with torso enabled
    cmd.clear();
    Vector dof(3);
    dof=1;
    iCubArmCartesianSolver::addDOFOption(cmd,dof);
    out.write(cmd);
    in.wait(reply);

    cout<<"xd      ="<<CartesianHelper::getTargetOption(reply)->toString()<<endl;
    cout<<"x       ="<<CartesianHelper::getEndEffectorPoseOption(reply)->toString()<<endl;
    cout<<"q [deg] ="<<CartesianHelper::getJointsOption(reply)->toString()<<endl;
    cout<<endl;

    // close up
    onlineSolver.close();
    in.close();
    out.close();
    rpc.close();

    return 0;
}


      
