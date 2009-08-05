
#include <ace/OS.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PortablePair.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/iKinIpOpt.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;


typedef PortablePair<Bottle, Vector> CommandMessage;


class rxPort : public BufferedPort<Bottle>
{
public:
    rxPort(const string &armType, BufferedPort<CommandMessage> &_txPort, double _radius,
           double _hyster, double _disTime, int _link) :
           txPort(_txPort), radius(_radius), hyster(_hyster), disTime(_disTime), link(_link)
    {
        arm=new iCubArm(armType);

        iKinChain &mainChain=*arm->asChain();
        chain=new iKinChain(mainChain.getH0());

        for (int i=0; i<link+3; i++)
            *chain << mainChain[i];

        solver=new iKinIpOptMin(*chain,IKINCTRL_POSE_FULL,1e-4,100);

        solver->setUserScaling(true,100.0,100.0,100.0);

        state=0;
    }

    ~rxPort()
    {
        delete solver;
        delete chain;
        delete arm;
    }

private:
    BufferedPort<CommandMessage> &txPort;
    double radius;
    double hyster;
    double disTime;
    int    link;

    double t0;

    int state;

    iCubArm      *arm;
    iKinChain    *chain;
    iKinIpOptMin *solver;

    virtual void onRead(Bottle &b)
    {
        int n=b.size();
        Vector q_in(n<7?7:n); q_in=0.0;

        for (int i=0; i<n; i++)
            q_in[i]=b.get(i).asDouble();

        Vector q=chain->setAng((M_PI/180.0)*q_in);
        Vector x=chain->EndEffPose();
        Vector x_in=x;
        Vector x_out=x_in;

        double rho=sqrt(x[0]*x[0]+x[1]*x[1]);
        double phi=atan2(x[1],x[0]);
        double rho2=rho;
        double phi2=phi;
        double dt;

        if (state==0 && rho<radius)
        {
            state=1;

            CommandMessage &cmd=txPort.prepare();
            cmd.head.clear();
            cmd.head.addVocab(Vocab::encode("poss"));
            cmd.body=q_in;
            txPort.write();

            x[0]=(radius+hyster)*cos(phi);
            x[1]=(radius+hyster)*sin(phi);

            Vector dummyVector(1);

            t0=Time::now();
            q=solver->solve(q,x,0.0,dummyVector,dummyVector,0.0,dummyVector,dummyVector);
            dt=Time::now()-t0;

            x=chain->EndEffPose();

            rho2=sqrt(x[0]*x[0]+x[1]*x[1]);
            phi2=atan2(x[1],x[0]);

            x_out=x;

            Vector q_out(16); q_out=0.0;

            int i;
            for (i=0; i<q.size(); i++)
                q_out[i]=(180.0/M_PI)*q[i];

            for (; i<7; i++)
                q_out[i]=q_in[i];

            cmd=txPort.prepare();
            cmd.head.clear();
            cmd.head.addVocab(Vocab::encode("poss"));
            cmd.body=q_out;
            txPort.write();
    
            cout << "q_rx      = " << q_in.toString()                   << endl;
            cout << "x_rx      = " << x_in.toString()                   << endl;
            cout << "rho       = " << rho << " < " << radius            << endl;
            cout << "Projection executed in " << dt << " s:"            << endl;
            cout << "q_tx      = " << q_out.toString()                  << endl;
            cout << "x_tx      = " << x_out.toString()                  << endl;
            cout << "rho       = " << rho2                              << endl;
            cout << "delta_phi = " << (180.0/M_PI)*(phi2-phi) << " deg" << endl;

            cout << endl << endl;
        }
        else
            cout << "rho = " << rho << endl;

        if (state==1)
            if (Time::now()-t0>=disTime)
                state=0;
    }
};



class cylGuardModule: public RFModule
{
private:
    BufferedPort<CommandMessage>  cgTxPort;
    rxPort                       *cgRxPort;
    Port                          rpcPort;
    string                        portName;

public:
    cylGuardModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string armType;
        double radius;
        double hyster;
        double disTime;
        int    link;

        Time::turboBoost();

        if (rf.check("name"))
            portName=rf.find("name").asString();
        else
            portName="/cylGuard";

        if (rf.check("arm"))
            armType=rf.find("arm").asString();
        else
            armType="right";

        if (rf.check("radius"))
            radius=rf.find("radius").asDouble();
        else
            radius=0.2;

        if (rf.check("hyster"))
            hyster=rf.find("hyster").asDouble();
        else
            hyster=0.01;

        if (rf.check("time"))
            disTime=rf.find("time").asDouble();
        else
            disTime=2.0;

        if (rf.check("link"))
            link=rf.find("link").asInt();
        else
            link=7;

        link=link<1 ? 1 : (link>7 ? 7 : link);

        string rpcPortName=portName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        string txPortName=portName+"/out";        
        cgTxPort.open(txPortName.c_str());

        string rxPortName=portName+"/in";
        cgRxPort=new rxPort(armType,cgTxPort,radius,hyster,disTime,link);
        cgRxPort->useCallback();
        cgRxPort->open(rxPortName.c_str());
        
        return true;
    }

    virtual bool close()
    {
        cgRxPort->interrupt();
        cgTxPort.interrupt();
        rpcPort.interrupt();

        cgRxPort->close();
        cgTxPort.close();
        rpcPort.close();

        delete cgRxPort;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name    name:    port name (default /cylGuard)"                                            << endl;
        cout << "\t--arm     arm:     iCub arm 'left' or 'right' (default: 'right')"                            << endl;
        cout << "\t--radius  radius:  radius of the guard cylinder in m (default 0.2 m)"                        << endl;
        cout << "\t--hyster  hyster:  hysteresis threshold in m (default 0.01 m)"                               << endl;
        cout << "\t--time    disTime: time in s during which the check is disabled after trigger (default 2 s)" << endl;
        cout << "\t--link    link:    the link whose end-effector shall be put under control (default 7)"       << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    cylGuardModule mod;

    return mod.runModule(rf);
}


