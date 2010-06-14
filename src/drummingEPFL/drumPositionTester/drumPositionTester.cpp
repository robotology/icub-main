#include "drumPositionTester.h"

#include <yarp/math/Math.h>
using namespace yarp::math;
#include <iCub/ctrlMath.h>
using namespace ctrl;

#include <iCub/iKinVocabs.h>

//#define TEST


void DrumPositionTester::DoStuff(Stuff &stuff)
{
    int c = stuff.a * stuff.b;
}

DrumPositionTester::DrumPositionTester()
{
    DoStuff(Stuff(1,2));
}

DrumPositionTester::~DrumPositionTester(void)
{
}

bool DrumPositionTester::open(Searchable& config)
{
	OpenIKSolver("right_arm");
	OpenIKSolver("left_arm");

    outFileLeft.open(FILE_LEFT.c_str());
    outFileRight.open(FILE_RIGHT.c_str());

    x = MIN_X;
    y = MIN_Y;
    z = MIN_Z;

	return true;
}


bool DrumPositionTester::close()
{
	CloseIKSolver("left_arm");
	CloseIKSolver("right_arm");
	
	delete iKinPorts["left_arm"];
	delete iKinPorts["right_arm"];

	return true;
}


bool DrumPositionTester::updateModule(void)
{

    if(x < MAX_X)
    {
        x += STEP;
    }
    else
    {
        if(y < MAX_Y)
        {
            y += STEP;
            x = MIN_X;
        }
        else
        {
            if(z < MAX_Z)
            {
                z += STEP;
                x = MIN_X;
                y = MIN_Y;
            }
            else
            {
                return false;
            }
        }
    }

    Vector xp(3);
	xp[0] = x;
	xp[1] = y;
	xp[2] = z;	

	double precisionL, precisionR;
	Vector resultQL = Solve(xp, "left_arm", precisionL);
    Vector resultQR = Solve(xp, "right_arm", precisionR);

	outFileLeft << x << "\t" << y << "\t" << z << "\t"  << precisionL << "\t";
    outFileRight << x << "\t" << y << "\t" << z << "\t"  << precisionR << "\t";
    for(int i=0; i<resultQL.size(); ++i)
    {
        outFileLeft << resultQL[i] << "\t"; 
        outFileRight << resultQR[i] << "\t";
    }
    outFileLeft << endl;
    outFileRight << endl;

	return true;
}

double DrumPositionTester::getPeriod(void)
{
	return MODULE_PERIOD;
}

void DrumPositionTester::OpenIKSolver(string arm)
{
	cout << "=====================================" << endl;
	cout << "Opening IKin Cartesian Solver for " << arm  << endl;
	cout << "=====================================" << endl;
	// declare the on-line arm solver
	string solverName = "DrumCartesianSolver/" + arm ;

	iKinPorts[arm] = new IKinPort;
	iKinPorts[arm]->in.open(("/" + (string)MODULE_NAME + "/" + arm + "/in").c_str());
	iKinPorts[arm]->out.open(("/" + (string)MODULE_NAME + "/" + arm + "/out").c_str());
	iKinPorts[arm]->rpc.open(("/" + (string)MODULE_NAME + "/" + arm + "/rpc").c_str());

    Network::connect(("/" + solverName + "/out").c_str(),iKinPorts[arm]->in.getName().c_str());
    Network::connect(iKinPorts[arm]->out.getName().c_str(),("/" + solverName + "/in").c_str());
    Network::connect(iKinPorts[arm]->rpc.getName().c_str(),("/" + solverName + "/rpc").c_str());

	Bottle cmd, reply;


	//cmd.clear();
	//cmd.addVocab(IKINSLV_VOCAB_CMD_SET);
 //   cmd.addVocab(IKINSLV_VOCAB_OPT_LIM);
 //   cmd.addInt(4);
	//cmd.addDouble(10);
	//cmd.addDouble(90);
 //   iKinPorts[arm]->rpc.write(cmd,reply);
	//cout<<reply.size()<<endl;  


	
}


void DrumPositionTester::CloseIKSolver(string arm)
{
	iKinPorts[arm]->in.close();
	iKinPorts[arm]->out.close();
	iKinPorts[arm]->rpc.close();
}


Vector DrumPositionTester::Solve(const Vector &xd, string partName, double &precision)
{
	Vector resultQ(7);

	Bottle cmd, reply;
	cmd.clear();

	cout << partName << " - XD : " << xd[0] << " - " << xd[1] << " - " << xd[2] << endl;

	iCubArmCartesianSolver::addTargetOption(cmd, xd);

	iKinPorts[partName]->out.write(cmd);
	iKinPorts[partName]->in.read(reply);

	Bottle *xdBottle = CartesianSolver::getTargetOption(reply);
	Bottle *xBottle = CartesianSolver::getEndEffectorPoseOption(reply);
	Bottle *qBottle = CartesianSolver::getJointsOption(reply);

	Vector delta(3);
	delta[0] = xBottle->get(0).asDouble() - xdBottle->get(0).asDouble();
	delta[1] = xBottle->get(1).asDouble() - xdBottle->get(1).asDouble();
	delta[2] = xBottle->get(2).asDouble() - xdBottle->get(2).asDouble();

	double deltaNorm2 = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];
	precision = sqrt(deltaNorm2);

	for(int i =0; i<7; ++i)
    {
		resultQ[i] = qBottle->get(i).asDouble();
    }

	return resultQ;
}
