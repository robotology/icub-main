#include "ArmDrumStickSolver.h"
#include "ICubArmDrumStick.h"

#include "iKiniCubShoulderConstr.h"

PartDescriptor *ArmDrumStickSolver::getPartDesc(Searchable &options)
{
	// standard option processing copied from the ArmCartesianSolver (parent) class
	type="right";
	if (options.check("type"))
	{    
		type=options.find("type").asString();
		if (type!="left" && type!="right")
			type="right";
	}

	string robot="/icub";
	if (options.check("robot"))
		robot=options.find("robot").asString();

	//recovers the drumStickLenth from the config file
	double drumStickLength = 0;
	if(options.check("drumStickLength"))
		drumStickLength = options.find("drumStickLength").asDouble();

	Property optTorso("(device remote_controlboard)");
	Property optArm("(device remote_controlboard)");

	string partTorso  ="torso";
	string remoteTorso="/"+robot+"/"+partTorso;
	string localTorso ="/"+slvName+"/"+partTorso;
	optTorso.put("remote",remoteTorso.c_str());
	optTorso.put("local",localTorso.c_str());

	string partArm = type=="left" ? "left_arm" : "right_arm";
	string remoteArm="/"+robot+"/"+partArm;
	string localArm ="/"+slvName+"/"+partArm;
	optArm.put("remote",remoteArm.c_str());
	optArm.put("local",localArm.c_str());

	PartDescriptor *p=new PartDescriptor;

	//instantiation of the custom Arm + Drumstick system.
	p->lmb=new ICubArmDrumStick(type, drumStickLength);


	//and some more standard stuff from the parent class
	p->chn=p->lmb->asChain();
	p->cns=new iCubShoulderConstr(p->chn);
	p->prp.push_back(optTorso);
	p->prp.push_back(optArm);
	p->rvs.push_back(true);     // torso
	p->rvs.push_back(false);    // arm
	p->num=2;                   // number of parts

	return p;
}			