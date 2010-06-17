//
// An example on using iCubWholeBody to estimate the measurements of the FT sensors
// for all the arms and legs, exploiting the modeled dynamic and the inertial sensor 
// measurements.
//
// Author: Serena Ivaldi - <serena.ivaldi@iit.it>

#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace iDyn;


int main()
{
	// declare an icub = head + left arm + right arm + torso + left leg + right leg
	// the kinematic and dynamic parameters of each link are automatically set using the 
	// CAD model data.
	// icub default parameters are:
	// mode    = DYNAMIC : the mode for computing wrenches, considering q,dq,d2q,mass,inertia of each link;
	//                     the other main mode is STATIC, which only considers q and mass.
	// verbose = VERBOSE : the verbosity level; I suggest to use VERBOSE during debug, even if you see a lot 
	//                     of messages; then to turn it off with NO_VERBOSE only if everything is really fine;
	//                     if you are using iCubWholeBody you probably don't need it a lot because the inner
	//                     classes have been tested first with iCub, but if you use iDyn in general and use
	//                     it on your own code, it's better to turn verbosity on, it may help with the library
	//                     use. 
    iCubWholeBody icub;

	// just priting some information to see how the class works
	// iCubWholeBody is basically a container, with Upper and Lower Torso. these two are
	// in fact two public pointers to the iCubUpperTorso and iCubLowerTorso objects, each
	// having all the useful methods for each limb. To access to a specific limb, you must 
	// 'address' it as a string. The string name is quite familiar, it recalls yarp ports..
	cout<<endl
		<<"iCub has many DOF: "<<endl
		<<" - head      : "<<icub.upperTorso->getNLinks("head")<<endl
		<<" - left arm  : "<<icub.upperTorso->getNLinks("left_arm")<<endl
		<<" - right arm : "<<icub.upperTorso->getNLinks("right_arm")<<endl
		<<" - torso     : "<<icub.lowerTorso->getNLinks("torso")<<endl
		<<" - left leg  : "<<icub.lowerTorso->getNLinks("left_leg")<<endl
		<<" - right leg : "<<icub.lowerTorso->getNLinks("right_leg")<<endl<<endl;

	// now we set the joint angles for all the limbs
	// if connected to the real robot, we can take this values from the encoders
	Vector q_head(icub.upperTorso->getNLinks("head"));			q_head.zero();
	Vector q_larm(icub.upperTorso->getNLinks("left_arm"));		q_larm.zero();
	Vector q_rarm(icub.upperTorso->getNLinks("right_arm"));		q_rarm.zero();
	Vector q_torso(icub.lowerTorso->getNLinks("torso"));		q_torso.zero();
	Vector q_lleg(icub.lowerTorso->getNLinks("left_leg"));		q_lleg.zero();
	Vector q_rleg(icub.lowerTorso->getNLinks("right_leg"));		q_rleg.zero();
	
	// here we set the inertial sensor (head) measurements
	Vector w0(3); Vector dw0(3); Vector ddp0(3);
	w0=dw0=ddp0=0.0; ddp0[2]=9.81;

	// here we set the external forces at the end of each limb
	Matrix FM_up(6,3); FM_up.zero();
	Matrix FM_lo(6,2); FM_lo.zero();

	// here we set the joints position, velocity and acceleration 
	// for all the limbs!
	icub.upperTorso->setAng("head",q_head);
	icub.upperTorso->setAng("right_arm",q_rarm);
	icub.upperTorso->setAng("left_arm",q_larm);
	//
	icub.upperTorso->setDAng("head",q_head);
	icub.upperTorso->setDAng("right_arm",q_rarm);
	icub.upperTorso->setDAng("left_arm",q_larm);
	//
	icub.upperTorso->setD2Ang("head",q_head);
	icub.upperTorso->setD2Ang("right_arm",q_rarm);
	icub.upperTorso->setD2Ang("left_arm",q_larm);
	//
	icub.lowerTorso->setAng("torso",q_torso);
	icub.lowerTorso->setAng("right_leg",q_rleg);
	icub.lowerTorso->setAng("left_leg",q_lleg);
	//
	icub.lowerTorso->setDAng("torso",q_torso);
	icub.lowerTorso->setDAng("right_leg",q_rleg);
	icub.lowerTorso->setDAng("left_leg",q_lleg);
	//
	icub.lowerTorso->setD2Ang("torso",q_torso);
	icub.lowerTorso->setD2Ang("right_leg",q_rleg);
	icub.lowerTorso->setD2Ang("left_leg",q_lleg);
	

	// initialize the head with the kinematic measures retrieved 
	// by the inertial sensor on the head
	icub.upperTorso->setInertialMeasure(w0,dw0,ddp0);
	
	// solve the kinematic of all the limbs attached to the upper torso
	// i.e. head, right arm and left arm
	// then solve the wrench phase, where the external wrenches are set
	// to be acting only on the end-effector of each limb
	// then the limbs with a FT sensor (arms) estimate the sensor wrench
	// note: FM_up = 6x3, because we have 3 limbs with external wrench input
	// note: afterAttach=false is default
	Matrix fm_sens_up = icub.upperTorso->estimateSensorsWrench(FM_up);
	
	// connect upper and lower torso: they exchange kinematic and wrench information
	icub.attachTorso();
	
	// now solve lower torso the same way of the upper
	// note: FM_lo = 6x2, because we have 2 limbs with external wrench input to be set
	// since the torso receives the external wrench output from the upperTorso node
	// during the attachTorso()
	// note: afterAttach=false is set to true, because we specify that the torso
	// already received the wrench for initialization by the upperTorso during
	// the attachTorso()
	Matrix fm_sens_lo = icub.lowerTorso->estimateSensorsWrench(FM_lo,true);

	cout<<endl
		<<"Estimate FT sensor measurements on upper body: "<<endl
		<<" left  : "<<fm_sens_up.getCol(0).toString()<<endl
		<<" right : "<<fm_sens_up.getCol(1).toString()<<endl
		<<endl
		<<"Upper Torso: "<<endl
		<<" kin :   w= "<<icub.upperTorso->getTorsoAngVel().toString()<<endl
		<<"        dw= "<<icub.upperTorso->getTorsoAngAcc().toString()<<endl
		<<"       ddp= "<<icub.upperTorso->getTorsoLinAcc().toString()<<endl
		<<" wre :   F= "<<icub.upperTorso->getTorsoForce().toString()<<endl
		<<"        Mu= "<<icub.upperTorso->getTorsoMoment().toString()<<endl
		<<endl
		<<"Estimate FT sensor measurements on lower body: "<<endl
		<<" left  : "<<fm_sens_lo.getCol(0).toString()<<endl
		<<" right : "<<fm_sens_lo.getCol(1).toString()<<endl
		<<endl
		<<"Lower Torso: "<<endl
		<<" kin :   w= "<<icub.lowerTorso->getTorsoAngVel().toString()<<endl
		<<"        dw= "<<icub.lowerTorso->getTorsoAngAcc().toString()<<endl
		<<"       ddp= "<<icub.lowerTorso->getTorsoLinAcc().toString()<<endl
		<<" wre :   F= "<<icub.lowerTorso->getTorsoForce().toString()<<endl
		<<"        Mu= "<<icub.lowerTorso->getTorsoMoment().toString()<<endl
		<<endl;

	cin.get();
    return 0;
}
      
      
