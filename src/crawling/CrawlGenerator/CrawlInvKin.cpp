#include "CrawlInvKin.h"


//constraints on the shoulder, taken from Ugo's code!
iCubShoulderConstr::iCubShoulderConstr(unsigned int dofs, double lower_bound_inf,
                                       double upper_bound_inf) :
                                      iKinLinIneqConstr(lower_bound_inf,upper_bound_inf)
{
    unsigned int offs=dofs<=7 ? 0 : 3;

    double joint1_0= 10.0*(M_PI/180.0);
    double joint1_1= 15.0*(M_PI/180.0);
    double joint2_0=-33.0*(M_PI/180.0);
    double joint2_1= 60.0*(M_PI/180.0);
    double m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    double n=joint1_0-m*joint2_0;    

    // Linear inequalities matrix
    C.resize(5,dofs); C.zero();
    // constraints on the cables length
    C(0,offs)=1.71; C(0,offs+1)=-1.71;
    C(1,offs)=1.71; C(1,offs+1)=-1.71; C(1,offs+2)=-1.71;
                    C(2,offs+1)=1.0;   C(2,offs+2)=1.0;
    // constraints to prevent arm from touching torso
                    C(3,offs+1)=1.0;   C(3,offs+2)=-m;
    // constraints to limit shoulder abduction
                    C(4, offs+1)=1.0;

    // lower and upper bounds
    lB.resize(5); uB.resize(5);
    lB[0]= -347.0*(M_PI/180.0); uB[0]=upperBoundInf;
    lB[1]=-366.57*(M_PI/180.0); uB[1]=112.42*(M_PI/180.0);
    lB[2]=  -66.6*(M_PI/180.0); uB[2]= 213.3*(M_PI/180.0);
    lB[3]=n;                    uB[3]=upperBoundInf;
    lB[4]=lowerBoundInf;        uB[4]=SHOULDER_MAXABDUCTION;
}


/********************************************
 * IK CLASS
 * ****************************************/

IKManager::IKManager()
{  
  this->rate = ((double)rate)/1000.0;
  
  leftArm = new iCubArm("left");
  rightArm = new iCubArm("right");
  leftLeg = new iCubLeg("left");		
  rightLeg = new iCubLeg("right");

  //the left arm is controlling the torso 
  leftArm->releaseLink(0);
  leftArm->releaseLink(1);
  leftArm->releaseLink(2); 
  
  legLength = 0.23;
  
  dShoulder = 0.110; //distance from middle torso to shoulder in coronal plane (x)
  dHip = 0.068;
  DShoulder = 0.143; ////distance from middle torso to shoulder in sagittal plane (y)
  DHip = 0.088;    
}

IKManager::~IKManager()
{
	delete chain;
    delete leftArm;
    delete rightArm;
    delete leftLeg;
    delete rightLeg;	
}

/* mvt_parameters 

0 left_arm amplitude
1 left_arm target
2 right_arm amplitude
3 right_arm target
   0 shoulder pitch
   1 shoulder roll 
   2 shoulder yaw
   3 elbow
   4 forearm
   5 wrist pitch
   6 wrist roll
 
4 left_leg amplitude
5 left_leg target
6 right_leg amplitude
7 right_leg target
   0 hip pitch
   1 hip roll 
   2 hip yaw
   3 knee
   4 ankle pitch
   5 ankle yaw
   
8 torso amplitude
9 torso target
   0 torso pitch
   1 torso roll
   2 torso yaw
    
10 head amplitude
11 head target
   0 head pitch
   1 head yaw
   2 head roll

*  limb parameters:  
  thigh = 0.2236;
  calf = 0.213; 
*/


double IKManager::getArmAmplitude(double* positions, double leg_amplitude)
{     
    iKinChain *armChain, *legChain; 
    //we simply solve: r_l*w_l = r_a*w_a 
    //radius*amplitude of arm and leg should be the equal to have approx the same step length
	
	//legChain=leftLeg->asChain();
    Vector pose_q, pose_x0;
    
    //for(int i=0; i<4; i++)
    //{
    	//pose_q.push_back(position[i]);
	//}
    //pose_q.push_back(-0.7);
    //pose_q.push_back(0.0);
	
	//legChain->setAng(pose_q);
	//pose_x0=legChain->Pose(2);
    //double legLength2=0;
    //for(int i=0;i<3;i++) legLength2=pose_x0[i]*pose_x0[i];
    //double legLength=sqrt(legLength2);
		
	//pose_q.clear();
	//pose_x0.clear();
	//legChain->clear();

    armChain=leftArm->asChain();    
    for(int i=0; i<4; i++)
    {
    	pose_q.push_back(positions[i]);
    }
    pose_q.push_back(-1.57);
    pose_q.push_back(-1.0);
    pose_q.push_back(0.1);

    armChain->setAng(pose_q);
    pose_x0=armChain->Pose(5);
    double armLength2=0;
    for(int i=0;i<3;i++) armLength2+=pose_x0[i]*pose_x0[i];
    double armLength=sqrt(armLength2);
    
    ACE_OS::printf("leg length %f, arm length %f\n", legLength, armLength);
    double arm_amplitude = legLength*leg_amplitude/armLength;
    
    pose_q.clear();
    pose_x0.clear();
    armChain->clear();
    
	return arm_amplitude;
}
double IKManager::getTurnParams(double turn_angle, double amplitude, int side, int limb)
{ 
  double final_amplitude;
  double c, alpha, beta, Gamma, B, R, deltaLeg, deltaArm;
  
  double gamma = M_PI-turn_angle;
  ACE_OS::printf("turn angle %f\n", turn_angle);
  c = sqrt(DShoulder*DShoulder + DHip*DHip -2*DHip*DShoulder*cos(gamma));
  double temp = (DShoulder*DShoulder-c*c-DHip*DHip)/(2*c*DHip);
  
  if(fabs(temp)>1)
  {
      ACE_OS::printf("error while computing amplitudes\n");
      return amplitude;
  }
  
  
  alpha = acos(temp);  
  beta = M_PI - alpha -gamma;
  
  printf("c %f alpha %f beta %f\n", c, alpha, beta);

  deltaLeg=dHip*sin(alpha); 
  deltaArm=dShoulder*sin(beta);

        
  int turnWhere=0;
  turn_angle >0 ? turnWhere = 1 : turnWhere = -1; // 1: left, -1:right
  
  double distLimb = 0;
  limb == 1 ? distLimb=dShoulder : distLimb=dHip; //we don't care about the torso and head  
  
  double grand, small, rgrand, rsmall;
  
  grand=c+deltaLeg+deltaArm;
  small=c-deltaLeg-deltaArm;
  
  Gamma=M_PI-2*alpha;
  R= fabs(c/2/cos(M_PI/2-alpha));
  
  
  rgrand=(R+distLimb)/R;
  rsmall=(R-distLimb)/R;
  //rgrand = grand/c;
  //rsmall=small/c;
  
  ACE_OS::printf("variables are rgrand %f rsmall %f\n", rgrand, rsmall);
  
  //radius of curvature
  //R = B*tan(Gamma/2);
  
  ////adjustment of the hands' position 
  //delta = M_PI-gamma; 
  //delta = (double) fmod((float) delta, (float)2*M_PI); 
  
  //ACE_OS::printf("delta: %f, R: %f\n", delta, R);


  switch(side)
  {    
    case 0:
    
        final_amplitude = amplitude;
        ACE_OS::printf("Nothing to compute (torso or head)\n");
        
        break;
    
    case 1:  //left
    
        if(turnWhere==1)//left
        {
            final_amplitude=amplitude*rsmall;
            ACE_OS::printf("Left side turning left, ");
        }
        if(turnWhere==-1)//right
        {
            final_amplitude=amplitude*rgrand;
            ACE_OS::printf("Left side turning right,  ");
        }
        
        break;
    
    case 2://right 
    
        if(turnWhere==1)
        {
            final_amplitude=amplitude*rgrand;
            ACE_OS::printf("Right side turning left, ");
        }
        if(turnWhere==-1)
        {
            final_amplitude=amplitude*rsmall;
            ACE_OS::printf("Right side turning right,  ");
        }
        
        break;
  }
  
  ACE_OS::printf("final amplitude: %f\n", final_amplitude);
 
  return final_amplitude;
}




