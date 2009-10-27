#include "CrawlInvKin.h"

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
  DShoulder = 0.143; //distance from middle torso to shoulder in sagittal plane (y)
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

double IKManager::getArmAmplitude(double* positions, double leg_amplitude)
{     
    iKinChain *armChain, *legChain; 
    //we simply solve: r_l*w_l = r_a*w_a 
    //radius*amplitude of arm and leg should be the equal to have approx the same step length
    
    

    Vector pose_q, pose_x0, pose_x1;

    armChain=leftArm->asChain();    
    for(int i=0; i<4; i++)
    {
    	pose_q.push_back(positions[i]);
    }
    pose_q.push_back(-1.57);
    pose_q.push_back(-1.0);
    pose_q.push_back(0.1);

    armChain->setAng(pose_q);
    pose_x0=armChain->Pose(3);
    pose_x1=armChain->Pose(7);
    double armLength2=0;
    for(int i=0;i<3;i++) armLength2+=(pose_x0[i]-pose_x1[i])*(pose_x0[i]-pose_x1[i]);
    double armLength=sqrt(armLength2);
    
    //d=sqrt(l2-(L-r)2) r= long jambe, L=longueur bras, l=longueur torse 
    double d= sqrt(DShoulder*DShoulder-(armLength-legLength)*(armLength-legLength));
    //(x,y)=(legLength*cos(leg_amplitude/2),-d+sin(leg_amplitude/2)))
    double x=legLength*cos(leg_amplitude/2);
    double y=d-legLength*sin(leg_amplitude/2);
    double m=sqrt(x*x+y*y);
    double h=(armLength*armLength+legLength*legLength-m*m)/(2*DShoulder);
    double phi=acos(h/m);
    double psi=acos((DShoulder-h)/armLength);
    double nu=phi-acos(x/m);
    double theta=M_PI/2-nu-phi;
    
    ACE_OS::printf("d %f x %f y %f m %f h %f phi %f psi %f nu %f theta %f\n", d, x, y, m, h, phi, psi, nu, theta);
    
    //ACE_OS::printf("leg length %f, arm length %f\n", legLength, armLength);
    //double arm_amplitude = legLength*leg_amplitude/armLength;
    double arm_amplitude = -2*theta;
    
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
  turn_angle >0.0 ? turnWhere = 1 : turnWhere = -1; // 1: left, -1:right
  
  double distLimb = 0;
  limb == 1 ? distLimb=dShoulder : distLimb=dHip; //we don't care about the torso and head  
  
  double grand, small, rgrand, rsmall;
  
  grand=c+deltaLeg+deltaArm;
  small=c-deltaLeg-deltaArm;
  
  Gamma=M_PI-2*alpha;
  R= fabs(c/2/cos(M_PI/2-alpha));
  
  ACE_OS::printf("radius of curvature is %f\n", R);
    
  rgrand=(R+distLimb)/R;
  rsmall=(R-distLimb)/R;

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




