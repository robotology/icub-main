#include "moduleHand.hpp"

#include <unistd.h>
#include <sstream>
#include <iomanip>


#include <yarp/os/Network.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


//MAKE_MODULE_DLL_INTERFACE(iqrcommon::moduleYarped,"First IQR Yarp Module")

moduleHand::moduleHand()  {
  // tid = 0;
  
  //handEncs = addOutputToGroup("Hand Encoders", "Hand Encoders");
  // handCmds = addInputFromGroup("Hand Motors Commands", "Hand Motors Commands");	
}

moduleHand::~moduleHand(){ 
}

void moduleHand::init(){
    Network::init();

    Property optHand("(device remote_controlboard) (remote /icub/right_arm) (local /grab/hand)");

    drvHand=new PolyDriver(optHand);

    if (drvHand->isValid())
    {
        drvHand->view(limHand);
    	drvHand->view(encHand);
	drvHand->view(posHand);

        encHand->getAxes(&nAxes);

        min.resize(nAxes);
        max.resize(nAxes);

        for (int i=0; i<nAxes; i++)
        {
            limHand->getLimits(i,&min[i],&max[i]);
            //cout<<"#"<<i<<": ("<<min[i]<<","<<max[i]<<")"<<endl;
        }

        Vector speed(nAxes);
        speed=10; // [deg/s]
        posHand->setRefSpeeds(speed.data());
    }
    else
    {
      //cerr << "Device drivers not available! => ARGHing" << endl;
    }
}
/*
yarp::sig::Vector moduleHand::scaleFromRobot(const yarp::sig::Vector& v){
    Vector ret(nAxes);
    
    for (int i=0; i<nAxes; i++)
    {
        ret[i]=(v[i]-min[i])/(max[i]-min[i]);
        ret[i]=ret[i]<0.0?0.0:ret[i];
        ret[i]=ret[i]>1.0?1.0:ret[i];
    }    

    return ret;
}
*/
/*
yarp::sig::Vector iqrcommon::moduleYarped::scaleToRobot(const yarp::sig::Vector& v){
    Vector ret(nAxes);
    
    for (int i=0; i<nAxes; i++)
        ret[i]=v[i]*(max[i]-min[i])+min[i];   

    return ret;
}
*/
/*
void iqrcommon::moduleYarped::update(){
    Vector v1(nAxes);
    Vector v2(nAxes);
 
    qmutexThread->lock();
    encHead->getEncoders(v1.data());
    v2=scaleFromRobot(v1);

    //debug.beg
    cout<<"got: "<<v2.toString()<<endl;
    //debug.end

    StateArray &stateArrayIn = headEncs->getStateArray();
    for (int i=0; i<nAxes; i++)   
    	stateArrayIn[0][i]=v2[i];
    qmutexThread->unlock();

    qmutexThread->lock();
    StateArray &stateArrayOut = headCmds->getTarget()->getStateArray();
    for (int i=0; i<nAxes; i++)
    	v1[i]=stateArrayOut[0][i];
    v2=scaleToRobot(v1);
    posHead->positionMove(v2.data()); 

    //debug.beg
    cout<<"sent: "<<v1.toString()<<endl;
    //debug.end

    qmutexThread->unlock();

    usleep(1e6);

    //debug.beg
    cout<<endl<<endl;
    //debug.end    
}
*/

bool moduleHand::okToThrowPosition(){
   Vector v1(nAxes);
   Vector v2(nAxes);
 
   
    encHand->getEncoders(v1.data());
    v2=v1;

    // check if grab done
    float eps = 5;
    if ((fabs(v2[15] - 69) < eps) && (fabs(v2[14] - 50) < eps) &&(fabs(v2[13] - 21) < eps) &&(fabs(v2[12] - 44) < eps) && (fabs(v2[11] - 37) < eps)&& (fabs(v2[10] - 21) < eps) && (fabs(v2[9] - 13) < eps) &&  (fabs(v2[8] - 69) < eps)){

      // go to ball delivery position
      
      return true;
    }
    return false;

    
}

void moduleHand::grab(){
   Vector v1(nAxes);
   Vector v2(nAxes);
 
   
    encHand->getEncoders(v1.data());
    v2=v1;

    v2[15] = 69;
    v2[14] = 50;
    v2[13] = 21;
    v2[12] = 44;
    v2[11] = 37;  
    v2[10] = 21; 
    v2[9] = 13; 
    v2[8] = 69; 
    v2[4] = -90; 


    // be happy
    system("echo 'set all hap' | yarp rpc /icub/face/emotions/in");
    system("echo 'set all hap' | yarp rpc /icub/face/emotions/in");

    posHand->positionMove(v2.data());
  
}


bool moduleHand::checkBlockHand(){
  Vector v1(nAxes);
  Vector v2(nAxes);
 
   
    encHand->getEncoders(v1.data());
    v2=v1;


  float eps = 10;
   if ((fabs(v2[15] - 0) < eps) && (fabs(v2[14] - 0) < eps) &&(fabs(v2[13] - 0) < eps) &&(fabs(v2[12] - 0) < eps) && (fabs(v2[11] - 0) < eps)&& (fabs(v2[10] - 0) < eps) && (fabs(v2[9] - 0) < eps) &&  (fabs(v2[8] - 0) < eps) ){
     return false;

   }
   else{
     return true;
   }
}

void moduleHand::release(){

  
  
   Vector v1(nAxes);
   Vector v2(nAxes);
 
   
    encHand->getEncoders(v1.data());
    v2=v1;

    //float eps = 10;
    //if ((fabs(v2[0] + 85) < eps) && (fabs(v2[1] - 33) < eps) && (fabs(v2[2] - 36) < eps) && (fabs(v2[3] - 34) < eps)){
       
      v2[15] = 0;
      v2[14] = 0;
      v2[13] = 0;
      v2[12] = 0;
      v2[11] = 0;  
      v2[10] = 0; 
      v2[9] = 0; 
      v2[8] = 0; 
      v2[4] = 0; 


      // be angry
      system("echo 'set all evi' | yarp rpc /icub/face/emotions/in");
      system("echo 'set all evi' | yarp rpc /icub/face/emotions/in");

      posHand->positionMove(v2.data());
      
      //}


   
}



void moduleHand::cleanup(){
    delete drvHand;

    Network::fini();
}
