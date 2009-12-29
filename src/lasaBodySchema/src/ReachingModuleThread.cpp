// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Micha Hersch
 * email:   micha.hersch@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#define ICUB_IS_HERE

//#define NO_OUTPUT_CONNECTION

#define RANDOM_TARGETS

#include <ace/OS_NS_stdio.h>
#include "ReachingModuleThread.h"
#include "KChainOrientationBodySchema.h"


//#include <ace/OS.h>

int cartesian_dim=0;
int joint_angle_dim=0;


ReachingThread::ReachingThread(int th_period):RateThread(th_period){
  jointMapping = NULL;
  bodydata = NULL;
#ifdef WITH_HEAD
  head = NULL;
#endif
  listen_cnt = 0;
  //  cnt = 1; //for debugging

}


ReachingThread::~ReachingThread(){
    if(jointMapping) delete[] jointMapping;
    if(bodydata) delete[] bodydata;
    cout<<"thread deleted"<<endl;
}
                         


bool ReachingThread::init(Searchable &s){
  char tmp1[100],tmp2[100];
  Value& struc = s.find("structure");
  if(struc.isNull()){
    ACE_OS::printf("no robot structure file given\n");
    return false;
  }

  body = new KinematicTree(struc.asString().c_str());


  int telesc = s.check("pointing");
  Value& opt = s.find("orientation");
  if(opt.isNull()){
      cartesian_dim = 3;
    arm = new KChainBodySchema(telesc);
    ((KChainBodySchema *)arm)->SetTolerance(100);
    // cout<<"orientation is discarded"<<endl;
  }
  else{
    cartesian_dim =6;
    arm = new KChainOrientationBodySchema(telesc);
    ((KChainOrientationBodySchema *)arm)->SetTolerance(100);
    //     cout<<"orientation is considered"<<endl;
  }
 
  simulation = s.check("simulation");
  Value& from = s.find("from");
  Value& to = s.find("to");
  if(! from.isNull() && ! to.isNull()){
      if(!body->LoadChain(from.asString().c_str(),to.asString().c_str(),arm)){
          cout<<"cannot load chain going from "<<from.asString().c_str()<<" to "<<to.asString().c_str()<<endl;
      }
  }
  else{// default values
      body->LoadChain("r_sfe","r_hand",arm);
  }
    cout<<arm<<endl;

//     if(!opt.isNull()){
//         if(opt.asInt() == 1){//not yet implemented
//             ((KChainOrientationBodySchema *)arm)->AddVirtualRotationAxis();
//         }
//     }
  nbDOFs=joint_angle_dim = arm->GetNbJoints();
  arm->UpdateDimension();

  joint_vec_t low_range, up_range;
  body->GetArticulatedTree()->FindAngleRanges(low_range.GetArray(),up_range.GetArray(),(KinematicChain *)arm);
  arm->SetAngleBounds(low_range,up_range);



  cout<<"found "<<joint_angle_dim<<" joints"<<endl;
  // arm->Print(cout);
  // ! important to do this after joint_angle_dim and cartesian_dim have been set  
  reach = new Reaching(); 
  reach->SetBodySchema(arm);
  cout<<"reaching ok"<<endl;
  //  reach->MatchToBodySchema();
  // reach->SetStill();

  if (s.check("jointControl")){
      reach->PureJointControl();

  }

  if(s.check("pointing")){
      pointing = true;
      arm->SetIkTrials(1);
  }
  else{
      pointing = false;
  }
  active_mode =true;

  Value& part = s.find("part");
  if(!part.isNull()){
      strcpy(partname,part.asString().c_str());
  }
  else{
       strcpy(partname,"right_arm");
  }


  Value& gran = s.find("granularity");
  if(gran.isNull()){
    time_granularity = 10;
  }
  else{
    time_granularity = gran.asDouble();
  }

  initMapping(s);
  
  //opening intput ports
  sprintf(tmp1,"%s",getName("target:i").c_str());
  if(!targetPort.open(tmp1)){
      ACE_OS::printf("Cannot open port %s\n",tmp1);
  }

   sprintf(tmp1,"%s",getName("body_schema:i").c_str());
   if(!bodyPort.open(tmp1)){
       ACE_OS::printf("Cannot open port %s\n",tmp1);
   }

   sprintf(tmp1,"%s",getName("robotState:i").c_str());
   if(!robotJointPort.open(tmp1)){
       ACE_OS::printf("Cannot open port %s\n",tmp1);
   }
   sprintf(tmp1,"%s",getName("cmd:i").c_str());
   if(!cmdPort.open(tmp1)){
       ACE_OS::printf("Cannot open port %s\n",tmp1);
   }



   cout<<"body size ... "<<endl;
   body_data_size = body->GetTreeSize()*6;
   bodydata = new float[body_data_size];

   //opening output ports
   cout<<"mapping initialized"<<endl;

  ///connection to the thread
   sprintf(tmp1,"%s",getName("vc_command:o").c_str());
   if(!vcFastCommand_port.open(tmp1)){
    ACE_OS::printf("Cannot open port %s\n",tmp1);
    return false;
  }
  


#ifdef ICUB_IS_HERE
   if(! simulation){
   Property options(s.toString());

  ////////////////////////////////////////////////////////////////////
  ////Getting access to the Polydriver of the arm  (taken from the drumming)
  ////////////////////////////////////////////////////////////////////
  ConstString partName(partname);
  Property ddOptions;
 
  /*  
  ddOptions.put("robot","icub");
  ddOptions.put("device","remote_controlboard");
  sprintf(tmp1,getName("encoders/in"));
  sprintf(tmp2,"/icub/%s",partName.c_str());
  ddOptions.put("local",tmp1);
  ddOptions.put("remote",tmp2);

  ddPart = new PolyDriver(ddOptions);

  if(!ddPart->isValid())
    {
      ACE_OS::printf("Device not available. Here are the known devices:\n");
      ACE_OS::printf("%s", Drivers::factory().toString().c_str());
      return false;
    }

  ///encoders interface
  if(!ddPart->view(PartEncoders))
    {
      ACE_OS::printf("Cannot view the encoders interface of %s\n",partName.c_str());
      return false;
    }
  
  */
 ///////////////////////////////////////////////////////////////////////
  ////////Connection to the velocity control module (taken from drumming)
  ///////////////////////////////////////////////////////////////////////

  ///normal connection
#ifndef NO_OUTPUT_CONNECTION
  sprintf(tmp2,"/icub/vc/%s/fastCommand",partName.c_str());
    if(!Network::connect(tmp1,tmp2,"udp")){
    ACE_OS::printf("Cannot connect port %s to port %s\n",tmp1,tmp2);
    return false;
  }
#endif    

  sprintf(tmp1,"%s",getName("vc_parameters/out").c_str());
  if(!vcControl_port.open(tmp1)){
      ACE_OS::printf("Cannot open vcControl port of %s\n",partName.c_str());
      return false;
    }
      


#ifndef NO_OUTPUT_CONNECTION  
  sprintf(tmp2,"/icub/vc/%s/input",partName.c_str());
  if(!Network::connect(tmp1,tmp2)){
      ACE_OS::printf("Cannot connect port % to port %s\n",tmp1,tmp2);
      return false;
    }
#endif
    if(options.check("nbDOFs"))
        nbDOFs = options.find("nbDOFs").asInt();
    else{
        ACE_OS::printf("Please specify the nbDOFs of part%s\n",partName.c_str());
        return false;
    }

    //specifying the gains (also taken from the drumming) 
     ///reading the Kp gains in the conf file
    if(options.check("controlGains"))
        {
            Bottle& botG = options.findGroup("controlGains");
            if(botG.size()!=nbDOFs+1)
                ACE_OS::printf("wrong number of gains\n");
            else{
                for(int i=0;i<nbDOFs;i++){
                    double gain = botG.get(i+1).asDouble();
                    Bottle& cmd = vcControl_port.prepare();
                    cmd.clear();
                    cmd.addVocab(Vocab::encode("gain"));
                    cmd.addInt(jointMapping[i]);
                    cmd.addDouble(gain);
                    vcControl_port.write(true);
                    Time::delay(0.1);
                }
            }
        }
    else{
        ACE_OS::printf("no gains defined, using 0\n");
    }
    if(options.check("maxVelocity")){
      Bottle& mv = options.findGroup("maxVelocity");
      if(mv.size()!=nbDOFs+1)
          ACE_OS::printf("wrong number of max velocity\n");
      else{
          for(int i=0;i<nbDOFs;i++){
              double vel = mv.get(i+1).asDouble();
              Bottle& cmd = vcControl_port.prepare();
              cmd.clear();
              cmd.addVocab(Vocab::encode("svel"));
              cmd.addInt(jointMapping[i]);
              cmd.addDouble(vel);
              vcControl_port.write(true);
              Time::delay(0.1);
          }
      }
    }    
    else{
        ACE_OS::printf("no max velocity defined, using default\n");
    }
   }
#else

  Value& dest = s.find("destination");
  if(!dest.isNull()){
    strcpy(tmp1,vcFastCommand_port.getName().c_str());
    strcpy(tmp2,dest.asString().c_str());
    if(!Network::connect(tmp1,tmp2)){
      ACE_OS::printf("Cannot connect port %s to port %s\n",tmp1,tmp2);
      return false;
    }
  }
#endif

  
  // updating the posture with the actual one


  if(!simulation){
      if(!matchBodySchemaToRobot()){
          return false;
      }
  }
  else{
      reach->RandomTarget();
  }

  

  reach->MatchToBodySchema();

  
  // measuring time
  original_time = Time::now();
  time_last = 0;
  time_now = time_last;
  cout<<"initialization done"<<endl;;
  return true;
}

bool ReachingThread::matchBodySchemaToRobot(){
    joint_vec_t angles;
    if(Network::connect("/icub/right_arm/state:o",
                        robotJointPort.getName().c_str())){
        if(robotJointPort.ReadPosition(angles.GetArray(),nbDOFs,true)){
            cout<<"current position "<<endl;
               angles.Print();   
               angles *= deg2rad;
               arm->SetPosition(angles);
               reach->SetTargetAngle(angles);
               return true;
           }
    }
    else{
      ACE_OS::printf("cannot read robot state\n");;    
    }
    return false;
}

//this function is now obsolete 
bool ReachingThread::initMapping(Searchable& s){
  Value& mapping = s.find("mapping");
  //  cout<<"looking for nb joints"<<endl;
  int n=arm->GetNbJoints();
  cout<<"found "<<n<<" joints"<<endl;
  if(mapping.isNull()){
    mappingSize = n;
    jointMapping = new int[mappingSize];  
    for(int i=0;i<n;i++){
      jointMapping[i]=i;
    }
    return false;
  }
  else{
    Bottle *b =mapping.asList();
    int ms = b->size();
    mappingSize = min(ms,n);
    if(ms != n){
      ACE_OS::printf("warning: mapping size %d is not equal to joint size %d",ms,n);
    }
    jointMapping = new int[mappingSize];  
    for(int i=0;mappingSize;i++){
      jointMapping[i] = b->get(i).asInt();
    }
    return true;
  }
}



ConstString ReachingThread::getName(const char *sub){
  if(module)return module->getName(sub); 
  else{ConstString s("/reaching"); return s;}
}



bool ReachingThread::listen(){
    cart_vec_t tar;
    joint_vec_t head_angles;
    int new_t,n;
 

  //listening for body schema
  n = body->GetTreeSize();
  if(bodyPort.ReadBodySchema(bodydata,body_data_size)){//+3 for last link
      cout<<"got schema"<<endl;
      body->GetArticulatedTree()->Deserialize(bodydata,body_data_size);
      new_schema=1;
  }

  //listening to target
  if(targetPort.getInputCount()>0){
      if(cartesian_dim == 3){
          new_t = (int) targetPort.ReadPosition(tar.GetArray());
      }
      else{
          new_t = targetPort.ReadPositionAndOrientation(tar.GetArray(),tar.GetArray()+3);
      }

    if(new_t){
        reach->SetLocalTarget(tar,!pointing);
        joint_vec_t ta;
        reach->GetTargetAngle(ta);
        ta.Print();
    }
  }
  else{// we generate the targets
#ifdef RANDOM_TARGETS
      if(reach->TargetReached() || !(rand()%1000)){
          reach->RandomTarget(tar,!pointing);
          if(tar.Size()==6){
              targetPort.SendPositionAndOrientation(tar.GetArray(), tar.GetArray()+3);
          }
          else{
              targetPort.SendPosition(tar.GetArray());
          }
      }
#endif
  }
  return true;
}
  


void ReachingThread::sendAnglesFormatVC(joint_vec_t& angles){
   Bottle& cmd = vcFastCommand_port.prepare();
   cmd.clear();
   for(int i=0;i<mappingSize;i++){
// #ifdef ICUB_IS_HERE not necessary anymore 
//        cmd.addInt(jointMapping[i]);
// #endif 
       cmd.addDouble(angles[i]*180/PIf);
   }
   vcFastCommand_port.write(true);
   //   cout<<cmd.toString().c_str()<<endl;
}

bool ReachingThread::sendOutput(){
 joint_vec_t angles;
 // body->GetAngles(angles);
 reach->GetAnglePosition(angles);
 // reach->GetTargetAngle(angles);
 sendAnglesFormatVC(angles);
 
 return true;
}

void ReachingThread::run(){
    Time::delay(0.0001);
    //  cout<<"start"<<endl;
  time_now = Time::now()-original_time;
  if(cmdPort.ShouldQuit()){
      Bottle cmd("quit");
      Bottle reply;
      module->safeRespond(cmd,reply);
  }
  //  cout<<"listen..."<<endl;
  if(++listen_cnt == 20){
      listen();
      listen_cnt=0;
  }
  if(active_mode){
    float integration_time = (time_now-time_last)*1000 - 0.5*time_granularity; //
    for(float tim=0; tim<integration_time; tim+=time_granularity){
        switch(new_schema){ // to avoid jumps due to high cartesian velocity because of new schema
        case 0: break;
        case 1: 
        case 2: reach->PureJointControl();new_schema++;break;
        case 3: reach->HybridControl();new_schema=0;break;
        } 
        reach->ReachingStep(time_granularity);
        // reach->ReachingStep();
    }
  }
    sendOutput();
    time_last = time_now;
    //   cout<<"stop"<<endl;
    //   if(!(cnt--))module->close();//for debugging
}

bool ReachingThread::threadInit(){
  return true;
}


void ReachingThread::setZeroGains(){
 for(int i=0;i<nbDOFs;i++)
	    {
            Bottle& cmd = vcControl_port.prepare();
            cmd.clear();
            cmd.addVocab(Vocab::encode("gain"));
            cmd.addInt(jointMapping[i]);
            cmd.addDouble(0.0);
            vcControl_port.write(true);
            Time::delay(0.1);
	    }

    Bottle& cmd = vcControl_port.prepare();
    cmd.clear();
    cmd.addVocab(Vocab::encode("susp"));
    vcControl_port.write(true);
}


void ReachingThread::threadRelease(){
    if(!simulation){
        setZeroGains();
    }
    //closing all ports
     targetPort.close();
     bodyPort.close();
     robotJointPort.close();
     vcFastCommand_port.close();
     vcControl_port.close();
     cmdPort.close();

    //deallocating memory
    if(jointMapping){delete [] jointMapping;}
    delete body;
    delete arm;
    delete[] bodydata;



}

ReachingModule::ReachingModule():Module(){}



bool ReachingModule::open(Searchable &s){
  m_thread.setModule(this);  
  if(m_thread.init(s)){
    cout<<"starting thread"<<endl;
    return m_thread.start();
  }
  return false;
} 
//  // Time::TurboBoost() not yet
//   bool ok=true;
//   active_mode = true;
//   exit = false;
//   ok *= outport.SetPortName("/proprio");
//   ok *= outport.Start();
//   ok *= targetport.SetPortName("/target/out");
//   ok *= targetport.Start();
//   //  ok *= cmdport.Start();
//   body = new KChainBodySchema();
//   body->Load(body_fname);
//   body->Print(cout);
//   reach->SetBodySchema(body);
//   reach->MatchToBodySchema();
//   reach->RandomTarget();
  
//   if(ok){
//     return start();
//   }
//   else{
//     return false;
//   }
// }



//void ReachingModule::listen(){
//   Bottle *b =cmdport.read(false);
//   if(b){
//     int cmd = b->get(0).asInt();
//     switch(cmd){
//     case ReachCmd_Active:
//       active_mode = true;
//       break;
//     case ReachCmd_Passive:
//       active_mode = false;
//       break;
//     case ReachCmd_SetPosition:
//       break;

//     }
//   }
//}


// void ReachingModule::sendOutput(){
//   joint_vec_t angles;
//   body->GetAngles(angles);
//   // reach->GetTargetAngle(angles);
//   outport.SendFeedback(angles.GetArray(), body->GetNbJoints());
// }

// void ReachingModule::run(){
//   time_now = Time:now();
//   listen();
//   if(reach->TargetReached() || !(rand()%100000)){
//     cart_vec_t tar;
//     reach->RandomTarget(tar);
//     targetport.SendPosition(tar[0],tar[1],tar[2]);
//   }
//   if(active_mode){
//     reach->ReachingStep(time_now-time_last);
//   }
//     sendOutput();
//     time_last = time_now;
// }

bool ReachingModule::close(){
  m_thread.stop();
  return true;
}



int usage(){
  cout<<"usage: reaching_module [--file <config_file> | --structure <structure_file> ";
  cout<<"[--orientation 1] [--name <port_base_name>]] [--jointControl] [--granularity <granularity>]"<<endl;
  cout<<"<config_file>: standard icub module configuration file containing the module options"<<endl;
  cout<<"<structure_file>: xml file describing the geometry of the manipulator ";
  cout<<"(using home-made convention, not D.H.)"<<endl;
  cout<<"--jointControl: if you want a pure angular controller"<<endl;
  cout<<"<granularity>: the integration constant of the dynamical system in [ms], default value=10"<<endl;
  cout<<"--orientation 1: if you want to specify the target orientation as well as position"<<endl;
  return 1;
}

int main(int argc, char *argv[]){
  ReachingModule rmod;
  Property prop;
  if(argc==1){
    return usage();
  }
  prop.fromCommand(argc,argv);
  if(prop.check("help")){
    return usage();
  }
  rmod.runModule(argc,argv);
  cout<<"closing"<<endl;
}
