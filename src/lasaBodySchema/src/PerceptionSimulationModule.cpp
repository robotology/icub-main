// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C)
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * author: Micha Hersch
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
#include <ace/OS_NS_stdio.h>
#include "PerceptionSimulationModule.h"
//#include <ace/OS.h>

int cartesian_dim=0;
int joint_angle_dim =0;

PerceptionSimulationModule::PerceptionSimulationModule(){
  delay=0.2;
  last_time =0;
}
PerceptionSimulationModule::~PerceptionSimulationModule(){}

int PerceptionSimulationModule::loadChain(KinematicChain *chain,Searchable &s,
                                           const char *from, const char *to, 
                                           const char *from_def, const char *to_def){

    Value& vfrom =s.find(from);
    Value& vto = s.find(to);
    if(! vfrom.isNull() && ! vto.isNull()){
        if(!body->LoadChain(vfrom.asString().c_str(),vto.asString().c_str(),chain)){
            cout<<"cannot load chain going from "<<vfrom.asString().c_str()<<" to "
                <<vto.asString().c_str()<<endl;
        }
    }
    else{// default values
        body->LoadChain(from_def,to_def,chain);
    }
    cout<<"kinematic chain found ("<<chain->GetNbJoints()<<" joints)"<<endl;
    return chain->GetNbJoints();
}


bool PerceptionSimulationModule::open(Searchable &s){
 Value& struc = s.find("structure");
 if(struc.isNull()){
   ACE_OS::printf("no robot structure file given\n");
   return false;
 }
 cartesian_dim = 3; //to do before call to "new body"
 body = new KinematicTree(struc.asString().c_str());
 eyes_arm = body->GetChain(0);
 eye_world = body->GetChain(1);
 head = body->GetChain(2); 
 arm = body->GetChain(3); 

 if(!loadChain(head,s,"head_base","head_end","neck_tilt","r_eye")){
     head=NULL;
 }
 if(!loadChain(arm,s,"arm_base","arm_end","r_sfe","r_hand")){
     arm=NULL;
 }
 if(!loadChain(eye_world,s,"eye","world","neck_tilt","r_eye")){
     eye_world=NULL;
 }

 if(s.check("no_static")){
     eyes_arm=NULL;
 }
 else{
     if(!loadChain(eyes_arm,s,"stereo","marker","eyes","r_hand")){
         eyes_arm=NULL;
     }
 }

 if(arm){arm_angles.Resize(arm->GetNbJoints());}
 if(head){ head_angles.Resize(head->GetNbJoints());}
 position.Resize(cartesian_dim);

 // opening communication ports 

 if(!proprioception_head.open(getName("proprioception_head:o").c_str())){
   ACE_OS::printf("Cannot open port %s\n",getName("proprioception_head:o").c_str());
   return false;
 }
 if(!proprioception_arm.open(getName("proprioception_arm:o").c_str())){
   ACE_OS::printf("Cannot open port %s\n",getName("proprioception_arm:o").c_str());
   return false;
 }
 
 if(!efferent_head.open(getName("efferent_head:i").c_str())){
     ACE_OS::printf("Cannot open port %s\n",getName("efferent_head:i").c_str());
     return false;
 }
 efferent_head.setStrict(false);

 if(!efferent_arm.open(getName("efferent_arm:i").c_str())){
     ACE_OS::printf("Cannot open port %s\n",getName("efferent_arm:i").c_str());
     return false;
 }
 efferent_arm.setStrict(false);


 if(!vision.open(getName("vision:o").c_str())){
   ACE_OS::printf("Cannot open port %s\n",getName("vision:o").c_str());
   return false;
  }
 if(!visualRotation.open(getName("visual_rotation:o").c_str())){
   ACE_OS::printf("Cannot open port %s\n",getName("visual_rotation:o").c_str());
   return false;
  }

 if(!cmdPort.open(getName("cmd:i").c_str())){
   ACE_OS::printf("Cannot open port %s\n",getName("cmd:i").c_str());
   return false;
  }


//  vision.addOutput("/vision/in");
//  proprioception.addOutput("proprioception/in");

 Value& opt = s.find("delay");
 if(!opt.isNull()){
     delay = opt.asDouble();
 }

 
 return true;
}





bool PerceptionSimulationModule::updateModule(){
    
    Rotation nrot;
    int new_pos = 0;
 
    Time::delay(delay);
    if(cmdPort.ShouldQuit()){return false;}
    if(efferent_head.getInputCount()>0 && (efferent_arm.getInputCount()>0 || ! eyes_arm)){
        if(efferent_head.ReadPosition(head_angles.GetArray(),head_angles.Size())){
            head_angles *= PIf/180.0;
            head->SetAngles(head_angles.GetArray());
            new_pos =1;
        }
        if(efferent_arm.ReadPosition(arm_angles.GetArray(),arm_angles.Size())){
            arm_angles *= PIf/180.0;
            arm->SetAngles(arm_angles.GetArray());
        }       
    }
    else{
        body->GetArticulatedTree()->RandomAngle();
        head->GetAngles(head_angles.GetArray());
        arm->GetAngles(arm_angles.GetArray());
    }
    
    stamp.update();   
    //    cout<<"stamp "<<stamp.getTime()<<endl;
    if(new_pos){
        if(eye_world){
            eye_world->GlobalRotation(nrot);
            rot.InvertRotationAngle();
            rot = rot*nrot;
            visualRotation.SendRotation(rot.GetRotationParam(),last_time,stamp.getTime());
            rot = nrot;
        }
        
     if(eyes_arm){
         eyes_arm->ForwardKinematics(position.GetArray());
         //       cout<<"sending ";position.Print();
         vision.setEnvelope(stamp);
         vision.SendPosition(position.GetArray());    
     }
 
     head_angles *= 180.0/PIf;
     arm_angles *= 180.0/PIf;

     proprioception_head.setEnvelope(stamp);
     proprioception_head.SendPosition(head_angles.GetArray(),head_angles.Size());
     proprioception_arm.setEnvelope(stamp);   
     proprioception_arm.SendPosition(arm_angles.GetArray(),arm_angles.Size());
     last_time = stamp.getTime();
 //     cout<<"time "<<stamp.getTime()<<endl;
//      head_angles.Print();
//      arm_angles.Print();
//      position.Print();
         }
    
      return true;
}

bool PerceptionSimulationModule::close(){
  vision.close();
  visualRotation.close();
  proprioception_arm.close();
  proprioception_head.close();
  efferent_head.close();
  efferent_arm.close();
  cmdPort.close();
  delete body;
  return true;
}

// bool PerceptionSimulationModule::respond(const Bottle& command, Bottle& reply) {
//   switch (command.get(0).asVocab()){
//   case VOCAB1('+'):{
//     delay += 0.1;
//     // delay = max(0,delay);
//     reply.addVocab(Vocab::encode("delay"));
//     reply.addDouble(delay);
//     return true;
//   }
//   case VOCAB1('-'):{
//     delay -= 0.1;
//     // delay = max(0,delay);
//     reply.addVocab(Vocab::encode("delay"));
//     reply.addDouble(delay);
//     return true;
//   }
//   default:
//     reply.add("command not recognized");
//     return false;
//   }
//   return false;
// }

int usage(){ 
  cout<<"usage: perception_module [--file <config_file> | --structure <structure_file>] [--delay <delay[s]>]"<<endl;
  return 1;
}

int main(int argc, char *argv[]){
  PerceptionSimulationModule module; 
  Property prop;
  if(argc==1){
    return usage();
  }
  if(prop.check("help")){
    return usage();
  } 
  module.runModule(argc,argv);
}
