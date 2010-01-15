// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <ace/OS_NS_stdio.h>
#include "BodySchemaLearning.h"


int cartesian_dim=3;
int joint_angle_dim=0;




BodySchemaLearningModule::BodySchemaLearningModule(){
  update_cnt =0;
  body_data =NULL;
  outputFrequency = 50;
  outlier_thresh = 500;
  static_update= 0;
  rotation_update= 0;
}

int BodySchemaLearningModule::loadChain(KinematicChain *chain,Searchable &s,
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

bool BodySchemaLearningModule::open(Searchable& s){
  //  char tmp1[100],tmp2[100];

  //Loading the body schema structure
  Value& struc = s.find("structure");
  if(struc.isNull()){
    ACE_OS::printf("no robot structure file given\n");
    return false;
  }


  body = new KinematicTree(struc.asString().c_str());
  body_data_size = body->GetTreeSize()*6;
  body_data = new float[body_data_size];

  eyes_arm  = body->GetChain(0);
  eye_world = body->GetChain(1);
  head = body->GetChain(2); 
  arm = body->GetChain(3);

#define FOR_ERIC //who does not want to learn the head body schema
#ifdef FOR_ERIC  // quick and dirty before the demo
  string jnames[4] = {"neck_tilt","neck_swing","neck_pan","eye_tilt"};
  for(int i=0;i<4;i++){
      body->GetArticulatedTree()->FindJoint(jnames[i])->GetJoint()->NoLearning();
  }
#endif



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
 
 if(arm){arm_proprio.Resize(arm->GetNbJoints());}
 if(head){ head_proprio.Resize(head->GetNbJoints());}


  Value& buffer = s.find("bufferSize");
  int buff_size = buffer.isNull()?30:buffer.asInt();//30: default value
  head_buffer = new TimedBuffer(buff_size, head->GetNbJoints());
  arm_buffer = new TimedBuffer(buff_size,arm->GetNbJoints());



  if(s.check("random_head")){
      //      head->RandomAxes();
      head->SetRotationAxis(0,0.f,1.f,0.f);
      head->SetRotationAxis(2,1.f,0.f,0.f);
      head->SetRotationAxis(3,0.f,0.f,1.f);
  }


  Value& logfile = s.find("logfile");
  if(!logfile.isNull()){
      log.open(logfile.asString().c_str());
      logging = true;
      if(log.fail()){
          cout<<"cannot open file "<<logfile.asString().c_str()<<endl;
          return false;
      }
      if(s.check("catchup")){
          cout<<"catching up from "<<logfile.asString().c_str()<<endl;
          int ccnt=0;
          while(simulUpdate()){ccnt++;}
          log.seekp(0,ios_base::end);
          cout<<"loaded "<<ccnt<<" points"<<endl;
          
      }
  }
  else{
      logging = false;
  }


  if(s.check("simulation")){ //no port in that case     
      simul = true;
      return true;
  }
  else{
      simul = false;
  }

  Value& opt = s.find("frequency");
  if(! opt.isNull()){
    outputFrequency = opt.asInt();
  }

  opt = s.find("rate");
  if(! opt.isNull()){
    //   body.SetRate(opt.asDouble()); later
  }
  
  Value& thresh = s.find("threshold");
  if(!thresh.isNull()){
     outlier_thresh = thresh.asDouble();
     cout<<"threshold "<<outlier_thresh<<endl;
  }
  markerPosition.Zero();

  //Opening the communication ports
  if(!commands.open(getName("cmd:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("cmd:i").c_str());
     return false;
  }
    
  if(!bsPort.open(getName("body_schema:o").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("body_schema:o").c_str());
    return false;
  }

  if(!proprioception_arm.open(getName("proprioception_arm:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("proprioception_arm:i").c_str());
    return false;
  }


  if(!proprioception_head.open(getName("proprioception_head:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("proprioception_head:i").c_str());
    return false;
  }

  if(!vision.open(getName("vision:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("vision:i").c_str());
    return false;
  }

 if(!visual_rotation_port.open(getName("visual_rotation:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("visual_rotation:i").c_str());
    return false;
  }

     //sending out initial body schema
 Time::delay(5);
 body->GetArticulatedTree()->Serialize(body_data,body_data_size);
 bsPort.SendBodySchema(body_data,body_data_size);
 
 return true;
}


int BodySchemaLearningModule::listen(){
      Stamp stamp;
      if(proprioception_head.ReadPosition(head_proprio.GetArray(),head_proprio.Size())){
        if(proprioception_head.getEnvelope(stamp)){
            head_proprio *= PIf/180;
            head_buffer->add(head_proprio.GetArray(),stamp.getTime());
        }
        else{
            cout<<"cannot read head stamp"<<endl;
        }
    }
    
    if(proprioception_arm.ReadPosition(arm_proprio.GetArray(),arm_proprio.Size())){
        if(proprioception_arm.getEnvelope(stamp)){
            arm_proprio *= PIf/180;
            arm_buffer->add(arm_proprio.GetArray(),stamp.getTime());
        }
        else{
            cout<<"cannot read arm stamp"<<endl;
        }
    }
    return 1;
}

bool BodySchemaLearningModule::listenToStaticVision(){
    if(vision.ReadPosition(visualPosition.GetArray())){
        static_update=1;
        // get the time stamp
        //possibly do some conversion
        return true;
    }
    return false;
}


bool BodySchemaLearningModule::listenToVisualRotation(){
    if(visual_rotation_port.ReadRotation(seenRotation,&t0,&t1)){
        rotation_update=1;
        return true;
    }
    return false;
}

// #ifdef ICUB_IS_HERE
//        // float tmp= visualPosition.GetArray()[0];
// //        visualPosition.GetArray()[0]=-visualPosition.GetArray()[1];
// //        visualPosition.GetArray()[1]=-tmp;
// //        visualPosition.GetArray()[2]*=-1; 
//        float tmp= visualPosition.GetArray()[0];
//        visualPosition.GetArray()[0]=-visualPosition.GetArray()[1]+95;
//        visualPosition.GetArray()[1]=tmp-178;
//        visualPosition.GetArray()[2]+=451;
// #endif
//     }


// void BodySchemaLearningModule::sendOutput(){}


bool BodySchemaLearningModule::close(){
  commands.close();//update to stop
  bsPort.close();//update to stop
  proprioception_arm.close();
  vision.close();
  proprioception_head.close();
  visual_rotation_port.close();
  if(logging){log.close();}
  if(body_data){ delete[] body_data;}
  delete head_buffer;
  delete arm_buffer;
  delete body;
  return true;
}



bool BodySchemaLearningModule::updateBody(){
    float dist, drot;
    int ind_h, ind_a;
    dist=drot=-1.f;
//   visualPosition.Print();
//   proprioceptivePosition.Print();
    if(eyes_arm){
        if(vision.ReadPosition(visualPosition.GetArray())){
            //only for this calibration
            visualPosition[0] *= -1;
            visualPosition[1] *= -1;
            Stamp stamp;
            if(vision.getEnvelope(stamp)){
                ind_h = head_buffer->lookFor(stamp.getTime());
                if(ind_h >=0){
                    ind_a = arm_buffer->lookFor(stamp.getTime());
                    if(ind_a >=0){
                        Vector angles(eyes_arm->GetNbJoints());
                        head->SetAngles(head_buffer->get(ind_h));
                        arm->SetAngles(arm_buffer->get(ind_a));
                        eyes_arm->GetAngles(angles.GetArray());//update could be optimized to avoid setting angles twice
                        log<<"1 "<<angles<<visualPosition<<outlier_thresh<<endl;
                        dist = eyes_arm->Update(angles.GetArray(),visualPosition.GetArray(),outlier_thresh); //could add marker position
                        for(int i=0;i<3;i++){
                            eyes_arm->Update(angles.GetArray(),visualPosition.GetArray(),outlier_thresh); //could add marker position             
                        }               
                    }
                    else{
                        cout<<"cannot find stamp in arm buffer "<<stamp.getTime()<<endl;
                    }        
                }
                else{
                cout<<"cannot find stamp in head buffer "<<stamp.getTime()<<endl;
                }
            }
            else{
                cout<<"cannot read envelope on vision buffer "<<stamp.getTime()<<endl;
            }
        }
    }
    if(visual_rotation_port.ReadRotation(seenRotation,&t0,&t1)){
      Vector ew0_angles(eye_world->GetNbJoints());
      Vector ew1_angles(eye_world->GetNbJoints());
      Vector ew_diff(eye_world->GetNbJoints());
        
      // find the position at time t1
      int ind = head_buffer->lookFor(t1);
      if(ind>=0){
          head->SetAngles(head_buffer->get(ind));
          eye_world->GetAngles(ew1_angles.GetArray());
          
          
          //finding the position at time t0
          ind = head_buffer->lookFor(t0);
          if(ind>=0){
              head->SetAngles(head_buffer->get(ind));
              eye_world->GetAngles(ew0_angles.GetArray());
              
              ew_diff = ew1_angles - ew0_angles;
              //             ew0_angles.Print();
              //               ew_diff.Print();
              log<<"0 "<<ew0_angles<<ew_diff<<seenRotation[0]<<" "<<seenRotation[1]<<
                 " "<<seenRotation[2]<<endl;
              for(int i=0;i<3;i++){
                  drot = eye_world->UpdateWithFullJacobian(ew0_angles.GetArray(),ew_diff.GetArray(),seenRotation);
              }
              rotation_update = 0;
          }
          else{
              cout<<"cannot find time in head buffer "<<t0<<endl;;
          }
      }
      else{
          cout<<"cannot find time in head buffer "<<t1<<endl;;
      }
  }
    bool ret_val = dist > 0 || drot>0;
    if(ret_val){
        if(!(rand()%1))cout<<"drot "<<drot<<" dist "<<dist<<endl;
    }
    return ret_val;
}

bool BodySchemaLearningModule::updateModule(){
    if(simul)return simulUpdate();
    if(commands.ShouldQuit()){return false;}
    listen();
    if(updateBody()){
        if((++update_cnt) == outputFrequency){
            body->GetArticulatedTree()->Serialize(body_data,body_data_size);
            bsPort.SendBodySchema(body_data,body_data_size);   
            update_cnt=0;
        }
    }
    //  else{
    //   Time::delay(0.1);
    // }
    return true;
}


bool BodySchemaLearningModule::simulUpdate(){
    Vector ew_angles(eye_world->GetNbJoints());
    Vector ew_diff(eye_world->GetNbJoints());
    Vector angles(eyes_arm->GetNbJoints());

    //  cout<<"char "<<c<<endl;
    float dist,drot;

  //   log.seekg (0, ios::end);
//     int length = log.tellg();
//     log.seekg ((int)RND(length), ios::beg);
//     log.ignore(300,'\n');

    char c =log.peek();
    if (log.eof()){
        cout<<"end of file"<<endl;
        log.seekg(0);
        log.clear();
        return (++update_cnt) < 1;
    }
    if(c=='1'){
        log>>c;
        for(unsigned int i=0;i<angles.Size();i++){
            log>>angles.GetArray()[i];
        }
        for(unsigned int i=0;i<visualPosition.Size();i++){
            log>>visualPosition.GetArray()[i];
        }
        log>>outlier_thresh;
 //        angles.Print();
//         visualPosition.Print();
        //only for this calibration
//         visualPosition[0] *= -1;
//         visualPosition[2] *= -1;
        dist = eyes_arm->Update(angles.GetArray(),visualPosition.GetArray(),outlier_thresh);
        cout<<"dist "<<dist<<endl;
        cout<<"h_axes ";head->PrintAxes(cout);
        cout<<"a_axes ";arm->PrintAxes(cout);
        for(int i=0;i<3;i++)
            dist = eyes_arm->Update(angles.GetArray(),visualPosition.GetArray(),outlier_thresh);
    }
    else{
        log>>c;
        for(unsigned int i=0;i<ew_angles.Size();i++){
            log>>ew_angles.GetArray()[i];
        }
        for(unsigned int i=0;i<ew_diff.Size();i++){
            log>>ew_diff.GetArray()[i];
        }
        //        ew_angles.Print();
        log>>seenRotation[0]>>seenRotation[1]>>seenRotation[1];
        
        //uncomment for data analysis
        for(int i=0;i<4;i++){
            drot = eye_world->UpdateWithFullJacobian(ew_angles.GetArray(),
                                                     ew_diff.GetArray(),seenRotation);
        }
        cout<<"h_axes ";head->PrintAxes(cout);

    }
    log.ignore(2,'\n');
//     update_cnt++;  
//     if (update_cnt==2)return false;
    return true;
}




int usage(){
    cout<<"usage: body_schema_module [--file <config_file> | --structure <structure_file>] "
        <<"[--name <port_base_name>] [--threshold <outlier_threshold>] "
        <<"[--frequency <output_frequenct>] [--rate <learning_rate>] ]"<<endl;//update
    return 1;
}


TimedBuffer::TimedBuffer(int si, int wi){
    size= si;
    width = wi;
    buffer = new float[size*width];
    time_buf = new double[size];
    cnt=0;
    full=false;
}

TimedBuffer::~TimedBuffer(){
    delete[] buffer;
    delete[] time_buf;
}

void TimedBuffer::add(float *v, double t){
    memcpy(buffer+cnt*width,v,width*sizeof(float));
    time_buf[cnt] = t;
    cnt++;
    if(cnt==size){
        cnt=0;
        full=true;
    }
}
// assumed time entries are more or less increasing

float *TimedBuffer::get(int index){
    return buffer+index*width;
}

int TimedBuffer::lookFor(double t){
    int i=cnt;
    int lasti=cnt;
    while(true){
        if(i<0){
            if(full){ //wrapping 
                i= size-1;
            }
            else{
                return -1;
            }
        }
        // buffer entirely visited
        if(i==cnt && lasti==cnt+1){
            cout<<"index not found";
            return -1;
        }
        //found
        if(time_buf[lasti]>=t && time_buf[i]<=t){
            return t>0.5*(time_buf[i]+time_buf[lasti])?lasti:i;
        }
        lasti=i;
        i--;
    }
}


int main(int argc, char *argv[]){
  BodySchemaLearningModule module;
  Property prop;
  srand(time(NULL));
  if(argc==1){
    return usage();
  }
  if(prop.check("help")){
    return usage();
  } 
  module.runModule(argc,argv);
}

