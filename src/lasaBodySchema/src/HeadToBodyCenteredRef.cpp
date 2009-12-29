#include <ace/OS_NS_stdio.h>
#include "HeadToBodyCenteredRef.h"
//#include <ace/OS.h>


int cartesian_dim=3;
int joint_angle_dim=0;

HeadToBodyCenteredRef::HeadToBodyCenteredRef(){

}

bool HeadToBodyCenteredRef::open(Searchable& s){
Value& struc = s.find("structure");
  if(struc.isNull()){
    ACE_OS::printf("no robot structure file given\n");
    return false;
  }
  body = new KinematicTree(struc.asString().c_str());
  head = new KChainBodySchema(); //KChainOrientationBodySchema if you want to determine visually the rotation
  
  Value& from = s.find("from");
  Value& to = s.find("to");
  if(! from.isNull() && ! to.isNull()){
      if(!body->LoadChain(from.asString().c_str(),to.asString().c_str(),head)){
          cout<<"cannot load chain going from "<<from.asString().c_str()<<" to "<<to.asString().c_str()<<endl;
      }
  }
  else{// default values
      body->LoadChain("torso","eyes",head);
  }
    cout<<head<<endl;
    //  head.Load(struc.asString().c_str());
  joint_angle_dim = head->GetNbJoints();
  cout<<"nb joints "<<joint_angle_dim<<endl;
  proprio.Resize(joint_angle_dim);

  //buffer for body schema reading
  body_data_size = body->GetTreeSize()*6;
  bodydata = new float[body_data_size];

  //opening ports 
  if(!in.open(getName("position:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("position:i").c_str());
    return false;
  }
  if(!out.open(getName("position:o").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("position:o").c_str());
    return false;
  }
  if(!headProprio.open(getName("proprio_head:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("proprio_head:i").c_str());
    return false;
  }
  if(!bodyPort.open(getName("body_schema:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("body_schema:i").c_str());
    return false;
  }

if(!cmdPort.open(getName("cmd:i").c_str())){
    ACE_OS::printf("Cannot open port %s\n",getName("cmd:i").c_str());
    return false;
  }
  return true;



}

bool HeadToBodyCenteredRef::close(){
  in.close();
  out.close();
  headProprio.close();
  bodyPort.close();
  cmdPort.close();
  delete[] bodydata;
 return true;
}



bool HeadToBodyCenteredRef::updateModule(){
  // cart_vec_t in, out;
  

  if(cmdPort.ShouldQuit()){return false;}
  
  // checking if the body schema can be updated
  if(bodyPort.ReadBodySchema(bodydata,body_data_size)){
    body->GetArticulatedTree()->Deserialize(bodydata,body_data_size);
  }

  //getting the latest propioceptive informatin
  headProprio.ReadPosition(proprio.GetArray(),joint_angle_dim);
  proprio *=deg2rad;
  // headProprio.Print();

  //checking if an input position has arrived
  if(in.ReadPosition(visionPosHead.GetArray())){
    float d = visionPosHead.Norm();
 
    // this is where you can change the signs, but do the same in BodySchemaLearning.cpp
    visionPosHead.GetArray()[0] *= -1;
    visionPosHead.GetArray()[2] *= -1;
    if(d < 1000 && visionPosHead.GetArray()[2] > 100){ //exclude vision outliers 
 
    head->SetForwardKinematics(proprio.GetArray(),visionPosBody.GetArray(), 
				 visionPosHead.GetArray());
    //sending out the result
    out.SendPosition(visionPosBody.GetArray());
    }
  }


  return true;
}


int usage(){
  cout<<"usage: HeadToBodyModule  [--file <config_file> | --structure <structure_file> --from <head-joint> --to <tail_joint>]"<<endl;
  return 1;
}

int main(int argc, char *argv[]){
  HeadToBodyCenteredRef module;
  Property prop;
  if(argc==1){
    return usage();
  }
   module.runModule(argc,argv);
}
