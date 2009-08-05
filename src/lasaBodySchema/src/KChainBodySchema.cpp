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
#include "TreeParser.h"
#include "KChainBodySchema.h"


//  KChainBodySchema::KChainBodySchema(char *filename){
//      telescopic
//  }


// KChainBodySchema::KChainBodySchema(const KinematicChain& chain){
//   int i,n;
//   n = chain.GetNbJoints();
//   for(i=0;i<n;i++){
//     AddJoint( new RigidTransfo(*(chain.GetTransfo(i))),chain.IsInverted(i));
//   }
//   modality = chain.GetModality();
//   tol =chain.GetTolerance();
// }

void  KChainBodySchema::Jacobian(Matrix& jac){
  Matrix mat(1,4*joint_angle_dim,true);
  int n = nb_joints;
  jac.Resize(3,n,false);
  GetJacobian(mat.Array());
  for(int i=0;i<n;i++){
    for(int j=0;j<3;j++){
      jac.Array()[j*n+i] = mat.Array()[i*4+j];
    }
  }
}


/**
 * @brief samples the workspace to find adequate weights
 */
void KChainBodySchema::GetWeights(cart_vec_t& cart_weights, joint_vec_t& joint_weights){
  joint_vec_t angles;
  CVector3_t kstack[MAX_LINKS];
  CVector3_t pos;
  int n =50; //number of samples
#ifdef WITH_LAST_LINK
  last_link->GetTranslation(pos);
  int last = nb_joints;
#else
  v_clear(pos);
  int last = nb_joints-1;
#endif
  cart_weights.Zero();
  joint_weights.Zero();
  cart_weights += 1;
  for(int i=0;i<n;i++){
    SetRandomAngle(angles);
    ForwardKinematicsStack(pos,kstack);
    for(int j=0;j<last;j++){
      joint_weights[j] += v_length(kstack[j+1]);
    }
  }
  joint_weights /=(float) n;

  for(int j=0;j<last;j++){
    if(joint_weights[j]<epsilon){ //avoiding deviding by 0 
      joint_weights[j] = 100000;
    }
    else{
      joint_weights[j] = 1.f/joint_weights[j];
    }
    
  }
  joint_weights.Print();
    
}


int KChainBodySchema::InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles){
  int done =ik_trials;
  float dist;
  joint_vec_t a;
  while(done){
      dist = TryInverseKinematics(pos,angles);
      cout<<"ikdist "<<dist<<endl;
      if(dist<=tol){
          return done;
      }
      else{
          SetRandomAngle(a);//new start
          done--;
      }
  }
  return done;
  //  return TryInverseKinematics(pos,config,angles)<tol;   

}
    


float KChainBodySchema::TryInverseKinematics(const cart_vec_t& pos,joint_vec_t& angles){
 CVector3_t stack[MAX_LINKS];
  CVector3_t tar,newrod,rod,diff;
  float dist=-1.0f;
  float cand; //candidate
  for(int j=0;j<20;j++){
    InverseKinematicsStack(pos.GetArray(),stack);
#ifdef WITH_LAST_LINK
    last_link->GetTranslation(rod);
    if(telescopic){// useful for pointing to, or gazing to)
        v_normalize(rod,newrod);
        float f=v_dot(newrod,stack[nb_joints]);
        if(f>0){
            v_scale(newrod,f,rod);
        }
    }
#else
    v_clear(rod);
#endif
        
    for(int i=nb_joints-1;i>=0;i--){
      if(IsInverted(i)==-1){
          v_copy(stack[i],tar);
          Translate(i,rod,newrod); //getting to joint
          cand =joints[i]->AimingAngle(tar,newrod);
	//	joints[i]->InvertRotationAngle();
          cand = AngleInClosestRange(i,cand);
          joints[i]->SetAngle(-cand);//check if that's right
          Rotate(i,newrod,rod);   // updating "rod"
          v_sub(tar,rod,diff);
      }
      else{
          Rotate(i,stack[i+1],tar); //rotating back
          cand =joints[i]->AimingAngle(tar,newrod);
          cand = AngleInClosestRange(i,cand);
          joints[i]->SetAngle(cand);
          Rotate(i,rod,newrod);
          Translate(i,newrod,rod);
          v_sub(tar,newrod,diff);
      }
      dist=v_length(diff);
      if(dist<=tol)
          {
              GetAngles(angles);//trial
              return dist;
          }
      //     cout<<v_length(diff)<<endl;;
    }
  }
  GetAngles(angles);
  return dist;
}



void KChainBodySchema::Load(const char *filename){
  //#ifdef NOT_YET   
  Tree *xml_tree = new Tree();
  Tree *child = xml_tree;
  CVector3_t v;
  float f, min_a,max_a;
  int index=0;

  xml_tree->LoadFromFile(filename);
  while (child){ 
    string name = child->GetData();
    cout<<"adding "<<name<<endl;
    Tree_List *subTrees = child->GetSubTrees();
    child =NULL;
    RigidTransfo *rt =new RigidTransfo();
    for(unsigned int i=0;i<(*subTrees).size();i++){
      if((*subTrees)[i]->GetName().compare("Axis")==0){
	istringstream s((*subTrees)[i]->GetData());
	s >> v[0]>>v[1]>>v[2];
	rt->SetRotationAxis(v);
      }
      if((*subTrees)[i]->GetName().compare("Angle")==0){
	istringstream s((*subTrees)[i]->GetData());
	s >> f;
	rt->SetAngle(f*deg2rad);
      }
      if((*subTrees)[i]->GetName().compare("Position")==0){
	istringstream s((*subTrees)[i]->GetData());
	s >> v[0]>>v[1]>>v[2];
	rt->SetTranslation(v);
      }
      if((*subTrees)[i]->GetName().compare("Range")==0){
	istringstream s((*subTrees)[i]->GetData());
	s>>min_a>>max_a;
	//      cout<<range[0]<<" "<<range[1]<<endl;
      }
      if((*subTrees)[i]->GetName().compare("Children")==0){
	Tree_List *children = (*subTrees)[i]->GetSubTrees();
	//	cout<<" children "<<children->size()<<endl;
	if(children->size()>0){
	  child = (*children)[0];
	  index++;
	}
      }
   }
    if(rt->AxisOk()){ //there is a rotation
     min_angle.Resize(nb_joints+1);
     max_angle.Resize(nb_joints+1);
     min_angle[nb_joints] = min_a;
     max_angle[nb_joints] = max_a;
     AddJoint(rt);
    }
#ifdef WITH_LAST_LINK
    else{           // it is the last link
      SetLastLink(rt->GetTranslation());
      delete rt;
    }
#endif
  }
  min_angle *= deg2rad;
  max_angle *= deg2rad;
  delete xml_tree;
  //#endif
}



#ifdef OLD_INV_KIN
int KChainBodySchema::InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles){
  float f;
  int it=0;
  f=InverseKinPosCCD(pos.GetArray()); 
  if(f>tol){
    cout<<"can't find inverse kinematics"<<endl;
    return 0;
  }
  KinematicChain::GetAngles(angles.GetArray()); 
  if(!AnglesInRange(angles)){
    Matrix jac,pjac,mat(joint_angle_dim,joint_angle_dim);
    joint_vec_t dangle,middle;
    middle = (min_angle+max_angle)*0.5;  
    do{ // gets back to allowable range
      Jacobian(angles,jac);
      jac.Inverse(pjac); //pseudo-inverse
      if(!Matrix::IsInverseOk())
	{cout<<"bad matrix inversion in InverseKinematics"<<endl;}
      mat.Identity();
      mat -= pjac*jac;
      dangle = mat*(middle-angles)*0.01;
      angles += dangle;      
      //     cout<<"ik "<<angles<<endl;
      //mat.Print();
    }while(!AnglesInRange(angles) && it++ <500);
    if(it<=100){
      cout<<"can't bring target configuration in range"<<endl;
    }
    else{
      cout<<"in range"<<endl;
    }
    f=InverseKinPosCCD(pos.GetArray());
  }
  return f<=tol;
}



#endif
