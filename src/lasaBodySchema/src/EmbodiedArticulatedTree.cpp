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

#include "EmbodiedArticulatedTree.h"
#include "EmbodiedKinematicChain.h"
#include "GL/glut.h"



EmbodiedArticulatedTree::EmbodiedArticulatedTree()
:ArticulatedTree()
{
  solid=NULL;
}

EmbodiedArticulatedTree::EmbodiedArticulatedTree(const pTree config){
 unsigned int i,j;
  float f;
  CVector3_t v;
  parent = NULL;
  solid=NULL;  
  children.clear(); 
  
  name = config->GetData();
 cout<<"adding "<<name<<endl;
  Tree_List *subTrees = config->GetSubTrees();
  //  cout<<"sub "<<subTrees->size()<<endl;
  for(i=0;i<(*subTrees).size();i++){
    if((*subTrees)[i]->GetName().compare("Axis")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
	cout<<v<<endl;
      joint.SetRotationAxis(v);
    }
    if((*subTrees)[i]->GetName().compare("Angle")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> f;
      joint.SetAngle(f*deg2rad);
    }
#ifdef WITH_RANGES
    if((*subTrees)[i]->GetName().compare("Range")==0){
      istringstream s((*subTrees)[i]->GetData());
      s>>range[0]>>range[1];
      range[0] *= deg2rad;
      range[1] *= deg2rad;
      //      cout<<range[0]<<" "<<range[1]<<endl;
    }
#endif
    if((*subTrees)[i]->GetName().compare("Position")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
      joint.SetTranslation(v);
    }
    if((*subTrees)[i]->GetName().compare("Children")==0){
      Tree_List *children = (*subTrees)[i]->GetSubTrees();
      //      cout<<" children "<<children->size()<<endl;
      for(j=0;j<children->size();j++){
	AddTree(new EmbodiedArticulatedTree((*children)[j]));
      }
    }
  }
}


EmbodiedArticulatedTree::~EmbodiedArticulatedTree(){
   if(solid)delete solid;
}

void EmbodiedArticulatedTree::RenderBegin(){
  CMatrix4_t mat;
  CVector3_t tr;
  joint.GetTranslation(tr);
  joint.RotToMatrix(mat);
  glPushMatrix();
  glTranslatef(tr[0],tr[1],tr[2]);
  glMultMatrixf(mat);
}

void EmbodiedArticulatedTree::RenderEnd(){
  glPopMatrix();
}


void EmbodiedArticulatedTree::Render(){
  RenderBegin();
  if(solid){
      solid->Render();
      solid->Unhighlight();
  }
   ArticulatedTreeList_t::const_iterator it; 
  for(it=children.begin(); it!=children.end(); ++it){
    ((EmbodiedArticulatedTree *)(*it))->Render();
  }
  RenderEnd();
}

void EmbodiedArticulatedTree::StreamTree(ostream& out) const {
 CVector3_t v;
 out<<"<Segment> "<<name<<endl;
 joint.GetRotationAxis(v);
 out<<"<Axis> "<<v<<" </Axis>"<<endl;
 out<<"<Angle> "<<joint.GetAngle()*rad2deg<<" </Angle>"<<endl;
 joint.GetTranslation(v);
 out<<"<Position> "<<v<<" </Position>"<<endl;
 //  Invert(tmp);
 if(solid){
   out<<"<Shape>"<<endl;
   solid->Stream(out);
   out<<"</Shape>"<<endl;
 }
 if(children.size()>0){
   out<<"<Children> "<<endl;
    ArticulatedTreeList_t::const_iterator it; 
   for(it=children.begin(); it!=children.end(); ++it){
      (*it)->StreamTree(out);
    }
    out<<"</Children>"<<endl;
  } 
 out<<"</Segment>"<<endl;

}

//initializes all the shape objects
// file format
// <Body>
//   <Shape>
//     <Name> r_eb </Name>
//     <Type> Capsule </Type>
//     <Params> 
//        <Radius> 10 </Radius> 
//        <Axis> 0 0 1 </Axis>
//     </Params>
//   </Shape>
// </Body>  
int EmbodiedArticulatedTree::SetAllShapes(pTree shape_tree){
  int i,n,j,nj,cnt=0;
  pArticulatedTree_t tree;
  Tree_List *shapes = shape_tree->GetSubTrees();
  string type;
  n= shapes->size();
  cout<<n<<" shapes investigated"<<endl;
  for(i=0;i<n;i++){
    if(shapes->at(i)->GetName().compare("Shape")==0){
      Tree_List *descr = shapes->at(i)->GetSubTrees();
      nj = descr->size();
      tree = NULL;
      type = "";
      pTree params =NULL;
      for(j=0;j<nj;j++){
	if(descr->at(j)->GetName().compare("Name")==0){
	  string name = descr->at(j)->GetData();
	  tree = FindJoint(name);
	}
	if(descr->at(j)->GetName().compare("Type")==0){
	  type = descr->at(j)->GetData();
	}
	if(descr->at(j)->GetName().compare("Params")==0){
	  params = descr->at(j);
	}
	if(tree && type!="" && params){
	  if(!type.compare("Capsule")){
          if(!params->Find(string("Axis"))){
              // getting default capsule axis
              if(tree->GetChildren()->size()>0){
                  char tmp[80];
                  string axis_val;
                  CVector3_t axis;
                  tree->GetChildren()->at(0)->GetJoint()->GetTranslation(axis);
                  sprintf(tmp,"%.5f %.5f %.5f",axis[0],axis[1],axis[2]);
                  axis_val = tmp;
                  cout<<tmp<<endl;
                  params->AddSubTree(new Tree(string("Axis"),axis_val));
              }
          } 
      ((EmbodiedArticulatedTree *)tree)->SetShape(new Capsule(params));
      cnt++;
	  }
	  else{
	    if(!type.compare("Sphere")){
	      ((EmbodiedArticulatedTree *)tree)->SetShape(new Sphere(params));
	      cnt++;
	    }
	    else{
	      if(!type.compare("Parallelipiped")){
		((EmbodiedArticulatedTree *)tree)->
		  SetShape(new Parallelipiped(params));
	      cnt++;
	      }
	    }
	  }
	}
      }
    }
  }
  return cnt;
}

void  EmbodiedArticulatedTree::SetShape(Shape *sol){
  if(solid)delete solid;
  solid=sol;
}

void EmbodiedArticulatedTree::UpdateShape(){
    
    CVector3_t tr[10];//max number of children
    int i=0;
    ArticulatedTreeList_t::const_iterator it; 
    if(solid){    
        for(it=children.begin(); it!=children.end(); ++it){
            (*it)->GetJoint()->GetTranslation(tr[i]);
            i++;
        }

        solid->Update(tr,i);
    }
    for(it=children.begin(); it!=children.end(); ++it){
        ((EmbodiedArticulatedTree *)(*it))->UpdateShape();
    }  
}
 

EmbodiedKinematicTree::EmbodiedKinematicTree():KinematicTree(){
   e_root = (EmbodiedArticulatedTree *)root;
}

EmbodiedKinematicTree::EmbodiedKinematicTree(char *filename1, char *filename2){
  Tree *shape_tree = new Tree();
  xml_tree = new Tree();
  solid = NULL;
  xml_tree->LoadFromFile(string(filename1));
  root = new EmbodiedArticulatedTree(xml_tree);
  shape_tree->LoadFromFile(string(filename2));
  cout<<"file loaded"<<endl;
  e_root =(EmbodiedArticulatedTree *)root;
  e_root->SetAllShapes(shape_tree);
  cout<<(*e_root)<<endl;
}


int EmbodiedKinematicTree::CheckAllCollisions (KinematicTree *kt2=NULL){

  collision_arg_t arg;
  arg.dist = 0.1; 
  arg.kt_update = kt2;
  arg.kt = this;
  arg.kc = new EmbodiedKinematicChain();
  return root->CheckPairsOfNodes(EmbodiedKinematicTree::CollisionDetection,&arg);
}

/**
 * @brief detects if  there is a collision between link t1 and t2. This works
 * by creating a KinematicChain from t1 to t2 and computing the position of
 * link t1 in the frame of ref of t2. Consecutive links are discarded, as they
 * touch each other at the joint
 * @param dist the distance below which contact is assumed 
 * @param t1 the first link
 * @param t2 the second link
 * @return 1 if there is a collision, 0 otherwise
 */


int EmbodiedKinematicTree::CollisionDetection(pArticulatedTree_t t1, pArticulatedTree_t t2, void *arg){
 collision_arg_t *state;
 float distance=-1.0f;
 RigidTransfo rt;
 EmbodiedArticulatedTree *from,*to;
 state = (collision_arg_t *) arg;
 from = (EmbodiedArticulatedTree *)t1;
 to = (EmbodiedArticulatedTree *)t2;
 if(from->GetParent()!= to && to->GetParent()!= from){
   if(from->GetShape() && to->GetShape()){
     state->kt->LoadChain(t1,t2,state->kc, TOUCH_MODALITY);   
     state->kc->GlobalTransfo(rt);
     distance = from->GetShape()->Distance(*(to->GetShape()),rt);
//      if(distance<0){
//        cout<<from->name<<" - "<<to->name<<" "<<distance<<endl;
//      }
     if(distance<=state->dist){
       char c=7;
       cout<<from->name<<" - "<<to->name<<" "<<distance<<endl;
       from->Highlight();
       to->Highlight();
       cout<<c;
       return 1;
     }
   }
 }
 return 0;
}
