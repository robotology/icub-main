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
#include "KinematicTree.h"

#define STAND_UP
ArticulatedTree::ArticulatedTree(){
  parent=NULL;
  children.clear();
  // inverse = 1;
}

#ifndef NO_CSOLID
ArticulatedTree::ArticulatedTree(const pCSolidTree_t stree){
  parent = NULL;
  children.clear();
  //  inverse=1;
  joint.RotFromMatrix(stree->m_ref.m_orient);
  joint.SetTranslation(stree->m_ref.m_origin);
  pArticulatedTree_t next;
  CSolidTreeList_t::const_iterator it;
  for(it=stree->m_next.begin(); it!=stree->m_next.end(); ++it){
    next = new ArticulatedTree(*it);
    children.push_back(next);
    next->parent= this;
  }
}

#endif


ArticulatedTree::ArticulatedTree(const pTree config){
  unsigned int i,j;
  float f;
  CVector3_t v;
  parent = NULL;
  children.clear(); 


  name = config->GetData();
  //  cout<<"adding "<<name<<endl;
  Tree_List *subTrees = config->GetSubTrees();
  //  cout<<"sub "<<subTrees->size()<<endl;
  for(i=0;i<(*subTrees).size();i++){
    if((*subTrees)[i]->GetName().compare("Axis")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
      joint.SetRotationAxis(v);
    }
    if((*subTrees)[i]->GetName().compare("Angle")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> f;
      joint.SetAngle(f*deg2rad);
    }
    if((*subTrees)[i]->GetName().compare("Position")==0){
      istringstream s((*subTrees)[i]->GetData());
      s >> v[0]>>v[1]>>v[2];
      joint.SetTranslation(v);
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
    if((*subTrees)[i]->GetName().compare("Children")==0){
      Tree_List *children = (*subTrees)[i]->GetSubTrees();
      //      cout<<" children "<<children->size()<<endl;
      for(j=0;j<children->size();j++){
	AddTree(new ArticulatedTree((*children)[j]));
      }
    }
  }
} 

ArticulatedTree::ArticulatedTree(const pArticulatedTree_t tree){
  ArticulatedTreeList_t::const_iterator it;
  joint = tree->joint;
  //  inverse = tree->inverse;
  name = tree->name;
  for(it=tree->children.begin(); it!=tree->children.end(); ++it){
    AddTree(new ArticulatedTree(*it));
  }
}
ArticulatedTree::~ArticulatedTree(){
  ArticulatedTreeList_t::const_iterator it; 
  for(it=children.begin(); it!=children.end(); ++it){
    delete (*it);
  }
}

void ArticulatedTree::AddTree(pArticulatedTree_t ptree){
 ptree->parent = this; 
 children.push_back(ptree);
}

pArticulatedTree_t ArticulatedTree::FindJoint(string& s){
  if(name.compare(s)==0){
    return this;
  }
  else{
    ArticulatedTreeList_t::const_iterator it;
    pArticulatedTree_t res;
    for(it=children.begin(); it!=children.end(); ++it){
      res =(*it)->FindJoint(s);
      if(res != NULL){ 
	return res;
      }
    }
    return NULL;
  }
}

/**
 * @brief tells whether the tree contains a given subtree.
 * @param t the subtree to look for
 * @param exclude a subtree to exclude from search (typically because it has already been searched)
 * @retun NULL if the tree does not contain the given subtree, a pointer to the subtree (i.e. t) otherwise
 */

pArticulatedTree_t ArticulatedTree::FindSubTree(pArticulatedTree_t t, pArticulatedTree_t exclude){
  if(this==t){
    return this;
  }
  else{
    ArticulatedTreeList_t::const_iterator it;
    pArticulatedTree_t res;
    for(it=children.begin(); it!=children.end(); ++it){
      if(*it != exclude){
	res =(*it)->FindSubTree(t,exclude);
	if(res != NULL){ 
	  return res;
	}
      }
    }
    return NULL;
  }
}


void ArticulatedTree::RandomAxis(){
  ArticulatedTreeList_t::const_iterator it;
  joint.RandomAxis();
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->RandomAxis();
  }
}

void ArticulatedTree::RandomAngle(){
  ArticulatedTreeList_t::const_iterator it;
#ifdef WITH_RANGES
  float angle = RND(range[1]-range[0])+range[0];
  joint.SetAngle(angle);
#else
  joint.RandomAngle();
#endif
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->RandomAngle();
  }  
}
void ArticulatedTree::ZeroPosition(){
  ArticulatedTreeList_t::const_iterator it;
  joint.SetAngle(0);
#ifdef STAND_UP
    if(name=="r_knee"){
      joint.SetAngle(-pi/2);
    } else{  
      if(name =="l_knee"){
	joint.SetAngle(pi/2);
      }
    }
#endif
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->ZeroPosition();
  }
}



void ArticulatedTree::NoTranslation(){
  ArticulatedTreeList_t::const_iterator it;
  CVector3_t v;
  v_set(RND(0.01)-0.005,RND(0.01)-0.005,RND(0.01)-0.005,v)
  joint.SetTranslation(v);
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->NoTranslation();
  }
}

/**
 * @brief copies the values from another tree with the same structure.
 * Assumes that all the tree structure is already allocated
 * @param tree the tree to be copied
 */
void ArticulatedTree::CopyTree(pArticulatedTree_t tree){
  unsigned int j;
  CVector3_t v;
  tree->GetAxis(v);
  joint.SetRotationAxis(v);
  //  tree->GetJoint()->GetRotationParam(v);
  //  joint.SetTransfo(v);
  joint.SetTranslation(tree->GetJoint()->GetTranslation());
  for(j=0;j<children.size();j++){
    children[j]->CopyTree(tree->GetChildren()->at(j));
  } 
}


void ArticulatedTree::StreamTree(ostream& out) const {
  CVector3_t v;
  // int tmp = inverse;
  //  Invert(1);
  out<<"<Segment> "<<name<<endl;
  joint.GetRotationAxis(v);
  out<<"<Axis> "<<v<<" </Axis>"<<endl;
  out<<"<Angle> "<<joint.GetAngle()*rad2deg<<" </Angle>"<<endl;
  joint.GetTranslation(v);
  out<<"<Position> "<<v<<" </Position>"<<endl;
  //  Invert(tmp);
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


void ArticulatedTree::Reshape(CVector3_t offset){//make use of v
  CVector3_t ax,v1,tr;
  float norm,vn;
  if(parent){
    joint.GetRotationAxis(ax);
    joint.GetTranslation(tr);
    if(offset){
      v_add(tr,offset,tr);
    }
    vn = v_normalize(tr,v1);
    norm = v_dot(v1,ax);
    v_scale(ax,norm*vn,v1);
    v_sub(tr,v1,ax);
    //    cout<<"reshape "<<v1<<endl;
    joint.SetTranslation(ax);
  }
  ArticulatedTreeList_t::const_iterator it; 
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->Reshape(v1);
  }
}

void ArticulatedTree::Reshape2(CVector3_t v1){//make use of v
  CVector3_t ax,tr,v;
  float norm,vn;
  ArticulatedTreeList_t::const_iterator it; 
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->Reshape2(v);
    
  if(parent){
    parent->GetAxis(ax);
    joint.GetTranslation(tr);
    v_add(tr,v,tr);
    vn = v_normalize(tr,tr);
    norm = v_dot(tr,ax);
    v_scale(ax,norm*vn,v1);
    v_sub(tr,v1,ax);
    //    cout<<"reshape "<<v1<<endl;
    joint.SetTranslation(ax);
  }
  }

}

#ifndef NO_CSOLID

pCSolidTree_t ArticulatedTree::CreateCSolidTreeStruct(){
  pCSolidTree_t st = new CSolidTree();
  if(children.size()>0){
    ArticulatedTreeList_t::const_iterator it; 
    for(it=children.begin(); it!=children.end(); ++it){
      st->AddSolid((*it)->CreateCSolidTreeStruct());
    }
  }
  return st;
}


void ArticulatedTree::FillCSolidTree(pCSolidTree_t solid){

//   if(parent){ 
//       //          GetParent()->GetJoint()->GetTranslation(solid->m_ref.m_origin);
//   GetParent()->GetJoint()->RotToMatrix(solid->m_ref.m_orient);
//   } 
  joint.GetTranslation(solid->m_ref.m_origin);   
  joint.RotToMatrix(solid->m_ref.m_orient);
    solid->m_name = name;
    joint.GetRotationAxis(solid->m_position);//rotation axis is put in m_position var
  // joint.GetTranslation(v);
//   solid->m_size);
//   v_scale(solid->m_size,-0.5,solid->m_position);

  // v_length(v);
  // v_set(10,20,30,solid->m_size);
  for(unsigned int i=0; i<children.size();i++){
    children[i]->FillCSolidTree(solid->m_next[i]);
  }
}
  
#endif


/**
 * @brief goes through all unordered pairs of nodes in the tree and performs a given function.
 * Pairs of the sames nodes are skipped. Uncomment the corresponding line of code if you want 
 * them.
 * @param f the function performed with all pairs of nodes 
 * @param arg a pointer to an addition argument that is passed to f
 */
int ArticulatedTree::CheckPairsOfNodes(int (*f)(pArticulatedTree_t, pArticulatedTree_t, void*), void *arg){
  unsigned int i,j;
  int nb=0;
  //nb = (*f)(this,this,arg); //if one wants pairs of same elements
   for(i=0; i<children.size();i++){ 
     nb+=children[i]->CheckPairsOfNodes(f,arg); // the pair of nodes is in the same subtree i
     nb+=children[i]->CheckTreeNode1(this,f,arg); // the pairs composed of this and all descent of i
     for(j=i+1; j<children.size();j++){           // the pairs composed of one elmnt in i and one in j 
       nb+= children[j]->CheckTreeNode2(children[i],f,arg);
     }
   }
   return nb;
}

int ArticulatedTree::CheckTreeNode1(pArticulatedTree_t tree, int (*f)(pArticulatedTree_t, pArticulatedTree_t,void*), void* arg){
    ArticulatedTreeList_t::const_iterator it;
    int nb=0;
    nb = (*f)(this,tree,arg);
    for(it=children.begin(); it!=children.end(); ++it){
      nb+=(*it)->CheckTreeNode1(tree,f,arg);
    }
    return nb;
}

int ArticulatedTree::CheckTreeNode2(pArticulatedTree_t tree, int (*f)(pArticulatedTree_t, pArticulatedTree_t,void*), void* arg){
    ArticulatedTreeList_t::const_iterator it;
    int nb=0;
    nb+=CheckTreeNode1(tree,f,arg);
    for(it=tree->GetChildren()->begin(); it!=tree->GetChildren()->end(); ++it){
      nb+=CheckTreeNode2(*it,f,arg);
    }
 
    return nb;
}


int ArticulatedTree::Serialize(float *data,int data_size)const{
    int n=1; 
    ArticulatedTreeList_t::const_iterator it;
    if(data_size<6){
        cout<<"warning: writing outside buffer in ArticulatedTree::Serialize"
            <<endl;
        return 0;
    }
    v_copy(joint.GetTranslation(),data);
    v_copy(joint.GetRotationAxis(),data+3);
    for(it=children.begin(); it!=children.end(); ++it){
        n+=(*it)->Serialize(data+n*6,data_size-n*6);
        
    }
    return n;
}

int ArticulatedTree::Deserialize(const float *data,int data_size){

    int n=1; 
    ArticulatedTreeList_t::const_iterator it;
    if(data_size<6){
        cout<<"warning: writing outside buffer in ArticulatedTree::Deserialize"
            <<endl;
        return 0;
    }
    joint.SetTranslation(data);
    joint.SetRotationAxis(data+3);
    for(it=children.begin(); it!=children.end(); ++it){
        n+=(*it)->Deserialize(data+n*6,data_size-n*6);
        
    }
    return n;
}


int ArticulatedTree::GetAngleList(float *list){
  ArticulatedTreeList_t::const_iterator it;
  int n=1;
  *list= joint.GetAngle();
  for(it=children.begin(); it!=children.end(); ++it){
    n+=(*it)->GetAngleList(list+n);
  }
  return n;
}


#ifdef WITH_RANGES
int ArticulatedTree::GetAngleRangeList(float *list_min,float *list_max){
  ArticulatedTreeList_t::const_iterator it;
  int n=1;
  *list_min=range[0];
  *list_max=range[1];
  for(it=children.begin(); it!=children.end(); ++it){
    n+=(*it)->GetAngleRangeList(list_min+n,list_max+n);
  }
  return n;
}

//lower_range and upper_range are assumed to have room for the right number of elements
int ArticulatedTree::FindAngleRanges(float *lower_range, float *upper_range,
                                     KinematicChain *chain){
    ArticulatedTreeList_t::const_iterator it;
    int i,n,k=0;
    n = chain->GetNbJoints();
    for(i=0;i<n;i++){
        if(&joint == chain->GetTransfo(i)){
            lower_range[i] = range[0];
            upper_range[i] = range[1];
            k=1;
            break;
        }
    }
    for(it=children.begin(); it!=children.end(); ++it){
        if(k==n) return k;
        k+=(*it)->FindAngleRanges(lower_range,upper_range,chain);
    }
    return k;
}

#endif


int ArticulatedTree::SetAngleList(float *list){
  ArticulatedTreeList_t::const_iterator it;
  int n=1;
  joint.SetAngle(*list);
  for(it=children.begin(); it!=children.end(); ++it){
    n+=(*it)->SetAngleList(list+n);
  }
  return n;
}

int ArticulatedTree::GetSize()const{
  ArticulatedTreeList_t::const_iterator it;
  int n=1;
  for(it=children.begin(); it!=children.end(); ++it){
    n+=(*it)->GetSize();
  }
  return n;
}

void ArticulatedTree::PrintAngles(){
  ArticulatedTreeList_t::const_iterator it;
  cout<<joint.GetAngle()<<" ";
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->PrintAngles();
  }
}


void ArticulatedTree::PrintList(){
  ArticulatedTreeList_t::const_iterator it;
  cout<<name<<endl;
  for(it=children.begin(); it!=children.end(); ++it){
    (*it)->PrintList();
  }
}

ostream& operator<<(ostream& out, const ArticulatedTree& at){
  at.StreamTree(out);
  return out;
}

KinematicTree::KinematicTree(){
  xml_tree = NULL;
  root = NULL;
  solid = NULL;
  nb_chains=0;
}

KinematicTree::KinematicTree(const char *filename){
  xml_tree = new Tree();
  solid = NULL;
  xml_tree->LoadFromFile(string(filename));
  root = new ArticulatedTree(xml_tree);
  //solid = root->CreateCSolidTreeStruct();
}

// dangerous for memory deallocation
// KinematicTree::KinemtaticTree(pArticulatedTree_t atree){
//   solid=NULL;
//   xml_tree = NULL:
//   root = atree;
// }


KinematicTree::~KinematicTree(){
  if(xml_tree)delete xml_tree;
  if(root) delete root;
#ifndef NO_CSOLID
  if(solid)delete solid;
#endif
}



/**
 * valid only for a direct link
 */ 


#ifdef WITH_LAST_LINK

/**
 * Extract the kinematic chain. The chain is assumed to end with a translation 
 * (the last rotation is discarded if the corresponding transformation is not
 * inverted). 
 */
int KinematicTree::LoadChain(pArticulatedTree_t from, pArticulatedTree_t to,
                             KinematicChain *kc, modality_t mod)const{

 
    kc->Clear();
    kc->SetModality(mod);
    //    kc->AddLastLink(to->GetJoint());
    int ret= LoadChainRec(from,to,kc);
    if(ret){
        if(kc->IsInverted(ret-1)==1){//not inverted
            kc->AddLastLink(kc->GetTransfo(ret-1));
            kc->RemoveJoint();
            return ret-1;
        }
    }
    return ret;
}

#else



int KinematicTree::LoadChain(pArticulatedTree_t from, pArticulatedTree_t to, KinematicChain *kc,
			     modality_t mod)const{
  kc->Clear();
  kc->SetModality(mod);
  //  cout<<"done"<<endl;
  return LoadChainRec(from,to,kc);
}

#endif


inline int KinematicTree::LoadChain(pArticulatedTree_t from, pArticulatedTree_t to, int i,
				     modality_t mod){
  return LoadChain(from,to,&(chains[i]),mod);
}


/**
 * @brief finds the path joining two nodes of a tree and builds the corresponding kinematic
 * chain. The two nodes must belong to the same tree to avoid infinite recursion.
 * @param starting node
 * @param final node
 * @param chain where to put the KinematicChain
 * @return the number of nodes in the path
 * @todo optimize it to avoid looking many times in the same subtree
 */
int KinematicTree::LoadChainRec(pArticulatedTree_t from, pArticulatedTree_t to, 
				KinematicChain *chain,pArticulatedTree_t exclude)const{
  if(to==from){
      if(chain->GetNbJoints()){ //the chain is on two branches - 
          return chain->GetNbJoints();
      }
      else{//the chain is on the same branch
          cout<<"adding "<<to->name<<endl;
          return chain->AddJoint(to->GetJoint(),1);
      }     
  }
  // from is not above to
  if(from->FindSubTree(to,exclude)==NULL){
    chain->AddJoint(from->GetJoint(),-1);
    cout<<"adding "<<from->name<<endl;
    if(from->GetParent()==to){
        cout<<"adding "<<to->name<<endl;
        return chain->AddJoint(to->GetJoint(),-1);
    }
    return LoadChainRec(from->GetParent(),to,chain,from);
  }
  LoadChainRec(from,to->GetParent(),chain);
  cout<<"adding "<<to->name<<endl;
  return chain->AddJoint(to->GetJoint(),1);
}


 int KinematicTree::LoadChain(string& from, string& to, int i, modality_t mod){
  return LoadChain(from,to,&(chains[i]),mod);
}
int KinematicTree::LoadChain(string& from, string& to, KinematicChain *kc, modality_t mod){
  pArticulatedTree_t f, t;
  f=root->FindJoint(from); 
  t=root->FindJoint(to);
  if(f && t){
    return LoadChain(f,t,kc,mod);
  }
  else{
    return -1;
  }
}

KinematicChain * KinematicTree::GetNewChain(string& from, string& to,modality_t mod){
  KinematicChain *kc = new KinematicChain();
  if(LoadChain(from,to,kc,mod)>0){
    return kc;
  }
  else{
    return NULL;
  }
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


int KinematicTree::CollisionDetection(pArticulatedTree_t t1, pArticulatedTree_t t2, void *arg){
  collision_arg_t *state;
  // CVector3_t v,v2,p1,p2;
  float k1,k2,distance=-1.0f;
  state = (collision_arg_t *) arg;
  //  KinematicChain kc;
  //  KinematicTree kt;
  //  RigidTransfo& rt = state->rt;
  if(t1->GetParent()==t2 || t2->GetParent()==t1){return 0;}
  if(t1->GetParent()==state->t1 && t2 == state->t2 && 0) {// and t2 is not a subtree of t1 -> just add a transformation  for optimizing processing speed
   //  rt = t1->GetJoint();
//     state->kc->AddJoint(rt,-1);
//     //  rt->InverseTransform(state->pos_stack[state->top],state->pos_stack[state->top-1]); 
//     //  state->top--;
//     rt = rt
//     state->t1 = t1;
  }
  else{
    state->kt->LoadChain(t1,t2,state->kc, TOUCH_MODALITY);   
}

   distance = state->kc->CollisionDetection(k1,k2);
  if(distance<0){
    return 0;
  }
  if(distance<=state->dist){
    //    cout<<t1->name<<" "<<t2->name<<" "<<distance<<endl;
    if(state->kt_update){
      float angles[MAX_LINKS];
      state->kc->GetAngles(angles);
       state->kt_update->LoadChain(t1->name,t2->name,&(state->kc_update),TOUCH_MODALITY);      
       state->kc_update.UpdateTouch(angles,k1,k2);
    }    
    return 1;
  }
  else{
    return 0;
  }
}
 
//static collision_arg_t arg;

int KinematicTree::CheckAllCollisions(KinematicTree *kt2=NULL){
  collision_arg_t arg;
  arg.dist = 10; 
  arg.kt_update = kt2;
  arg.kt = this;
  arg.kc = new KinematicChain();
  return root->CheckPairsOfNodes(KinematicTree::CollisionDetection,&arg);
}

// KinematicTree::Render(){
//   if(!solid){
//     if(root){
//       solid = root->CreateCSolidTreeStruct();
//     }
//   }
  
// }





//#define TEST_TREES2

#ifdef TEST_TREES


int main(int argc, char *argv[]){
  cout<<"loading tree"<<endl;
  CVector3_t pos;
  float angles[MAX_LINKS];
  string s1("head");
  string s2("r_wrist");
  string s3("r_elbow"),s4(" ");
  pArticulatedTree_t at2;
  KinematicChain *c1,*c2;
  KinematicTree *kt1 = new KinematicTree(argv[1]);
  KinematicTree *kt2 = new KinematicTree(argv[1]);
  at2 =kt2->GetArticulatedTree();
  //  at2->RandomAxis();
  at2->NoTranslation();
  kt1->LoadChain(s1,s2,0);
  kt2->LoadChain(s1,s2,0);
  c1 = kt1->GetChain(0);
  c2 = kt2->GetChain(0);
  for(int i=0;i<100000;i++){
    if(rand()%100==0){
      s4 = s2;
      s2 = s3;
      s3 = s4;

      kt1->LoadChain(s1,s2,0);
      kt2->LoadChain(s1,s2,0);
      c1 = kt1->GetChain(0);
      c2 = kt2->GetChain(0);
    }

    c1->RandomAngles(angles);
    c1->ForwardKinematics(angles,pos);
    c2->Update(angles,pos);
  }
  
  at2->StreamTree(cout);
//   kt->LoadChain(s1,s2,0);
//   cout<<"n joints: "<<kt->GetChain(0)->GetNbJoints()<<endl;
//   for(int i=0;i<kt->GetChain(0)->GetNbJoints();i++){
//     cout<<kt->GetChain(0)->Get
  



#ifdef GRAPHICAL
  cout<<"drawing"<<endl;
  DrawBodyApplication app(argc,argv);
  app.Init();
  app.SetBody(st);
  app.Run();
#endif
 cout<<"done";
  return 0;
}
#endif


#ifdef TEST_TREES2
// int func(pArticulatedTree_t t1,pArticulatedTree_t t2,void *arg){
//   //  cout<<t1->name<<" : "<<t2->name<<endl;
//   KinematicTree kt;
//   return kt.CollisionDetection(10,t1,t2,arg);
// }


int main(int argc, char *argv[]){
   pArticulatedTree_t at;
   int n=0;
   float l1,l2,dist;
   collision_arg_t arg;
   arg.dist = 10;
   KinematicTree *kt = new KinematicTree(argv[1]);
   at =kt->GetArticulatedTree();
   srand(time(NULL));
   do{
     at->RandomAngle();
     at->GetJoint()->SetAngle(0);
     n++;
   } while(!at->CheckPairsOfNodes(KinematicTree::CollisionDetection,&arg));
   cout<<"nb it "<<n<<endl;
   cout<<*at<<endl;

//    KinematicChain kc;
//    //   at->RandomAngle();
//    kt->LoadChain("r_sfe","r_wrist",&kc);
//    dist = kc.CollisionDetection(l1,l2);
//    cout<<"res "<<dist<<" "<<l1<<" "<<l2<<endl;
//    kt->LoadChain("r_wrist","r_sfe",&kc);
//    dist = kc.CollisionDetection(l1,l2);
//    cout<<"res "<<dist<<" "<<l1<<" "<<l2<<endl;
   
   return n;
}


#endif
