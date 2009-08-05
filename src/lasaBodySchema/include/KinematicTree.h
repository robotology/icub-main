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
#ifndef __KINEMATIC_TREE__
#define __KINEMATIC_TREE__
/**
 * @author Micha Hersch 2007
 */



#define KINEMATIC_TREE_VIEW
#define NO_CSOLID


#ifdef NO_CSOLID
typedef void *  pCSolidTree_t;
#include "TreeParser.h"
#else
#include "SolidTree.h"
#endif


#include "KinematicChain.h"

#define COLLISION
#define WITH_RANGES

#define MAX_CHAINS 15


class ArticulatedTree;

typedef ArticulatedTree  ArticulatedTree_t, *pArticulatedTree_t; 
typedef vector<pArticulatedTree_t> ArticulatedTreeList_t;


/**
 * This class defines a tree of rigid transformation able to represent a body
 * schema
 */
class ArticulatedTree
{
 protected:
  RigidTransfo joint;
  ArticulatedTree *parent;
  ArticulatedTreeList_t children;

#ifdef WITH_RANGES
  float range[2];
#endif


 public:
  string name;
 public:
  ArticulatedTree();

  ArticulatedTree(const pTree tree);
  ArticulatedTree(const pArticulatedTree_t tree);
  virtual ~ArticulatedTree();
  void CopyTree(pArticulatedTree_t tree);
  void AddTree(pArticulatedTree_t ptree);
  virtual void StreamTree(ostream& out)const;
  int GetSize() const;
  //  void Invert(int i=0) {if(i*inverse<0){joint.Invert();inverse=i;}}
  pArticulatedTree_t FindJoint(string& name);
  pArticulatedTree_t FindSubTree(pArticulatedTree_t t, pArticulatedTree_t exclude=NULL);
 
  pArticulatedTree_t GetParent(){return parent;}
  ArticulatedTreeList_t *GetChildren(){return &children;}
  RigidTransfo *GetJoint(){return &joint;};
  void RandomAxis();
  void NoTranslation();
  void RandomAngle();
  void ZeroPosition();
  void Reshape(CVector3_t offset=NULL);
  void Reshape2(CVector3_t offset=NULL);
  void GetAxis(CVector3_t axis){joint.GetRotationAxis(axis);}
/**
 * @brief goes through all pairs of nodes in the tree and performed a given function
 * @param f the function performed with all pairs of nodes 
 * @param arg a pointer to an addition argument that is passed to f
 */
  int CheckPairsOfNodes(int (*f)(pArticulatedTree_t,pArticulatedTree_t,void *),void *arg);
  int CheckTreeNode1(pArticulatedTree_t tree,
			int (*f)(pArticulatedTree_t,pArticulatedTree_t,void *),void *arg);
  int CheckTreeNode2(pArticulatedTree_t tree,
			int (*f)(pArticulatedTree_t,pArticulatedTree_t,void *),void *arg);
  /**
   * @ brief puts all the joint angles into a list
   * @ param list the array of angles. Use GetSize to know with what size to initializze this array
   */
  int GetAngleList(float *list);
  int SetAngleList(float *list);
#ifdef WITH_RANGES
  int GetAngleRangeList(float *list_min, float *list_max);
    float GetLowerRange(){return range[0];}
    float GetUpperRange(){return range[1];}
    /**
     *\brief recursively looks in the tree for the ranges corresponding to the joints in a KinematicChain
     */
    int FindAngleRanges(float *lower_range, float *upper_range,KinematicChain *chain);
#endif

    /**
     * @brief Serializes the translation vectors and rotation axes. Assumes that the other
     * side (i.e. Deserialize) knows the structure of the tree. No information about joint
     * angles is provided
     */
    int Serialize(float *data,int data_size)const;
    /**
     * @brief Deserializes the translation vectors and rotation axes. Assumes that the other
     * side (i.e. Serialize) knows the structure of the tree. No information about joint
     * angles is extracted
     */
    int Deserialize(const float *data,int data_size);
  void PrintList();
  void PrintAngles();

    // not used here
#ifndef NO_CSOLID
  ArticulatedTree(const pCSolidTree_t stree);
  void FillCSolidTree(pCSolidTree_t solid);
 pCSolidTree_t CreateCSolidTreeStruct();
#endif


};



/**
 * This class contains and ArticulatedTree and the KinematicChains to adapt it.
 *  
 */
class KinematicTree
{
 protected:
  pTree xml_tree;
  pArticulatedTree_t root;
  pCSolidTree_t solid;
  KinematicChain chains[MAX_CHAINS];
  int nb_chains;

 protected:
  int LoadChainRec(pArticulatedTree_t from, pArticulatedTree_t to, KinematicChain *chain,
		   pArticulatedTree_t exclude=NULL)const;
public:
    KinematicTree();
    KinematicTree(const char *filename);
    virtual ~KinematicTree();
    void FreeChains(){for(int i=0;i<MAX_CHAINS;i++)chains[i].FreeChain();}
    

    /**
     * \brief Loads a chain that refers to a path in the kinematic tree
     * as described in Hersch et al. (2008), IJHR. 
     * ifdef WITH_LAST_LINK, the last rotation is not taken into account,
     * only the translation is put into last_link
     *\return the number of joints in the chain
     *
     */
  int LoadChain(pArticulatedTree_t from, pArticulatedTree_t to,
		int i,modality_t mod=UNSPECIFIED_MODALITY);
    /**
     *\return the number of joints in the chain
     */ 
 int LoadChain(pArticulatedTree_t from, pArticulatedTree_t to,
		KinematicChain *chain,modality_t mod=UNSPECIFIED_MODALITY)const;
    /**
     *\return the number of joints in the chain
     */ 
 int LoadChain(string& from, string& to, KinematicChain *chain,
		modality_t mod=UNSPECIFIED_MODALITY);
    /**
     *\return the number of joints in the chain
     */ 
 int LoadChain(string& from, string& to, int i,
		modality_t mod=UNSPECIFIED_MODALITY); 
    /**
     *\return the number of joints in the chain
     */ 
    int LoadChain(const char *from, const char *to, KinematicChain *chain,
                  modality_t mod=UNSPECIFIED_MODALITY)
    {string s1(from),s2(to);return  LoadChain(s1, s2, chain,mod);}
    /**
     *\return the number of joints in the chain
     */   
    int LoadChain(const char *from, const char *to,int i,
                  modality_t mod=UNSPECIFIED_MODALITY)
    {string s1(from),s2(to);return  LoadChain(s1, s2,i,mod);}
    
    KinematicChain *GetChain(int i){return &(chains[i]);}
    KinematicChain *GetNewChain(string& from, string& to,
			      modality_t mod=UNSPECIFIED_MODALITY);
  pArticulatedTree_t GetArticulatedTree(){return root;}

    /**
     * \brief yields the size of a kinematicTree(number of nodes)
     * \returns the number of nodes in a tree
     * 
     **/
    int GetTreeSize(){return root?root->GetSize():0;}  
  virtual int CheckAllCollisions(KinematicTree *update_tree);

/**
 * @brief detects if  there is a collision between link t1 and t2 and possibly updates 
 * a articulated tree accordingly. This works
 * by creating a KinematicChain from t1 to t2 and computing the position of
 * link t1 in the frame of ref of t2. Consecutive links are discarded, as they
 * touch each other at the joint
 * @param dist the distance below which contact is assumed 
 * @param t1 the first link
 * @param t2 the second link
 * @return 1 if there is a collision, 0 otherwise
 */
  static int CollisionDetection(pArticulatedTree_t t1, pArticulatedTree_t t2, void *arg);
};


ostream& operator<<(ostream& out, const ArticulatedTree& at);


typedef struct {
  KinematicChain* kc;
  KinematicTree *kt;
  CVector3_t pos_stack[MAX_LINKS];
  CVector3_t *top;
  pArticulatedTree_t t1;
  pArticulatedTree_t t2;
  RigidTransfo rt;
  float dist;
  KinematicTree *kt_update;
  KinematicChain kc_update;
} collision_arg_t;


#endif
