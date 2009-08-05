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
#ifndef __KINEMATIC_CHAIN_H__
#define __KINEMATIC_CHAIN_H__

#include "RigidTransfo.h"

#define KC_INVERTED
#define BLOCK_KC
#define WITH_LAST_LINK


#define MAX_LINKS 20 /**< max number of links >*/

//#define OLD_UPDATE

enum modality_t{
  UNSPECIFIED_MODALITY = 0,
  TOUCH_MODALITY,
  VISION_MODALITY
};

/**
 * This class contains a chain of rigid transformations and ways of adapting it, as well
 * as forward and inverse kinematic functions. Each rotation represents a rotative joint
 * and the translations are vectors between two joints. Joints are numbered from the
 * most proximal (0) to the most distal one (n-1). A KinematicChain can have at most
 * MAX_LINKS joints
 */

class KinematicChain{

 protected:
#ifdef BLOCK_KC
    /**
     * used of code optimization purposes
     */
    RigidTransfo* joints_block[MAX_LINKS];//used of code optimization purposes
    
    /**
     * An array of pointer to rigid transformation representing the links and joints
     * of a kinematic chain. It points to the first joint of the chain in joints_block
     */
    RigidTransfo **joints;
    int reverse_block[MAX_LINKS];
    
    /**
     * if reverse[i] == -1 means that this rigid transfo is to be inverted. Useful 
     * when loading a KinematicChain from an ArticulatedTree in ascending order.
     * See Hersch et al, IJHR 2008 on http://infoscience.epfl.ch/record/117918  
     * Fig 7 for more explanations. 
     */
    int *reverse;
#else
    RigidTransfo* joints[MAX_LINKS];
    int reverse[MAX_LINKS];
#endif

  int nb_joints;
#ifdef WITH_LAST_LINK
  Translation *last_link; //last link
    int own_last_link; // flag indicating if last link belongs to a kinematic tree
#endif
  modality_t modality;
  float tol;//inverse kin tolerance


    


 protected:
  void RotationStack(CQuat_t *qstack);

/**
 * computes the position of a point in the frames of reference of all joints.
 * \param pos the position of the point in the frame of reference of the most
 * distal joint (the last_link if there is one)
 * \param pstack where to put all the results. After the call pstack[i]
 * contains the value after it was transformed by joint i,i.e in
 * the frame of ref of joint i-1
 */
  void ForwardKinematicsStack(CVector3_t pos,CVector3_t *pstack);


    /**
     * computes the position of a point in the frames of reference of all joints.
     * \param pos the position of the point in the frame of reference of the world
     * \param pstack where to put all the results. After the call ikstack[i]
     * contains the values after it was inverse transformed by joints up to i-1,
     * i.e. in the frame of ref of joint i-1
     */
    void InverseKinematicsStack(CVector3_t pos,CVector3_t *pstack);


    /**
     * computes the value to feed into ForwardKinematicStack
     * (depending on #ifdef WITH_LAST_LINK)
     */
    void InitKinematicStack(const CVector3_t local_pos,CVector3_t out)const;
public:
    KinematicChain(int n=0);
    virtual ~KinematicChain();

    /**
     * Deallocated memory for rotations and translations
     */
    void FreeChain();

    /**
     * \return the number of joints in the chain
     */
    int GetNbJoints()const{return nb_joints;};
    
    /**
     * Sets the translation vector of a given link 
     * \param i link index (assumed to be > 0)
     * 
     */
    int SetLink(int i, CVector3_t link);

#ifdef WITH_LAST_LINK
    /**
     * Adds a last link to the chain (so that it end up with a Translation)
     * \param trans a pointer to the translation to add 
     */
    void AddLastLink(Translation *trans)//{last_link=trans;}
    {if(last_link)delete last_link;last_link=trans;own_last_link =0;} //to avoid memory leak
 
    /**
     * Sets the value of the last translation
     */
    void SetLastLink(const CVector3_t llink){last_link->SetTranslation(llink);};
    
  Translation* GetLastLink()const{return last_link;};
#endif
  int SetAllLinks(CVector3_t *links);

  // int GetLinks(int i, CVector3_t link);

  /**
   * Sets the rotation axis of a given joint
   * \param i joint index (assumed to be >= 0)
   */  
  int SetRotationAxis(int i, CVector3_t axis);

    /**
     * Sets the rotation axis of a given joint
     * \param i joint index (assumed to be >= 0)
     */
    void SetRotationAxis(int i,float x,float y,float z)
    {CVector3_t a;v_set(x,y,z,a);SetRotationAxis(i,a);};
   
    /**
     * Get the rotation axis of a given joint
     * \param i joint index (assumed to be >= 0)
     * \param axis where to put the rotation axis
     */   
    int GetRotationAxis(int i, CVector3_t axis);
    
    /**
     * Get the rotation angle of a given joint
     * \param i index of the joint(assumed to be >= 0)
     * \return the rotation angle in rad
     */
    float GetAngle(int i);
    
    /**
     * Get the rotation angles of all joints
     * \param angles where to put to the angles (in rad)
     */
    void GetAngles(float *angles)
    {for(int i=0;i<nb_joints;i++)angles[i]=GetAngle(i);}

    /**
     * Set the rotation angles of all joints
     * \param all desired angles (in rad)
     */
    void SetAngles(float *angles);

    /**
     * Set the rotation angle of a given joint
     * \param i index of the joint(assumed to be >= 0)
     * \param angle the desired angle in rad
     */
    void SetAngle(int i,float angle){joints[i]->SetAngle(angle);}
    
    /**
     * Set the rotation axes of all joints
     * \param axes a pointer to an array of CVector3_t. Is assumed to
     * be of the proper length (i.e nb_joints)
     */
    int SetAllRotationAxes(CVector3_t *axes);

    /**
     * scales all the learning rates
     */
    void ScaleRate(float factor);

    /**
     * Computes the forward kinematics for a given angle configuration
     * \angles the desired angle configuration
     * \param pos where to put the result
     * \local_pos the point in the frame of reference of the end-effector, which position
     * we want to compute. Default is zero 
     */
    void SetForwardKinematics(float *angles, CVector3_t pos, CVector3_t local_pos=NULL){
        SetAngles(angles);ForwardKinematics(pos,local_pos);};
    
    /**
     * Computes the actual position of the end-effector
     * \param pos where to put the result
     * \local_pos the point in the frame of reference of the end-effector, which position
     * we want to compute. Default is zero 
     */
    void ForwardKinematics(CVector3_t pos,  CVector3_t local_pos=NULL)const;
    
    /**
     * \brief CCD inverse kinematics function for position constraints
     * Performs Cyclic Coordinate Descent global inverse kinematics (see Wang and Chen (1991)
     * or my (M. Hersch) thesis for a description of the algorithm
     *  
     * \param pos the target position 
     * \return distance to target position
     */
    float InverseKinPosCCD(CVector3_t pos);

    /**
     * Performs CCD inverse kinematics function for orientation constraints
     * Performs Cyclic Coordinate Descent global inverse kinematics (see Wang and Chen (1991)
     * or my (M. Hersch) thesis for a description of the algorithm. The formula is slightly
     * different from the original, it was derived differently. 
     *  
     * \param q the target orientation, more precisely the quaternion describing the desired
     * total rotation of the end-effector
     */
    void InverseKinRotCCD(CQuat_t q);
    
    /**
     * Performs CCD inverse kinematics function for position and orientation constraints
     * Performs Cyclic Coordinate Descent global inverse kinematics (see Wang and Chen (1991)
     * or my (M. Hersch) thesis for a description of the algorithm
     *  
     * \param pos the target position 
     * \param rot the target orientation, more precisely the quaternion describing the desired
     * total rotation of the end-effector 
     * \return distance to target position
     */
    float InverseKinematicsCCD(CVector3_t pos,CQuat_t rot);
    
    /**
     * Computes the position jacobian of the kinematic chain
     * \param jac the matrix where to put the jacobian. Its size is 
     * 4*n, where n is the number of joints the chain. The numbers
     * are arranged as jac[0]=dx0/dth0,jac[1]=dx1/dth0,jac[2]=dx2/dth0...
     *   jac[4]= dx0/dth1,... So columns start every four elements and the
     * last element of each column (i.e. jac[3], jac[7]..) are not as x has only 3 component.
     * those elements are there for historical reasons. 
     */
    void GetJacobian(float *jac);


    /**
     * Computes the full (position and orientation) jacobian of the kinematic chain
     * \param jac the matrix where to put the jacobian. Its size is 
     * 6*n, where n is the number of joints the chain. The numbers
     * are arranged as jac[0]=dx0/dth0,jac[1]=dx1/dth0,jac[2]=dx2/dth0...
     *   jac[6]= dx0/dth1,... So columns start every six elements. 
     * The first three rows correspond to the position and last three correspond to the
     * orientation, using the complex part of the quaternion. See Hersch et al (2008), "Iterative rigid
     * body transformation estimation for visual tracking",VISAPP08 for more details on this representation  
     */
    void GetFullJacobian(float *jac);


    /**
     * Updates the kinematic chain rotation axes and translation vectors using static updating algo 
     * see Hersch et al.(2008) IJHR 
     * \param angles the configuration of the kinematic chain
     * \param vision the visual position of the end-effector 
     * \param local_pos the position of the marker with respect to the end-effector
     * \return the distance between the seen and the predicted end-effector position. 
     */
    float Update(float *angles, CVector3_t vision,CVector3_t local_pos=NULL)
    {return Update(angles,vision,FLT_MAX,local_pos);};
    

    /**
     * Updates the kinematic chain rotation axes and translation vectors using static updating algo 
     * see Hersch et al.(2008) IJHR 
     * \param angles the configuration of the kinematic chain
     * \param vision the visual position of the end-effector 
     * \param threshold the maximal distance between seen and predicted position of the end-effector
     * for the update to take place. Used to discard outliers in the vision data
     * \param local_pos the position of the marker with respect to the end-effector
     * \return the distance between the seen and the predicted end-effector position. If the sign is negative
     * this distance is above the threshold and the chain was not updated (bad visual tracking assumed). 
     */ 
    float Update(float *angles, CVector3_t vision, float threshold,CVector3_t local_pos=NULL);
   

    /**
     * Updates the kinematic chain rotation axes and translation vectors using kinetic updating algo, by
     * comparing and joint angle displacement with the corresponding visual displacement. See Micha Hersch's
     * thesis for the formula 
     *  
     * \param angle_pos the configuration of the kinematic chain (in rad)
     * \param angle_disp the angular displacement for all joints (in rad)
     * \param visual_disp the visual displacement of the end-effector 
     * \param threshold the maximal distance between seen and predicted displacement of the end-effector
     * for the update to take place. Used to discard outliers in the vision data
     * \param local_pos the position of the marker with respect to the end-effector
     * \return the distance between the seen and the predicted end-effector displacement.
     * If the sign is negative
     * this distance is above the threshold and the chain was not updated (bad visual tracking assumed). 
     */ 
    float UpdateWithJacobian(float *angle_pos, float *angle_disp, CVector3_t visual_disp, 
			   float threshold, CVector3_t local_pos=NULL);
    
    
    /**
     * Updates the kinematic chain rotation axes  using kinetic updating algo, by
     * comparing and joint angle displacement with the corresponding visual rotation. See Micha Hersch's
     * thesis for the formula 
     *  
     * \param angle_pos the configuration of the kinematic chain (in rad)
     * \param angle_disp the angular displacement for all joints (in rad)
     * \param visual_disp the rotation resulting from the displacement
     * \return the a measure of distance between the seen and the predicted end-effector rotation.
     * If the sign is negative
     * this distance is above the threshold and the chain was not updated (bad visual tracking assumed). 
     */ 
    
    float UpdateWithFullJacobian(float *angle_pos, float *angle_disp,RigidTransfo *visual_disp);
    float UpdateWithFullJacobian(float *angle_pos, float *angle_disp,float *visual_disp)
    {RigidTransfo r;r.Rotation::SetTransfo(visual_disp);
        return UpdateWithFullJacobian(angle_pos,angle_disp,&r);}
    void LinkRef(float *angle,int n, CVector3_t in, CVector3_t out);
    int GetLink(int i,CVector3_t link);
    RigidTransfo *GetTransfo(int i)const{return joints[i];}
    int AddJoint(RigidTransfo *t,int inv=1){reverse[nb_joints]=inv;joints[nb_joints++]=t;return nb_joints;}
    void Clear(){memset((void *)joints,0,MAX_LINKS*sizeof(RigidTransfo *));nb_joints=0;}
    void RandomAngles(float *angles){for(int i=0;i<nb_joints;i++)angles[i]=RND(pi)-pi/2;}
    void RandomAxes();
    void SetRandomAngles(){float a[MAX_LINKS]; RandomAngles(a);SetAngles(a);}
    modality_t GetModality()const {return modality;}
    void SetModality(modality_t mod){modality=mod;}
    
    /**
   * @brief transforms a vector with the ith transformation (or link),
   * index starts at zero
   * @param i the index of the transformation to consider
   * @param in the vector to be transform
   * @param out where to put the result
   */
    void Transform(int i,CVector3_t in,CVector3_t out)const
    {if(reverse[i]==-1){joints[i]->InverseTransform(in,out);}else{joints[i]->Transform(in,out);}}
  
    void Rotate(int i,CVector3_t in,CVector3_t out)const
    {if(reverse[i]==-1){joints[i]->Rotation::InverseTransform(in,out);}
        else{joints[i]->Rotation::Transform(in,out);}}
    
    void CounterRotate(int i,CVector3_t in,CVector3_t out)const
    {if(reverse[i]!=-1){joints[i]->Rotation::InverseTransform(in,out);}
        else{joints[i]->Rotation::Transform(in,out);}}
    
    
    /**
     * @brief transforms a vector with the ith transformation (or link),
   * index starts at zero. Sets the rotation angle prior to performing 
   * the rotation. 
   * @param i the index of the transformation to consider
   * @param in the vector to be transform
   * @param angle the rotation angle
   * @param out where to put the result
   */
    void Transform(int i,CVector3_t in,float angle,CVector3_t out);
    
    void InverseTransform(int i,CVector3_t in,CVector3_t out)const
    {if(reverse[i]==-1){joints[i]->Transform(in,out);}else{joints[i]->InverseTransform(in,out);}}
    
    void Translate(int i,CVector3_t in,CVector3_t out)const
    {if(reverse[i]==-1){joints[i]->Translation::InverseTransform(in,out);}else{joints[i]->Translation::Transform(in,out);}}
    
    void GetTranslation(int i, CVector3_t t)
    {if(reverse[i]==-1){joints[i]->GetInverseTranslation(t);}else{joints[i]->GetTranslation(t);}}
    
    void GetQuaternion(int i, CQuat_t q) const
    {joints[i]->GetQuaternion(q);if(reverse[i]==-1){q_inv(q,q);}}
    
    void GetInverseQuaternion(int i, CQuat_t q) const
    {joints[i]->GetQuaternion(q);if(reverse[i]!=-1){q_inv(q,q);}}
    
    void QuaternionAxisAngleDerivative(int i,CMatrix4_t m)
    {joints[i]->QuaternionAxisAngleDerivative(m);if(reverse[i]==-1)m_rescale(-1,m,m);} 
    
    void QuaternionAxisDerivative(int i,CMatrix4_t m)
    {joints[i]->QuaternionAxisDerivative(m);if(reverse[i]==-1)m_rescale(-1,m,m);}

    void QuaternionAngleDerivative(int i,CQuat_t q)
    {if(reverse[i]==-1)joints[i]->InverseQuaternionAngleDerivative(q);
        else joints[i]->QuaternionAngleDerivative(q);}

#ifdef KC_INVERTED 
  int IsInverted(int i)const{return reverse[i];}
  void Invert(int i){reverse[i]*=-1;}
  void InvertAll(){for(int i=0;i<nb_joints;i++){Invert(i);}}
#endif
 int Copy(const KinematicChain& kc);
 int RemoveJoint(){if(nb_joints>0){nb_joints--;return 1;}return 0;}
 void DeleteJoint(){if(nb_joints>0){delete joints[--nb_joints];joints[nb_joints]=NULL;}}
 float GetTolerance()const{return tol;}
    void SetTolerance(float t){tol = t;}
 /**
  * @brief computes the composed of all rigid transformation of the chain
  * @param rot where to put the resulting transformation
  */
 void GlobalTransfo(RigidTransfo& rt);
 /**
  * @brief computes the composed of all rotations of the chain
  * @param rot where to put the resulting rotation
  */
 void GlobalRotation(Rotation& rot);
 void GlobalTranslation(CVector3_t trans){ForwardKinematics(trans);}

/**
 * @brief checks for a collision between the two extremal links of the chain
 */
 virtual float CollisionDetection(float& l1, float& l2);
 float UpdateTouch(float *angle, float k0, float k1);

    float ScaleChain(float len);
    void PrintAxes(ostream& out);
    

    /**
     * \brief serializes a kinematic chain into a float array
     * \param offset joint with which to start (for partial kinematic chain serialization) 
     */
    float *Serialize(float *data,int offset=0);

    /**
     * \brief reads a kinematic chain from a float array
     * \param offset joint with which to start in this chain (for partial kinematic chain updating) 
     */
    int Deserialize(const float *data,int offset=0);

    /**
     * \brief reads a kinematic chain from a float array
     * \param from where to start reading in the data
     * \param to where to finish reading in the data
     * \return the number of joints read
     */
    int Deserialize(const float *data,int from, int to);
};

ostream& operator<<(ostream& out, const KinematicChain& kc);
istream& operator>>(istream& in, KinematicChain& kc);
inline void SkipComments(istream& in){while(in.peek()=='#'){ while(in.get()!='\n'){}}}
#endif
