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
#ifndef __ROTATION_H__
#define __ROTATION_H__

#include "mathlib.h"
#include <assert.h>



/**
 * This class represents a rotation using some kind of (non-standard) quaternion parametrization
 * of rotation. See Hestenes, "New Foundations for Classical Mechanics",
 * pp 280 for the mathematical formalism 
 * 
 */
class Rotation {
  
protected:
  
    CVector3_t beta; /**< a vector whose norm is sin(theta/2) and direction is the rotation axis*/
    float alpha; /**< cos(theta/2)*/ 


    mutable CVector3_t axis; /**< the rotation axis*/
    mutable float norm;  //sin(theta/2)
    /**
     * specfies if axis and norm values are reliable.
     */
    mutable int axis_flag;


    float eps_b; /**< the learning rate for beta */
    float eps_a; /**< the learning rate for the axis */

    float base_eps_b;
    float base_eps_a;


    /**
     *@brief computes the matrix ...
     */

    void ComputeMatrixC(const CVector3_t v, CMatrix3_t out)const ;

    /**
     * @brief constructs the matrix d/db (b x v)
     * @param v the second factor of the product
     * @param out the resulting matrix
     */  
    void ComputeMatrixB(const CVector3_t v, CMatrix3_t out)const;



public:
    Rotation();
    virtual ~Rotation();
    int GetRotationAxis(CVector3_t axis)const;
    const float* GetRotationAxis()const{if(!AxisOk())SetAxis(); return axis;};
    virtual int SetTransfo(CVector3_t beta);
    void GetRotationParam(CVector3_t param)const{v_copy(beta,param);};
    const float *GetRotationParam()const{return beta;}
    void GetQuaternion(CQuat_t q)const {v_copy(beta,q);q[3]=alpha;};
    void GetInverseQuaternion(CQuat_t q)const {v_scale(beta,-1,q);q[3]=alpha;};
    void SetQuaternion(CQuat_t q);
    float GetRotationAngle()const;
    void Derivative(CMatrix3_t out) const;
    int SetAxis()const {norm = v_normalize(beta,axis);return axis_flag = (norm>epsilon);}
    int AxisOk()const{return axis_flag;}
    void CheckAxis()const{if(!AxisOk())SetAxis();}
    
    
    
    /**
     * @brief inverts the rotation angle
     */ 
    void InvertRotationAngle(){v_scale(beta,-1,beta);norm = -norm;}
    

    /**
     * @brief provides the rotation angle in order to align as much as possible two
     * vectors. This the updating step of the CCD inverse kinematics algorithm.
     * The resulting rotation (around a fixed axis) will rotate <with> in order to 
     * align it with <tar>
     * @param tar target direction of alignment (in an external frame of ref)
     * @param with direction to align to the target (in a frame of ref attached to the
     * zero position of the rotation)
     */
    float AimingAngle(const CVector3_t tar, const CVector3_t with);
    
    
    /**
     * @brief updates the rotation angle in order to align as much as possible two
     * vectors. This the updating step of the CCD inverse kinematics algorithm.
     * The resulting rotation (around a fixed axis) will rotate <with> in order to 
     * align it with <tar>
     * @param tar target direction of alignment (in an external frame of ref)
     * @param with direction to align to the target (in a frame of ref attached to the
     * zero position of the rotation)
     */
    void AimAt(const CVector3_t tar, const CVector3_t with){SetAngle(AimingAngle(tar,with));};
    
    
    /**
     * @brief computes dR(v)/dtheta
     * @param v the transformed vector (see above)
     * @param out the vector where to put the result 
     */ 
    void AngleDerivative(CVector3_t v, CVector3_t out);
    
    /**
     * @brief computes dR(v)/dnorm
     * @param v the transformed vector (see above)
     * @param out the vector where to put the result 
     */ 
    void NormDerivative(CVector3_t v, CVector3_t out);
 
    /**
     * @brief computes dR^{-1}(v)/dnorm
     * @param v the transformed vector (see above)
     * @param out the vector where to put the result 
     */ 
    void InverseNormDerivative(CVector3_t v, CVector3_t out);
    /**
     * @brief computes dR(v)/dbeta
     * @param v the transformed vector (see above)
     * @param out the matrix where to put the result 
     */ 
    void BetaDerivative(CVector3_t x, CMatrix3_t out)const;

    /**
     * @brief computes dR(v)/daxis
     * @param v the transformed vector (see above)
     * @param out the matrix where to put the result 
     */
    void AxisDerivative(CVector3_t x, CMatrix3_t out);
 
    /**
     * @brief computes dR^{-1}(v)/daxis
     * @param v the transformed vector (see above)
     * @param out the matrix where to put the result 
     */
    void InverseAxisDerivative(CVector3_t v, CMatrix3_t out);
  
    void AngleAxisDerivative(CVector3_t v, CMatrix3_t out);
 
    void Jacobian(CMatrix3_t out)const;


    /**
     * @brief update beta in a given direction
     */
    void Add(CVector3_t dbeta);

    /**
     * @brief updates the rotation axis in a given direction
     */
    void AddToAxis(CVector3_t daxis);
    /**
     * @brief updates the rotation angle (actually the norm of beta) 
     * of a given amount
     */
    void AddToNorm(float dnorm);

    /**
     * Set the rotation from a matrix
     * @param m rotation matrix (orthogonal matrix is assumed)
     */ 
    void  RotFromMatrix(const CMatrix3_t m);

    /**
     * @brief Sets the matrix from the rotation. It can be a rotation
     * or a homogeneous matrix
     */
    void  RotToMatrix(CMatrix3_t m)const;//{CQuat_t q;GetQuaternion(q);q_to_matrix(q,m);}

    /**
     * @brief sets the  parameter of the transformation
     * @param beta the new rotation parameter
     */


    void Copy(const Rotation& r);

    /**
     * @brief performs a gradient descent on the beta parameter of
     * the rotation and on the translation in order to have a rigid
     * transformation that brings v closer
     * to v_tr. Only the direction of the gradient of beta is taken
     * into account, not its size.
     * @param v vector a point that can be transformed
     * @param v_tr the observed image of v
     */
    void Update(CVector3_t v, CVector3_t v_tr);


    void UpdateAxis(CVector3_t v, CVector3_t v_tr,float angle);

    /**
     * @brief sets the learnig rate sizes
     * @param new_eps rotation step size
     * @param new_eps_tr translation step size
     */
    virtual void SetRate(float new_eps);

    /** 
     * @briefscale the learning rate sizes 
     * @param factor the scaling factor (for both rotation and
     * translation learning ratea)
     */
    virtual void ScaleRate(float factor);

    /**
     * @brief Performs the rotation
     * @param in the vector to be transformed
     * @param out the resulting (transformed) vector
     */
    void Transform(const CVector3_t in, CVector3_t out)const;

    /**
     * @brief Performs the inverse rigid tranformation
     * @param in the vector to be transformed
     * @param out the resulting (transformed) vector
     */
    void InverseTransform(const CVector3_t in, CVector3_t out);


    /**
     * @brief Performs compose this rotation with another one: beta_out = this*beta_in
     */
    void TransformRotation(const CVector3_t beta_in, CVector3_t beta_out);

    /**
     * @brief computes dq/dsin(theta/2) where q is the quaternion of the current rotation
     * @param out where to put the resulting quaternion
     */
    void QuaternionNormDerivative(CQuat_t out)const
    {CheckAxis();v_copy(axis,out);out[3]=alpha>epsilon?-norm/alpha:0;};
   
    /**
     * @brief computes dq/d(theta) where q is the quaternion of the current rotation
     * @param out where to put the resulting quaternion
     */
    void QuaternionAngleDerivative(CQuat_t out)const
    {float f=0.5*alpha;QuaternionNormDerivative(out);v4_scale(out,f,out);};
/**
     * @brief computes d(-q)/d(theta) where q is the quaternion of the current rotation
     * @param out where to put the resulting quaternion
     */
    void InverseQuaternionAngleDerivative(CQuat_t out)const
    {float f=0.5*alpha;InverseQuaternionNormDerivative(out);v4_scale(out,f,out);}; 
    /**
     * @brief computes dq/daxis where q is the quaternion of the current rotation
     * @param out where to put the resulting matrix
     */
    void QuaternionAxisDerivative(CMatrix4_t out){
        CheckAxis();m_identity(out);out[15]=0;m_rescale(norm,out,out);//can be optimized
    }
    void QuaternionAxisAngleDerivative(CMatrix4_t out)
    {CheckAxis(); float f=0.5*alpha;m_identity(out);out[15]=0;m_rescale(f,out,out);};//can be optimized

    void InverseQuaternionNormDerivative(CQuat_t out)const
    {CheckAxis();v_scale(axis,-1,out);out[3]=alpha>epsilon?-norm/alpha:0;};

    /**
     * @return 1 if angle is within ]-pi pi],) 0 otherwise and sets the correct 
     * angle 
     */
    int SetAngle(float angle);
    void SetNorm(float no){v_scale(axis,no,beta);alpha=sqrt(1-no*no);norm=no;}
    int SetRotationAxis(const CVector3_t a);
    int SetRotationParam(CVector3_t param);
    float GetAngle()const {return 2*asin(norm);}
    float GetAlpha()const {return alpha;}
    void Invert(){v_scale(beta,-1,beta);v_scale(axis,-1,axis);}
    void RandomAxis();
    //  void RandomAngle(){SetAngle(RND(2*pi)-pi);}
    void RandomAngle(){SetAngle(RND(pi)-pi/2);}
    void Identity(){v_clear(beta);alpha=1;}
    void MinimizeRotation(Rotation& rot);
    void MinimizeRotation(const CQuat_t q);
    float MinimizePositionAndRotationAngle(const CVector3_t tar, const CVector3_t with, const CQuat_t rot,float k=0.5);
    void MinimizePositionAndRotation(const CVector3_t tar, const CVector3_t with, const CQuat_t rot,float k=0.5)
    {SetAngle(MinimizePositionAndRotationAngle(tar,with,rot,k));};

    /** 
     * not the intuitive operator implementation (not a const function)  
     */
    Rotation& operator*(const Rotation& r);

};

/**
 * @brief output streaming operator
 */
ostream& operator<<(ostream& out, const Rotation& rt);


istream& operator>>(istream& in, Rotation& rt);
#endif
