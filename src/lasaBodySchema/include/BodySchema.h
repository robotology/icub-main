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
#ifndef __BODY_SCHEMA_H__
#define __BODY_SCHEMA_H__

#include "assert.h"
#include "Vector.h"
#include "Matrix.h"
#include <iostream>
#include <fstream>

using namespace std;

extern int cartesian_dim;
extern int joint_angle_dim;

class ConfigurationVector: public Vector{
public:
    ConfigurationVector():Vector(joint_angle_dim){};
    ConfigurationVector& operator=(const Vector& v){if(v.Size()==row)
            {Vector::operator=(v);}else{cout<<"bad joint space vector dimension"<<endl;}return *this;}
};


class PositionVector: public Vector{
public: 
    PositionVector():Vector(cartesian_dim){};
    PositionVector& operator=(const Vector& v){if(v.Size()==row)
            {Vector::operator=(v);}else{cout<<"bad cartesian space vector dimension"<<endl;}return *this;}
};

typedef ConfigurationVector joint_vec_t;

typedef PositionVector cart_vec_t;


/**
 * a class containing all the arm information so that it can be used by the Reaching class
 * This is a virtual class, instanciated by KChainBodySchema
 */
class BodySchema
{
protected:
    /**
     * joint angle lower limits
     */
    joint_vec_t min_angle;
    /**
     *joint angle upper limits
     */
    joint_vec_t max_angle;
    

    int ik_trials;
public:
    
    BodySchema()
    {min_angle.Zero();min_angle-= PIf/2; max_angle.Zero();max_angle += PIf/2;ik_trials=10; };
    void SetIkTrials(int n){ik_trials = n;};
    virtual ~BodySchema(){};
    virtual int GetNbJoints(){return joint_angle_dim;}
    virtual void Angle2Cart(joint_vec_t& angle, cart_vec_t& out){};
    virtual void Angle2Cart(cart_vec_t& out){};
    virtual void Jacobian(Matrix& jac){};
    /**
   * @brief full jacobian for position and orientation
   */
    virtual void FullJacobian(Matrix& jac){};
    virtual void Jacobian(joint_vec_t& v, Matrix& jac){};
    virtual void Print(ostream& out)const{out<<"no printing function defined"<<endl;};
    virtual void Load(istream& in){};
    virtual void Load(const char *filename){ifstream fin;fin.open(filename);Load(fin);fin.close();};  
    virtual void GetAngles(joint_vec_t& angles){};
    virtual void GetRandomAngle(joint_vec_t& angles)
    {angles.Random(); min_angle.Add(angles^(max_angle-min_angle),angles);}
    virtual void SetRandomAngle(joint_vec_t& angles){};
    virtual int InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles){return 0;};
    virtual int InverseKinematics(const cart_vec_t& pos,joint_vec_t& start_angle,joint_vec_t& angles){return 0;};
    virtual void GetWeights(cart_vec_t& cart_weights, joint_vec_t& joint_weights){};
    virtual void SetAnglesInRange(joint_vec_t& angle){
        int n=min(angle.Size(),min_angle.Size());//min_angle and max_angle are assumed to be the same size
        for(int i=0;i<n;i++)angle[i] = max(min_angle[i],min(max_angle[i],angle[i]));};
    
    /**
     * param angle assumed to be in [-pi pi]
     */
    float AngleInClosestRange(int i,float cand){
        if(cand > max_angle[i])return cand-max_angle[i]<min_angle[i]+2*PIf-cand? max_angle[i]:min_angle[i];
        if(cand < min_angle[i])return min_angle[i]-cand<cand-max_angle[i]+2*PIf? min_angle[i]:max_angle[i];
        return cand;
    }
    
    virtual void UpdateDimension(){min_angle.Resize(joint_angle_dim);max_angle.Resize(joint_angle_dim);}
    
    virtual int AnglesInRange(joint_vec_t& angles)
    {for(unsigned int i=0;i<angles.Size();i++){if(angles[i]<min_angle[i] || angles[i]>max_angle[i])return 0;}return 1;}
    virtual void SetPosition(joint_vec_t& angles){};
    virtual void GetAnglesLowerBound(joint_vec_t& lb){lb =min_angle;};
    virtual void GetAnglesUpperBound(joint_vec_t& ub){ub =max_angle;};
    virtual void SetAnglesLowerBound(joint_vec_t& lb){min_angle=lb;};
    virtual void SetAnglesUpperBound(joint_vec_t& ub){max_angle=ub;};
    void SetAngleBounds(joint_vec_t& lb, joint_vec_t& ub)
    {SetAnglesUpperBound(ub);SetAnglesLowerBound(lb);} 
    virtual joint_vec_t& GetAnglesLowerBound(){return min_angle;};
    virtual joint_vec_t& GetAnglesUpperBound(){return max_angle;};
    virtual float CartesianDistance(cart_vec_t& p1, cart_vec_t& p2){return (p1-p2).Norm();};
    /**
     * \brief serializes a kinematic chain into a float array
     * \param offset joint with which to start (for partial kinematic 
     * chain serialization) 
     */
    virtual float* Serialize(float *data, int offset){return NULL;};
    /**
     * \brief reads a kinematic chain from a float array
     * \param offset joint with which to start (for partial kinematic 
     * chain updating) 
     */
    virtual int Deserialize(const float *data,int offset){return 0;};
    /**
     * \brief reads a kinematic chain from a float array
     * \param from where to start reading in the data
     * \param to where to finish reading in the data
     * \return the number of joints read
     */  
    virtual int Deserialize(const float *data,int from, int to){return 0;};
};



/* ostream& operator<<(ostream& out, const BodySchema& bs){bs.Print(out);return out;}; */
/* istream& operator>>(istream& in, BodySchema& bs){bs.Load(in);return in;}; */


#endif
