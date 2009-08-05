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
#ifndef __K_CHAIN_BODY_SCHEMA_H__
#define __K_CHAIN_BODY_SCHEMA_H__


#define WITH_LAST_LINK

#include "BodySchema.h"
#include "KinematicChain.h"


/**
 *  This class is a wrapper for KinematicChain, so that it becomes a BodySchema
 */
class KChainBodySchema : public BodySchema, public KinematicChain{
    
    int telescopic;

public:
    KChainBodySchema():BodySchema(),KinematicChain(){telescopic=1;};
    KChainBodySchema(int teles):BodySchema(),KinematicChain(){telescopic=teles;};
    virtual ~KChainBodySchema(){};
    KChainBodySchema(char *filename):BodySchema(),KinematicChain(){BodySchema::Load(filename);};
    //  KChainBodySchema(const KinematicChain& chain);
    int GetNbJoints(){return nb_joints;}
    virtual void Angle2Cart(joint_vec_t& angles,cart_vec_t& pos)
    {SetForwardKinematics(angles.GetArray(),pos.GetArray());}
    void Angle2Cart(cart_vec_t& pos)
    {ForwardKinematics(pos.GetArray());}
    virtual int InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles);
    virtual int InverseKinematics(const cart_vec_t& pos,joint_vec_t& start_angle,joint_vec_t& angles)
    {SetAngles(start_angle.GetArray()); return InverseKinematics(pos,angles);}
    virtual float TryInverseKinematics(const cart_vec_t& pos,joint_vec_t& angles);
    virtual void SetRandomAngle(joint_vec_t& angles)
    {GetRandomAngle(angles);SetAngles(angles.GetArray());}
    virtual void Jacobian(Matrix& jac);//jac is assumed to of the right dimension
    virtual void Jacobian(joint_vec_t& angles, Matrix& jac){SetAngles(angles.GetArray());Jacobian(jac);}
    virtual void Load(istream& in){in>>*((KinematicChain *)this);};
    virtual void Load(const char *filename);//{ifstream in(filename);Load(in);in.close();};  
    virtual void Print(ostream& out)const{out<<*((KinematicChain *)this);} 
    virtual void GetWeights(cart_vec_t& cart_weights, joint_vec_t& joint_weights);
    virtual void SetPosition(joint_vec_t& angles){SetAngles(angles.GetArray());};
    virtual void GetAngles(joint_vec_t& angles){KinematicChain::GetAngles(angles.GetArray());};
    virtual float *Serialize(float *data,int offset){return KinematicChain::Serialize(data,offset);};
    virtual int Deserialize(const float *data,int offset)
    {return KinematicChain::Deserialize(data,offset);};
    virtual int Deserialize(const float *data,int from,int to)
    {return KinematicChain::Deserialize(data,from,to);};
 
};



#endif
