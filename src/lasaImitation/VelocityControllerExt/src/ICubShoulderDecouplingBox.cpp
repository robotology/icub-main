// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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

#include "ICubShoulderDecouplingBox.h"

ICubShoulderDecouplingBox::ICubShoulderDecouplingBox(){
    Init();
}
ICubShoulderDecouplingBox::~ICubShoulderDecouplingBox(){
}
void ICubShoulderDecouplingBox::Init(){
    for(int i=0;i<3;i++){
        mDecFactors[i]          = 0.0;
        mDecAccum[i]            = 0.0;
        mDecVel[i]              = 0.0;
        mDecLearningFactor      = 0.000004;
        mDecTimeConstant        = 0.04;
        
        mVCEstSetPoint[i]       = 0.0;
        mEstPos[i]              = 0.0;
        mEstVel[i]              = 0.0;
        mEstAcc[i]              = 0.0;
        mEstCoefs[i][0]         = MAX_EST_COEFS0/2;
        mEstCoefs[i][1]         = MAX_EST_COEFS1/2;
        mEstLearningFactor0     = 0.00001;
        mEstLearningFactor1     = 0.00001;
        
        mSetPointResetFactor    = 0.5;
    }
    
}
void ICubShoulderDecouplingBox::Decouple(Vector &inputVel, Vector &outputVel, double dt){
    if(dt>0.001){
        double oneOnDt = 1.0/dt;
        double dtOnTC  = dt/mDecTimeConstant;
        mDecVel[0] = 0.0;
        mDecVel[1] = 0.0;
        mDecVel[2] = 0.0;
        
        double dAcc[3];

        dAcc[0] = (-mDecAccum[0] + mDecFactors[0] * inputVel[0]*fabs(inputVel[0]))*dtOnTC;
        mDecAccum[0] += dAcc[0];
        mDecVel[1] = dAcc[0]*oneOnDt;
        
        dAcc[1] = (-mDecAccum[1] + mDecFactors[1] * inputVel[0]*fabs(inputVel[0]))*dtOnTC;
        mDecAccum[1] += dAcc[1];
        mDecVel[2] = dAcc[1]*oneOnDt;

        dAcc[2] = (-mDecAccum[2] + mDecFactors[2] * inputVel[1]*fabs(inputVel[1]))*dtOnTC;
        mDecAccum[2] += dAcc[2];
        mDecVel[2]+= dAcc[2]*oneOnDt;

        outputVel[0] = inputVel[0] + mDecVel[0];
        outputVel[1] = inputVel[1] + mDecVel[1];
        outputVel[2] = inputVel[2] + mDecVel[2];
        
        
        for(int i=0;i<3;i++){
            outputVel[i] = inputVel[i] + mDecVel[i];
        }
    }
}

void ICubShoulderDecouplingBox::LearnDecoupling(Vector &targetVel, Vector &currPos, Vector &currVel, double dt){

    ProcessModel(targetVel,dt);
    
    mDecFactors[2] +=  mDecLearningFactor * currVel[1]*(mEstPos[2]-currPos[2])*dt;
    if(mDecFactors[2]>MAX_DEC_FACTOR)   mDecFactors[2] = MAX_DEC_FACTOR;
    if(mDecFactors[2]<0.00)             mDecFactors[2] = 0.00;

    mDecFactors[0] +=  mDecLearningFactor * currVel[0]*(mEstPos[1]-currPos[1])*dt;
    if(mDecFactors[0]>0.00)             mDecFactors[0] = 0.00;
    if(mDecFactors[0]<-MAX_DEC_FACTOR)  mDecFactors[0] =-MAX_DEC_FACTOR;

    mDecFactors[1] +=  mDecLearningFactor * currVel[0]*(mEstPos[2]-currPos[2])*dt;
    if(mDecFactors[1]>0.00)             mDecFactors[1] = 0.00;
    if(mDecFactors[1]<-MAX_DEC_FACTOR)  mDecFactors[1] =-MAX_DEC_FACTOR;
}

void ICubShoulderDecouplingBox::ResetSetPoint(Vector &currPos){
    for(int i=0;i<3;i++){
        mVCEstSetPoint[i] = currPos[i];
    }
}

void ICubShoulderDecouplingBox::UpdateSetPoint(Vector &targetVel, Vector &currPos, double dt){
    for(int i=0;i<3;i++){
        mVCEstSetPoint[i] += targetVel[i]*dt;
        mVCEstSetPoint[i] += (-mVCEstSetPoint[i] + currPos[i])*dt*mSetPointResetFactor;
        if(mVCEstSetPoint[i]> MAX_SETPOINT_POSITION)        mVCEstSetPoint[i]= MAX_SETPOINT_POSITION;
        else if(mVCEstSetPoint[i]<-MAX_SETPOINT_POSITION)   mVCEstSetPoint[i]=-MAX_SETPOINT_POSITION;
    }
}

void ICubShoulderDecouplingBox::LearnModel(Vector &targetVel, Vector &currPos, Vector &currVel, Vector &currAcc, double dt, int index){

    for(int i=0;i<3;i++){
        if(index>=0){
            if(i!=index) continue;
        }
        double dE   = currAcc[i] - mEstCoefs[i][0]*(mVCEstSetPoint[i]-currPos[i]) + mEstCoefs[i][1]*currVel[i];
        double dea  = -dE * (currPos[i]-mVCEstSetPoint[i]);
        double deb  = -dE * currVel[i];
        
        mEstCoefs[i][0] += mEstLearningFactor0 * dea;// * fabs(currVel[i]);
        if(mEstCoefs[i][0]<0.0)                 mEstCoefs[i][0] = 0.0;
        else if(mEstCoefs[i][0]>MAX_EST_COEFS0) mEstCoefs[i][0] = MAX_EST_COEFS0;

        mEstCoefs[i][1] += mEstLearningFactor1 * deb;// * fabs(currVel[i]);
        if(mEstCoefs[i][1]<0.0)                 mEstCoefs[i][1] = 0.0;
        else if(mEstCoefs[i][1]>MAX_EST_COEFS1) mEstCoefs[i][1] = MAX_EST_COEFS0;
    }
}

void ICubShoulderDecouplingBox::ProcessModel(Vector &targetVel, double dt){
    
    for(int i=0;i<3;i++){
        mEstAcc[i] = mEstCoefs[i][0]*(mVCEstSetPoint[i]-mEstPos[i]) - mEstCoefs[i][1]*mEstVel[i];
        if(mEstAcc[i]> MAX_EST_ACCEL)           mEstAcc[i]= MAX_EST_ACCEL;
        else if(mEstAcc[i]<-MAX_EST_ACCEL)      mEstAcc[i]=-MAX_EST_ACCEL;

        mEstVel[i] += mEstAcc[i]*dt;
        if(mEstVel[i]> MAX_EST_VELOCITY)        mEstVel[i]= MAX_EST_VELOCITY;
        else if(mEstVel[i]<-MAX_EST_VELOCITY)   mEstVel[i]=-MAX_EST_VELOCITY;

        mEstPos[i] += mEstVel[i]*dt;
        if(mEstPos[i]> MAX_EST_POSITION)        mEstPos[i]= MAX_EST_POSITION;
        else if(mEstPos[i]<-MAX_EST_POSITION)   mEstPos[i]=-MAX_EST_POSITION;
    }

}

