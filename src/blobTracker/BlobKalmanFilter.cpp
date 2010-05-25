// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "BlobKalmanFilter.h"
#include <stdio.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;

BlobKalmanFilter::BlobKalmanFilter(int width, int height) : 
	width(width),
	height(height),	
	KalmanFilter(8, 8, 0) 
{
    
    setProcessNoiseCovariance(1e-3);
    width = width;
    height = height;
    
    //set up input
    measured.reset();
    
    //set up output
    filtered.reset();
    
	numNoFeatureFrames = 0;
    noFeatureThreshold = 50;
    hasFeatureFrames = 0;
    validThreshold = 10;
    
    old_x = 0; old_y = 0;
    x_dot = 0; y_dot = 0;
	framesElapsed  = 0;
            
	previousMeasuredVals.resize(measurementSize);
	predictedCovariance.resize(stateSize,stateSize);
    
    roi.resize(2,2);

}

void BlobKalmanFilter::reset() {
    
	//stop();
	measured.reset();
    filtered.reset();
    
	numNoFeatureFrames = 0;
    hasFeatureFrames = 0;
    old_x = 0; old_y = 0;
    x_dot = 0; y_dot = 0;
    framesElapsed = 0;
	roi.zero();
	previousMeasuredVals.zero();
	predictedCovariance.eye();
}

//Set the feature threshold values
void BlobKalmanFilter::setFeatureThreshold(int nft, int vt) {
	noFeatureThreshold = nft;
    validThreshold = vt;
}

//Initialize the Kalman filter for the blob
void BlobKalmanFilter::initialize() {
    
	KalmanFilter::initialize();
    
	Matrix transitionMatrix(stateSize,stateSize);
	Vector initialState(stateSize);
    
	transitionMatrix.eye();
    
 	//Increase denominator to reduce effects of velocity
	transitionMatrix[0][2] = 1/5;
	transitionMatrix[1][3] = 1/5;
    
	initialState[0] = measured.firstMoment.x;
	initialState[1] = measured.firstMoment.y;
	initialState[2] = x_dot;
	initialState[3] = y_dot;
	initialState[4] = measured.roi[0][0];
	initialState[5] = measured.roi[0][1];
	initialState[6] = measured.roi[1][0];
	initialState[7] = measured.roi[1][1];
	
	previousMeasuredVals[0] = measured.firstMoment.x;
	previousMeasuredVals[1] = measured.firstMoment.y;
	previousMeasuredVals[2] = x_dot;
	previousMeasuredVals[3] = y_dot;
	previousMeasuredVals[4] = measured.roi[0][0];
	previousMeasuredVals[5] = measured.roi[0][1];
	previousMeasuredVals[6] = measured.roi[1][0];
	previousMeasuredVals[7] = measured.roi[1][1];
	
	old_x = (int)measured.firstMoment.x; 
	old_y = (int)measured.firstMoment.y;
    
	setTransitionMatrix(transitionMatrix);
    setInitialState(initialState);    
	start();
	
}

//This function calls the update on the kalman filter
void BlobKalmanFilter::update(double dt)  {
	// - check measured data validity
    //   if valid and not initialized: initialze filter
    //   else if valid and initialized: update filter and estimates
    //   if not valid: check counter to see if it is time to reset flag
    if (measured.isValid) {
		numNoFeatureFrames = 0; //has feature, reset "No Feature" count 
        hasFeatureFrames++;
        if (!isActive) {
            initialize();
            filterUpdate(dt);
            //if this is a new filter, then the data is initially invalid
            if (hasFeatureFrames < validThreshold) {
                filtered.isValid = false;
            }
        } else {
            filterUpdate(dt);
            if (hasFeatureFrames > validThreshold) {
                //make sure the variable does not overflow
                hasFeatureFrames = validThreshold + 1;
            } else {
                filtered.isValid = false;
            }
        }
   	} else {
		numNoFeatureFrames++;
        //check count to see if it is time to reset flag
        if (numNoFeatureFrames > noFeatureThreshold) {
            reset();
        } else {
            filterUpdate(dt);
            if (hasFeatureFrames < validThreshold) {
                filtered.isValid = false;
            }
        }        
        framesElapsed = numNoFeatureFrames + 1;        
	}
    
}

void BlobKalmanFilter::filterUpdate(double dt) {
    
	Vector predictedVals(stateSize); //State Vals predicted by the Kalman Filter
	Vector measuredVals(measurementSize);
	Matrix transitionMatrix(stateSize,stateSize);
    
	int x=0, y=0; 
	Matrix roi(2,2);
	Vector vel(2);
    
	transitionMatrix.eye();
	//transitionMatrix[0][2] = 1/5;
	//transitionMatrix[1][3] = 1/5;

    transitionMatrix[0][2] = dt;
	transitionMatrix[1][3] = dt;
    
	//Make Prediction using Kalman Filter
    if (isActive) {
        
		setTransitionMatrix(transitionMatrix);
        
        //predict(&predictedVals, &predictedCovariance);
		predict(&predictedVals);
        
        roi[0][0] = predictedVals[4];
        roi[0][1] = predictedVals[5];
		roi[1][0] = predictedVals[6];
		roi[1][1] = predictedVals[7];
        
		/*
          cout << "prediction:" << endl;
          for(int i =0; i<predictedVals.size(); i++) {
          cout <<  predictedVals[i]  << endl;
          }
          cout << endl;
		*/
        
		if (checkValidData((int)predictedVals[0], (int)predictedVals[1], roi)) {
            x = (int)predictedVals[0];
            y = (int)predictedVals[1];
            vel[0] = predictedVals[2];  //u_dot  (feature motion model)
            vel[1] = predictedVals[3];  //v_dot
            filtered.isValid = true;
        } else {
            x = width / 2;
            y = height / 2;
            filtered.isValid = false;
        }
        
        ///////////////////////////////////////////////
        // update filtered data structure from estimates
        filtered.firstMoment.x = x;
        filtered.firstMoment.y = y;
        filtered.roi = roi;        
        filtered.firstMomentCov = predictedCovariance;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                filtered.roiCov[i][j] = predictedCovariance[i+4][j+4];
        
    }
        
    //isValid is already set when computing the first and second moments
    framesElapsed = numNoFeatureFrames + 1;
    if (measured.isValid) {
        x = measured.firstMoment.x;
        y = measured.firstMoment.y;
        //x_dot = (x - old_x) / framesElapsed;
        //y_dot = (y - old_y) / framesElapsed;
        x_dot = (x - old_x) / dt;
        y_dot = (y - old_y) / dt;
        
        //////////////////////////////////////////////////////
        // update kalman filter model if the measurement has not jumped too far
        
        measuredVals[0] = measured.firstMoment.x;
        measuredVals[1] = measured.firstMoment.y;
        measuredVals[2] = x_dot;
        measuredVals[3] = y_dot;
        measuredVals[4] = measured.roi[0][0];
        measuredVals[5] = measured.roi[0][1];
        measuredVals[6] = measured.roi[1][0];
        measuredVals[7] = measured.roi[1][1];
        
        for (int i = 0; i < measurementSize; i++) {
            previousMeasuredVals[i] = measuredVals[i];
        }
    }
    
    //Increase denominator to reduce effects of velocity
    transitionMatrix[0][2] = (float)dt;
    transitionMatrix[1][3] = (float)dt;
    
    setTransitionMatrix(transitionMatrix);
    
    if (numNoFeatureFrames == 0) {
		/*
          cout << "updating KF with:" << endl;
          for (int i = 0; i < measurementSize; i++)                {
          cout << measuredVals[i] << endl;
          }
          cout << endl;
		*/
		KalmanFilter::update(measuredVals);
    }
    
    old_x = x;
    old_y = y;
}

bool BlobKalmanFilter::checkValidData(int x, int y, Matrix cov) {
    
	if ((cov[0][0] == 1 && cov[0][1] == 0 && cov[1][0] == 0 && cov[1][1] == 1)) {
        //cout << "BKF not a valid cov" << endl;
        return false;
	}
    
	PrincipleComponent pcFiltered;
    BlobUtilities util;
    
    util.getEigenVectorFromCov(cov, &pcFiltered);
    
    if (x < 0 || y < 0 || x > width || y > height) {
		//cout << "BKF out of bounds" << endl;
        return false;
    }
	if ((int)pcFiltered.mag1 <= 0 || (int)pcFiltered.mag2 <= 0) {
        //cout << "BKF error" << endl;
        return false;
	}
    
    return true;
}

