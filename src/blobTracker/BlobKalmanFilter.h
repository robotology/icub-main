// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _BLOB_KALMAN_FILTER__H_
#define _BLOB_KALMAN_FILTER__H_

#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include "util.h" 


class KalmanFilter {
    
public:
    
	// size of the KF state
	int stateSize;
    
    //Size of Measurement vector
	int measurementSize;
    
    //Size of control vector
    int controlSize;
    
    //Open CV Kalman Filter
    CvKalman *kalman;
    
	// active flag
    bool isActive;
    
	CvMat Mmat; // measurement matrix

    const CvMat *p;
    const CvMat *cov;

	KalmanFilter(int stateSize, int measurementSize, int controlSize) :
        stateSize(stateSize),
		measurementSize(measurementSize),
		controlSize(controlSize),
		isActive(false)
	{
        kalman = cvCreateKalman(stateSize, measurementSize, controlSize);         
   	}
    
	void start() { isActive = true; }
	void stop() { isActive = false; }
    
	virtual void initialize() {    
	    setMeasurementMatrix(1);
	    setProcessNoiseCovariance(1e-6);
	    setErrorCovariancePost(50);
 	}

    virtual void update(yarp::sig::Vector measuredVals) {   
        float m[measuredVals.size()];
		for(int i=0; i<measuredVals.size(); i++) {
			m[i] = (float)measuredVals[i];
		}
		Mmat = cvMat(measuredVals.size(), 1, CV_32FC1, m);	       	
        cvKalmanCorrect(kalman, &Mmat);
    }


    void setTransitionMatrix(yarp::sig::Matrix transitionMatrix) {       
		int c=0;
		for(int i=0; i<transitionMatrix.rows(); i++) {
			for(int j=0; j<transitionMatrix.cols(); j++) {
                kalman->transition_matrix->data.fl[c]=(float)transitionMatrix[i][j];
				c++;			
			}
		}
	}
    
    void setInitialState(yarp::sig::Vector initialState) {
		for(int i=0; i<initialState.size(); i++) {
            kalman->state_post->data.fl[i]=(float)initialState[i];
		}
	}
    
    void predict(yarp::sig::Vector *predictedVals, yarp::sig::Matrix *predictedCovariance) {
		predict(predictedVals);
		predictedCovariance->eye();	   	
		getErrorCovariancePost(predictedCovariance);
	}

    void predict(yarp::sig::Vector *predictedVals) {
		p = cvKalmanPredict(kalman, 0);        
        if( (p->rows != predictedVals->size()) || (p->cols != 1) ) {
			fprintf(stderr, "KalmanFilter problem getting prediction!!\n");
			predictedVals->zero();			
		} else {
			for(int i=0; i<predictedVals->size(); i++) {
				(*predictedVals)[i] = (double)(p->data.fl[i]);
			}		
		}
	}
    
    void setMeasurementNoiseCovariance(double val) {       
        cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(val) );
    }
    
    void setMeasurementMatrix(double val){
        cvSetIdentity( kalman->measurement_matrix, cvRealScalar(val) );
    }

    void setProcessNoiseCovariance(double val){
        cvSetIdentity( kalman->process_noise_cov, cvRealScalar(val) );
    }

    void setErrorCovariancePost(double val) {
        cvSetIdentity( kalman->error_cov_post, cvRealScalar(val) );
    }

    void setErrorCovariancePre(double val) {
        cvSetIdentity( kalman->error_cov_pre, cvRealScalar(val) );
    }

	yarp::sig::Matrix * getErrorCovariancePost(yarp::sig::Matrix *Mcov) {
        cov = kalman->error_cov_post;
        int c=0;
        for(int i=0; i<stateSize; i++) {
            for(int j=0; j<stateSize; j++) {
                (*Mcov[i][j]) = (double)(cov->data.fl[c++]);
            }
        }
        return Mcov;
    }
    
};
  

class BlobKalmanFilter : public KalmanFilter {
    
public:
    
    //Image width and height
    int width;
    int height;
    
    int numNoFeatureFrames;
    int noFeatureThreshold;
    int hasFeatureFrames;
    int validThreshold;
	int framesElapsed;
    
    int old_x, old_y;
	int x_dot, y_dot;
	yarp::sig::Matrix roi;       
    
    yarp::sig::Vector previousMeasuredVals;
    yarp::sig::Matrix predictedCovariance;
    
    //Initialize the blobs contain information of the measured and filtered blob
    FeatureBlob measured; //measurement blobs
    FeatureBlob filtered; //filtered blobs
    
    BlobKalmanFilter(int _width, int _height);
	
	void initialize();
	void update(double dt=1.0);
	void reset();
	void setFeatureThreshold(int nft, int vt);
	void filterUpdate(double dt);
	bool checkValidData(int x, int y, yarp::sig::Matrix cov);
	
};


#endif
