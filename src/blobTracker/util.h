// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <yarp/sig/Matrix.h>
#include <cv.h>
#include <iostream>

class PrincipleComponent {
public:
    double angle1;	//angle of the first principle component
    double mag1;		//magnitude of the first principle component
    double angle2;	//angle of the 2nd principle component;
    double mag2;	 	//magnitude of the 2nd principle component
    
    PrincipleComponent() {
        angle1 = 0.0;
        mag1 = 0.0;
        angle2 = 0.0;
        mag2 = 0.0;
    }
};
  
class FeatureBlob {
public:
	CvPoint firstMoment;   //first moment
    yarp::sig::Matrix roi; //Spread of the ROI
	yarp::sig::Matrix firstMomentCov;  //Variance of first moment
    yarp::sig::Matrix roiCov; //Variance of roi;
	bool isValid;
    
	FeatureBlob() {
		roi.resize(2,2);		
		firstMomentCov.resize(2,2);		
		roiCov.resize(4,4);		
		reset();
	}
    
	void reset() {
        isValid = false;
        firstMoment.x = 0; firstMoment.y = 0;
        firstMomentCov.eye();
		roi.eye();		    
	}
    
};

class BlobUtilities  {
    
public:
    
	BlobUtilities() { }
    
    //sets the box params which fits the covariance (and returns area of ellipse)
    double getBox2DFromCov(CvPoint &center, yarp::sig::Matrix cov, CvBox2D *box) {

        CvPoint2D32f 	_center;
        CvSize2D32f 	_size;
        PrincipleComponent 	pc;
	    double area;
        
        _center.x = (float)center.x;
        _center.y = (float)center.y;
        
	    if(getEigenVectorFromCov(cov, &pc)) {
            _size.width = (float)pc.mag1 * 3.2;
            _size.height = (float)pc.mag2 * 3.2;

            box->center.x = _center.x;
            box->center.y = _center.y;           
            box->size.width = _size.width;
            box->size.height = _size.height;
            box->angle = -(float)(pc.angle2*180.0/M_PI);           

            area = M_PI*pc.mag1*pc.mag2;

            //std::cout << "box angles: (" << (pc.angle1*180.0/M_PI) 
            //        << ", " << (pc.angle2*180.0/M_PI) << ")" << std::endl;
            
	    } else {
            _center.x = 0.0;
            _center.y = 0.0;
            _size.width = 0;
            _size.height = 0;
            box->center.x = _center.x;
            box->center.y = _center.y;
            box->size.width = _size.width;
            box->size.height = _size.height;
            box->angle = 0;
            area = 0;            

            //std::cout << "incorrect box" << std::endl;
        }
        
        return area;
        
    }

    //Computes the Eigen vector from the covariance matrix
    bool getEigenVectorFromCov(yarp::sig::Matrix cov, PrincipleComponent *pc) {

        double ang1, ang2;
        double lambda0, lambda1;
        double T; 
        double tmp;
        double det;
        
        if(cov[0][0]==-1 && cov[0][1]==-1 && cov[1][0]==-1 && cov[1][1]==-1){
            return false;
        }
           
        //compute eigen value
		T = cov[0][0] + cov[1][1];
        det = cov[0][0] * cov[1][1] - cov[0][1] * cov[1][0];
        lambda0 = fabs(T / 2.0 + sqrt(4 * cov[0][1] * cov[0][1] + (cov[0][0] - cov[1][1]) * (cov[0][0] - cov[1][1])) / 2.0);
        lambda1 = fabs(T / 2.0 - sqrt(4 * cov[0][1] * cov[0][1] + (cov[0][0] - cov[1][1]) * (cov[0][0] - cov[1][1])) / 2.0);
        //std::cout << "Cov : [" <<  cov[0][0] << " " << cov[0][1] << " " << cov[1][0] << " " << cov[1][1] << endl;
        //std::cout << "lambda0 : " <<  lambda0 << "\t lambda1 : " << lambda1 << " \t T : " << T << std::endl;
        
        if(lambda1 > lambda0) {
            tmp = lambda0;
            lambda0 = lambda1;
            lambda1 = tmp;
        }
        
        //angle of the 1st principle component
        ang1 = 0.5*atan2(2*cov[0][1], cov[0][0]-cov[1][1]);
        //adjust angle so it falls between (0,2PI)
        if(ang1<0) {
            ang1 = M_PI+ang1;
        }
        //angle of the 2nd principle component
        ang2 = ang1+M_PI/2.0;
        //adjust angle so it falls between (0,2PI)
        if(ang2>2*M_PI) {
            ang2=ang2-2.0*M_PI;
        }
        
        pc->angle1 = ang1;
        pc->angle2 = ang2;
        pc->mag1 = sqrt(lambda0);
        pc->mag2 = sqrt(lambda1);
        
        return true;
    }
    
	void getSingleBlobCharacteristics(IplImage* img, yarp::sig::Vector *center, yarp::sig::Matrix *covar) {
        
		int meanx = 0;
		int meany = 0;
		int count = 0;
		
		int height = img->height;
		int width = img->width;
		int step = img->widthStep;
		int channels = img->nChannels;
        
		int c;
		double a,b;
        
		for(int i=0; i<height; i++) {
			for(int j=0; j<width; j++) {
				c = 0;
				for(int k=0; k<channels; k++) {
					c += img->imageData[i*step+j*channels+k];
                    if(c!=0) {
                        meanx += j;
                        meany += i;
                        count++;
                    }
				}
			}
		}
        
		if (count > 0) {
            meanx /= count;
            meany /= count;
            
            (*center)[0] = meanx;
            (*center)[1] = meany;
            
            covar->eye();
			for(int i=0; i<height; i++) {
				for(int j=0; j<width; j++) {
					c = 0;
					for(int k=0; k<channels; k++) {
						c += img->imageData[i*step+j*channels+k];
                        if(c!=0) {
                            a = j - meanx;
                            b = i - meany;
                            (*covar)[0][0] += a * a;
                            (*covar)[0][1] += a * b;
                            (*covar)[1][0] += b * a;
                            (*covar)[1][1] += b * b;
                        }
                    }
                }
			}
            
			for(int i=0; i<2; i++) {
				for(int j=0; j<2; j++) {
					(*covar)[i][j] /= count;
				}			
			}
		}
	}
    
    
	double sqr(double x) { 
        return x*x; 
    } 
    
};



