// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#include <iCub/vis/OptFlowEMD.h>

OptFlowEMD::OptFlowEMD(){
    
    // configuration 
    _oldImgSize.width = -1;
    _oldImgSize.height = -1;
        
    _oldLPFx0 = NULL;
    _oldLPFx1 = NULL;
    _oldLPFy0 = NULL;
    _oldLPFy1 = NULL;

}

OptFlowEMD::~OptFlowEMD(){
    releaseImages();
}

void OptFlowEMD::initImages(CvSize size){
    releaseImages();
    _oldLPFx0 = cvCreateImage(size, IPL_DEPTH_32F, 1);
    _oldLPFx1 = cvCreateImage(size, IPL_DEPTH_32F, 1);
    _oldLPFy0 = cvCreateImage(size, IPL_DEPTH_32F, 1);
    _oldLPFy1 = cvCreateImage(size, IPL_DEPTH_32F, 1);
}

void OptFlowEMD::releaseImages(){
    if (_oldLPFx0 != NULL)
        cvReleaseImage(&_oldLPFx0);
    _oldLPFx0 = NULL;
    if (_oldLPFx1 != NULL)
        cvReleaseImage(&_oldLPFx1);
    _oldLPFx1 = NULL;
    if (_oldLPFy0 != NULL)
        cvReleaseImage(&_oldLPFy0);
    _oldLPFy0 = NULL;
    if (_oldLPFy1 != NULL)
        cvReleaseImage(&_oldLPFy1);
    _oldLPFy1 = NULL;
}

bool OptFlowEMD::open(yarp::os::Searchable &config){
    return configure(config);
}

bool OptFlowEMD::configure (yarp::os::Searchable &config){

    // configuration
    _alpha = (float)config.check("alpha",
                                yarp::os::Value(0.28),
                                "EMD delay parameter (double)").asDouble();
                                
    _threshold =(float)config.check("threshold",
                                    yarp::os::Value(80.0),
                                    "EMD motion threshold (double)").asDouble();
    _thresholdSquared = pow(_threshold,2.0f);
    _constrain = config.check("constrain", yarp::os::Value(0),
                              "Constrain salience values to <=constrainValue? (1/0)").asInt()!=0;
    _constrainValue = (float)config.check("constrainValue", yarp::os::Value(255.0),
                        "Constrain salience values to this value if constrain = 1 (double)").asDouble();
    _constrainValueSquared = pow(_constrainValue,2.0f);
    _scale = (float)config.check("scale",yarp::os::Value(1.0),
                    "Scale salience values (double).").asDouble();

    // TODO reimplement distortion for cv version
    //if (config.check("distortion", "2x2 distortion matrix in list format: topLeft, "
    //                 "topRight, bottomLeft, bottomRight. Default = 1 0 0 1.")){
    //    //cout << "distortion" << endl;
    //    yarp::os::Bottle &bot = config.findGroup("distortion");
    //    if (bot != NULL){
    //        //cout << "bottle != NULL" << endl;
    //    if (bot.size() == 5){
    //            //cout << "bottle of size 5" << endl;
    //            for (int i = 1; i < 5; i++){
    //                //cout << "value: " << bot.get(i).asDouble() << endl;
    //                _matrix[i-1] = (float)(bot.get(i).asDouble());
    //            }
    //        }
    //    }
    //}
    
    _oldImgSize.width = -1;
    _oldImgSize.height = -1;

    return true;
}

bool OptFlowEMD::close(){
    releaseImages();
    return true;
}

void OptFlowEMD::calculate_flow(IplImage* imageT, IplImage* imageTMinus1, IplImage* velx, IplImage* vely, IplImage* abs){

     __BEGIN__
            
    CV_FUNCNAME( "OptFlowGenGrad::calculate_flow" );
    
    CvMat stubA, *srcA = (CvMat*)imageT;   // stubA takes the new header data for the matrix according to ROI
    CvMat stubB, *srcB = (CvMat*)imageTMinus1;
    CvMat stubx, *vel_x = (CvMat*)velx;
    CvMat stuby, *vel_y = (CvMat*)vely;
    CvMat stubAbs, *abs_ = NULL;
    if (abs != NULL)
        abs_ = (CvMat*)abs;

    // see GetMat function doc: This returns a matrix header with the current image ROI!
    // this gives basically a view on the ROI, stubA takes the header data of the matrix
    //  srcA is pointed to this new 'augmented' data-header
    CV_CALL(  srcA = cvGetMat(  srcA, &stubA ));
    CV_CALL(  srcB = cvGetMat(  srcB, &stubB ));
    CV_CALL(  vel_x = cvGetMat(  vel_x, &stubx ));
    CV_CALL(  vel_y = cvGetMat(  vel_y, &stuby ));
    if (abs_ != NULL)
        CV_CALL(  abs_ = cvGetMat (  abs_, &stubAbs ));


    if( !CV_ARE_TYPES_EQ(  srcA,  srcB ))
        CV_ERROR( CV_StsUnmatchedFormats, "Source images have different formats" );

    if( !CV_ARE_TYPES_EQ(  vel_x,  vel_y ))
        CV_ERROR( CV_StsUnmatchedFormats, "Destination images have different formats" );

    if (abs_ != NULL)
        if (!CV_ARE_TYPES_EQ( vel_x, abs_))
            CV_ERROR( CV_StsUnmatchedFormats, "Destination images have different formats" );

    if( !CV_ARE_SIZES_EQ(  srcA,  srcB )   ||
        !CV_ARE_SIZES_EQ(  vel_x,  vel_y ) ||
        !CV_ARE_SIZES_EQ(  srcA,  vel_x ))
        CV_ERROR( CV_StsUnmatchedSizes, "Some images have different sizes" );

    if(abs_ != NULL)
        if (!CV_ARE_SIZES_EQ( srcA, abs_))
            CV_ERROR( CV_StsUnmatchedSizes, "Some images have different sizes" );

    if( CV_MAT_TYPE(  srcA->type )  != CV_8UC1)
        CV_ERROR( CV_StsUnsupportedFormat, "Source images must have 8uC1 type");
    
    if( CV_MAT_TYPE(  vel_x->type ) != CV_32FC1 )
        CV_ERROR( CV_StsUnsupportedFormat, "Destination images must have 32fC1 type" );

    if( srcA->step  !=  srcB->step  ||
        vel_x->step !=  vel_y->step)
        CV_ERROR( CV_BadStep, "source and destination images have different step" );

    if (abs_ != NULL)
        if (vel_x->step != abs_->step)
            CV_ERROR( CV_BadStep, "source and destination images have different step" );
   
    if (abs_ != NULL){
        IPPI_CALL( calcOptFlowEMD( (uchar*)srcA->data.ptr, (uchar*)srcB->data.ptr, srcA->step, cvGetMatSize( srcA ),
                                vel_x->data.fl, vel_y->data.fl, vel_x->step,
                                abs_->data.fl));
    }
    else{
        IPPI_CALL( calcOptFlowEMD( (uchar*)srcA->data.ptr, (uchar*)srcB->data.ptr, srcA->step, cvGetMatSize( srcA ),
                                vel_x->data.fl, vel_y->data.fl, vel_x->step,
                                NULL));
    }
    
    __END__
     
}

// opt flow calculation
CvStatus CV_STDCALL
OptFlowEMD::calcOptFlowEMD    ( uchar*  imgA,
                                uchar*  imgB,
                                int     imgStep,
                                CvSize  imgSize,
                                float*  velocityX,
                                float*  velocityY,
                                int     velStep,
                                float*  absolute)
{

    int imgWidth = imgSize.width;
    int imgHeight = imgSize.height;
    float HPF0, HPF1, LPF0, LPF1;
    float emdX, emdY, emdAbs;
    float factor;
    int step, pos;
    
    /* Checking bad arguments */
    if( imgA == NULL )
        return CV_NULLPTR_ERR;
    if( imgB == NULL )
        return CV_NULLPTR_ERR;

    if( imgSize.width <= 0 )
        return CV_BADSIZE_ERR;
    if( imgSize.height <= 0 )
        return CV_BADSIZE_ERR;
    if( imgSize.width > imgStep )
        return CV_BADSIZE_ERR;

    if( (velStep & 3) != 0 )
        return CV_BADSIZE_ERR;

    velStep /= 4;

    // change buffer image sizes if image size changed
    if (_oldImgSize.width != imgSize.width || 
        _oldImgSize.height != imgSize.height)
        initImages(imgSize);
    
    _totFlowX = 0.0f;
    _totFlowY = 0.0f;
    _totFlowAbs = 0.0f;
    _maxFlowPixelAbs = 0.0f;
    _maxFlowPixelX = 0.0f;
    _maxFlowPixelY = 0.0f;
    
    for(register short y = 0; y < imgSize.height - 1; y++){
        step = y*imgStep;
        for(register short x = 0; x < imgSize.width - 1; x++){
            pos = step + x;

            // *** x-direction ***
            
            HPF0 =  (float)(imgA[pos] - imgB[pos]);
            HPF1 =  (float)(imgA[pos + 1] - imgB[pos + 1]);

            LPF0 = (float)((_alpha * HPF0) + ((1.0 - _alpha) * ((float*)(_oldLPFx0->imageData + _oldLPFx0->widthStep*y))[x] ));
            LPF1 = (float)((_alpha * HPF1) + ((1.0 - _alpha) * ((float*)(_oldLPFx1->imageData + _oldLPFx1->widthStep*y))[x] ));

            // EMDOutput
            emdX =  (LPF0*HPF1) - (LPF1*HPF0);
            //cout << "vel x: " <<  velocityX[step + x] << endl;
            
            // save the LPF's
            ((float*)(_oldLPFx0->imageData + _oldLPFx0->widthStep*y))[x] = LPF0;
            ((float*)(_oldLPFx1->imageData + _oldLPFx1->widthStep*y))[x] = LPF1;

            // *** y-direction ***

            HPF0 =  (float)(imgA[pos] - imgB[pos]);
            HPF1 =  (float)(imgA[(y+1)*imgStep + x] - imgB[(y+1)*imgStep + x]);

            LPF0 = (float)((_alpha * HPF0) + ((1.0 - _alpha) * ((float*)(_oldLPFy0->imageData + _oldLPFy0->widthStep*y))[x] ));
            LPF1 = (float)((_alpha * HPF1) + ((1.0 - _alpha) * ((float*)(_oldLPFy1->imageData + _oldLPFy1->widthStep*y))[x] ));

            // EMDOutput
            emdY =  (LPF0*HPF1) - (LPF1*HPF0);

            // save the LPF's
            ((float*)(_oldLPFy0->imageData + _oldLPFy0->widthStep*y))[x] = LPF0;
            ((float*)(_oldLPFy1->imageData + _oldLPFy1->widthStep*y))[x] = LPF1;

            // ** absolute value square ***

             emdAbs = emdX * emdX + emdY * emdY;

             if (emdAbs > _thresholdSquared){
                 
                 if (!_constrain){
                     velocityX[pos] = emdX * _scale;
                     velocityY[pos] = emdY * _scale;
                     if (absolute != NULL)
                        absolute[pos] = sqrt(emdAbs) * _scale;
                     //cout << "absFlow: " << x << " " << y << " " << sqrt(emdAbs) << endl;
                     _totFlowX += velocityX[pos];
                     _totFlowY += velocityY[pos];
                     if (emdAbs > _maxFlowPixelAbs)
             	        _maxFlowPixelAbs = emdAbs * _scale;
                     if (emdX > _maxFlowPixelX)
             	        _maxFlowPixelX = velocityX[pos];
                     if (emdY > _maxFlowPixelY)
             	        _maxFlowPixelY = velocityY[pos];
                 }
                 else{  // constrain to _constrainValue
                    emdAbs = sqrt(emdAbs) * _scale;
                    if (emdAbs > _constrainValue){
                        factor = _constrainValue / emdAbs;
                        velocityX[pos] = emdX * _scale * factor;
                        velocityY[pos] = emdY * _scale * factor;
                        if (absolute != NULL)
                            absolute[pos] = _constrainValue;
                    }
                    else{
                        velocityX[pos] = emdX * _scale;
                        velocityY[pos] = emdY * _scale;
                        if (absolute != NULL)
                            absolute[pos] = emdAbs;
                    }
                 }
             }
             else{
                 velocityX[pos] = 0.0f;
                 velocityY[pos] = 0.0f;
                 if (absolute != NULL)
                    absolute[pos] = 0.0f;  
             }
        }
    }
	_totFlowAbs = sqrt(pow(_totFlowX,2) + pow(_totFlowY,2));
	_maxFlowPixelAbs = sqrt(_maxFlowPixelAbs);
	
    // buffering old image size
    _oldImgSize.width = imgSize.width;
    _oldImgSize.height = imgSize.height;

    return CV_OK;
}

void OptFlowEMD::draw_flow(IplImage *image, IplImage *rgbX, IplImage *rgbY){
	
	int   arraypos = 0;
	for (int y = 0; y < image->height; y++){
		arraypos = image->widthStep*y;
        for (int x = 0; x < image->width; x++){
			if ((short)(((uchar*)(rgbX->imageData + arraypos))[x*3+0]) + (short)(((uchar*)(rgbY->imageData + arraypos))[x*3+0]) != 0){
				((uchar*)(image->imageData + arraypos))[x*3+0] = ((uchar*)(rgbX->imageData + arraypos))[x*3+0];
				((uchar*)(image->imageData + arraypos))[x*3+1] = ((uchar*)(rgbY->imageData + arraypos))[x*3+0];
				((uchar*)(image->imageData + arraypos))[x*3+2] = (uchar)0; 
			}
        }
    }                
} 
     
float OptFlowEMD::getTotalFlowX(){
	return _totFlowX;	
}

float OptFlowEMD::getTotalFlowY(){
	return _totFlowY;	
}

float OptFlowEMD::getTotalFlowAbs(){
	return _totFlowAbs;	
}

float OptFlowEMD::getMaxFlowAbsPerPixel(){
	return _maxFlowPixelAbs;	
}

           
