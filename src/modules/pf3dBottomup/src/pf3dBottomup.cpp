/*
 * A bottom-up approach for generating particles for the "pf3dTracker" module
 *
 * Copyright (C) 2010 RobotCub Consortium
 *
 * Author: Martim Brandao
 * Note: Should you use or reference my work on your own research, please let me know (mbrandao _AT_ isr.ist.utl.pt)
 *
 * Image sequence as input, N particles (3D position of balls) as output.
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/pf3dBottomup.hpp>

#include <highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>

//member that is repeatedly called by YARP, to give this object the chance to do something.
//should this function return "false", the object would be terminated.
//I already have one image, when I get here (I either acquire it in the initialization method or in the end of this same method).
bool pf3dBottomup::updateModule()
{
    if(_doneInitializing)
    {
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* contours = 0;
        double raio=0.03;
        int num_detected_objects;

        if(_blur>0) cvSmooth(image,image, CV_GAUSSIAN, 0, 0, _blur, 0);
        cvCvtColor(image, hsv, CV_BGR2HSV );

        cvInRangeS(hsv, cvScalar(0,_maskSmin,MIN(_maskVmin,_maskVmax),0), 
                cvScalar(181,256,MAX(_maskVmin,_maskVmax),0), mask); //update mask
        cvSplit(hsv, hue, sat, val, 0 );

        // Histogram Backprojection
        cvCalcBackProject(&hue, backproject, _object_model.hist);
        cvAnd(backproject, mask, backproject, 0);

        // Bottom-up detection algorithm...

        //cvThreshold(backproject, backproject, 40, 0, CV_THRESH_TOZERO );

        // Normalize backprojection to global max
        normalize_to_global_max(backproject);

        // Build scale-space
        cvConvert(backproject, infloat);
        ss.BuildAll((float*)infloat->imageData);

        // Segmentation (localmaxima + floodfill)
        scale_space_segmentation(backproject, &ss, backprojectmask2);

        // 3D Localization
        cvSetImageROI(backprojectmask2, cvRect(1,1,image->width,image->height));
        num_detected_objects = object_localization_simple(backprojectmask2, &_object_model, &_camera);
        //object_localization(backprojectmask2, _object_model, _camera);
        cvResetImageROI(backprojectmask2);

        // Output particles
        if(num_detected_objects>0){
            Bottle& particleOutput=_outputParticlePort.prepare(); int count;
            particleOutput.clear();
            particleOutput.addInt(_nParticles);
            for(count=0;count<_nParticles;count++)
            {
                particleOutput.addDouble((double)cvmGet(_object_model.particles,0,count));
                particleOutput.addDouble((double)cvmGet(_object_model.particles,1,count));
                particleOutput.addDouble((double)cvmGet(_object_model.particles,2,count));
            }
            //particleOutput.addInt(num_detected_objects);
            _outputParticlePort.write();
        }

        // acquire a new image
        _yarpImage = _inputVideoPort.read(); //read one image from the buffer.
            //temporary cheating (resize to 640x480)
            cvResize((IplImage*)_yarpImage->getIplImage(), image, CV_INTER_LINEAR);
            //--end
        cvCvtColor(image, image, CV_RGB2BGR);

    }    

    return true; //continue. //in this case it means everything is fine.
}


//------------------------------------------------------------------------------------------------------------


//member function that set the object up.
bool pf3dBottomup::configure(ResourceFinder &rf)
{
    _doneInitializing=false;

    string trackedObjectColorTemplate;

    //***********************************
    //Read options from the command line.
    //***********************************
    _inputVideoPortName = rf.check("inputVideoPort",
                Value("/pf3dBottomup/video:i"),
                "Input video port (string)").asString();
    _outputParticlePortName = rf.check("outputParticlePort",
                Value("/pf3dBottomup/particles:o"),
                "Output particle port (string)").asString();

    _inputVideoPort.open(_inputVideoPortName);
    _outputParticlePort.open(_outputParticlePortName);

    _nParticles = rf.check("nParticles",
                Value("100"),
                "Number of particles used in the tracker (int)").asInt();
    _calibrationImageWidth = rf.check("w",
                Value(640),
                "Image width  (int)").asInt();
    _calibrationImageHeight = rf.check("h",
                Value(480),
                "Image height (int)").asInt();
    _camera.fx = rf.check("perspectiveFx",
                Value(1),
                "Focal distance * kx  (double)").asDouble();
    _camera.fy = rf.check("perspectiveFy",
                Value(1),
                "Focal distance * ky  (double)").asDouble();
    _camera.cx = rf.check("perspectiveCx",
                Value(1),
                "X position of the projection center, in pixels (double)").asDouble();
    _camera.cy = rf.check("perspectiveCy",
                Value(1),
                "Y position of the projection center, in pixels (double)").asDouble();
    _scaleSpaceLevels = rf.check("scaleSpaceLevels",
                Value(3),
                "Number of levels on the scale space (int)").asInt();
    _maskVmin = rf.check("maskVmin",
                Value(15),
                "Minimum acceptable image value (int)").asInt();
    _maskVmax = rf.check("maskVmax",
                Value(255),
                "Maximum acceptable image value (int)").asInt();
    _maskSmin = rf.check("maskSmin",
                Value(70),
                "Minimum acceptable image saturation (int)").asInt();
    _blur = rf.check("Blur",
                Value(0),
                "Blur variance applied to the input image (int)").asInt();
    _object_model.raio_esfera = rf.check("sphereRadius",
                Value(0.03),
                "Radius of the sphere in case that is our object (double)").asDouble();

    trackedObjectColorTemplate = rf.findFile("trackedObjectColorTemplate").c_str();
    if(trackedObjectColorTemplate==""){ 
        cout<<"Couldnt find color model specified in pf3dBottomup.ini\n";
        return false;
    }

    // object model
    calc_hist_from_model_2D(trackedObjectColorTemplate, &_object_model.hist, _maskVmin, _maskVmax);
    _object_model.particles = cvCreateMat(3,_nParticles,CV_32FC1);

    // camera model
    _camera.fov = 2*atan(_calibrationImageHeight/(2*_camera.fy))*180/PI; //field of view in degrees
    _camera.aspect = _calibrationImageWidth/(double)_calibrationImageHeight * (_camera.fy/_camera.fx); //aspect ratio
    _camera.znear = 0.01; //Z near
    _camera.zfar = 1000; //Z far

    // scale space
    _scaleSpaceLevels=3;
    _scaleSpaceScales[0]=16.0 * _calibrationImageWidth/640;
    _scaleSpaceScales[1]= 8.0 * _calibrationImageWidth/640;
    _scaleSpaceScales[2]= 4.0 * _calibrationImageWidth/640;
    ss.AllocateResources(_calibrationImageHeight, _calibrationImageWidth, _scaleSpaceLevels, _scaleSpaceScales);

    // read one image from the stream
    _yarpImage = _inputVideoPort.read();
    image = cvCreateImage(cvSize(_calibrationImageWidth, _calibrationImageHeight), 8, 3);
    cvCvtColor((IplImage*)_yarpImage->getIplImage(), image, CV_RGB2BGR);

    // allocate all images
    infloat =         cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1);
    hsv =            cvCreateImage( cvGetSize(image), 8, 3 );
    hue =            cvCreateImage( cvGetSize(image), 8, 1 );
    sat =            cvCreateImage( cvGetSize(image), 8, 1 );
    val =            cvCreateImage( cvGetSize(image), 8, 1 );
    mask =            cvCreateImage( cvGetSize(image), 8, 1 );
    backproject =        cvCreateImage( cvGetSize(image), 8, 1 );
    backprojectmask2 =    cvCreateImage( cvSize(image->width+2,image->height+2), 8, 1 );

    // done
    _doneInitializing=true;

    return true;
}


//member that closes the object.
bool pf3dBottomup::close()
{
    // ports
    _inputVideoPort.close();
    _outputParticlePort.close();

    //resources
    ss.FreeResources();
    cvReleaseMat(&_object_model.particles);
    cvReleaseImage(&image);
    cvReleaseImage(&infloat);
    cvReleaseImage(&hsv);
    cvReleaseImage(&hue);
    cvReleaseImage(&sat);
    cvReleaseImage(&val);
    cvReleaseImage(&mask);
    cvReleaseImage(&backproject);
    cvReleaseImage(&backprojectmask2);

    return true;
}


//member that closes the object.
bool pf3dBottomup::interruptModule()
{
    //ports
    _inputVideoPort.interrupt();
    _outputParticlePort.interrupt();

    return true;
}


//constructor
pf3dBottomup::pf3dBottomup()
{
    ;
}


//destructor
pf3dBottomup::~pf3dBottomup()
{
    cout<<"oh my god! they killed kenny!    you bastards!\n";
}



//------------------------------------------------------------------------------------------------------------
//
//    Segmentation and localization functions
//
//------------------------------------------------------------------------------------------------------------



//compute histogram from color template
void pf3dBottomup::calc_hist_from_model_2D(string file, CvHistogram **objhist, int _vmin, int _vmax)
{
    float max_val = 0.f;
    float bins[35][10], *curbin;
    int i, j;
    float kernFactor=0.1F;
    IplImage *histmodel = 0, *histmask = 0, *histhue = 0, *histsat = 0, *histval = 0;
    CvHistogram *hist;

    int dims[]       = {35, 10};
    float cbRanges[] = {0, 180};
    float crRanges[] = {0, 255};
    float* ranges[]  = {cbRanges, crRanges};
    IplImage* imgs[] = {0, 0};

    histmodel = cvLoadImage(file.c_str());
    cvCvtColor(histmodel, histmodel, CV_BGR2HSV);
    histmask = cvCreateImage(cvGetSize(histmodel), 8, 1);
    histhue = cvCreateImage(cvGetSize(histmodel), 8, 1);
    histsat = cvCreateImage(cvGetSize(histmodel), 8, 1);
    histval = cvCreateImage(cvGetSize(histmodel), 8, 1);
    cvInRangeS(histmodel, cvScalar(0,70,MIN(_vmin,_vmax),0),cvScalar(181,256,MAX(_vmin,_vmax),0), histmask);
    cvSplit(histmodel, histhue, histsat, histval, 0);

    imgs[0]=histhue; imgs[1]=histsat;
    hist = cvCreateHist(2, dims, CV_HIST_ARRAY, ranges, CV_HIST_UNIFORM);

    cvCalcHist((IplImage **)imgs, hist, 0, histmask);

    //martim: add some variance to the histogram
    for(i=0;i<dims[0];i++)
        for(j=0;j<dims[1];j++)
            bins[i][j]=0;
    for(i=0;i<dims[0];i++)
        for(j=0;j<dims[1];j++){
            bins[i][j]+=(float)((1+kernFactor)*cvGetReal2D(hist->bins,i,j));
            //hue e' circular
            if(i>0) bins[i-1][j]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            else bins[dims[0]-1][j]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            if(i<dims[0]-1) bins[i+1][j]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            else bins[0][j]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            //saturacao nao e' circular
            if(j>0) bins[i][j-1]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            if(j<dims[1]-1) bins[i][j+1]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            if(j>1) bins[i][j-2]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
            if(j<dims[1]-2) bins[i][j+2]+=(float)(kernFactor*cvGetReal2D(hist->bins,i,j));
        }
    for(i=0;i<dims[0];i++)
        for(j=0;j<dims[1];j++){
            curbin = (float*)cvPtr2D(hist->bins, i, j);
            (*curbin) = bins[i][j];
        }
    //--end

    cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
    //cvNormalizeHist(hist, 2500); //tolerancia. 1000 durante o dia (pouca variabilidade); 10000 durante a noite
    cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );

    *objhist = hist;

    cvReleaseImage(&histmodel);
    cvReleaseImage(&histmask);
    cvReleaseImage(&histhue);
}

//normalize grayscale image so that its global max becomes = 255
void pf3dBottomup::normalize_to_global_max(IplImage *img)
{
    int i,j;
    int step;
    uchar *data;
    int pmax=0, max=0;
    int M=255;
    int px1=0,numpx1=0,px2=0,numpx2=0;

    step = img->widthStep/sizeof(uchar);
    data = (uchar *)img->imageData;

    //find global maximum
    for(i=0 ; i<img->height ; i++)
        for(j=0 ; j<img->width ; j++)
            if(data[i*step+j] > max){ pmax=i*step+j; max=data[pmax]; }
    //max must be big enough
    if(max > 50){
        //normalize
        for(i=0 ; i<img->height ; i++)
            for(j=0 ; j<img->width ; j++)
                data[i*step+j] = ((data[i*step+j]*M)/max);
    }
}

//segmentation algorithm (local maxima followed by kind of floodfill) on all scale-space levels, joined in one resulting grayscale img
void pf3dBottomup::scale_space_segmentation(IplImage *img, ScaleSpace *ss, IplImage *result)
{
    int pmax=0;
    int l,i,j,si,sj,aux;
    int r=1;
    IplImage *outgray, *outfloat, *floodmask;
    float *data;
    int step;
    uchar *maxdata;
    int maxstep;

    step = img->widthStep/sizeof(uchar);

    outgray = cvCreateImage(cvGetSize(img), 8, 1);
    outfloat = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
    floodmask = cvCreateImage(cvGetSize(result), 8, 1 );

    cvZero(result);

    // For each level... do segmentation
    for(l=0 ; l < ss->GetLevels() ; l++){
        int thres = 128; //maximos so acima deste valor...
        int max = 0;
        cvZero(floodmask);

        outfloat->imageData = (char*)(ss->GetLevel(l));

        //aux
        cvConvert(outfloat, outgray);
        data = (float *)outfloat->imageData;
        step = outfloat->widthStep/sizeof(float);
        maxdata = (uchar *)outgray->imageData;
        maxstep = outgray->widthStep/sizeof(uchar);

        //localmaxima v2
        for(i=r;i<img->height-r;i++){
            for(j=r;j<img->width-r;j++){
                //maximos so a partir de threshold. todo: T depende da variancia!
                if(data[i*step+j] < thres){ maxdata[i*maxstep+j]=0; continue; }

                //3x3 window
                for(si=i-r,aux=0 ; si<=i+r && !aux ; si++)
                    for(sj=j-r ; sj<=j+r ; sj++)
                        if(!(si==i && sj==j) && data[i*step+j] <= data[si*step+sj]){ 
                            maxdata[i*maxstep+j]=0; aux=1; break; 
                        }

                //em cada maximo: FloodFill
                if(!aux){
                    float T=80; //data[i*step+j]*0.8;
                    cvFloodFill2(outfloat, cvPoint(j,i), cvScalar(255), 
                            cvScalar(T), cvScalar(T), NULL, 
                            4+(255<<8)+CV_FLOODFILL_MASK_ONLY+CV_FLOODFILL_FIXED_RANGE,                                 floodmask);
                }
            }
        }
        cvOr(floodmask, result, result, 0);
        //--end
    }

    cvReleaseImage(&floodmask);
    cvReleaseImage(&outfloat);
    cvReleaseImage(&outgray);
}

//guess 3D position of object from segmentation (assuming it is a ball). returns number of objects
int pf3dBottomup::object_localization_simple(IplImage *segm, ObjectModel *model, CameraModel *camera)
{
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = 0;
    CvMoments moments;
    CvPoint center;
    double raio=0.03;
    double m00,m01,m10,area;
    double fx,fy,cx,cy;
    int i,j,count=0;

    fx=camera->fx; fy=camera->fy; cx=camera->cx; cy=camera->cy;
    raio = model->raio_esfera;

    //careful! "segm" image will change...
    cvFindContours( segm, storage, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) ); 

    while(contours)
    {
        area = fabs(cvContourArea( contours, CV_WHOLE_SEQ ));
        if(area>300.0 * segm->width/640){ //threshold area ---> depends on image size....
            double uu,vv,raiopx,xx,yy,zz;

            cvMoments(contours,&moments,0);
            m00 = cvGetSpatialMoment( &moments, 0, 0);
            m10 = cvGetSpatialMoment( &moments, 1, 0);
            m01 = cvGetSpatialMoment( &moments, 0, 1);
            uu = m10/m00;
            vv = m01/m00;
            center = cvPoint((int)uu, (int)vv);
            raiopx = sqrt(1.0*area/PI);
            zz = raio/( ((uu+raiopx-cx)/fx)-((uu-cx)/fx) ); //usar eixo vv tb? media
            xx = zz*(uu-cx)/fx;
            yy = zz*(vv-cy)/fy;

            cvmSet(model->particles,0,count, xx*1000);
            cvmSet(model->particles,1,count, yy*1000);
            cvmSet(model->particles,2,count, zz*1000);

            count++;
        }
        contours = contours->h_next;
    }

    // generate particles (fill the rest of particle vector with scattered versions of the measured ones)
    for(i=0 , j=count ; i<count ; i++)
        for( ; j<count+(i+1)*(_nParticles-count)/count ; j++){
            cvmSet(model->particles,0,j, cvmGet(model->particles,0,i)+SCATTER(30));
            cvmSet(model->particles,1,j, cvmGet(model->particles,1,i)+SCATTER(30));
            cvmSet(model->particles,2,j, cvmGet(model->particles,2,i)+SCATTER(50));
        }

    cvReleaseMemStorage( &storage );

    return count;
}



