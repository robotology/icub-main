/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Harold Martinez
 * email:   martinez@ifi.uzh.ch
 *
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

#include <iCub/DisparityMapModule.h>



DisparityMapModule::DisparityMapModule(){

	
	aRight=cvCreateMat(3,3,CV_64FC1);
	aLeft=cvCreateMat(3,3,CV_64FC1);
	tRight=cvCreateMat(3,1,CV_64FC1);
	tLeft=cvCreateMat(3,1,CV_64FC1);
	rotRight=cvCreateMat(3,3,CV_64FC1);
	rotLeft=cvCreateMat(3,3,CV_64FC1);
	
	//OpenCV variables
	state=cvCreateStereoBMState( CV_STEREO_BM_BASIC, 15 );
	
	filterSize=5;
	filterCap=63;
    windowSize=7;
	minDisparity=-64;
	numDisparities=128;
	threshold=0;
	uniqueness=5;

	
}




DisparityMapModule::~DisparityMapModule(){

	

}

bool DisparityMapModule::open(Searchable& config){
  
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini");
        return false;
    }

	deviationLeft = config.check("deviationLeft",
                                    //Value(-0.12),
									Value(0),
                                    "The angle deviation of the left eye from 0 in rad (double).").asDouble();

	deviationRight = config.check("deviationRight",
                                    //Value(-0.0017),
									Value(0),
                                    "The angle deviation of the right eye from 0 in rad (double)").asDouble();
	deviationX = config.check("deviationX",
                                    //Value(-0.003),
									Value(0.1),
                                    "The angle deviation of the right eye with respect to the left in the X direction in rad (double).").asDouble();
	deviationZ = config.check("deviationZ",
                                    //Value(-0.006),
									Value(0),
                                    "The angle deviation of the right eye with respect to the left in the Z direction in rad (double).").asDouble();
	
	Leftfx = config.check("Leftfx",
                                    //Value(-0.006),
									Value(0),
                                    "The left camera parameter fx (double).").asDouble();
	Leftfy = config.check("Leftfy",
                                    //Value(-0.006),
									Value(0),
                                    "The left camera parameter fy (double).").asDouble();
	Leftcx = config.check("Leftcx",
                                    //Value(-0.006),
									Value(0),
                                    "The left camera parameter cx (double).").asDouble();
	Leftcy = config.check("Leftcx",
                                    //Value(-0.006),
									Value(0),
                                    "The left camera parameter cy (double).").asDouble();

	Rightfx = config.check("Rightfx",
                                    //Value(-0.006),
									Value(0),
                                    "The right camera parameter fx (double).").asDouble();
	Rightfy = config.check("Rightfy",
                                    //Value(-0.006),
									Value(0),
                                    "The right camera parameter fy (double).").asDouble();
	Rightcx = config.check("Rightcx",
                                    //Value(-0.006),
									Value(0),
                                    "The right camera parameter cx (double).").asDouble();
	Rightcy = config.check("Rightcx",
                                    //Value(-0.006),
									Value(0),
                                    "The right camera parameter cy (double).").asDouble();
	
	oldWidth = 700;
	oldHeight = 700;
	

	// check for toplevel configuration options
    // we need to know the name of the server controlboard to connect to
    Value *valNameControlboard;
    if(!config.check("nameControlboard",
                     valNameControlboard,
                     "Port name of the server controlboard to connect to (string).")){
        cout << endl;                         
        cout << "Please specify the port name of the server controlboard to connect to." << endl;
        return false;
    }

	// open remote controlboard
    Bottle botControlboard(string(
                           string("(device remote_controlboard) ") +
                           string("(local ") + string(this->getName("controlboard").c_str()) + string(") ") +
                           string("(remote ") + string(valNameControlboard->asString().c_str()) + string(") ")
                           ).c_str());
    _dd.open(botControlboard);
    if(!_dd.view(_ienc)){
        cout << "Motor controlboard does not implement IEncoders!" << endl;
        return false;
    }
    if(!_dd.view(_ipos)){
        cout << "Motor controlboard does not implement IPositionControl!" << endl;
        return false;
    }
    _numAxes = 0;
    _ienc->getAxes(&_numAxes);
    if (_numAxes == 0){
        cout << "*** Controlboard not available! Setting encoder values to zero." << endl;
        _encoders = new double[6];
        for (int i = 0; i < 6; i++)
            _encoders[i] = 0.0;
    }
    else if (_numAxes < 6){
        cout << "Number of motor axes outside expected range (available: " << _numAxes << " required 6)." << endl;
        return false;
    }
    else{
        _encoders = new double[_numAxes];
		
    }
   	
    //Intrinsic camera parameters;
	
	cvSet(rotLeft, cvScalar(0));
	cvSet(rotRight, cvScalar(0));
	cvSet(tRight, cvScalar(0));
	cvSet(tLeft, cvScalar(0));
	cvSet(aRight, cvScalar(0));
	cvSet(aLeft, cvScalar(0));

	cvmSet(rotLeft, 0, 0, 1 );
	cvmSet(rotLeft, 1, 1, 1 );
	cvmSet(rotLeft, 2, 2, 1 );

	//Left fx
	cvmSet(aLeft, 0, 0, Leftfx);
	//Left cx
	cvmSet(aLeft, 0, 2, Leftcx);
	//Left fy
	cvmSet(aLeft, 1, 1, Leftfy);
	//Left cy
	cvmSet(aLeft, 1, 2, Leftcy);
	cvmSet(aLeft, 2, 2, 1);

	//Right fx
	cvmSet(aRight, 0, 0, Rightfx);
	//Right cx
	cvmSet(aRight, 0, 2, Rightcx);
	//Right fy
	cvmSet(aRight, 1, 1, Rightfy);
	//Right cy
	cvmSet(aRight, 1, 2, Rightcy);
	cvmSet(aRight, 2, 2, 1);

	

    // open image ports
    imgPortRight.open(getName("right"));
    imgPortLeft.open(getName("left"));
    imgOutColor.open(getName("outColor"));
	imgPortDepthOpenCv.open(getName("mapOpenCv"));
	imgPortTransfomRight.open(getName("transformright"));
	imgPortTransfomLeft.open(getName("transformleft"));
	imgPortDerivXDepth.open(getName("Xderivate"));
	imgPortDerivYDepth.open(getName("Yderivate"));
	imgZDP.open(getName("ZDP"));
	imgOutD.open(getName("disparity"));
	imgDepthMap.open(getName("depthMap"));
	
    encoders.open(getName("encoders"));
    // open config port
    _configPort.open(getName("conf"));
    attach(_configPort, true);

    Time::turboBoost();
	


	

    return true;
}

bool DisparityMapModule::close(){
    
    imgPortRight.close();
    imgPortLeft.close();
	imgOutColor.close();
	imgPortDepthOpenCv.close();
    //imgPortDepthMap.close();
	imgPortTransfomRight.close();
	imgPortTransfomLeft.close();
	imgPortDerivXDepth.close();
	imgPortDerivYDepth.close();
	imgZDP.close();
	encoders.close();
	imgOutD.close();
	
    
    return true;
}

bool DisparityMapModule::interruptModule(){
    
    imgPortRight.interrupt();
    imgPortLeft.interrupt();
	imgOutColor.interrupt();
	imgPortDepthOpenCv.interrupt();
    //imgPortDepthMap.interrupt();
    imgPortTransfomRight.interrupt();
	imgPortTransfomLeft.interrupt();
	imgPortDerivXDepth.interrupt();
	imgPortDerivYDepth.interrupt();
	imgZDP.interrupt();
	encoders.interrupt();
	imgOutD.interrupt();
	
    return true;
}


bool DisparityMapModule::updateModule(){
	

	//_framerate.addStartTime(yarp::os::Time::now());
	
	clock_t begin=clock();

   int timeStart = Time::now();
    // read image from port
    ImageOf<PixelRgb> *imgRight = imgPortRight.read();
	ImageOf<PixelRgb> *imgLeft = imgPortLeft.read();
	ImageOf<PixelRgb> &destRight=imgPortTransfomRight.prepare();
	ImageOf<PixelRgb> &destLeft=imgPortTransfomLeft.prepare();
	ImageOf<PixelFloat> &imgDepthOpenCv =imgPortDepthOpenCv.prepare();
	ImageOf<PixelRgb> &imgColorOut =imgOutColor.prepare();
	ImageOf<PixelFloat> &xDerivate =imgPortDerivXDepth.prepare();
	ImageOf<PixelFloat> &yDerivate=imgPortDerivYDepth.prepare();
	ImageOf<PixelFloat> &mapZDP=imgZDP.prepare();
	ImageOf<PixelMono> &imageDisparity=imgOutD.prepare();
	ImageOf<PixelFloat> &imageDepth=imgDepthMap.prepare();
	
	Bottle &encodersValue =encoders.prepare();

	//read the encoders
	_ienc->getEncoders(_encoders); 
	
	if (imgRight==NULL || imgLeft==NULL){
		return true;
	}
    // if image size changes, need to resize buffered images
    if (imgRight->width() != imgLeft->width()|| imgRight->height() != imgLeft->height()){        
		return false;
    }
	//the image should be 320X240
	if (imgRight->width() !=320 || imgRight->height() !=240){        
		return false;
    }
		
	
	//the deviation of the cameras in the home position from the
	//straight direction, the actual encoder readings and the distance
	//between the left and the right camera are used to calculate the rotation
	//and translation of the right camera from the left camera 
	rotationTraslation(_encoders[4],_encoders[5]);	
	clock_t end=clock();
	


	
	destLeft.resize(*imgLeft);
	destRight.resize(*imgLeft);
	begin=clock();
	
	rectification((IplImage *)imgLeft->getIplImage(),(IplImage *)imgRight->getIplImage(),aRight,aLeft,rotRight,rotLeft,
		tRight, tLeft,(IplImage *)destLeft.getIplImage(),(IplImage *)destRight.getIplImage(),320,240);
	end=clock();
	

	
	// if image size changes becouse of the transformation, need to resize buffered images
    if (destRight.width() != destLeft.width()|| destRight.height() != destLeft.height()){        
		return false;
    }
	if (destRight.width() != oldWidth || destRight.height() != oldHeight ){
       
		oldWidth=destRight.width();
		oldHeight=destRight.height();
		
    }
	begin=clock();
	
	IplImage* imgGrayRight = cvCreateImage(cvSize(oldWidth, oldHeight), IPL_DEPTH_8U, 1);
	IplImage* imgGrayLeft = cvCreateImage(cvSize(oldWidth, oldHeight), IPL_DEPTH_8U, 1);
	
	//convertin images to gray scale to do the correlation
	imgDepthOpenCv.resize(destRight);
	cvCvtColor(destRight.getIplImage(), imgGrayRight, CV_RGB2GRAY);
	cvCvtColor(destLeft.getIplImage(), imgGrayLeft, CV_RGB2GRAY);

	//calculating the disparity map	
	mapZDP.resize(imgDepthOpenCv);
	CvMat* disp = cvCreateMat( oldHeight,oldWidth, CV_16S );
	CvMat* filter=cvCreateMat( oldHeight,oldWidth, CV_8U );
	finalMapOpenCV(imgGrayRight, imgGrayLeft,disp,minDisparity,numDisparities);

	//doing pyramids to improve the depth map
	CvMat* disp2 = cvCreateMat( oldHeight/2,oldWidth/2, CV_16S );
	IplImage* imgGrayRight2=cvCreateImage(cvSize(oldWidth/2, oldHeight/2), IPL_DEPTH_8U, 1);
	IplImage* imgGrayLeft2=cvCreateImage(cvSize(oldWidth/2, oldHeight/2), IPL_DEPTH_8U, 1);
	cvPyrDown( imgGrayRight, imgGrayRight2);
	cvPyrDown( imgGrayLeft, imgGrayLeft2);
	finalMapOpenCV(imgGrayRight2, imgGrayLeft2,disp2,minDisparity/2,numDisparities/2);

	CvMat* disp4 = cvCreateMat( oldHeight/4,oldWidth/4, CV_16S );
	IplImage* imgGrayRight4=cvCreateImage(cvSize(oldWidth/4, oldHeight/4), IPL_DEPTH_8U, 1);
	IplImage* imgGrayLeft4=cvCreateImage(cvSize(oldWidth/4, oldHeight/4), IPL_DEPTH_8U, 1);
	cvPyrDown( imgGrayRight2, imgGrayRight4);
	cvPyrDown( imgGrayLeft2, imgGrayLeft4);
	finalMapOpenCV(imgGrayRight4, imgGrayLeft4,disp4,minDisparity/4,numDisparities/4);

	CvMat* disp8 = cvCreateMat( oldHeight/8,oldWidth/8, CV_16S );
	IplImage* imgGrayRight8=cvCreateImage(cvSize(oldWidth/8, oldHeight/8), IPL_DEPTH_8U, 1);
	IplImage* imgGrayLeft8=cvCreateImage(cvSize(oldWidth/8, oldHeight/8), IPL_DEPTH_8U, 1);
	cvPyrDown( imgGrayRight4, imgGrayRight8);
	cvPyrDown( imgGrayLeft4, imgGrayLeft8);
	finalMapOpenCV(imgGrayRight8, imgGrayLeft8,disp8,minDisparity/8,numDisparities/8);

	// filling up the holes using the pyramid.
	upPyramidInformation(disp8,disp4, minDisparity/4);
	upPyramidInformation(disp4,disp2, minDisparity/2);
	upPyramidInformation(disp2,disp, minDisparity);
	cvNormalize( disp,(IplImage*)imgDepthOpenCv.getIplImage(), 0,255 , CV_MINMAX );
	imgColorOut.resize(imgDepthOpenCv);
	showColorDepthMap(imgColorOut,disp);
	imageDisparity.resize(imgDepthOpenCv);
	cvNormalize( disp,(IplImage*)imageDisparity.getIplImage(),0,numDisparities,CV_MINMAX );

	//calculating the depthMap

	/*imageDepth.resize(*imgRight);
    IplImage* disparityFloat=cvCreateImage(cvSize(oldWidth, oldHeight), IPL_DEPTH_32F, 1);
	cvConvertScale (disp,disparityFloat,1,0);*/


	//Zero disparity filter
	cvConvertScaleAbs( disp, filter,1,0);
	cvThreshold(filter, filter,16*2,1, CV_THRESH_BINARY_INV );
	cvNormalize( filter,(IplImage*)mapZDP.getIplImage(), 0,255 , CV_MINMAX );
    
	//derivate X and Y
	xDerivate.resize(imgDepthOpenCv);
	yDerivate.resize(imgDepthOpenCv);
	IplImage *xIplDerivate = (IplImage *)xDerivate.getIplImage();
	IplImage *yIplDerivate = (IplImage *)yDerivate.getIplImage();

	cvSobel( disp,xIplDerivate,1,0,5);
	cvSobel( disp,yIplDerivate,0,1,5);
	//saveFiles(xDerivate,"derivadaX.m");
	//calculate sign
	CvMat* signx = cvCreateMat( oldHeight,oldWidth, CV_32FC1);
	CvMat* signy = cvCreateMat( oldHeight,oldWidth, CV_32FC1);
	cvThreshold(xIplDerivate, signx,0,2, CV_THRESH_BINARY);
	cvThreshold(yIplDerivate, signy,0,2, CV_THRESH_BINARY);
	cvAddS( signx, cvScalar(-1), signx);
	cvAddS( signy, cvScalar(-1), signy);

	//calculate log change 0
	CvMat* logMatx = cvCreateMat( oldHeight,oldWidth, CV_32FC1);
	CvMat* logMaty = cvCreateMat( oldHeight,oldWidth, CV_32FC1);
	cvAbsDiffS( xIplDerivate, logMatx, cvScalar(0) );
	cvAbsDiffS( yIplDerivate, logMaty, cvScalar(0) );
	cvMaxS( logMatx,1,logMatx );
	cvMaxS( logMaty,1,logMaty );
	cvLog( logMatx, logMatx );
	cvLog( logMaty, logMaty );


	//putting all together
	cvMul( logMatx, signx, xIplDerivate);
	cvMul( logMaty, signy, yIplDerivate);
	cvNormalize(xIplDerivate, xIplDerivate,0,255,CV_MINMAX);
	cvNormalize( yIplDerivate, yIplDerivate,0,255,CV_MINMAX);
	
	//adding encoders
	encodersValue.clear();
	encodersValue.addDouble(_encoders[3]);
	encodersValue.addDouble(_encoders[4]);
	encodersValue.addDouble(_encoders[5]);
	
	cvReleaseMat(&filter);
	cvReleaseMat(&signx);
	cvReleaseMat(&signy);
	cvReleaseMat(&logMatx);
	cvReleaseMat(&logMaty);
	//cvReleaseMat(&medianx);
	//cvReleaseMat(&mediany);
	cvReleaseMat(&disp);
	cvReleaseMat(&disp2);
	cvReleaseMat(&disp4);
	cvReleaseMat(&disp8);
	//cvReleaseMat(&aNewRight);
	//cvReleaseMat(&aNewLeft);
	mutex.post();
	imgOutD.write();
	encoders.write();
	imgPortDerivXDepth.write();
	imgPortDerivYDepth.write();
	imgPortDepthOpenCv.write();
	imgOutColor.write();
	imgPortTransfomRight.write();
	imgPortTransfomLeft.write();
	imgZDP.write();

	//centerDepth->~ImageOf<yarp::sig::PixelFloat>();
	
	
	cvReleaseImage(&imgGrayRight);
	cvReleaseImage(&imgGrayLeft);
	cvReleaseImage(&imgGrayRight2);
	cvReleaseImage(&imgGrayLeft2);
	cvReleaseImage(&imgGrayRight4);
	cvReleaseImage(&imgGrayLeft4);
	cvReleaseImage(&imgGrayRight8);
	cvReleaseImage(&imgGrayLeft8);
	
    return true;
}
bool DisparityMapModule::respond(const Bottle &command,Bottle &reply){
   //TODO fill this method
    return true;
}
double DisparityMapModule::diffclock(clock_t clock1,clock_t clock2)
{
	double diffticks=clock1-clock2;
	double diffms=(diffticks*1000)/CLOCKS_PER_SEC;
	return diffms;
}
void DisparityMapModule::rotationTraslation(double j4,double j5){
	
	double tz=deviationZ;
	double tx=deviationX;
	double ty=j5*3.14159/180+deviationLeft;
	double diff = (j5-j4)*3.14159/180-deviationRight;
	
	cvmSet(rotRight, 0, 0, cos(ty)*cos(tz));
	cvmSet(rotRight, 0, 1, sin(tx)*sin(ty)*cos(tz)-cos(tx)*sin(tz));
	cvmSet(rotRight, 0, 2, cos(tx)*sin(ty)*cos(tz)+sin(tx)*sin(tz));
	
	cvmSet(rotRight, 1, 0, cos(ty)*sin(tz));
	cvmSet(rotRight, 1, 1, sin(tx)*sin(tx)*sin(tz)+cos(tx)*cos(tz));
	cvmSet(rotRight, 1, 2, cos(tx)*sin(tx)*sin(tz)-sin(tx)*cos(tz));
	
	cvmSet(rotRight, 2, 0, -sin(ty));
	cvmSet(rotRight, 2, 1, sin(tx)*cos(ty));
	cvmSet(rotRight, 2, 2, cos(tx)*cos(ty));
	//cvmSet(tRight, 0, 0, -65.98*cos(diff));
	//cvmSet(tRight, 1, 0, 1.4);
	//cvmSet(tRight, 2, 0, 65.98*sin(diff));
	cvmSet(tRight, 0, 0, -65.98*cos(diff));
	cvmSet(tRight, 1, 0, 1.4);
	cvmSet(tRight, 2, 0, 65.98*sin(diff));
	
	/*double tz=deviationZ;
	double tx=deviationX;
	double diff = (j5-j4)*3.14159/180-deviationRight;
	double ty=diff;
	cvmSet(rotRight, 0, 0, cos(ty)*cos(tz));
	cvmSet(rotRight, 0, 1, sin(tx)*sin(ty)*cos(tz)-cos(tx)*sin(tz));
	cvmSet(rotRight, 0, 2, cos(tx)*sin(ty)*cos(tz)+sin(tx)*sin(tz));
	cvmSet(rotRight, 1, 0, cos(ty)*sin(tz));
	cvmSet(rotRight, 1, 1, sin(tx)*sin(tx)*sin(tz)+cos(tx)*cos(tz));
	cvmSet(rotRight, 1, 2, cos(tx)*sin(tx)*sin(tz)-sin(tx)*cos(tz));
	cvmSet(rotRight, 2, 0, -sin(ty));
	cvmSet(rotRight, 2, 1, sin(tx)*cos(ty));
	cvmSet(rotRight, 2, 2, cos(tx)*cos(ty));
	//cvmSet(tRight, 0, 0, -65.98*cos(diff));
	//cvmSet(tRight, 1, 0, 1.4);
	//cvmSet(tRight, 2, 0, 65.98*sin(diff));
	cvmSet(tRight, 0, 0, -64.87*cos(diff)-8.588*sin(diff));
	cvmSet(tRight, 1, 0, -0.0968);
	cvmSet(tRight, 2, 0, -64.87*sin(diff)+8.588*cos(diff));*/
}

void DisparityMapModule::saveFiles( ImageOf<PixelFloat> &img,string name){
	//if(firstime){

		
	string file="D:/data/imagen_"+name;
		
		
	outClientFile.open(file.data(),ios::out);
	//outClientFile<<img;
	if(!outClientFile){
		cout << "It is not possible to open the file " << name << endl;
	}
	outClientFile<<"dephtMap"<<"=[";
	for(int x=0;x<img.width();x++){
		for(int y=0;y<img.height();y++){
			outClientFile<<img(x,y)<<",";
			if(y==(img.height()-1)){
				outClientFile<<";";
			}
		}
	}
	outClientFile<<"];";
	outClientFile.close();
}
void DisparityMapModule::saveFile( float *intens,string name){
	//if(firstime){
	int j,i,wi,w;
		
	string file="D:/data/imagen_vector_"+name;
		
	outClientFile.open(file.data(),ios::out);

	for(int j=0;j<oldHeight;j++){
		wi=oldWidth*j;
		for(int i=0; i<oldWidth;++i){
			w=wi+i;
			outClientFile<<intens[w]<<" ";
		}
	}	
	
	outClientFile.close();
}

void DisparityMapModule::finalMapOpenCV(IplImage *intenseRigth, 
                          IplImage *intenseLeft,
						 CvMat* disp, int minD, int numD){

   
	
	//for 5x5 up to 21x21
	state->preFilterSize=filterSize;
	// betwen 1 and 63
	state->preFilterCap=filterCap;
	// Could be 5x5,7x7, ..., 21x21
	state->SADWindowSize=windowSize;
	state->minDisparity=minD;
	//Number of pixels to search
    //post filters (knock out bad matches):
	//multiply of 16
	state->numberOfDisparities=numD;
	//minimum allowed
	state->textureThreshold=threshold;
	// Filter out if: [ match_val - min_match < uniqRatio*min_match ] over the corr window area
	state->uniquenessRatio=uniqueness;
	
    cvFindStereoCorrespondenceBM( intenseLeft, intenseRigth, disp, state );
	
	
	//cvNormalize( vdisp,, 0,256 , CV_MINMAX );
	//gray2Float (vdisp,(IplImage*)depthMap.getIplImage());
   
}
void DisparityMapModule::showColorDepthMap(ImageOf<PixelRgb>& imgColorOut, CvMat*  disp){
	
	int base = numDisparities;
	int min = minDisparity;
	int zero = -255*min/base;
	int limitUp=1.5;
	int limitDown=-1.5;
	int aux=0;
	short pixel=0;
	short *dispSrc;
	dispSrc=(disp->data.s);
	for (int i=0; i<oldWidth; i++) {
		for (int j=0; j<oldHeight; j++){
			
			pixel=(short)dispSrc[disp->width*j+i]/16;

			if(minDisparity < 0 && pixel<limitDown && pixel>=minDisparity ){
				aux=(pixel*255/min);
				imgColorOut(i,j).b = (unsigned char)255-aux;
				
				if(aux*3>255){
					imgColorOut(i,j).g = 0;
					imgColorOut(i,j).r = 0;
				}
				else{
					imgColorOut(i,j).g = (unsigned char)255-aux*3;
					imgColorOut(i,j).r = (unsigned char)255-aux*3;
				}
			}
			else if(minDisparity < 0 && pixel>limitUp){
				aux=(pixel*255/(base+min));
				imgColorOut(i,j).g = (unsigned char)255-aux;
				if(aux*3>255){
					imgColorOut(i,j).b = 0;
					imgColorOut(i,j).r = 0;
				}
				else{
					imgColorOut(i,j).b = (unsigned char)255-aux*3;
					imgColorOut(i,j).r = (unsigned char)255-aux*3;
				}
				
			}
			else if(minDisparity < 0 && pixel>=limitDown && pixel<=limitUp){
				imgColorOut(i,j).b =(unsigned char)255;
				imgColorOut(i,j).g =(unsigned char)255;
				imgColorOut(i,j).r =(unsigned char)255;
			}
			else {
				imgColorOut(i,j).b =(unsigned char)0;
				imgColorOut(i,j).g =(unsigned char)0;
				imgColorOut(i,j).r =(unsigned char)0;
			}
		}
	}

}

void DisparityMapModule::transformationMatrix(CvMat * Ar,CvMat * Al,CvMat *Rr, CvMat *Rl,CvMat *Tr, CvMat *Tl,double dx1,double dx2, double dy,
						  CvMat *TTl,CvMat *TTr){


CvMat *Pn1=0,*Pn2=0,*Po1=0,*Po2=0,*cr=0,*cl=0,*aux1=0,*aux2=0;
CvMat *v1=0,*v2=0,*v3=0,*Rl2=0;
CvMat *An1=0,*An2=0,*Rnr=0,*Rnl=0,*Tnr=0,*Tnl=0;


Pn2=cvCreateMat(3,3,CV_64FC1);
Pn1=cvCreateMat(3,3,CV_64FC1);
Po1=cvCreateMat(3,3,CV_64FC1);
Po2=cvCreateMat(3,3,CV_64FC1);
cr=cvCreateMat(3,1,CV_64FC1);
cl=cvCreateMat(3,1,CV_64FC1);
v1=cvCreateMat(3,1,CV_64FC1);
v2=cvCreateMat(3,1,CV_64FC1);
v3=cvCreateMat(3,1,CV_64FC1);
Rl2=cvCreateMat(1,3,CV_64FC1);

An1=cvCreateMat(3,3,CV_64FC1);
An2=cvCreateMat(3,3,CV_64FC1);
Rnr=cvCreateMat(3,3,CV_64FC1);
Rnl=cvCreateMat(3,3,CV_64FC1);
Tnr=cvCreateMat(3,1,CV_64FC1);
Tnl=cvCreateMat(3,1,CV_64FC1);



//optical center right
aux1=cvCreateMat(3,3,CV_64FC1);
aux2=cvCreateMat(3,1,CV_64FC1);
cvTranspose( Rr, aux1);
cvMatMul( aux1, Tr,cr );
cvConvertScale( cr, cr, -1);

//cout<<"cr= "<<cvmGet(cr,0,0)<<", "<<cvmGet(cr,1,0)<<", "<<cvmGet(cr,2,0)<<endl;

//optical center left
cvTranspose( Rl, aux1);
cvMatMul( aux1, Tl, cl );
cvConvertScale( cl, cl, -1);

//cout<<"cl= "<<cvmGet(cl,0,0)<<", "<<cvmGet(cl,1,0)<<", "<<cvmGet(cl,2,0)<<endl;


//new x axis baseline cr-cl
cvScaleAdd( cl, cvScalar(-1), cr, v1 );
//cout<<"v1= "<<cvmGet(v1,0,0)<<", "<<cvmGet(v1,1,0)<<", "<<cvmGet(v1,2,0)<<endl;
// new y axes (orthogonal to old z and new x)
cvGetRow( Rl, Rl2, 2);
cvTranspose( Rl2, Rl2);
cvCrossProduct( Rl2,v1, v2);
// new z axes (no choice, orthogonal to baseline and y)
cvCrossProduct(v1,v2,v3);
// new extrinsic (translation unchanged)
cvNormalize( v1,v1);
cvNormalize( v2,v2);
cvNormalize( v3,v3);
cvTranspose( v1,v1);
cvTranspose( v2,v2);
cvTranspose( v3,v3);
for(int i=0;i<3;i++){
	cvmSet( Rnr, 0, i, cvmGet( v1,0,i ) );
	cvmSet( Rnr, 1, i, cvmGet( v2,0,i ) );
	cvmSet( Rnr, 2, i, cvmGet( v3,0,i ) );
}

/*cout<<"Rnr=["<<cvmGet(Rnr,0,0)<<", "<<cvmGet(Rnr,0,1)<<", "<<cvmGet(Rnr,0,2)<<endl;
cout<<"     "<<cvmGet(Rnr,1,0)<<", "<<cvmGet(Rnr,1,1)<<", "<<cvmGet(Rnr,1,2)<<endl;
cout<<"     "<<cvmGet(Rnr,2,0)<<", "<<cvmGet(Rnr,2,1)<<", "<<cvmGet(Rnr,2,2)<<endl;*/

// new intrinsic (arbitrary) 

cvCopy(Rnr,Rnl);
cvCopy(Ar,An1);
cvmSet(An1,0,1,0);
cvCopy(Ar,An2);
cvmSet(An2,0,1,0);

/*cout<<"Ar=["<<cvmGet(Ar,0,0)<<", "<<cvmGet(Ar,0,1)<<", "<<cvmGet(Ar,0,2)<<endl;
cout<<"     "<<cvmGet(Ar,1,0)<<", "<<cvmGet(Ar,1,1)<<", "<<cvmGet(Ar,1,2)<<endl;
cout<<"     "<<cvmGet(Ar,2,0)<<", "<<cvmGet(Ar,2,1)<<", "<<cvmGet(Ar,2,2)<<endl;*/

// translate image centers 
cvmSet(An1,0,2,cvmGet( An1,0, 2 )+dx1);
cvmSet(An1,1,2,cvmGet( An1,1, 2 )+dy);

cvmSet(An2,0,2,cvmGet( An2,0, 2 )+dx2);
cvmSet(An2,1,2,cvmGet( An2,1, 2 )+dy);

/*cout<<"dx1= "<<dx1<<" dx2= "<<dx2<<" dy "<<dy<<endl<<endl;

cout<<"An2=["<<cvmGet(An2,0,0)<<", "<<cvmGet(An2,0,1)<<", "<<cvmGet(An2,0,2)<<endl;
cout<<"     "<<cvmGet(An2,1,0)<<", "<<cvmGet(An2,1,1)<<", "<<cvmGet(An2,1,2)<<endl;
cout<<"     "<<cvmGet(An2,2,0)<<", "<<cvmGet(An2,2,1)<<", "<<cvmGet(An2,2,2)<<endl;*/

// new projection matrices
cvMatMul( Rnr, cr, aux2 );
cvConvertScale( aux2, Tnr, -1);
cvMatMul( An2,Rnr,Pn2 );

cvMatMul( Rnr, cl, aux2 );
cvConvertScale( aux2, Tnl, -1);
cvMatMul( An1,Rnr,Pn1 );

//old projection matrices
cvMatMul( Ar,Rr,Po2 );
cvMatMul( Al,Rl,Po1 );
/*cout<<"Po2=["<<cvmGet(Po2,0,0)<<", "<<cvmGet(Po2,0,1)<<", "<<cvmGet(Po2,0,2)<<endl;
cout<<"     "<<cvmGet(Po2,1,0)<<", "<<cvmGet(Po2,1,1)<<", "<<cvmGet(Po2,1,2)<<endl;
cout<<"     "<<cvmGet(Po2,2,0)<<", "<<cvmGet(Po2,2,1)<<", "<<cvmGet(Po2,2,2)<<endl;
cout<<"Pn2=["<<cvmGet(Pn2,0,0)<<", "<<cvmGet(Pn2,0,1)<<", "<<cvmGet(Pn2,0,2)<<endl;
cout<<"     "<<cvmGet(Pn2,1,0)<<", "<<cvmGet(Pn2,1,1)<<", "<<cvmGet(Pn2,1,2)<<endl;
cout<<"     "<<cvmGet(Pn2,2,0)<<", "<<cvmGet(Pn2,2,1)<<", "<<cvmGet(Pn2,2,2)<<endl;

cout<<"Po1=["<<cvmGet(Po1,0,0)<<", "<<cvmGet(Po1,0,1)<<", "<<cvmGet(Po1,0,2)<<endl;
cout<<"     "<<cvmGet(Po1,1,0)<<", "<<cvmGet(Po1,1,1)<<", "<<cvmGet(Po1,1,2)<<endl;
cout<<"     "<<cvmGet(Po1,2,0)<<", "<<cvmGet(Po1,2,1)<<", "<<cvmGet(Po1,2,2)<<endl;
cout<<"Pn1=["<<cvmGet(Pn1,0,0)<<", "<<cvmGet(Pn1,0,1)<<", "<<cvmGet(Pn1,0,2)<<endl;
cout<<"     "<<cvmGet(Pn1,1,0)<<", "<<cvmGet(Pn1,1,1)<<", "<<cvmGet(Pn1,1,2)<<endl;
cout<<"     "<<cvmGet(Pn1,2,0)<<", "<<cvmGet(Pn1,2,1)<<", "<<cvmGet(Pn1,2,2)<<endl;*/
// rectifying image transformation
cvInvert( Po1, aux1,CV_LU );
cvMatMul( Pn1,aux1,TTl );
cvInvert( Po2, aux1,CV_LU );
cvMatMul( Pn2,aux1,TTr );


cvReleaseMat(&Pn1);
cvReleaseMat(&Pn2);
cvReleaseMat(&Po1);
cvReleaseMat(&Po2);
cvReleaseMat(&cr);
cvReleaseMat(&cl);
cvReleaseMat(&aux1);
cvReleaseMat(&aux2);
cvReleaseMat(&v1);
cvReleaseMat(&v2);
cvReleaseMat(&v3);
cvReleaseMat(&Rl2);
cvReleaseMat(&An1);
cvReleaseMat(&An2);
cvReleaseMat(&Rnr);
cvReleaseMat(&Rnl);
cvReleaseMat(&Tnr);
cvReleaseMat(&Tnl);
}

void DisparityMapModule::rectification(IplImage *left,IplImage *right,CvMat * Ar,CvMat * Al,CvMat *Rr, CvMat *Rl,
								   CvMat *Tr, CvMat *Tl,IplImage *destLeft,IplImage *destRight,
								   int originalWidth, int originalHeigh){

//  rectification without centeriing
CvMat *TTl=0,*TTr=0,*p=0,*px=0,*dL=0,*dR=0;


TTl=cvCreateMat(3,3,CV_64FC1);
TTr=cvCreateMat(3,3,CV_64FC1);
p=cvCreateMat(3,1,CV_64FC1);
px=cvCreateMat(3,1,CV_64FC1);
dL=cvCreateMat(3,1,CV_64FC1);
dR=cvCreateMat(3,1,CV_64FC1);

transformationMatrix(Ar,Al,Rr,Rl,Tr,Tl,0,0,0,TTl,TTr);
/*cout<<"TTl=["<<cvmGet(TTl,0,0)<<", "<<cvmGet(TTl,0,1)<<", "<<cvmGet(TTl,0,2)<<endl;
cout<<"     "<<cvmGet(TTl,1,0)<<", "<<cvmGet(TTl,1,1)<<", "<<cvmGet(TTl,1,2)<<endl;
cout<<"     "<<cvmGet(TTl,2,0)<<", "<<cvmGet(TTl,2,1)<<", "<<cvmGet(TTl,2,2)<<endl;
cout<<"TTr=["<<cvmGet(TTr,0,0)<<", "<<cvmGet(TTr,0,1)<<", "<<cvmGet(TTr,0,2)<<endl;
cout<<"     "<<cvmGet(TTr,1,0)<<", "<<cvmGet(TTr,1,1)<<", "<<cvmGet(TTr,1,2)<<endl;
cout<<"     "<<cvmGet(TTr,2,0)<<", "<<cvmGet(TTr,2,1)<<", "<<cvmGet(TTr,2,2)<<endl;*/
//centering LEFT image
cvmSet( p,0, 0,originalHeigh/2);
cvmSet( p,1, 0,originalWidth/2);
cvmSet( p,2, 0,1);
cvMatMul( TTl, p, px );

//cout<<"p=[ "<<cvmGet(p,0,0)<<", "<<cvmGet(p,1,0)<<", "<<cvmGet(p,2,0)<<endl;
//cout<<"px=[ "<<cvmGet(px,0,0)<<", "<<cvmGet(px,1,0)<<", "<<cvmGet(px,2,0)<<endl;
//TODO change this scale it is not correct;
double factora=cvmGet(px,2,0);
cvConvertScale( px,px,1/cvmGet(px,2,0));
cvSub( p, px,dL);
//cvConvertScale( dL,dL,1/factora);
//cout<<"dL=[ "<<cvmGet(dL,0,0)<<", "<<cvmGet(dL,1,0)<<", "<<cvmGet(dL,2,0)<<endl;

//centering RIGHT image
cvmSet( p,0, 0,originalHeigh/2);
cvmSet( p,1, 0,originalWidth/2);
cvmSet( p,2, 0,1);
cvMatMul( TTr, p, px );

//cout<<"p=[ "<<cvmGet(p,0,0)<<", "<<cvmGet(p,1,0)<<", "<<cvmGet(p,2,0)<<endl;
//cout<<"px=[ "<<cvmGet(px,0,0)<<", "<<cvmGet(px,1,0)<<", "<<cvmGet(px,2,0)<<endl;
cvConvertScale( px,px,1/cvmGet(px,2,0));
factora=cvmGet(px,2,0);

cvSub( p, px,dR);
//cvConvertScale( dR,dR,1/factora);
//cout<<"dR=[ "<<cvmGet(dR,0,0)<<", "<<cvmGet(dR,1,0)<<", "<<cvmGet(dR,2,0)<<endl;

//vertical diplacement must be the same
//rectification with centering

transformationMatrix(Ar,Al,Rr,Rl,Tr,Tl,cvmGet(dL,0,0),cvmGet(dR,0,0),cvmGet(dR,1,0),TTl,TTr);
/*cout<<"TTl=["<<cvmGet(TTl,0,0)<<", "<<cvmGet(TTl,0,1)<<", "<<cvmGet(TTl,0,2)<<endl;
cout<<"     "<<cvmGet(TTl,1,0)<<", "<<cvmGet(TTl,1,1)<<", "<<cvmGet(TTl,1,2)<<endl;
cout<<"     "<<cvmGet(TTl,2,0)<<", "<<cvmGet(TTl,2,1)<<", "<<cvmGet(TTl,2,2)<<endl;
cout<<"TTr=["<<cvmGet(TTr,0,0)<<", "<<cvmGet(TTr,0,1)<<", "<<cvmGet(TTr,0,2)<<endl;
cout<<"     "<<cvmGet(TTr,1,0)<<", "<<cvmGet(TTr,1,1)<<", "<<cvmGet(TTr,1,2)<<endl;
cout<<"     "<<cvmGet(TTr,2,0)<<", "<<cvmGet(TTr,2,1)<<", "<<cvmGet(TTr,2,2)<<endl;*/

cvWarpPerspective( left,destLeft, TTl,CV_INTER_CUBIC+CV_WARP_FILL_OUTLIERS,cvScalarAll(0) );
cvWarpPerspective( right,destRight, TTr,CV_INTER_CUBIC+CV_WARP_FILL_OUTLIERS,cvScalarAll(0) );



cvReleaseMat(&TTl);
cvReleaseMat(&TTr);
cvReleaseMat(&p);
cvReleaseMat(&px);
cvReleaseMat(&dL);
cvReleaseMat(&dR);

}

void DisparityMapModule::upPyramidInformation(CvMat *lowLayer,CvMat *layer, int minD){
// filling up the holes using the pyramid.
	CvMat* lowtoHigh = cvCreateMat( layer->height,layer->width, CV_16S );
	cvPyrUp( lowLayer, lowtoHigh );
	CvMat *cvTh = cvCreateMat( layer->height,layer->width, CV_16S );
	cvMinS( layer, (minD+1)*16, cvTh);
	cvMaxS( cvTh, minD*16, cvTh);
	cvNormalize( cvTh,cvTh, 0,1 , CV_MINMAX );
	cvMul(layer, cvTh,layer,1);
	cvSubRS( cvTh, cvScalar(1), cvTh);
	cvNormalize( cvTh,cvTh, 0,1 , CV_MINMAX );

	cvMul( lowtoHigh, cvTh, lowtoHigh,2);
	cvAdd(lowtoHigh,layer,layer);
	cvReleaseMat(&lowtoHigh);
	cvReleaseMat(&cvTh);
}
void DisparityMapModule::depthMapCalculation(IplImage *disparity,IplImage *depthMap){

	//depth is equal to b*f/d
	//baseline magnitud
	/*float x=cvmGet(tRight, 0, 0, -6.885*cos(diff))^2;
	float y=cvmGet(tRight, 1, 0, -0.11)^2;
	float z=cvmGet(tRight, 2, 0, 6.885*sin(diff))^2;
	float focalLength=Rightfx;
	float baseline=sqrt(x+z+y);
	cvConvertScale (disparity,depthMap,baseline,0);

	//depthMap = cv
	double tz=deviationZ;
	double tx=deviationX;
	double ty=j5*3.14159/180+deviationLeft;
	double diff = (j5-j4)*3.14159/180-deviationRight;
	cvmSet(rotRight, 0, 0, cos(ty)*cos(tz));
	cvmSet(rotRight, 0, 1, sin(tx)*sin(ty)*cos(tz)-cos(tx)*sin(tz));
	cvmSet(rotRight, 0, 2, cos(tx)*sin(ty)*cos(tz)+sin(tx)*sin(tz));
	cvmSet(rotRight, 1, 0, cos(ty)*sin(tz));
	cvmSet(rotRight, 1, 1, sin(tx)*sin(tx)*sin(tz)+cos(tx)*cos(tz));
	cvmSet(rotRight, 1, 2, cos(tx)*sin(tx)*sin(tz)-sin(tx)*cos(tz));
	cvmSet(rotRight, 2, 0, -sin(ty));
	cvmSet(rotRight, 2, 1, sin(tx)*cos(ty));
	cvmSet(rotRight, 2, 2, cos(tx)*cos(ty));
	//cvmSet(tRight, 0, 0, -65.98*cos(diff));
	//cvmSet(tRight, 1, 0, 1.4);
	//cvmSet(tRight, 2, 0, 65.98*sin(diff));
	cvmSet(tRight, 0, 0, -6.885*cos(diff));
	cvmSet(tRight, 1, 0, -0.11);
	cvmSet(tRight, 2, 0, 6.885*sin(diff));*/

}