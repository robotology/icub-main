// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/vis/CvFaceSalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::vis;

CvFaceSalience::CvFaceSalience(){
    
    _ocvGrayImg = NULL;
    _sizeOld.width = -1;
    _sizeOld.height = -1;
}

CvFaceSalience::~CvFaceSalience(){
    close();
}

bool CvFaceSalience::open(yarp::os::Searchable& config){
    
    // group filter already checks for a 
    // subconfiguration group of name [filter_name]
    return configure(config);
}

bool CvFaceSalience::close(){
    bool ok = Salience::close();
    releaseImages();
    return ok;
}

bool CvFaceSalience::configure(yarp::os::Searchable& config){

    bool ok = Salience::open(config);
    if (!ok)
        return false;

    string cascadeFile = config.check("haarCascadeFilename",
                            Value(""),
                            "Absolute path to the haar cascade xml file (string).").asString().c_str();
    if (cascadeFile == string("")){
        cout << endl << "No haar cascade data file specified." << endl << endl;
        return false;
    }
    _haarCascade = (CvHaarClassifierCascade*)cvLoad( cascadeFile.c_str(), 0, 0, 0 );
    if (!_haarCascade){
        cout << endl << "ERROR: Could not load classifier cascade (" << cascadeFile << ")" << endl << endl;;
        return false;
    }
    _storage = cvCreateMemStorage(0);
    return true;
}

void CvFaceSalience::initImages(CvSize size){
    releaseImages();
    _ocvGrayImg = cvCreateImage(size, 8, 1);
}

void CvFaceSalience::releaseImages(){
    if(_ocvGrayImg != NULL)
        cvReleaseImage(&_ocvGrayImg);
    _ocvGrayImg = NULL;
}

void CvFaceSalience::applyImpl(ImageOf<PixelRgb>& src, 
                           ImageOf<PixelRgb>& dest,
                           ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);
	dest.zero();
	sal.zero();

    if (_sizeOld.width != src.width() || 
        _sizeOld.height != src.height())
        initImages(cvSize(src.width(), src.height()));

    cvClearMemStorage( _storage );
    cvCvtColor(src.getIplImage(), _ocvGrayImg, CV_RGB2GRAY);
    cvEqualizeHist(_ocvGrayImg, _ocvGrayImg);

    CvSeq* faces = cvHaarDetectObjects( _ocvGrayImg, _haarCascade, _storage,
                                            1.1, 2, 0/*CV_HAAR_DO_CANNY_PRUNING*/,
                                            cvSize(5,5));
                                            //cvSize(30, 30) );

	CvPoint pt1, pt2;
	//cout << "faces: " << faces->total << endl;
	for( int i = 0; i < faces->total; i++ )
    {
        int j;
        CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
		int sizex = r->width;
		int sizey = r->height;
		if ((sizex-1)%2 != 0)
			sizex++;
		if ((sizey-1)%2 != 0)
			sizey++;
		
		// create kernel
		float **kernel = new float*[sizex];
		for (j = 0; j < sizex; j++)
			kernel[j] = new float[sizey];
		createGaussianFilter2D(kernel, sizex, sizey, (float)sizex/3.5f, (float)sizey/3.5f, false);

		for (int x = r->x; x < r->x+r->width; x++){
			for (int y = r->y; y < r->y+r->height; y++){
				dest(x,y).b = (unsigned char)(255.0f * kernel[x-r->x][y-r->y]);
				sal(x,y) = (float)255.0f * kernel[x-r->x][y-r->y];
			}
		}

		
        /*pt1.x = r->x;
        pt2.x = (r->x+r->width);
        pt1.y = r->y;
        pt2.y = (r->y+r->height);
        cvRectangle( dest.getIplImage(), pt1, pt2, CV_RGB(255,0,0), CV_FILLED, 8, 0 );
		cvRectangle( sal.getIplImage(), pt1, pt2, cvScalar(255.0f), CV_FILLED, 8, 0 );*/

		// delete kernel
		for (j = 0; j < sizex; j++)
			delete [] kernel[j];
		delete [] kernel;
    }

    // buffering old image size
    _sizeOld.width = src.width();
    _sizeOld.height = src.height();
}

void CvFaceSalience::createGaussianFilter2D(float **filter, int sizex, int sizey, float ex, float ey, bool normalize){

	if (((sizex-1)%2 == 0) && ((sizey-1)%2 == 0)){
		int extx = (sizex-1)/2;
		int exty = (sizey-1)/2;
		if (normalize){
			double sum=0.0;	
            int y;
			for(y=-exty;y<=exty;y++){
				for(int x=-extx;x<=extx;x++){
					sum+=(exp(-0.5*(((double)x/ex)*((double)x/ex)+((double)y/ey)*((double)y/ey))));
				}
			}
			for(y=-exty;y<=exty;y++){
				for(int x=-extx;x<=extx;x++){
					filter[x+extx][y+exty]=(exp(-0.5*(((double)x/ex)*((double)x/ex)+((double)y/ey)*((double)y/ey))))/sum;
					}
			}
		}
		else{
			for(int y=-exty;y<=exty;y++){

				for(int x=-extx;x<=extx;x++){
					filter[x+extx][y+exty]=(exp(-0.5*(((double)x/ex)*((double)x/ex)+((double)y/ey)*((double)y/ey))));
					}
			}
		}
	}
	else
	{
		cout << "AilImageProcessing::createGaussianFilter2D not possible due to invalid size (has to be odd)" << endl;
	}
}


