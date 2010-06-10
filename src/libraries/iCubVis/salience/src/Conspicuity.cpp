// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/vis/Conspicuity.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::vis;

Conspicuity::Conspicuity(){
    
    MAX_SALIENCE = 255.0f;

    _imgPyr = NULL;
    _imgConsp = NULL;
    _imgPyrTmp = NULL;
    _fltConspFactors = NULL;
    //_fltGlobMax = NULL;
    _imgDstTmp = NULL;

    _sizeOld.width = -1;
    _sizeOld.height = -1;
    _needInit = true;
}

Conspicuity::~Conspicuity(){
    close();
}

bool Conspicuity::close(){
  
    //_prtRgb.close();

    releaseImages();
    return true;
}

bool Conspicuity::open(yarp::os::Searchable& config){

     _sizeConsp = config.check("numCenterSurroundScales",
                            Value(3),
                            "Number of center surround scale levels (gaussian pyramide size = numScales + 2) (int).").asInt();
    if (_sizeConsp < 1)
        _sizeConsp = 1;
    _sizePyr = _sizeConsp + 2;


    //_prtRgb.open("/rgb");

    /* filterName = config.check( "filterName",
                            yarp::os::Value("emd"),
                            "Name of this instance of the emd filter (string).").asString(); */
    return true;
}


bool Conspicuity::initImages(int w, int h){
    releaseImages();

    // create images here
    bool ok = true;
    int width = 0, height = 0;
    _imgPyr = new IplImage*[_sizePyr];
    _imgPyrTmp = new IplImage*[_sizePyr];
    for (int i = 0; i < _sizePyr; i++){
        if (width%2 != 0 || height%2 != 0)
            ok = false;
        width = (int)( (float)w/(float)((pow((float)2,i+1))) );
        height = (int)( (float)h/(float)((pow((float)2,i+1))) );
        cout << "Pyramid " << i << ": " << width << " " << height << endl;
        _imgPyr[i] = cvCreateImage(cvSize(width, height),IPL_DEPTH_32F, 1);
        _imgPyrTmp[i] = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    }
    width = 0; height = 0;
    _imgConsp = new IplImage*[_sizeConsp];
    for (int j = 0; j < _sizeConsp; j++){
        if (width%2 != 0 || height%2 != 0)
            ok = false;
        width = (int)(0.5f + (float)w/(float)((pow((float)2,j+1))));
        height = (int)(0.5f + (float)h/(float)((pow((float)2,j+1))));
        _imgConsp[j] = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
        cout << "Conspicuity " << j << ": " << width << " " << height << endl;
    }

    _imgDstTmp = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);

    // create _fltConspFactors array
    _fltConspFactors = new float[_sizeConsp];
    //_fltGlobMax = new float[_sizeConsp];
    _needInit = false;

    if (!ok)
        cout << "iCub::Conspicuity(): Error, at least one image of the gaussian pyramid is of odd size." << endl;
    return ok;
}

void Conspicuity::releaseImages(){

    if (_imgPyr != NULL){
        for (int i = 0; i < _sizePyr; i++)
            cvReleaseImage(&_imgPyr[i]);
        delete [] _imgPyr;
        _imgPyr = NULL;
    }
    if (_imgConsp != NULL){
        for (int j = 0; j < _sizeConsp; j++)
            cvReleaseImage(&_imgConsp[j]);
        delete [] _imgConsp;
        _imgConsp = NULL;
    }
    if (_imgPyrTmp != NULL){
        for (int k = 0; k < _sizePyr; k++)
            cvReleaseImage(&_imgPyrTmp[k]);
        delete [] _imgPyrTmp;
        _imgPyrTmp = NULL;
    }
    if (_imgDstTmp != NULL){
        cvReleaseImage(&_imgDstTmp);
        _imgDstTmp = NULL;
    }
    if (_fltConspFactors != NULL)
        delete [] _fltConspFactors;
    _fltConspFactors = NULL;
}


void Conspicuity::apply(IplImage *src, IplImage *dst){

    // check input images
    if (src->width != dst->width || src->height != dst->height){
        cout << "Conspicuity::apply(): Error, src and dest have different size." << endl;
        return;
    }
        
    // init if necessary
    if (src->width != _sizeOld.width || src->height != _sizeOld.height || _needInit){
        if(!initImages(src->width, src->height))
            return;
    }

    // scale input image to 0.0 .... MAX_SALIENCE (good thing to do??)
    calcGlobalMaximum(src, _globalMax);
    cvScale(src, src, MAX_SALIENCE / _globalMax ); 

    // calculate gaussian pyramid
    cvPyrDown(src, _imgPyr[0]);
    for (int i = 0; i < (_sizePyr-1); i++)
        cvPyrDown(_imgPyr[i], _imgPyr[i+1]);

    // calculate center surround (conspicuity pyramid)
    for (int j = 0; j < _sizeConsp; j++){
        cvPyrUp(_imgPyr[j+2], _imgPyrTmp[j+1]); // raise j+2 to j level -> tmp pyramide
        cvPyrUp(_imgPyrTmp[j+1], _imgPyrTmp[j]);
        cvSub(_imgPyr[j], _imgPyrTmp[j], _imgConsp[j]); // calculate center surround (mexican hat) convolution
        cvAbs(_imgConsp[j], _imgConsp[j]); // calc absolute to get rid of negative peaks
        calcGlobalMaximum(_imgConsp[j], _globalMax);
        if (_globalMax > 0.0f)
            cvScale(_imgConsp[j], _imgConsp[j], MAX_SALIENCE / _globalMax ); // normalize each conspicuity map
        calcConspicuityFactor(_imgConsp[j], _fltConspFactors[j]); // calc conspicuity factor for this scale 
    }
    normalizeArray(_fltConspFactors, _sizeConsp);

    // sum conspicuity maps according to conspicuity factors
    cvScale(_imgConsp[0], _imgConsp[0], _fltConspFactors[0]);
    cvPyrUp(_imgConsp[0], dst);
    for (int k = 1; k < _sizeConsp; k++){
        for (int m = k; m > 0; m--)
            cvPyrUp(_imgConsp[m], _imgConsp[m-1]);
        cvPyrUp(_imgConsp[0], _imgDstTmp);
        cvScaleAdd(_imgDstTmp, cvScalar(_fltConspFactors[k]), dst, dst);
    }

    // debugging stuff

    /*cout << "Conspicuity map info: " << endl;
    for (int i = 0; i < _sizeConsp; i++)
        cout << "Map: " << i << " C" << pow((float)2,i+1) << "S" << pow((float)2,i+3) << " weight: " << _fltConspFactors[i] << " " << endl;
    cout << endl;*/
   
    // write test image
    /*ImageOf<PixelRgb> &rgb = _prtRgb.prepare();
    int index = 2;
    rgb.resize(_imgConsp[index]->width, _imgConsp[index]->height);
    float2Rgb(_imgConsp[index], (IplImage*)rgb.getIplImage());
    _prtRgb.write();*/

    /*cout << "Image Pyramides: " << endl;
    for (int i = 0; i < _sizePyr; i++){
        cout << _imgPyr[i]->width << " " << _imgPyr[i]->height << endl;
    }
    cout << "Image Conspicuity: " << endl;
    for (int j = 0; j < _sizeConsp; j++){
        cout << _imgConsp[j]->width << " " << _imgConsp[j]->height << endl;
    }*/

    _sizeOld.width = src->width;
    _sizeOld.height = src->height;
}


void Conspicuity::calcConspicuityFactor(IplImage *img, float &factor){

    // calculate pow((M-mAvg),2) according to Itti & Koch (Rapid Scene Analysis)
    float *data = (float*)img->imageData;
    //float max = 0.0f;
    float sum = 0.0f;
    int posRow, pos, posPlus, posMinus, count = 0;
    for (int y = 1; y < (img->height-1); y++){
        posRow = img->width * y;
        for (int x = 1; x < (img->width-1); x++){
            pos = posRow + x;
            posMinus = pos - img->width; // row before
            posPlus = pos + img->width; // next row
            // is (x,y) a local maxima?
            // Flat regions are discarded (peak needed)
            if ( data[pos] > data[pos-1] &&
                 data[pos] > data[pos+1] &&
                 data[pos] > data[posMinus] &&
                 data[pos] > data[posPlus] &&
                 data[pos] > data[posMinus-1] &&
                 data[pos] > data[posMinus+1] &&
                 data[pos] > data[posPlus-1] &&
                 data[pos] > data[posPlus+1]){
                sum += data[pos];
                //if (data[pos] > max)
                //    max = data[pos];
                count++;
            }
            /*if ( data[pos] == data[pos-1] &&
                 data[pos] == data[pos+1] &&
                 data[pos] == data[posMinus] &&
                 data[pos] == data[posPlus] &&
                 data[pos] == data[posMinus-1] &&
                 data[pos] == data[posMinus+1] &&
                 data[pos] == data[posPlus-1] &&
                 data[pos] == data[posPlus+1]){
                     if (data[pos] != 0.0f)
                        cout << "Plateau!!! at: " << data[pos] << endl;
            }*/
        }
    }
    //cout << "Global Maximum 2: " << max << endl;
    //cout << endl;
    //cout << "Factor: num local max's: " << count << endl;
    if (count == 1){
        factor = 1.0f;
        //cout << "1 global maximum!" << endl;
    }
    else if (count > 1){
        //sum -= max;
        sum -= MAX_SALIENCE; // subtract global maximum
        count--; 
        factor = pow( (MAX_SALIENCE - (sum/(float)count))/MAX_SALIENCE ,2);
        //cout <<  " average of non global max's: " << factor <<  endl;
    }
    else
        factor = 0.0f;
}

void Conspicuity::normalizeArray(float *arr, int size){
    float sum = 0.0f;
    for (int i = 0; i < size; i++)
        sum += arr[i];
    if (sum > 0.0f)
        for (int j = 0; j < size; j++)
            arr[j] /= sum;
}

