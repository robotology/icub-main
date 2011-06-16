// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/vis/DirectionalSalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::vis;

DirectionalSalience::DirectionalSalience(){
    MAX_SALIENCE = 255.0f;
    _dirs = NULL;
    _sigs = NULL;
    _wavs = NULL;
    _imgInRgbResized = NULL;
    _imgInGray = NULL;
    _imgInFloat = NULL;
    _imgGReal = NULL;
    _imgGImag = NULL;
    _imgGabor = NULL;
    _imgDCen = NULL;
    _imgDSur = NULL;
    _imgD = NULL;
    _imgResult = NULL;
    _conspD = NULL;
    _gabor = NULL;
    _gaussSur = NULL;
    _gaussCen = NULL;
}

DirectionalSalience::~DirectionalSalience(){
    close();
}

bool DirectionalSalience::open(yarp::os::Searchable& config){
    bool ok = Salience::open(config);
    if (!ok)
        return false;

    _scale = config.check("scale",
                          Value(1.0),
                          "Calculate directional salience at given scale of the input image (double [0...1]).").asDouble();
    _sig2wav = config.check("sig2wav",
                          Value(4.0),
                          " gabor_wavelength = sig2wav * gabor_sigma (double).").asDouble();
    _debug = config.check("debug",
                          Value(0),
                          "Activate debugging capabilities (int [0|1]).").asInt()!=0;
    _verbose = config.check("verbose",
                            Value(0),
                           "Activate debugging output (int [0|1]).").asInt()!=0;
    _dbgImgArray = config.check("dbgImgArray",
                          Value("gab"),
                          "Write from this image array to the debugging port (string).").asString(); 
    _dbgDirIndex = config.check("dbgDirIndex",
                          Value(0),
                          "Select this direction from image array to debug (int).").asInt();
    _dbgSizIndex = config.check("dbgSizIndex",
                          Value(0),
                          "Select this filter size from image array to debug (int).").asInt();
    _centerSize = config.check("centerSize",
                          Value(2.0),
                          "sigma of the according size multiplied by this factor is used for gaussian center filter (double).").asDouble();
    _surroundSize = config.check("surroundSize",
                          Value(8.0),
                          "sigma of the according size multiplied by this factor is used for gaussian surround filter.").asDouble();

    // filters for 4 directions
    _numDirs = 4;
    _dirs = new double[_numDirs];
    _dirs[0] = 0.0;
    _dirs[1] = 45.0;
    _dirs[2] = 90.0;
    _dirs[3] = 135.0;
    // 3 scales for each direction
    _numSizs = 3;
    _sigs = new double[_numSizs];
    _sigs[0] = 2.0;
    _sigs[1] = 4.0;
    _sigs[2] = 8.0;
    /*_sigs[0] = 1.0;
    _sigs[1] = 2.0;
    _sigs[2] = 4.0;*/
    _wavs = new double[_numSizs];
    for (int i = 0; i < _numSizs; i++)
        _wavs[i] = _sigs[i] * _sig2wav;
    // gabor filters
    _gabor = new FastGabor**[_numSizs];
    for (int s = 0; s < _numSizs; s++)
        _gabor[s] = new FastGabor*[_numDirs];
    for (int s = 0; s < _numSizs; s++)
        for (int d = 0; d < _numDirs; d++)
            _gabor[s][d] = new FastGabor();
    // gauss filters
    _gaussCen = new FastGauss*[_numSizs];
    _gaussSur = new FastGauss*[_numSizs];
    for (int s = 0; s < _numSizs; s++){
        _gaussCen[s] = new FastGauss();
        _gaussSur[s] = new FastGauss();
    }
    // conspicuity factors
    _conspD = new float*[_numSizs];
    for (int s = 0; s < _numSizs; s++){
        _conspD[s] = new float[_numDirs];
    }
    if (_debug)
        _prtDebug.open("/debug/salience/directional/o:img");
    yarp::os::Time::turboBoost();
    return true;
}


void DirectionalSalience::applyImpl(ImageOf<PixelRgb>& src, 
                          ImageOf<PixelRgb>& dest,
                          ImageOf<PixelFloat>& sal) {
    float max = 0.0f;
    int maxX, maxY;

    // time measurement                              
    double tStart = yarp::os::Time::now();
    // block other threads (e.g. configuration)
    _mutex.wait();
    // resizing/init
    sal.resize(src);
    if (_w != src.width() || _h != src.height()){
        _w = src.width();
        _h = src.height();
        _ws = (int)(_w * _scale + 0.5);
        _hs = (int)(_h * _scale + 0.5);
        init(_ws, _hs);
    }
    // scaling if _scale != 1.0
    cvResize((IplImage*)src.getIplImage(), _imgInRgbResized, CV_INTER_LINEAR );
    // convert input to grayscale and float image
    cvCvtColor( _imgInRgbResized, _imgInGray, CV_BGR2GRAY );
    cvConvert(_imgInGray, _imgInFloat);

    // compute 12 gabor filtered maps ( _numSizs*_numDirs)
    // + the corresponding conspicuity maps
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
                // compute gabor maps
                _gabor[s][d]->GaborFiltZeroMean((float*)_imgInFloat->imageData, 
                    (float*)_imgGReal->imageData, 
                    (float*)_imgGImag->imageData);
                // combine real and imaginary parts
                _gabor[s][d]->ComputeMagnitude((float*)_imgGReal->imageData, 
                    (float*)_imgGImag->imageData, 
                    (float*)_imgGabor[s][d]->imageData);
                // center- & surround-maps
                _gaussCen[s]->GaussFilt((float*)_imgGabor[s][d]->imageData, (float*)_imgDCen[s][d]->imageData);
                _gaussSur[s]->GaussFilt((float*)_imgGabor[s][d]->imageData, (float*)_imgDSur[s][d]->imageData);
                // center-surround
                cvSub(_imgDCen[s][d], _imgDSur[s][d], _imgD[s][d]);
                // absolute (mirror negative peaks)
                cvAbs(_imgD[s][d], _imgD[s][d]); 
                // calc peak value
                calcGlobalMaximum(_imgD[s][d], max, maxX, maxY);
                // normalize
                if (max > 0.0f)
                    cvScale(_imgD[s][d], _imgD[s][d], MAX_SALIENCE / max );
                // calc conspicuity
                calcConspicuityFactor(_imgD[s][d], _conspD[s][d]);
        }
    }
   
    // normalize conspicuity factors
    if (_verbose)
        cout << endl << "Conspicuity Factors: " << endl;
    float sum = 0.0f;
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            sum += _conspD[s][d];
        }
    }
    if (sum > 0.0f){
        for (int s = 0; s < _numSizs; s++){
            for (int d = 0; d < _numDirs; d++){
                _conspD[s][d] /= sum;
                if (_verbose)
                    cout << "factor: " << _conspD[s][d] << " sigma: " << _sigs[s] << " direction: " << _dirs[d] << endl;
            }
        }
    }

    // summing up all maps (normalized)
    cvZero(_imgResult);
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            cvScaleAdd(_imgD[s][d], cvScalar(_conspD[s][d]), _imgResult, _imgResult);
        }
    }

    // resize/copy back to output image
    cvResize(_imgResult, (IplImage*)sal.getIplImage());
   
    // output debug/analysis of requested map [name, size, direction]
    if (_debug){
        IplImage ***ocvImgArrayDebug = _mapImgTable[_dbgImgArray];
        IplImage *ocvImgDebug = ocvImgArrayDebug[_dbgSizIndex][_dbgDirIndex];
        ImageOf<PixelFloat> &yrpImgDebug = _prtDebug.prepare();
        yrpImgDebug.resize(ocvImgDebug->width, ocvImgDebug->height);
        cvCopy(ocvImgDebug, (IplImage*)yrpImgDebug.getIplImage());
        _prtDebug.write();
    }
    
    _mutex.post();
    double tEnd = yarp::os::Time::now();
    if (_verbose){
        cout << endl;
        cout << "Update time: " << tEnd - tStart << " Processed image size: " << _ws << " x " << _hs << " original size: " << _w << " x " << _h << endl;
        /*cout << endl;
        cout << "Conspicuity factors: " << endl;
        for (int s = 0; s < _numSizs; s++){
            for (int d = 0; d < _numDirs; d++){
                cout << "consp size " << s << " direction " << d << ": " << _conspD[s][d] << endl;
            }
        }*/
    }
}

void DirectionalSalience::init(int w, int h){
    release();
    // images
    _imgInRgbResized = cvCreateImage(cvSize(w,h),IPL_DEPTH_8U, 3);
    _imgInGray = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    _imgInFloat = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
    _imgGReal = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgGImag = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgResult = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgGabor = new IplImage**[_numSizs];
    _imgDCen = new IplImage**[_numSizs];
    _imgDSur = new IplImage**[_numSizs];
    _imgD = new IplImage**[_numSizs];
    for (int i = 0; i < _numSizs; i++){
        _imgGabor[i] = new IplImage*[_numDirs];
        _imgDCen[i] = new IplImage*[_numDirs];
        _imgDSur[i] = new IplImage*[_numDirs];
        _imgD[i] = new IplImage*[_numDirs];
    }
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            _imgGabor[s][d] = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
            _imgDCen[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgDSur[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgD[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
        }
    }
    // filters
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            _gabor[s][d]->AllocateResources(h,w,_sigs[s],_dirs[d],_sig2wav * _sigs[s]);
        }
    }
    for (int s = 0; s < _numSizs; s++){
		_gaussCen[s]->AllocateResources(h,w, _centerSize*_sigs[s]);
        _gaussSur[s]->AllocateResources(h,w, _surroundSize*_sigs[s]);
    }
    // image table
    _mapImgTable["gab"] = _imgGabor;
    _mapImgTable["cen"] = _imgDCen;
    _mapImgTable["sur"] = _imgDSur;
    _mapImgTable["fin"] = _imgD;
}
 
void DirectionalSalience::release(){
    // images
    if (_imgInRgbResized != NULL)
        cvReleaseImage(&_imgInRgbResized);
    _imgInRgbResized = NULL;
    if (_imgInGray != NULL)
        cvReleaseImage(&_imgInGray);
    _imgInGray = NULL;
    if (_imgInFloat != NULL)
        cvReleaseImage(&_imgInFloat);
    _imgInFloat = NULL;
    if (_imgGReal != NULL)
        cvReleaseImage(&_imgGReal);
    _imgGReal = NULL;
    if (_imgGImag != NULL)
        cvReleaseImage(&_imgGImag);
    _imgGImag = NULL;
    if (_imgResult != NULL)
        cvReleaseImage(&_imgResult);
    _imgResult = NULL;
    if (_imgGabor != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgGabor[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgGabor[s];
        delete [] _imgGabor;
        _imgGabor = NULL;
    }
    if (_imgDCen != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDCen[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDCen[s];
        delete [] _imgDCen;
        _imgDCen = NULL;
    }
    if (_imgDSur != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDSur[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDSur[s];
        delete [] _imgDSur;
        _imgDSur = NULL;
    }
    if (_imgD != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgD[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgD[s];
        delete [] _imgD;
        _imgD = NULL;
    }
    // filters
    if (_gabor != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                _gabor[s][d]->FreeResources();
    }
    if (_gaussCen != NULL){
        for (int s = 0; s < _numSizs; s++)
            _gaussCen[s]->FreeResources();
    }
    if (_gaussSur != NULL){
        for (int s = 0; s < _numSizs; s++)
            _gaussSur[s]->FreeResources();
    }
}

bool DirectionalSalience::close(){
    if (_debug)
        _prtDebug.close();
    release();
    if (_dirs != NULL)
        delete [] _dirs;
    _dirs = NULL;
    if (_sigs != NULL)
        delete [] _sigs;
    _sigs = NULL;
    if (_wavs != NULL)
        delete [] _wavs;
    _wavs = NULL;
    // gabor filters
    for (int s = 0; s < _numSizs; s++)
        for (int d = 0; d < _numDirs; d++)
            delete _gabor[s][d];
    for (int s = 0; s < _numSizs; s++)
        delete [] _gabor[s];
    delete [] _gabor;
    // gauss filters
    for (int s = 0; s < _numSizs; s++){
        delete _gaussCen[s]; 
        delete _gaussSur[s];
    }
    delete [] _gaussCen;
    delete [] _gaussSur;
    // conspicuity factors
    for (int s = 0; s < _numSizs; s++){
        delete [] _conspD[s];
    }
    delete [] _conspD;
    return true;
}

bool DirectionalSalience::respond(const Bottle &command,Bottle &reply){
    bool ok = false; // command executed successfully
    bool rec = false; // command recognized

    switch (command.get(0).asVocab()) {
    case SALIENCE_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX:{
                int index = command.get(2).asInt();
                ok = this->setDebugFilterScaleIndex(index);
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX:{
                int index = command.get(2).asInt();
                ok = this->setDebugFilterDirectionIndex(index);
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME:{
                string name(command.get(2).asString().c_str());
                ok = this->setDebugImageArrayName(name);
            }
                break;
            default:
                cout << "DirectionalSalience::respond(): received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;
    case SALIENCE_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(SALIENCE_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case SALIENCE_VOCAB_DIRECTIONAL_NUM_DIRECTIONS:{
                int num = this->getNumberDirections();
                reply.addInt(num);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_NUM_SCALES:{
                int num = this->getNumberScales();
                reply.addInt(num);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES:{
                Bottle &bot = reply.addList();
                bot.fromString(this->getDebugImageArrayNames().toString());
                ok = true;
            }
                break;
		    default:
                cout << "DirectionalSalience::respond(): received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    return ok;
}

void DirectionalSalience::calcGlobalMaximum(const IplImage *img, float &max, int &maxX, int &maxY) {
    float* data = (float*)img->imageData;
    int posRow, pos;
    max = 0.0f;
    for (int y = 0; y < img->height; y++){
        posRow = img->width * y;
        for (int x = 0; x < img->width; x++){
            pos = posRow + x;
            if (data[pos] > max){
                max = data[pos];
                maxX = x;
                maxY = y;
            }
        }
    }
}

void DirectionalSalience::calcGlobalMinimum(const IplImage *img, float &min) {
    float* data = (float*)img->imageData;
    int posRow, pos;
    min = 99999.9f; // todo better use -> float.h
    for (int y = 0; y < img->height; y++){
        posRow = img->width * y;
        for (int x = 0; x < img->width; x++){
            pos = posRow + x;
            if (data[pos] < min){
                min = data[pos];
            }
        }
    }
}

void DirectionalSalience::calcAdvancedConspicuityFactor(const IplImage *img, const double &sigmaSurround, const int &globMaxX, const int &globMaxY, float &factor){

    double sigma = 4*sigmaSurround;
    // calculate pow((M-mAvg),2) according to Itti & Koch (Rapid Scene Analysis)
    float *data = (float*)img->imageData;
    float sum = 0.0f;
    float dist;
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
                 data[pos] > data[posPlus+1] &&
                 data[pos] > 0.1f){
                dist = pow((float)(globMaxX-x),2) + pow((float)(globMaxY-y),2);
                if (dist < (float)pow(sigma*3.5,2)){
                    sum += (float)data[pos] * (float)exp(-0.5 * dist / pow(sigma,2));
                    count++;
				}
                else{
                    //sum += 0.0f;
                }
            }
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
        sum -= MAX_SALIENCE; // subtract global maximum
        count--; 
        factor = pow( (MAX_SALIENCE - (sum/(float)count))/MAX_SALIENCE ,2);
        //cout <<  " average of non global max's: " << factor <<  endl;
    }
    else
        factor = 0.0f;
}

void DirectionalSalience::calcConspicuityFactor(const IplImage *img, float &factor){

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
                count++;
            }
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
        sum -= MAX_SALIENCE; // subtract global maximum
        count--; 
        factor = pow( (MAX_SALIENCE - (sum/(float)count))/MAX_SALIENCE ,2);
        //cout <<  " average of non global max's: " << factor <<  endl;
    }
    else
        factor = 0.0f;
}

int DirectionalSalience::getNumberScales(){
    return _numSizs;
}

int DirectionalSalience::getNumberDirections(){
    return _numDirs;
}

bool DirectionalSalience::setDebugFilterScaleIndex(int size){
    bool ok = true;
    _mutex.wait(); 
    if (size < _numSizs){
        _dbgSizIndex = size; 
    }
    else{
        ok = false;
    }
    _mutex.post(); 
    return ok;
}

bool DirectionalSalience::setDebugFilterDirectionIndex(int direction){
    bool ok = true;
    _mutex.wait(); 
    if (direction < _numDirs){
        _dbgDirIndex = direction; 
    }
    else{
        ok = false;
    }
    _mutex.post(); 
    return ok;
}

Bottle DirectionalSalience::getDebugImageArrayNames(){
    Bottle bottle;
    for(std::map<string, IplImage***>::const_iterator i = _mapImgTable.begin(); i != _mapImgTable.end(); i++){
        bottle.add(Value(i->first.c_str()));
    }
    return bottle;
}

bool DirectionalSalience::setDebugImageArrayName(string imgArray){
    bool ok = true;
    _mutex.wait();
    //if (_mapImgTable.find(imgArray) != NULL){
        _dbgImgArray = imgArray;
    //}
    //else{
    //    ok = false;
    //}
    _mutex.post();
    return ok;
}

/*
// normalize conspicuity factors
    if (_verbose)
        cout << endl << "Conspicuity Factors: " << endl;
    float sum = 0.0f;
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            sum += _conspD[s][d];
            sum += _conspDMin[s][d];
        }
    }
    if (sum > 0.0f){
        for (int s = 0; s < _numSizs; s++){
            for (int d = 0; d < _numDirs; d++){
                _conspD[s][d] /= sum;
                _conspDMin[s][d] /= sum;
                if (_verbose)
                    cout << "plus: " << _conspD[s][d] << " minus: " << _conspDMin[s][d] << " sigma: " << _sigs[s] << " direction: " << _dirs[d] << endl;
            }
        }
    }
    */
