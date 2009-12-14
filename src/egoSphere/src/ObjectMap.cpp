// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// highgui.h seems to cause conflicts in windows vs8
// also, it does not seem to be required. 
// Commented out. Lorenzo.
//#include <highgui.h>
#include <iCub/ObjectMap.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

template <class T> T portable_min(T x, T y) {
    return (x<y)?x:y;
}

int cnt;
ObjectMap::ObjectMap(){
    _matRotS2W = new double[9];
    _matRotW2S = new double[9];
    _imgMapRgb.resize(0,0);
    cnt = 0;
}

ObjectMap::~ObjectMap(){
    delete [] _matRotS2W;
    delete [] _matRotW2S;
}

bool ObjectMap::open(Searchable& config){

    bool ok = true;

    if(!config.check("name")){
        std::cout << "ObjectMap: Error, module base name not found in configuration. Start the module with the --name option.." << std::endl;
        return false;
    }

	// module base name
    std::string strModuleName = std::string(config.find("name").asString().c_str());

    // look for group EGO_SPHERE_OBJECT_MAP
    Bottle botConfigObject(config.toString().c_str());
    botConfigObject.setMonitor(config.getMonitor());
    if (!config.findGroup("EGO_SPHERE_OBJECT_MAP").isNull()){
        botConfigObject.clear();
        botConfigObject.fromString(config.findGroup("EGO_SPHERE_OBJECT_MAP", "Loading visual map configuration  from group EGO_SPHERE_OBJECT_MAP.").toString());
    }

    _salienceDecayRate = botConfigObject.check("decayObject",
            Value(0.9995),
            "Decay for the object saliency map (double).").asDouble();
    _resXObject = botConfigObject.check("resXObject",
                                            Value(640),
                                            "Width of internal object map (int)").asInt();
    _resYObject = botConfigObject.check("resYObject",
                                            Value(480),
                                            "Height of internal object map (int)").asInt();
    
    //cout<<"_resXObject "<<_resXObject<<" _resYObject "<<_resYObject<<endl;
    
    _imgCart.resize(_resXObject,_resYObject);
    _imgRemapX.resize(_resXObject,_resYObject);
    _imgRemapY.resize(_resXObject,_resYObject);
    _imgSpher.resize(_resXObject,_resYObject);
    _imgMapResA.resize(_resXObject,_resYObject);
    
    _prtObjects.open(std::string(strModuleName + std::string("/mapObject/bottle_in")).c_str());

    return ok;
}

bool ObjectMap::interrupt(){
    _prtObjects.interrupt();
    return true;
}

bool ObjectMap::close(){
    _prtObjects.close();
    return true;
}

bool ObjectMap::reset(){
    if (_imgMapResA.width() > 0 && _imgMapResA.height()  > 0)
        cvSet(((IplImage*)_imgMapResA.getIplImage()), cvScalar(0.0f));
    if (_imgMapResB.width() > 0 && _imgMapResB.height()  > 0)
        cvSet(((IplImage*)_imgMapResB.getIplImage()), cvScalar(0.0f));
    return true;
}

bool ObjectMap::read(){
    _objectPositions = _prtObjects.read(false);
    return true;
}

bool ObjectMap::updateDecay(){
    if (_salienceDecayRate < 1.0){
        cvScale(((IplImage*)_imgMapResA.getIplImage()),((IplImage*)_imgMapResA.getIplImage()),_salienceDecayRate);
    }
    return true;
}

bool ObjectMap::process(double azREye, double elREye, int xREye, int yREye, double *rotREye,
                        double azLEye, double elLEye, int xLEye, int yLEye, double *rotLEye,
                        double azHead, double elHead, int xHead, int yHead, double *rotHead,
                        bool blnSaccadicSuppression)
{
    if (_objectPositions != NULL){ 

    
        VectorOf<double> objs(5);
        objs[1] = 15.; //"variance", width of the blob
        objs[3] = 7.5; //"variance", height of the blob

    //cout << "bottle: "<< ObjectPositions->toString() << endl;
        
        if(!_objectPositions->find("size").isNull()){
            double encoders[6];

            int size = _objectPositions->find("size").asInt();
            char num[10];

            Bottle encoders_b = _objectPositions->findGroup("encoders");
            encoders[0] = encoders_b.get(1).asDouble();
            encoders[1] = encoders_b.get(2).asDouble();
            encoders[2] = encoders_b.get(3).asDouble();
            encoders[3] = encoders_b.get(4).asDouble();
            encoders[4] = encoders_b.get(5).asDouble();
            encoders[5] = encoders_b.get(6).asDouble();

            for(int i=0; i< size; i++){
                sprintf(num,"%d\0",i); 
                Bottle object = _objectPositions->findGroup(num); 
                if(!object.isNull()){
                    yarp::os::ConstString label = object.find("label").asString();

                    double azimuth = object.find("azimuth").asDouble();
                    double elevation = object.find("elevation").asDouble();
                    objs[0] = azimuth;
                    objs[2] = elevation;
                    //cout << "Object "<<label<<" in {"<< azimuth <<","<< elevation <<"}";
                
//                 cvNamedWindow( "getMap before" );
//                 cvShowImage("getMap before", _mapObjects.getMap(_egoImgSize.width, _egoImgSize.height).getIplImage());

                    addObservation(objs, encoders);
//                 if(_simulation){
/*                    cvNamedWindow( "getMap after" );
                    cvShowImage("getMap after", _mapObjects.getMap(_egoImgSize.width, _egoImgSize.height).getIplImage());
                    cvWaitKey(0);*/
//                 }
                }
            }
        //cout << " encoders: "<<encoders[0]<<" "<<encoders[1]<<" "<<encoders[2] <<" "<<encoders[3] <<" "<<encoders[4]<<" "<<encoders[5] <<endl; 
        }
        
        //_objectPositions == NULL; ///I'm not sure if this should be done, or if this HAS to be done, because since _objectPositions is a global var, it won't go to NULL on her own or will she? If she doesn't, then this map will be wasting resources most of the time reprocessing the same bottle.
        //EDIT: nop, it goes to NULL on its own, cool :)
    }
//     else{ ///SIMULATING, DEBUG
//         VectorOf<double> objs(5);
//         objs[1] = 30; //"variance", width of the blob
//         objs[3] = 15; //"variance", height of the blob
//         double encoders[6];
//         encoders[0] = 0.0;//up        
//         encoders[1] = 0.0;//rotates
//         encoders[2] = 0.0;//left..............
//         encoders[3] = 0.0;
//         encoders[4] = 0.0;
//         encoders[5] = 0.;
//         objs[0]=-15.0;  //azimuth 
//         objs[2]=0.0;  //elevation //up, OK!
//         addObservation(objs, encoders);        
//         cnt++;
//         if(cnt >= 180)
//             cnt = -179;
//     }
    
    return true;
}

bool ObjectMap::addObservation(VectorOf<double> &vctSound, double *encoders){

    // i) render distribution to cartesian map
    // ii) project cartesian map to spherical map
    // iii) add temporary spherical map to final spherical map

    if (vctSound.size() < 5){
        cout << "Error ObjectMap::addObservation: Object observation vector size < 5." << endl;
        return false;
    }

    //cout <<"Obj in"<< (float)vctSound(0) <<" , "<< vctSound(2)<< " encoders: "<<encoders[0]<<" "<<encoders[1]<<" "<<encoders[2] <<" "<<encoders[3] <<" "<<encoders[4]<<" "<<encoders[5] <<endl; 
    
    // cartesian image size
    int hCart = _imgCart.height();
    int wCart = _imgCart.width();

    // extract sound localization values
    float azimuth = (float)vctSound(0);
    float elevation = (float)vctSound(2);
    float intensity = (float)255.0f; // vctSound(4);
    float sigmaXDeg = (float)vctSound(1);
    float sigmaYDeg = (float)vctSound(3);
    float sigmaXPix = _resXObject * sigmaXDeg/360.0;
    float sigmaYPix = _resYObject * sigmaYDeg/180.0;
    int sizeXPix = (int)(4.0f*sigmaXPix);
    int sizeYPix = (int)(4.0f*sigmaYPix);
    ((sizeXPix-1)%2==0)?sizeXPix:sizeXPix++; // make odd
    ((sizeYPix-1)%2==0)?sizeYPix:sizeYPix++; // make odd
    
    // i)
    // render sound distribution at location (0,0) to imgCart
    cvSet(_imgCart.getIplImage(), cvScalar(0.0f)); // zero
    //PixelFloat pix = 255.0f;
    //yarp::sig::draw::addRectangle(_imgCart, pix, wCart/2, hCart/2, wCart, hCart);
    //yarp::sig::draw::addCircle(_imgCart, pix, _imgCart.width()/2, _imgCart.height()/2, 10);
    renderGaussianDistribution(_imgCart,
                               _imgCart.width()/2, _imgCart.height()/2, 
                               sizeXPix, sizeYPix, 
                               sigmaXPix, sigmaYPix, 
                               intensity);
   
    //encoders[0] = 0.0;
    //encoders[1] = 0.0;
    //encoders[2] = 0.0;
    //encoders[3] = 0.0;
    //encoders[4] = 0.0;
    //encoders[5] = 0.0;
    /// calculate rotation matrix world2camera-centered-coordinates
    _robMatRot = _headKin.fkine(encoders, 3) * _headKin.getRotationFromAzimuthElev2(azimuth, elevation); ///in good conscience we should use the foward kinematics of the eye (left i believe) instead of the 3rd join which is the neck, but by doing "fkine(encoders, 'l')" or "fkine(encoders, 'r')", the resulting behaviour is just a massive fail
    
//     _robMatRot = _headKin.getRotationFromAzimuthElev2(azimuth, elevation);
//     cout << "getRotationFromAz Elevation: " << endl;
//     _robMatRot.print();
//         
//     _robMatRot = _headKin.fkine(encoders, 'l');
//     cout << "fkin 'l' eye: " << endl;
//     _robMatRot.print();
    
    //_robMatRot = _headKin.fkine(encoders, 'l') * _headKin.getRotationFromAzimuthElev2(azimuth, elevation); //AFTER
//     cout << "Mat All after fkin 'l' eye: " << endl;
//     _robMatRot.print();    
    
    EgoSphereModule::convertRobMatrix(_robMatRot, _matRotS2W);
//     cout << "_matRotS2W: " << endl;
//     cout << _matRotS2W[0] << " " << _matRotS2W[1] << " " << _matRotS2W[2] << endl;
//     cout << _matRotS2W[3] << " " << _matRotS2W[4] << " " << _matRotS2W[5] << endl;
//     cout << _matRotS2W[6] << " " << _matRotS2W[7] << " " << _matRotS2W[8] << endl;
    EgoSphereModule::transpose(_matRotS2W, _matRotW2S);
//     cout << "_matRotW2S: " << endl;
//     cout << _matRotW2S[0] << " " << _matRotW2S[1] << " " << _matRotW2S[2] << endl;
//     cout << _matRotW2S[3] << " " << _matRotW2S[4] << " " << _matRotW2S[5] << endl;
//     cout << _matRotW2S[6] << " " << _matRotW2S[7] << " " << _matRotW2S[8] << endl;

//     double *gazeLEye = new double[3]; // 1x3
//     EgoSphereModule::calcGazeVector(_matRotW2S, gazeLEye); // matrix to gaze vector
//     double azLEye, elLEye; // 1x1
//     EgoSphereModule::calcAngles(gazeLEye, azLEye, elLEye); // gaze vector to azimuth/elevation
//     
//     cout<<endl;
//     cout<<"azLEye "<<azLEye<<", elLEye "<<elLEye<<endl;
//     delete[] gazeLEye;
    
    // ii)
    // compute remap maps
    double focalLength = 0.92*(double)min(_resYObject,_resXObject); // corresponds ~ what dragonfly's have
    //cout << "x: " << _resXObject << "y: " << _resYObject << " focal: " << focalLength << " cx: " << (double)wCart/2.0 << " cy: " << (double)hCart/2.0 << endl;
    compute_egosp_map( _resYObject, _resXObject,
                       _resYObject, _resXObject,
                       focalLength, focalLength, // fx, fy
                       (double)_resXObject/2.0,  (double)_resYObject/2.0,
                       _matRotS2W, //world to sound rotation    
                       (float*)((IplImage*)_imgRemapX.getIplImage())->imageData, 
                       (float*)((IplImage*)_imgRemapY.getIplImage())->imageData);
    // remap cartesian map to spherical map
    cvRemap(_imgCart.getIplImage(), _imgSpher.getIplImage(),
            _imgRemapX.getIplImage(), _imgRemapY.getIplImage(), 
            CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
            cvScalarAll(0)); // remap and fill outliers with 0's

//     cvNamedWindow( "_imgCart" );
//     cvShowImage("_imgCart", _imgCart.getIplImage());
//     cvWaitKey();
    
    // iii)
    // add new temporary map to final spherical map
    // spherFinal(x,y) = max(spherTemp(x,y),spherFinal(x,y))
    cvMax(_imgSpher.getIplImage(), _imgMapResA.getIplImage(), _imgMapResA.getIplImage());
    
    return true;
}


bool ObjectMap::write(bool saccadicSuppression){
	// object map does not write additional output
    return true;
}

yarp::sig::ImageOf<yarp::sig::PixelFloat>& ObjectMap::getSalienceMap(int width, int height){
    _imgMapResB.resize(width, height);
    cvResize(_imgMapResA.getIplImage(), _imgMapResB.getIplImage(), CV_INTER_LINEAR );
    return _imgMapResB;
}

yarp::sig::ImageOf<yarp::sig::PixelRgb>& ObjectMap::getVisualizationMap(int width, int height){
	// not implemented
    return _imgMapRgb;
}

bool ObjectMap::setWeightObject(float w){
    _weightObject = w;
    return true;
}

float ObjectMap::getWeightObject(){
    return _weightObject;
}


bool ObjectMap::setSalienceDecay(double rate){
    _salienceDecayRate = rate;
    return true;
}

double ObjectMap::getSalienceDecay(){
    return _salienceDecayRate;
}


void ObjectMap::renderGaussianDistribution(ImageOf<PixelFloat> &img,
                                           int posX, int posY, 
                                           int sizex, int sizey, 
                                           float sigmaX, float sigmaY, 
                                           float height)
{
    IplImage *ipl = (IplImage*)img.getIplImage();   
    //cout << "going to render.." << posX << " " << posY << " " << sizex << " " << sizey << " " << sigmaX << " " << sigmaY << " " << height << endl;
    if (((sizex-1)%2 == 0) && ((sizey-1)%2 == 0)){
        int extx = (sizex-1)/2;
        int exty = (sizey-1)/2;
        int currX, currY;
        for(int y=-exty;y<=exty;y++){
            for(int x=-extx;x<=extx;x++){
                currX = x+posX;
                currY = y+posY;
                // if inside object map
                //cout << "rendering at: " << posX << " " << posY << endl;
                if (posX >= 0 && posX < img.width() &&
                    posY >= 0 && posY < img.height()){
                        //cout << "rendering at: " << posX << " " << posY << " value: " << height*(float)(exp(-0.5*(((float)x/sigmaX)*((float)x/sigmaX)+((float)y/sigmaY)*((float)y/sigmaY)))) << endl;
                    ((float*)(ipl->imageData + ipl->widthStep*(currY)))[currX] = height*(float)(exp(-0.5*(((float)x/sigmaX)*((float)x/sigmaX)+((float)y/sigmaY)*((float)y/sigmaY))));
                    }
            }
        }
    }
    else
    {
        cout << "ObjectMap::renderGaussianDistribution not possible due to invalid size (has to be odd)" << endl;
    }
}
