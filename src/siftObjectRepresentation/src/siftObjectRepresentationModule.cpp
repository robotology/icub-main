// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2008 Dario Figueira
 * 
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#include "siftObjectRepresentationModule.h"
#include <ace/Time_Value.h>
#include <ace/Date_Time.h>
#include <iostream>
#include <fstream>
#include <sstream>

const double rabo[]= {1, 2};

siftObjectRepresentationModule::siftObjectRepresentationModule() {
    //Bottle object("(label \"123\n\")");
    //yarp::os::ConstString label = object.find("label").asString();
    //int objectNumber = atoi(label.c_str());
    //cout << "objectNumber " << objectNumber << " label "<< label.c_str() << "object "<< object.toString().c_str() <<endl;
}

// open function for the YARP module
// is called when the module is created, initial parameters can be set in an ini file
// (and will be parsed here)
bool siftObjectRepresentationModule::open(Searchable &config) {
    // check if called with help option
    if(config.check("help", "if present, display usage message")) {
        printf("No help available \n");
        return false;
    }

    Value *image;
    if(config.check("test", image, "if present, load an image and compare it to the database.")){
        cout<<image->asString().c_str()<<endl;

        Value *image2;
        if(config.check("test2", image2, "if present, load a second image and compare it to the first.")){
            SIFT_Image i(image->asString().c_str());
            SIFT_Image i2(image2->asString().c_str());
            _recognizer.insert_database_image( i );
            _recognizer.match_to_database(i2);
            _recognizer.display_matching();
            return false;
        }


        _recognizer.load_database();
        SIFT_Image i(image->asString().c_str());
        _recognizer.match_to_database(i);
        _recognizer.display_matching();
        return false;
    }
    
    Bottle botConfig(config.toString().c_str());
    botConfig.setMonitor(config.getMonitor());
    //look for the SIFT_OBJECT_REPRESENTATION group from where to extract parameters
    if (!config.findGroup("SIFT_OBJECT_REPRESENTATION").isNull()){
        botConfig.clear();
        botConfig.fromString(config.findGroup("SIFT_OBJECT_REPRESENTATION", "Loading parameters from group SIFT_OBJECT_REPRESENTATION.").toString());
    }
    else{
        cout << "Can't find SIFT_OBJECT_REPRESENTATION group to read the parameters. Trying to load from raw." <<endl;
    }
    _storeNewObjects = (bool)botConfig.check("storeNewObjects",Value(1),"If it should look at both eyes and try to identify and store new close-by objects (int [0|1]).").asInt();
    _orderSaccades = (bool)botConfig.check("orderSaccades",Value(0),"If we can order the robot to look where we want him to (int [0|1]).").asInt();
    _simulation = (bool)botConfig.check("simulation",Value(0),"Simulate presence of robot (int [0|1]).").asInt();
    _saveOriginalLeftImages = botConfig.check("saveOriginalLeftImages",0,"Whether to save or not the images seen by the left eye.").asInt();
    _saveOriginalRightImages = botConfig.check("saveOriginalRightImages",0,"Whether to save or not the images seen by the right eye.").asInt();
    _saveDrawnUponImages = botConfig.check("saveDrawnUponImages",0,"Whether to save or not the images  with the drawn detections on them.").asInt();
    _writeDetections = botConfig.check("saveDetections",0,"Whether to save or not the bottle of each object detection to a txt file.").asInt();
    //create folder where to save this experiments images and/or detections
    if(_saveOriginalLeftImages || _saveOriginalRightImages || _saveDrawnUponImages || _writeDetections){
        std::string timeString;
        {
            ACE_Time_Value t = ACE_OS::gettimeofday();
            std::stringstream ss;
            ss.fill('0');
            ss << t.sec();
            timeString = ss.str();
        }
        _imagePath = "images";
        mkdir(_imagePath.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
        _imagePath.append("/exp");
        _imagePath.append(timeString.c_str());
        mkdir(_imagePath.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
        _imagePath.append("/");
        _filePath = _imagePath;
        _filePath.append("bottles.txt");
    }
        
    //look for the focal lenghts to calculate disparity
    if (!config.findGroup("CAMERA_CALIBRATION_LEFT").isNull()){
        botConfig.clear();
        botConfig.fromString(config.findGroup("CAMERA_CALIBRATION_LEFT", "Loading calibration parameters from group CAMERA_CALIBRATION_LEFT.").toString());

        _left_fx = botConfig.check("fx",
                                  Value(0),
                                  "Focal length in x (double).").asDouble();
        _left_fy = botConfig.check("fy",
                                  Value(0),
                                  "Focal length in y (double).").asDouble();
        if(_left_fx == 0 || _left_fy == 0){
            cout <<"Unavailable focal lengths, quitting."<<endl;
            return false;
        }
    }
    else{
        cout << "Can't find CAMERA_CALIBRATION_LEFT group to read the focal lenghts. Quitting." <<endl;
        return false;
    }

    if(!_simulation){
        // we need to know the name of the server controlboard to connect to
        Value *valNameControlboard;
        if(!config.check("controlboard",
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
        //expected bottle result: (device remote_controlboard) (local /chica/sift/controlboard) (remote /controlboard)
        _dd.open(botControlboard);
        if(!_dd.view(_ienc)){
            cout << "Motor controlboard does not implement IEncoders!" << endl;
            return false;
        }
    
        //getting the axes information from the controlboard
        _numAxes = 0;
        _ienc->getAxes(&_numAxes);
        if (_numAxes == 0){
            cout << "*** Controlboard not available! Quitting. Check is controlboard name is correct (it needs to have a backslash first) or turn \"simulation\" on (--simulation 1)" << endl;
            return false;
        }
        else if (_numAxes < 6){
            cout << "Number of motor axes outside expected range (available: " << _numAxes << " required 6)." << endl;
            return false;
        }
        else{
            _encoders = new double[_numAxes];
        }
    }
    else{//if simulation, no real controlboard is available
        _numAxes=6;
        _encoders = new double[_numAxes];
        _encoders[0] = 0.1; _encoders[1] = 0.01; _encoders[2] = -0.02; _encoders[3] = -0.01; _encoders[4] = 0.05; _encoders[5] = 0.0;
    }
    //ControlGaze2 asks for the saccade orders to be numbered, and if we send a saccade with an already used number it will ignore it, therefore we generate random initial saccade number and count up from there
    srand(time(NULL)); 
    _saccadeIndex = rand() % 20000;
    
    //When ordering saccades, this will be the number of the prefered object to gaze at. If it is at -1 it will gaze at the average position of all available objects
    _preferedObject = -1; //the object numbers are >=0
 
    SIFT_Image::init( "" );
    Matching::init("" );
    Network::init(); ///what's this for?

    _prtImgLeft.open(getName("left"));
    _prtImgRight.open(getName("right"));
    _prtObjectPositions.open(getName("positions"));
    if(_orderSaccades)
        _prtVctPosOut.open(getName("sacPos"));
            
    //load database
    _recognizer.load_database();

    return true;
}

// standard destructor
siftObjectRepresentationModule::~siftObjectRepresentationModule() {}

// close function for the YARP module
// is called when the module is destroyed, clean up is done here
bool siftObjectRepresentationModule::close() {

    _prtImgLeft.close();
    _prtImgRight.close();
    _prtObjectPositions.close();
    if(_orderSaccades)
        _prtVctPosOut.close();
    
    _recognizer.save_database();

    Network::fini();

    //encoders
    _dd.close();
    if (_encoders != NULL)
        delete [] _encoders;
    _encoders = NULL;

    return true;
}

// interrupt function for the YARP module
// is called when the module communication is interrupted, this is passed on to all ports
bool siftObjectRepresentationModule::interruptModule() {
    _prtImgLeft.interrupt();
    _prtImgRight.interrupt();
    _prtObjectPositions.interrupt();
    if(_orderSaccades)
        _prtVctPosOut.interrupt();
    return true;
}

//helper function to save images to the disk
void siftObjectRepresentationModule::SaveImage( IplImage* image, std::string filename ) {
    char name[1000];
    
    sprintf( name, "%s%s.jpg", _imagePath.c_str(), filename.c_str() );
//     IplImage* imageBGR = cvCreateImage( cvGetSize(image), image->depth, 3 );
//     cvCvtColor( image, imageBGR, CV_RGB2BGR );
//     cvSaveImage( name, imageBGR );
//     cvReleaseImage(&imageBGR);
    
    cvSaveImage( name, image );
}

// update function for the YARP module
// is called when the module is updated (received data),
bool siftObjectRepresentationModule::updateModule() {

    ImageOf<PixelBgr> *left_image = _prtImgLeft.read(true); 
    ImageOf<PixelBgr> *right_image = _prtImgRight.read(false);
    Bottle *orderToLookAt = _prtObjectPositions.read(false);
            
//     std::string timeString;
//     {
//         ACE_Time_Value t = ACE_OS::gettimeofday();
//         std::stringstream ss;
//         ss.fill('0');
//         ss << t.sec();
//         ss << ":";
//         ss.fill('0');
//         ss.width(6);
//         ss << t.usec();
//         timeString = ss.str();
//     }
    
    if(!_simulation){
        if (_numAxes != 0)
            _ienc->getEncoders(_encoders); /// read encoder value
    }

    if(orderToLookAt != NULL){
        //cout << orderToLookAt->toString() <<endl;
        if(!orderToLookAt->find("LookAt").isNull() && orderToLookAt->find("LookAt").isInt()){
            int objectNumber = orderToLookAt->find("LookAt").asInt();
            cout<<"Looking at object number " << objectNumber << " preferably from now on."<<endl;
            _preferedObject = objectNumber;
        }
        else{
            cout << "Bottle badly formated, either it didn't have a 'LookAt' or the number wasn't an 'Int'. bottle:"<< orderToLookAt->toString() <<endl;
        }
    }
    
    if(left_image == NULL && right_image == NULL) { // application has been asked to quit
        return true;
    }
    
    if(left_image != NULL && right_image != NULL && _storeNewObjects) {
        IplImage * left_ipl = (IplImage*) left_image->getIplImage();
        IplImage * right_ipl = (IplImage*) right_image->getIplImage();

        if(left_ipl != NULL && right_ipl != NULL) {
            cout << "IplImages extracted of size "<< left_ipl->width<<"x"<<left_ipl->height<<" and "<< right_ipl->width<<"x"<<right_ipl->height;

            ///Two threads, each to process one SIFT image
            double t_start = (double)cvGetTickCount();
            Semaphore semaphore(0);
            ThreadSift tl; 
            tl.set_Semaphore(&semaphore); ///not sure how to guarantee that the constructor of "Thread" is still called while using a new constructor, so using this gimmik
            ThreadSift tr; 
            tr.set_Semaphore(&semaphore);
            tl.img = left_ipl;
            tr.img = right_ipl;
            
            tl.start();
            tr.start();
            semaphore.wait();
            //cout << "One thread finished"<<endl;
            semaphore.wait();
            //cout << "Second thread finished"<<endl;
            double t_end = (double)cvGetTickCount() - t_start;
            cout << " Parallel SIFT_Image processing finished in "<< t_end/(cvGetTickFrequency()*1000.)<<"ms."<<endl;
            //although the thread has already completed it's work, this officially terminates the thread
            tl.stop();
            tr.stop();

            //Matching left input image with database
            _recognizer.match_to_database(tl.sift_im);
            Bottle b;
            int size = _recognizer.make_bottle(&b, _encoders, _left_fx, _left_fy);
            cout << "Bottle: "<<b.toString()<<endl;
            if(size > 0){
                _prtObjectPositions.prepare() = b;
                _prtObjectPositions.write();
                if(_orderSaccades){
                    orderSaccade(b);
                }
            }
            //Writting to port
            ImageOf<PixelBgr> yarpReturnLeftImage;
            yarpReturnLeftImage.wrapIplImage(_recognizer.update_matched_image());
            _prtImgLeft.prepare() = yarpReturnLeftImage;
            _prtImgLeft.write();

            //save images to file
            int sec = b.find("sec").asInt();
            int usec = b.find("usec").asInt();
            if(_saveDrawnUponImages){
                stringstream filename;
                filename <<"match"<< sec <<"."<<usec;
                SaveImage(_recognizer.get_matched_image(),filename.str());
            }
            if(_saveOriginalRightImages){
                stringstream filename;
                filename <<"oriRight"<< sec <<"."<<usec;
                SaveImage(right_ipl,filename.str());
            }
            if(_saveOriginalLeftImages){
                stringstream filename2;
                filename2 <<"oriLeft"<< sec <<"."<<usec;
                SaveImage(left_ipl,filename2.str());
            }
            //save bottles to file
            if(_writeDetections){
                FILE * pFile = NULL;
                pFile = fopen (_filePath.c_str(),"a");
                if (pFile==NULL) {
                    cout <<"Not able to open " << _filePath <<endl;
                    return false;
                }
                fprintf (pFile,"%s\n",b.toString().c_str());
                fclose(pFile);
            }

            //Storing to DataBase last
            //Is there an object very close that we do not know?
            _dispar.go(tl.sift_im, tr.sift_im);
            if(_dispar.there_is_a_close_object){
                cout << "Close object detected ("<<_dispar.d_matches.get_size()<<" matches)! Does it already exist? ";
                //cout << "Close object in database?... ";
                ///close_obj is a SIFT_Image with fewer SIFT features in it's database than it is possible to extrac from it's image, because we only store features that correctly match with the right eye image (to be sure it's close features); so match_to_database(_dispar.close_obj.image_color) will match ALL the extractable features to the database, therefore truly checking if the object present in the image is new or not.
                ///When we implement the "tell the robot what he is looking at" we will remove this, because the more SIFT_Image objects the better, because they will all have the correct label, therefore improving the detection rate of the object shown.
                _recognizer.match_to_database(_dispar.close_obj.image_color);
                if(_recognizer.models.is_empty()) {
                    cout<<"The Object is new! Saving new object!"<<endl;
                    //Writting to /right port (the last stored object)
                    ImageOf<PixelBgr> yarpReturnRightImage;
                    yarpReturnRightImage.wrapIplImage(_dispar.close_obj.image_color);
                    _prtImgRight.prepare() = yarpReturnRightImage;
                    _prtImgRight.write();

                    _dispar.close_obj.draw_features(CROSS);
                    //store object.
                    ///Here we could ask user for object name, now we save object with name "number of objects"
                    _dispar.close_obj.label.assign(itos(_recognizer.training_samples.get_size()));
                    _dispar.close_obj.label.append("\n");
                    int near_obj_index = _recognizer.insert_database_image(_dispar.close_obj); 
                    
                    /**This checking is necessary to not store objects that cannot be seen, sometimes small images are stored but aren't found (because of small number of features) */
                    cout<< "Verifying if matchable.... ";
                    _recognizer.match_to_database(_dispar.close_obj); 
                    if(_recognizer.models.is_empty()){
                        cout << "Object could not be recognized after saving, deleting it instead."<<endl;
                        _recognizer.training_samples.remove_record(near_obj_index);
//                                 _recognizer.display_database_images();
                    }else
                        cout<<"Object recognized, leaving it saved in the database."<<endl;

                }else{
                    cout << "Object in close proximity, but is already known."<<endl;
                    //do something? send bottle with average_object_disparity?
                }
            }
        }
    }
    ///If not looking for new objects, only recognizing old ones (or if for some reason no image came from the right eye this iteration)
    else if(left_image != NULL){ 
        IplImage * left_ipl = (IplImage*) left_image->getIplImage();
        if(left_ipl != NULL){
            cout << "Left IplImage extracted of size "<< left_ipl->width<<"x"<<left_ipl->height<<endl;
            //Matching to Database
            _recognizer.match_to_database(left_ipl);
            Bottle b;
            int size = _recognizer.make_bottle(&b, _encoders, _left_fx, _left_fy);
            cout << "Bottle: "<<b.toString()<<endl;
            if(size > 0){
                //Writting bottle to port if anything was found
                _prtObjectPositions.prepare() = b;
                _prtObjectPositions.write();
                
                if(_orderSaccades){
                    orderSaccade(b);
                }
            }
            //Writting image to port (having added the features and bounding boxes of detected objects)
            ImageOf<PixelBgr> yarpReturnLeftImage;
            yarpReturnLeftImage.wrapIplImage(_recognizer.update_matched_image());
            _prtImgLeft.prepare() = yarpReturnLeftImage;
            _prtImgLeft.write();
           //save images to file
            int sec = b.find("sec").asInt();
            int usec = b.find("usec").asInt();
            if(_saveDrawnUponImages){
                stringstream filename;
                filename <<"match"<< sec <<"."<<usec;
                SaveImage(_recognizer.get_matched_image(),filename.str());
            }
             if(_saveOriginalLeftImages){
                stringstream filename2;
                filename2 <<"oriLeft"<< sec <<"."<<usec;
                SaveImage(left_ipl,filename2.str());
            }
            //save bottles to file
            if(_writeDetections){
                FILE * pFile = NULL;
                pFile = fopen (_filePath.c_str(),"a");
                if (pFile==NULL) {
                    cout <<"Not able to open " << _filePath <<endl;
                    return false;
                }
                fprintf (pFile,"%s\n",b.toString().c_str());
                fclose(pFile);
            }

        }
    }
        
    cout << "---------------------------- Finished a cycle --------------------------" <<endl;
    return true;
}

void siftObjectRepresentationModule::orderSaccade(Bottle ObjectPositions){
    double azimuth = 0, elevation = 0;
    if(!ObjectPositions.find("size").isNull() && ObjectPositions.find("size").isInt()){
        int size = ObjectPositions.find("size").asInt();
        char num[10];
        for(int i=0; i< size; i++){
            sprintf(num,"%d\0",i); 
            Bottle object = ObjectPositions.findGroup(num); 
            if(!object.isNull()){
                yarp::os::ConstString label = object.find("label").asString();
                int objectNumber = atoi(label.c_str());
                if(objectNumber == _preferedObject){
                    azimuth = object.find("abs_azimuth").asDouble();
                    elevation = object.find("abs_elevation").asDouble();
                    size = 1;
                    cout << "Looking at prefered object " << _preferedObject << "! ";
                    break;
                }
                azimuth = azimuth + object.find("abs_azimuth").asDouble();
                elevation = elevation + object.find("abs_elevation").asDouble();
            }
        }
        azimuth = azimuth/size;
        elevation = elevation/size;
    }
    _saccadeIndex++;    
     // prepare output vector
    VectorOf<double> &vctPos = _prtVctPosOut.prepare();
    vctPos.resize(5);
    vctPos(0) = azimuth;
    vctPos(1) = elevation;
    vctPos(2) = (double)(int)'a'; // absolute (neck reference) coordinates are sent
    vctPos(3) = (double)(int)'s'; // receiver module should do saccades
    vctPos(4) = _saccadeIndex;
    // write output vector
    _prtVctPosOut.write();
    cout <<"Saccading to (" << azimuth << " , " << elevation << ")."<<endl;
}
