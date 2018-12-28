#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;


#define LEFT    0
#define RIGHT   1

class stereoCalibThread : public Thread
{
private:

    ImageOf<PixelRgb> *imageL;
    ImageOf<PixelRgb> *imageR;
    Mat Left;
    Mat Right;

    string moduleName;
    string robotName;
    yarp::sig::Vector qL;
    yarp::sig::Vector qR;

    Semaphore* mutex;

    int numOfPairs;
    bool stereo;
    Mat Kleft;
    Mat Kright;
    
    Mat DistL;
    Mat DistR;
    double vergence;
    double version;


    yarp::dev::PolyDriver polyHead;
    yarp::dev::IEncoders *posHead;

    yarp::dev::PolyDriver polyTorso;
    yarp::dev::IEncoders *posTorso;

    Mat R;
    Mat T;
    Mat Q;
    string inputLeftPortName;
    string inputRightPortName;
    string outNameRight;
    string outNameLeft;
    string camCalibFile;
    string currentPathDir;
    std::vector<string> imageListR;
    std::vector<string> imageListL;
    std::vector<string> imageListLR;

    BufferedPort<ImageOf<PixelRgb> > imagePortInLeft;
    BufferedPort<ImageOf<PixelRgb> > imagePortInRight;
    BufferedPort<ImageOf<PixelRgb> > outPortRight;
    BufferedPort<ImageOf<PixelRgb> > outPortLeft;

    Port *commandPort;
    string imageDir;
    int startCalibration;
    int boardWidth;
    int boardHeight;
    float squareSize;
    string boardType;
    char pathL[256];
    char pathR[256];
    void printMatrix(Mat &matrix);
    bool checkTS(double TSLeft, double TSRight, double th=0.08);
    void preparePath(const char * imageDir, char* pathL, char* pathR, int num);
    void saveStereoImage(const char * imageDir, const Mat& left, const Mat& right, int num);
    void monoCalibration(const vector<string>& imageList, int boardWidth, int boardHeight, Mat &K, Mat &Dist);
    void stereoCalibration(const vector<string>& imagelist, int boardWidth, int boardHeight,float sqsizee);
    void saveCalibration(const string& extrinsicFilePath, const string& intrinsicFilePath);
    void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners);
    bool updateIntrinsics( int width, int height, double fx, double fy,double cx, double cy, double k1, double k2, double p1, double p2, const string& groupname);
    bool updateExtrinsics(Mat Rot, Mat Tr, const string& groupname);
    void saveImage(const char * imageDir, const Mat& left, int num);
    void stereoCalibRun();
    void monoCalibRun();

public:


    stereoCalibThread(ResourceFinder &rf, Port* commPort, const char *imageDir);
    void startCalib();
    void stopCalib();
    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();

};


