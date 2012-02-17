#include <iostream>
#include <fstream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Stamp.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace yarp::sig;
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  
class stereoCalibThread : public Thread
{
private:

    ImageOf<PixelRgb> *imageL;
    ImageOf<PixelRgb> *imageR;
    IplImage * imgL;
    IplImage * imgR;

    int numOfPairs;
    bool stereo;
    Mat Kleft;
    Mat Kright;
    
    Mat DistL;
    Mat DistR;

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
    char pathL[256];
    char pathR[256];
    void printMatrix(Mat &matrix);
    bool checkTS(double TSLeft, double TSRight, double th=0.020);
    void preparePath(const char * imageDir, char* pathL, char* pathR, int num);
    void saveStereoImage(const char * imageDir, IplImage* left, IplImage * right, int num);
    void monoCalibration(const vector<string>& imageList, int boardWidth, int boardHeight, Mat &K, Mat &Dist);
    void stereoCalibration(const vector<string>& imagelist, int boardWidth, int boardHeight,float sqsizee);
    void saveCalibration(const string& extrinsicFilePath, const string& intrinsicFilePath);
    void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners);
    bool updateIntrinsics( int width, int height, double fx, double fy,double cx, double cy, double k1, double k2, double p1, double p2, const string& groupname);
    bool updateExtrinsics(Mat Rot, Mat Tr, const string& groupname);
    void saveImage(const char * imageDir, IplImage* left, int num);
    void stereoCalibRun();
    void monoCalibRun();

public:


    stereoCalibThread(ResourceFinder &rf, Port* commPort, const char *imageDir);
    void startCalib();
    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();

};


