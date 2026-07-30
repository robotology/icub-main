#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <atomic>
#include <cstddef>
#include <deque>

#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgcodecs.hpp>



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

struct StampedFrame
{
    ImageOf<PixelRgb> image;
    Stamp stamp;
};

struct SynchronizedPair
{
    ImageOf<PixelRgb> left;
    ImageOf<PixelRgb> right;

    Stamp leftStamp;
    Stamp rightStamp;

    double timeStampDelta{0.0};
};

struct SynchronizerStatistics
{
    std::size_t pairedFrames{0};
    std::size_t droppedLeftFrames{0};
    std::size_t droppedRightFrames{0};

    double accumulatedTimeStampDelta{0.0};
    double maxTimeStampDelta{0.0};
};

struct StereoCalibStatus
{
    std::string state;

    std::size_t pairedFrames{0};
    std::size_t droppedLeftFrames{0};
    std::size_t droppedRightFrames{0};

    double meanTimestampDeltaMs{0.0};
    double maxTimestampDeltaMs{0.0};
};

class StereoPairSynchronizer
{
private:
    std::deque<StampedFrame> leftQueue;
    std::deque<StampedFrame> rightQueue;

    double _toleranceSeconds{0.020}; // 20 milliseconds
    std::size_t _maxQueueSize{5};

    SynchronizerStatistics stats;
    void trimLeftQueue();
    void trimRightQueue();

public:

    void configure(double tolerance, std::size_t maxQueueSize);
    void reset();
    void pushLeft(const ImageOf<PixelRgb>& leftFrame, const Stamp& timestamp);
    void pushRight(const ImageOf<PixelRgb>& rightFrame, const Stamp& timestamp);
    bool tryPopPair(SynchronizedPair& pair);
    const SynchronizerStatistics& getStatistics() const { return stats; }
};

class stereoCalibThread : public Thread
{
private:

    // States for stereoCalibrationThead
    // defined inside the class since they are totally referred to this class
    enum class CalibrationState
    {
        Idle = 0,
        Collecting,
        Calibrating,
        Completed,
        Error = 255
    };

    std::atomic<CalibrationState> calibrationState{CalibrationState::Idle};
    std::atomic<bool> collectionResetRequested{false};

    StereoPairSynchronizer synchronizer;

    double syncToleranceSeconds{0.020};
    std::size_t syncQueueSize{5};

    ImageOf<PixelRgb> *imageL;
    ImageOf<PixelRgb> *imageR;
    Mat Left;
    Mat Right;

    string moduleName;
    string robotName;
    yarp::sig::Vector qL;
    yarp::sig::Vector qR;

    mutex mtx;

    int numOfPairs;
    bool stereo;
    Mat Kleft;
    Mat Kright;
    
    Mat DistL;
    Mat DistR;
    double vergence;
    double version;

    bool standalone;
    yarp::dev::PolyDriver polyHead;
    yarp::dev::IEncoders *posHead;

    yarp::dev::PolyDriver polyTorso;
    yarp::dev::IEncoders *posTorso;

    Mat R;
    Mat T;
    Mat Q;
    Size lastImageSize;
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
    double monoCalibration(const vector<string>& imageList, int boardWidth, int boardHeight, Mat &K, Mat &Dist, const char* cameraName);
    void stereoCalibration(const vector<string>& imagelist, int boardWidth, int boardHeight,float sqsizee);
    void saveCalibration(const string& extrinsicFilePath, const string& intrinsicFilePath);
    void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners);
    bool updateIntrinsics( int width, int height, double fx, double fy,double cx, double cy, double k1, double k2, double p1, double p2, const string& groupname);
    bool updateExtrinsics(Mat Rot, Mat Tr, const string& groupname);
    void saveImage(const char * imageDir, const Mat& left, int num);
    void stereoCalibRun();
    void monoCalibRun();
    void processSynchronizedPair(SynchronizedPair& pair, int count, Size boardSize);

public:

    stereoCalibThread(ResourceFinder &rf, Port* commPort, const char *imageDir);
    StereoCalibStatus getStatus() const;
    void startCalib();
    void stopCalib();
    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();

};


