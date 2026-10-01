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
#include "CalibrationTypes.h"
#include "CalibrationWriter.h"
#include "FisheyeCalibrationEngine.h"
#include "PinholeCalibrationEngine.h"

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

    // These fields are available only after a successful engine calibration.
    bool calibrationAvailable{false};
    std::string calibrationMode;
    double leftMonocularRms{-1.0};
    double rightMonocularRms{-1.0};
    double stereoRms{-1.0};
    double baselineNorm{-1.0};
    std::string lastCalibrationError;
};

class StereoPairSynchronizer
{
private:
    std::deque<StampedFrame> leftQueue;
    std::deque<StampedFrame> rightQueue;

    double toleranceSeconds{0.020}; // 20 milliseconds
    std::size_t maxQueueSize{5};

    mutable std::mutex _mutex;
    SynchronizerStatistics stats;
    void trimLeftQueue();
    void trimRightQueue();

public:

    void configure(double tolerance, std::size_t maxQueueSize);
    void reset();
    void pushLeft(const ImageOf<PixelRgb>& leftFrame, const Stamp& timestamp);
    void pushRight(const ImageOf<PixelRgb>& rightFrame, const Stamp& timestamp);
    bool tryPopPair(SynchronizedPair& pair);
    SynchronizerStatistics getStatistics() const { std::lock_guard<std::mutex> lock(_mutex); return stats; }
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

    double _syncToleranceSeconds{0.020};
    std::size_t _syncQueueSize{5};

    double minCaptureIntervalSeconds{2.0};
    double lastProcessedCandidateTime{-1.0};
    double minimumBoardSpanRatio{0.15};

    Size _expectedImageSize{};

    Mat LeftRgb;
    Mat RightRgb;

    string moduleName;
    string robotName;
    yarp::sig::Vector qL;
    yarp::sig::Vector qR;

    mutable std::mutex mtx;

    int numOfPairs;

    bool standalone;
    yarp::dev::PolyDriver polyHead;
    yarp::dev::IEncoders *posHead;

    yarp::dev::PolyDriver polyTorso;
    yarp::dev::IEncoders *posTorso;

    string inputLeftPortName;
    string inputRightPortName;
    string outNameRight;
    string outNameLeft;
    string camCalibFile;

    stereo_calib::ChessboardConfiguration _chessboardConfiguration;
    std::vector<stereo_calib::StereoObservation> _observations;
    std::size_t _rejectedDetections{0};

    stereo_calib::CameraModel _cameraModel{stereo_calib::CameraModel::Pinhole};
    stereo_calib::CalibrationMode _calibrationMode{stereo_calib::CalibrationMode::StereoFull};

    stereo_calib::FisheyeCalibrationEngine _fisheyeCalibrationEngine;
    stereo_calib::FisheyeCalibrationOptions _fisheyeCalibrationOptions;

    stereo_calib::PinholeCalibrationEngine _pinholeCalibrationEngine;
    stereo_calib::PinholeCalibrationOptions _pinholeCalibrationOptions;

    stereo_calib::CalibrationResult _calibrationResults;
    stereo_calib::CalibrationWriter _calibrationWriter;

    std::string _calibrationError;

    bool _saveImages{false};
    bool _drawDiagnosticCorners{true};
    std::string _observationsFile;


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
    void stereoCalibRun();
    bool shouldQueueFrameForCollection(const Stamp& timestamp) const;
    void processSynchronizedPair(SynchronizedPair& pair, Size boardSize);

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
