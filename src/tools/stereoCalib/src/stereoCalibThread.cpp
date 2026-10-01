#include <algorithm>
#include <cmath>
#include <utility>
#include <sstream>
#include <yarp/cv/Cv.h>
#include <yarp/os/LogStream.h>
#include "stereoCalibThread.h"


YARP_LOG_COMPONENT(STEREOCALIBRATIONTHREAD, "yarp.tools.stereoCalibThread")

namespace
{
 
    std::string formatCalibrationMatrix(const cv::Mat& matrix)
    {
        std::ostringstream stream;
        stream << matrix;
        return stream.str();
    }
 
    void logCameraCalibration(const char* cameraName,
                            const stereo_calib::CameraCalibrationResult& camera)
    {
        yCInfo(STEREOCALIBRATIONTHREAD) << "[CAMERA_CALIBRATION_" << cameraName << "]";
        yCInfo(STEREOCALIBRATIONTHREAD) << "w" << camera.imageSize.width << "h" << camera.imageSize.height;
        yCInfo(STEREOCALIBRATIONTHREAD) << "fx" << camera.K.at<double>(0, 0)
                << "fy" << camera.K.at<double>(1, 1)
                << "cx" << camera.K.at<double>(0, 2)
                << "cy" << camera.K.at<double>(1, 2);
        if(camera.model == stereo_calib::CameraModel::Fisheye)
        {
            yCInfo(STEREOCALIBRATIONTHREAD) << "k1" << camera.D.at<double>(0, 0)
                    << "k2" << camera.D.at<double>(1, 0)
                    << "k3" << camera.D.at<double>(2, 0)
                    << "k4" << camera.D.at<double>(3, 0);
        }
        else if(camera.model == stereo_calib::CameraModel::Pinhole)
        {
            yCInfo(STEREOCALIBRATIONTHREAD) << "k1" << camera.D.at<double>(0, 0)
                    << "k2" << camera.D.at<double>(1, 0)
                    << "p1" << camera.D.at<double>(2, 0)
                    << "p2" << camera.D.at<double>(3, 0);
        }
        yCInfo(STEREOCALIBRATIONTHREAD) << "K =" << formatCalibrationMatrix(camera.K);
        yCInfo(STEREOCALIBRATIONTHREAD) << "monocular RMS =" << camera.rms;
    }
 
    void logCalibrationResult(const stereo_calib::CalibrationResult& result)
    {
        yCInfo(STEREOCALIBRATIONTHREAD) << "========== Calibration result (not written to disk) ==========";
    
        if(result.leftCamera.isValid())
        {
            logCameraCalibration("LEFT", result.leftCamera);
        }
        if(result.rightCamera.isValid())
        {
            logCameraCalibration("RIGHT", result.rightCamera);
        }
    
        if(result.stereo.isValid())
        {
            cv::Mat homogeneousTransform = cv::Mat::eye(4, 4, CV_64F);
            result.stereo.R.copyTo(homogeneousTransform(cv::Rect(0, 0, 3, 3)));
            result.stereo.T.reshape(1, 3).copyTo(homogeneousTransform(cv::Rect(3, 0, 1, 3)));
    
            yCInfo(STEREOCALIBRATIONTHREAD) << "[STEREO_DISPARITY]";
            yCInfo(STEREOCALIBRATIONTHREAD) << "Stereo RMS =" << result.stereo.rms
                    << "baseline norm =" << cv::norm(result.stereo.T);
            yCInfo(STEREOCALIBRATIONTHREAD) << "R =" << formatCalibrationMatrix(result.stereo.R);
            yCInfo(STEREOCALIBRATIONTHREAD) << "T =" << formatCalibrationMatrix(result.stereo.T.t());
            // HN is the same homogeneous transform layout used by outputCalib.ini.
            yCInfo(STEREOCALIBRATIONTHREAD) << "HN =" << formatCalibrationMatrix(homogeneousTransform);
        }
        yCInfo(STEREOCALIBRATIONTHREAD) << "======================================================================";
    }
    
    stereo_calib::CameraModel parseCameraModel(const std::string& modelString)
    {
        if(modelString == "pinhole")
        {
            return stereo_calib::CameraModel::Pinhole;
        }
        else if(modelString == "fisheye")
        {
            return stereo_calib::CameraModel::Fisheye;
        }
        else
        {
            yCError(STEREOCALIBRATIONTHREAD) << "Invalid camera model string:" << modelString
                    << ". Setting to Invalid.";
            return stereo_calib::CameraModel::Invalid;
        }
    }

    stereo_calib::CalibrationMode parseCalibrationMode(const std::string& modeString)
    {
        if(modeString == "MonocularLeft")
        {
            return stereo_calib::CalibrationMode::MonocularLeft;
        }
        else if(modeString == "MonocularRight")
        {
            return stereo_calib::CalibrationMode::MonocularRight;
        }
        else if(modeString == "MonocularBoth")
        {
            return stereo_calib::CalibrationMode::MonocularBoth;
        }
        else if(modeString == "StereoFull")
        {
            return stereo_calib::CalibrationMode::StereoFull;
        }
        else
        {
            yCError(STEREOCALIBRATIONTHREAD) << "Invalid calibration mode string:" << modeString
                    << ". Using default StereoFull.";
            return stereo_calib::CalibrationMode::StereoFull;
        }
    }
} // namespace

void StereoPairSynchronizer::configure(double tolerance, std::size_t maxQueueSize)
{
    this->toleranceSeconds = tolerance;
    this->maxQueueSize = maxQueueSize;
}

void StereoPairSynchronizer::reset()
{
    leftQueue.clear();
    rightQueue.clear();

    std::lock_guard<std::mutex> lock(_mutex);
    stats = SynchronizerStatistics{};
}

void StereoPairSynchronizer::pushLeft(const ImageOf<PixelRgb>& leftFrame, const Stamp& timestamp)
{
    StampedFrame frame;

    // Must be a deep copy. Do not retain pointer to the input-port buffer
    frame.image.copy(leftFrame);
    frame.stamp = timestamp;

    leftQueue.push_back(std::move(frame));
    trimLeftQueue();
}

void StereoPairSynchronizer::pushRight(const ImageOf<PixelRgb>& rightFrame, const Stamp& timestamp)
{
    StampedFrame frame;

    // Must be a deep copy. Do not retain pointer to the input-port buffer
    frame.image.copy(rightFrame);
    frame.stamp = timestamp;

    rightQueue.push_back(std::move(frame));
    trimRightQueue();
}


void StereoPairSynchronizer::trimLeftQueue()
{
    while(leftQueue.size() > maxQueueSize) 
    {
        leftQueue.pop_front();
        std::lock_guard<std::mutex> lock(_mutex);
        ++stats.droppedLeftFrames;
    }
}

void StereoPairSynchronizer::trimRightQueue()
{
    while(rightQueue.size() > maxQueueSize) 
    {
        rightQueue.pop_front();
        std::lock_guard<std::mutex> lock(_mutex);
        ++stats.droppedRightFrames;
    }
}

bool StereoPairSynchronizer::tryPopPair(SynchronizedPair& pair)
{
    while(!leftQueue.empty() && !rightQueue.empty()) 
    {
        const double leftStamp = leftQueue.front().stamp.getTime();
        const double rightStamp = rightQueue.front().stamp.getTime();

        const double absoluteDelta = std::abs(leftStamp - rightStamp);
        if(absoluteDelta <= toleranceSeconds) 
        {
            pair.left = std::move(leftQueue.front().image);
            pair.right = std::move(rightQueue.front().image);
            pair.leftStamp = leftQueue.front().stamp;
            pair.rightStamp = rightQueue.front().stamp;

            pair.timeStampDelta = absoluteDelta;

            leftQueue.pop_front();
            rightQueue.pop_front();

            std::lock_guard<std::mutex> lock(_mutex);
            ++stats.pairedFrames;
            stats.accumulatedTimeStampDelta += absoluteDelta;
            stats.maxTimeStampDelta = std::max(stats.maxTimeStampDelta, absoluteDelta);

            return true;
        }

        if(leftStamp < rightStamp) 
        {
            // the oldest left frame cannot be paired with this or any newer right frame, so drop it
            leftQueue.pop_front();
            std::lock_guard<std::mutex> lock(_mutex);
            ++stats.droppedLeftFrames;
        } 
        else 
        {
            rightQueue.pop_front();
            std::lock_guard<std::mutex> lock(_mutex);
            ++stats.droppedRightFrames;
        }
    }
    return false;
}


stereoCalibThread::stereoCalibThread(ResourceFinder &rf, Port* commPort, const char *imageDir)
{
    moduleName=rf.check("name", Value("stereoCalib"),"module name (string)").asString().c_str();
    robotName=rf.check("robotName",Value("icub"), "module name (string)").asString().c_str();

    this->inputLeftPortName = "/"+moduleName;
    this->inputLeftPortName +=rf.check("imgLeft",Value("/cam/left:i"),"Input image port (string)").asString().c_str();

    this->inputRightPortName = "/"+moduleName;
    this->inputRightPortName += rf.check("imgRight", Value("/cam/right:i"),"Input image port (string)").asString().c_str();

    this->outNameRight = "/"+moduleName;
    this->outNameRight += rf.check("outRight",Value("/cam/right:o"),"Output image port (string)").asString().c_str();

    this->outNameLeft = "/"+moduleName;
    this->outNameLeft +=rf.check("outLeft",Value("/cam/left:o"),"Output image port (string)").asString().c_str();

    Bottle stereoCalibOpts=rf.findGroup("STEREO_CALIBRATION_CONFIGURATION");
    this->boardWidth = stereoCalibOpts.check("boardWidth", Value(8)).asInt32();
    this->boardHeight = stereoCalibOpts.check("boardHeight", Value(6)).asInt32();
    this->numOfPairs = stereoCalibOpts.check("numberOfPairs", Value(30)).asInt32();
    if(this->numOfPairs < 30)
    {
        yCWarning(STEREOCALIBRATIONTHREAD) << "numberOfPairs must be at least 30; using 30";
        this->numOfPairs = 30;
    }
    this->squareSize = (float)stereoCalibOpts.check("boardSize", Value(0.09241)).asFloat64();
    this->boardType =  stereoCalibOpts.check("boardType", Value("CHESSBOARD")).asString();
    const double syncToleranceMs = stereoCalibOpts.check("syncToleranceMs", Value(20.0)).asFloat64();
    _syncToleranceSeconds = syncToleranceMs / 1000.0;
    const int configuredQueueSize = stereoCalibOpts.check("syncQueueSize", Value(5)).asInt32();
    if(configuredQueueSize <= 0)
    {
        yCWarning(STEREOCALIBRATIONTHREAD) << "Invalid syncQueueSize; using 5";
        _syncQueueSize = 5;
    }
    else
    {
        _syncQueueSize = static_cast<std::size_t>(configuredQueueSize);
    }

    if(_syncToleranceSeconds <= 0.0)
    {
        yCWarning(STEREOCALIBRATIONTHREAD) << "Invalid syncToleranceMs; using 20 ms";
        _syncToleranceSeconds = 0.020;
    }

    synchronizer.configure(_syncToleranceSeconds, _syncQueueSize);
    
    this->minCaptureIntervalSeconds = stereoCalibOpts.check("minCaptureIntervalSeconds", Value(2.0)).asFloat64();
    this->minimumBoardSpanRatio = stereoCalibOpts.check("minimumBoardSpanRatio", Value(0.15)).asFloat64();
    this->commandPort=commPort;
    this->imageDir=imageDir;
    this->collectionResetRequested.store(false);
    this->calibrationState.store(CalibrationState::Idle);
    // All new calibration modes use the synchronized-observation pipeline.
    // In particular, completion must never bypass CalibrationWriter.
    this->camCalibFile=rf.getHomeContextPath().c_str();
    this->standalone = rf.check("standalone", Value(false)).asBool();
    string fileName= "outputCalib.ini";

    this->camCalibFile=this->camCalibFile+"/"+fileName.c_str();

    _observationsFile = stereoCalibOpts.check(
        "observationsFile", Value("calibrationObservations.yml")).asString();
    if(!_observationsFile.empty() && _observationsFile.front() != '/')
    {
        _observationsFile = this->imageDir + "/" + _observationsFile;
    }

    // TODO: remove. This is a duplication. Enough to set: _chessboardConfiguration.cornersX stereoCalibOpts.check("boardWidth", Value(8)).asInt32();
    _chessboardConfiguration.cornersX = this->boardWidth;
    _chessboardConfiguration.cornersY = this->boardHeight;

    // TODO: remove as well --> duplication
    _chessboardConfiguration.squareSizeMeters = this->squareSize;

    _saveImages = stereoCalibOpts.check("saveImages", Value(1)).asInt32() != 0;
    _drawDiagnosticCorners = stereoCalibOpts.check("drawDiagnosticCorners", Value(1)).asInt32() != 0;
    _cameraModel = parseCameraModel(stereoCalibOpts.check("cameraModel", Value("pinhole")).asString());
    _calibrationMode = parseCalibrationMode(stereoCalibOpts.check("calibrationMode", Value("StereoFull")).asString());

    if(!_chessboardConfiguration.isValid())
    {
        yCError(STEREOCALIBRATIONTHREAD) << "Invalid chessboard configuration";
    }
}

bool stereoCalibThread::threadInit()
{
     if (!imagePortInLeft.open(inputLeftPortName.c_str())) {
      cout  << ": unable to open port " << inputLeftPortName << endl;
      return false;
   }

   if (!imagePortInRight.open(inputRightPortName.c_str())) {
      cout << ": unable to open port " << inputRightPortName << endl;
      return false;
   }

    if (!outPortLeft.open(outNameLeft.c_str())) {
      cout << ": unable to open port " << outNameLeft << endl;
      return false;
   }

    if (!outPortRight.open(outNameRight.c_str())) {
      cout << ": unable to open port " << outNameRight << endl;
      return false;
   }

    //when in standalone mode we won't open the remote control board devices
    if(standalone) return true;

    //TODO: develop what to do with the control boards for head and torso
    // in the legacy implementation the kinematic chain was calculated and joint position added to the output file
    // now why we need that? is it uselful? do we need to change it to a check on steadiness for the calibration procedure
    Property optHead;
    optHead.put("device","remote_controlboard");
    optHead.put("remote",("/"+robotName+"/head").c_str());
    optHead.put("local","/"+moduleName+"/client/head");
    if (!polyHead.open(optHead) ||
        !polyHead.view(posHead) ||
        posHead == nullptr)
    {
        yCError(STEREOCALIBRATIONTHREAD) << "Unable to acquire the head encoder interface";
        return false;
    }

    Property optTorso;
    optTorso.put("device","remote_controlboard");
    optTorso.put("remote",("/"+robotName+"/torso").c_str());
    optTorso.put("local","/"+moduleName+"/client/torso");

    bool useTorso=true;
    if (!polyTorso.open(optTorso) ||
        !polyTorso.view(posTorso) ||
        posTorso == nullptr)
    {
        yCWarning(STEREOCALIBRATIONTHREAD, "Unable to connect to torso! Continuing without...");
        useTorso=false;
    }

    yarp::sig::Vector head_angles(6,0.0);
    posHead->getEncoders(head_angles.data());

    yarp::sig::Vector torso_angles(3,0.0);
    if (useTorso)
        posTorso->getEncoders(torso_angles.data());

    qL.resize(torso_angles.length()+head_angles.length()-1);
    for(size_t i=0; i<torso_angles.length(); i++)
        qL[i]=torso_angles[torso_angles.length()-i-1];

    for(size_t i=0; i<head_angles.length()-2; i++)
        qL[i+torso_angles.length()]=head_angles[i];
    qL[7]=head_angles[4]+(0.5-(LEFT))*head_angles[5];
    qL=iCub::ctrl::CTRL_DEG2RAD*qL;

    qR.resize(torso_angles.length()+head_angles.length()-1);
    for(size_t i=0; i<torso_angles.length(); i++)
        qR[i]=torso_angles[torso_angles.length()-i-1];

    for(size_t i=0; i<head_angles.length()-2; i++)
        qR[i+torso_angles.length()]=head_angles[i];
    qR[7]=head_angles[4]+(0.5-(RIGHT))*head_angles[5];
    qR=iCub::ctrl::CTRL_DEG2RAD*qR;

    return true;
}
void stereoCalibThread::run(){
    yCInfo(STEREOCALIBRATIONTHREAD, "Running synchronized fisheye calibration pipeline... \n");
    stereoCalibRun();
}

void stereoCalibThread::processSynchronizedPair(SynchronizedPair& pair, Size boardSize)
{
    // Check minimum capture interval
    const double pairTime = 0.5 * (pair.leftStamp.getTime() + pair.rightStamp.getTime()); //average pair timestamp
    const double previousProcessedCandidateTime = lastProcessedCandidateTime;
    if(previousProcessedCandidateTime >= 0.0 && (pairTime - previousProcessedCandidateTime) < minCaptureIntervalSeconds)
    {
        yCDebug(STEREOCALIBRATIONTHREAD) << "Skipping candidate pair due to minimum capture interval";
        return;
    }
    if(previousProcessedCandidateTime >= 0.0)
    {
        yCDebug(STEREOCALIBRATIONTHREAD) << "Timestamp delta between processed pairs:" << (pairTime - previousProcessedCandidateTime) << "seconds";
    }
    lastProcessedCandidateTime = pairTime;

    
    bool foundL=false;
    bool foundR=false;
    const Size leftSize(pair.left.width(), pair.left.height());
    const Size rightSize(pair.right.width(), pair.right.height());

    if(leftSize != _expectedImageSize || rightSize != _expectedImageSize)
    {
        if(_expectedImageSize.empty())
        {
            _expectedImageSize = leftSize;
        }
    }

    if(leftSize != _expectedImageSize || rightSize != _expectedImageSize)
    {
        yCError(STEREOCALIBRATIONTHREAD) << "Left and right images have different sizes:" <<
            "Left:" << leftSize.width << "x" << leftSize.height <<
            "Right:" << rightSize.width << "x" << rightSize.height;
        {
            std::lock_guard<std::mutex> lock(mtx);
            _calibrationError = "Input images do not match the expected calibration resolution.";
            _calibrationResults = stereo_calib::CalibrationResult{};
        }
        calibrationState.store(CalibrationState::Error);

        return;
    }

    LeftRgb=yarp::cv::toCvMat(pair.left);
    RightRgb=yarp::cv::toCvMat(pair.right);

    // Color adjust
    Mat leftGray, rightGray;
    cvtColor(LeftRgb,leftGray,CV_RGB2GRAY);
    cvtColor(RightRgb,rightGray,CV_RGB2GRAY);

    std::vector<Point2f> leftCorners;
    std::vector<Point2f> rightCorners;

    //TODO: for now we are only using CHESSBOARD GRID. For circle grid and so on we will add an update in the future.
    if(boardType == "CIRCLES_GRID") {
        foundL = findCirclesGrid(LeftRgb, boardSize, leftCorners, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
        foundR = findCirclesGrid(RightRgb, boardSize, rightCorners, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
        yCWarning(STEREOCALIBRATIONTHREAD) << "Board type:" << boardType << "not yet implemented.";
    } else if(boardType == "ASYMMETRIC_CIRCLES_GRID") {
        foundL = findCirclesGrid(LeftRgb, boardSize, leftCorners, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
        foundR = findCirclesGrid(RightRgb, boardSize, rightCorners, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
        yCWarning(STEREOCALIBRATIONTHREAD) << "Board type:" << boardType << "not yet implemented.";
    } else if(boardType == "CHESSBOARD_SECTOR_BASED") {
        foundL = findChessboardCornersSB(leftGray, boardSize, leftCorners);
        foundR = findChessboardCornersSB(rightGray, boardSize, rightCorners);
        yCWarning(STEREOCALIBRATIONTHREAD) << "Board type:" << boardType << "not yet implemented.";
    } else {
        foundL = findChessboardCorners(leftGray, boardSize, leftCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
        foundR = findChessboardCorners(rightGray, boardSize, rightCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
    }

    if(foundL && foundR) 
    {
        yCDebug(STEREOCALIBRATIONTHREAD) << "Found calibration board corners in both left and right images";

        const Rect leftBoardBounds = boundingRect(leftCorners);
        const Rect rightBoardBounds = boundingRect(rightCorners);
        const bool leftBoardTooSmall =
            leftBoardBounds.width < leftSize.width * minimumBoardSpanRatio ||
            leftBoardBounds.height < leftSize.height * minimumBoardSpanRatio;
        const bool rightBoardTooSmall =
            rightBoardBounds.width < rightSize.width * minimumBoardSpanRatio ||
            rightBoardBounds.height < rightSize.height * minimumBoardSpanRatio;

        if(leftBoardTooSmall || rightBoardTooSmall)
        {
            yCWarning(STEREOCALIBRATIONTHREAD) << "Skipping stereo pair: chessboard is too small."
                       << "Left board:" << leftBoardBounds.width << "x" << leftBoardBounds.height
                       << "of" << leftSize.width << "x" << leftSize.height << ";"
                       << "right board:" << rightBoardBounds.width << "x" << rightBoardBounds.height
                       << "of" << rightSize.width << "x" << rightSize.height << "."
                       << "Each board must span at least" << (minimumBoardSpanRatio * 100.0)
                       << "% of both image dimensions.";
            
            ++_rejectedDetections;
            return;
        }

        TermCriteria criteria = TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01);
        cornerSubPix(leftGray, leftCorners, Size(5,5), Size(-1,-1), criteria);
        cornerSubPix(rightGray, rightCorners, Size(5,5), Size(-1,-1), criteria);

        // Detection and geometry have succeeded.  Build and validate the
        // complete observation before assigning its accepted index.
        stereo_calib::StereoObservation observation;
        observation.imageSize = Size(pair.left.width(), pair.left.height());

        observation.objectPoints = _chessboardConfiguration.createObjectPoints();
        observation.leftImagePoints = leftCorners;
        observation.rightImagePoints = rightCorners;
        
        observation.leftTimestampSeconds = pair.leftStamp.getTime();
        observation.rightTimestampSeconds = pair.rightStamp.getTime();
        observation.timestampDeltaSeconds = pair.timeStampDelta;

        observation.leftSequenceNumber = pair.leftStamp.getCount();
        observation.rightSequenceNumber = pair.rightStamp.getCount();

        if(!observation.isValid())
        {
            yCError(STEREOCALIBRATIONTHREAD) << "Generated invalid stereo observation";
            ++_rejectedDetections;
            return;
        }

        std::size_t observationIndex = 0;
        observationIndex = _observations.size();

        // Raw images are saved before drawing any optional diagnostics and
        // only after the pair has become an accepted observation candidate.
        if(_saveImages)
        {
            std::string imageError;
            if(!_calibrationWriter.writeImagePair(imageDir, observationIndex,
                                                  LeftRgb, RightRgb,
                                                  observation.leftImageFilename,
                                                  observation.rightImageFilename,
                                                  imageError))
            {
                yCError(STEREOCALIBRATIONTHREAD) << "Could not save accepted calibration image pair:" << imageError;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    if (calibrationState.load() != CalibrationState::Collecting)
                    {
                        return;
                    }
                    
                    _calibrationError = imageError;
                    _calibrationResults = stereo_calib::CalibrationResult{};
                    calibrationState.store(CalibrationState::Error);
                }
                return;
            }
        }

        // Detection and validation have completed.
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (calibrationState.load() != CalibrationState::Collecting)
            {
                return;
            }
            _observations.push_back(std::move(observation));
        }

        // Diagnostic overlays are deliberately the final step: they never
        // affect the persisted raw dataset or the stored corner coordinates.
        if(_drawDiagnosticCorners)
        {
            drawChessboardCorners(LeftRgb, boardSize, leftCorners, foundL);
            drawChessboardCorners(RightRgb, boardSize, rightCorners, foundR);
        }

        ImageOf<PixelRgb>& outimL = outPortLeft.prepare();
        outimL = pair.left;
        outPortLeft.setEnvelope(pair.leftStamp);
        outPortLeft.write();

        ImageOf<PixelRgb>& outimR = outPortRight.prepare();
        outimR = pair.right;
        outPortRight.setEnvelope(pair.rightStamp);
        outPortRight.write();

    }
    else
    {
        ++_rejectedDetections;
    }

   return;
}

StereoCalibStatus stereoCalibThread::getStatus() const
{
    StereoCalibStatus result;

    std::lock_guard<std::mutex> lock(mtx);
    
    switch(calibrationState.load())
    {
    case CalibrationState::Idle:
        result.state = "Idle";
        break;
    case CalibrationState::Collecting:
        result.state = "Collecting";
        break;
    case CalibrationState::Calibrating:
        result.state = "Calibrating";
        break;
    case CalibrationState::Completed:
        result.state = "Completed";
        break;
    case CalibrationState::Error:
        result.state = "Error";
        break;
    }

    const auto syncStats = synchronizer.getStatistics();

    result.pairedFrames = syncStats.pairedFrames;
    result.droppedLeftFrames = syncStats.droppedLeftFrames;
    result.droppedRightFrames = syncStats.droppedRightFrames;
    
    if(syncStats.pairedFrames > 0)
    {
        result.meanTimestampDeltaMs = 1000.0 * syncStats.accumulatedTimeStampDelta / static_cast<double>(syncStats.pairedFrames);
    }

    result.maxTimestampDeltaMs = 1000.0 * syncStats.maxTimeStampDelta;

    result.lastCalibrationError = _calibrationError;
    if(calibrationState.load() == CalibrationState::Completed && _calibrationResults.isValid())
    {
        result.calibrationAvailable = true;
        switch(_calibrationResults.mode)
        {
        case stereo_calib::CalibrationMode::MonocularLeft:
            result.calibrationMode = "MonocularLeft";
            break;
        case stereo_calib::CalibrationMode::MonocularRight:
            result.calibrationMode = "MonocularRight";
            break;
        case stereo_calib::CalibrationMode::MonocularBoth:
            result.calibrationMode = "MonocularBoth";
            break;
        case stereo_calib::CalibrationMode::StereoFull:
            result.calibrationMode = "StereoFull";
            break;
        }

        if(_calibrationResults.leftCamera.isValid())
        {
            result.leftMonocularRms = _calibrationResults.leftCamera.rms;
        }
        if(_calibrationResults.rightCamera.isValid())
        {
            result.rightMonocularRms = _calibrationResults.rightCamera.rms;
        }
        if(_calibrationResults.stereo.isValid())
        {
            result.stereoRms = _calibrationResults.stereo.rms;
            result.baselineNorm = _calibrationResults.quality.baseline;
        }
    }

    return result;
}

bool stereoCalibThread::shouldQueueFrameForCollection(const Stamp& timestamp) const
{
    return lastProcessedCandidateTime < 0.0 ||
           (timestamp.getTime() - lastProcessedCandidateTime) >= minCaptureIntervalSeconds;
}

void stereoCalibThread::stereoCalibRun()
{
    Size boardSize;
    boardSize.width=this->boardWidth;
    boardSize.height=this->boardHeight;

    while (!isStopping()) 
    {
        // Keep the worker alive for RPC status inspection after a calibration
        // failure, but stop consuming/processing camera data until a new start
        // request changes the state back to Collecting.
        if(calibrationState.load() == CalibrationState::Error)
        {
            Time::delay(0.1);
            continue;
        }

        if(collectionResetRequested.exchange(false)) 
        {
            synchronizer.reset();
            lastProcessedCandidateTime = -1.0;
            _expectedImageSize = {};
            _observations.clear();
            _rejectedDetections = 0;
            {
                std::lock_guard<std::mutex> lock(mtx);
                _calibrationResults = stereo_calib::CalibrationResult{};
                _calibrationError.clear();
            }
        }

        bool areFramesReceived = false;
        ImageOf<PixelRgb> *tmpL = imagePortInLeft.read(false);

        if(tmpL != nullptr) 
        {
            areFramesReceived = true;
            
            Stamp TSLeft;
            imagePortInLeft.getEnvelope(TSLeft);

            // Always publish preview immediately
            ImageOf<PixelRgb>& outimL = outPortLeft.prepare();
            outimL = *tmpL;
            outPortLeft.setEnvelope(TSLeft);
            outPortLeft.write();

            // Only enqueue a separate copy while collecting
            if(calibrationState.load() == CalibrationState::Collecting &&
               shouldQueueFrameForCollection(TSLeft))
            {
                synchronizer.pushLeft(*tmpL, TSLeft); // this is done for not consuming memory bandwidth for data that will never be used by the calibration
            }
        }
        
        ImageOf<PixelRgb> *tmpR = imagePortInRight.read(false);
        if(tmpR!=nullptr)
        {
            areFramesReceived = true;

            Stamp TSRight;
            imagePortInRight.getEnvelope(TSRight);

            // Always publish preview immediately
            ImageOf<PixelRgb>& outimR=outPortRight.prepare();
            outimR = *tmpR;
            outPortRight.setEnvelope(TSRight);
            outPortRight.write();
            

            // Only enqueue a separate copy while collecting
            if(calibrationState.load() == CalibrationState::Collecting &&
               shouldQueueFrameForCollection(TSRight))
            {
                synchronizer.pushRight(*tmpR, TSRight);
            }
        }

        std::vector<stereo_calib::StereoObservation> observationSnapshot;
        
        if(calibrationState.load() == CalibrationState::Collecting) 
        {
            SynchronizedPair pair;
            while(calibrationState.load() == CalibrationState::Collecting &&
                synchronizer.tryPopPair(pair)) 
            {
                // Process the synchronized pair
                processSynchronizedPair(pair, boardSize);

                if(calibrationState.load() != CalibrationState::Collecting)
                {
                    break;
                }
                if(_observations.size() >= static_cast<std::size_t>(numOfPairs))
                {
                    yCInfo(STEREOCALIBRATIONTHREAD, "Collected %zu valid stereo observations. Stopping collection.", _observations.size());
                    observationSnapshot = _observations;
                    calibrationState.store(CalibrationState::Calibrating);
                    yCInfo(STEREOCALIBRATIONTHREAD) << "Observation collection complete";
                    break;
                }
            }
        }

        if(calibrationState.load() == CalibrationState::Calibrating)
        {
            yCInfo(STEREOCALIBRATIONTHREAD) << "Starting the calibration process";
            if(observationSnapshot.empty())
            {
                observationSnapshot = _observations;
            }

            stereo_calib::CalibrationResult calibrationResult;
            std::string calibrationError;
            bool success = false;
            if (_cameraModel == stereo_calib::CameraModel::Pinhole)
            {
                stereo_calib::PinholeCalibrationOptions options = _pinholeCalibrationOptions;
                options.common.calibrationMode = _calibrationMode;
                options.common.imageSize = observationSnapshot.front().imageSize; 
                success = _pinholeCalibrationEngine.calibrate(
                    observationSnapshot,
                    options,
                    calibrationResult,
                    calibrationError
                );
            }
            else if(_cameraModel == stereo_calib::CameraModel::Fisheye)
            {
                stereo_calib::FisheyeCalibrationOptions options = _fisheyeCalibrationOptions;
                options.common.calibrationMode = _calibrationMode;
                options.common.imageSize = observationSnapshot.front().imageSize;
                options.cameraFocalLengthGuess = 625.0;
                success = _fisheyeCalibrationEngine.calibrate(
                    observationSnapshot,
                    options,
                    calibrationResult,
                    calibrationError
                );
            }
            else
            {
                calibrationError = "Unsupported camera model.";
                success = false;
            }
            
            // success and writer should be common between fisheye and pinhole engines, so this block can be shared
            if(!success)
            {
                // The engine converts OpenCV exceptions into a diagnostic.  Log
                // it here, where YARP logging is allowed, and stop calibration
                // processing while keeping the Error state and status available.
                yCError(STEREOCALIBRATIONTHREAD) << "Calibration failed:" << calibrationError;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    _calibrationResults = stereo_calib::CalibrationResult{};
                    _calibrationError = calibrationError.empty()
                        ? "Calibration failed without an error message."
                        : calibrationError;
                }
                calibrationState.store(CalibrationState::Error);
                continue;
            }

            const auto syncStats = synchronizer.getStatistics();
            calibrationResult.quality.rejectedDetections = _rejectedDetections;
            calibrationResult.quality.acceptedObservations = observationSnapshot.size();
            calibrationResult.quality.synchronizedPairs = syncStats.pairedFrames;
            if(syncStats.pairedFrames > 0)
            {
                calibrationResult.quality.meanTimestampDeltaMs = 1000.0 * syncStats.accumulatedTimeStampDelta / static_cast<double>(syncStats.pairedFrames);
            }
            calibrationResult.quality.maxTimestampDeltaMs = syncStats.maxTimeStampDelta;
            calibrationResult.quality.baseline = cv::norm(calibrationResult.stereo.T);

            std::string persistenceError;
            if(!_calibrationWriter.write(camCalibFile, calibrationResult, persistenceError))
            {
                yCError(STEREOCALIBRATIONTHREAD) << "Could not save calibration results:" << persistenceError;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    _calibrationResults = std::move(calibrationResult);
                    _calibrationError = persistenceError;
                }
                calibrationState.store(CalibrationState::Error);
                continue;
            }
            if(!_calibrationWriter.writeObservations(_observationsFile, observationSnapshot, persistenceError))
            {
                yCError(STEREOCALIBRATIONTHREAD) << "Could not save calibration observations:" << persistenceError;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    _calibrationResults = std::move(calibrationResult);
                    _calibrationError = persistenceError;
                }
                calibrationState.store(CalibrationState::Error);
                continue;
            }

            logCalibrationResult(calibrationResult);
            {
                std::lock_guard<std::mutex> lock(mtx);
                _calibrationResults = std::move(calibrationResult);
                _calibrationError.clear();
            }

            calibrationState.store(CalibrationState::Completed);
            yCInfo(STEREOCALIBRATIONTHREAD) << "Entire calibration process completed";
        }

        if(!areFramesReceived) 
        {
            Time::delay(0.001); // Sleep for a short duration to avoid busy waiting
        }
        cout.flush();
   }
}

void stereoCalibThread::threadRelease()
{
    imagePortInRight.close();
    imagePortInLeft.close();
    outPortLeft.close();
    outPortRight.close();
    commandPort->close();

    if (polyHead.isValid())
        polyHead.close();

    if (polyTorso.isValid())
        polyTorso.close();
}

void stereoCalibThread::onStop() {
    // TODO: the following 2 should not ne necessary since we already store their state in stop()
    calibrationState.store(CalibrationState::Idle);
    collectionResetRequested.store(false);
    imagePortInRight.interrupt();
    imagePortInLeft.interrupt();
    outPortLeft.interrupt();
    outPortRight.interrupt();
    commandPort->interrupt();

}
void stereoCalibThread::startCalib() {

    std::lock_guard<std::mutex> lock(mtx);
    
    const CalibrationState currentState = calibrationState.load();

    if(currentState == CalibrationState::Collecting || currentState == CalibrationState::Calibrating)
    {
        yCWarning(STEREOCALIBRATIONTHREAD) << "Cannot start a new calibration while calibration is already running";
        return;
    }

    _calibrationResults = stereo_calib::CalibrationResult{};
    _calibrationError.clear();
    calibrationState.store(CalibrationState::Collecting);
    collectionResetRequested.store(true);

    yCInfo(STEREOCALIBRATIONTHREAD) << "Calibration collection started";
}

void stereoCalibThread::stopCalib() {
    std::lock_guard<std::mutex> lock(mtx);

    if(calibrationState.load() != CalibrationState::Collecting)
    {
        yCWarning(STEREOCALIBRATIONTHREAD) << "Cannot stop calibration collection when it is not running";
        return;
    }
    calibrationState.store(CalibrationState::Idle);
    collectionResetRequested.store(true);

    yCInfo(STEREOCALIBRATIONTHREAD) << "Calibration collection stopped";
}
