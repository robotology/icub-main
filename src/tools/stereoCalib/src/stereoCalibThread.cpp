#include <algorithm>
#include <cmath>
#include <utility>
#include <chrono>
#include <thread>
#include <sstream>
#include <yarp/cv/Cv.h>
#include "stereoCalibThread.h"

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
    yInfo() << "[CAMERA_CALIBRATION_" << cameraName << "]";
    yInfo() << "w" << camera.imageSize.width << "h" << camera.imageSize.height;
    yInfo() << "fx" << camera.K.at<double>(0, 0)
            << "fy" << camera.K.at<double>(1, 1)
            << "cx" << camera.K.at<double>(0, 2)
            << "cy" << camera.K.at<double>(1, 2);
    yInfo() << "D = [k1 k2 k3 k4] =" << formatCalibrationMatrix(camera.D.t());
    yInfo() << "K =" << formatCalibrationMatrix(camera.K);
    yInfo() << "monocular RMS =" << camera.rms;
}
 
void logCalibrationResult(const stereo_calib::CalibrationResult& result)
{
    yInfo() << "========== Fisheye calibration result (not written to disk) ==========";
 
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
 
        yInfo() << "[STEREO_DISPARITY]";
        yInfo() << "Stereo RMS =" << result.stereo.rms
                << "baseline norm =" << cv::norm(result.stereo.T);
        yInfo() << "R =" << formatCalibrationMatrix(result.stereo.R);
        yInfo() << "T =" << formatCalibrationMatrix(result.stereo.T.t());
        // HN is the same homogeneous transform layout used by outputCalib.ini.
        yInfo() << "HN =" << formatCalibrationMatrix(homogeneousTransform);
    }
 
    if(result.rectification.isValid())
    {
        yInfo() << "R1 =" << formatCalibrationMatrix(result.rectification.R1);
        yInfo() << "R2 =" << formatCalibrationMatrix(result.rectification.R2);
        yInfo() << "P1 =" << formatCalibrationMatrix(result.rectification.P1);
        yInfo() << "P2 =" << formatCalibrationMatrix(result.rectification.P2);
        yInfo() << "Q =" << formatCalibrationMatrix(result.rectification.Q);
    }
 
    if(result.mode == stereo_calib::CalibrationMode::StereoFull)
    {
        yInfo() << "Rectification vertical error [mean median RMS p95 max] ="
                << result.quality.meanVerticalRectificationErrorPx
                << result.quality.medianVerticalRectificationErrorPx
                << result.quality.rmsVerticalRectificationErrorPx
                << result.quality.p95VerticalRectificationErrorPx
                << result.quality.maxVerticalRectificationErrorPx;
    }
    yInfo() << "======================================================================";
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
        ++stats.droppedLeftFrames;
    }
}

void StereoPairSynchronizer::trimRightQueue()
{
    while(rightQueue.size() > maxQueueSize) 
    {
        rightQueue.pop_front();
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

            ++stats.pairedFrames;
            stats.accumulatedTimeStampDelta += absoluteDelta;
            stats.maxTimeStampDelta = std::max(stats.maxTimeStampDelta, absoluteDelta);

            return true;
        }

        if(leftStamp < rightStamp) 
        {
            // the oldest left frame cannot be paired with this or any newer right frame, so drop it
            leftQueue.pop_front();
            ++stats.droppedLeftFrames;
        } 
        else 
        {
            rightQueue.pop_front();
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
    this->boardWidth =  stereoCalibOpts.check("boardWidth", Value(8)).asInt32();
    this->boardHeight= stereoCalibOpts.check("boardHeight", Value(6)).asInt32();
    this->numOfPairs= stereoCalibOpts.check("numberOfPairs", Value(30)).asInt32();
    if(this->numOfPairs < 3)
    {
        yWarning() << "numberOfPairs must be at least 3; using 3";
        this->numOfPairs = 3;
    }
    this->squareSize= (float)stereoCalibOpts.check("boardSize", Value(0.09241)).asFloat64();
    this->boardType=  stereoCalibOpts.check("boardType", Value("CHESSBOARD")).asString();
    const double syncToleranceMs = stereoCalibOpts.check("syncToleranceMs", Value(20.0)).asFloat64();
    _syncToleranceSeconds = syncToleranceMs / 1000.0;
    const int configuredQueueSize = stereoCalibOpts.check("syncQueueSize", Value(5)).asInt32();
    if(configuredQueueSize <= 0)
    {
        yWarning() << "Invalid syncQueueSize; using 5";
        _syncQueueSize = 5;
    }
    else
    {
        _syncQueueSize = static_cast<std::size_t>(configuredQueueSize);
    }

    if(_syncToleranceSeconds <= 0.0)
    {
        yWarning() << "Invalid syncToleranceMs; using 20 ms";
        _syncToleranceSeconds = 0.020;
    }

    synchronizer.configure(_syncToleranceSeconds, _syncQueueSize);

    this->commandPort=commPort;
    this->imageDir=imageDir;
    this->collectionResetRequested.store(false);
    this->calibrationState.store(CalibrationState::Idle);
    this->currentPathDir=rf.getHomeContextPath().c_str();
    const bool legacyMonoRequested =
        stereoCalibOpts.check("MonoCalib", Value(0)).asInt32() != 0;
    // All new calibration modes use the synchronized-observation pipeline.
    // In particular, completion must never bypass CalibrationWriter.
    this->stereo = true;
    this->camCalibFile=rf.getHomeContextPath().c_str();
    this->standalone = rf.check("standalone");
    string fileName= "outputCalib.ini"; //rf.find("from").asString().c_str();

    this->camCalibFile=this->camCalibFile+"/"+fileName.c_str();

    _observationsFile = stereoCalibOpts.check(
        "observationsFile", Value("calibrationObservations.yml")).asString();
    if(!_observationsFile.empty() && _observationsFile.front() != '/')
    {
        _observationsFile = currentPathDir + "/" + _observationsFile;
    }


    _chessboardConfiguration.cornersX = this->boardWidth;
    _chessboardConfiguration.cornersY = this->boardHeight;

    _chessboardConfiguration.squareSizeMeters = this->squareSize;

    _saveImages = stereoCalibOpts.check("saveImages", Value(0)).asInt32() != 0;
    _drawDiagnosticCorners = stereoCalibOpts.check("drawDiagnosticCorners", Value(1)).asInt32() != 0;

    const std::string configuredMode = stereoCalibOpts.check("calibrationMode", Value("")).asString();
    if(configuredMode == "MonocularLeft")
    {
        _calibrationOptions.calibrationMode = stereo_calib::CalibrationMode::MonocularLeft;
    }
    else if(configuredMode == "MonocularRight")
    {
        _calibrationOptions.calibrationMode = stereo_calib::CalibrationMode::MonocularRight;
    }
    else if(configuredMode == "MonocularBoth")
    {
        _calibrationOptions.calibrationMode = stereo_calib::CalibrationMode::MonocularBoth;
    }
    else if(configuredMode.empty() && legacyMonoRequested)
    {
        yWarning() << "MonoCalib is deprecated; using MonocularLeft with the synchronized observation pipeline.";
        _calibrationOptions.calibrationMode = stereo_calib::CalibrationMode::MonocularLeft;
    }
    else if(configuredMode.empty() || configuredMode == "StereoFull")
    {
        _calibrationOptions.calibrationMode = stereo_calib::CalibrationMode::StereoFull;
    }
    else
    {
        yWarning() << "Unknown calibrationMode; using StereoFull:" << configuredMode;
        _calibrationOptions.calibrationMode = stereo_calib::CalibrationMode::StereoFull;
    }

    if(!_chessboardConfiguration.isValid())
    {
        yError() << "Invalid chessboard configuration";
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

    //mono calibration does not need the joint positions initialised below
    if(!stereo || standalone) return true;

    // Property optHead;
    // optHead.put("device","remote_controlboard");
    // optHead.put("remote",("/"+robotName+"/head").c_str());
    // optHead.put("local","/"+moduleName+"/client/head");
    // if (polyHead.open(optHead))
    //     polyHead.view(posHead);
    // else
    // {
    //     cout<<"Devices not available"<<endl;
    //     return false;
    // }

    // Property optTorso;
    // optTorso.put("device","remote_controlboard");
    // optTorso.put("remote",("/"+robotName+"/torso").c_str());
    // optTorso.put("local","/"+moduleName+"/client/torso");

    // bool useTorso=true;
    // if (polyTorso.open(optTorso))
    //     polyTorso.view(posTorso);
    // else
    // {
    //     yWarning("Unable to connect to torso! Continuing without...");
    //     useTorso=false;
    // }

    // yarp::sig::Vector head_angles(6,0.0);
    // posHead->getEncoders(head_angles.data());

    // yarp::sig::Vector torso_angles(3,0.0);
    // if (useTorso)
    //     posTorso->getEncoders(torso_angles.data());

    // qL.resize(torso_angles.length()+head_angles.length()-1);
    // for(size_t i=0; i<torso_angles.length(); i++)
    //     qL[i]=torso_angles[torso_angles.length()-i-1];

    // for(size_t i=0; i<head_angles.length()-2; i++)
    //     qL[i+torso_angles.length()]=head_angles[i];
    // qL[7]=head_angles[4]+(0.5-(LEFT))*head_angles[5];
    // qL=iCub::ctrl::CTRL_DEG2RAD*qL;

    // qR.resize(torso_angles.length()+head_angles.length()-1);
    // for(size_t i=0; i<torso_angles.length(); i++)
    //     qR[i]=torso_angles[torso_angles.length()-i-1];

    // for(size_t i=0; i<head_angles.length()-2; i++)
    //     qR[i+torso_angles.length()]=head_angles[i];
    // qR[7]=head_angles[4]+(0.5-(RIGHT))*head_angles[5];
    // qR=iCub::ctrl::CTRL_DEG2RAD*qR;

    return true;
}
void stereoCalibThread::run(){
    yInfo("Running synchronized fisheye calibration pipeline... \n");
    stereoCalibRun();
}

void stereoCalibThread::processSynchronizedPair(SynchronizedPair& pair, Size boardSize)
{
    // Check minimum capture interval
    const double pairTime = 0.5 * (pair.leftStamp.getTime() + pair.rightStamp.getTime()); //average pair timestamp
    const double previousProcessedCandidateTime = lastProcessedCandidateTime;
    if(previousProcessedCandidateTime >= 0.0 && (pairTime - previousProcessedCandidateTime) < minCaptureIntervalSeconds)
    {
        yDebug() << "Skipping candidate pair due to minimum capture interval";
        return;
    }
    if(previousProcessedCandidateTime >= 0.0)
    {
        yDebug() << "Timestamp delta between processed pairs:" << (pairTime - previousProcessedCandidateTime) << "seconds";
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
        yError() << "Left and right images have different sizes:" <<
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

    if(boardType == "CIRCLES_GRID") {
        foundL = findCirclesGrid(LeftRgb, boardSize, leftCorners, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
        foundR = findCirclesGrid(RightRgb, boardSize, rightCorners, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
    } else if(boardType == "ASYMMETRIC_CIRCLES_GRID") {
        foundL = findCirclesGrid(LeftRgb, boardSize, leftCorners, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
        foundR = findCirclesGrid(RightRgb, boardSize, rightCorners, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
    } else if(boardType == "CHESSBOARD_SECTOR_BASED") {
        foundL = findChessboardCornersSB(leftGray, boardSize, leftCorners);
        foundR = findChessboardCornersSB(rightGray, boardSize, rightCorners);
    } else {
        foundL = findChessboardCorners(leftGray, boardSize, leftCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
        foundR = findChessboardCorners(rightGray, boardSize, rightCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
    }

    if(foundL && foundR) 
    {
        yDebug() << "Found chessboard corners in both left and right images";

        const Rect leftBoardBounds = boundingRect(leftCorners);
        const Rect rightBoardBounds = boundingRect(rightCorners);
        const double minimumBoardSpanRatio = 0.15;
        const bool leftBoardTooSmall =
            leftBoardBounds.width < leftSize.width * minimumBoardSpanRatio ||
            leftBoardBounds.height < leftSize.height * minimumBoardSpanRatio;
        const bool rightBoardTooSmall =
            rightBoardBounds.width < rightSize.width * minimumBoardSpanRatio ||
            rightBoardBounds.height < rightSize.height * minimumBoardSpanRatio;

        if(leftBoardTooSmall || rightBoardTooSmall)
        {
            yWarning() << "Skipping stereo pair: chessboard is too small."
                       << "Left board:" << leftBoardBounds.width << "x" << leftBoardBounds.height
                       << "of" << leftSize.width << "x" << leftSize.height << ";"
                       << "right board:" << rightBoardBounds.width << "x" << rightBoardBounds.height
                       << "of" << rightSize.width << "x" << rightSize.height << "."
                       << "Each board must span at least" << (minimumBoardSpanRatio * 100.0)
                       << "% of both image dimensions.";
            std::lock_guard<std::mutex> lock(mtx);
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
            yError() << "Generated invalid stereo observation";
            std::lock_guard<std::mutex> lock(mtx);
            ++_rejectedDetections;
            return;
        }

        std::size_t observationIndex = 0;
        {
            std::lock_guard<std::mutex> lock(mtx);
            observationIndex = _observations.size();
        }

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
                yError() << "Could not save accepted calibration image pair:" << imageError;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    _calibrationError = imageError;
                    _calibrationResults = stereo_calib::CalibrationResult{};
                }
                calibrationState.store(CalibrationState::Error);
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
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
        std::lock_guard<std::mutex> lock(mtx);
        ++_rejectedDetections;
    }
    /**
    if(_observations.size() > numOfPairs) {
        yInfo(" Running Left Camera Calibration... \n");
        yDebug(" Number of images used for calibration: %ld \n",imageListL.size());
        double leftRms = monoCalibration(imageListL,this->boardWidth,this->boardHeight,this->Kleft,this->DistL,"left");

        yDebug(" Number of images used for calibration: %ld \n",imageListR.size());
        yInfo(" Running Right Camera Calibration... \n");
        double rightRms = monoCalibration(imageListR,this->boardWidth,this->boardHeight,this->Kright,this->DistR,"right");
        if (leftRms > 0.0 && rightRms > 0.0)
        {
            double rmsDelta = fabs(leftRms - rightRms);
            yInfo("Mono calibration RMS delta between left and right cameras: %g", rmsDelta);
        }

        yInfo(" Starting Stereo Calibration... \n");
        stereoCalibration(imageListLR, this->boardWidth,this->boardHeight,this->squareSize);

        yInfo(" Saving Calibration Results... \n");
        updateIntrinsics(RightRgb.cols,RightRgb.rows,Kright.at<double>(0,0),Kright.at<double>(1,1),Kright.at<double>(0,2),Kright.at<double>(1,2),DistR.at<double>(0,0),DistR.at<double>(0,1),DistR.at<double>(0,2),DistR.at<double>(0,3),"CAMERA_CALIBRATION_RIGHT");
        updateIntrinsics(LeftRgb.cols,LeftRgb.rows,Kleft.at<double>(0,0),Kleft.at<double>(1,1),Kleft.at<double>(0,2),Kleft.at<double>(1,2),DistL.at<double>(0,0),DistL.at<double>(0,1),DistL.at<double>(0,2),DistL.at<double>(0,3),"CAMERA_CALIBRATION_LEFT");

        // Mat Rot=Mat::eye(3,3,CV_64FC1);
        // Mat Tr=Mat::zeros(3,1,CV_64FC1);

        updateExtrinsics(this->R,this->T,"STEREO_DISPARITY");

        yInfo("Calibration Results Saved in %s \n", camCalibFile.c_str());

        calibrationState.store(CalibrationState::Completed);
        count=1;
        imageListR.clear();
        imageListL.clear();
        imageListLR.clear();
    }
    */
   return;
}

StereoCalibStatus stereoCalibThread::getStatus() const
{
    StereoCalibStatus result;
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

    std::lock_guard<std::mutex> lock(mtx);
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
        if(_calibrationResults.mode == stereo_calib::CalibrationMode::StereoFull)
        {
            result.medianVerticalRectificationErrorPx =
                _calibrationResults.quality.medianVerticalRectificationErrorPx;
            result.p95VerticalRectificationErrorPx =
                _calibrationResults.quality.p95VerticalRectificationErrorPx;
            result.maxVerticalRectificationErrorPx =
                _calibrationResults.quality.maxVerticalRectificationErrorPx;
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
    Size boardSize, imageSize;
    boardSize.width=this->boardWidth;
    boardSize.height=this->boardHeight;
    int count=1;

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
            std::lock_guard<std::mutex> lock(mtx);
            _observations.clear();
            _rejectedDetections = 0;
            _calibrationResults = stereo_calib::CalibrationResult{};
            _calibrationError.clear();
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
            while(synchronizer.tryPopPair(pair)) 
            {
                // Process the synchronized pair
                processSynchronizedPair(pair, boardSize);

                std::size_t observationsCount = 0;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    observationsCount = _observations.size();
                    if(observationsCount >= static_cast<std::size_t>(numOfPairs))
                    {
                        observationSnapshot = _observations;
                    }
                }

                if(observationsCount >= static_cast<std::size_t>(numOfPairs))
                {
                    yInfo("Collected %zu valid stereo observations. Stopping collection.", observationsCount);
                    if(_saveImages)
                    {
                        //saveAcceptedPair();
                    }
                    calibrationState.store(CalibrationState::Calibrating);
                    yInfo() << "Observation collection complete";
                    break;
                }
            }
        }

        if(calibrationState.load() == CalibrationState::Calibrating)
        {
            yInfo() << "Starting the calibration process";
            if(observationSnapshot.empty())
            {
                std::lock_guard<std::mutex> lock(mtx);
                observationSnapshot = _observations;
            }
            _calibrationOptions.imageSize = observationSnapshot.front().imageSize;

            stereo_calib::CalibrationResult calibrationResult;
            std::string calibrationError;
            const bool success = _calibrationEngine.calibrate(
                observationSnapshot,
                _calibrationOptions,
                calibrationResult,
                calibrationError);
            if(!success)
            {
                // The engine converts OpenCV exceptions into a diagnostic.  Log
                // it here, where YARP logging is allowed, and stop calibration
                // processing while keeping the Error state and status available.
                yError() << "Fisheye calibration failed:" << calibrationError;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    _calibrationResults = stereo_calib::CalibrationResult{};
                    _calibrationError = calibrationError.empty()
                        ? "Fisheye calibration failed without an error message."
                        : calibrationError;
                }
                calibrationState.store(CalibrationState::Error);
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(mtx);
                calibrationResult.quality.rejectedDetections = _rejectedDetections;
            }

            std::string persistenceError;
            if(!_calibrationWriter.write(camCalibFile, calibrationResult, persistenceError))
            {
                yError() << "Could not save calibration results:" << persistenceError;
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
                yError() << "Could not save calibration observations:" << persistenceError;
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
            yInfo() << "Entire calibration process completed";
        }

        if(!areFramesReceived) 
        {
            Time::delay(0.001); // Sleep for a short duration to avoid busy waiting
        }
        cout.flush();
   }
}



void stereoCalibThread::monoCalibRun()
{
    while(imagePortInLeft.getInputCount()==0 && imagePortInRight.getInputCount()==0)
    {
        yInfo("Connect one camera.. \n");
        Time::delay(1.0);

        if(isStopping())
            return;

    }

    bool left= imagePortInLeft.getInputCount()>0?true:false;

    string cameraName;

    if(left)
        cameraName="LEFT";
    else
        cameraName="RIGHT";

    yInfo("CALIBRATING %s CAMERA \n",cameraName.c_str());


    int count=1;
    Size boardSize, imageSize;
    boardSize.width=this->boardWidth;
    boardSize.height=this->boardHeight;



    while (!isStopping()) {
       if(left)
            imageL = std::move(imagePortInLeft.read(false));
       else
            imageL = std::move(imagePortInRight.read(false));

       if(imageL!=NULL){
            bool foundL=false;
            mtx.lock();
            if(calibrationState.load() == CalibrationState::Calibrating) {

                string pathImg=imageDir;
                preparePath(pathImg.c_str(),pathL,pathR,count);
                string iml(pathL);
                LeftRgb=yarp::cv::toCvMat(*imageL);
                std::vector<Point2f> pointbufL;

                if(boardType == "CIRCLES_GRID") {
                    foundL = findCirclesGrid(LeftRgb, boardSize, pointbufL, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
                } else if(boardType == "ASYMMETRIC_CIRCLES_GRID") {
                    foundL = findCirclesGrid(LeftRgb, boardSize, pointbufL, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
                } else {
                    foundL = findChessboardCorners(LeftRgb, boardSize, pointbufL, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                }

                if(foundL) {
                        cvtColor(LeftRgb,LeftRgb,CV_RGB2BGR);
                        saveImage(pathImg.c_str(),LeftRgb,count);
                        imageListL.push_back(iml);
                        Mat cL(pointbufL);
                        drawChessboardCorners(LeftRgb, boardSize, cL, foundL);
                        count++;
                }

                if(count>numOfPairs) {
                    yInfo(" Running %s Camera Calibration... \n", cameraName.c_str());
                    monoCalibration(imageListL,this->boardWidth,this->boardHeight,this->Kleft,this->DistL,cameraName.c_str());

                    yInfo(" Saving Calibration Results... \n");
                    updateIntrinsics(LeftRgb.cols,LeftRgb.rows,Kleft.at<double>(0,0),Kleft.at<double>(1,1),Kleft.at<double>(0,2),
                                     Kleft.at<double>(1,2),DistL.at<double>(0,0),DistL.at<double>(0,1),DistL.at<double>(0,2),
                                     DistL.at<double>(0,3),left?"CAMERA_CALIBRATION_LEFT":"CAMERA_CALIBRATION_RIGHT");
                    yInfo("Calibration Results Saved in %s \n", camCalibFile.c_str());

                    calibrationState.store(CalibrationState::Completed);
                    count=1;
                    imageListL.clear();
                }
            }
            mtx.unlock();
            ImageOf<PixelRgb>& outimL=outPortLeft.prepare();
            outimL=*imageL;
            outPortLeft.write();

            ImageOf<PixelRgb>& outimR=outPortRight.prepare();
            outimR=*imageL;
            outPortRight.write();

            cout.flush();

        }
   }


 }
void stereoCalibThread::threadRelease()
{
    imagePortInRight.close();
    imagePortInLeft.close();
    outPortLeft.close();
    outPortRight.close();
    commandPort->close();

    // if (polyHead.isValid())
    //     polyHead.close();

    // if (polyTorso.isValid())
    //     polyTorso.close();
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
    const CalibrationState currentState = calibrationState.load();

    if(currentState == CalibrationState::Calibrating)
    {
        yWarning() << "Cannot start a new calibration while calibration is already running";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        _calibrationResults = stereo_calib::CalibrationResult{};
        _calibrationError.clear();
    }
    collectionResetRequested.store(true);
    calibrationState.store(CalibrationState::Collecting);

    yInfo() << "Calibration collection started";
}

void stereoCalibThread::stopCalib() {
    collectionResetRequested.store(true);
    calibrationState.store(CalibrationState::Idle);

    yInfo() << "Calibration collection stopped";
}

void stereoCalibThread::printMatrix(Mat &matrix) {
    int row = matrix.rows;
    int col = matrix.cols;
        cout << endl;
    for(int i = 0; i < matrix.rows; i++)
    {
        const double* Mi = matrix.ptr<double>(i);
        for(int j = 0; j < matrix.cols; j++)
            cout << Mi[j] << " ";
        cout << endl;
    }
        cout << endl;
}


bool stereoCalibThread::checkTS(double TSLeft, double TSRight, double th) {
    double diff = fabs(TSLeft-TSRight);
    if(diff <th)
        return true;
    else return false;

}

void stereoCalibThread::preparePath(const char * imageDir, char* pathL, char* pathR, int count) {
    char num[5];
    sprintf(num, "%i", count);


    strncpy(pathL,imageDir, strlen(imageDir));
    pathL[strlen(imageDir)]='\0';
    strcat(pathL,"left");
    strcat(pathL,num);
    strcat(pathL,".png");

    strncpy(pathR,imageDir, strlen(imageDir));
    pathR[strlen(imageDir)]='\0';
    strcat(pathR,"right");
    strcat(pathR,num);
    strcat(pathR,".png");

}


void stereoCalibThread::saveStereoImage(const char * imageDir, const Mat& left, const Mat& right, int num) {
    char pathL[256];
    char pathR[256];
    preparePath(imageDir, pathL,pathR,num);

    yInfo("Saving stereo images number %d \n",num);

    imwrite(pathL,left);
    imwrite(pathR,right);
}

void stereoCalibThread::saveImage(const char * imageDir, const Mat& left, int num) {
    char pathL[256];
    preparePath(imageDir, pathL,pathR,num);

    yInfo("Saving images number %d \n",num);

    imwrite(pathL,left);
}

bool stereoCalibThread::updateIntrinsics(int width, int height, double fx, double fy,double cx, double cy, double k1, double k2, double k3, double k4, const string& groupname){

    std::vector<string> lines;

    bool append = false;

    ifstream in;
    in.open(camCalibFile.c_str()); //camCalibFile.c_str());

    if(in.is_open()){
        // file exists
        string line;
        bool sectionFound = false;
        bool sectionClosed = false;

        // process lines
        while(std::getline(in, line)){
            // check if we left calibration section
            if (sectionFound == true && line.find("[", 0) != string::npos)
                sectionClosed = true;   // also valid if no groupname specified
            // check if we enter calibration section
            if (line.find(string("[") + groupname + string("]"), 0) != string::npos)
                sectionFound = true;
            // if no groupname specified
            if (groupname == "")
                sectionFound = true;
            // if we are in calibration section (or no section/group specified)
            if (sectionFound == true && sectionClosed == false){
                // replace w line
                if (line.find("w",0) ==0){
                    stringstream ss;
                    ss << width;
                    line = "w " + string(ss.str());
                }
                // replace h line
                if (line.find("h",0) ==0){
                    stringstream ss;
                    ss << height;
                    line = "h " + string(ss.str());
                }
                // replace fx line
                if (line.find("fx",0) != string::npos){
                    stringstream ss;
                    ss << fx;
                    line = "fx " + string(ss.str());
                }
                // replace fy line
                if (line.find("fy",0) != string::npos){
                    stringstream ss;
                    ss << fy;
                    line = "fy " + string(ss.str());
                }
                // replace cx line
                if (line.find("cx",0) != string::npos){
                    stringstream ss;
                    ss << cx;
                    line = "cx " + string(ss.str());
                }
                // replace cy line
                if (line.find("cy",0) != string::npos){
                    stringstream ss;
                    ss << cy;
                    line = "cy " + string(ss.str());
                }
                // replace k1 line
                if (line.find("k1",0) != string::npos){
                    stringstream ss;
                    ss << k1;
                    line = "k1 " + string(ss.str());
                }
                // replace k2 line
                if (line.find("k2",0) != string::npos){
                    stringstream ss;
                    ss << k2;
                    line = "k2 " + string(ss.str());
                }
                // replace k3 line
                if (line.find("k3",0) != string::npos){
                    stringstream ss;
                    ss << k3;
                    line = "k3 " + string(ss.str());
                }
                // replace k4 line
                if (line.find("k4",0) != string::npos){
                    stringstream ss;
                    ss << k4;
                    line = "k4 " + string(ss.str());
                }
            }
            // buffer line
            lines.push_back(line);
        }

        in.close();

        // rewrite file
        if (!sectionFound){
            append = true;
            cout << "Camera calibration parameter section " + string("[") + groupname + string("]") + " not found in file " << camCalibFile << ". Adding group..." << endl;
        }
        else{
            // rewrite file
            ofstream out;
            out.open(camCalibFile.c_str(), ios::trunc);
            if (out.is_open()){
                for (int i = 0; i < (int)lines.size(); i++)
                    out << lines[i] << endl;
                out.close();
            }
            else
                return false;
        }

    }
    else{
        append = true;
    }

    if (append){
        // file doesn't exist or section is appended
        ofstream out;
        out.open(camCalibFile.c_str(), ios::app);
        if (out.is_open()){
            out << string("[") + groupname + string("]") << endl;
            out << endl;
            out << "w  " << width << endl;
            out << "h  " << height << endl;
            out << "fx " << fx << endl;
            out << "fy " << fy << endl;
            out << "cx " << cx << endl;
            out << "cy " << cy << endl;
            out << "k1 " << k1 << endl;
            out << "k2 " << k2 << endl;
            out << "k3 " << k3 << endl;
            out << "k4 " << k4 << endl;
            out << endl;
            out.close();
        }
        else
            return false;
    }

    return true;
}

double stereoCalibThread::monoCalibration(const vector<string>& imageList, int boardWidth, int boardHeight, Mat &K, Mat &Dist, const char* cameraName)
{
    vector<vector<Point2f> > imagePoints;
    Size boardSize, imageSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    int flags=0;
    int i;

    float squareSize = this->squareSize;
    float aspectRatio = 1.f;
    if (squareSize <= 0.0f)
    {
        yWarning("Mono calibration: invalid square size %f, using 1.0", squareSize);
        squareSize = 1.0f;
    }

    Mat view, viewGray;

    for(i = 0; i<(int)imageList.size();i++)
    {
        view = cv::imread(imageList[i], IMREAD_COLOR);
        if (view.empty())
        {
            yWarning("Mono calibration: could not read image %s", imageList[i].c_str());
            continue;
        }

        if (imageSize == Size())
            imageSize = view.size();
        else if (view.size() != imageSize)
        {
            yWarning("Mono calibration: image %s has size %dx%d while first image had %dx%d",
                     imageList[i].c_str(), view.cols, view.rows, imageSize.width, imageSize.height);
            continue;
        }

        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, CV_BGR2GRAY);

        bool found = false;
        if(boardType == "CIRCLES_GRID") {
            found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
        } else if(boardType == "ASYMMETRIC_CIRCLES_GRID") {
            found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
        } else {
            found = findChessboardCorners(viewGray, boardSize, pointbuf,
                                        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        }

        if(found)
        {
            if (pointbuf.size() != static_cast<size_t>(boardWidth * boardHeight))
            {
                yWarning("Mono calibration: skipping image %s because %zu corners were detected, expected %d",
                         imageList[i].c_str(), pointbuf.size(), boardWidth * boardHeight);
                continue;
            }

            Rect bbox = boundingRect(pointbuf);
            if (bbox.width < view.cols * 0.15 || bbox.height < view.rows * 0.15)
            {
                yWarning("Mono calibration: skipping image %s because the detected board is too small (bbox %dx%d) with respect to the image size (%dx%d)",
                         imageList[i].c_str(), bbox.width, bbox.height, view.cols, view.rows);
                continue;
            }

            TermCriteria subpixCriteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001);
            cornerSubPix(viewGray, pointbuf, Size(7, 7), Size(-1, -1), subpixCriteria);
            drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
            imagePoints.push_back(pointbuf);
        }
        else
        {
            yWarning("Mono calibration: no chessboard detected in %s", imageList[i].c_str());
        }
    }

    if (imagePoints.size() < 2)
    {
        yError("Mono calibration: only %zu valid image(s) remain after filtering; aborting", imagePoints.size());
        return 0.0;
    }

    std::vector<Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    std::vector<std::vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    K = Mat::eye(3, 3, CV_64F);
    Dist = Mat::zeros(4, 1, CV_64F);

    bool usedPinholeGuess = false;
    try
    {
        Mat pinholeK = Mat::eye(3, 3, CV_64F);
        Mat pinholeDist = Mat::zeros(5, 1, CV_64F);
        std::vector<Mat> pinholeRvecs, pinholeTvecs;
        int pinholeFlags = cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_TANGENT_DIST;
        double pinholeRms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, pinholeK, pinholeDist,
                                               pinholeRvecs, pinholeTvecs, pinholeFlags);
        yInfo("Pinhole initial guess RMS for %s camera: %g", cameraName, pinholeRms);
        if (pinholeK.rows == 3 && pinholeK.cols == 3)
        {
            K = pinholeK.clone();
            usedPinholeGuess = true;
        }
    }
    catch (const cv::Exception& e)
    {
        yWarning("Pinhole initial guess failed for %s camera: %s", cameraName, e.what());
    }

    if (!usedPinholeGuess)
    {
        double focal = std::max(imageSize.width, imageSize.height) * 0.8;
        K = Mat::eye(3, 3, CV_64F);
        K.at<double>(0,0) = focal;
        K.at<double>(1,1) = focal;
        K.at<double>(0,2) = imageSize.width * 0.5;
        K.at<double>(1,2) = imageSize.height * 0.5;
    }
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        K.at<double>(0,0) = aspectRatio;

    try
    {
        int calFlags = fisheye::CALIB_USE_INTRINSIC_GUESS | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_SKEW;
        double rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, K, Dist, rvecs, tvecs,
                        calFlags,
                        TermCriteria(TermCriteria::EPS+TermCriteria::MAX_ITER, 100, 1e-5));

        yInfo("Mono calibration RMS for %s camera: %g", cameraName, rms);
        yInfo("Mono calibration intrinsics for %s camera: fx=%g fy=%g cx=%g cy=%g",
              cameraName, K.at<double>(0,0), K.at<double>(1,1), K.at<double>(0,2), K.at<double>(1,2));
        yInfo("Mono calibration distortion for %s camera: k1=%g k2=%g k3=%g k4=%g",
              cameraName, Dist.at<double>(0,0), Dist.at<double>(1,0), Dist.at<double>(2,0), Dist.at<double>(3,0));
        cout.flush();
        return rms;
    }
    catch (const cv::Exception& e)
    {
        yError("OpenCV mono calibration raised an exception: %s", e.what());
        yError("Mono calibration failed with %zu valid images", imagePoints.size());
        throw;
    }
}


namespace {
void logStereoCalibrationDebugInfo(const std::vector<std::vector<cv::Point2f> >& imagePointsLeft,
                                  const std::vector<std::vector<cv::Point2f> >& imagePointsRight,
                                  const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                  const std::vector<std::string>& imagelist,
                                  const cv::Size& imageSize,
                                  int boardWidth,
                                  int boardHeight)
{
    yInfo("Stereo calibration debug: %zu image pairs, board %dx%d, image size %dx%d",
          imagePointsLeft.size(), boardWidth, boardHeight, imageSize.width, imageSize.height);

    for (size_t i = 0; i < imagePointsLeft.size(); ++i)
    {
        yInfo("pair[%zu]: left corners=%zu right corners=%zu object points=%zu", i,
              imagePointsLeft[i].size(), imagePointsRight[i].size(), objectPoints[i].size());
        if (imagePointsLeft[i].size() != imagePointsRight[i].size())
        {
            yError("pair[%zu] has mismatched corner counts: left=%zu right=%zu", i,
                   imagePointsLeft[i].size(), imagePointsRight[i].size());
        }
        if (imagePointsLeft[i].size() != objectPoints[i].size())
        {
            yError("pair[%zu] has mismatched corners/object-points: corners=%zu object=%zu", i,
                   imagePointsLeft[i].size(), objectPoints[i].size());
        }
        if (i < imagelist.size() / 2)
        {
            yInfo("pair[%zu] images: %s / %s", i, imagelist[2 * i].c_str(), imagelist[2 * i + 1].c_str());
        }
    }
}
}

void stereoCalibThread::stereoCalibration(const vector<string>& imagelist, int boardWidth, int boardHeight,float sqsize)
{
    Size boardSize;
    boardSize.width=boardWidth;
    boardSize.height=boardHeight;
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    std::vector<std::vector<Point2f> > imagePoints[2];
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    std::vector<string> goodImageList;
    bool differentSizes = false;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = cv::imread(filename, IMREAD_GRAYSCALE);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                yWarning() <<"The image " << filename << " has the size different from the first image size.\n";
                differentSizes = true;
            }
            bool found = false;
            std::vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);

                if(boardType == "CIRCLES_GRID") {
                    found = findCirclesGrid(timg, boardSize, corners, CALIB_CB_SYMMETRIC_GRID  | CALIB_CB_CLUSTERING);
                } else if(boardType == "ASYMMETRIC_CIRCLES_GRID") {
                    found = findCirclesGrid(timg, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
                } else {
                    found = findChessboardCorners(timg, boardSize, corners,
                                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                }

                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( !found )
                break;
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    yInfo("%i pairs have been successfully detected.\n",j);
    nimages = j;
    if( nimages < 2 )
    {
        yError("Error: too few pairs detected \n");
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);

    std::vector<std::vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);
    objectPoints.resize(nimages, objectPoints[0]);

    yInfo("Running stereo calibration ...\n");

    //logStereoCalibrationDebugInfo(imagePoints[0], imagePoints[1], objectPoints, imagelist, imageSize, boardWidth, boardHeight);

    Mat cameraMatrix[2], distCoeffs[2];
    Mat E, F;
    int flags = fisheye::CALIB_FIX_INTRINSIC | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_SKEW;
    TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 100, 1e-5);

    yInfo("Stereo calibration intrinsics: left empty=%d right empty=%d", this->Kleft.empty(), this->Kright.empty());
    yInfo("Stereo calibration distortion: left empty=%d right empty=%d", this->DistL.empty(), this->DistR.empty());

    if (this->Kleft.empty() || this->Kright.empty())
    {
        if (differentSizes){
            yError("Images have different sizes. Please make sure to compute intrinsic parameters before running stereo calibration. Quitting...");
            exit (-1);
        }
        yError("Stereo calibration: intrinsics are empty; cannot proceed with fixed-intrinsic stereo solve.");
        return;
    }

    this->R = Mat::eye(3, 3, CV_64F);
    this->T = Mat::zeros(3, 1, CV_64F);
    this->T.at<double>(0, 0) = 0.06;
    yInfo("Stereo calibration initial guess: R=identity, T=(%.4f, %.4f, %.4f)", this->T.at<double>(0,0), this->T.at<double>(1,0), this->T.at<double>(2,0));

    // store image size for later saving of rectification matrices
    this->lastImageSize = imageSize;

    try
    {
        yInfo("Using precomputed intrinsic parameters with fixed intrinsics and a baseline-based initial translation");
        double rms = fisheye::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                this->Kleft, this->DistL,
                this->Kright, this->DistR,
                imageSize, this->R, this->T,
                flags, criteria);
        yInfo("done with RMS error= %f\n",rms);
    }
    catch (const cv::Exception& e)
    {
        yError("OpenCV stereo calibration raised an exception: %s", e.what());
        yError("Stereo calibration failed with %zu left pairs and %zu right pairs", imagePoints[0].size(), imagePoints[1].size());
        throw;
    }

    // Compute the fundamental matrix for the undistorted stereo pair.
    cameraMatrix[0] = this->Kleft;
    cameraMatrix[1] = this->Kright;
    distCoeffs[0] = this->DistL;
    distCoeffs[1] = this->DistR;

    Mat R, T;
    T = this->T;
    R = this->R;
    Mat Tx = Mat::zeros(3, 3, CV_64F);
    Tx.at<double>(0, 1) = -T.at<double>(2, 0);
    Tx.at<double>(0, 2) =  T.at<double>(1, 0);
    Tx.at<double>(1, 0) =  T.at<double>(2, 0);
    Tx.at<double>(1, 2) = -T.at<double>(0, 0);
    Tx.at<double>(2, 0) = -T.at<double>(1, 0);
    Tx.at<double>(2, 1) =  T.at<double>(0, 0);

    F = cameraMatrix[1].inv().t() * Tx * R * cameraMatrix[0].inv();
    yInfo("Computed fundamental matrix from stereo extrinsics.");

    double err = 0;
    int npoints = 0;
    std::vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            fisheye::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            yDebug() << "Calculated undistorted points for image " << i << " camera " << k;
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    yInfo("average reprojection err = %f\n",err/npoints);
    cout.flush();
}


void stereoCalibThread::saveCalibration(const string& extrinsicFilePath, const string& intrinsicFilePath){

    if( Kleft.empty() || Kright.empty() || DistL.empty() || DistR.empty() || R.empty() || T.empty()) {
            cout << "Error: cameras are not calibrated! Run the calibration or set intrinsic and extrinsic parameters \n";
            return;
    }

    FileStorage fs(intrinsicFilePath+".yml", cv::FileStorage::Mode::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << Kleft << "D1" << DistL << "M2" << Kright << "D2" << DistR;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    fs.open(extrinsicFilePath+".yml", cv::FileStorage::Mode::WRITE);
    if( fs.isOpened() )
    {
        // compute rectification and projection matrices for fisheye model
        try {
            Mat R1, R2, P1, P2, Qr;
            int flags = 0; // consider fisheye::CALIB_ZERO_DISPARITY if desired
            fisheye::stereoRectify(Kleft, DistL, Kright, DistR, this->lastImageSize, R, T, R1, R2, P1, P2, Qr, flags, Size());
            fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Qr;
        }
        catch (const cv::Exception &e) {
            yError("stereoRectify failed: %s", e.what());
            fs << "R" << R << "T" << T << "Q" << Q;
        }
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

}

void stereoCalibThread::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.resize(0);

    if(boardType == "ASYMMETRIC_CIRCLES_GRID") {
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
    } else {
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
    }
}

bool stereoCalibThread::updateExtrinsics(Mat Rot, Mat Tr, const string& groupname)
{
    std::vector<string> lines;
    bool append = false;

    ifstream in;
    in.open(camCalibFile.c_str()); //camCalibFile.c_str());

    if(in.is_open()){
        // file exists
        string line;
        bool sectionFound = false;
        bool sectionClosed = false;

        // process lines
        while(std::getline(in, line)){
            // check if we left calibration section
            if (sectionFound == true && line.find("[", 0) != string::npos)
                sectionClosed = true;   // also valid if no groupname specified
            // check if we enter calibration section
            if (line.find(string("[") + groupname + string("]"), 0) != string::npos)
                sectionFound = true;
            // if no groupname specified
            if (groupname == "")
                sectionFound = true;
            // if we are in calibration section (or no section/group specified)
            if (sectionFound == true && sectionClosed == false){
                // replace w line
                if (line.find("HN",0) != string::npos){
                    stringstream ss;
                    ss << " (" << Rot.at<double>(0,0) << " " << Rot.at<double>(0,1) << " " << Rot.at<double>(0,2) << " " << Tr.at<double>(0,0) << " "
                               << Rot.at<double>(1,0) << " " << Rot.at<double>(1,1) << " " << Rot.at<double>(1,2) << " " << Tr.at<double>(1,0) << " "
                               << Rot.at<double>(2,0) << " " << Rot.at<double>(2,1) << " " << Rot.at<double>(2,2) << " " << Tr.at<double>(2,0) << " "
                               << 0.0                 << " " << 0.0                 << " " << 0.0                 << " " << 1.0                << ")";
                    line = "HN" + string(ss.str());
                }

                if (!standalone) {
                    if (line.find("QL", 0) != string::npos) {
                        line = "QL (" + string(qL.toString().c_str()) + ")";
                    }
                    if (line.find("QR", 0) != string::npos) {
                        line = "QR (" + string(qR.toString().c_str()) + ")";
                    }
                }
            }
            // buffer line
            lines.push_back(line);
        }

        in.close();

        // rewrite file
        if (!sectionFound){
            append = true;
            cout << "Camera calibration parameter section " + string("[") + groupname + string("]") + " not found in file " << camCalibFile << ". Adding group..." << endl;
        }
        else{
            // rewrite file
            ofstream out;
            out.open(camCalibFile.c_str(), ios::trunc);
            if (out.is_open()){
                for (int i = 0; i < (int)lines.size(); i++)
                    out << lines[i] << endl;
                out.close();
            }
            else
                return false;
        }

    }
    else{
        append = true;
    }

    if (append){
        // file doesn't exist or section is appended
        ofstream out;
        out.open(camCalibFile.c_str(), ios::app);
        if (out.is_open()){
            out << endl;
            out << string("[") + groupname + string("]") << endl;
            out << "HN (" << Rot.at<double>(0,0) << " " << Rot.at<double>(0,1) << " " << Rot.at<double>(0,2) << " " << Tr.at<double>(0,0) << " "
                          << Rot.at<double>(1,0) << " " << Rot.at<double>(1,1) << " " << Rot.at<double>(1,2) << " " << Tr.at<double>(1,0) << " "
                          << Rot.at<double>(2,0) << " " << Rot.at<double>(2,1) << " " << Rot.at<double>(2,2) << " " << Tr.at<double>(2,0) << " "
                          << 0.0                 << " " << 0.0                 << " " << 0.0                 << " " << 1.0                << ")";
            out << endl;
            out << "QL (" << qL.toString().c_str() << ")" << endl;
            out << "QR (" << qR.toString().c_str() << ")" << endl;
            out.close();
        }
        else
            return false;
    }

    return true;
}
