#include "FisheyeCalibrationEngine.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace
{

constexpr std::size_t minimumObservations = 30;
constexpr std::size_t minimumPointsPerObservation = 12;

bool isFinite(const cv::Point2f& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool isFinite(const cv::Point3f& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace

namespace stereo_calib
{

bool FisheyeCalibrationEngine::calibrate(
    const std::vector<StereoObservation>& observations,
    const FisheyeCalibrationOptions& options,
    CalibrationResult& result,
    std::string& errorMessage) const
{
    result = CalibrationResult{};
    errorMessage.clear();

    CalibrationResult calculated;
    calculated.model = CameraModel::Fisheye;
    calculated.leftCamera.model = CameraModel::Fisheye;
    calculated.rightCamera.model = CameraModel::Fisheye;
    calculated.mode = options.common.calibrationMode;

    try
    {
        if (!options.isValid())
        {
            errorMessage = "Invalid fisheye calibration options.";
            return false;
        }

        if (!validateObservations(observations, options.common.imageSize, errorMessage))
        {
            return false;
        }

        switch (options.common.calibrationMode)
        {
        case CalibrationMode::MonocularLeft:
            if (!calibrateMonocular(observations, CameraSide::Left, options,
                                    calculated.leftCamera, errorMessage))
            {
                return false;
            }
            break;

        case CalibrationMode::MonocularRight:
            if (!calibrateMonocular(observations, CameraSide::Right, options,
                                    calculated.rightCamera, errorMessage))
            {
                return false;
            }
            break;

        case CalibrationMode::MonocularBoth:
            if (!calibrateMonocular(observations, CameraSide::Left, options,
                                    calculated.leftCamera, errorMessage) ||
                !calibrateMonocular(observations, CameraSide::Right, options,
                                    calculated.rightCamera, errorMessage))
            {
                return false;
            }
            break;

        case CalibrationMode::StereoFull:
            if (!calibrateMonocular(observations, CameraSide::Left, options,
                                    calculated.leftCamera, errorMessage) ||
                !calibrateMonocular(observations, CameraSide::Right, options,
                                    calculated.rightCamera, errorMessage) ||
                !calibrateStereo(observations, options, calculated.leftCamera,
                                 calculated.rightCamera, calculated.stereo, errorMessage))
            {
                return false;
            }
            break;

        default:
            errorMessage = "Unknown calibration mode.";
            return false;
        }

        if (!calculated.isValid())
        {
            errorMessage = "Calibration produced an invalid result.";
            return false;
        }

        result = std::move(calculated);
        return true;
    }
    catch (const cv::Exception& exception)
    {
        errorMessage = std::string("OpenCV calibration error: ") + exception.what();
    }
    catch (const std::exception& exception)
    {
        errorMessage = std::string("Calibration error: ") + exception.what();
    }

    result = CalibrationResult{};
    return false;
}

bool FisheyeCalibrationEngine::validateObservations(
    const std::vector<StereoObservation>& observations,
    const cv::Size& expectedImageSize,
    std::string& errorMessage) const
{
    if (observations.size() < minimumObservations)
    {
        errorMessage = "At least " + std::to_string(minimumObservations) +
                       " stereo observations are required.";
        return false;
    }

    std::size_t expectedPointCount = 0;
    std::vector<cv::Point3f> referenceObjectPoints;
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        const StereoObservation& observation = observations[index];
        const std::string prefix = "Observation #" + std::to_string(index + 1) + ": ";

        if (observation.imageSize != expectedImageSize)
        {
            std::ostringstream message;
            message << prefix << "image size is " << observation.imageSize.width << "x"
                    << observation.imageSize.height << ", expected " << expectedImageSize.width
                    << "x" << expectedImageSize.height << ".";
            errorMessage = message.str();
            return false;
        }

        if (!observation.isValid())
        {
            errorMessage = prefix + "has inconsistent point lists or an invalid timestamp delta.";
            return false;
        }

        if (observation.objectPoints.size() < minimumPointsPerObservation)
        {
            errorMessage = prefix + "contains fewer than " +
                           std::to_string(minimumPointsPerObservation) + " calibration points.";
            return false;
        }

        if (!std::isfinite(observation.leftTimestampSeconds) ||
            !std::isfinite(observation.rightTimestampSeconds) ||
            !std::isfinite(observation.timestampDeltaSeconds))
        {
            errorMessage = prefix + "contains a non-finite timestamp.";
            return false;
        }

        for (std::size_t pointIndex = 0; pointIndex < observation.objectPoints.size(); ++pointIndex)
        {
            if (!isFinite(observation.objectPoints[pointIndex]) ||
                !isFinite(observation.leftImagePoints[pointIndex]) ||
                !isFinite(observation.rightImagePoints[pointIndex]))
            {
                errorMessage = prefix + "point #" + std::to_string(pointIndex + 1) +
                               " contains a non-finite coordinate.";
                return false;
            }
        }

        if (index == 0)
        {
            expectedPointCount = observation.objectPoints.size();
            referenceObjectPoints = observation.objectPoints;
        }
        else if (observation.objectPoints.size() != expectedPointCount)
        {
            errorMessage = prefix + "has " + std::to_string(observation.objectPoints.size()) +
                           " object points, expected " + std::to_string(expectedPointCount) + ".";
            return false;
        }
        else
        {
            for (std::size_t pointIndex = 0; pointIndex < expectedPointCount; ++pointIndex)
            {
                if (cv::norm(observation.objectPoints[pointIndex] - referenceObjectPoints[pointIndex]) > 1e-6)
                {
                    errorMessage = prefix + "uses object points different from observation #1.";
                    return false;
                }
            }
        }
    }

    return true;
}

bool FisheyeCalibrationEngine::calibrateMonocular(
    const std::vector<StereoObservation>& observations,
    CameraSide cameraSide,
    const FisheyeCalibrationOptions& options,
    CameraCalibrationResult& result,
    std::string& errorMessage) const
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> leftImagePoints;
    std::vector<std::vector<cv::Point2f>> rightImagePoints;
    if (!extractCalibrationPoints(observations, objectPoints, leftImagePoints, rightImagePoints, errorMessage))
    {
        return false;
    }

    const std::vector<std::vector<cv::Point2f>>& imagePoints =
        cameraSide == CameraSide::Left ? leftImagePoints : rightImagePoints;
    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64F);
    intrinsic.at<double>(0, 0) = options.cameraFocalLengthGuess;
    intrinsic.at<double>(1, 1) = options.cameraFocalLengthGuess;
    intrinsic.at<double>(0, 2) = options.common.imageSize.width * 0.5;
    intrinsic.at<double>(1, 2) = options.common.imageSize.height * 0.5;
    cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;

    const double rms = cv::fisheye::calibrate(objectPoints, imagePoints, options.common.imageSize,
                                                intrinsic, distortion, rotationVectors,
                                                translationVectors, options.monocularFlags,
                                                options.criteria);
    
    result = CameraCalibrationResult{};
    result.model = CameraModel::Fisheye;
    result.imageSize = options.common.imageSize;
    result.K = intrinsic.clone();
    result.D = distortion.reshape(1, 4).clone();
    result.rotationVectors = std::move(rotationVectors);
    result.translationVectors = std::move(translationVectors);
    result.rms = rms;

    if (result.rotationVectors.size() != observations.size() ||
        result.translationVectors.size() != observations.size())
    {
        errorMessage = "OpenCV returned an incomplete set of monocular poses.";
        return false;
    }

    result.perViewRms.reserve(observations.size());
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        std::vector<cv::Point2f> projectedPoints;
        cv::fisheye::projectPoints(objectPoints[index], projectedPoints,
                                   result.rotationVectors[index], result.translationVectors[index],
                                   result.K, result.D);
        if (projectedPoints.size() != imagePoints[index].size())
        {
            errorMessage = "OpenCV returned an incomplete projected point set.";
            return false;
        }
        const double viewRms = cv::norm(projectedPoints, imagePoints[index], cv::NORM_L2) /
                               std::sqrt(static_cast<double>(projectedPoints.size()));
        result.perViewRms.push_back(viewRms);
    }

    if(!result.isValid())
    {
        errorMessage = "Monocular calibration produced invalid camera parameters.";
        return false;
    }

    if(result.rotationVectors.size() != observations.size())
    {
        errorMessage = "Monocular calibration did not produce one pose per observation.";
        return false;
    }
    return true;
}

bool FisheyeCalibrationEngine::calibrateStereo(
    const std::vector<StereoObservation>& observations,
    const FisheyeCalibrationOptions& options,
    const CameraCalibrationResult& leftCamera,
    const CameraCalibrationResult& rightCamera,
    StereoCalibrationResult& result,
    std::string& errorMessage) const
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> leftImagePoints;
    std::vector<std::vector<cv::Point2f>> rightImagePoints;
    if (!extractCalibrationPoints(observations, objectPoints, leftImagePoints, rightImagePoints, errorMessage))
    {
        return false;
    }

    cv::Mat leftK = leftCamera.K.clone();
    cv::Mat leftD = leftCamera.D.clone();
    cv::Mat rightK = rightCamera.K.clone();
    cv::Mat rightD = rightCamera.D.clone();
    cv::Mat rotation;
    cv::Mat translation;
    const double rms = cv::fisheye::stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints,
                                                      leftK, leftD, rightK, rightD, options.common.imageSize,
                                                      rotation, translation, options.stereoFlags,
                                                      options.criteria);

    result = StereoCalibrationResult{};
    result.R = rotation.clone();
    result.T = translation.reshape(1, 3).clone();
    result.rms = rms;
    if (!result.isValid())
    {
        errorMessage = "Stereo calibration produced invalid extrinsic parameters.";
        return false;
    }
    return true;
}

bool FisheyeCalibrationEngine::extractCalibrationPoints(
    const std::vector<StereoObservation>& observations,
    std::vector<std::vector<cv::Point3f>>& objectPoints,
    std::vector<std::vector<cv::Point2f>>& leftImagePoints,
    std::vector<std::vector<cv::Point2f>>& rightImagePoints,
    std::string& errorMessage) const
{
    objectPoints.clear();
    leftImagePoints.clear();
    rightImagePoints.clear();
    objectPoints.reserve(observations.size());
    leftImagePoints.reserve(observations.size());
    rightImagePoints.reserve(observations.size());

    for (const StereoObservation& observation : observations)
    {
        objectPoints.push_back(observation.objectPoints);
        leftImagePoints.push_back(observation.leftImagePoints);
        rightImagePoints.push_back(observation.rightImagePoints);
    }
    if (objectPoints.empty())
    {
        errorMessage = "No calibration points are available.";
        return false;
    }
    return true;
}

} // namespace stereo_calib
