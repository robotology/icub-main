#include "FisheyeCalibrationEngine.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace
{

constexpr std::size_t minimumObservations = 30;
constexpr std::size_t minimumPointsPerObservation = 12;

bool isFinite(const cv::Mat& matrix)
{
    return !matrix.empty() && cv::checkRange(matrix, true, nullptr);
}

bool isFinite(const cv::Point2f& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool isFinite(const cv::Point3f& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isValidCameraResult(const stereo_calib::CameraCalibrationResult& result)
{
    return result.imageSize.width > 0 && result.imageSize.height > 0 &&
           result.K.rows == 3 && result.K.cols == 3 && result.K.type() == CV_64F &&
           result.D.total() == 4 && result.D.type() == CV_64F &&
           std::isfinite(result.rms) && result.rms >= 0.0 &&
           result.K.at<double>(0, 0) > 0.0 && result.K.at<double>(1, 1) > 0.0 &&
           isFinite(result.K) && isFinite(result.D) &&
           result.rotationVectors.size() == result.translationVectors.size() &&
           result.rotationVectors.size() == result.perViewRms.size() &&
           std::all_of(result.perViewRms.begin(), result.perViewRms.end(),
                       [](double value) { return std::isfinite(value) && value >= 0.0; });
}

bool isValidStereoResult(const stereo_calib::StereoCalibrationResult& result)
{
    if (result.R.rows != 3 || result.R.cols != 3 || result.R.type() != CV_64F ||
        result.T.total() != 3 || result.T.type() != CV_64F ||
        !std::isfinite(result.rms) || result.rms < 0.0 ||
        !isFinite(result.R) || !isFinite(result.T))
    {
        return false;
    }

    return std::abs(cv::determinant(result.R) - 1.0) < 1e-3 && cv::norm(result.T) > 1e-9;
}

bool isValidRectificationResult(const stereo_calib::RectificationResult& result)
{
    return result.R1.rows == 3 && result.R1.cols == 3 && result.R1.type() == CV_64F &&
           result.R2.rows == 3 && result.R2.cols == 3 && result.R2.type() == CV_64F &&
           result.P1.rows == 3 && result.P1.cols == 4 && result.P1.type() == CV_64F &&
           result.P2.rows == 3 && result.P2.cols == 4 && result.P2.type() == CV_64F &&
           result.Q.rows == 4 && result.Q.cols == 4 && result.Q.type() == CV_64F &&
           isFinite(result.R1) && isFinite(result.R2) && isFinite(result.P1) &&
           isFinite(result.P2) && isFinite(result.Q);
}

bool isValidQuality(const stereo_calib::CalibrationQualityMetrics& quality)
{
    return quality.acceptedObservations > 0 &&
           std::isfinite(quality.baseline) && quality.baseline > 0.0 &&
           std::isfinite(quality.meanVerticalRectificationErrorPx) && quality.meanVerticalRectificationErrorPx >= 0.0 &&
           std::isfinite(quality.medianVerticalRectificationErrorPx) && quality.medianVerticalRectificationErrorPx >= 0.0 &&
           std::isfinite(quality.rmsVerticalRectificationErrorPx) && quality.rmsVerticalRectificationErrorPx >= 0.0 &&
           std::isfinite(quality.p95VerticalRectificationErrorPx) && quality.p95VerticalRectificationErrorPx >= 0.0 &&
           std::isfinite(quality.maxVerticalRectificationErrorPx) && quality.maxVerticalRectificationErrorPx >= 0.0;
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
    calculated.mode = options.calibrationMode;

    try
    {
        if (!options.isValid())
        {
            errorMessage = "Invalid fisheye calibration options.";
            return false;
        }

        if (!validateObservations(observations, options.imageSize, errorMessage))
        {
            return false;
        }

        switch (options.calibrationMode)
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
                                 calculated.rightCamera, calculated.stereo, errorMessage) ||
                !computeRectification(options, calculated.leftCamera, calculated.rightCamera,
                                      calculated.stereo, calculated.rectification, errorMessage) ||
                !evaluateRectification(observations, calculated.leftCamera, calculated.rightCamera,
                                       calculated.stereo, calculated.rectification,
                                       calculated.quality, errorMessage))
            {
                return false;
            }
            break;

        default:
            errorMessage = "Unknown calibration mode.";
            return false;
        }

        if (!calculated.isValid() ||
            (calculated.mode == CalibrationMode::StereoFull && !isValidQuality(calculated.quality)))
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
    intrinsic.at<double>(0, 2) = options.imageSize.width * 0.5;
    intrinsic.at<double>(1, 2) = options.imageSize.height * 0.5;
    cv::Mat distortion = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;

    const double rms = cv::fisheye::calibrate(objectPoints, imagePoints, options.imageSize,
                                                intrinsic, distortion, rotationVectors,
                                                translationVectors, options.monocularFlags,
                                                options.criteria);

    result = CameraCalibrationResult{};
    result.imageSize = options.imageSize;
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

    if (!isValidCameraResult(result))
    {
        errorMessage = "Monocular calibration produced invalid camera parameters.";
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
                                                      leftK, leftD, rightK, rightD, options.imageSize,
                                                      rotation, translation, options.stereoFlags,
                                                      options.criteria);

    result = StereoCalibrationResult{};
    result.R = rotation.clone();
    result.T = translation.reshape(1, 3).clone();
    result.rms = rms;
    if (!isValidStereoResult(result))
    {
        errorMessage = "Stereo calibration produced invalid extrinsic parameters.";
        return false;
    }
    return true;
}

bool FisheyeCalibrationEngine::computeRectification(
    const FisheyeCalibrationOptions& options,
    const CameraCalibrationResult& leftCamera,
    const CameraCalibrationResult& rightCamera,
    const StereoCalibrationResult& stereo,
    RectificationResult& result,
    std::string& errorMessage) const
{
    result = RectificationResult{};
    cv::fisheye::stereoRectify(leftCamera.K, leftCamera.D, rightCamera.K, rightCamera.D,
                               options.imageSize, stereo.R, stereo.T, result.R1, result.R2,
                               result.P1, result.P2, result.Q,
                               options.zeroDisparity ? cv::CALIB_ZERO_DISPARITY : 0,
                               options.imageSize, options.rectificationBalance,
                               options.rectificationFovScale);
    result.outputImageSize = options.imageSize;
    result.balance = options.rectificationBalance;
    result.fovScale = options.rectificationFovScale;
    result.zeroDisparity = options.zeroDisparity;

    if (!isValidRectificationResult(result))
    {
        errorMessage = "Stereo rectification produced invalid matrices.";
        return false;
    }
    return true;
}

bool FisheyeCalibrationEngine::evaluateRectification(
    const std::vector<StereoObservation>& observations,
    const CameraCalibrationResult& leftCamera,
    const CameraCalibrationResult& rightCamera,
    const StereoCalibrationResult& stereo,
    const RectificationResult& rectification,
    CalibrationQualityMetrics& quality,
    std::string& errorMessage) const
{
    std::vector<double> verticalErrors;
    verticalErrors.reserve(observations.size() * observations.front().objectPoints.size());
    double timestampDeltaSumMs = 0.0;

    for (const StereoObservation& observation : observations)
    {
        std::vector<cv::Point2f> rectifiedLeftPoints;
        std::vector<cv::Point2f> rectifiedRightPoints;
        cv::fisheye::undistortPoints(observation.leftImagePoints, rectifiedLeftPoints,
                                     leftCamera.K, leftCamera.D, rectification.R1, rectification.P1);
        cv::fisheye::undistortPoints(observation.rightImagePoints, rectifiedRightPoints,
                                     rightCamera.K, rightCamera.D, rectification.R2, rectification.P2);
        if (rectifiedLeftPoints.size() != rectifiedRightPoints.size() || rectifiedLeftPoints.empty())
        {
            errorMessage = "Rectification returned inconsistent point sets.";
            return false;
        }

        for (std::size_t pointIndex = 0; pointIndex < rectifiedLeftPoints.size(); ++pointIndex)
        {
            const double error = std::abs(static_cast<double>(rectifiedLeftPoints[pointIndex].y) -
                                          static_cast<double>(rectifiedRightPoints[pointIndex].y));
            if (!std::isfinite(error))
            {
                errorMessage = "Rectification produced a non-finite vertical error.";
                return false;
            }
            verticalErrors.push_back(error);
        }

        timestampDeltaSumMs += observation.timestampDeltaSeconds * 1000.0;
        quality.maxTimestampDeltaMs = std::max(quality.maxTimestampDeltaMs,
                                               observation.timestampDeltaSeconds * 1000.0);
    }

    if (verticalErrors.empty())
    {
        errorMessage = "No rectified correspondences are available for quality evaluation.";
        return false;
    }

    const double sum = std::accumulate(verticalErrors.begin(), verticalErrors.end(), 0.0);
    const double sumSquares = std::inner_product(verticalErrors.begin(), verticalErrors.end(),
                                                  verticalErrors.begin(), 0.0);
    std::sort(verticalErrors.begin(), verticalErrors.end());

    quality.synchronizedPairs = observations.size();
    quality.acceptedObservations = observations.size();
    quality.rejectedDetections = 0;
    quality.baseline = cv::norm(stereo.T);
    quality.meanTimestampDeltaMs = timestampDeltaSumMs / static_cast<double>(observations.size());
    quality.meanVerticalRectificationErrorPx = sum / static_cast<double>(verticalErrors.size());
    quality.rmsVerticalRectificationErrorPx = std::sqrt(sumSquares / static_cast<double>(verticalErrors.size()));
    const std::size_t middle = verticalErrors.size() / 2;
    quality.medianVerticalRectificationErrorPx = verticalErrors.size() % 2 == 0
        ? 0.5 * (verticalErrors[middle - 1] + verticalErrors[middle])
        : verticalErrors[middle];
    const std::size_t p95Index = static_cast<std::size_t>(std::ceil(0.95 * verticalErrors.size())) - 1;
    quality.p95VerticalRectificationErrorPx = verticalErrors[p95Index];
    quality.maxVerticalRectificationErrorPx = verticalErrors.back();

    if (!isValidQuality(quality))
    {
        errorMessage = "Rectification quality metrics are invalid.";
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
