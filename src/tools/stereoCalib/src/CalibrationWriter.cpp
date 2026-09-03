#include "CalibrationWriter.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{

bool isFinite(const cv::Mat& matrix)
{
    return !matrix.empty() && cv::checkRange(matrix, true, nullptr);
}

bool isFinite(double value)
{
    return std::isfinite(value);
}

bool isValidCamera(const stereo_calib::CameraCalibrationResult& camera)
{
    return camera.isValid() && camera.K.type() == CV_64F && camera.D.type() == CV_64F &&
           camera.K.at<double>(0, 0) > 0.0 && camera.K.at<double>(1, 1) > 0.0 &&
           isFinite(camera.K) && isFinite(camera.D) && isFinite(camera.rms);
}

bool isValidStereo(const stereo_calib::StereoCalibrationResult& stereo)
{
    return stereo.isValid() && stereo.R.type() == CV_64F && stereo.T.type() == CV_64F &&
           isFinite(stereo.R) && isFinite(stereo.T) && isFinite(stereo.rms);
}

bool isValidRectification(const stereo_calib::RectificationResult& rectification)
{
    return rectification.isValid() && rectification.R1.type() == CV_64F &&
           rectification.R2.type() == CV_64F && rectification.P1.type() == CV_64F &&
           rectification.P2.type() == CV_64F && rectification.Q.type() == CV_64F &&
           isFinite(rectification.R1) && isFinite(rectification.R2) &&
           isFinite(rectification.P1) && isFinite(rectification.P2) &&
           isFinite(rectification.Q) && isFinite(rectification.balance) &&
           isFinite(rectification.fovScale);
}

bool isValidStereoQuality(const stereo_calib::CalibrationQualityMetrics& quality)
{
    return quality.acceptedObservations > 0 && isFinite(quality.baseline) && quality.baseline > 0.0 &&
           isFinite(quality.meanTimestampDeltaMs) && isFinite(quality.maxTimestampDeltaMs) &&
           isFinite(quality.meanVerticalRectificationErrorPx) &&
           isFinite(quality.medianVerticalRectificationErrorPx) &&
           isFinite(quality.rmsVerticalRectificationErrorPx) &&
           isFinite(quality.p95VerticalRectificationErrorPx) &&
           isFinite(quality.maxVerticalRectificationErrorPx) &&
           quality.meanTimestampDeltaMs >= 0.0 && quality.maxTimestampDeltaMs >= 0.0 &&
           quality.meanVerticalRectificationErrorPx >= 0.0 &&
           quality.medianVerticalRectificationErrorPx >= 0.0 &&
           quality.rmsVerticalRectificationErrorPx >= 0.0 &&
           quality.p95VerticalRectificationErrorPx >= 0.0 &&
           quality.maxVerticalRectificationErrorPx >= 0.0;
}

double matrixValue(const cv::Mat& matrix, int index)
{
    return matrix.reshape(1, 1).at<double>(0, index);
}

void writeMatrix(std::ostream& output, const std::string& name, const cv::Mat& matrix)
{
    output << name << " (";
    for (int index = 0; index < static_cast<int>(matrix.total()); ++index)
    {
        if (index != 0)
        {
            output << ' ';
        }
        output << matrixValue(matrix, index);
    }
    output << ")\n";
}

void writeCamera(std::ostream& output, const char* side,
                 const stereo_calib::CameraCalibrationResult& camera)
{
    output << "[CAMERA_CALIBRATION_" << side << "]\n";
    output << "projection fisheye\n";
    output << "drawCenterCross 0\n\n";
    output << "w " << camera.imageSize.width << "\n";
    output << "h " << camera.imageSize.height << "\n";
    output << "fx " << matrixValue(camera.K, 0) << "\n";
    output << "fy " << matrixValue(camera.K, 4) << "\n";
    output << "cx " << matrixValue(camera.K, 2) << "\n";
    output << "cy " << matrixValue(camera.K, 5) << "\n";
    output << "k1 " << matrixValue(camera.D, 0) << "\n";
    output << "k2 " << matrixValue(camera.D, 1) << "\n";
    output << "k3 " << matrixValue(camera.D, 2) << "\n";
    output << "k4 " << matrixValue(camera.D, 3) << "\n\n";
}

std::string temporaryObservationFile(const std::string& filename)
{
    const std::size_t separator = filename.find_last_of("/\\");
    const std::size_t extension = filename.find_last_of('.');
    if (extension != std::string::npos &&
        (separator == std::string::npos || extension > separator))
    {
        return filename.substr(0, extension) + ".tmp" + filename.substr(extension);
    }
    return filename + ".tmp.yml";
}

std::string joinPath(const std::string& directory, const std::string& filename)
{
    if (directory.empty() || directory.back() == '/' || directory.back() == '\\')
    {
        return directory + filename;
    }
    return directory + '/' + filename;
}

bool isValidObservation(const stereo_calib::StereoObservation& observation)
{
    if (!observation.isValid() || !isFinite(observation.leftTimestampSeconds) ||
        !isFinite(observation.rightTimestampSeconds) ||
        !isFinite(observation.timestampDeltaSeconds))
    {
        return false;
    }
    for (std::size_t index = 0; index < observation.objectPoints.size(); ++index)
    {
        const cv::Point3f& objectPoint = observation.objectPoints[index];
        const cv::Point2f& leftPoint = observation.leftImagePoints[index];
        const cv::Point2f& rightPoint = observation.rightImagePoints[index];
        if (!isFinite(objectPoint.x) || !isFinite(objectPoint.y) || !isFinite(objectPoint.z) ||
            !isFinite(leftPoint.x) || !isFinite(leftPoint.y) ||
            !isFinite(rightPoint.x) || !isFinite(rightPoint.y))
        {
            return false;
        }
    }
    return true;
}

} // namespace

namespace stereo_calib
{

bool CalibrationWriter::write(const std::string& outputFile,
                              const CalibrationResult& result,
                              std::string& errorMessage) const
{
    errorMessage.clear();
    if (outputFile.empty())
    {
        errorMessage = "Calibration output filename is empty.";
        return false;
    }
    if (!validateResult(result, errorMessage))
    {
        return false;
    }

    const std::string temporaryFile = outputFile + ".tmp";
    std::ofstream output(temporaryFile.c_str(), std::ios::out | std::ios::trunc);
    if (!output.is_open())
    {
        errorMessage = "Could not open temporary calibration file '" + temporaryFile +
                       "': " + std::strerror(errno);
        return false;
    }

    output << std::setprecision(17);
    output << "# Generated by stereoCalib. Do not edit while calibration is running.\n";
    switch (result.mode)
    {
    case CalibrationMode::MonocularLeft:  output << "calibrationMode MonocularLeft\n\n"; break;
    case CalibrationMode::MonocularRight: output << "calibrationMode MonocularRight\n\n"; break;
    case CalibrationMode::MonocularBoth:  output << "calibrationMode MonocularBoth\n\n"; break;
    case CalibrationMode::StereoFull:     output << "calibrationMode StereoFull\n\n"; break;
    }

    const bool writesLeftCamera = result.mode != CalibrationMode::MonocularRight;
    const bool writesRightCamera = result.mode != CalibrationMode::MonocularLeft;
    if (writesLeftCamera)
    {
        writeCamera(output, "LEFT", result.leftCamera);
    }
    if (writesRightCamera)
    {
        writeCamera(output, "RIGHT", result.rightCamera);
    }

    if (result.mode == CalibrationMode::StereoFull)
    {
        cv::Mat homogeneousTransform = cv::Mat::eye(4, 4, CV_64F);
        result.stereo.R.copyTo(homogeneousTransform(cv::Rect(0, 0, 3, 3)));
        result.stereo.T.reshape(1, 3).copyTo(homogeneousTransform(cv::Rect(3, 0, 1, 3)));

        output << "[STEREO_DISPARITY]\n";
        // This is deliberately left-to-right: Xright = R * Xleft + T.
        writeMatrix(output, "HN", homogeneousTransform);
        writeMatrix(output, "R", result.stereo.R);
        writeMatrix(output, "T", result.stereo.T);
        output << "\n[STEREO_RECTIFICATION]\n";
        writeMatrix(output, "R1", result.rectification.R1);
        writeMatrix(output, "R2", result.rectification.R2);
        writeMatrix(output, "P1", result.rectification.P1);
        writeMatrix(output, "P2", result.rectification.P2);
        writeMatrix(output, "Q", result.rectification.Q);
        output << "balance " << result.rectification.balance << "\n";
        output << "fovScale " << result.rectification.fovScale << "\n";
        output << "zeroDisparity " << (result.rectification.zeroDisparity ? 1 : 0) << "\n\n";
    }

    output << "[CALIBRATION_QUALITY]\n";
    if (writesLeftCamera)
    {
        output << "leftRms " << result.leftCamera.rms << "\n";
    }
    if (writesRightCamera)
    {
        output << "rightRms " << result.rightCamera.rms << "\n";
    }
    if (result.mode == CalibrationMode::StereoFull)
    {
        output << "stereoRms " << result.stereo.rms << "\n";
        output << "baseline " << result.quality.baseline << "\n";
        output << "synchronizedPairs " << result.quality.synchronizedPairs << "\n";
        output << "acceptedObservations " << result.quality.acceptedObservations << "\n";
        output << "rejectedDetections " << result.quality.rejectedDetections << "\n";
        output << "meanTimestampDeltaMs " << result.quality.meanTimestampDeltaMs << "\n";
        output << "maxTimestampDeltaMs " << result.quality.maxTimestampDeltaMs << "\n";
        output << "meanVerticalErrorPx " << result.quality.meanVerticalRectificationErrorPx << "\n";
        output << "medianVerticalErrorPx " << result.quality.medianVerticalRectificationErrorPx << "\n";
        output << "rmsVerticalErrorPx " << result.quality.rmsVerticalRectificationErrorPx << "\n";
        output << "p95VerticalErrorPx " << result.quality.p95VerticalRectificationErrorPx << "\n";
        output << "maxVerticalErrorPx " << result.quality.maxVerticalRectificationErrorPx << "\n";
    }

    output.flush();
    if (!output.good())
    {
        errorMessage = "Failed while writing temporary calibration file '" + temporaryFile + "'.";
        return false;
    }
    output.close();
    if (output.fail())
    {
        errorMessage = "Could not close temporary calibration file '" + temporaryFile + "'.";
        return false;
    }

    std::ifstream validation(temporaryFile.c_str());
    if (!validation.good())
    {
        errorMessage = "Could not validate temporary calibration file '" + temporaryFile + "'.";
        return false;
    }
    validation.close();
    if (std::rename(temporaryFile.c_str(), outputFile.c_str()) != 0)
    {
        errorMessage = "Could not replace calibration file '" + outputFile + "' with temporary file: " +
                       std::strerror(errno);
        return false;
    }
    return true;
}

bool CalibrationWriter::writeObservations(const std::string& outputFile,
                                          const std::vector<StereoObservation>& observations,
                                          std::string& errorMessage) const
{
    errorMessage.clear();
    if (outputFile.empty())
    {
        errorMessage = "Observation output filename is empty.";
        return false;
    }
    for (std::size_t index = 0; index < observations.size(); ++index)
    {
        if (!isValidObservation(observations[index]))
        {
            errorMessage = "Observation #" + std::to_string(index) + " is invalid.";
            return false;
        }
    }

    const std::string temporaryFile = temporaryObservationFile(outputFile);
    try
    {
        cv::FileStorage storage(temporaryFile, cv::FileStorage::WRITE);
        if (!storage.isOpened())
        {
            errorMessage = "Could not open temporary observation file '" + temporaryFile + "'.";
            return false;
        }

        storage << "observations" << "[";
        for (std::size_t index = 0; index < observations.size(); ++index)
        {
            const StereoObservation& observation = observations[index];
            storage << "{";
            storage << "index" << static_cast<int>(index);
            storage << "imageWidth" << observation.imageSize.width;
            storage << "imageHeight" << observation.imageSize.height;
            storage << "leftTimestampSeconds" << observation.leftTimestampSeconds;
            storage << "rightTimestampSeconds" << observation.rightTimestampSeconds;
            storage << "timestampDeltaSeconds" << observation.timestampDeltaSeconds;
            storage << "leftSequenceNumber" << std::to_string(observation.leftSequenceNumber);
            storage << "rightSequenceNumber" << std::to_string(observation.rightSequenceNumber);
            storage << "leftImageFilename" << observation.leftImageFilename;
            storage << "rightImageFilename" << observation.rightImageFilename;
            storage << "objectPoints" << cv::Mat(observation.objectPoints);
            storage << "leftImagePoints" << cv::Mat(observation.leftImagePoints);
            storage << "rightImagePoints" << cv::Mat(observation.rightImagePoints);
            storage << "}";
        }
        storage << "]";
        storage.release();

        cv::FileStorage validation(temporaryFile, cv::FileStorage::READ);
        const cv::FileNode serializedObservations = validation["observations"];
        if (!validation.isOpened() || serializedObservations.type() != cv::FileNode::SEQ ||
            serializedObservations.size() != observations.size())
        {
            errorMessage = "Could not validate temporary observation file '" + temporaryFile + "'.";
            return false;
        }
    }
    catch (const cv::Exception& exception)
    {
        errorMessage = std::string("Could not write observation file: ") + exception.what();
        return false;
    }
    if (std::rename(temporaryFile.c_str(), outputFile.c_str()) != 0)
    {
        errorMessage = "Could not replace observation file '" + outputFile + "' with temporary file: " +
                       std::strerror(errno);
        return false;
    }
    return true;
}

bool CalibrationWriter::writeImagePair(const std::string& outputDirectory,
                                       std::size_t observationIndex,
                                       const cv::Mat& leftImg,
                                       const cv::Mat& rightImg,
                                       std::string& leftFile,
                                       std::string& rightFile,
                                       std::string& errorMessage) const
{
    errorMessage.clear();
    leftFile.clear();
    rightFile.clear();
    if (outputDirectory.empty() || leftImg.empty() || rightImg.empty())
    {
        errorMessage = "Cannot save an image pair without an output directory and two images.";
        return false;
    }
    if (leftImg.size() != rightImg.size())
    {
        errorMessage = "Cannot save an image pair with different image sizes.";
        return false;
    }

    std::ostringstream name;
    name << std::setfill('0') << std::setw(4) << observationIndex;
    leftFile = "left_" + name.str() + ".png";
    rightFile = "right_" + name.str() + ".png";

    cv::Mat leftToWrite;
    cv::Mat rightToWrite;
    try
    {
        if (leftImg.channels() == 3)
        {
            cv::cvtColor(leftImg, leftToWrite, cv::COLOR_RGB2BGR);
        }
        else
        {
            leftToWrite = leftImg.clone();
        }
        if (rightImg.channels() == 3)
        {
            cv::cvtColor(rightImg, rightToWrite, cv::COLOR_RGB2BGR);
        }
        else
        {
            rightToWrite = rightImg.clone();
        }

        if (!cv::imwrite(joinPath(outputDirectory, leftFile), leftToWrite))
        {
            errorMessage = "Could not write left calibration image '" + joinPath(outputDirectory, leftFile) + "'.";
            return false;
        }
        if (!cv::imwrite(joinPath(outputDirectory, rightFile), rightToWrite))
        {
            errorMessage = "Could not write right calibration image '" + joinPath(outputDirectory, rightFile) + "'.";
            return false;
        }
    }
    catch (const cv::Exception& exception)
    {
        errorMessage = std::string("Could not write calibration images: ") + exception.what();
        return false;
    }
    return true;
}

bool CalibrationWriter::validateResult(const CalibrationResult& result,
                                       std::string& errorMessage) const
{
    errorMessage.clear();
    switch (result.mode)
    {
    case CalibrationMode::MonocularLeft:
        if (!isValidCamera(result.leftCamera))
        {
            errorMessage = "Monocular-left result does not contain valid left fisheye intrinsics.";
            return false;
        }
        return true;
    case CalibrationMode::MonocularRight:
        if (!isValidCamera(result.rightCamera))
        {
            errorMessage = "Monocular-right result does not contain valid right fisheye intrinsics.";
            return false;
        }
        return true;
    case CalibrationMode::MonocularBoth:
        if (!isValidCamera(result.leftCamera) || !isValidCamera(result.rightCamera))
        {
            errorMessage = "Monocular-both result does not contain valid fisheye intrinsics for both cameras.";
            return false;
        }
        return true;
    case CalibrationMode::StereoFull:
        if (!isValidCamera(result.leftCamera) || !isValidCamera(result.rightCamera) ||
            !isValidStereo(result.stereo) || !isValidRectification(result.rectification) ||
            !isValidStereoQuality(result.quality))
        {
            errorMessage = "Stereo result is missing valid intrinsics, transform, rectification, or quality metrics.";
            return false;
        }
        return true;
    }
    errorMessage = "Unknown calibration mode.";
    return false;
}

} // namespace stereo_calib
