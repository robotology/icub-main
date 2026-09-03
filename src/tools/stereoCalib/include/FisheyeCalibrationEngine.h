#ifndef ICUB_STEREOCALIB_FISHEYE_CALIBRATION_ENGINE_H
#define ICUB_STEREOCALIB_FISHEYE_CALIBRATION_ENGINE_H

#include <string>
#include <vector>

#include "CalibrationTypes.h"

namespace stereo_calib
{

class FisheyeCalibrationEngine
{
public:
    bool calibrate(
        const std::vector<StereoObservation>& observations,
        const FisheyeCalibrationOptions& options,
        CalibrationResult& result,
        std::string& errorMessage) const;

private:
    bool validateObservations(
        const std::vector<StereoObservation>& observations,
        const cv::Size& expectedImageSize,
        std::string& errorMessage) const;

    bool calibrateMonocular(
        const std::vector<StereoObservation>& observations,
        CameraSide cameraSide,
        const FisheyeCalibrationOptions& options,
        CameraCalibrationResult& result,
        std::string& errorMessage) const;

    bool calibrateStereo(
        const std::vector<StereoObservation>& observations,
        const FisheyeCalibrationOptions& options,
        const CameraCalibrationResult& leftCamera,
        const CameraCalibrationResult& rightCamera,
        StereoCalibrationResult& result,
        std::string& errorMessage) const;

    bool computeRectification(
        const FisheyeCalibrationOptions& options,
        const CameraCalibrationResult& leftCamera,
        const CameraCalibrationResult& rightCamera,
        const StereoCalibrationResult& stereo,
        RectificationResult& result,
        std::string& errorMessage) const;

    bool evaluateRectification(
        const std::vector<StereoObservation>& observations,
        const CameraCalibrationResult& leftCamera,
        const CameraCalibrationResult& rightCamera,
        const StereoCalibrationResult& stereo,
        const RectificationResult& rectification,
        CalibrationQualityMetrics& quality,
        std::string& errorMessage) const;

    bool extractCalibrationPoints(
        const std::vector<StereoObservation>& observations,
        std::vector<std::vector<cv::Point3f>>& objectPoints,
        std::vector<std::vector<cv::Point2f>>& leftImagePoints,
        std::vector<std::vector<cv::Point2f>>& rightImagePoints,
        std::string& errorMessage) const;

};

} // namespace stereo_calib

#endif
