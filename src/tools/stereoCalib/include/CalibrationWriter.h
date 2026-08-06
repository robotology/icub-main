#ifndef ICUB_STEREOCALIB_CALIBRATION_WRITER_H
#define ICUB_STEREOCALIB_CALIBRATION_WRITER_H

#include <string>

#include "CalibrationTypes.h"

namespace stereo_calib
{

class CalibrationWriter
{
public:
    bool write(
        const std::string& outputFile,
        const CalibrationResult& result,
        std::string& errorMessage) const;

    bool writeObservations(
        const std::string& outputFile,
        const std::vector<StereoObservation>& observations,
        std::string& errorMessage) const;

    bool writeImagePair(
        const std::string& outputDirectory,
        std::size_t observationIndex,
        const cv::Mat& leftImg,
        const cv::Mat& rightImg,
        std::string& leftFile,
        std::string& rightFile,
        std::string& errorMessage) const;
private:
    bool validateResult(
        const CalibrationResult& result,
        std::string& errorMessage) const;
};

} // namespace stereo_calib

#endif
