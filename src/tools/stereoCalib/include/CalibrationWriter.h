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

private:
    bool validateResult(
        const CalibrationResult& result,
        std::string& errorMessage) const;
};

} // namespace stereo_calib

#endif