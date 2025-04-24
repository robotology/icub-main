#include "FineCalibrationChecker.h"
#include <iostream>

int main()
{
    // Create an instance of FineCalibrationChecker
    FineCalibrationChecker checker;

    // Run the calibration
    checker.runCalibration();

    // Check if the calibration was successful
    if (checker.isCalibrationSuccessful()) {
        std::cout << "Calibration was successful!" << std::endl;
    } else {
        std::cout << "Calibration failed!" << std::endl;
    }

    return 0;
}