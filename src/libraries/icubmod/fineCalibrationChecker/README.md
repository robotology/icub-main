# Fine Calibration Checker Device

## Overview

This is a YARP device designed for `ergoCub` and `iCub` platforms that provides information about the accuracy of the joints calibration procedure. It helps collect and analyze data related to miscalibration errors on specific systems.
The device interfaces with robot **control boards** and **raw encoder data** streams through YARP.

The device performs calibration checking by:
- Connecting to remote control boards and raw encoder data ports
- Reading and comparing joint positions with expected golden positions (in degrees)
- Analyzing calibration accuracy and generating detailed reports
- Providing visual and numerical output for calibration analysis

## Main Features

The main features of this tool include:
- Configuration through XML files containing joint information and parameters
- Integration with motion control and raw values publisher network wrappers
- Remote control board connectivity for joint and encoder data access
- Comparison of golden hard-stop positions with live encoder readings
- Analysis of differences between expected and measured joint positions
- Generation of CSV output files and optional visual feedback
- Asynchronous operation as a threaded YARP device

This tool helps developers and technicians verify and fine-tune the mechanical calibration of robot joints, ensuring accurate and reliable operation.

## Configuration Parameters

The device requires the following parameters in its configuration file:
- `devicename`: Name of the device instance (default: "fineCalibrationChecker")
- `robotname`: Name of the robot matching configuration files
- `remoteRawValuesPort`: Port name for raw values data (e.g., "/setup/raw_data")
- `axesNamesList`: List of joint names matching remapper configuration
- `goldPositions`: Encoder positions in iCubDegrees at chosen zero positions
- `calibrationDeltas`: Calibration deltas from calibration files
- `encoderResolutions`: Full encoder resolutions (e.g., 16384 for AEA)
- `axesSigns`: Encoder signs based on primary encoder resolution
- `withGui`: Enable/disable visual output display

## Setup and Usage

To run the module:

1. Place the configuration XML file in your desired location (e.g., `./checker/fineCalibrationChecker.xml`)
2. Configure the `rawvaluespublisherremapper` and `rawValuesPublisherServer` devices
3. Add the device wrappers to the main robot configuration file
4. Start the `yarprobotinterface` application
5. The device will automatically execute if properly configured
6. Results are saved to a CSV file in the `yarprobotinterface` working directory
7. If `withGui` is enabled, a visualization of the output data will be displayed

For configuration examples, refer to `fineCalibrationCheckerConfig.xml`.




