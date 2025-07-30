# Fine Calibration Checker Tool

## Overview

This is a tool designed for ergoCub and iCub platforms aimed at giving to the user information about the accuracy of the joints calibration procedure, allowing to collect data that might help in analyzing miscalibration errors on a specific system. 
It has been thought to interface with **YARP** devices and robot **control boards**.
The tool is made of two main parts:
- a **module** that does not much other than managing a thread, which contains all the core functionalities of the application
- a **thread** that does all the job for collecting the desired data about the joint calibration

## FineCalibrationChecker Module

The module is built on the inheritance of the `YARP` interface `yarp::os::RFModule` and does not much other than managing a `yarp::os::Thread` and keep it running with a specific period until the user decides to stop the whole process.

## FineCalibrationChcker Thread

The thread is basically just an implementation of a `yarp::os::Thread` but it contains all the core implementations of this tool. 
Specifically it does the following actions:
- parse the configuration data given by the file `config.ini`
- initialize all the data structures needed for storing the data
- parse the `.csv` file that stores the input position data, which must be present in the `app/fineCalibrationChecker` directory named as `zeroPositionsData.csv` (an example file is already installed in the repo)
- open devices and connect to remote control boards and ports
- run the calibration check
- give in output the requested results

## Main Features

Therefore the main features of this tool can be summarized as follows:
- Reads configuration parameters and joint information from a configuration file.
- Connects to remote control boards and a raw values publisher to access joint and encoder data.
- Executes a calibration routine, including checking golden hard-stop positions and comparing them with live encoder readings.
- Evaluates the difference (delta) between expected and measured joint positions, logs the results, and generates output files and images for analysis.
- Runs as a threaded YARP module, allowing for asynchronous calibration and monitoring.

This tool is useful for developers and technicians who need to verify and fine-tune the mechanical calibration of the robot’s joints, ensuring accurate and reliable operation.

## Commands and files

In order to correctly run the module one should type the following command from the directory that stores `config.ini` file and the `zeroPositionData.csv` file:

```
fineCalibrationChecker --from conf/config.ini
```

It should be noted then, that in the same directory the aforementioned file for the ground truth positions should be stored and it should have the defined name.
It has to contain the data organized line by line and with the following structure:

```
axes_name,gold_position,joint_encoder_full_resolution
```
