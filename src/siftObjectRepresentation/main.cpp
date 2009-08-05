// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
@ingroup icub_module
\defgroup icub_siftObjectRepresentation siftObjectRepresentation

Detects objects shown to the robot, saves their SIFT representation and detects and outputs the locations (in angles) of known objects around the robot.

\section intro_sec Description
This module uses SIFTs to recognise and detects almost any kind of object in the robot's surroundings.
The module:
-       Extracts the SIFT features from the inputed images, from the expected left and right eye of the robot.
-       Matches the SIFT features from one eye image to the other and calculates disparity between them (assumes parallel eyes)
-       Saves to a database the SIFT features with large disparity (supposed close object) labeling them by order of detections (although more fancy labeling could be inputed here)
-       Matches extracted sift features to the database to detect known objects in the left image and draws a bounding box around detected ojects
-       Outputs a bottle containing the label of detected objects and position in azimuth and elevation (relative to the iCub's neck).
-       Outputs an image with the extracted features in yellow plus the drawn bounding box.

You might find it convenient to include an image:
\image html EXAMPLE.jpg
\image latex EXAMPLE.eps "MODULENAME running on Linux" width=10cm

\section lib_sec Libraries
List here dependencies. Often these are just YARP libraries.
Yarp
OpenCV
KINEMATICS
GtkPlus
GSL

\section parameters_sec Parameters
Provide a comprehensive list of the parameters you can pass to the module. For example:

--file icubEyes.ini: configuration file to use
 
\section portsa_sec Ports Accessed
The module does not automatically access any port, but requires for full functionality:
- camCalib/right/out
- camCalib/left/out
- egoSphere/objects

\section portsc_sec Ports Created
Output ports:
- /chica/sift/left: streams out a ImageOf<PixelBgr> which contains the inputed left image overlaid with yellow SIFT features and detected object's bounding boxes
- /chica/sift/right: outputs a ImageOf<PixelBgr> which contains a picture of the last saved object with the saved SIFT features in blue
- /chica/sift/positions: 

Input ports:
- /chica/sift/left: inputs a ImageOf<PixelBgr> from the left eye of the robot
- /chica/sift/right: inputs a ImageOf<PixelBgr> from the right eye of the robot

\section in_files_sec Input Data Files
Database of saved objects should be in folder /database:
 * - database.txt that has the number and names of objects stored
 * - x.txt which has the features of object "x"
 * - x.png the snapshot of the object "x" when it was stored

\section out_data_sec Output Data Files
If your module writes data to a file, say so.
 
\section conf_file_sec Configuration Files
The module requires a description of the robot through the parameter 
--file.icubEyes.ini

The file consists in a few sections:
\code
name        /chica/sift
controlboad /controlboard
simulation 0
saveNewObjects 0

[CAMERA_CALIBRATION_LEFT]
fx 455.39
fy 454.972
\endcode

\e name determines the name of the module

\e controlboard should be the name of the iCub's motor's controlboard

\e simulation simulates the existance of a controlboard (fixed encoder values)

\e storeNewImages determines if the right eye image is used to look for close-by objects which could be new and therefore saved

\e fx is the focal lenght in x of the left eye

\e fy is the focal lenght in y of the left eye

\section tested_os_sec Tested OS
Linux
Does not compile in Windows with Visual Studio

\section example_sec Example Instantiation of the Module
./siftObjectRecognition --file $ICUB_ROOT/app/attention_objects/conf/icubEyes.ini --name /chica/sift

\author Dario Figueira

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/siftObjectRepresentationModule.
 **/

/*
 * Authors: 2008 Dario Figueira
 *         
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#include "siftObjectRepresentationModule.h"

int main(int argc, char* argv[]){

    // create the Sift algorithm module
    siftObjectRepresentationModule module;
    module.setName("/sift");

    // run the module
    return module.runModule(argc, argv);
}
