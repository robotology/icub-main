/**
*  latest iCub simulation
**/

//---------------------------------------TEMPORARY-----------------------------------------------------------
For an updated description of the simulator please refer to: http://eris.liralab.it/wiki/Simulator_README
//-----------------------------------------------------------------------------------------------------------

TABLE OF CONTENTS:

    * 1 BEFORE COMPILING
    * 2 RUNNING THE PROGRAM:
    * 3 RECENT CHANGES:
          o 3.1 SIM:
          o 3.2 YARPED features:
    * 4 USING THE FEATURES:
          o 4.1 CONTROLLING THE ROBOT:
          o 4.2 CAMERA:
                + 4.2.1 images:
          o 4.3 OBJECT INFORMATION:
          o 4.4 OBJECT CREATION:
          o 4.5 HAND POSITIONS:
          o 4.6 TOUCH SENSORS:
                + 4.6.1 Touch sensors:
          o 4.7 GRAB FUNCTION:
                + 4.7.1 "grab" function:
          o 4.8 FACE EXPRESSIONS:
          o 4.9 PROJECT A VIDEO STREAM ON A SCREEN

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
BEFORE COMPILING
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

iCub Simulator prerequisites: Cmake, ACE, YARP, GTK, iCUB repository, ODE, SDL Please make sure you have everything installed correctly by following this link 	http://eris.liralab.it/wiki/ODE

Once the libraries are installed, set your environment variables to:

ODE_DIR -> root directory of the compiled library
ICUB_DIR-> root directory of the iCub repository
SDLDIR -> root directory of the SDL directory (At least in windows, plus don't forget to add sdl.dll to your path)

By default the simulator uses single precision. The user has the option to modify this by switching flag (USE_ODE_DOUBLE) in ccmake(linux) or cmake(windows)

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
RUNNING THE PROGRAM:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

The iCubSimulator uses the ResourceFinder to read configuration parameters from a file placed in a directory called simConfig in $ICUB_ROOT/app. This makes changing the parameters used by the module by switching its "initialization context" much easier. The iCubSimulator looks for all the configuration parameters in a file called simulator.ini which sets up all the required parameters accordingly. By default the parameters are in $ICUB_ROOT/app/simConfig therefore by running run iCub_SIM from the source directory, it will pick up all the default configuration files. If the users would like to run the simulator with their own configuration files, we have to tell the module (the ResourceFinder) where to look for the file ‘simulator.ini’. We do this by specifying the initialization context, that is the name of the directory that should be used to locate the file ‘simulator.ini'.

A "iCub_parts_activation" initialization file is added to the "conf" subdirectory. This file contains setup for the simulated iCub and for the vision module (setting it directly on/off) the iCub setup file looks like this:

[SETUP]
elevation on

[PARTS]
legs on
torso on 
left_arm on
left_hand off
right_arm on 
right_hand off
head on
fixed_hip on

[VISION]
cam on

[RENDER]
objects on
cover on

Make sure you select the parts you would like to use otherwise they will not respond. It is also recommended to turn off the parts not required to save computational effort.

On linux, you'll need to be careful not to put windows over the simulator window - otherwise the output on the camera ports will be random in those areas :-).

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
RECENT CHANGES:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

SIM:

    * New ressource finder for the configuration of the simulator
    * Added the possibility to create cylinders in the world dynamic or static
    * Added rotation command for the boxes in the world (see section 4.4) 
    * Added colour choice in making of new objects (see section 4.4)
    * iCub parts activation file (see section 2)
    * Fixed vision left/right with proper rotations
    * Possibility to create static objects and dynamic objects in the world...with fixed crashing issues ;-)
    * Stable hand/finger functionality for grasping/manipulating objects
    * Better collision detection and friction
    * Much more performant (without fingers > 35 fps with fingers > 25 fps)
    * Multiple object in environment...the two standards ball and cube and possibility of creating up to 100 boxes see below. 

YARPED features:

    * Fixed sensor issues for both hand functionality
    * Faster video streaming
    * Object (box) can now be created and placed in the world with any size and at any position
    * Any object coordinates can be retrieve
    * Can retrieve robots hand coordinates
    * Grab "magnet" and release object possibility for the no finger functionality 

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
USING THE FEATURES:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    * Please set the desired iCub parts activation before running.
    * Manual navigation in the environment is possible. Use keys w,a,s,d and mouse for rotating. 

iCub_SIM.exe 

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------        
CONTROLLING THE ROBOT:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Current robot ports: please refer to http://eris.liralab.it/wiki/ICub_joints for further informations

yarp rpc /icubSim/left_leg/rpc:i  (The left leg has 6 joints in the standard configuration)
yarp rpc /icubSim/right_leg/rpc:i  (The structure is identical to the left leg)
yarp rpc /icubSim/torso/rpc:i     (The Torso has 3 joints in the standard configuration)
yarp rpc /icubSim/left_arm/rpc:i  (The arm includes the hand for a total of 16 controlled degrees of freedom)
yarp rpc /icubSim/right_arm/rpc:i (The structure is identical to the left arm)
yarp rpc /icubSim/head/rpc:i      (The head has 6 joints in the standard configuration)

A user can see the commands that the "rpc:i" port supports by using "yarp rpc" to send it the message "help".

For example:

yarp rpc /icubSim/left_arm/rpc:i
set pos 0 45
will command axis 0 of the left_arm (Shoulder pitch) to 45 degrees.

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CAMERA:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

images:

It is possible to get the images from the left and right eye as well as a environment view. Open a yarpview window: "yarpview /example"

connect yarpview to /icubSim/cam for view of robot
connect yarpview to /icubSim/cam/left for view from left camera
connect yarpview to /icubSim/cam/right for view from right camera
connect yarpview to /icubSim/cam/ for the environment view

eg: "yarp connect /icubSim/cam/left /example" 

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
OBJECT INFORMATION:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

There are currently different objects in the world ball, cube, and box. For the box please refer to 3.2

in the world port:

"yarp rpc /icubSim/world"

    * GET example: 

"world get cube"  (or ball) this will return the x y z coordinates of the object

    * SET example: 

"world set box x y z"  eg: "world set ball 1.0 0.1 2.0"

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
OBJECT CREATION:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

It is currently possible to create boxes and cylinders in the world

in the world port:

yarp rpc /icubSim/world

world mk box (three params for size) (three params for pos) (three params for colour)    (This creates a box affected by gravity)

world mk cyl (radius lenght) (three params for pos) (three params for colour)    (This creates a cylinder affected by gravity)


world mk sbox (three params for size) (three params for pos) (three params for colour)    (This creates a static box)

world mk scyl (radius lenght) (three params for pos) (three params for colour)    (This creates a static cylinder) 

eg:

world mk box 0.03 0.03 0.03 0.3 0.2 1 1 0 0     
world mk cyl 0.1 0.2 0.0 0.9 1.0 0 0 1     

the first box will be named box/cyl 1, second box/cyl 2 and so on...

get/set objects:

to get and set positions for these newly created objects:

world get box (num)   or     world set box (num) x y z

eg:

world get box 1    or     world set box 1 2 2 2
world get cyl 1    or     world set cyl 1 2 2 2

rotate objects:

If you need to rotate the boxes or the cylinders just use the following function:

world rot (object)(num) rotx roty rotz 

(where: rotx = rotation in degrees on the x axis roty = rotation in degrees on the y axis rotz = rotation in degrees on the z axis )

objects for now can be box/sbox/cyl/scyl

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
HAND POSITIONS:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

in the world port:

yarp rpc /icubSim/world

hand positions:

world get lhand 

or

world get rhand

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TOUCH SENSORS:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Touch sensors:

6 touch sensors were added on each of the hands:

    * palm
    * last part of index finger
    * last part of middle finger
    * last part of ring finger
    * last part of little finger
    * last part of thumb 

    * new port added 

the sensor data is streamed to this port: /icubSim/touch

to connect to it simply create a yarp read port eg:

yarp read /reader /icubSim/touch

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
GRAB FUNCTION:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Grab "magnet" and release object possibility for the no finger functionality (by popular demand)

"grab" function:

world grab (obj) (hand) 1  -to grab
world grab (obj) (hand) 0  -to release

eg:

world grab ball left 1

to grab the newly created objects: (box)

world grab box (num) (hand) 1 or 0

eg:

world grab box 1 right 1

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
FACE EXPRESSIONS:
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Face expressions can now be used with the simulator. First of all, compile and run the face expression module that you can find in:

\iCubSimulation\FaceExpressions\

Run the simulator and connect these ports:

yarp connect /face/eyelids /icubSim/face/eyelids 
yarp connect /face/image/out /icubSim/texture

Open writer so you can set face expressions in the simulator

yarp write /writer /icubSim/face/raw/in

Face expressions are set by changing hexadecimal values of the following subsystems:

left eyebrow (from L00 to L08)
right eyebrow (from R00 to R08)
mouth (from M00 to M3F)
eyelids (from S24(closed) to S48(opened))

The above commands in the brackets will set iCub's mood to sort of happy. Only one command is send at a time. The first letter in the command specifies the subsystem and the rest is hexadecimal number, which is then decoded to 8 bits binary number. For example L followed by 02 sends this binary number (0000 0010) to the left eyebrow subsystem, which activates 2nd port. In similar way, L followed by 04 would activated 3rd port of the left eyebrow subsystem as the binary number for hex 02 is (0000 0100).

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
PROJECT A VIDEO STREAM ON A SCREEN
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

This section describes how to create a simple "screen" in the simulator that would take video stream and display it as a texture.

Simple screen creation:

in iCub.h ~ line 33 in class ICubSim : public ICubData {

- Declare a geom as :

dGeomID screenGeom;

in iCub.cpp: ~ line 861 in void ICubSim::init( - Create the geom as :

screenGeom = dCreateBox(space,1.0,1.0,0.01); //this will create a thin screen of width 1m height 1m and depth 0.01m

~ line 678 in void ICubSim::setPosition( - Position your geom in the world as =

dGeomSetPosition(screenGeom,0.0,1.0,0.7); //This will position the screen in front of the robot (parameters are x y z)

~ line 175 in void ICubSim::draw( - draw the newly created screen as :

glColor3d(1.0,1.0,1.0);
glPushMatrix();LDEsetM(dGeomGetPosition(screenGeom),dGeomGetRotation(screenGeom));
DrawBox(1.0,1.0,0.001,false,textured,2);glPopMatrix();

this gets the position and the rotation of the geom and draws it in the world.

Pay attention at the parameters in the Draw function

DrawBox(1.0,1.0,0.001,false,textured,2)

x, y , z , wireframe, texture, whichTexture

wireframe => draw it as wireframe = false 
textured => use texture = true (textured)
whichTexture => which texture to use = in this case use "2"


Setting up the port:

If you look in the $ICUB_ROOT/src/iCubSimulation/conf directory, you'll find a file called Video.ini. Its contents are:

// Uncomment the line that starts with "textures" to declare what // video textures you want. You can have several video textures. // Each video texture is assigned a textureIndex, which corresponds // to the texture indices used in the simulator. // If you don't uncomment the textures line, no ports will be created.

textures sky

[sky] textureIndex 2 <---------HERE use the same texture number as previously set port /icubSim/texture

What happens here is we set up a yarp "port" name for each texture in the simulator that you want to control via a stream.


To test these settings enter the following commands:

yarpdev --device test_grabber --name /test/video --mode ball
yarp connect /test/video /icubSim/texture

At this stage you can send web cam streams or video sequences directly to the port /icubSim/texture and it will be displayed as a texture on an object. 

