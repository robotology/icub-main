
The Control Basis framework is designed to fascillitate the construction and exploration of 
control programs in a principled and formal framework.  This directory holds the core files in 
the YARP implementation of a Control Basis API (or CBAPI).  


------------
DEPENDENCIES
------------

The CBAPI library requieres the iKin library.


------------
INSTALLATION
------------

To compile the CBAPI library, simply "make" and "make install" in this directory.  



----------------------------
RUNNING ICUB CBAPI RESOURCES
----------------------------

*** USING THE "iCubControlBasisResources" MODULE ***

A module exists that allows a user to start and run CBAPI resources for the iCub or the iCub simulator.
To run this program with the simulator, run:

> iCubControlBasisResources  --from iCubSimControlBasisResources.ini

for the real robot:

> iCubControlBasisResources  --from iCubControlBasisResources.ini

The default context should find these these config files, but they are located in:

$ICUB_ROOT/iCub/app/controlBasis/conf/

To edit which resources are to be run, edit these *.ini files.

The documentation for this module is located at:
http://eris.liralab.it/iCub/dox/html/group__icub__iCubControlBasisResources.html


*** USING THE "controlBasisResource" MODULE ***

An individual resource can be started by running the controlBasisResource module.  
Paremters for this module include:

--type <cb_tybe>:		specifies the type of resource (e.g., yarpConfiguration, endEffector, triangulatedPosition, etc.)
--robot <robot_name>:		specifies the robot device (e.g., /icub or /icubSim, etc.)
--part <part_name>:  		specifies the part of the robot (e.g., right_arm, left_leg, torso, head, etc.)
--cbName <cb_name>: 		a user-specified name to refer to the resource by (this is optional. the default is the <part_name>)
--configFile <file>:		the name of the configuration file that may be necessary for the resource
--velocityMode <bool>:		yarpConfiguration resources can be configured to send position commands to either a PolyDriver device or velocityControl module.
--simulationMode <bool>:	specifies whether this resource refers to a simulated robot or not
--updateDelay <double>:		the delay (in milleseconds) the resource will sleep between updates

The documentation for the controlBasisResource module is located at:
http://eris.liralab.it/iCub/dox/html/group__icub__controlBasisResource.html


*Note, the iCub configuration resources should run in velocityMode.  This means that it is necessary to start the velocityControl module
for the requisite part.  The iCub simulator can run in either velocityMode or not.  The velocityControl module documentation is located at:
http://eris.liralab.it/iCub/dox/html/group__icub__velocityControl.html


------------
APPLICATIONS
------------

A CBAPI GUI exists that allows a user to create and run multi-objective (prioritized) control 
laws.  To run this program:

> cd $ICUB_ROOT/src/gui/controlBasis/
> make
> make install;
> controlBasisGui


In this same directory, there exists a simple GUI for starting and editing resources that can
be used as references to controllers.  To run this program:

> referenceGui



------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------

AUTHOR: STEPHEN HART
EMAIL: stephen.hart@iit.it
LAST UPDATED: 15/2/2010




