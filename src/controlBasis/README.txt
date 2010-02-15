
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

These config files are located in:

$ICUB_ROOT/iCub/app/controlBasis/conf/

To edit which resources are to be run, edit these *.ini files.


*** USING THE "controlBasisResource" MODULE ***

TODO..


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




