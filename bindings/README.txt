# Copyright: (C) 2011 IITRBCS
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

This is a stub for python/java/... bindings to ICUB libraries.
To build:
 * Build repository as usual.   On a 64-bit machine, consider 
   rebuilding the ICUB repository with ICUB_SHARED_LIBRARY set 
   to TRUE/ON.
 * Set ICUB_DIR environment variable to your build directory.
 * Create a new build directory for e.g. python binding, and enter it.
 * Run cmake with the directory this README.txt is in as its source
   directory.
 * Set one of CREATE_PYTHON, CREATE_JAVA, ... options to TRUE/ON.
 * Set *only* *one* option on.  For multiple languages, use multiple
   build directories.
 * Go ahead and compile.  If you have trouble, consider rebuilding
   the ICUB repository with ICUB_SHARED_LIBRARY set to TRUE/ON.

Note, for a lot of purposes you'll also need the YARP bindings. 
See $YARP_ROOT/example/swig

==================================================================

Python example:
  import icub
  import yarp
  d = yarp.Drivers.factory()
  print d.toString().c_str()
  # list should include cartesiancontrollerclient etc, available
  # from PolyDriver

To run for testing, be in the directory you built the icub bindings,
place the above test in test.py, and do:
  PYTHONPATH=/path/to/yarp/python/bindings/ python test.py

