// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef __PPEVENTDEBUGGER__
#define __PPEVENTDEBUGGER__

/**
*
@ingroup icub_module
\defgroup icub_ppEventDebugger ppEventDebugger
Wrapper library to use the parallel port for debugging purposes.

\section intro_sec Description
This library wraps the parallel port on Linux and Windows and can be used
to perform debugging using an oscilloscope. If you instantiate an object of
ppEventDebugger you can call methods like set() and reset() to activate/reset 
all bits on the parallel port.

In Linux your program will need root access to open the parallel port.

Example:

Suppose you want to determine the time it takes to execute a certain part 
of the code.

\code
static ppEventDebugger debugger(0x378);

debugger.set();
// start code you want to time
...
...
// end code you want to time
debugger.reset();
\endcode

Now connect the oscilloscope to any data bit of the parallel port and you should see a 
square wave.

Note that this is not particularly useful for timing code on a local machine, but it becomes
very handy when you want to time events on different machines.

\section lib_sec Libraries
YARP. On Windows PortTalk from BeyondLogic, this is not a library but a device
driver which provides user level access to the parallel port.


\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/ppEventDebugger/ppEventDebugger.h
**/

/**
* \defgroup icub_ppEventDebuggerClass ppEventDebuggerClass
*/
class ppEventDebugger
{
private:
    short base;
    bool initialized;
public:
    /**
    * Default constructor.
    */
    ppEventDebugger();

    /**
    * constructor. Opens the parallel port.
    * @param address: address of the parallel port.
    */
    ppEventDebugger(unsigned int address);

    /*
    * Destructor, calls close.
    */
    ~ppEventDebugger();

    /**
    * Open parallel port. Notice: on Linux you need
    * root access.
    * @address specify address of the parallel port
    * @return true/false
    */
    bool open(unsigned int address);

    /**
    * Close parallel port.
    */ 
    void close();

    /**
    * Set all bits of the output port to 1
    */
    void set();

    /**
    * Reset all bits to zero.
    */
    void reset();
};

#endif
