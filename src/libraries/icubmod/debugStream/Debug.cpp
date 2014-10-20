/*
 * Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "Debug.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <stdlib.h>
#include <string.h>

#include <yarp/os/Os.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>

#ifndef WIN32

 #define RED    (colored_output ? "\033[01;31m" : "")
 #define GREEN  (colored_output ? "\033[01;32m" : "")
 #define YELLOW (colored_output ? "\033[01;33m" : "")
 #define BLUE   (colored_output ? "\033[01;34m" : "")
 #define CLEAR  (colored_output ? "\033[00m" : "")

 bool LogStream::Debug::colored_output(getenv("ICUB_COLORED_OUTPUT") && (strcmp(getenv("ICUB_COLORED_OUTPUT"), "1") == 0));
 //bool LogStream::Debug::verbose_output(getenv("ICUB_VERBOSE_OUTPUT") && (strcmp(getenv("ICUB_VERBOSE_OUTPUT"), "1") == 0));
 bool LogStream::Debug::trace_output(getenv("ICUB_TRACE_ENABLE") && (strcmp(getenv("ICUB_TRACE_ENABLE"), "1") == 0));

#else // WIN32

 // TODO colored and verbose_output for WIN32
 #define RED    ""
 #define GREEN  ""
 #define YELLOW ""
 #define BLUE   ""
 #define CLEAR  ""

 bool LogStream::Debug::colored_output(false);
 bool LogStream::Debug::trace_output(false);
#endif // WIN32

class LogStream::NetworkForwarder
{
   public:
      void start();
      void stop();
      static NetworkForwarder* getInstance();
      void forward_output (MsgType t, const std::ostringstream &s, const char *file, unsigned int line, const char *func);
   protected:
      NetworkForwarder();
      ~NetworkForwarder();
      friend class NetworkForwarderDestroyer; 
   private:
      yarp::os::BufferedPort<yarp::os::Bottle>* outputPort;
      yarp::os::Semaphore mutex;
      private:
      NetworkForwarder(NetworkForwarder const&){};
      NetworkForwarder& operator=(NetworkForwarder const&){};
      static NetworkForwarder* instance;
      static NetworkForwarderDestroyer destroyer;
};

class LogStream::NetworkForwarderDestroyer
{
    public:
        NetworkForwarderDestroyer(NetworkForwarder * = 0);
        ~NetworkForwarderDestroyer();
        void SetSingleton(NetworkForwarder *s);
    private:
        NetworkForwarder* singleton;
};

LogStream::NetworkForwarder*         LogStream::NetworkForwarder::instance = NULL; 
LogStream::NetworkForwarderDestroyer LogStream::NetworkForwarder::destroyer;

LogStream::NetworkForwarderDestroyer::NetworkForwarderDestroyer(NetworkForwarder *s)
{
    singleton = s;
}

LogStream::NetworkForwarderDestroyer::~NetworkForwarderDestroyer()
{
    if (singleton)
    {
        delete singleton;
    }
}

void LogStream::NetworkForwarderDestroyer::SetSingleton(NetworkForwarder *s)
{
    singleton = s;
}

LogStream::NetworkForwarder* LogStream::NetworkForwarder::getInstance()
{ 
    /*
    //automatic instance
    if (instance == 0)
    {
        instance = new LogStream::NetworkForwarder();
    }*/
    return instance;
};

void LogStream::NetworkForwarder::start()
{
    if (instance == 0)
    {
        instance = new LogStream::NetworkForwarder();
    }
}

void LogStream::NetworkForwarder::stop()
{
    if (instance == 0)
    {
        delete instance;
        instance = 0;
    }
}

LogStream::NetworkForwarder::NetworkForwarder()
{
    yarp::os::Network::init();
    //yarpInstance = new yarp::os::Network;
    
    outputPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    outputPort->open("/log/test_machine/test_proc/test_pid");
    //yarp::os:Network::connect("/log/test_machine/test_proc/test_pid","logger");
};

LogStream::NetworkForwarder::~NetworkForwarder()
{
    if (outputPort)
    {
        outputPort->interrupt();
        outputPort->close();
        delete outputPort;
        outputPort=0;
        //delete yarpInstance;
        //yarpInstance =0 ;
                yarp::os::Network::fini();
    }

};

void LogStream::Debug::forward_output(MsgType t,
                                         const std::ostringstream &s,
                                         const char *file,
                                         unsigned int line,
                                         const char *func)
{
    NetworkForwarder* n = NetworkForwarder::getInstance();
    if (n)
    {
        n->forward_output(stream->type, stream->oss, stream->file, stream->line, stream->func);
    }
    delete stream;
}

void LogStream::Debug::print_output(MsgType t,
                                         const std::ostringstream &s,
                                         const char *file,
                                         unsigned int line,
                                         const char *func)
{
    switch (t)
    {
        case TraceType:
            if(trace_output)
            {
                std::cout << "[" << GREEN  << "TRACE"   << CLEAR << "]" << func << s.str() << std::endl;
            }
            break;
        case InfoType:
            {
                std::cerr << "[" << GREEN  << "INFO"    << CLEAR << "]" << s.str() << std::endl;
            }
            break;
        case DebugType:
            {
                std::cout << "[" << BLUE   << "DEBUG"   << CLEAR << "]" << s.str() << std::endl;
            }
            break;
        case WarningType:
            {
                std::cerr << "[" << YELLOW << "WARNING" << CLEAR << "]" << s.str() << std::endl;
            }
            break;
        case ErrorType:
            {
                std::cerr << "[" << RED    << "ERROR"   << CLEAR << "]" << s.str() << std::endl;
            }
            break;
        case FatalType:
            {
                std::cerr << "[" << RED    << "FATAL"   << CLEAR << "]" << s.str() << std::endl;
            }
            yarp::os::exit(-1);
            break;
        default:
            break;
    }
}

void LogStream::NetworkForwarder::forward_output(MsgType t,
                                         const std::ostringstream &s,
                                         const char *file,
                                         unsigned int line,
                                         const char *func)
{
    mutex.wait();
    switch (t)
    {
        case DebugType:
            {
                std::cout << "[DEBUG]" << s.str() << std::endl;
            }
            break;
        case InfoType:
            {
                std::cerr << "[INFO]" << s.str() << std::endl;
            }
            break;
        case WarningType:
            {
                std::cerr << "[WARNING]" << s.str() << std::endl;
            }
            break;
        case ErrorType:
            {
                std::cerr << "[ERROR]" << s.str() << std::endl;
            }
            break;
        case FatalType:
            {
                std::cerr << "[FATAL]" << s.str() << std::endl;
            }
            //yarp::os::exit(-1);
            break;
        default:
            break;
    }
    /*
    yarp::os::Bottle& b = outputPort->prepare();
    b.clear();
    b.addString("[/log/test_machine/test_proc/test_pid]");
    b.addString("ciao");
    outputPort->write();
    */
    //yarp::os::Bottle b;
    //b.addString("[/log/test_machine/test_proc/test_pid]");
    //b.addString("ciao");
    //this->outputPort->write(b);
    mutex.post();
}