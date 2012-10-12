/*
 * Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef MyRobotInterface_DEBUG_H
#define MyRobotInterface_DEBUG_H

#include <iosfwd>
#include <sstream>
#include <vector>
#include <string.h>
#include <yarp/os/ConstString.h>


namespace MyRobotInterface
{

enum MsgType { TraceType, DebugType, WarningType, ErrorType, FatalType };


class Debug {
    struct Stream {
        Stream(MsgType t) : type(t), ref(1) {}
        std::ostringstream oss;
        MsgType type;
        int ref;
    } *stream;
public:

    static int spaces;

    inline Debug(MsgType type) :
        stream(new Stream(type))
    {
    }

    inline Debug(const Debug &o) : stream(o.stream) {
        ++stream->ref;
    }

    inline ~Debug() {
        if (!--stream->ref) {
            print_output(stream->type, stream->oss);
            delete stream;
        }
    }

    inline Debug& operator<<(bool t) {
        stream->oss << (t ? "true" : "false");
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(char t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(signed short t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(unsigned short t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(signed int t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(unsigned int t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(signed long t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(unsigned long t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(signed long long t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(unsigned long long t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(float t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(double t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(const char* t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
    inline Debug& operator<<(const void * t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }

    inline Debug& operator<<(const std::string &t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }

    inline Debug& operator<<(yarp::os::ConstString t) {
        stream->oss << t.c_str();
        stream->oss << ' ';
        return *this;
    }

    template <typename T>
    inline Debug& operator<<(const std::vector<T> &t)
    {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }

    /*!
     * \brief Set the output file used by trace()
     */
    static void setTraceFile(const std::string &filename);

    /*!
     * \brief Set the output file used by debug()
     */
    static void setOutputFile(const std::string &filename);

    /*!
     * \brief Set the output file used by warning(), error() and fatal()
     */
    static void setErrorFile(const std::string &filename);

private:
    void print_output(MsgType t, const std::ostringstream &s);

    static std::ofstream ftrc; /// Used by trace()
    static std::ofstream fout; /// Used by debug()
    static std::ofstream ferr; /// Used by warning(), error() and fatal()

    static bool colored_output;
};

}

#define trace()  { \
					MyRobotInterface::Debug::spaces++; \
					char str[MyRobotInterface::Debug::spaces-1]; \
					memset(str, 0x3E, MyRobotInterface::Debug::spaces); \
					str[MyRobotInterface::Debug::spaces] = '\0'; \
					MyRobotInterface::Debug(MyRobotInterface::TraceType) << str <<  __PRETTY_FUNCTION__  << __LINE__; \
					}

#define endtrace()	if(MyRobotInterface::Debug::spaces > 0 ) MyRobotInterface::Debug::spaces--;

//#define trace() MyRobotInterface::Debug(MyRobotInterface::TraceType) << __PRETTY_FUNCTION__ << __FILE__ << __LINE__ \
//		MyRobotInterface::Debug::spaces++;

inline MyRobotInterface::Debug debug() {
    return MyRobotInterface::Debug(MyRobotInterface::DebugType);
}
inline MyRobotInterface::Debug warning() {
    return MyRobotInterface::Debug(MyRobotInterface::WarningType);
}
inline MyRobotInterface::Debug error() {
    return MyRobotInterface::Debug(MyRobotInterface::ErrorType);
}
inline MyRobotInterface::Debug fatal() {
    return MyRobotInterface::Debug(MyRobotInterface::FatalType);
}


template <typename T>
inline std::ostringstream& operator<<(std::ostringstream &oss, const std::vector<T> &t)
{
    for (typename std::vector<T>::const_iterator it = t.begin(); it != t.end(); it++) {
        const T &p = *it;
        oss << p;
    }
    return oss;
}


#endif // MyRobotInterface_DEBUG_H
