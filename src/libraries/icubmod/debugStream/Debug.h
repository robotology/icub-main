/*
 * Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef LogStream_DEBUG_H
#define LogStream_DEBUG_H

#include <iosfwd>
#include <sstream>
#include <vector>
#include <yarp/os/ConstString.h>

template <typename T>
std::ostringstream& operator<<(std::ostringstream &oss, const std::vector<T> &t);


namespace LogStream
{
enum MsgType { TraceType, DebugType, WarningType, ErrorType, FatalType, InfoType };

class NetworkForwarder;
/*
class NetworkForwarder
{
   public:
      void start();
      void stop();
      static NetworkForwarder* getInstance();
      void forward_output (MsgType t, const std::ostringstream &s, const char *file, unsigned int line, const char *func);
   protected:
      NetworkForwarder();
      ~NetworkForwarder();
   private:
      yarp::os::BufferedPort<yarp::os::Bottle>* outputPort;
      private:
      NetworkForwarder(NetworkForwarder const&){};
      NetworkForwarder& operator=(NetworkForwarder const&){};
      static NetworkForwarder* instance;
};
*/
class Debug {
    struct Stream {
        Stream(MsgType t, const char *fn, unsigned int l, const char *f) : type(t), file(fn), line(l), func(f), ref(1) {}
        std::ostringstream oss;
        MsgType type;
        const char *file;
        unsigned int line;
        const char *func;
        int ref;
    } *stream;
public:

    inline Debug(MsgType type,
                 const char *file,
                 unsigned int line,
                 const char *func) :
        stream(new Stream(type, file, line, func))
    {
    }

    inline Debug(const Debug &o) : stream(o.stream) {
        ++stream->ref;
    }

    inline ~Debug()
    {
        if (!--stream->ref)
        {
            print_output(stream->type, stream->oss, stream->file, stream->line, stream->func);
            forward_output(stream->type, stream->oss, stream->file, stream->line, stream->func);
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

#ifndef YARP_CONSTSTRING_IS_STD_STRING
    inline Debug& operator<<(const std::string &t) {
        stream->oss << t;
        stream->oss << ' ';
        return *this;
    }
#endif

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

private:
    void print_output(MsgType t,
                      const std::ostringstream &s,
                      const char *file,
                      unsigned int line,
                      const char *func);
    void forward_output(MsgType t,
                      const std::ostringstream &s,
                      const char *file,
                      unsigned int line,
                      const char *func);
    static bool colored_output;
    static bool trace_output;
};

}

#ifdef __GNUC__
#define __YFUNCTION__ __PRETTY_FUNCTION__
#else // __GNUC__
//#define __YFUNCTION__ __func__
#define __YFUNCTION__ ""
#pragma message("WARNING-> acemor redefined __YFUNCTION__ because compilation fails on visual studio 2012")
#endif // __GNUC__

#define yTrace()   LogStream::Debug(LogStream::TraceType,   __FILE__, __LINE__, __YFUNCTION__)
#define yDebug()   LogStream::Debug(LogStream::DebugType,   __FILE__, __LINE__, __YFUNCTION__)
#define yWarning() LogStream::Debug(LogStream::WarningType, __FILE__, __LINE__, __YFUNCTION__)
#define yError()   LogStream::Debug(LogStream::ErrorType,   __FILE__, __LINE__, __YFUNCTION__)
#define yFatal()   LogStream::Debug(LogStream::FatalType,   __FILE__, __LINE__, __YFUNCTION__)
#define yInfo()    LogStream::Debug(LogStream::InfoType,    __FILE__, __LINE__, __YFUNCTION__)
#define YFIXME_NOTIMPLEMENTED(what) yWarning() << "FIXME:" << what << "not yet implemented()";

template <typename T>
inline std::ostringstream& operator<<(std::ostringstream &oss, const std::vector<T> &t)
{
    for (typename std::vector<T>::const_iterator it = t.begin(); it != t.end(); it++) {
        const T &p = *it;
        oss << p << ' ';
    }
    return oss;
}

#endif // LogStream_DEBUG_H

// eof


