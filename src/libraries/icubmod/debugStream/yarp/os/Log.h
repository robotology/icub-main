/*
 * Copyright (C) 2012-2014  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef _YARP2_LOGGING_
#define _YARP2_LOGGING_

#include <yarp/os/api.h>
#include <iosfwd>

#ifndef YARP_NO_DEPRECATED
# include <cstdio>
# include <cstdlib>
# include <yarp/os/ConstString.h>
#endif

#if defined (__GNUC__)
# define __YFUNCTION__ __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
# define __YFUNCTION__ __FUNCSIG__
#elif (__cplusplus <= 199711)
# define __YFUNCTION__ __func__
#else
# define __YFUNCTION__ "(unknown function)"
#endif // __GNUC__

namespace yarp {
namespace os {


class LogStream;

class YARP_OS_API Log
{
public:

    enum LogType {
        TraceType,
        DebugType,
        InfoType,
        WarningType,
        ErrorType,
        FatalType
    };

    Log(const char *file,
                  const unsigned int line,
                  const char *func);
    Log();

    void trace(const char *msg, ...) const;
    void debug(const char *msg, ...) const;
    void info(const char *msg, ...) const;
    void warning(const char *msg, ...) const;
    void error(const char *msg, ...) const;
    void fatal(const char *msg, ...) const;

    LogStream trace() const;
    LogStream debug() const;
    LogStream info() const;
    LogStream warning() const;
    LogStream error() const;
    LogStream fatal() const;

    /*!
     * \brief Set the output file used by yTrace()
     */
    static void setTraceFile(const std::string &filename);

    /*!
     * \brief Set the output file used by yDebug() and yInfo()
     */
    static void setOutputFile(const std::string &filename);

    /*!
     * \brief Set the output file used by yWarning(), yError() and yFatal()
     */
    static void setErrorFile(const std::string &filename);

private:
    const char *file;
    const unsigned int line;
    const char *func;


    static void print_output(LogType t,
                             const char *msg,
                             const char *file,
                             unsigned int line,
                             const char *func);

    static std::ofstream ftrc; /// Used by yTrace()
    static std::ofstream fout; /// Used by yDebug() and yInfo()
    static std::ofstream ferr; /// Used by yWarning(), yError() and yFatal()

    static bool colored_output;
    static bool verbose_output;
    static bool trace_output;

    friend class LogStream;

#ifndef YARP_NO_DEPRECATED
    YARP_DEPRECATED virtual void debug(const ConstString& txt) const {}; ///< \deprecated since YARP 2.3.64
    YARP_DEPRECATED virtual void info(const ConstString& txt) const {}; ///< \deprecated since YARP 2.3.64
    YARP_DEPRECATED virtual void warning(const ConstString& txt) const {}; ///< \deprecated since YARP 2.3.64
    YARP_DEPRECATED virtual void error(const ConstString& txt) const {}; ///< \deprecated since YARP 2.3.64
    YARP_DEPRECATED virtual void fail(const ConstString& txt) const {}; ///< \deprecated since YARP 2.3.64
#endif // YARP_NO_DEPRECATED

}; // class Log

} // namespace os
} // namespace yarp


#define yTrace   yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).trace
#define yDebug   yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).debug
#define yInfo    yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).info
#define yWarning yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).warning
#define yError   yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).error
#define yFatal   yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).fatal

#define YARP_FIXME_NOTIMPLEMENTED(what) yWarning("FIXME: %s not yet implemented", what);

/**
 * Very basic assertion macro.
 */
#define YARP_ASSERT(x) if (!(x)) { yFatal("Assertion failure (%s)", #x); }


#ifndef YARP_NO_DEPRECATED

/**
 * Low level function for printing a stack trace, if implemented (gcc/Linux).
 *
 * \deprecated
 */
YARP_OS_API YARP_DEPRECATED void yarp_print_trace(FILE *out, const char *file, int line);


YARP_OS_API YARP_DEPRECATED void __yarp_error(const yarp::os::ConstString& str); ///< \deprecated since YARP 2.3.64
YARP_OS_API YARP_DEPRECATED void __yarp_warn(const yarp::os::ConstString& str); ///< \deprecated since YARP 2.3.64
YARP_OS_API YARP_DEPRECATED void __yarp_info(const yarp::os::ConstString& str); ///< \deprecated since YARP 2.3.64
YARP_OS_API YARP_DEPRECATED void __yarp_debug(const yarp::os::ConstString& str); ///< \deprecated since YARP 2.3.64

YARP_OS_API YARP_DEPRECATED bool yarp_show_error(); ///< \deprecated since YARP 2.3.64
YARP_OS_API YARP_DEPRECATED bool yarp_show_warn(); ///< \deprecated since YARP 2.3.64
YARP_OS_API YARP_DEPRECATED bool yarp_show_info(); ///< \deprecated since YARP 2.3.64
YARP_OS_API YARP_DEPRECATED bool yarp_show_debug(); ///< \deprecated since YARP 2.3.64

/**
 * \deprecated since YARP 2.3.64
 */
#define YARP_LOG_ERROR(x) __yarp_error(x)

/**
 * \deprecated since YARP 2.3.64
 */
#define YARP_LOG_WARN(x) __yarp_warn(x)

/**
 * \deprecated since YARP 2.3.64
 */
#define YARP_LOG_INFO(x) __yarp_info(x)

/**
 * \deprecated since YARP 2.3.64
 */
#define YARP_LOG_DEBUG(x) __yarp_debug(x) ///< \deprecated

#endif // YARP_NO_DEPRECATED

#endif // _YARP2_LOGGING_
