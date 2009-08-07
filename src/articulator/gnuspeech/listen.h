// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef LISTEN_INC
#define LISTEN_INC

#include <yarp/os/NetInt32.h>
#ifdef YARP2_WINDOWS
#define int16_t __int16
#endif

#ifdef __cplusplus
extern "C" {
#endif

  void init_listen(const char *name);

  void plisten(unsigned char *sample, int len);

#ifdef __cplusplus
};
#endif

#endif
