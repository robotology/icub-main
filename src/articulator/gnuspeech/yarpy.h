// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef YARPY_INC
#define YARPY_INC

#ifdef __cplusplus
extern "C" {
#endif

#include "structs.h"

  void yarpy();

  void setParams(TRMParameters *params);

  double getParam(int x, int y);

  double getTime();

  void setParamTarget(int x, int y, double v);

#ifdef __cplusplus
};
#endif

#endif
