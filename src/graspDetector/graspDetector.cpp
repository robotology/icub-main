// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "graspDetector.h"

graspDetector::graspDetector(int n, fingerDetector **fingDet, int rate): RateThread(rate)
{
    nFingers = n;
    fd = fingDet;
}

graspDetector::~graspDetector()
{    }

bool graspDetector::threadInit()
{
    return 1;
}

void graspDetector::run()
{
  fprintf(stderr, "Current grasp status is: ");
  for(int i=0; i < nFingers; i++)
    fprintf(stderr, " %d", (int) fd[i]->status);
  fprintf(stderr, "\n");
}

void graspDetector::stop()
{
 
}
