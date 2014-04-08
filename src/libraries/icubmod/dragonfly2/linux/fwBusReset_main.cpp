// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//  L      I  N   N  U   U  X   X
//  L      I  NN  N  U   U   X X
//  L      I  N N N  U   U    X
//  L      I  N  NN  U   U   X X
//  LLLLL  I  N   N   UUU   X   X


#include <stdio.h>
#include <stdlib.h>
#include "FirewireCameraDC1394-DR2_2.h"

int main(int argc,char** argv)
{
    int wait_msec=10000;

    fprintf(stderr,"FireWire bus reset...\n");
    if (argc>1) wait_msec=atoi(argv[1]);                
    CFWCamera_DR2_2::busReset(0,0.001*double(wait_msec));
    fprintf(stderr,"...done.\n");

    return 0;    
}
