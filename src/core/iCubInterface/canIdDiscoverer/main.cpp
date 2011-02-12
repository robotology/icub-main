/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include "canIdDiscoverer.h"
#include <stdio.h>

int main(int argc, const char **argv)
{
  CanIdDiscoverer d;

  if ((argc!=3)||(std::string(argv[1])!=std::string("--device")))
      {
          printf("Please specify the device you want to use (es: --device pcan)\n");
          return -1;
      }
      
  ICUB_CAN_IDS ids=d.discover(argv[2]);

  ids.print();

  return 0;
}
