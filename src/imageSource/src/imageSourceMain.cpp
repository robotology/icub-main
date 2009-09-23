/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


/*
 * Audit Trail
 * -----------
 * 21/09/09  First version validated   DV
 */ 

#include "iCub/imageSource.h"

int main(int argc, char * argv[])
{
   /* initialize yarp network */

   Network yarp;

   /* create your module */

   ImageSource imageSource; 

   /* prepare and configure the resource finder */

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("imageSource.ini"); //overridden by --from parameter
   rf.setDefaultContext("imageSource/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */

   imageSource.runModule(rf);

   return 0;
}
