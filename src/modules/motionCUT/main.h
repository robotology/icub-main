/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Carlo Ciliberto, Ugo Pattacini
 * email:   carlo.ciliberto@iit.it, ugo.pattacini@iit.it
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

/**
@ingroup icub_module

\defgroup motionCUT motionCUT
 
Detects independent moving points of a grid used to sample the 
input images. The algorithm works also with moving cameras.

Copyright (C) 2010 RobotCub Consortium
 
Authors: Carlo Ciliberto and Ugo Pattacini 
 
Date: first release on the night of 03/05/2010 :)

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module exploits the pyramidal Lucas-Kanade algorithm to
detect independent moving points over a selectable grid of 
nodes. The algorithm is designed in such a way that it works 
also - and especially - with moving cameras. 
 
\note the name motionCUT stands for <i>motion Cover/Uncover 
      Trick</i> and refers to its working principle that is
      detailed in the paper: Ciliberto C., Pattacini U., Natale
      L., Nori F. and Metta G., "Reexamining Lucas-Kanade Method
      for Real-Time Independent Motion Detection:
      Application to the iCub Humanoid Robot", <i>IEEE/RSJ
      International Conference on Intelligent Robots and
      Systems</i>, San Francisco, CA, USA, 2011.
 
\note <b>If you're going to use the motionCUT for your work, 
      please quote it within any resulting publication</b>.
 
\note We warmly suggest to use OpenCV in conjunction with 
      multi-threading layers such as OpenMP and TBB in order to
      achieve high performances for motion detection. Refer to
      the OpenCV documentation for the details.
 
\note A video on iCub employing \e motionCUT can be seen <a
      href="http://www.youtube.com/watch?v=Ql8Qe0oxHaY">here</a>.
 
\section lib_sec Libraries 
YARP libraries and OpenCV

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.
 
--coverXratio \e ratioX
- The parameter \e ratioX identifies the portion of the x-axis 
  of the image covered by the grid nodes. Example: if
  ratioX=0.75, then the central 3/4 of the x-axis will be
  covered with points.
 
--coverYratio \e ratioY
- The analogous for the y-axis image.
 
--nodesStep \e step
- The parameter \e step selects the step in pixels between two 
  consecutive grid nodes.
 
--winSize \e size
- The parameter \e size selects window size used by the 
  algorithm.
 
--recogThres \e thres
- The parameter \e thres, given in percentage, specifies the 
  error threshold that allows discriminating between background
  and independent moving nodes as result of a matching carried
  out on the windows whose size is determined by \e winSize
  parameter. Usually very small values, such as 0.5%, have to be
  used. Indicatively, a value of 0.5% means that the two
  templates for the matching must have a similarity measure of
  99.5% to prevent the relative node from being recognized as
  independent moving node.
 
--adjNodesThres \e min 
- This parameter allows filtering out the \e salt-and-pepper 
  noise over the output image, by specifying the minimum number
  of adjacent nodes that must be active (i.e. that undergo the
  motion) in the neighbourhood of any single node to keep it
  active.
 
--blobMinSizeThres \e min 
- This parameter allows filtering out blobs whose nodes number 
  is lower than <min>.

--framesPersistence \e frames
- This parameter allows increasing the node persistence over 
  consecutive frames implementing a sort of low-pass filter. The
  value \e frames specifies the number of consecutive frames for
  which if a node gets active it is kept on.
 
--cropRadius \e radius 
- This parameter allows changing the size of a cropping window
  around the center of the largest blob detected, which will be
  then sent out through the corresponding port.
 
--numThreads \e threads
- This parameter allows controlling the maximum number of 
  threads allocated by parallelized OpenCV functions. This
  option is available only if the OpenMP layer is supported. By
  contrast, the TBB layer automatically determines the number of
  threads.\n
  \e #  > 0 : assign # threads to OpenCV; \n
  \e # == 0 : assign all threads to OpenCV; \n
  \e #  < 0 : assign all threads but # to OpenCV; \n
  The default value is -1 meaning that all threads equal to the
  number of available cores BUT ONE will be used.
 
--verbosity 
- Enable the dump of log messages.
 
\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
- <i> /<stemName>/img:i </i> accepts the incoming images. 
 
- <i> /<stemName>/img:o </i> outputs the input images with the 
  grid layer on top. This port propagates the time-stamp carried
  by the input image.
 
- <i> /<stemName>/nodes:o </i> outputs the x-y location of the 
  currently active nodes in this format: (nodesStep <val>)
  (<n0.x> <n0.y>) (<n1.x> <n1.y>) ... . This port propagates the
  time-stamp carried by the input image.
 
- <i> /<stemName>/blobs:o </i> outputs the x-y location of blobs
  centroids along with their size in this format: (<b0.cx>
  <b0.cy> <b0.size>) (<b1.cx> <b1.cy> <b1.size>) ... The output
  blobs list is sorted according to their size (decreasing
  order). This port propagates the time-stamp carried
  by the input image.

- <i> /<stemName>/crop:o </i> outputs a window of fixed size obtained from
  a ROI around the center of mass of the largest blob detected.
 
- <i> /<stemName>/opt:o </i> outputs monochrome images 
  containing just the grid nodes signalling independent
  movements. This port propagates the time-stamp carried
  by the input image.
 
- <i> /<stemName>/rpc </i> for RPC communication. 
 
\section rpcProto_sec RPC protocol 
The parameters <i> winSize, recogThres, adjNodesThres, 
framesPersistence, numThreads, verbosity </i> can be changed/retrieved 
through the commands set/get. Moreover the further switch \e 
inhibition can be accessed in order to enable/disable the motion 
detection at run-time. 
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux and Windows.

\author Carlo Ciliberto and Ugo Pattacini
*/ 

#include <stdio.h>
#include <string>
#include <set>
#include <vector>
#include <deque>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


