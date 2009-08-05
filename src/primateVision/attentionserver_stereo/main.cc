/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_attentionserver_stereo AttnServer_Stereo
 *  
 *This AttnServer_Stereo module connects to the SalServerL, IORServerL, and TSBServerL; as well as the SalServerR, IORServerR, and TSBServerR to receive saliency, inhibition-of-return, and task-dependent spatial bias data, for the construction of dual mono fixation maps that are scanned for attentional peaks. Fixational peaks in both left and right maps are moderated by several mechanisms, and an attentional peak is selected and returned. Before saccade is initiated, stereo cross-checking is conducted to ensure the saccade motion commands will result in both cameras attending the exact same scene point. Where cross-checking fails, both cameras conduct equi-angular saccade, and it is expected that the ZDFServer (when run in parallel), will tune fixation upon the object that elicited the attentional response, segmenting and tracking it.  The recserver can be contacted by an AttnClient_stereo to initiate attentional saccade to the attention point.
 *
 * This module should be called in the following way:\n \b attnserver_stereo \b -c \a attnS.cfg\n
 * where attnS.cfg is the config file.  Default configfile in primateVision/config/attnS.cfg is used if none specified.
 *
 *
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/motion
 * <li> /salserverL/sal
 * <li> /iorserverL/ior
 * <li> /tsbserverL/tsb
 * <li> /salserverR/sal
 * <li> /iorserverR/ior
 * <li> /tsbserverR/tsb
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /attnserverS/input/sal_l
 * <li> /attnserverS/input/ior_l
 * <li> /attnserverS/input/tsb_l
 * <li> /attnserverS/input/sal_r
 * <li> /attnserverS/input/ior_r
 * <li> /attnserverS/input/tsb_r
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /attnserverS/output/attn
 * <li> /attnserverS/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/attentionserver_stereo/main.cc 
 * 
 */


/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include "attn_st.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/attnS.cfg");


  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }


  //Create & launch flow processing thread:
  AttnServer_Stereo*s = new AttnServer_Stereo(&fname);


  return a.exec();

  
}
