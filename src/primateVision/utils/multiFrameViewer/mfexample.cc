/**
 * @ingroup icub_primatevision_utils_multiframeviewer
 * \defgroup icub_primatevision_utils_multiframeviewer_mfexample MFExample
 *
 * An example application for the MultiFrameViewer display class.
 *
 * This module should be called in the following way:\n \b mfexample \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/multiframeviewer/mfexample.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


/**************************************************************
             mfexample:  Andrew Dankers 2003
***************************************************************
Example that uses  multiFrameViewer to display images. 
multiFrameViewer is used to 
display image from file show that it works with any QImage.
It can be used on any depth of QImage: grey, colour etc.
**************************************************************/    

#include "multiFrameViewer.h"
#include <qapplication.h>
#include <qimage.h>


#define NUMIMAGES 2 //number of displayed images in each mfv

using namespace iCub::contrib::primateVision;

int main(int argc, char **argv)
{
  QApplication *a = new QApplication(argc, argv);
  
  //data structures:
  QImage *colourq = new QImage("mf.jpg","JPEG");
   
  //setup viewers:
  multiFrameViewer *mfv1 = new multiFrameViewer();
  mfv1->setCaption("MF Example, first window");
  mfv1->show();

  multiFrameViewer *mfv2 = new multiFrameViewer(colourq->width()*2,colourq->height());
  mfv2->setCaption("MF Example, second window, fixed size.");
  mfv2->show();
  
  QImage *displayimages[NUMIMAGES]={colourq,colourq};
  int locations[NUMIMAGES*2]={0,0,colourq->width(),0};


  while(1){ 
    //loop so that if the 
    //window is moved, the images are
    //re-sent to display  
    mfv1->showViews(NUMIMAGES,displayimages,locations);
    mfv2->showViews(NUMIMAGES,displayimages,locations);
    usleep(50000);
  }
  
  //clean up and exit
  //  delete greyq;
  delete colourq;
  return a->exec();
}

