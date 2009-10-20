/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "multiFrameViewer.h"
#include <timing.h>


iCub::contrib::primateVision::multiFrameViewer::multiFrameViewer(int w_, int h_,QWidget *parent, const char *name)
        : QWidget(parent, name)
{

        if (w_!=0 && h_!=0){
         setFixedSize(w_,h_);  
        }
	
	//p = new QPainter(this);

	autoclear = true;

  //printf("MFV created\n");
       
} 

void iCub::contrib::primateVision::multiFrameViewer::showViews(int numviews_, QImage **viewframes_, int *locations_)
{

  //printf("MFV:showViews \n");
  
  locations = locations_;
  viewframes = viewframes_;
  numviews = numviews_;

  if (autoclear){
    //clear background:
    repaint();
  }
  else{
    //print only the new info, without removing old content: 
    for (int i = 0;i<numviews;i++){
      repaint(locations[i*2],locations[i*2+1],viewframes[i]->width(),viewframes[i]->height());
    }
  }
  

}


void iCub::contrib::primateVision::multiFrameViewer::paintEvent( QPaintEvent *){

  //draw all passed images within the requested repainted area:
  for (int i = 0;i<numviews;i++){
    bitBlt(this,locations[i*2],locations[i*2+1],viewframes[i],0,0,0,0,1);
  }
  
}

iCub::contrib::primateVision::multiFrameViewer::~multiFrameViewer()
{

}

