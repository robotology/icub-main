/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "multiFrameViewer.h"


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

  if (!autoclear){
    //THIS IS FOR the 'MOSAIC' CLASS TO LEAVE BACKGROUND
    //ACCROSS AUGMENTATION. ONLY WORKS ON 1 IMAGE:

    //print only the new info in first image rectangle
    //without removing (old) content elsewhere: 
    repaint(locations[0],locations[1],viewframes[0]->width(),viewframes[0]->height());
  }
  else{
    //clear background and repaint all images:
    repaint();
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

