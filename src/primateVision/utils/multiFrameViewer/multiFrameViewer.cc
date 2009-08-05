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

  //printf("MFV created\n");
       
} 

void iCub::contrib::primateVision::multiFrameViewer::showViews(int numviews_, QImage **viewframes_, int *locations_)
{

        //printf("MFV:showViews \n");

        locations = locations_;
        viewframes = viewframes_;
        numviews = numviews_;

        repaint();
}


void iCub::contrib::primateVision::multiFrameViewer::paintEvent( QPaintEvent *){

    //printf("paintEvent \n");

    QPainter p(this);

    for (int i = 0;i<numviews;i++){
      p.drawImage( QPoint(locations[i*2],locations[i*2+1]), *viewframes[i]);
    }
}

iCub::contrib::primateVision::multiFrameViewer::~multiFrameViewer()
{

}

