// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__
#define __GTKMM_ICUB_INTERFACE_GUI_CLIENT_H__

#include <gtkmm.h>
#include <yarp/os/all.h>
#include <yarp/dev/RemoteFrameGrabber.h>

class iCubFocusGuiThread : public yarp::os::Thread
{
public:
    iCubFocusGuiThread(yarp::os::Searchable &config)
    {
        mR=mD=0.0;
        
        //yarp::os::Property p;
        //p.put("local",loc);
        //p.put("remote",rem);
        // grabber
        pFG=new yarp::dev::RemoteFrameGrabber();
        pFG->open(config);
    }

    ~iCubFocusGuiThread()
    {
        if (pFG)
        {
            pFG->close();
            delete pFG;
            pFG=NULL;
        }
    }

    double getValue(){ return mR; }
    
    void run()
    { 
        while (!isStopping())
        {
            if (pFG->getImage(img))
            {
                int mW=img.width();
                int mH=img.height();
                unsigned char *p=img.getRawImage();

                double Rx,Gx,Bx;
                double Ry,Gy,By;
                double D=0.0;

                for (int y=1; y<mH-1; ++y)
                {
                    for (int x=1; x<mW-1; ++x)
                    {
                        Rx=p[R(x+1,y-1)]-p[R(x-1,y-1)]+2*(p[R(x+1,y)]-p[R(x-1,y)])+p[R(x+1,y+1)]-p[R(x-1,y+1)];
                        Ry=p[R(x-1,y+1)]-p[R(x-1,y-1)]+2*(p[R(x,y+1)]-p[R(x,y-1)])+p[R(x+1,y+1)]-p[R(x+1,y-1)];

                        Gx=p[G(x+1,y-1)]-p[G(x-1,y-1)]+2*(p[G(x+1,y)]-p[G(x-1,y)])+p[G(x+1,y+1)]-p[G(x-1,y+1)];
                        Gy=p[G(x-1,y+1)]-p[G(x-1,y-1)]+2*(p[G(x,y+1)]-p[G(x,y-1)])+p[G(x+1,y+1)]-p[G(x+1,y-1)];

                        Bx=p[B(x+1,y-1)]-p[B(x-1,y-1)]+2*(p[B(x+1,y)]-p[B(x-1,y)])+p[B(x+1,y+1)]-p[B(x-1,y+1)];
                        By=p[B(x-1,y+1)]-p[B(x-1,y-1)]+2*(p[B(x,y+1)]-p[B(x,y-1)])+p[B(x+1,y+1)]-p[B(x+1,y-1)];

                        D+=Rx*Rx+Gx*Gx+Bx*Bx+Ry*Ry+Gy*Gy+By*By;
                    }
                }

                if (mD<D) mD=D;

                mR=D/mD;

                mSigWindow();  
            }
        }
    }
    
    Glib::Dispatcher mSigWindow;
    
protected:
    int mW,mH;
    
    double mR;
    double mD;

    yarp::dev::RemoteFrameGrabber *pFG;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> img;

    inline int R(int x,int y){ return (y*mW+x)*3;   }
    inline int G(int x,int y){ return (y*mW+x)*3+1; }
    inline int B(int x,int y){ return (y*mW+x)*3+2; }
};

class iCubFocusGuiWindow : public Gtk::Window
{
public:
    iCubFocusGuiWindow(yarp::os::Searchable &config)
    {
        // gui
        set_title("Camera focus GUI");
        set_border_width(5);
        set_default_size(512,100);

        add(mVBox);
        mVBox.pack_start(mPB);
        show_all();

        //yarp::os::Time::delay(1.0);
        
        mThread=new iCubFocusGuiThread(config);
        mThread->mSigWindow.connect(sigc::mem_fun(*this,&iCubFocusGuiWindow::run));
        mThread->start();
    }

    virtual ~iCubFocusGuiWindow()
    {
        if (mThread)
        {
            mThread->stop();
            delete mThread;
            mThread=NULL;
        }
    }

    void run()
    {
        mPB.set_fraction(mThread->getValue());
    }

protected:
    double mD;
    Gtk::VBox mVBox;
    Gtk::ProgressBar mPB;
    iCubFocusGuiThread* mThread;
};

#endif