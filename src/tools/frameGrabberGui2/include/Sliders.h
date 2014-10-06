// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SLIDERS_H__
#define __SLIDERS_H__

#include <gtkmm.h>
#include <yarp/dev/RemoteFrameGrabberDC1394.h>

typedef enum {
  DC1394_FEATURE_BRIGHTNESS=0,
  DC1394_FEATURE_EXPOSURE,
  DC1394_FEATURE_SHARPNESS,
  DC1394_FEATURE_WHITE_BALANCE,
  DC1394_FEATURE_HUE,
  DC1394_FEATURE_SATURATION,
  DC1394_FEATURE_GAMMA,
  DC1394_FEATURE_SHUTTER,
  DC1394_FEATURE_GAIN,
  DC1394_FEATURE_IRIS,
  DC1394_FEATURE_FOCUS,
  DC1394_FEATURE_TEMPERATURE,
  DC1394_FEATURE_TRIGGER,
  DC1394_FEATURE_TRIGGER_DELAY,
  DC1394_FEATURE_WHITE_SHADING,
  DC1394_FEATURE_FRAME_RATE,
  DC1394_FEATURE_ZOOM,
  DC1394_FEATURE_PAN,
  DC1394_FEATURE_TILT,
  DC1394_FEATURE_OPTICAL_FILTER,
  DC1394_FEATURE_CAPTURE_SIZE,
  DC1394_FEATURE_CAPTURE_QUALITY
} dc1394feature_id_t;

class FrameGrabberGUIControl2;

class DC1394SliderBase
{
public:
    virtual ~DC1394SliderBase(){}
    static int GetHeight(){ return m_Height; }
    virtual void Refresh()=0;
    virtual void Propagate()=0;
protected:
    static int m_Height;
    bool m_bInactive;
    int m_nInternalChange;
};

class DC1394Slider : public DC1394SliderBase
{
public:
    //DC1394Slider(){}
    virtual ~DC1394Slider();
    DC1394Slider(dc1394feature_id_t feature,char* label,Gtk::VBox &vbox,FrameGrabberGUIControl2 *fg);
    
    void Refresh();
    void Propagate();

    void slider_handler();
    void onepush_handler();
    void auto_handler();
    void pwr_handler();
    void set_value(double val);

protected:
    double m_old_value,m_new_value;
    FrameGrabberGUIControl2 *pFG;
    dc1394feature_id_t m_Feature;
    Gtk::CheckButton* pPwr;
    Gtk::RadioButton *pRBa,*pRBm;
    Gtk::HScale m_Slider;
    Gtk::Button m_OnePush;
    Glib::ustring m_Name;
};

class DC1394SliderWB : public DC1394SliderBase
{
public:
    virtual ~DC1394SliderWB();
    DC1394SliderWB(Gtk::VBox &vbox,yarp::dev::RemoteFrameGrabberControlsDC1394 *fg);

    void Refresh();
    void Propagate();
    
    void slider_handler();
    void onepush_handler();
    void automan_handler();
    void pwr_handler();
    void set_value(double blue,double red);

protected:
    double m_old_red,m_new_red,m_old_blu,m_new_blu;
    yarp::dev::RemoteFrameGrabberControlsDC1394 *pFG;
    Gtk::CheckButton* pPwr;
    Gtk::RadioButton *pRBa,*pRBm;
    Gtk::HScale m_Red,m_Blue;
    Gtk::Button m_OnePush;
};

#endif
