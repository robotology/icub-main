// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __FRAME_GRABBER_GUI_CONTROL2_H__
#define __FRAME_GRABBER_GUI_CONTROL2_H__

static const char *video_mode_labels[]={
    "160x120 YUV444",
    "320x240 YUV422",
    "640x480 YUV411", 
    "640x480 YUV422",
    "640x480 RGB8",
    "640x480 MONO8",
    "640x480 MONO16",
    "800x600 YUV422",
    "800x600 RGB8",
    "800x600_MONO8",
    "1024x768 YUV422",
    "1024x768 RGB8",
    "1024x768 MONO8",
    "800x600 MONO16",
    "1024x768 MONO16",
    "1280x960 YUV422",
    "1280x960 RGB8",
    "1280x960_MONO8", 
    "1600x1200 YUV422",
    "1600x1200 RGB8",
    "1600x1200 MONO8",
    "1280x960 MONO16",
    "1600x1200_MONO16",
    "EXIF",
    "FORMAT7 0",
    "FORMAT7 1",
    "FORMAT7 2",
    "FORMAT7 3",
    "FORMAT7 4",
    "FORMAT7 5",
    "FORMAT7 6",
    "FORMAT7 7"
};

static const char *video_rate_labels[]={"1.875 fps","3.75 fps","7.5 fps","15 fps","30 fps","60 fps","120 fps","240 fps"};

static const char *color_coding_labels[]={"MONO8","YUV411","YUV422","YUV444","RGB8","MONO16","RGB16","MONO16S","RGB16S","RAW8","RAW16"};

static const char *iso_speed_labels[]={"100 Mbps","200 Mbps","400 Mbps","800 Mbps","1600 Mbps","3200 Mbps"};

static const char *op_mode_labels[]={"LEGACY","1394b"};

#include <gtkmm.h>
#include <yarp/dev/RemoteFrameGrabberDC1394.h>
#include "Sliders.h"

/**
 *  A graphical control interface for a remote framegrabber.    
 */
class FrameGrabberGUIControl2 : public Gtk::Window, virtual public yarp::dev::RemoteFrameGrabberControlsDC1394
{
public:
    /**
     *  Constructor.
     *
     *  Creates a graphical control interface for a remote framegrabber.
     *  @param loc local yarp port
     *  @param rem remote framegrabber yarp port 
     */
    FrameGrabberGUIControl2(char* loc, char* rem) : 
      // base classes
      Gtk::Window(),
      yarp::dev::RemoteFrameGrabberControlsDC1394(),
      
      //members   
      m_Refresh1("Refresh"),
      m_Refresh2("Refresh"),
      m_broadcast("Broadcast"),
      m_propagate("Propagate"),
      m_LabelControls1("Features"),
      m_LabelControls2("Features (adv)"),      
      m_LabelFormat("Format"),
      m_LabelCamera("Camera"),
      m_LabelMaxXDim("Max width"),
      m_LabelMaxYDim("Max height"),  
      m_xdim(2.0,0),
      m_ydim(2.0,0),
      m_xoff(2.0,0),
      m_yoff(2.0,0),
      m_bpp(1.0,0),
      m_power("Camera Power"),
      m_transmission("Transmission"),
      m_reset("Reset camera"),
      m_defaults("Load default params")
    {
        // yarp2
        yarp::os::Property config;
        config.put("remote",rem);
        config.put("local",loc);

        open(config);

        // GTKMM
        set_title("Grabber Remote GUI");
        
        // REFRESH & BROADCAST

        /*
        m_propagate.set_sensitive(false);
        pHBox->pack_start(m_propagate,Gtk::PACK_EXPAND_PADDING);
        
        pHBox->pack_start(m_broadcast,Gtk::PACK_EXPAND_PADDING);
        m_VBoxFeat1.pack_start(*pHBox,Gtk::PACK_SHRINK,12);
        */

        //
        m_MenuColorCodingCNT=0;
        m_bppCNT=0;

        // FEATURES
        m_nFeatures=0;
        
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_SHUTTER,(char*)"Shutter",m_VBoxFeat1,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_BRIGHTNESS,(char*)"Brightness",m_VBoxFeat1,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_GAIN,(char*)"Gain",m_VBoxFeat1,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_EXPOSURE,(char*)"Exposure",m_VBoxFeat1,this);
        m_pSli[m_nFeatures++]=new DC1394SliderWB(m_VBoxFeat1,this);

        int height_panel_1=DC1394SliderBase::GetHeight();

        Gtk::HBox *pHBox=new Gtk::HBox();
        pHBox->pack_start(m_Refresh1,Gtk::PACK_EXPAND_PADDING);
        m_VBoxFeat1.pack_start(*pHBox,Gtk::PACK_SHRINK,12);

        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_SHARPNESS,(char*)"Sharpness",m_VBoxFeat2,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_HUE,(char*)"Hue",m_VBoxFeat2,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_SATURATION,(char*)"Saturation",m_VBoxFeat2,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_GAMMA,(char*)"Gamma",m_VBoxFeat2,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_IRIS,(char*)"Iris",m_VBoxFeat2,this);
        m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_FOCUS,(char*)"Focus",m_VBoxFeat2,this);
        //m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_TEMPERATURE,(char*)"Temperature",m_VBoxFeat2,this);
        //m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_TRIGGER,(char*)"Trigger",m_VBoxFeat2,this);
        //m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_TRIGGER_DELAY,(char*)"Trigger delay",m_VBoxFeat2,this);
        //m_pSli[m_nFeatures++]=new DC1394Slider(DC1394_FEATURE_WHITE_SHADING,(char*)"White shading",m_VBoxFeat2,this);
    
        int height_panel_2=DC1394SliderBase::GetHeight()-height_panel_1;

        pHBox=new Gtk::HBox();
        pHBox->pack_start(m_Refresh2,Gtk::PACK_EXPAND_PADDING);
        m_VBoxFeat2.pack_start(*pHBox,Gtk::PACK_SHRINK,12);

        m_Notebook.append_page(m_VBoxFeat1,m_LabelControls1);
        m_Notebook.append_page(m_VBoxFeat2,m_LabelControls2);
    
        // FORMAT
        m_MenuMode.set_name("Video Format");
        unsigned int mask=getVideoModeMaskDC1394();
        printf("VIDEO MODE MASK %x\n",mask);
        for (int i=0,e=0; i<32; ++i)
        {
            if ((1<<i) & mask)
            {
                m_VideoModeLut[e]=i;
                m_MenuMode.insert_text(e++,video_mode_labels[i]);
            }       
        }
        
        pHBox=new Gtk::HBox();
        m_MenuMode.set_size_request(64,32);
        Gtk::Label *pLabel=new Gtk::Label("Video Format");
        pLabel->set_size_request(96,32);
        pHBox->pack_start(*(pLabel),Gtk::PACK_SHRINK,6);
        pHBox->pack_start(m_MenuMode,Gtk::PACK_EXPAND_WIDGET,6);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,2);
                
        m_MenuFPS.set_name("Framerate");
        m_MenuFPS.clear_items();
        mask=getFPSMaskDC1394();
        for (int i=0,e=0; i<8; ++i)
        {
            if ((1<<i) & mask)
            {
                m_FPSLut[e]=i;
                m_MenuFPS.insert_text(e++,video_rate_labels[i]);
            }
        }
        m_MenuFPS.set_size_request(64,32);
        pHBox=new Gtk::HBox();
        pLabel=new Gtk::Label("Framerate");
        pLabel->set_size_request(96,32);
        pHBox->pack_start(*pLabel,Gtk::PACK_SHRINK,6);
        pHBox->pack_start(m_MenuFPS,Gtk::PACK_EXPAND_WIDGET,6);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,2);
                
        m_MenuISO.set_name("ISO Speed");
        m_MenuISO.insert_text(0,iso_speed_labels[0]);
        m_MenuISO.insert_text(1,iso_speed_labels[1]);
        m_MenuISO.insert_text(2,iso_speed_labels[2]);
        //m_MenuISO.insert_text(3,iso_speed_labels[3]);

        m_MenuISO.set_size_request(64,32);
        pHBox=new Gtk::HBox();
        pLabel=new Gtk::Label("ISO Speed");
        pLabel->set_size_request(96,32);
        pHBox->pack_start(*(pLabel),Gtk::PACK_SHRINK,6);
        pHBox->pack_start(m_MenuISO,Gtk::PACK_EXPAND_WIDGET,6);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,2);
        
        // FORMAT 7

        m_VBoxForm.pack_start(*(new Gtk::HSeparator()),Gtk::PACK_SHRINK,16);
        m_VBoxForm.pack_start(*(new Gtk::Label("FORMAT 7")),Gtk::PACK_SHRINK,6);

        m_MenuColorCoding.set_name("Color coding");
        mask=getColorCodingMaskDC1394(getVideoModeDC1394());
       
        printf("COLOR CODING MASK %x\n",mask);

        for (int i=0,e=0; i<32; ++i)
        {
            if ((1<<i) & mask)
            {
                m_ColorCodingLut[e]=i;
                m_MenuColorCoding.insert_text(e++,color_coding_labels[i]);
            }       
        }
        m_MenuColorCoding.set_size_request(64,32);
        pHBox=new Gtk::HBox();
        pLabel=new Gtk::Label("Color Coding");
        pLabel->set_size_request(96,32);
        pHBox->pack_start(*(pLabel),Gtk::PACK_SHRINK,6);
        pHBox->pack_start(m_MenuColorCoding,Gtk::PACK_EXPAND_WIDGET,6);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,8);

        m_VBoxForm.pack_start(*(new Gtk::HSeparator()),Gtk::PACK_SHRINK,16);

        pHBox=new Gtk::HBox();
        pHBox->pack_start(m_LabelMaxXDim,Gtk::PACK_EXPAND_WIDGET,8);
        pHBox->pack_start(m_LabelMaxYDim,Gtk::PACK_EXPAND_WIDGET,8);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,8);

        m_xdim.set_numeric();
        m_ydim.set_numeric();
        m_xoff.set_numeric();
        m_yoff.set_numeric();
    
        m_xdim.set_update_policy(Gtk::UPDATE_IF_VALID);
        m_ydim.set_update_policy(Gtk::UPDATE_IF_VALID);
        m_xoff.set_update_policy(Gtk::UPDATE_IF_VALID);
        m_yoff.set_update_policy(Gtk::UPDATE_IF_VALID);

        pHBox=new Gtk::HBox();
        pHBox->pack_start(*(new Gtk::Label("Width")),Gtk::PACK_EXPAND_WIDGET);
        pHBox->pack_start(m_xdim,Gtk::PACK_EXPAND_WIDGET,8);
        pHBox->pack_start(*(new Gtk::Label("Height")),Gtk::PACK_EXPAND_WIDGET);
        pHBox->pack_start(m_ydim,Gtk::PACK_EXPAND_WIDGET,8);
        pHBox->pack_start(*(new Gtk::Label("X off")),Gtk::PACK_EXPAND_WIDGET);
        pHBox->pack_start(m_xoff,Gtk::PACK_EXPAND_WIDGET,8);
        pHBox->pack_start(*(new Gtk::Label("Y off")),Gtk::PACK_EXPAND_WIDGET);
        pHBox->pack_start(m_yoff,Gtk::PACK_EXPAND_WIDGET,8);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,8);

        m_bpp.set_numeric();
        m_bpp.set_increments(1.0,10.0);
        m_bpp.set_update_policy(Gtk::UPDATE_IF_VALID);
        m_bpp.set_range(0.0,100.0);

        m_VBoxForm.pack_start(*(new Gtk::HSeparator()),Gtk::PACK_SHRINK,16);

        pHBox=new Gtk::HBox();
        Gtk::HBox *packBox=new Gtk::HBox();
        packBox->pack_start(m_bpp,Gtk::PACK_SHRINK,8);
        packBox->pack_start(*(new Gtk::Label("% size bytes per packet")),Gtk::PACK_SHRINK,8);
        pHBox->pack_start(*(packBox),Gtk::PACK_EXPAND_PADDING,8);
        m_VBoxForm.pack_start(*(pHBox),Gtk::PACK_SHRINK,8);
        
        m_Notebook.append_page(m_VBoxForm,m_LabelFormat);

        // CAMERA

        m_MenuOpMode.set_name("Operation Mode");
        m_MenuOpMode.insert_text(0,"LEGACY");
        m_MenuOpMode.insert_text(1,"1394b");
        m_MenuOpMode.set_size_request(64,32);
        pHBox=new Gtk::HBox();
        pLabel=new Gtk::Label("Op. Mode");
        pLabel->set_size_request(96,32);
        pHBox->pack_start(*(pLabel),Gtk::PACK_SHRINK,6);
        pHBox->pack_start(m_MenuOpMode,Gtk::PACK_EXPAND_WIDGET,6);
        m_VBoxCamera.pack_start(*(pHBox),Gtk::PACK_SHRINK,6);

        m_VBoxCamera.pack_start(*(new Gtk::HSeparator()),Gtk::PACK_SHRINK,16);
        pHBox=new Gtk::HBox();
        pHBox->pack_start(m_power,Gtk::PACK_EXPAND_PADDING,6);
        m_VBoxCamera.pack_start(*(pHBox),Gtk::PACK_SHRINK,6);
        //pHBox=new Gtk::HBox();
        pHBox->pack_start(m_transmission,Gtk::PACK_EXPAND_PADDING,6);
        m_VBoxCamera.pack_start(*(pHBox),Gtk::PACK_SHRINK,6);
        pHBox=new Gtk::HBox();
        pHBox->pack_start(m_reset,Gtk::PACK_EXPAND_PADDING,6);
        m_VBoxCamera.pack_start(*(pHBox),Gtk::PACK_SHRINK,6);
        //pHBox=new Gtk::HBox();
        pHBox->pack_start(m_defaults,Gtk::PACK_EXPAND_PADDING,6);
        m_VBoxCamera.pack_start(*(pHBox),Gtk::PACK_SHRINK,6);

        m_power.set_active();

        m_Notebook.append_page(m_VBoxCamera,m_LabelCamera);

        add(m_Notebook);

        show_all_children();

        if (height_panel_2>height_panel_1) height_panel_1=height_panel_2;
        if (height_panel_1<460) height_panel_1=460;
        set_size_request(340,height_panel_1+32);

        Init();
        
        // handlers
        m_Refresh1.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_refresh_pressed));
        m_Refresh2.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_refresh_pressed));
        //m_propagate.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_propagate_pressed));
        //m_broadcast.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_broadcast_change));
        m_MenuMode.signal_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_mode_change));
        m_MenuFPS.signal_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_framerate_change));
        m_MenuISO.signal_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_iso_speed_change));
        m_MenuOpMode.signal_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_operation_mode_change));
        m_MenuColorCoding.signal_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_color_coding_change));
        m_xdim.signal_value_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_format7_window_change));
        m_ydim.signal_value_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_format7_window_change));
        m_xoff.signal_value_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_format7_window_change));
        m_yoff.signal_value_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_format7_window_change));
        m_bpp.signal_value_changed().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_bpp_change));
        m_power.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_power_onoff_change));
        m_transmission.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_transmission_onoff_change));
        m_reset.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_reset_change));
        m_defaults.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl2::on_load_defaults_change));
    }

    /**
     *  Destructor.
     *
     *  Closes the remote framegrabber interface. 
     */
    virtual ~FrameGrabberGUIControl2()
    {
        for (int n=0; n<m_nFeatures; ++n) delete m_pSli[n];

        close();
    }
    
    void Reload()
    {
        unsigned int video_mode=getVideoModeDC1394();

        m_MenuMode.set_active_text(video_mode_labels[video_mode]);
        printf("video mode %s %d\n",video_mode_labels[video_mode],video_mode);

        m_MenuColorCoding.unset_active();        
        m_MenuColorCoding.clear_items();

        unsigned int mask=getColorCodingMaskDC1394(video_mode);

        for (int i=0,e=0; i<32; ++i)
        {
            if ((1<<i) & mask)
            {
                m_ColorCodingLut[e]=i;
                m_MenuColorCoding.insert_text(e++,color_coding_labels[i]);
            }       
        }

        unsigned int xmax,ymax,xstep,ystep,xoffstep,yoffstep;
        getFormat7MaxWindowDC1394(xmax,ymax,xstep,ystep,xoffstep,yoffstep);
        unsigned int xdim=0,ydim=0;
        int x0=0,y0=0;
        getFormat7WindowDC1394(xdim,ydim,x0,y0);
     
        m_xdim.set_sensitive(false);
        m_ydim.set_sensitive(false);
        m_xoff.set_sensitive(false);
        m_yoff.set_sensitive(false);

        if (xstep<2) xstep=2;
        if (ystep<2) ystep=2;
        
        m_xdim.set_range(0,xmax);
        m_ydim.set_range(0,ymax);
        m_xdim.set_increments(xstep,0);
        m_ydim.set_increments(ystep,0);
        m_xdim.set_value(xdim);
        m_ydim.set_value(ydim);

        int xoffMax=(xmax-xdim)/2;
        int yoffMax=(ymax-ydim)/2;

        m_xoff.set_range(-xoffMax,xoffMax);
        m_yoff.set_range(-yoffMax,yoffMax);
        m_xoff.set_increments(xoffstep,0);
        m_yoff.set_increments(yoffstep,0);
        m_xoff.set_value(x0);
        m_yoff.set_value(y0);

        char buff[16];
        sprintf(buff,"Max width %d",xmax);
        m_LabelMaxXDim.set_text(buff);
        sprintf(buff,"Max height %d",ymax);
        m_LabelMaxYDim.set_text(buff); 

        if (mask) // FORMAT 7
        {
            m_MenuColorCoding.set_sensitive(true);
            ++m_MenuColorCodingCNT;
            m_MenuColorCoding.set_active_text(color_coding_labels[getColorCodingDC1394()]);

            m_MenuFPS.set_sensitive(false);
            m_MenuFPS.unset_active();

            m_bpp.set_sensitive(true);
            ++m_bppCNT;
            m_bpp.set_value(getBytesPerPacketDC1394());    
            m_xdim.set_sensitive(true);
            m_ydim.set_sensitive(true); 
            m_xoff.set_sensitive(true);
            m_yoff.set_sensitive(true); 
        }
        else
        {
            m_MenuFPS.set_sensitive(false);
            m_MenuFPS.unset_active();        
            m_MenuFPS.clear_items();
            
            mask=getFPSMaskDC1394();

            for (int i=0,e=0; i<8; ++i)
            {
                if ((1<<i) & mask)
                {
                    m_FPSLut[e]=i;
                    m_MenuFPS.insert_text(e++,video_rate_labels[i]);
                }
            }
             
            m_MenuFPS.set_active_text(video_rate_labels[getFPSDC1394()]);
            m_MenuFPS.set_sensitive(true);   

            m_MenuColorCoding.set_sensitive(false);         
            m_bpp.set_sensitive(false);
        }

        m_MenuISO.set_active_text(iso_speed_labels[getISOSpeedDC1394()]);

        for (int n=0; n<m_nFeatures; ++n) m_pSli[n]->Refresh();
    }

protected:
    int m_nFeat;

    Gtk::VBox m_VBoxFeat1,m_VBoxFeat2; ///< Control sliders container
    Gtk::VBox m_VBoxForm; ///< Control sliders container
    Gtk::VBox m_VBoxForm7;
    Gtk::VBox m_VBoxCamera;

    Gtk::ComboBoxText m_MenuMode,m_MenuFPS,m_MenuISO,m_MenuOpMode,m_MenuColorCoding;
    unsigned int m_VideoModeLut[32];
    unsigned int m_ColorCodingLut[32];
    unsigned int m_FPSLut[8];

    int m_MenuColorCodingCNT;
    int m_bppCNT;

    int m_nFeatures;
    DC1394SliderBase* m_pSli[16];

    Gtk::Button m_Refresh1,m_Refresh2; ///< Read control values from framegrabber.
    Gtk::Button m_reset;
    Gtk::Button m_defaults;
    Gtk::Button m_propagate;
    Gtk::CheckButton m_broadcast;
    Gtk::CheckButton m_power;
    Gtk::CheckButton m_transmission;

    Gtk::SpinButton m_xdim,m_ydim,m_xoff,m_yoff;
    Gtk::SpinButton m_bpp;
 
    Gtk::Notebook m_Notebook;
    Gtk::Label m_LabelControls1,m_LabelControls2,m_LabelFormat,m_LabelFormat7,m_LabelCamera;
    Gtk::Label m_LabelMaxXDim,m_LabelMaxYDim;

    virtual void Init()
    {
        //printf("Init\n");

        //for (int n=0; n<m_nFeatures; ++n) m_pSli[n]->Refresh();

        m_MenuMode.set_active_text(video_mode_labels[getVideoModeDC1394()]);

        printf("video mode %s\n",video_mode_labels[getVideoModeDC1394()]);

        bool bFormat7=getVideoModeDC1394()>=24;
    
        m_MenuColorCoding.set_sensitive(bFormat7);
        m_xdim.set_sensitive(bFormat7);
        m_ydim.set_sensitive(bFormat7);
        m_xoff.set_sensitive(bFormat7);
        m_yoff.set_sensitive(bFormat7);
       
        m_MenuFPS.set_sensitive(!bFormat7);
        if (!bFormat7)
        {
            m_MenuFPS.set_active_text(video_rate_labels[getFPSDC1394()]);
            //printf("fps=%d\n",m_FPSLut[getFPSDC1394()]);
        }        

        m_MenuISO.set_active_text(iso_speed_labels[getISOSpeedDC1394()]);
        m_MenuOpMode.set_active_text(getOperationModeDC1394()?"1394b":"LEGACY");
        m_MenuColorCoding.set_active_text(color_coding_labels[getColorCodingDC1394()]);

        unsigned int xmax,ymax,xstep,ystep,xoffstep,yoffstep;
        getFormat7MaxWindowDC1394(xmax,ymax,xstep,ystep,xoffstep,yoffstep);

        unsigned int xdim,ydim;
        int x0,y0;
        getFormat7WindowDC1394(xdim,ydim,x0,y0);
        
        if (xstep<2) xstep=2;
        if (ystep<2) ystep=2;
        
        m_xdim.set_range(0,xmax);
        m_ydim.set_range(0,ymax);
        m_xdim.set_increments(xstep,0);
        m_ydim.set_increments(ystep,0);
        m_xdim.set_value(xdim);
        m_ydim.set_value(ydim);

        int xoffMax=(xmax-xdim)/2;
        int yoffMax=(ymax-ydim)/2;

        m_xoff.set_range(-xoffMax,xoffMax);
        m_yoff.set_range(-yoffMax,yoffMax);
        m_xoff.set_increments(xoffstep,0);
        m_yoff.set_increments(yoffstep,0);
        m_xoff.set_value(x0);
        m_yoff.set_value(y0);

        char buff[16];
        sprintf(buff,"Max width %d",xmax);
        m_LabelMaxXDim.set_text(buff);
        sprintf(buff,"Max height %d",ymax);
        m_LabelMaxYDim.set_text(buff);

        m_bpp.set_value(getBytesPerPacketDC1394());
        m_bpp.set_sensitive(bFormat7);
        m_transmission.set_active(getTransmissionDC1394());
    }

    /*
    virtual void Propagate()
    {
        printf("Propagating\n");

        for (int n=0; n<m_nFeatures; ++n) m_pSli[n]->Propagate();
        setVideoModeDC1394(m_VideoModeLut[m_MenuMode.get_active_row_number()]);

        setFPSDC1394(m_FPSLut[m_MenuFPS.get_active_row_number()]);
        setISOSpeedDC1394(m_MenuISO.get_active_row_number());
        setOperationModeDC1394(m_MenuOpMode.get_active_row_number());
        setColorCodingDC1394(m_ColorCodingLut[m_MenuColorCoding.get_active_row_number()]);

        //setBytesPerPacketDC1394((unsigned int)m_bpp.get_value());
        setFormat7WindowDC1394((unsigned int)m_xdim.get_value(),(unsigned int)m_ydim.get_value());
    }
    */

    //////////////////
    // SIGNAL HANDLERS
    //////////////////

    //virtual void on_notebook_switch_page(GtkNotebookPage* page, guint page_num){}

    /// Refresh button pressed
    virtual void on_refresh_pressed()
    { 
        printf("on_refresh_pressed()\n");
        Reload();
        //if (m_broadcast.get_active()) Propagate();
    }

    virtual void on_propagate_pressed()
    {
        printf("on_propagate_pressed()\n");
        //Propagate();
    }

    virtual void on_broadcast_change()
    {
        printf("on_broadcast_change()\n");
        bool bActive=m_broadcast.get_active();
        setBroadcastDC1394(bActive);
        m_propagate.set_sensitive(bActive);
        //if (bActive) Propagate();
    }

    virtual void on_mode_change()
    {
        printf("on_mode_change()\n");
        unsigned int video_mode=m_VideoModeLut[m_MenuMode.get_active_row_number()];
        setVideoModeDC1394(video_mode);
        Reload();
    }

    virtual void on_color_coding_change()
    {
        if (!m_MenuColorCoding.is_sensitive()) return;
        
        if (m_MenuColorCodingCNT)
        {
            --m_MenuColorCodingCNT;
            return;
        }
        
        int row_number=m_MenuColorCoding.get_active_row_number();       

        if (row_number<0) return;

        printf("on_color_coding_change()\n");
        setColorCodingDC1394(m_ColorCodingLut[row_number]);
        
        for (int n=0; n<m_nFeatures; ++n) m_pSli[n]->Refresh();
        //Refresh();    
    }

    virtual void on_framerate_change()
    {
        if (!m_MenuFPS.is_sensitive()) return;

        int row_number=m_MenuFPS.get_active_row_number();       

        if (row_number<0) return;      

        printf("on_framerate_change()\n");
        setFPSDC1394(m_FPSLut[row_number]);
    }

    virtual void on_bpp_change()
    {
      if (!m_bpp.is_sensitive()) return;
    
      if (m_bppCNT)
      {
          --m_bppCNT;
          return;
      }
    
      printf("on_bpp_change()\n");
      setBytesPerPacketDC1394((unsigned int)m_bpp.get_value());
    }

    virtual void on_iso_speed_change()
    {
        printf("on_iso_speed_change()\n");
        setISOSpeedDC1394(m_MenuISO.get_active_row_number());
        Reload();
    }

    virtual void on_operation_mode_change()
    {
        printf("on_operation_mode_change()\n");
        setOperationModeDC1394(m_MenuOpMode.get_active_row_number());
    }
    virtual void on_format7_window_change()
    {
        if (!m_xdim.is_sensitive() || !m_ydim.is_sensitive() || !m_xoff.is_sensitive() || !m_yoff.is_sensitive()) return;

        printf("on_format7_window_change()\n");

        unsigned int xdim=(unsigned)m_xdim.get_value_as_int();
        unsigned int ydim=(unsigned)m_ydim.get_value_as_int();
        int x0=(unsigned)m_xoff.get_value_as_int();
        int y0=(unsigned)m_yoff.get_value_as_int();

        setFormat7WindowDC1394(xdim,ydim,x0,y0);
    }

    virtual void on_power_onoff_change()
    {
        printf("on_power_onoff_change()\n");
        setPowerDC1394(m_power.get_active());
    }
    
    virtual void on_transmission_onoff_change()
    {
        printf("on_transmission_onoff_change()\n");
        setTransmissionDC1394(m_transmission.get_active());
    }

    virtual void on_reset_change()
    {
        printf("on_reset_change()\n");
        setResetDC1394();
        Reload();

        m_bpp.set_value(getBytesPerPacketDC1394());
        m_bpp.set_sensitive(getColorCodingMaskDC1394(getVideoModeDC1394()));
        m_transmission.set_active(getTransmissionDC1394());
    }

    virtual void on_load_defaults_change()
    {
        printf("on_load_defaults_change()\n");
        setDefaultsDC1394();
        Reload();

        m_bpp.set_value(getBytesPerPacketDC1394());
        m_bpp.set_sensitive(getColorCodingMaskDC1394(getVideoModeDC1394()));
        setTransmissionDC1394(true);
        m_transmission.set_active(getTransmissionDC1394());
    }
};

#endif
