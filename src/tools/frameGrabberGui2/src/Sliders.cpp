// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <Sliders.h>
#include <FrameGrabberGUIControl2.h>

int DC1394SliderBase::m_Height=0;

    DC1394Slider::~DC1394Slider()
    {
        if (m_bInactive) return;
        delete pPwr;
        delete pRBa;
        delete pRBm;
    }
    
    DC1394Slider::DC1394Slider(dc1394feature_id_t feature,char* label,Gtk::VBox &vbox,FrameGrabberGUIControl2 *fg) 
        : m_Slider(0.0,1.005,0.005),m_OnePush("One Push")
    {
        if (!((pFG=fg)->hasFeatureDC1394(m_Feature=feature)))
        {
            printf("feature %s is inactive\n",label);
            m_bInactive=true;
            return;
        }
        
        m_nInternalChange=0;
        m_bInactive=false;

        m_Name=label;

        m_Height+=70;

        m_Slider.set_size_request(256,40);
        
        vbox.pack_start(*(new Gtk::HSeparator()),Gtk::PACK_SHRINK,2);
        
        Gtk::HBox* pHBox=new Gtk::HBox();
        pHBox->pack_start(*(new Gtk::Label(label,0)),Gtk::PACK_EXPAND_PADDING);
        pHBox->pack_start(*(pPwr=new Gtk::CheckButton("pwr")),Gtk::PACK_SHRINK,0);
        pHBox->pack_start(*(pRBa=new Gtk::RadioButton("auto")),Gtk::PACK_SHRINK,0);
        const Glib::ustring s_man("man");
        Gtk::RadioButtonGroup rbg=pRBa->get_group();
        pHBox->pack_start(*(pRBm=new Gtk::RadioButton(rbg,s_man,false)),Gtk::PACK_SHRINK,0);
        pHBox->pack_start(m_OnePush,Gtk::PACK_SHRINK,8);

        vbox.pack_start(*(pHBox),Gtk::PACK_SHRINK);
        vbox.pack_start(m_Slider,Gtk::PACK_SHRINK,0);

        m_Slider.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
        //m_old_value=m_new_value=pFG->getFeatureDC1394(m_Feature);
        m_old_value=-1.0;

        m_Slider.signal_value_changed().connect(sigc::mem_fun(*this,&DC1394Slider::slider_handler));
        m_OnePush.signal_clicked().connect(sigc::mem_fun(*this,&DC1394Slider::onepush_handler));
        pPwr->signal_clicked().connect(sigc::mem_fun(*this,&DC1394Slider::pwr_handler));
        pRBa->signal_clicked().connect(sigc::mem_fun(*this,&DC1394Slider::auto_handler));

        Refresh();
    }

    void DC1394Slider::Refresh()
    {
        if (m_bInactive) return;
       
        bool bON=pFG->getActiveDC1394(m_Feature);
        bool bAuto=pFG->getModeDC1394(m_Feature);

        pPwr->set_active(bON);

        pPwr->set_sensitive(pFG->hasOnOffDC1394(m_Feature));
        pRBa->set_sensitive(bON && pFG->hasAutoDC1394(m_Feature));
        pRBm->set_sensitive(bON && pFG->hasManualDC1394(m_Feature));
        m_Slider.set_sensitive(bON && !bAuto);
        m_OnePush.set_sensitive(bON && pFG->hasOnePushDC1394(m_Feature));

        if (bAuto) 
            pRBa->set_active(true);
        else
            pRBm->set_active(true);
        
        if (m_old_value!=(m_new_value=pFG->getFeatureDC1394(m_Feature)))
        {
            ++m_nInternalChange;
            m_Slider.set_value(m_old_value=m_new_value);
        }
    }

    void DC1394Slider::Propagate()
    {
        if (m_bInactive) return;
    
        pFG->setFeatureDC1394(m_Feature,m_Slider.get_value());
        pFG->setModeDC1394(m_Feature,pRBa->get_active());
        pFG->setActiveDC1394(m_Feature,pPwr->get_active());
    }

    void DC1394Slider::slider_handler()
    {
        printf("++++++++++++++\n");

        if (m_nInternalChange>0)
        {
            --m_nInternalChange;
            return;
        }        

        pFG->setFeatureDC1394(m_Feature,m_Slider.get_value());
        printf("%s new value %lf\n",m_Name.c_str(),m_Slider.get_value());
    }

    void DC1394Slider::onepush_handler()
    { 
        //m_old_value=pFG->getFeatureDC1394(m_Feature);
        pFG->setOnePushDC1394(m_Feature);
        
        if (m_old_value!=(m_new_value=pFG->getFeatureDC1394(m_Feature)))
        {
            ++m_nInternalChange;
            m_Slider.set_value(m_old_value=m_new_value);
        }
        
        pFG->Reload();
    }
    
    void DC1394Slider::auto_handler()
    { 
        bool bON=pFG->getActiveDC1394(m_Feature);
        bool bAuto=pRBa->get_active();
        pFG->setModeDC1394(m_Feature,bAuto); 
        m_Slider.set_sensitive(bON && !bAuto);
        printf("%s\n",pRBa->get_active()?"auto":"man");
        
        pFG->Reload();
    }
    
    void DC1394Slider::pwr_handler()
    { 
        bool bON=pPwr->get_active();
        pFG->setActiveDC1394(m_Feature,bON);
        pRBa->set_sensitive(bON && pFG->hasAutoDC1394(m_Feature)); 
        pRBm->set_sensitive(bON && pFG->hasManualDC1394(m_Feature));
        m_Slider.set_sensitive(bON && pRBm->get_active());
        m_OnePush.set_sensitive(bON && pFG->hasOnePushDC1394(m_Feature));                 
        printf("power %s\n",pPwr->get_active()?"on":"off");
        
        pFG->Reload();
    }

    void DC1394Slider::set_value(double val)
    { 
        m_Slider.set_value(val); 
    }

////////////////////////////////////////////

    DC1394SliderWB::~DC1394SliderWB()
    {
        if (m_bInactive) return;
        delete pPwr;
        delete pRBa;
        delete pRBm;
    }
    
    DC1394SliderWB::DC1394SliderWB(Gtk::VBox &vbox,yarp::dev::RemoteFrameGrabberControlsDC1394 *fg) 
        : m_Red(0.0,1.005,0.005),m_Blue(0.0,1.005,0.005),m_OnePush("One Push")
    {
        if (!(pFG=fg)->hasFeatureDC1394(DC1394_FEATURE_WHITE_BALANCE))
        {
            m_bInactive=true;
            return;
        }

        m_nInternalChange=0;

        m_bInactive=false;

        m_Height+=100;

        vbox.pack_start(*(new Gtk::HSeparator()),Gtk::PACK_SHRINK,2);

        Gtk::HBox* pHBox=new Gtk::HBox();
        pHBox->pack_start(*(new Gtk::Label("White Balance",0)),Gtk::PACK_EXPAND_PADDING);
        pHBox->pack_start(*(pPwr=new Gtk::CheckButton("pwr")),Gtk::PACK_SHRINK,0);
        pHBox->pack_start(*(pRBa=new Gtk::RadioButton("auto")),Gtk::PACK_SHRINK,0);
        const Glib::ustring s_man("man");
        Gtk::RadioButtonGroup rbg=pRBa->get_group();
        pHBox->pack_start(*(pRBm=new Gtk::RadioButton(rbg,s_man,false)),Gtk::PACK_SHRINK,0);
        pHBox->pack_start(m_OnePush,Gtk::PACK_SHRINK,8);
        vbox.pack_start(*(pHBox),Gtk::PACK_SHRINK);
        
        m_Blue.set_size_request(256,40);
        m_Red.set_size_request(256,40);
        Gtk::HBox *pRbox=new Gtk::HBox();
        pRbox->pack_start(*(new Gtk::Label("RED ",0)),Gtk::PACK_SHRINK,4);
        pRbox->pack_start(m_Red,Gtk::PACK_EXPAND_WIDGET,10);
        vbox.pack_start(*(pRbox),Gtk::PACK_SHRINK,0);
        Gtk::HBox *pBbox=new Gtk::HBox();
        pBbox->pack_start(*(new Gtk::Label("BLUE",0)),Gtk::PACK_SHRINK,4);
        pBbox->pack_start(m_Blue,Gtk::PACK_EXPAND_WIDGET,10);
        vbox.pack_start(*(pBbox),Gtk::PACK_SHRINK,0);

        //pFG->getWhiteBalanceDC1394(m_old_blu,m_old_red);
        //m_new_red=m_old_red;
        //m_new_blu=m_old_blu;

        m_old_red=m_old_blu=-1.0;

        m_Blue.signal_value_changed().connect(sigc::mem_fun(*this,&DC1394SliderWB::slider_handler));
        m_Blue.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
        m_Red.signal_value_changed().connect(sigc::mem_fun(*this,&DC1394SliderWB::slider_handler));
        m_Red.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
        pPwr->signal_clicked().connect(sigc::mem_fun(*this,&DC1394SliderWB::pwr_handler));
        pRBa->signal_clicked().connect(sigc::mem_fun(*this,&DC1394SliderWB::automan_handler));
        m_OnePush.signal_clicked().connect(sigc::mem_fun(*this,&DC1394SliderWB::onepush_handler));

        Refresh();
    }

    void DC1394SliderWB::Refresh()
    {
        if (m_bInactive) return;
        
        bool bON=pFG->getActiveDC1394(DC1394_FEATURE_WHITE_BALANCE);
        bool bAuto=pFG->getModeDC1394(DC1394_FEATURE_WHITE_BALANCE);

        if (bAuto) 
            pRBa->set_active(true);
        else
            pRBm->set_active(true);

        pPwr->set_active(bON);

        pPwr->set_sensitive(pFG->hasOnOffDC1394(DC1394_FEATURE_WHITE_BALANCE));
        pRBa->set_sensitive(bON && pFG->hasAutoDC1394(DC1394_FEATURE_WHITE_BALANCE));
        pRBm->set_sensitive(bON && pFG->hasManualDC1394(DC1394_FEATURE_WHITE_BALANCE));
        m_Blue.set_sensitive(bON && !bAuto);
        m_Red.set_sensitive(bON && !bAuto);
        m_OnePush.set_sensitive(bON && pFG->hasOnePushDC1394(DC1394_FEATURE_WHITE_BALANCE));

        pFG->getWhiteBalanceDC1394(m_new_blu,m_new_red);
        if (m_new_blu!=m_old_blu)
        {
            ++m_nInternalChange;
            m_Blue.set_value(m_old_blu=m_new_blu);
        }

        if (m_new_red!=m_old_red)
        {
            ++m_nInternalChange;
            m_Red.set_value(m_old_red=m_new_red);
        }
    }

    void DC1394SliderWB::Propagate()
    {
        if (m_bInactive) return;
    
        pFG->setWhiteBalanceDC1394(m_Blue.get_value(),m_Red.get_value());
        pFG->setModeDC1394(DC1394_FEATURE_WHITE_BALANCE,pRBa->get_active());
        pFG->setActiveDC1394(DC1394_FEATURE_WHITE_BALANCE,pPwr->get_active());
    }

    void DC1394SliderWB::slider_handler()
    {
        printf("************\n");
        if (m_nInternalChange>0)
        {
            --m_nInternalChange;
            return;
        }

        pFG->setWhiteBalanceDC1394(m_Blue.get_value(),m_Red.get_value());
        printf("white balance %f %f\n",m_Blue.get_value(),m_Red.get_value());
    }
    
    void DC1394SliderWB::onepush_handler()
    { 
        //pFG->getWhiteBalanceDC1394(m_old_blu,m_old_red);
        pFG->setOnePushDC1394(DC1394_FEATURE_WHITE_BALANCE); 
        pFG->getWhiteBalanceDC1394(m_new_blu,m_new_red);
        printf("one push\n");

        if (m_new_blu!=m_old_blu)
        {
            ++m_nInternalChange;
            m_Blue.set_value(m_old_blu=m_new_blu);
        }

        if (m_new_red!=m_old_red)
        {
            ++m_nInternalChange;
            m_Red.set_value(m_old_red=m_new_red);
        }
    }

    void DC1394SliderWB::automan_handler()
    {
        bool bAuto=pRBa->get_active();
        pFG->setModeDC1394(DC1394_FEATURE_WHITE_BALANCE,bAuto); 
        m_Red.set_sensitive(!bAuto); 
        m_Blue.set_sensitive(!bAuto);
        printf("%s\n",pRBa->get_active()?"auto":"man");
    }

    void DC1394SliderWB::pwr_handler()
    { 
        bool bON=pPwr->get_active();
        pFG->setActiveDC1394(DC1394_FEATURE_WHITE_BALANCE,bON);
        pRBa->set_sensitive(bON && pFG->hasAutoDC1394(DC1394_FEATURE_WHITE_BALANCE)); 
        pRBm->set_sensitive(bON && pFG->hasManualDC1394(DC1394_FEATURE_WHITE_BALANCE));
        m_Red.set_sensitive(bON && pRBm->get_active());
        m_Blue.set_sensitive(bON && pRBm->get_active());
        m_OnePush.set_sensitive(bON && pFG->hasOnePushDC1394(DC1394_FEATURE_WHITE_BALANCE));                 
        printf("power %s\n",pPwr->get_active()?"on":"off");
    }

    void DC1394SliderWB::set_value(double blue,double red)
    { 
        m_Blue.set_value(blue); 
        m_Red.set_value(red); 
    }
