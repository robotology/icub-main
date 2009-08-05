// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __FRAME_GRABBER_GUI_CONTROL_H__
#define __FRAME_GRABBER_GUI_CONTROL_H__

#include <gtkmm.h>
#include <yarp/dev/RemoteFrameGrabber.h>

#define SLIDER_COND_CONNECT(val,slider,label,handler) \
{ \
	if (val>-0.5) \
	{ \
		m_Height+=60; \
		slider.set_value(val); \
		slider.signal_value_changed().connect(sigc::mem_fun(*this,&handler)); \
		m_Box.pack_start(*Gtk::manage(new Gtk::HSeparator())); \
		m_Box.pack_start(*Gtk::manage(new Gtk::Label(label,0))); \
		m_Box.pack_start(*Gtk::manage(&slider)); \
	} \
} 

/**
 *	A graphical control interface for a remote framegrabber.	
 */
class FrameGrabberGUIControl : public Gtk::Window, public yarp::dev::RemoteFrameGrabber
{
public:
    /**
	 *	Constructor.
	 *
	 *	Creates a graphical control interface for a remote framegrabber.
	 *	@param loc local yarp port
	 *	@param rem remote framegrabber yarp port 
	 */
	FrameGrabberGUIControl(char* loc, char* rem) : 
	  // base classes
	  Gtk::Window(),
	  yarp::dev::RemoteFrameGrabber(),
	  
	  //members
	  m_bri(0.0,1.0,0.005),
	  m_exp(0.0,1.0,0.005),
	  m_sha(0.0,1.0,0.005),
	  m_red(0.0,1.0,0.005),
	  m_blu(0.0,1.0,0.005),
	  m_hue(0.0,1.0,0.005),
	  m_sat(0.0,1.0,0.005),
	  m_gam(0.0,1.0,0.005),	  	  	  
	  m_shu(0.0,1.0,0.005),
	  m_gai(0.0,1.0,0.005),
	  m_iri(0.0,1.0,0.005),
	  m_get("Refresh")
	{
		// yarp2

		yarp::os::Property config;
		config.put("remote",rem);
		config.put("local",loc);

		open(config);

		// GTKMM
		set_title("Grabber Remote GUI");

		m_bri.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_exp.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_sha.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_red.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_blu.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_hue.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_sat.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_gam.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_shu.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_gai.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		m_iri.set_update_policy(Gtk::UPDATE_DISCONTINUOUS);
		        
		double bri0=getBrightness();
        double exp0=getExposure();
		double sha0=getSharpness();
		double red0,blu0;
		getWhiteBalance(blu0,red0);
		double hue0=getHue();
		double sat0=getSaturation();
		double gam0=getGamma();		
		double shu0=getShutter();				        
		double gai0=getGain();
		double iri0=getIris();

		add(m_Box);

		m_get.signal_clicked().connect(sigc::mem_fun(*this,&FrameGrabberGUIControl::on_get_pressed));
		m_Box.pack_start(*Gtk::manage(&m_get));

		m_Height=24;

		SLIDER_COND_CONNECT(bri0,m_bri,"Brightness",FrameGrabberGUIControl::on_bri_changed)
		SLIDER_COND_CONNECT(exp0,m_exp,"Exposure",FrameGrabberGUIControl::on_exp_changed)
		SLIDER_COND_CONNECT(sha0,m_sha,"Sharpness",FrameGrabberGUIControl::on_sha_changed)		
		SLIDER_COND_CONNECT(red0,m_red,"Red Balance",FrameGrabberGUIControl::on_red_changed)
		SLIDER_COND_CONNECT(blu0,m_blu,"Blue Balance",FrameGrabberGUIControl::on_blu_changed)		
		SLIDER_COND_CONNECT(hue0,m_hue,"Hue",FrameGrabberGUIControl::on_hue_changed)
		SLIDER_COND_CONNECT(sat0,m_sat,"Saturation",FrameGrabberGUIControl::on_sat_changed)
		SLIDER_COND_CONNECT(gam0,m_gam,"Gamma",FrameGrabberGUIControl::on_gam_changed)		
		SLIDER_COND_CONNECT(shu0,m_shu,"Shutter",FrameGrabberGUIControl::on_shu_changed)		
		SLIDER_COND_CONNECT(gai0,m_gai,"Gain",FrameGrabberGUIControl::on_gai_changed)
		SLIDER_COND_CONNECT(iri0,m_iri,"Iris",FrameGrabberGUIControl::on_iri_changed)

		show_all_children();

		set_size_request(280,m_Height);
	}

    /**
	 *	Destructor.
	 *
	 *	Closes the remote framegrabber interface. 
	 */
	virtual ~FrameGrabberGUIControl()
	{
		close();
	}

protected:
	int m_Height;

	Gtk::VBox m_Box; ///< Control sliders container
	
	Gtk::HScale m_bri; ///< Brightness control slider. Normalized [0.0-1.0]
	Gtk::HScale m_exp; ///< Exposure control slider. Normalized [0.0-1.0]
	Gtk::HScale m_sha; ///< Sharpness control slider. Normalized [0.0-1.0]	
	Gtk::HScale m_red; ///< Red balance control slider. Normalized [0.0-1.0]
	Gtk::HScale m_blu; ///< Blue balance control slider. Normalized [0.0-1.0]
	Gtk::HScale m_hue; ///< Hue control slider. Normalized [0.0-1.0]
	Gtk::HScale m_sat; ///< Saturation control slider. Normalized [0.0-1.0]
	Gtk::HScale m_gam; ///< Gamma balance control slider. Normalized [0.0-1.0]	
	Gtk::HScale m_shu; ///< Shutter control slider. Normalized [0.0-1.0]		
	Gtk::HScale m_gai; ///< Gain control slider. Normalized [0.0-1.0]
	Gtk::HScale m_iri; ///< Iris control slider. Normalized [0.0-1.0]

	Gtk::Button m_get; ///< Read control values from framegrabber.

	// signal handlers

	/// Refresh button pressed
	virtual void on_get_pressed()
	{
		double val;
		if ((val=getBrightness())>-0.5) m_bri.set_value(val);
		if ((val=getExposure())>-0.5)   m_exp.set_value(val);
		if ((val=getSharpness())>-0.5)  m_sha.set_value(val);
		double red,blu;
		getWhiteBalance(blu,red);
		if (red>-0.5)                   m_red.set_value(red);		
		if (blu>-0.5)                   m_blu.set_value(blu);
		if ((val=getHue())>-0.5)        m_hue.set_value(val);
		if ((val=getSaturation())>-0.5) m_sat.set_value(val);
		if ((val=getGamma())>-0.5)      m_gam.set_value(val);
		if ((val=getShutter())>-0.5)    m_shu.set_value(val);
		if ((val=getGain())>-0.5)		m_gai.set_value(val);
		if ((val=getIris())>-0.5)		m_iri.set_value(val);
	}

	/// Brightness changed signal handler.
	virtual void on_bri_changed() 
	{
		double val=m_bri.get_value();
		g_print("on_bri_changed(%f)\n",val);
		setBrightness(val);
	}
	/// Exposure changed signal handler.
	virtual void on_exp_changed() 
	{
		double val=m_exp.get_value();
		g_print("on_exp_changed(%f)\n",val);
		setExposure(val);
	}
	/// Sharpness changed signal handler.
	virtual void on_sha_changed() 
	{
		double val=m_sha.get_value();
		g_print("on_sha_changed(%f)\n",val);
		setSharpness(val);
	}
	/// Red balance changed signal handler.
	virtual void on_red_changed() 
	{
		double red=m_red.get_value();
		double blu=m_blu.get_value();
		g_print("on_white_changed(%f,%f)\n",blu,red);
		setWhiteBalance(blu,red);
	}
	/// Blue balance changed signal handler.
	virtual void on_blu_changed() 
	{
		double red=m_red.get_value();
		double blu=m_blu.get_value();
		g_print("on_white_changed(%f,%f)\n",blu,red);
		setWhiteBalance(blu,red);
	}
	/// Hue changed signal handler.
	virtual void on_hue_changed() 
	{
		double val=m_hue.get_value();
		g_print("on_hue_changed(%f)\n",val);
		setHue(val);
	}
	/// Saturation changed signal handler.
	virtual void on_sat_changed() 
	{
		double val=m_sat.get_value();
		g_print("on_sat_changed(%f)\n",val);
		setSaturation(val);
	}
	/// Gamma changed signal handler.
	virtual void on_gam_changed() 
	{
		double val=m_gam.get_value();
		g_print("on_gam_changed(%f)\n",val);
		setGamma(val);
	}
	/// Shutter changed signal handler.
	virtual void on_shu_changed() 
	{
		double val=m_shu.get_value();
		g_print("on_shu_changed(%f)\n",val);
		setShutter(val);
	}
	/// Gain changed signal handler.
	virtual void on_gai_changed() 
	{
		double val=m_gai.get_value();
		g_print("on_gai_changed(%f)\n",val);
		setGain(val);
	}
		/// Iris changed signal handler.
	virtual void on_iri_changed() 
	{
		double val=m_iri.get_value();
		g_print("on_iri_changed(%f)\n",val);
		setIris(val);
	}
};

#endif
