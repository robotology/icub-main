// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETVIEWER__
#define __ICUB_QWIDGETVIEWER__

// std
#include <iostream>
#include <string>

// iCub
#include <qwidgetviewerbase.h>
#include <yarp/gui/QWidgetConnection.h>
#include <yarp/gui/QGLVideo.h>
#include <yarp/gui/QWidgetYarp.h>


// Qt
#include <qgroupbox.h>
#include <qpushbutton.h>
#include <qlabel.h>
#include <qwidget.h>
#include <qlineedit.h>
#include <qlayout.h>
#include <qgl.h>
#include <qtimer.h>
#include <qscrollview.h>
#include <qvbox.h>
#include <qsizepolicy.h>

// yarp
#include <yarp/os/Searchable.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>

namespace yarp {
    namespace gui {
        class QWidgetViewer;
    }
}

class yarp::gui::QWidgetViewer : 
	public QWidgetViewerBase,
	public QWidgetYarp {

	Q_OBJECT

public:

    QWidgetViewer(QWidget* parent = 0, const char* name = 0, bool showInOrigSize = false);
	QWidgetViewer(yarp::os::Searchable &config, QWidget* parent = 0, const char* name = 0, bool showInOrigSize = true);
    virtual ~QWidgetViewer();

	virtual void init(yarp::os::Searchable &config, QWidget* parent, const char* name, bool showInOrigSize);

	virtual void setImageSourcePortName(std::string name);
	virtual std::string getImageSourcePortName();
	virtual void setLocalPortName(std::string name);
	virtual std::string getLocalPortName();

	virtual bool open(yarp::os::Searchable & config);
	virtual bool close();

	virtual void updateUI();

	virtual QWidget* getQWidget(){return this;}

protected:
 
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > _prtImgIn;
	yarp::os::BufferedPort<yarp::os::Bottle> _prtClickOut;

	QWidgetConnection *_qwConnection;
	QWidgetConnection *_qwConnectionClick;

	bool _hasClickPort;
	QGLVideo *_qwVideo;
	int _counter;
	double _time;
	bool _fixedSize;
	int _width;
	int _height;

protected slots:
	virtual void timerDone();
};
#endif 

