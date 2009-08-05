// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETAPPLICATION__
#define __ICUB_QWIDGETAPPLICATION__

// std
#include <iostream>
#include <string>

// iCub
#include <yarp/gui/QWidgetYarp.h>

// Qt
#include <qlabel.h>
#include <qwidget.h>
#include <qlayout.h>
#include <qtabwidget.h>

// yarp
#include <yarp/os/Searchable.h>

namespace yarp {
    namespace gui {
        class QWidgetApplication;
    }
}

class yarp::gui::QWidgetApplication :
	public QWidget,
	public yarp::gui::QWidgetYarp {

	Q_OBJECT

public:

    QWidgetApplication(QWidget* parent = 0, const char* name = 0);
	QWidgetApplication(yarp::os::Searchable &config, QWidget* parent = 0, const char* name = 0);
    virtual ~QWidgetApplication();

	virtual void init(QWidget* parent, const char* name);

	virtual QWidget* getQWidget(){return this;}

protected:

	QBoxLayout *_baseLayout; 
    QTabWidget *_tabWidget;

	 bool open(yarp::os::Searchable & config);
	 bool close();

	 // load a widget specified in 'config' into the parent widget
	 bool loadWidget(yarp::os::Searchable &config, QWidget* parent, QLayout* parentLayout);
	 // load a page configuration specified in 'config' into the parent widget
	 bool loadPage(yarp::os::Searchable &config, QWidget* parent, QLayout* parentLayout);
	 // load an application configuration specified in 'config' into the parent widget
	 bool loadApplication(yarp::os::Searchable &config, QWidget *parent, QLayout* parentLayout);
};
#endif 

