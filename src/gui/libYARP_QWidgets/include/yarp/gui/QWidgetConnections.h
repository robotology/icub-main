// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETCONNECTIONS__
#define __ICUB_QWIDGETCONNECTIONS__

// std
#include <iostream>
#include <string>
#include <vector>

// iCub
#include <qwidgetconnectionsbase.h>
#include <yarp/gui/QWidgetConnection.h>
#include <yarp/gui/QWidgetYarp.h>

// Qt
#include <qpushbutton.h>
#include <qlabel.h>
#include <qwidget.h>
#include <qlayout.h>
#include <qscrollview.h>
#include <qvbox.h>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Searchable.h>

namespace yarp {
    namespace gui {
        class QWidgetConnections;
    }
}

class yarp::gui::QWidgetConnections : 
	public QWidgetConnectionsBase,
	public QWidgetYarp {

	Q_OBJECT

public:
    QWidgetConnections();
	QWidgetConnections(yarp::os::Searchable & config, QWidget *parent, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~QWidgetConnections();

	virtual bool addConnection(std::string strSource, std::string strTarget);

	virtual void updateUI();
	virtual bool connectAllPorts();
	virtual bool disconnectAllPorts();

	virtual QWidget* getQWidget(){return this;}

protected:

	std::vector<QWidgetConnection*> _vecConnections;

	QVBoxLayout *_scrlFrameLayout;
	QFrame *_scrlFrame;


    virtual void btnConnectAll_clicked();
	virtual void btnDisconnectAll_clicked();
	virtual void btnAddConnection_clicked();
	virtual void btnRemoveConnection_clicked();

};
#endif 

