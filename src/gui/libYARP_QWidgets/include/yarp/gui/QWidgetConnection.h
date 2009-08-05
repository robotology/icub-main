// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETCONNECTION__
#define __ICUB_QWIDGETCONNECTION__

// std
#include <iostream>
#include <string>

// iCub
#include <qwidgetconnectionbase.h>

// Qt
#include <qpushbutton.h>
#include <qlabel.h>
#include <qwidget.h>
#include <qlineedit.h>
#include <qtimer.h>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Searchable.h>

namespace yarp {
    namespace gui {
        class QWidgetConnection;
    }
}

class yarp::gui::QWidgetConnection : public QWidgetConnectionBase
{
	Q_OBJECT

public:
    QWidgetConnection();
    QWidgetConnection(QWidget *parent, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~QWidgetConnection();
	
	// source port
	virtual void setSourcePortName(std::string name);
	virtual std::string getSourcePortName();
	virtual void enableSourcePortEdit(bool edit);

	// target port
	virtual void setTargetPortName(std::string name);
	virtual std::string getTargetPortName();
	virtual void enableTargetPortEdit(bool edit);

	// connection
	virtual bool connectPorts(bool updateUI = true);
	virtual bool disconnectPorts(bool updateUI = true);

	// update
	virtual void updateUI(); // update all UI elements
	virtual void updateUISourcePortButton(); // calls checkSourcePort(false)
	virtual void updateUITargetPortButton(); // calls checkTargetPort(false)
	virtual void updateUIConnectionButton(); // calls checkConnection(false)
	virtual bool checkConnection(bool updateUIConnectionButton = true);
	virtual bool checkSourcePort(bool updateUISourcePortButton = true);
	virtual bool checkTargetPort(bool updateUITargetPortButton = true);

protected:

	std::string _strOldSourcePort;
	std::string _strOldTargetPort;

    virtual void btnConnect_pressed();
	virtual void btnSource_clicked();
	virtual void btnTarget_clicked();
	virtual void txeTarget_returnPressed();
	virtual void txeSource_returnPressed();

	virtual bool connectPorts(std::string source, std::string target);
	virtual bool disconnectPorts(std::string source, std::string target);

	virtual bool checkPort(std::string name);
};
#endif 

