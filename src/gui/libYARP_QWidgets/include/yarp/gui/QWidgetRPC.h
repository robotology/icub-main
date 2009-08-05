// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETRPC__
#define __ICUB_QWIDGETRPC__

// std
#include <iostream>
#include <string>
#include <deque>

// yarp
#include <yarp/os/all.h>
#include <yarp/StreamConnectionReader.h>
#include <yarp/OutputProtocol.h>
#include <yarp/Carriers.h>
#include <yarp/Readable.h>

// iCub
#include <qwidgetrpcbase.h>
#include<yarp/gui/QWidgetConnection.h>
#include <yarp/gui/QWidgetYarp.h>

// Qt
#include <qtimer.h>
#include <qlayout.h>
#include <qtextedit.h>
#include <qgroupbox.h>
#include <qstring.h>

namespace yarp {
    namespace gui {
        class QWidgetRPC;
    }
}

class yarp::gui::QWidgetRPC : 
	public QWidgetRPCBase,
	public QWidgetYarp {

	Q_OBJECT

public:
    QWidgetRPC();
	QWidgetRPC(yarp::os::Searchable &config, QWidget *parent, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~QWidgetRPC();

	virtual void updateUI();

	virtual QWidget* getQWidget(){return this;}

protected:

	yarp::gui::QWidgetConnection *_qwConnection;
	yarp::os::Port _prtRPC;

	std::deque<std::string> _lstCmds;
	int _posCmds;
	int _numCmdsBuffered;

	bool close();

	void lneStdin_returnPressed();
	void btnStdin_clicked();
	void keyPressEvent (QKeyEvent *qkeyevent);
	bool send(std::string command);

};
#endif 

