// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/QWidgetOutput.h>

using namespace yarp::gui;
using namespace yarp::os;
using namespace std;

QWidgetOutput::QWidgetOutput():
	_qwConStdout(NULL){
	 
}

QWidgetOutput::QWidgetOutput(Searchable &config, QWidget* parent, const char* name, bool modal, WFlags fl)
	: QWidgetOutputBase( parent, name, fl ),
	_qwConStdout(NULL){

	txeStdout->setMaxLogLines(999);

	grbCon->setOrientation(Qt::Vertical);
	grbCon->setColumns(1);
	grbCon->setInsideMargin(3);
	grbCon->setInsideSpacing(3);
	
	string strStdoutLocal("");
	string strStdoutRemote("");

	if(config.check("name")){
		strStdoutLocal = string(config.find("name").asString().c_str());
	}
	if(config.check("STDOUT")){
		strStdoutRemote = config.findGroup("STDOUT").find("remote").asString().c_str();
	}
	else{
		if(config.check("remote")){
			strStdoutRemote = config.find("remote").asString().c_str();
		}
		else{
			strStdoutRemote = "/module/stdout";
		}
	}

	_prtStdout.useCallback(_stdoutCallback);
	_prtStdout.open(strStdoutLocal.c_str());

	_qwConStdout = new QWidgetConnection(grbCon);
	_qwConStdout->setSourcePortName(strStdoutRemote);
	_qwConStdout->setTargetPortName(strStdoutLocal);
	_qwConStdout->connectPorts();

	// start the UI update loop
	QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerDone()) );
    timer->start(10, FALSE ); // update status every 0.1s
}

QWidgetOutput::~QWidgetOutput(){
	this->close();
}

void QWidgetOutput::timerDone(){
	txeStdout->append(_stdoutCallback.getReadings().c_str());
}

void QWidgetOutput::updateUI(){
	if(_qwConStdout != NULL){
		_qwConStdout->updateUI();
	}
}

bool QWidgetOutput::close(){
	_prtStdout.interrupt();
	_prtStdout.close();
	return true;
}

