// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/QWidgetConnection.h>
#include <yarp/Companion.h>

#include <qlayout.h>

using namespace yarp::gui;
using namespace yarp;
using namespace yarp::os;
using namespace std;

QWidgetConnection::QWidgetConnection() {
	 enableSourcePortEdit(true);
	 enableTargetPortEdit(true);
	_strOldSourcePort = "";
	_strOldTargetPort = "";

	QLayout *mainLayout = this->layout();
	if(mainLayout != NULL){
		mainLayout->setSpacing(3);
		mainLayout->setMargin(3);
	}

	this->updateUI();
}

QWidgetConnection::QWidgetConnection(QWidget* parent, const char* name, bool modal, WFlags fl)
	: QWidgetConnectionBase( parent, name, fl ){
	enableSourcePortEdit(true);
	enableTargetPortEdit(true);
	_strOldSourcePort = "";
	_strOldTargetPort = "";

	QLayout *mainLayout = this->layout();
	if(mainLayout != NULL){
		mainLayout->setSpacing(3);
		mainLayout->setMargin(3);
	}

	this->updateUI();
}

QWidgetConnection::~QWidgetConnection(){

}

void QWidgetConnection::setSourcePortName(std::string name){
	txeSource->setText(QString(name.c_str()));
}

void QWidgetConnection::setTargetPortName(std::string name){
	txeTarget->setText(QString(name.c_str()));
}

std::string QWidgetConnection::getSourcePortName(){
	return std::string((const char*)txeSource->text());
}

std::string QWidgetConnection::getTargetPortName(){
	return std::string((const char*)txeTarget->text());
}

void QWidgetConnection::enableSourcePortEdit(bool edit){
	txeSource->setReadOnly(!edit);
}

void QWidgetConnection::enableTargetPortEdit(bool edit){
	txeTarget->setReadOnly(!edit);
}	

void QWidgetConnection::updateUI(){
	this->updateUISourcePortButton();
	this->updateUITargetPortButton();
	this->updateUIConnectionButton();
}

void QWidgetConnection::updateUISourcePortButton(){
	this->checkSourcePort(true);
}

void QWidgetConnection::updateUITargetPortButton(){
	this->checkTargetPort(true);
}

void QWidgetConnection::updateUIConnectionButton(){
	this->checkConnection(true);
}

bool QWidgetConnection::checkConnection(bool updateUIConnectionButton){
	bool ok = false;
	// check connection
	if(	std::string((const char*)txeSource->text()) != std::string("") && 
		std::string((const char*)txeTarget->text()) != std::string("")){
		ok = yarp::os::Network::isConnected(	std::string((const char*)txeSource->text()).c_str(), 
												std::string((const char*)txeTarget->text()).c_str());
	}
	if(updateUIConnectionButton){
		if(ok){
			btnConnect->setPaletteBackgroundColor( QColor( 0, 255, 0 ) );
		}
		else{
			btnConnect->setPaletteBackgroundColor( QColor( 255, 0, 0 ) );
		}
	}
	return ok;
}

bool QWidgetConnection::checkSourcePort(bool updateUISourcePortButton){
	bool ok = checkPort(this->getSourcePortName());
	if(updateUISourcePortButton){
		if(ok){
			btnSource->setText("OK");
			btnSource->setPaletteBackgroundColor(QColor( 0, 255, 0));
		}
		else{
			btnSource->setText("KO");
			btnSource->setPaletteBackgroundColor(QColor(255, 0, 0));
		}
	}
	return ok;
}

bool QWidgetConnection::checkTargetPort(bool updateUITargetPortButton){
	bool ok = checkPort(this->getTargetPortName());
	if(updateUITargetPortButton){
		if(ok){
			btnTarget->setText("OK");
			btnTarget->setPaletteBackgroundColor(QColor( 0, 255, 0));
		}
		else{
			btnTarget->setText("KO");
			btnTarget->setPaletteBackgroundColor(QColor(255, 0, 0));
		}
	}
	return ok;
}

bool QWidgetConnection::connectPorts(bool updateUI){
	bool ok = false;
	// disconnect previous  connection
	if(_strOldSourcePort != std::string("") && _strOldTargetPort != std::string("")){
		this->disconnectPorts(_strOldSourcePort, _strOldTargetPort);
	}
	// establish new connection
	ok = this->connectPorts(this->getSourcePortName(), this->getTargetPortName());
	if(updateUI){
		this->updateUI();
	}
	return ok;
}

bool QWidgetConnection::disconnectPorts(bool updateUI){
	bool ok = false;
	if(this->getSourcePortName() != std::string("") && this->getTargetPortName() != std::string("")){
		ok = this->disconnectPorts(this->getSourcePortName(), this->getTargetPortName());
	}
	else{
		ok = false;
	}
	if(updateUI){
		this->updateUI();
	}
	return ok;
}

bool QWidgetConnection::connectPorts(std::string source, std::string target){
	bool ok = false;
	// there is no problem if connection exists already
	if(	source != std::string("") && target != std::string("")){
		ok = yarp::os::Network::connect(source.c_str(), target.c_str());
	}
	// successfully established connection, save port names
	if(ok){
		_strOldSourcePort = source;
		_strOldTargetPort = target;
	}
	return ok;
}

bool QWidgetConnection::disconnectPorts(std::string source, std::string target){
	bool ok = false;
	if(	source != std::string("") && target != std::string("")){
		ok = yarp::os::Network::disconnect(	source.c_str(), target.c_str());
	}
	// successfully disconnected connection, no ports to disconnect anymore
	if(ok){
		_strOldSourcePort = "";
		_strOldTargetPort = "";
	}
	return ok;
}


bool QWidgetConnection::checkPort(std::string name){
	if(name == std::string("")){
		return false;
	}
	int res = 1;
	res = yarp::Companion::exists(name.c_str(), true);
	if(res == 0){
		return true;
	}
	else{
		return false;
	}
}


// UI interaction

void QWidgetConnection::btnConnect_pressed(){
	if(!this->checkConnection(false)){
		this->connectPorts(false);
	}
	else{
		this->disconnectPorts(false);
	}
	this->updateUIConnectionButton();
}	

void QWidgetConnection::btnSource_clicked(){
	this->updateUISourcePortButton();
}

void QWidgetConnection::btnTarget_clicked(){
	this->updateUITargetPortButton();
}

void QWidgetConnection::txeTarget_returnPressed(){
	this->connectPorts(false);
	this->updateUI();
}

void QWidgetConnection::txeSource_returnPressed(){
	this->connectPorts(false);
	this->updateUI();
}

