// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/QWidgetRPC.h>

#include <yarp/PortCommand.h>
#include <yarp/BufferedConnectionWriter.h>
#include <yarp/NameClient.h>
#include <yarp/String.h>

using namespace yarp::gui;
using namespace yarp::os;
using namespace yarp;
using namespace std;

QWidgetRPC::QWidgetRPC() :
	_qwConnection(NULL),
	_posCmds(-1),
	_numCmdsBuffered(0){
_numCmdsBuffered = 30;
}

QWidgetRPC::QWidgetRPC(Searchable &config, QWidget* parent, const char* name, bool modal, WFlags fl)
	: QWidgetRPCBase( parent, name, fl ),
	_qwConnection(NULL),
	_posCmds(-1),
	_numCmdsBuffered(0){

	// read config
	std::string strRemote("");
	std::string strLocal("");
	if(config.check("name")){
		strLocal = std::string(config.find("name").asString().c_str()) + std::string("/conf");
	}
	if(config.check("RPC")){
		strRemote = config.findGroup("RPC").find("remote").asString().c_str();
	}
	else{
		if(config.check("remote")){
			strRemote = config.find("remote").asString().c_str();
		}
		else{
			strRemote = "/module/conf";
		}
	}

	// setup gui
	frmRPC->setMargin(0);
	frmRPC->layout()->setSpacing(3);
	frmRPC->layout()->setMargin(0);
	grbConnection->setColumnLayout(0, Qt::Vertical );
    QVBoxLayout *grbConnectionLayout = new QVBoxLayout(grbConnection->layout());
	grbConnection->layout()->setSpacing(3);
    grbConnection->layout()->setMargin(3);
    grbConnectionLayout->setAlignment(Qt::AlignTop);

	_prtRPC.open(strLocal.c_str());

	// setup connection widget
	_qwConnection = new QWidgetConnection(grbConnection, "qwConnection");
    grbConnectionLayout->addWidget(_qwConnection);
	_qwConnection->setSourcePortName(strLocal.c_str());
	_qwConnection->setTargetPortName(strRemote.c_str());
	_qwConnection->connectPorts(); // try to connect (don't care if doesn't work)

	_numCmdsBuffered = 30;

	txeResponse->setMaxLogLines(100);

	
}

QWidgetRPC::~QWidgetRPC(){
	this->close();
}

bool QWidgetRPC::close(){
	_prtRPC.interrupt();
	_prtRPC.close();
	return true;
}

void QWidgetRPC::btnStdin_clicked(){
	this->send(std::string((const char*)lneStdin->text()).c_str());
}

void QWidgetRPC::lneStdin_returnPressed(){
	this->send(std::string((const char*)lneStdin->text()).c_str());
}

bool QWidgetRPC::send(std::string command){

	lneStdin->setText("");
	_posCmds = -1;

	yarp::os::ConstString txt(command.c_str());

	if(txt != ""){
		if (txt[0]<32 && txt[0]!='\n' && 
			txt[0]!='\r' && txt[0]!='\0') {
			txeResponse->append(QString("ERROR: Strange character encountered\n"));
			return false;  // for example, horrible windows ^D
		}
		Bottle botReq;
		botReq.fromString(txt.c_str());
	    Bottle botRes;
	
		bool ok = _prtRPC.write(botReq, botRes);

		if (!ok || botRes.size() == 0) {
			txeResponse->append(QString("ERROR: Cannot write/read on connection\n"));
			return false;
		}
		if (yarp::String(botRes.get(0).asString())=="<ACK>") {
				txeResponse->append(QString("Acknowledged\n"));
		} else {
			ConstString txt;
			if (botRes.get(0).toString()=="help") {
				std::string strHelp("Help:\n");
				for (int i=1; i < botRes.size(); i++) {
					strHelp += std::string(botRes.get(i).toString().c_str()) + std::string("\n");
				}
				txt = strHelp.c_str();
			}
			else{
				txt = botRes.toString().c_str();
			}	
			txeResponse->append(QString(QString("Response: ") + QString(txt.c_str()) + QString("\n")));
		}
		
		_lstCmds.push_front(command);
		if((int)_lstCmds.size() > _numCmdsBuffered){
			_lstCmds.pop_back();
		}
	} // if txt != ""
	return true;
}


void QWidgetRPC::keyPressEvent (QKeyEvent *qkeyevent) {
	
	if(lneStdin->hasFocus()){
		switch (qkeyevent->key()){
			case Qt::Key_Up:
				_posCmds++;
				if(_posCmds >= (int)_lstCmds.size()){
					_posCmds = (int)_lstCmds.size() - 1;
				}
				if(_posCmds >= 0){
					lneStdin->setText(QString(_lstCmds[_posCmds].c_str()));
				}
				break;
			case Qt::Key_Down:
				_posCmds--;
				if(_posCmds < -1){
					_posCmds = -1;
				}
				if(_posCmds >= (int)_lstCmds.size()){
					_posCmds = (int)_lstCmds.size() - 1;
				}
				if(_posCmds >= 0){
					lneStdin->setText(QString(_lstCmds[_posCmds].c_str()));
				}
				else{
					lneStdin->setText(QString(""));
				}
				break;
			default:
				qkeyevent->ignore();
		}
	}
	else{
		qkeyevent->ignore();
	}
}

void QWidgetRPC::updateUI(){
	if(_qwConnection != NULL){
		_qwConnection->updateUI();	
	}
}
