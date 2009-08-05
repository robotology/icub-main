// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/QWidgetConnections.h>

using namespace yarp::gui;
using namespace yarp::os;
using namespace std;

QWidgetConnections::QWidgetConnections(){

}

QWidgetConnections::QWidgetConnections(Searchable& config, QWidget* parent, const char* name, bool modal, WFlags fl)
: QWidgetConnectionsBase( parent, name, fl ),
_scrlFrame(NULL),
_scrlFrameLayout(NULL){


	// frmConnections layout
	QVBoxLayout *frmConnectionsLayout = new QVBoxLayout(frmConnections, 3, 3, "frmConnections"); 
    
    // scroll view
    QScrollView *scrlView = new QScrollView(frmConnections);
    frmConnectionsLayout->addWidget(scrlView);
    scrlView->setResizePolicy(QScrollView::AutoOneFit);
	//scrlView->setHScrollBarMode(QScrollView::ScrollBarMode::AlwaysOff);

    // vbox for sub configurable buttons
    _scrlFrame = new QFrame(scrlView->viewport());
    scrlView->addChild(_scrlFrame);

	_scrlFrameLayout = new QVBoxLayout(_scrlFrame, 0, -1, "scrlFrameLayout"); 
	_scrlFrameLayout->setSpacing(3);
	_scrlFrameLayout->setMargin(3);
	//frmViewerLayout->setAlignment(Qt::AlignCenter);

	std::string strSrcPort("");
	std::string strTrgPort("");
	std::string strCarrier("");

	if(config.check("CONNECTIONS")){
		Bottle botConnections = config.findGroup("CONNECTIONS");
		for(int i = 1; i < botConnections.size(); i++){ // 0 is the group title			
			Value valEntry = botConnections.get(i);
			Bottle *botConnection = valEntry.asList();
			if(botConnection != NULL){
				if(botConnection->size() >= 2){
					strSrcPort = std::string(botConnection->get(0).asString().c_str());
					strTrgPort = std::string(botConnection->get(1).asString().c_str());
				}
				if(botConnection->size() >= 3){
					strCarrier = std::string(botConnection->get(2).asString().c_str());
				}
				this->addConnection(strSrcPort, strTrgPort);
			}
		}
	}
	else{
		Bottle bot(config.toString());
		for(int i = 0; i < bot.size(); i++){
			Bottle *botSub = bot.get(i).asList();
			std::string test = config.toString().c_str();			
			if(botSub != NULL){
				std::string test2 = botSub->get(0).asString().c_str();
				if(std::string(botSub->get(0).asString().c_str()).find(std::string("connection")) == 0){
					strSrcPort = botSub->get(1).asString().c_str();
					strTrgPort = botSub->get(2).asString().c_str();
					this->addConnection(strSrcPort, strTrgPort);
				}
			}
		}
	}
	QSpacerItem *spacer = new QSpacerItem( 0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding );
	_scrlFrameLayout->addItem(spacer);
}

QWidgetConnections::~QWidgetConnections(){
	_vecConnections.clear(); 
}

bool QWidgetConnections::addConnection(std::string strSource, std::string strTarget){
	QWidgetConnection *qwCon = new QWidgetConnection(_scrlFrame);
	_scrlFrameLayout->addWidget(qwCon, 0, Qt::AlignTop); // AlignTop not working :-( ?
	qwCon->setSourcePortName(strSource);
	qwCon->setTargetPortName(strTarget);
	_vecConnections.push_back(qwCon);
	qwCon->show();
	return true;
}

void QWidgetConnections::btnConnectAll_clicked(){
	this->connectAllPorts();
}

void QWidgetConnections::btnDisconnectAll_clicked(){
	this->disconnectAllPorts();
}

void QWidgetConnections::btnAddConnection_clicked(){
	this->addConnection(std::string(""), std::string(""));
}

void QWidgetConnections::btnRemoveConnection_clicked(){
	if(_vecConnections.size() > 0){
		QWidget *qwCon = _vecConnections.back();
		_scrlFrameLayout->remove(qwCon);
		_vecConnections.pop_back();
		delete qwCon;
		qwCon = NULL;		
	}
}

bool QWidgetConnections::connectAllPorts(){
	for(int i = 0; i < _vecConnections.size(); i++){
		_vecConnections[i]->connectPorts();
	}
	return true;
}

bool QWidgetConnections::disconnectAllPorts(){
	for(int i = 0; i < _vecConnections.size(); i++){
		_vecConnections[i]->disconnectPorts();
	}
	return true;
}

void QWidgetConnections::updateUI(){
	for(int i = 0; i < _vecConnections.size(); i++){
		_vecConnections[i]->updateUI();
	}
}

