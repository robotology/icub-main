// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<iCub/QWidgetSalience.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::gui;
using namespace iCub::contrib;

QWidgetSalience::QWidgetSalience():
	_qwViewer(NULL),
	_qwOutput(NULL),
	_qwRPC(NULL),
	_qwConnections(NULL),
	_qwSalience(NULL) {

}

QWidgetSalience::~QWidgetSalience(){
   
}


QWidgetSalience::QWidgetSalience(Searchable& config, QWidget* parent, const char* name, bool modal, WFlags fl)
	: QWidget(parent, name, fl),
	_qwViewer(NULL),
	_qwOutput(NULL),
	_qwRPC(NULL),
	_qwConnections(NULL),
	_qwSalience(NULL){	

	Property prop;
	prop.fromString(config.toString());
	if(!prop.check("name")){
		prop.put("name", "/guiSalience");
	}

	this->setCaption("iCub Salience Module Interface");

	// main layout (vbox)
    QVBoxLayout *mainLayout = new QVBoxLayout( this, 3, 3, "QWidgetSalienceBaseLayout"); 

	// add a vertical split as first entry to main layout
	QSplitter *splitMain = new QSplitter(this);
	mainLayout->addWidget(splitMain);

	// add horizontal splitter to left part of main splitter
	QSplitter *splitLeft = new QSplitter(Qt::Vertical, splitMain);
	Property propViewer; propViewer.fromString(config.toString());
	propViewer.put("name", std::string(std::string(propViewer.find("name").asString().c_str()) + std::string("/viewer")).c_str());
	_qwViewer = new QWidgetViewer(propViewer, splitLeft);
	_qwConnections = new QWidgetConnections(config, splitLeft);

	// add horizontal splitter to right part of main splitter
	QSplitter *splitRight = new QSplitter(Qt::Vertical, splitMain);
	Property propSalienceCtrl; propSalienceCtrl.fromString(config.toString());
	propSalienceCtrl.put("name", std::string(std::string(propSalienceCtrl.find("name").asString().c_str()) + std::string("/salienceControls")).c_str());
	_qwSalience = new GuiSalience(propSalienceCtrl, splitRight);
	Property propOutput; propOutput.fromString(config.toString());
	propOutput.put("name", std::string(std::string(propOutput.find("name").asString().c_str()) + std::string("/stdout")).c_str());
	_qwOutput = new QWidgetOutput(propOutput, splitRight);
	Property propRPC; propRPC.fromString(config.toString());
	propRPC.put("name", std::string(std::string(propRPC.find("name").asString().c_str()) + std::string("/rpc")).c_str());
	_qwRPC = new QWidgetRPC(propRPC, splitRight);
	
	// add a frame as second entry in main layout
	QFrame *frmBottom = new QFrame(this);
	frmBottom->setMaximumHeight(35);
	mainLayout->addWidget(frmBottom);

	// add a layout to the bottom frame
	QHBoxLayout *frmBottomLayout = new QHBoxLayout(frmBottom, 0, -1, "frmBottomLayout"); 
	frmBottomLayout->setSpacing(3);
	frmBottomLayout->setMargin(3);

	// add a button to the bottom frame
	QPushButton *btnCheckAll = new QPushButton(frmBottom);
	btnCheckAll->setText("check all ports and connections");
	frmBottomLayout->addWidget(btnCheckAll);
	connect( btnCheckAll, SIGNAL( clicked() ), this, SLOT( btnCheckAll_clicked() ) );

	this->resize(850,650);
	QValueList<int> valsSplitMain;
	valsSplitMain.append(400);
	valsSplitMain.append(450);
	splitMain->setSizes(valsSplitMain);
	QValueList<int> valsSplitLeft;
	valsSplitLeft.append(400);
	valsSplitLeft.append(250);
	splitLeft->setSizes(valsSplitLeft);
	QValueList<int> valsSplitRight;
	valsSplitRight.append(350);
	valsSplitRight.append(150);
	valsSplitRight.append(150);
	splitRight->setSizes(valsSplitRight);

	// position screen center
    QWidget* desk = QApplication::desktop();
    this->move(desk->width()/2 - this->width()/2,desk->height()/2 - this->height()/2);

}


void QWidgetSalience::btnCheckAll_clicked(){
	_qwConnections->updateUI();
	_qwViewer->updateUI();
	_qwOutput->updateUI();
	_qwRPC->updateUI();
	_qwSalience->updateUI();
}
