// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iCub/QViewer.h>

using namespace iCub::contrib;
using namespace yarp::os;
using namespace yarp::gui;
using namespace std;

QViewer::QViewer(){

}

QViewer::QViewer(Searchable& config, QWidget* parent, const char* name, bool modal, WFlags fl)
	: QViewerBase( parent, name, fl ),
	_qwViewer(NULL){

	Property prop;
	prop.fromString(config.toString());
	if(!prop.check("name")){
		prop.put("name", "/qViewer");
	}

	this->setCaption("Yarp Image Viewer");

	// main layout (vbox)
    QVBoxLayout *mainLayout = new QVBoxLayout( this, 1, 1, "QWidgetModuleDefaultBaseLayout"); 

	_qwViewer = new QWidgetViewer(prop, this);
	mainLayout->addWidget(_qwViewer);

	this->resize(400,350);

	// position screen center
    /*QWidget* desk = QApplication::desktop();
    this->move(desk->width()/2 - this->width()/2,desk->height()/2 - this->height()/2);*/

}

QViewer::~QViewer()
{
	//_qwViewer closes itself on destruction
}


