// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <yarp/gui/QWidgetApplication.h>
#include <yarp/gui/QWidgetFactory.h>

#include <qtabwidget.h>
#include <qsplitter.h>

#include <yarp/os/Property.h>

#include <sstream>

using namespace yarp::gui;
using namespace yarp::os;
using namespace std;


QWidgetApplication::QWidgetApplication(QWidget* parent, const char* name) :
	_baseLayout(NULL),
	_tabWidget(NULL) {
	this->init(parent, name);
}

QWidgetApplication::QWidgetApplication(Searchable &config, QWidget* parent, const char* name) :
	QWidget(parent, name),
	_baseLayout(NULL),
	_tabWidget(NULL) {
	this->init(parent, name);
	this->open(config);
}

QWidgetApplication::~QWidgetApplication(){
	this->close();
}

void QWidgetApplication::init(QWidget* parent, const char* name){

	this->setName( "QWidgetApplication" );
    _baseLayout = new QVBoxLayout( this, 3, 3, "QWidgetApplicationLayout"); 
	this->clearWState(WState_Polished);
	this->setCaption( tr( "Yarp Application Interface" ) );
	this->resize(850,650);
}

bool QWidgetApplication::open(Searchable & config){

	bool ok = true;
 
	// single widget configuration (either in [WIDGET] config file or from command line without any group)
	if(config.check("WIDGET") || config.check("guiWidget")){
		ok = this->loadWidget(config, this, _baseLayout) && ok;
	}
	// single page configuration
	else if(config.check("PAGE")){
		ok = this->loadPage(config, this, _baseLayout) && ok;
	}
	// application configuration
	else if(config.check("APPLICATION")){
		ok = this->loadApplication(config, this, _baseLayout) && ok;
	} 
	
	return ok;
}

bool QWidgetApplication::close(){
	
	return true;
}


bool QWidgetApplication::loadWidget(yarp::os::Searchable &config, QWidget* parent, QLayout* parentLayout){
	bool ok = true;
	if(config.check("guiWidget")){ 
		// found a guiWidget definition
		std::string strGuiWidget = config.find("guiWidget").asString().c_str();
		std::string strGuiDisplayName = config.check ("guiDisplayName", Value("Yarp Module Interface")).asString().c_str();					
		QWidgetFactories& pool = QWidgetFactories::getPool();
		QWidgetYarp *widget = pool.get(strGuiWidget.c_str(), config, parent);	// instantiate a widget with the widget factory (guiWidget, guiConfig)		
		// add widget
		if(widget != NULL){
			if(widget->getQWidget() != NULL){
				parent->setCaption(strGuiDisplayName.c_str());
				parentLayout->add(widget->getQWidget());
				ok = ok && true;
			}
			else{
				ok = false;
			}
		}
		else{
			QLabel *lblMessage = new QLabel(std::string(std::string("\n\n\nA widget registered as '") + strGuiWidget + std::string("' was not found or could not be instantiated.\nPlease check your configuration.")).c_str(), parent);
			lblMessage->setAlignment(Qt::AlignHCenter);
			parentLayout->add(lblMessage);
			ok = false;
		}
	} // guiWidget found
	else{
		QLabel *lblMessage = new QLabel("\n\n\nNo 'guiWidget' key found in configuration.", parent);
		lblMessage->setAlignment(Qt::AlignHCenter);
		parentLayout->add(lblMessage);
		ok = false;
	}
	return ok;
}

bool QWidgetApplication::loadPage(yarp::os::Searchable &config, QWidget* parent, QLayout* parentLayout){

	bool ok = true;

	// set the caption of this page (in case it is the main widget)
	std::string strGuiDisplayName = "No Name";
	if(config.check("guiDisplayName")){
		strGuiDisplayName = config.find("guiDisplayName").asString().c_str();
	}
	parent->setCaption(strGuiDisplayName.c_str());

	// add the splitters

	// add a vertical split as first entry to main layout
	QSplitter *splitMain = new QSplitter(parent);
	parentLayout->add(splitMain);

	// add horizontal splitter to left part of main splitter
	QSplitter *splitLeft = new QSplitter(Qt::Vertical, splitMain);
	QWidget *qwTopLeft = new QWidget(splitLeft);
	QVBoxLayout *layoutTopLeft = new QVBoxLayout(qwTopLeft, 0, 0); 
	QWidget *qwBottomLeft = new QWidget(splitLeft);
	QVBoxLayout *layoutBottomLeft = new QVBoxLayout(qwBottomLeft, 0, 0); 

	// add horizontal splitter to right part of main splitter
	QSplitter *splitRight = new QSplitter(Qt::Vertical, splitMain);
	QWidget *qwTopRight = new QWidget(splitRight); 
	QVBoxLayout *layoutTopRight = new QVBoxLayout(qwTopRight, 0, 0); 
	QWidget *qwBottomRight = new QWidget(splitRight); 
	QVBoxLayout *layoutBottomRight = new QVBoxLayout(qwBottomRight, 0, 0); 

	// load widgets 
	Bottle bot(config.toString());
	for(int i = 0; i < bot.size(); i++){
		Value &val = bot.get(i);
		Bottle *lstBot = val.asList(); // a normal key-value pair is also a list (!)
		if(lstBot != NULL){
			Property prop(lstBot->toString());
			if(prop.check("guiConfig") && prop.check("position")){ 
				Property propWidget;
				propWidget.fromConfigFile(prop.find("guiConfig").asString().c_str());	
				if(std::string(prop.find("position").asString().c_str()) == std::string("topLeft")){
					ok = this->loadWidget(propWidget, qwTopLeft, layoutTopLeft) && ok;
				}
				if(std::string(prop.find("position").asString().c_str()) == std::string("topRight")){
					ok = this->loadWidget(propWidget, qwTopRight, layoutTopRight) && ok;
				}
				if(std::string(prop.find("position").asString().c_str()) == std::string("bottomLeft")){
					ok = this->loadWidget(propWidget, qwBottomLeft, layoutBottomLeft) && ok;
				}
				if(std::string(prop.find("position").asString().c_str()) == std::string("bottomRight")){
					ok = this->loadWidget(propWidget, qwBottomRight, layoutBottomRight) && ok;
				}
			}
		}
	}
	return ok;
}

bool QWidgetApplication::loadApplication(yarp::os::Searchable &config, QWidget *parent, QLayout* parentLayout){

	bool ok = true;

	// set the caption of this page (in case it is the main widget)
	std::string strGuiDisplayName = "Yarp Application Interface";
	if(config.check("guiDisplayName")){
		strGuiDisplayName = config.find("guiDisplayName").asString().c_str();
	}
	parent->setCaption(strGuiDisplayName.c_str());

	// add a QTabWidget
	QTabWidget *tabWidget = new QTabWidget(this, "tabWidget");
	_baseLayout->addWidget(tabWidget);
	tabWidget->show();

	// load pages
	Bottle bot(config.toString());
	for(int i = 0; i < bot.size(); i++){		
		Value &val = bot.get(i);
		Bottle *lstBot = val.asList();
		if(lstBot != NULL){
			Property prop(lstBot->toString());
			if(prop.check("guiConfig")){ 

				std::string strGuiConfig = prop.find("guiConfig").asString().c_str();
				Property subProp;
				subProp.fromConfigFile(strGuiConfig.c_str());	

				std::string strTabName = "No Name";
				if(subProp.check("guiDisplayName")){
					strTabName = subProp.find("guiDisplayName").asString().c_str();
				}
				QWidget *qwPage = new QWidget(this);
				QVBoxLayout *layoutPage = new QVBoxLayout(qwPage, 0, 0);
				tabWidget->addTab(qwPage, strTabName.c_str());

				// if sub-configuration describes a page
				if(subProp.check("PAGE")){
					ok = this->loadPage(subProp, qwPage, layoutPage) && ok;
				}
				// if sub-configuration describes a widget
				else{
					ok = this->loadWidget(subProp, qwPage, layoutPage) && ok;
				}
			} // if key guiConfig found
		} // if list not empty		
	} // for all bottle entries
	return ok;
}


// load default gui widgets specified by --modules <num> property
//if(config.check("modules")){
//	int numDefaultModules = config.find("modules").asInt();
//	stringstream sstr("");
//	for (int n = 0; n < numDefaultModules; n++){
//		sstr.str("");
//		sstr << (n+1);
//		std::string strGuiDisplayName = std::string("Module ") + sstr.str();
//		std::string strGuiPortName = std::string("/guiModule") + sstr.str();
//		std::string strGuiWidget = "default";
//		Property propDefault;
//		propDefault.put("name", strGuiPortName.c_str());
//		// instantiate a widget with the widget factory (guiWidget, guiConfig)
//		QWidgetFactories& pool = QWidgetFactories::getPool();
//		QWidgetYarp *widget = pool.get(strGuiWidget.c_str(), propDefault, this);

//		// add a tab with guiName name and widget
//		if(widget != NULL){
//			if(widget->getQWidget() != NULL){
//				_tabWidget->addTab(widget->getQWidget(), QString(strGuiDisplayName.c_str()));
//			}
//		}
//	} // for each default gui
//} // --modules <num>


