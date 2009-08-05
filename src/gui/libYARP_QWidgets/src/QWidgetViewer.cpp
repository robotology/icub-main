// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/QWidgetViewer.h>

#define DEBUG 0

using namespace yarp::gui;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


QWidgetViewer::QWidgetViewer(QWidget* parent, const char* name, bool showInOrigSize)
	: QWidgetViewerBase( parent, name, NULL ),
	_qwConnection(NULL),
	_qwConnectionClick(NULL),
	_qwVideo(NULL),
	_counter(0),
	_time(0),
	_width(0),
	_height(0),
	_fixedSize(showInOrigSize),
	_hasClickPort(false){

	Property p;
	this->init(p, parent, name, showInOrigSize);	
}

QWidgetViewer::QWidgetViewer(Searchable &config, QWidget* parent, const char* name, bool showInOrigSize)
	: QWidgetViewerBase( parent, name, NULL ),
	_qwConnection(NULL),
	_qwConnectionClick(NULL),
	_qwVideo(NULL),
	_counter(0),
	_time(0),
	_width(0),
	_height(0),
	_fixedSize(showInOrigSize),
	_hasClickPort(false){

	this->init(config, parent, name, showInOrigSize);
	this->open(config);
}

QWidgetViewer::~QWidgetViewer(){
	this->close();
}

void QWidgetViewer::init(Searchable &config, QWidget* parent, const char* name, bool showInOrigSize){

	if(config.check("VIEWER")){
		if(config.findGroup("VIEWER").check("click")){
			_hasClickPort = true;
		}
	}
	else{
		if(config.check("click")){
			_hasClickPort = true;
		}
	}

	if(!_hasClickPort){
		grbConnection->setColumnLayout(1, Qt::Vertical );
	}
	else{
		grbConnection->setColumnLayout(2, Qt::Vertical );
	}
	grbConnection->layout()->setSpacing(3);
    grbConnection->layout()->setMargin(3);

	_qwConnection = new QWidgetConnection(grbConnection, "qwConnection");

	if(_hasClickPort){
		_qwConnectionClick = new QWidgetConnection(grbConnection, "qwConnectionClick");	
	}

	QVBoxLayout *frmViewerLayout = new QVBoxLayout(frmViewer, 0, -1, "frmViewerLayout"); 
	frmViewer->layout()->setSpacing(3);
	frmViewer->layout()->setMargin(3);
	frmViewerLayout->setAlignment(Qt::AlignCenter);

	// setup for standalone version with stretchable rendering
	if(!showInOrigSize){
		_qwVideo = new QGLVideo(frmViewer, "qwVideo");
		_qwVideo->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
		frmViewerLayout->addWidget(_qwVideo);
	}
	// setup for integrated version with 1:1 rendering and scrollview
	else{
		 // scroll view
		QScrollView *scrlView = new QScrollView(frmViewer);
		frmViewerLayout->addWidget(scrlView);
		scrlView->setResizePolicy(QScrollView::AutoOneFit);

		QFrame *scrlFrame = new QFrame(scrlView->viewport());
		scrlFrame->setMargin(0);
		scrlView->addChild(scrlFrame);
		QGridLayout *scrlFrameLayout = new QGridLayout(scrlFrame, 1, 1, 0, 0, "scrlFrameLayout"); 

		_qwVideo = new QGLVideo(scrlFrame, "qwVideo");
		scrlFrameLayout->addWidget(_qwVideo, 0, 0 );
	}
}

bool QWidgetViewer::open(Searchable & config){

	bool ok = true;
	string strLocal("");
	string strRemote("");
	string strLocalClick("");
	string strRemoteClick("");

	// read configuration
	if(config.check("name")){
		strLocal = std::string(config.find("name").asString().c_str()) + std::string("/image_in");
		strLocalClick = std::string(config.find("name").asString().c_str()) + std::string("/click_out");
	}
	if(config.check("VIEWER")){
		strRemote = config.findGroup("VIEWER").find("remote").asString().c_str();
		if(_hasClickPort){
			strRemoteClick = config.findGroup("VIEWER").find("click").asString().c_str();
		}
	}
	else{
		if(config.check("remote")){
			strRemote = config.find("remote").asString().c_str();
		}
		else{
			strRemote = "/module/image_out";
		}
		if(config.check("click")){
			if(_hasClickPort){
				strRemoteClick = config.find("click").asString().c_str();
			}
		}
	}
	
	// open image reading port
	ok = ok && _prtImgIn.open(strLocal.c_str());

	// open click writing port
	if(_hasClickPort){
		ok = ok && _prtClickOut.open(strLocalClick.c_str());
	}

	// setup connection widget
	_qwConnection->setSourcePortName(strRemote.c_str());
	_qwConnection->setTargetPortName(strLocal.c_str());
	_qwConnection->connectPorts(); // try to connect (don't care if doesn't work)

	// setup click connection widget
	if(_qwConnectionClick != NULL){
		_qwConnectionClick->setSourcePortName(strLocalClick.c_str());
		_qwConnectionClick->setTargetPortName(strRemoteClick.c_str());
		_qwConnectionClick->connectPorts();
	}

	// start the update loop
	QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerDone()) );
    timer->start(50, FALSE ); // aim at 20 fps

	return ok;
}

bool QWidgetViewer::close(){
	if(_hasClickPort){
		_prtClickOut.interrupt();
		_prtClickOut.close();
	}
	_prtImgIn.interrupt();
	_prtImgIn.close();
	return true;
}

void QWidgetViewer::setImageSourcePortName(std::string name){
	_qwConnection->setSourcePortName(name);
}

std::string QWidgetViewer::getImageSourcePortName(){
	return _qwConnection->getSourcePortName();
}

void QWidgetViewer::setLocalPortName(std::string name){
	_qwConnection->setTargetPortName(name);
}

std::string QWidgetViewer::getLocalPortName(){
	return _qwConnection->getTargetPortName();
}

void QWidgetViewer::timerDone(){

	if(this->isVisible()){

		double startTime = 0.0;
		if(DEBUG == 1){
			startTime = yarp::os::Time::now();
		}

		// read and draw image
		ImageOf<PixelRgb> *img = _prtImgIn.read(false);
		if(img != NULL){
			// check for change in size of incoming image
			if(_width != img->width() || _height != img->height()){
				_width = img->width();
				_height = img->height();
				// adjust viewer widget size
				if(_fixedSize){
					_qwVideo->setMinimumSize(_width,_height);
					_qwVideo->setMaximumSize(_width,_height);
				}
			}
			_qwVideo->paintImage(*img);
		}
		else{
			//std::cout << "missing image at: " << yarp::os::Time::now() << endl;
		}

		// write mouse click
        if(_hasClickPort){
			int mouseX = -1;
			int mouseY = -1;
			if(_qwVideo->getMouseClick(mouseX, mouseY)){
				yarp::os::Bottle& bot = _prtClickOut.prepare();
				bot.clear();
				bot.addInt(mouseX);
				bot.addInt(mouseY);
				_prtClickOut.write();
			}
        }

		double stopTime = 0.0;
		if(DEBUG == 1){
			stopTime = yarp::os::Time::now();
			if(_counter < 20){
				_time += (stopTime - startTime);
			}
			else{
				_counter = 0;
				cout << "framerate: " << 1.0/(_time/20.0) << endl;
				_time = 0.0;
			}
			_counter++;
		}

	}
}

void QWidgetViewer::updateUI(){
	if(_qwConnection != NULL){
		_qwConnection->updateUI();
	}
	if(_qwConnectionClick != NULL){
		_qwConnectionClick->updateUI();
	}
}
