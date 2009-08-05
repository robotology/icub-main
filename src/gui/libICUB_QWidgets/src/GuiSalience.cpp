/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/GuiSalience.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::gui;
using namespace iCub::contrib;

GuiSalience::GuiSalience():
	frmMainLayout(NULL),
    _scrlView(NULL),
    _scrlFrame(NULL),
	_scrlFrameLayout(NULL),
	_spacer(NULL),
	_qwConfConnection(NULL){

}

GuiSalience::GuiSalience(Searchable& config, QWidget* parent, const char* name, bool modal, WFlags fl)
	: GuiSalienceBase( parent, name, fl ),
	frmMainLayout(NULL),
    _scrlView(NULL),
    _scrlFrame(NULL),
	_scrlFrameLayout(NULL),
	_spacer(NULL),
	_qwConfConnection(NULL)
{

	Property prop;
	if(!config.check("name")){
		prop.put("name", "/salienceGui");
	}
	else{
		prop.put("name", config.find("name"));
	}
	if(config.check("REMOTE_SALIENCE")){
		prop.put("remote", config.findGroup("REMOTE_SALIENCE").find("remote").asString().c_str());
	}
	else{
		if(config.check("remote")){
			prop.put("remote", config.find("remote").asString().c_str());
		}
		else{
			prop.put("remote", "/salience/conf");
		}
	}

	// salience controls
	// *****************
    frmMainLayout = new QVBoxLayout( frmMain, 3, 3, "frmMainLayout"); 
    
        // scroll view
    _scrlView = new QScrollView(frmMain);
    frmMainLayout->addWidget(_scrlView);
    _scrlView->setResizePolicy(QScrollView::AutoOneFit);

    // frame for filter widgets
    _scrlFrame = new QFrame(_scrlView->viewport());
    _scrlView->addChild(_scrlFrame);
	_scrlFrameLayout = new QVBoxLayout(_scrlFrame, 3, 3, "_scrlFrameLayout");
	
	// setup connection widget
	grbConfigurationConnection->setColumnLayout(0, Qt::Vertical );
    QVBoxLayout *grbConnectionLayout = new QVBoxLayout(grbConfigurationConnection->layout());
	grbConfigurationConnection->layout()->setSpacing( 3 );
    grbConfigurationConnection->layout()->setMargin( 3 );
    grbConnectionLayout->setAlignment(Qt::AlignTop);
	_qwConfConnection = new QWidgetConnection(grbConfigurationConnection);
    grbConnectionLayout->addWidget(_qwConfConnection);

	yarp::os::ConstString strLocalPort = std::string(std::string(prop.find("name").asString().c_str())+ std::string("/conf")).c_str();
	yarp::os::ConstString strRemotePort = prop.find("remote").asString();

	_remoteSalience.open(strLocalPort);
    
	_qwConfConnection->setTargetPortName(strRemotePort.c_str());
	_qwConfConnection->setSourcePortName(strLocalPort.c_str());

    this->initializeUI();

    // line edit validators
    lneThreshold->setValidator(new QDoubleValidator(this));
    lneBlur->setValidator(new QIntValidator(this));

    // set gui values to current values
    QVariant thr(_remoteSalience.getSalienceThreshold());
    lneThreshold->setText(thr.asString());
    QVariant nb(_remoteSalience.getNumBlurPasses());
    lneBlur->setText(nb.asString());

}

GuiSalience::~GuiSalience()
{
    _remoteSalience.close();
}


void GuiSalience::lneThreshold_returnPressed()
{
    QVariant thr(lneThreshold->text());
	_remoteSalience.setSalienceThreshold(thr.asDouble());
}


void GuiSalience::lneBlur_returnPressed()
{
    QVariant nb(lneBlur->text());
    _remoteSalience.setNumBlurPasses(nb.asInt());
}

void GuiSalience::btnReinitialize_clicked(){
	this->initializeUI();
}

void GuiSalience::initializeUI(){
	while(!_lstFilterWidgets.empty()){
		delete _lstFilterWidgets[_lstFilterWidgets.size()-1];
		_lstFilterWidgets.pop_back();
	}
	int numFilters = _remoteSalience.getChildCount();
	for (int i = 0; i < numFilters; i++){
		GuiSalienceFilter *filter = new GuiSalienceFilter(i, &_remoteSalience, _scrlFrame);
		_scrlFrameLayout->addWidget(filter);
		filter->show();
        _lstFilterWidgets.push_back(filter);
	}
	if(_spacer != NULL){
		_scrlFrameLayout->removeItem(_spacer);
		delete _spacer;
		_spacer = NULL;
	}
	_spacer = new QSpacerItem( 0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding );
	_scrlFrameLayout->addItem(_spacer);
}

void GuiSalience::updateUI(){
	if(_qwConfConnection != NULL){
		_qwConfConnection->updateUI();
	}
}

//void GuiSalience::lneTmpBlur_returnPressed()
//{
//    QVariant size(lneTmpBlur->text());
//    _remoteSalience.setTemporalBlur(size.asInt());
//}
