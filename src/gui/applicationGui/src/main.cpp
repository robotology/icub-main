// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/ResourceFinder.h>

// iCub
#include <yarp/gui/QWidgetApplication.h>
#include <yarp/gui/QWidgetFactory.h>
#include <yarp/gui/QWidgetModuleDefault.h>
#include <yarp/gui/QWidgetViewer.h>
#include <yarp/gui/QWidgetConnections.h>
#include <yarp/gui/QWidgetControlboard.h>
#include <iCub/QWidgetSalience.h>
#include <iCub/GuiSalience.h>

// Qt
#include <qapplication.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::gui;
using namespace iCub::contrib;

namespace iCub {
	namespace contrib {
		class GuiApplication;
	}
}

using namespace iCub::contrib;

/**
*
* applicationGui module class
*
* \see icub_applicationgui
*
*/
class iCub::contrib::GuiApplication : public Module {

private:
	Property _config;

public:

	GuiApplication(){

	}

	virtual ~GuiApplication(){
	}

	virtual bool open(Searchable& config) {

		// locate configuration file
		ResourceFinder rf;        
		if (config.check("context")){
			rf.setDefaultContext(config.find("context").asString());
		}
		if (config.check("from")){
			rf.setDefaultConfigFile(config.find("from").asString());
		}
		else if (config.check("file")){
			rf.setDefaultConfigFile(config.find("file").asString());
		}
		else{
			rf.setDefaultConfigFile("icubAttentionSelection.ini");
		}
		rf.configure("ICUB_ROOT",0,NULL);
		_config.fromString(rf.toString(), false);
		_config.fromString(config.toString(), false);
		_config.setMonitor(config.getMonitor());

		if (!_config.check("name"))
			_config.put("name", getName());
		_config.setMonitor(config.getMonitor());
		return true;
	}

	virtual bool close() {
		return true;
	}

	virtual bool interruptModule() {
		return true;
	}

	virtual bool updateModule() {

		// initialize Qt
		int argc = 0;
		QApplication app(argc,NULL);
		QWidgetApplication *gui = new QWidgetApplication(_config);
		app.setMainWidget(gui);
		gui->show();
		app.exec();
		delete gui;
		return false;
	}

};

/**
* @ingroup icub_module
* @ingroup icub_guis
*
* Attention system main gui.
*
* \defgroup icub_applicationgui applicationgui
*
* 
* 
* \see iCub::contrib::GuiApplication
*
* \author Jonas Ruesch
*
*/
int main(int argc, char *argv[]) {

	QWidgetFactories& pool = QWidgetFactories::getPool();
	pool.add(new QWidgetFactoryOf<QWidgetModuleDefault>("default"));
	pool.add(new QWidgetFactoryOf<QWidgetSalience>("salience"));
	pool.add(new QWidgetFactoryOf<GuiSalience>("salience_controls"));
	pool.add(new QWidgetFactoryOf<QWidgetViewer>("viewer"));
	pool.add(new QWidgetFactoryOf<QWidgetRPC>("rpc"));
	pool.add(new QWidgetFactoryOf<QWidgetConnections>("connections"));
	pool.add(new QWidgetFactoryOf<QWidgetControlboard>("controlboard"));

	Network yarp;
	GuiApplication module;
	module.setName("/applicationGui"); // set default name of module
	return module.runModule(argc,argv);
}
