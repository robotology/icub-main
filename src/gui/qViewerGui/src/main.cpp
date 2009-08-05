// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/QViewer.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

namespace iCub {
    namespace contrib {
        class QViewerGuiModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * qviewergui module class
 *
 * \see icub_qviewergui
 *
 */
class iCub::contrib::QViewerGuiModule : public Module {

private:
    Property _config;

public:

    QViewerGuiModule(){
        
    }

    virtual ~QViewerGuiModule(){
    }

    virtual bool open(Searchable& config) {
        _config.fromString(config.toString());
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
        QViewer *gui = new QViewer(_config);
        app.setMainWidget(gui);
        gui->show();
        app.exec();
        delete gui;
        return false;
    }

};

/**
 * @ingroup icub_module
 *
 * \defgroup icub_qviewergui qviewergui
 *
 * An alternative image stream viewer using Qt and OpenGL
 * 
 * \see iCub::contrib::QViewer
 *
 * \author Jonas Ruesch
 *
 */
int main(int argc, char *argv[]) {

    Network yarp;
    QViewerGuiModule module;
    module.setName("/qviewergui"); // set default name of module
    return module.runModule(argc,argv);
}
