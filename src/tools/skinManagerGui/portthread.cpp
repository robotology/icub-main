#include "portthread.h"

PortThread::PortThread(QObject *parent) :
    QThread(parent)
{
    guiRpcPort = NULL;
    driftCompMonitorPort = NULL;
    driftCompInfoPort = NULL;
}

void PortThread::run()
{
    guiRpcPort = new Port();
    driftCompMonitorPort = new BufferedPort<Vector>();
    driftCompInfoPort = new BufferedPort<Bottle>();

    portCreated();

    exec();

    delete driftCompInfoPort;
    delete driftCompMonitorPort;
    delete guiRpcPort;
}

void PortThread::closePorts()
{
    if(guiRpcPort){
        guiRpcPort->interrupt();
        guiRpcPort->close();
    }
    if(driftCompMonitorPort){
        driftCompMonitorPort->interrupt();
        driftCompMonitorPort->close();
    }
    if(driftCompInfoPort){
        driftCompInfoPort->interrupt();
        driftCompInfoPort->close();
    }
}

void PortThread::stop()
{
    quit();
    while (isRunning()) {
        QThread::msleep(50);
    }
}


Bottle PortThread::sendRpcCommand(bool responseExpected, SkinManagerCommand cmd){
    Bottle resp;
    // check whether the port is connected
    if(guiRpcPort->getOutputCount()==0){
        openErrorDialog(QString("Connection to the rpc port of the skinDriftCompensation\n"
            "module not available. Connect and try again."));
        return resp;
    }

    // create the bottle
    Bottle b;
    b.addInt(cmd);
    //g_print("Going to send rpc msg: %s\n", b.toString().c_str());
    if(responseExpected){
        guiRpcPort->write(b, resp);
    }else{
        guiRpcPort->write(b);
    }
    return resp;
}
