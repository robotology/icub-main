#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Network yarp;
    ResourceFinder rf;
    string guiName;
    unsigned int gXpos, gYpos;

    rf.setDefaultConfigFile("skinManGui.ini");      //overridden by --from parameter
    rf.setDefaultContext("skinGui");                    //overridden by --context parameter
    rf.configure(argc, argv);

    gXpos=10;
    gYpos=10;
    if (rf.check("xpos")) gXpos=rf.find("xpos").asInt();
    if (rf.check("ypos")) gYpos=rf.find("ypos").asInt();

//    string driftCompRpcPortName     = rf.check("skinManRpcPort", Value("/skinManager/rpc")).asString().c_str();
//    string driftCompMonitorPortName = rf.check("skinManMonitorPort", Value("/skinManager/monitor:o")).asString().c_str();
//    string driftCompInfoPortName    = rf.check("skinManInfoPort", Value("/skinManager/info:o")).asString().c_str();


    MainWindow w(&rf);
    w.show();

    return (a.exec()!=0?1:0);
}



