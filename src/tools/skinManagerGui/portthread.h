#ifndef PORTTHREAD_H
#define PORTTHREAD_H

#include <QThread>

#include <string>
#include <sstream>
#include <iomanip>					// io manipulator (setw, setfill)
#include <cstdarg>
//#include <cstdlib>
#include <math.h>
#include <vector>


#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <iCub/skinDynLib/rpcSkinManager.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinManager;

class PortThread : public QThread
{
    Q_OBJECT
public:
    explicit PortThread(QObject *parent = 0);
    void closePorts();
    void stop();
    Bottle sendRpcCommand(bool responseExpected, SkinManagerCommand cmd);

protected:
    void run();

public:
    Port					*guiRpcPort;             // to send rpc command to the module
    BufferedPort<Vector>	*driftCompMonitorPort;   // for reading streaming data (frequency, drift)
    BufferedPort<Bottle>	*driftCompInfoPort;      // for reading sporadic msgs (errors, warnings)

signals:

    void portCreated();
    void openErrorDialog(QString);

public slots:

};

#endif // PORTTHREAD_H
