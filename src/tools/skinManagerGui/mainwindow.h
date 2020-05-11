#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "portthread.h"
#include "loadingwidget.h"

#include <QTimer>

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


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ResourceFinder *rf, QWidget *parent = 0);
    ~MainWindow();
public:
    void initGui();
    bool initGuiStatus();
    void printLog(QString);
    double round(double value, int decimalDigit);

private:
    void changeSmooth(int val);
    void setMsgFreq(bool freqUpdated, double freq);

private:
    Ui::MainWindow *ui;
    ResourceFinder *rf;
    PortThread portThread;
    LoadingWidget loadingWidget;
    QTimer calibratingTimer;
    QTimer updateTimer;
    bool smoothSliderPressed;
    double prevFreq;

    // global data
    uint                   timeoutId;              // id of the timeout callback function
    vector<string>			portNames;				// names of the skin input ports
    vector<unsigned int>	portDim;				// number of taxels of the input ports
    double					currentSmoothFactor;    // current smooth factor value
    bool                    initDone;               // true if the gui has been initialized
    unsigned int            currentThreshold;       // current safety threshold
    double                  currentCompGain;        // current compensation gain
    double                  currentContCompGain;    // current contact compensation gain
    double                  currentMaxNeighDist;    // current max neighbor distance
    int                     currentSampleFreq;
    int                     currentSampleNum;       // size of the dataPlot array


private slots:
    void onUpdateTimer();
    void onPortCreated();
    void onOpenErrorDialog(QString);
    void onCalibratingTimer();
    void onCalibrate();
    void onThreashold();
    void onBinarization(bool btnState);
    void onSmooth(bool btnState);
    void onSmoothValueChanged(int);
    void onSmoothValuePressed();
    void onSmoothValueReleased();
    void onSpinNeighborChanged(double);
    void onSpinCompContactGainChanged(double);
    void onSpinCompGainChanged(double);
    void onSpinThresholdChanged(int);
    void onSampleFreqChanged(int);

};

#endif // MAINWINDOW_H
