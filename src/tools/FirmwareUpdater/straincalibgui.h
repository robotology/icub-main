#ifndef STRAINCALIBGUI_H
#define STRAINCALIBGUI_H

#include <QDialog>
#include <QSignalMapper>
#include <QFile>
#include <stdio.h>
#include <QLineEdit>
#include <QTimer>
#include <QFuture>
#include <QFutureWatcher>
#include <QValidator>

//#include <conio.h>
//#include <time.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <deque>
#include "expected_values.h"
#include "strainInterface.h"
#include "firmwareupdatercore.h"
namespace Ui {
class StrainCalibGui;
}

class StrainCalibGui : public QDialog
{
    Q_OBJECT

public:
    explicit StrainCalibGui(QString device, int bus , int pid,
                            FirmwareUpdaterCore *core,QWidget *parent = 0);
    ~StrainCalibGui();

private:
    void turnOnButtons();
    void turnOffButtons();
    void showMenu();
    void bias_sensor();
    void terminate_section();
    void close_files();
    int remap_trials();
    bool get(const unsigned int number, vector<cDownloader::strain_value_t> &values, bool debugprint = true);
    bool print(const vector<cDownloader::strain_value_t> &values, FILE *fp, QList<float> = QList<float>());
#ifdef ATI_SENS
    void acquire_1000_samples();
#else
    bool acquire_samples(int samples);
#endif

private:
    Ui::StrainCalibGui *ui;
    QDoubleValidator validator;
    QFutureWatcher<bool> watcher;
    strainInterface::Config config;
    FirmwareUpdaterCore *core;
    QSignalMapper signalMapper;
    QList <QPushButton*> buttons;
    QList <QLineEdit*> readValues;
    QList <QLineEdit*> expValues;
    QList <QLineEdit*> errValues;
    int currentBtnIndex;
    expected_values_handler_class expected_values_handler;
    //strainInterface sI;
    int trial;
    FILE *fp;
    QFile file;
    signed_elem_class           trial_bias;
    signed_elem_class           last_value;
    QTimer timer;
    int current_trial;

    bool isSamplesAcquisitionActive;
    QMutex mutex;
    bool tick_acquisition(int samples);

    bool enabledebugprints;

private slots:
    void onButtonClick(bool b);
    void onAcquireData(bool b);
    void onTimerTimeout();
    void onFutureFinished();
    void onFreeAcqMode(bool);
};

#endif // STRAINCALIBGUI_H
