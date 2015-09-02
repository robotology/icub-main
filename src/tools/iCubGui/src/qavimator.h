/*
 * qavimator.h
 */

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Copyright (c) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alessandro Scalzo <alessandro.scalzo@iit.it>
 *          Davide Perrone <dperrone@aitek.it>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Based on:
 *
 *   Qavimator
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *   Released under the terms of the GNU GPL v2.0.
 */


#ifndef QAVIMATOR_H
#define QAVIMATOR_H

#include <QMainWindow>
#define UNTITLED_NAME "Untitled.avm"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include "playstate.h"

#include <QLineEdit>

namespace Ui {
class qavimator;
}

class qavimator : public QMainWindow
{
    Q_OBJECT

public:
    explicit qavimator(yarp::os::ResourceFinder& config,QWidget *parent = 0);
    ~qavimator();
    void fileExit();

private:
    Ui::qavimator *ui;
    int nFPS;
    int width;
    int height;

protected:
    // prevent closing of main window if there are unsaved changes
    virtual void closeEvent(QCloseEvent* event);
    void configure();
    void helpAbout();
    void setupToolBar();
    void setFPS(int fps);

signals:
    void enableRotation(bool);
    void resetCamera();

protected slots:
    void readSettings();
    void configChanged();
    void backgroundClicked();
    void on_fileExitAction_triggered();
    void on_optionsConfigureiCubGUIAction_triggered();
    void on_helpAboutAction_triggered();
    void onResetCamera();
    void onFpsSpinValueChanged(int num);
};

#endif // QAVIMATOR_H
