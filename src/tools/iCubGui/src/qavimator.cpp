/*
 * qavimator.cpp
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

#include "qavimator.h"
#include "ui_qavimator.h"

#include <QEvent>
#include <QMessageBox>
#include <QMetaObject>
#include <QSettings>
#include <QCloseEvent>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>


#include "animationview.h"
#include "settings.h"
#include "settingsdialog.h"

#define ANIM_FILTER "Animation Files (*.avm *.bvh)"
#define PROP_FILTER "Props (*.prp)"
#define PRECISION   100

#define SVN_ID      "$Id: qavimator.cpp,v 1.1 2014/12/19 10:33:13 dperrone Exp $"


qavimator::qavimator(yarp::os::ResourceFinder &config, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::qavimator)
{
    ui->setupUi(this);
    setupToolBar();
    ui->animationView->init(config);
    nFPS=10;

    width=850;
    height=600;


    readSettings();
    // default size

    if (config.check("name")){
        GUI_NAME=std::string(config.find("name").asString().c_str());
    }
    if (GUI_NAME[0]!='/'){
        GUI_NAME=std::string("/")+GUI_NAME;
    }
    if (config.check("width")){
        width=config.find("width").asInt32();
    }
    if (config.check("height")){
        height=config.find("height").asInt32();
    }

    //sanity check
    if(width<100){
        width=100;
    }
    if(height<100){
        height=100;
    }

    this->resize(width, height);

    int xpos=32,ypos=32;
    if (config.check("xpos")){
        xpos=config.find("xpos").asInt32();
    }
    if (config.check("ypos")){
        ypos=config.find("ypos").asInt32();
    }
    this->move(xpos,ypos);

    setWindowTitle(GUI_NAME.c_str());

    connect(ui->animationView,SIGNAL(backgroundClicked()),this,SLOT(backgroundClicked()));
    connect(this,SIGNAL(resetCamera()),ui->animationView,SLOT(resetCamera()));

    ui->animationView->startTimer(1000/nFPS);


}

qavimator::~qavimator()
{
    //if (animationView) delete animationView;
    //animationView=0;
    fileExit();

    delete ui;


}

void qavimator::setupToolBar()
{
    QToolBar *toolBar = ui->toolBar;
    QLabel *label = new QLabel("FPS:",this);
    QSpinBox *fpsSpin = new QSpinBox(this);
    fpsSpin->setMaximum(1);
    fpsSpin->setMaximum(50);
    fpsSpin->setValue(10);
    QIcon ico = QIcon(":/icons/resetcamera.png");
    QPushButton *resetCamera = new QPushButton(ico,"",this);
    resetCamera->setToolTip("Reset camera view to default position");
    toolBar->addWidget(label);
    toolBar->addWidget(fpsSpin);
    toolBar->addSeparator();
    toolBar->addWidget(resetCamera);
    connect(fpsSpin,SIGNAL(valueChanged(int)),this,SLOT(onFpsSpinValueChanged(int)));
    connect(resetCamera,SIGNAL(clicked()),this,SLOT(onResetCamera()));


}

void qavimator::readSettings()
{
    QSettings settings("iCub","iCubGui");
    settings.beginGroup("/qavimator");


    // OpenGL presets
    Settings::setFog(true);
    Settings::setFloorTranslucency(33);

    // defaults for ease in/ease out
    Settings::setEaseIn(false);
    Settings::setEaseOut(false);

    int figureType=0;

    bool settingsFound=settings.value("/settings").toBool();
    if(settingsFound){
        width=settings.value("/mainwindow_width").toInt();
        height=settings.value("/mainwindow_height").toInt();

        // OpenGL settings
        Settings::setFog(settings.value("/fog").toBool());
        Settings::setFloorTranslucency(settings.value("/floor_translucency").toInt());

        figureType=settings.value("/figure").toInt();

        settings.endGroup();
    }


}


// slot gets called by AnimationView::mouseButtonClicked()
void qavimator::backgroundClicked()
{
    emit enableRotation(false);
}






// Menu Action: File / Exit
void qavimator::fileExit()
{
    QSettings settings("iCub","iCubGui");
    settings.beginGroup("/qavimator");

    // make sure we know next time, that there actually was a settings file
    settings.setValue("/settings",true);

    //settings.writeEntry("/figure",figureCombo->currentItem());
    settings.setValue("/mainwindow_width",size().width());
    settings.setValue("/mainwindow_height",size().height());


    // OpenGL settings
    settings.setValue("/fog",Settings::fog());
    settings.setValue("/floor_translucency",Settings::floorTranslucency());

    // settings for ease in/ease outFrame
    settings.setValue("/ease_in",Settings::easeIn());
    settings.setValue("/ease_out",Settings::easeOut());

    settings.endGroup();


    ui->animationView->stopTimer();

    close();

}


// Menu Action: Options / Configure iCubGUI
void qavimator::configure()
{
    SettingsDialog dialog;
    connect(&dialog,SIGNAL(configChanged()),this,SLOT(configChanged()));

    dialog.exec();
}

void qavimator::configChanged()
{
    ui->animationView->repaint();
}

// Menu Action: Help / About ...
void qavimator::helpAbout()
{
    QMessageBox::about(this,QObject::tr("About iCubGui"),QObject::tr("iCubGui - joint gui for iCub<br />%1").arg(SVN_ID));
}


// prevent closing of main window if there are unsaved changes
void qavimator::closeEvent(QCloseEvent* event)
{
    ui->animationView->stopTimer();
    //animationView=0;
    event->accept();
}

void qavimator::on_fileExitAction_triggered()
{
    //qDebug("qavimator::fileExit() not implemented");
    fileExit();
}

void qavimator::on_optionsConfigureiCubGUIAction_triggered()
{
    configure();
}

void qavimator::on_helpAboutAction_triggered()
{
    helpAbout();
}


void qavimator::onResetCamera()
{
    emit resetCamera();
}



void qavimator::onFpsSpinValueChanged(int newValue)
{
    setFPS(newValue);
}
void qavimator::setFPS(int fps)
{
    qDebug("qavimator::setFPS(%d)",fps);
    if (fps<1) fps=1; else if (fps>50) fps=50;
    nFPS=fps;

    ui->animationView->stopTimer();
    ui->animationView->startTimer(1000/nFPS);
}
