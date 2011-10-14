/*
* qavimator.cpp   
*/

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Based on:
 *
 *   Qavimator
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *   Released under the terms of the GNU GPL v2.0.
 */

#include <qevent.h>
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <qmetaobject.h>
#include <qsettings.h>

#include "qavimator.h"
#include "animationview.h"
#include "settings.h"
#include "settingsdialog.h"

#define ANIM_FILTER "Animation Files (*.avm *.bvh)"
#define PROP_FILTER "Props (*.prp)"
#define PRECISION   100

#define SVN_ID      "$Id: qavimator.cpp,v 1.3 2009/07/24 19:17:53 ale-scalzo Exp $"

qavimator::qavimator(yarp::os::ResourceFinder& config) : QMainWindow(0)
{
    nFPS=10;

    setupUi(this,config);

    setCaption("iCubGui");

    readSettings();

    connect(animationView,SIGNAL(backgroundClicked()),this,SLOT(backgroundClicked()));
    connect(this,SIGNAL(resetCamera()),animationView,SLOT(resetCamera()));

    if(qApp->argc()>1)
    {
        fileOpen(qApp->argv()[1]);
    }

    // if opening of files didn't work or no files were specified on the
    // command line, open a new one
    if(openFiles.count()==0) fileNew();

    // prepare play button icons
    stopIcon.setPixmap(config.findPath("icons/stop.png").c_str(), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
    playIcon.setPixmap(config.findPath("icons/play.png").c_str(), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
    // playback stopped by default
    setPlaystate(PLAYSTATE_STOPPED);
    nextPlaystate();
}

qavimator::~qavimator()
{
    //if (animationView) delete animationView;
    //animationView=0;
    fileExit();
}

// FIXME:: implement a static Settings:: class
void qavimator::readSettings()
{
    QSettings settings;
    settings.beginGroup("/qavimator");

    // if no settings found, start up with defaults
    int width=850;
    int height=600;

    jointLimits=true;
    lastPath=QString::null;

    // OpenGL presets
    Settings::setFog(true);
    Settings::setFloorTranslucency(33);

    // defaults for ease in/ease out
    Settings::setEaseIn(false);
    Settings::setEaseOut(false);

    int figureType=0;
    int showTimelinePanel=false;

    bool settingsFound=settings.readBoolEntry("/settings");
    if(settingsFound)
    {
        jointLimits=settings.readBoolEntry("/joint_limits");
        showTimelinePanel=settings.readBoolEntry("/show_timeline");

        int width=settings.readNumEntry("/mainwindow_width");
        int height=settings.readNumEntry("/mainwindow_height");

        lastPath=settings.readEntry("/last_path");

        // OpenGL settings
        Settings::setFog(settings.readBoolEntry("/fog"));
        Settings::setFloorTranslucency(settings.readNumEntry("/floor_translucency"));

        // sanity
        if(width<50) width=50;
        if(height<50) height=50;

        figureType=settings.readNumEntry("/figure");

        settings.endGroup();
    }

    resize(width,height);

    optionsJointLimitsAction->setOn(jointLimits);
    optionsShowTimelineAction->setOn(showTimelinePanel);
}


// slot gets called by AnimationView::mouseButtonClicked()
void qavimator::backgroundClicked()
{
    emit enableRotation(false);
}

void qavimator::nextPlaystate()
{
    switch(playstate)
    {
    case PLAYSTATE_STOPPED:
        {
            // start looping animation
            setPlaystate(PLAYSTATE_PLAYING);
            animationView->startTimer(1000/nFPS);
            break;
        }
    case PLAYSTATE_PLAYING:
        {
            setPlaystate(PLAYSTATE_STOPPED);     
            animationView->stopTimer();
            break;
        }
    default:
        qDebug("qavimator::nextPlaystate(): unknown playstate %d",(int) playstate);
    }
}

// ------ Menu Action Slots (Callbacks) -----------

// Menu action: File / New
void qavimator::fileNew()
{
}

QString qavimator::selectFileToOpen(const QString& caption)
{
    return QString::null;
}

// Menu action: File / Open ...
void qavimator::fileOpen()
{
}

void qavimator::fileOpen(const QString& name)
{
}

// Menu Action: File / Save
void qavimator::fileSave()
{
}

// Menu Action: File / Save As...
void qavimator::fileSaveAs()
{
}


// Menu Action: File / Exit
void qavimator::fileExit()
{
    QSettings settings;
    settings.beginGroup("/qavimator");

    // make sure we know next time, that there actually was a settings file
    settings.writeEntry("/settings",true);

    settings.writeEntry("/joint_limits",optionsJointLimitsAction->isOn());
    settings.writeEntry("/show_timeline",optionsShowTimelineAction->isOn());

    //settings.writeEntry("/figure",figureCombo->currentItem());
    settings.writeEntry("/mainwindow_width",size().width());
    settings.writeEntry("/mainwindow_height",size().height());

    settings.writeEntry("/last_path",lastPath);

    // OpenGL settings
    settings.writeEntry("/fog",Settings::fog());
    settings.writeEntry("/floor_translucency",Settings::floorTranslucency());

    // settings for ease in/ease outFrame
    settings.writeEntry("/ease_in",Settings::easeIn());
    settings.writeEntry("/ease_out",Settings::easeOut());

    settings.endGroup();

    setPlaystate(PLAYSTATE_STOPPED);

    if (animationView)
    {
        animationView->stopTimer();
        delete animationView;
    }
    animationView=0;

    // remove all widgets and close the main form
    qApp->exit(0);
}

// Menu Action: Options / Joint Limits
void qavimator::setJointLimits(bool on)
{
    jointLimits=on;
}

// Menu Action: Options / Configure iCubGUI
void qavimator::configure()
{
    SettingsDialog* dialog=new SettingsDialog(this);
    connect(dialog,SIGNAL(configChanged()),this,SLOT(configChanged()));

    dialog->exec();

    delete dialog;
}

void qavimator::configChanged()
{
    animationView->repaint();
}

// Menu Action: Help / About ...
void qavimator::helpAbout()
{
    QMessageBox::about(this,QObject::tr("About iCubGui"),QObject::tr("iCubGui - joint gui for iCub<br />%1").arg(SVN_ID));
}

// checks if a file already exists at the given path and displays a warning message
// returns true if it's ok to save/overwrite, else returns false
bool qavimator::checkFileOverwrite(const QFileInfo& fileInfo)
{
    // get file info
    if(fileInfo.exists())
    {
        int answer=QMessageBox::question(this,tr("File Exists"),tr("A file with the name \"%1\" does already exist. Do you want to overwrite it?").arg(fileInfo.fileName()),QMessageBox::Yes,QMessageBox::No,QMessageBox::NoButton);
        if(answer==QMessageBox::No) return false;
    }
    return true;
}

// helper function to prevent feedback between the two widgets
void qavimator::setSliderValue(QSlider* slider,QLineEdit* edit,float value)
{
    slider->blockSignals(true);
    edit->blockSignals(true);
    slider->setValue((int)(value*PRECISION));
    edit->setText(QString::number(value));
    edit->blockSignals(false);
    slider->blockSignals(false);
}

// convenience function to set window title in a defined way
void qavimator::setCurrentFile(const QString& fileName)
{
    currentFile=fileName;
    setCaption("iCubGui");
}

void qavimator::setPlaystate(PlayState state)
{
    playstate=state;

    // set play button icons according to play state
    if(state==PLAYSTATE_STOPPED)
    {
        playButton->setIconSet(playIcon);
        qDebug("qavimator::setPlaystate(): STOPPED");
    }
    else if(state==PLAYSTATE_PLAYING)
    {
        playButton->setIconSet(stopIcon);
        qDebug("qavimator::setPlaystate(): PLAYING");
    }
    else
        qDebug("qavimator::setPlaystate(): unknown playstate %d",(int) state);
}

// prevent closing of main window if there are unsaved changes
void qavimator::closeEvent(QCloseEvent* event)
{
    /*
    if(!clearOpenFiles())
    event->ignore();
    else
    */

    setPlaystate(PLAYSTATE_STOPPED);

    if (animationView)
    {
        animationView->stopTimer();

        delete animationView;
    }
    animationView=0;
    event->accept();
}

// -------------------------------------------------------------------------
// autoconnection from designer UI

// ------- Menu Action Slots --------

void qavimator::on_fileNewAction_triggered()
{
    qDebug("qavimator::fileNew() not implemented");
    //fileNew();
}

void qavimator::on_fileOpenAction_triggered()
{
    qDebug("qavimator::fileOpen() not implemented");
    //fileOpen();
}

void qavimator::on_fileSaveAction_triggered()
{
    qDebug("qavimator::fileSave() not implemented");
    //fileSave();
}

void qavimator::on_fileSaveAsAction_triggered()
{
    qDebug("qavimator::fileSaveAs() not implemented");
    //fileSaveAs();
}

void qavimator::on_fileExitAction_triggered()
{
    //qDebug("qavimator::fileExit() not implemented");
    fileExit();
}

void qavimator::on_optionsJointLimitsAction_toggled(bool on)
{
    qDebug("qavimator::setJointLimits() not implemented");
    setJointLimits(on);
}

void qavimator::on_optionsShowTimelineAction_toggled(bool on)
{
    qDebug("qavimator::showTimeline() not implemented");
}

void qavimator::on_optionsConfigureiCubGUIAction_triggered()
{
    configure();
}

void qavimator::on_helpAboutAction_triggered()
{
    helpAbout();
}

// ------- Additional Toolbar Element Slots --------

void qavimator::on_resetCameraAction_triggered()
{
    emit resetCamera();
}

// ------- UI Element Slots --------

void qavimator::on_playButton_clicked()
{
    nextPlaystate();
}


void qavimator::on_fpsSpin_valueChanged(int newValue)
{
    setFPS(newValue);
}

// end autoconnection from designer UI
// -------------------------------------------------------------------------
