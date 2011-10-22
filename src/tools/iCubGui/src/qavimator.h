/*
 * qavimator.h
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

#ifndef QAVIMATOR_H
#define QAVIMATOR_H

#define UNTITLED_NAME "Untitled.avm"

#include "ui_mainapplicationform.h"
#include "playstate.h"

#include <qfileinfo.h>
#include <qiconset.h>

class qavimator : public QMainWindow, Ui::MainWindow
{
  Q_OBJECT

  public:
    qavimator(yarp::os::ResourceFinder& config);
    ~qavimator();

  signals:
    void enableRotation(bool state);
    void enablePosition(bool state);
    void enableProps(bool state);
    void enableEaseInOut(bool state);
    void resetCamera();
    void protectFrame(bool state);

  protected slots:
    void readSettings();
    void configChanged();    
    void backgroundClicked();
    
    // autoconnection from designer UI

    // ------- Menu Action Slots --------
    void on_fileNewAction_triggered();
    void on_fileOpenAction_triggered();
    void on_fileSaveAction_triggered();
    void on_fileSaveAsAction_triggered();
    void on_fileExitAction_triggered();

    void on_optionsJointLimitsAction_toggled(bool on);
    void on_optionsShowTimelineAction_toggled(bool on);
    void on_optionsConfigureiCubGUIAction_triggered();

    void on_helpAboutAction_triggered();

    // ------- Additional Toolbar Element Slots -------

    void on_resetCameraAction_triggered();

    // ------- UI Element Slots --------
    void on_playButton_clicked();
    void on_fpsSpin_valueChanged(int num);
    // end autoconnection from designer UI

  protected:
    // prevent closing of main window if there are unsaved changes
    virtual void closeEvent(QCloseEvent* event);

    void fileNew();
    void fileOpen();
    void fileOpen(const QString& fileName);
    void fileSave();
    void fileSaveAs();
  public:
    void fileExit();
  protected:
    void setJointLimits(bool on);
    void showTimeline(bool state);
    void configure();

    void helpAbout();
    void nextPlaystate();
    
	int nFPS;
    void setFPS(int fps)
	{
		qDebug("qavimator::setFPS(%d)",fps);
		if (fps<1) fps=1; else if (fps>50) fps=50;
		nFPS=fps;
		if (playstate==PLAYSTATE_PLAYING)
		{
			animationView->stopTimer();
			animationView->startTimer(1000/nFPS);
		}
	}

    void setSliderValue(QSlider* slider,QLineEdit* edit,float value);

    QString selectFileToOpen(const QString& caption);
    void addToOpenFiles(const QString& fileName);
    void removeFromOpenFiles(unsigned int which);
    bool clearOpenFiles();

    void setPlaystate(PlayState state);

	bool checkFileOverwrite(const QFileInfo& fileInfo);
    void setCurrentFile(const QString& fileName);
    void enableInputs(bool state);

    void updateFps();
    void updateInputs();

    QString currentFile;
    QStringList openFiles;
    // last path used for open or save
    QString lastPath;

    // mapping of combo box indexes to node ids
    QList<int> nodeMapping;

    // icons for play button
    QIconSet playIcon;
    QIconSet stopIcon;

    // holds the current playing status
    PlayState playstate;

    bool jointLimits;
};

#endif


