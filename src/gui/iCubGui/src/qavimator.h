/*
 * qavimator.h
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

    /*
    void partClicked(BVHNode* node);
    void partDragged(BVHNode* node,double changeX,double changeY,double changeZ);
    void propClicked(Prop* prop);
    void propDragged(Prop* prop,double x,double y,double z);
    void propScaled(Prop* prop,double x,double y,double z);
    void propRotated(Prop* prop,double x,double y,double z);
    */
    
    void backgroundClicked();

    /*
    void frameTimeout();
    
    void setCurrentFrame(int frame);

    void selectAnimation(Animation* animation);
    void clearProps();
    */
    
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
    //void on_selectAnimationCombo_activated(int);
    void on_figureCombo_activated(int);
    void on_scaleSpin_valueChanged(int newValue);
    //void on_editPartCombo_activated(int);
    void on_xRotationEdit_returnPressed();
    void on_xRotationEdit_lostFocus();
    void on_xRotationSlider_valueChanged(int);
    void on_yRotationEdit_returnPressed();
    void on_yRotationEdit_lostFocus();
    void on_yRotationSlider_valueChanged(int);
    void on_zRotationEdit_returnPressed();
    void on_zRotationEdit_lostFocus();
    void on_zRotationSlider_valueChanged(int);
    void on_xPositionEdit_returnPressed();
    void on_xPositionEdit_lostFocus();
    void on_xPositionSlider_valueChanged(int);
    void on_yPositionEdit_returnPressed();
    void on_yPositionEdit_lostFocus();
    void on_yPositionSlider_valueChanged(int);
    void on_zPositionEdit_returnPressed();
    void on_zPositionEdit_lostFocus();
    void on_zPositionSlider_valueChanged(int);

    //void on_currentFrameSlider_valueChanged(int newValue);
    void on_playButton_clicked();
    //void on_keyframeButton_toggled(bool on);
    //void on_loopInSpinBox_valueChanged(int newValue);
    //void on_loopOutSpinBox_valueChanged(int newValue);
    //void on_framesSpin_valueChanged(int num);
    void on_fpsSpin_valueChanged(int num);
    // end autoconnection from designer UI

  protected:
    // prevent closing of main window if there are unsaved changes
    virtual void closeEvent(QCloseEvent* event);

    void fileNew();
    void fileOpen();
    void fileOpen(const QString& fileName);
    // "add" a new file without clearing the old one(s)
    //void fileAdd();
    //void fileAdd(const QString& fileName);
    void fileSave();
    void fileSaveAs();
    //void fileExportForSecondLife();
    //void fileLoadProps();
    //void fileSaveProps();
    void fileExit();

    //void editCut();
    //void editCopy();
    //void editPaste();
    //void editOptimizeBVH();

    //void showSkeleton(bool on);
    void setJointLimits(bool on);
    //void setLoop(bool on);
    //void setProtectFirstFrame(bool on);
    void showTimeline(bool state);
    void configure();

    void helpAbout();

    //void animationChanged(int which);
    //void setAvatarShape(int shape);
    void setAvatarScale(int percent);
    void partChoice();
    void rotationValue();
    void rotationSlider();
    void positionValue();
    void positionSlider();

    //void frameSlider(int position);
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
    //void updateKeyBtn();
    void updateInputs();
    //void updatePropSpins(const Prop* prop);

    // calculates the longest running time of all loaded animations, returns it
    // and stores it in longestRunningTime member variable
    //double calculateLongestRunningTime();

    void setX(float x);
    void setY(float y);
    void setZ(float z);

    float getX();
    float getY();
    float getZ();

    void setXPos(float x);
    void setYPos(float y);
    void setZPos(float z);

    float getXPos();
    float getYPos();
    float getZPos();

    QString currentFile;
    QStringList openFiles;
    // last path used for open or save
    QString lastPath;
    //QTimer timer;
    // list of animation ids mapped to combo box indexes
    //QList<Animation*> animationIds;

    // mapping of combo box indexes to node ids
    QList<int> nodeMapping;

    //BVHNode* currentPart;

    // icons for play button
    QIconSet playIcon;
    //QIcon loopIcon;
    QIconSet stopIcon;

    // holds the current playing status
    PlayState playstate;

    //bool loop;
    bool jointLimits;
    //bool frameDataValid;
    // if set the first frame of an animation is protected
    //bool protectFirstFrame;
    // will be true if a frame is protected
    //bool protect;

    // holds the longest running time of all currently opened animations
    //double longestRunningTime;
};

#endif
