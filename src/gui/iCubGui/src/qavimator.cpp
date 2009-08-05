/*
 * qavimator.cpp   
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

qavimator::qavimator() : QMainWindow(0)
{
	nFPS=10;

  setupUi(this);

  setCaption("iCubGui");
  //setAttribute(Qt::WA_DeleteOnClose);
  
  readSettings();

  connect(animationView,SIGNAL(backgroundClicked()),this,SLOT(backgroundClicked()));
  connect(this,SIGNAL(enablePosition(bool)),positionGroupBox,SLOT(setEnabled(bool)));
  connect(this,SIGNAL(enableRotation(bool)),rotationGroupBox,SLOT(setEnabled(bool)));
  connect(this,SIGNAL(resetCamera()),animationView,SLOT(resetCamera()));
  
  xRotationSlider->setPageStep(10*PRECISION);
  yRotationSlider->setPageStep(10*PRECISION);
  zRotationSlider->setPageStep(10*PRECISION);
  xPositionSlider->setPageStep(10*PRECISION);
  yPositionSlider->setPageStep(10*PRECISION);
  zPositionSlider->setPageStep(10*PRECISION);

  if(qApp->argc()>1)
  {
    fileOpen(qApp->argv()[1]);
  }

  // if opening of files didn't work or no files were specified on the
  // command line, open a new one
  if(openFiles.count()==0) fileNew();

  // prepare play button icons
  QString dataPath(getenv("ICUB_DIR"));
  dataPath+="/src/gui/iCubGui/icons/";
  stopIcon.setPixmap(dataPath+"stop.png", QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
  playIcon.setPixmap(dataPath+"play.png", QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
  // playback stopped by default
  setPlaystate(PLAYSTATE_STOPPED);

  updateInputs();
}

qavimator::~qavimator()
{
	if (animationView) delete animationView;
	animationView=0;
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

  figureCombo->setCurrentItem(figureType);
  //setAvatarShape(figureType);
}


// slot gets called by AnimationView::mouseButtonClicked()
void qavimator::backgroundClicked()
{
  emit enableRotation(false);
  editPartCombo->setCurrentItem(0);
  //updateKeyBtn();
}

// gets called whenever a body part rotation slider is moved
void qavimator::rotationSlider()
{
  double x=getX();
  double y=getY();
  double z=getZ();

  setX(x);
  setY(y);
  setZ(z);

  /*
  Animation* anim=animationView->getAnimation();
  if(animationView->getSelectedPart())
  {
    anim->setRotation(animationView->getSelectedPart(),x,y,z);
    animationView->repaint();
  }
  */
  //updateKeyBtn();
}

// gets called whenever a body part rotation value field gets changed
void qavimator::rotationValue()
{
  double x=xRotationEdit->text().toDouble();
  double y=yRotationEdit->text().toDouble();
  double z=zRotationEdit->text().toDouble();

  double min_x=xRotationSlider->minValue()/PRECISION;
  double min_y=yRotationSlider->minValue()/PRECISION;
  double min_z=zRotationSlider->minValue()/PRECISION;

  double max_x=xRotationSlider->maxValue()/PRECISION;
  double max_y=yRotationSlider->maxValue()/PRECISION;
  double max_z=zRotationSlider->maxValue()/PRECISION;

  if(x<min_x) x=min_x;
  if(y<min_y) y=min_y;
  if(z<min_z) z=min_z;

  if(x>max_x) x=max_x;
  if(y>max_y) y=max_y;
  if(z>max_z) z=max_z;

  setX(x);
  setY(y);
  setZ(z);

  /*
  Animation* anim=animationView->getAnimation();
  if(animationView->getSelectedPart())
  {
    anim->setRotation(animationView->getSelectedPart(), x, y, z);
    animationView->repaint();
  }
  */
  //updateKeyBtn();
}

void qavimator::positionSlider()
{
  double x=getXPos();
  double y=getYPos();
  double z=getZPos();

  setXPos(x);
  setYPos(y);
  setZPos(z);

  //animationView->getAnimation()->setPosition(x,y,z);
  animationView->repaint();

  //updateKeyBtn();
}

void qavimator::positionValue()
{
  qDebug("qavimator::positionValue()");

  double x=xPositionEdit->text().toDouble();
  double y=yPositionEdit->text().toDouble();
  double z=zPositionEdit->text().toDouble();

  double min_x=xPositionSlider->minValue()/PRECISION;
  double min_y=yPositionSlider->minValue()/PRECISION;
  double min_z=zPositionSlider->minValue()/PRECISION;

  double max_x=xPositionSlider->maxValue()/PRECISION;
  double max_y=yPositionSlider->maxValue()/PRECISION;
  double max_z=zPositionSlider->maxValue()/PRECISION;

  if(x<min_x) x=min_x;
  if(y<min_y) y=min_y;
  if(z<min_z) z=min_z;

  if(x>max_x) x=max_x;
  if(y>max_y) y=max_y;
  if(z>max_z) z= max_z;

  setXPos(x);
  setYPos(y);
  setZPos(z);

  //animationView->getAnimation()->setPosition(x,y,z);
  animationView->repaint();

  //updateKeyBtn();
}

void qavimator::updateInputs()
{
  // deactivate redraw to reduce interface "jitter" during updating
  setUpdatesEnabled(false);
  
  /*
  Animation* anim=animationView->getAnimation();

  if(anim && currentPart)
  {
    currentPart->setSliders(xRotationSlider,yRotationSlider,zRotationSlider,xPositionSlider,yPositionSlider,zPositionSlider);
    
    Rotation rot=anim->getRotation(currentPart);

    double x=rot.x;
    double y=rot.y;
    double z=rot.z;

    RotationLimits rotLimits=anim->getRotationLimits(currentPart);

    double xMin=rotLimits.xMin;
    double yMin=rotLimits.yMin;
    double zMin=rotLimits.zMin;
    double xMax=rotLimits.xMax;
    double yMax=rotLimits.yMax;
    double zMax=rotLimits.zMax;

    if(currentPart->type==BVH_ROOT)
    {
      xRotationSlider->setRange(-359*PRECISION, 359*PRECISION);
      yRotationSlider->setRange(-359*PRECISION, 359*PRECISION);
      zRotationSlider->setRange(-359*PRECISION, 359*PRECISION);
    }
    else
    {
      xRotationSlider->setRange((int)(xMin*PRECISION), (int)(xMax*PRECISION));
      yRotationSlider->setRange((int)(yMin*PRECISION), (int)(yMax*PRECISION));
      zRotationSlider->setRange((int)(zMin*PRECISION), (int)(zMax*PRECISION));
    }

    setX(x);
    setY(y);
    setZ(z);
  }
  else
  */
  //emit enableRotation(false);

  //emit enablePosition(!protect);

// we do that in nextPlaystate() now
//  playButton->setIcon(playing ? stopIcon : playIcon);


  // prevent feedback loop
  scaleSpin->blockSignals(true);
  //scaleSpin->setValue(anim->getAvatarScale()*100.0+0.1);  // +0.1 to overcome rounding errors
  scaleSpin->setValue(100);
  scaleSpin->blockSignals(false);

  //updateKeyBtn();

  if(playstate==PLAYSTATE_STOPPED)
    emit enableInputs(true);
  else
    emit enableInputs(false);

  // reactivate redraw
  setUpdatesEnabled(true);
}

void qavimator::enableInputs(bool state)
{
  /*
  // protection overrides state for keyframe button
  if(protect) state=false;

  // do not enable rotation if we have no part selected
  if(!currentPart) state=false;
  emit enableRotation(state);
  */
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

  updateInputs();
}

void qavimator::setAvatarScale(int percent)
{
  //animationView->getAnimation()->setAvatarScale(percent/100.0);
  animationView->repaint();
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
  /*
  if(!clearOpenFiles())
    return;
  */
  
  QSettings settings;
  settings.beginGroup("/qavimator");

  // make sure we know next time, that there actually was a settings file
  settings.writeEntry("/settings",true);

  settings.writeEntry("/joint_limits",optionsJointLimitsAction->isOn());
  settings.writeEntry("/show_timeline",optionsShowTimelineAction->isOn());

  settings.writeEntry("/figure",figureCombo->currentItem());
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

  if (animationView) delete animationView;
  animationView=0;

  // remove all widgets and close the main form
  qApp->exit(0);
}

// Menu Action: Options / Joint Limits
void qavimator::setJointLimits(bool on)
{
  jointLimits=on;
  /*
  Animation* anim=animationView->getAnimation();
  if(anim)
  {
    anim->useRotationLimits(on);
    animationView->repaint();
    updateInputs();
  }
  */
}


// Menu Action: Oprions / Show Timeline
void qavimator::showTimeline(bool on)
{
  /*
  // hack to get 3D view back in shape
  qApp->processEvents();
  QSize oldSize=animationView->size();
  animationView->resize(oldSize.width(),oldSize.height()-1);
  qApp->processEvents();
  animationView->resize(oldSize);
  */
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

void qavimator::setX(float x)
{
  setSliderValue(xRotationSlider,xRotationEdit,x);
}

void qavimator::setY(float y)
{
  setSliderValue(yRotationSlider,yRotationEdit,y);
}

void qavimator::setZ(float z)
{
  setSliderValue(zRotationSlider,zRotationEdit,z);
}

float qavimator::getX()
{
  return xRotationSlider->value()/PRECISION;
}

float qavimator::getY()
{
  return yRotationSlider->value()/PRECISION;
}

float qavimator::getZ()
{
  return zRotationSlider->value()/PRECISION;
}

void qavimator::setXPos(float x)
{
  setSliderValue(xPositionSlider,xPositionEdit,x);
}

void qavimator::setYPos(float y)
{
  setSliderValue(yPositionSlider,yPositionEdit,y);
}

void qavimator::setZPos(float z)
{
  setSliderValue(zPositionSlider,zPositionEdit,z);
}

float qavimator::getXPos()
{
  return xPositionSlider->value()/PRECISION;
}

float qavimator::getYPos()
{
  return yPositionSlider->value()/PRECISION;
}

float qavimator::getZPos()
{
  return zPositionSlider->value()/PRECISION;
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
	if (animationView) delete animationView;
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
	showTimeline(on);
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


void qavimator::on_figureCombo_activated(int newShape)
{
  //setAvatarShape(newShape);
}

void qavimator::on_scaleSpin_valueChanged(int newValue)
{
  setAvatarScale(newValue);
}

void qavimator::on_xRotationEdit_returnPressed()
{
  rotationValue();
}

void qavimator::on_xRotationEdit_lostFocus()
{
  rotationValue();
}

void qavimator::on_xRotationSlider_valueChanged(int)
{
  rotationSlider();
}

void qavimator::on_yRotationEdit_returnPressed()
{
  rotationValue();
}

void qavimator::on_yRotationEdit_lostFocus()
{
  rotationValue();
}

void qavimator::on_yRotationSlider_valueChanged(int)
{
  rotationSlider();
}

void qavimator::on_zRotationEdit_returnPressed()
{
  rotationValue();
}

void qavimator::on_zRotationEdit_lostFocus()
{
  rotationValue();
}

void qavimator::on_zRotationSlider_valueChanged(int)
{
  rotationSlider();
}

void qavimator::on_xPositionEdit_returnPressed()
{
  positionValue();
}

void qavimator::on_xPositionEdit_lostFocus()
{
  positionValue();
}

void qavimator::on_xPositionSlider_valueChanged(int)
{
  positionSlider();
}

void qavimator::on_yPositionEdit_returnPressed()
{
  positionValue();
}

void qavimator::on_yPositionEdit_lostFocus()
{
  positionValue();
}

void qavimator::on_yPositionSlider_valueChanged(int)
{
  positionSlider();
}

void qavimator::on_zPositionEdit_returnPressed()
{
  positionValue();
}

void qavimator::on_zPositionEdit_lostFocus()
{
  positionValue();
}

void qavimator::on_zPositionSlider_valueChanged(int)
{
  positionSlider();
}


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
