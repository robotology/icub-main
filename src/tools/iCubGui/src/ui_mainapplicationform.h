/*
 * ui_mainapplicationform.h
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

#ifndef UI_MAINAPPLICATIONFORM_H
#define UI_MAINAPPLICATIONFORM_H

#include <qaction.h>
#include <qlayout.h>
#include <qtabwidget.h>
#include <qcombobox.h>
#include <qspinbox.h>
#include <qgroupbox.h>
#include <qpopupmenu.h>
#include <qmainwindow.h>
#include <qiconset.h>
#include <qlabel.h>
#include <qlineedit.h>
#include <qpushbutton.h>
#include <qmenubar.h>

#include "animationview.h"

//QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *fileNewAction;
    QAction *fileOpenAction;
    QAction *fileAddAction;
    QAction *fileSaveAction;
    QAction *fileSaveAsAction;
    QAction *fileExitAction;

    QAction *optionsJointLimitsAction;
    QAction *optionsShowTimelineAction;
    QAction *optionsConfigureiCubGUIAction;
    
    QAction *helpAboutAction;

    QAction *resetCameraAction;
    
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QGridLayout *gridLayout1;
    AnimationView *animationView;
    
    //QTabWidget *avatarPropsTab;
    /*
    QWidget *avatarTab;
    QGridLayout *gridLayout2;
    QLabel *figureLabel;
    QComboBox *figureCombo;
    QLabel *scaleLabel;
    QSpinBox *scaleSpin;
    QLabel *editPartLabel;
    QComboBox *editPartCombo;
    QGroupBox *rotationGroupBox;
    QGridLayout *gridLayout3;
    QLabel *xRotationLabel;
    QLineEdit *xRotationEdit;
    QLabel *shiftLabel;
    QSlider *xRotationSlider;
    QLabel *yRotationLabel;
    QLineEdit *yRotationEdit;
    QLabel *altLabel;
    QSlider *yRotationSlider;
    QLabel *zRotationLabel;
    QLineEdit *zRotationEdit;
    QLabel *ctrlLabel;
    QSlider *zRotationSlider;
    QGroupBox *positionGroupBox;
    QGridLayout *gridLayout4;
    QLabel *xPositionLabel;
    QLineEdit *xPositionEdit;
    QSlider *xPositionSlider;
    QLabel *ypositionLabel;
    QLineEdit *yPositionEdit;
    QSlider *yPositionSlider;
    QLabel *zPositionLabel;
    QLineEdit *zPositionEdit;
    QSlider *zPositionSlider;
    QSpacerItem *spacerItem;
    QGridLayout *gridLayout5;
    QHBoxLayout *hboxLayout;
    QSpacerItem *spacerItem1;
    */
    QPushButton *playButton;
    QLabel *fpsLabel;
    QSpinBox *fpsSpin;
    
    QPopupMenu *menuFile;
    QPopupMenu *menuOptions;
    QPopupMenu *menuHelp;
    QToolBar *toolBar;

	#define setShortcut(s) setAccel(QKeySequence(QString(s)))

	void setupUi(QMainWindow *MainWindow,yarp::os::ResourceFinder& config)
    {
        MainWindow->setName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(703, 818);
        
        // default size
        int width=850;
        int height=600;

        if (config.check("width")) width=config.find("width").asInt();
        if (config.check("height")) height=config.find("height").asInt();
        
        //sanity check
        if(width<50) width=50;
        if(height<50) height=50;

        MainWindow->resize(width, height);

        int xpos=32,ypos=32;
        if (config.check("xpos")) xpos=config.find("xpos").asInt();
        if (config.check("ypos")) ypos=config.find("ypos").asInt();
        MainWindow->move(xpos,ypos);

        fileNewAction = new QAction(MainWindow);
        fileNewAction->setName(QString::fromUtf8("fileNewAction"));
        QIconSet icon;
        icon.setPixmap(QPixmap(config.findPath("icons/filenew.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileNewAction->setIconSet(icon);
        fileNewAction->setText("New");
        fileNewAction->setShortcut("Ctrl+N");
		MainWindow->connect(fileNewAction,SIGNAL(activated()),MainWindow,SLOT(on_fileNewAction_triggered()));

        fileOpenAction = new QAction(MainWindow);
        fileOpenAction->setName(QString::fromUtf8("fileOpenAction"));
        QIconSet icon1;
        icon1.setPixmap(QPixmap(config.findPath("icons/fileopen.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileOpenAction->setIconSet(icon1);
        fileOpenAction->setText("Open ...");
        fileOpenAction->setShortcut("Ctrl+O");
		MainWindow->connect(fileOpenAction,SIGNAL(activated()),MainWindow,SLOT(on_fileOpenAction_triggered()));
        
		fileSaveAction = new QAction(MainWindow);
        fileSaveAction->setName(QString::fromUtf8("fileSaveAction"));
        QIconSet icon3;
        icon3.setPixmap(QPixmap(config.findPath("icons/filesave.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileSaveAction->setIconSet(icon3);
		fileSaveAction->setText("Save");
        fileSaveAction->setShortcut("Ctrl+S");
        MainWindow->connect(fileSaveAction,SIGNAL(activated()),MainWindow,SLOT(on_fileSaveAction_triggered()));

		fileSaveAsAction = new QAction(MainWindow);
        fileSaveAsAction->setName(QString::fromUtf8("fileSaveAsAction"));
        QIconSet icon4;
        icon4.setPixmap(QPixmap(config.findPath("icons/filesaveas.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileSaveAsAction->setIconSet(icon4);
		fileSaveAsAction->setText("Save As...");
        fileSaveAsAction->setShortcut("Ctrl+A");
        MainWindow->connect(fileSaveAsAction,SIGNAL(activated()),MainWindow,SLOT(on_fileSaveAsAction_triggered()));

		fileExitAction = new QAction(MainWindow);
        fileExitAction->setName(QString::fromUtf8("fileExitAction"));        
        fileExitAction->setText("Exit");
        fileExitAction->setShortcut("Ctrl+Q");
		MainWindow->connect(fileExitAction,SIGNAL(activated()),MainWindow,SLOT(on_fileExitAction_triggered()));

        optionsJointLimitsAction = new QAction(MainWindow);
        optionsJointLimitsAction->setName(QString::fromUtf8("optionsJointLimitsAction"));
        optionsJointLimitsAction->setToggleAction(true);
		optionsJointLimitsAction->setText("Joint Limits");
		MainWindow->connect(optionsJointLimitsAction,SIGNAL(toggled(bool)),MainWindow,SLOT(on_optionsJointLimitsAction_toggled(bool)));

        optionsShowTimelineAction = new QAction(MainWindow);
        optionsShowTimelineAction->setName(QString::fromUtf8("optionsShowTimelineAction"));
        optionsShowTimelineAction->setToggleAction(true);
		optionsShowTimelineAction->setText("Show Timeline");
		MainWindow->connect(optionsShowTimelineAction,SIGNAL(toggled(bool)),MainWindow,SLOT(on_optionsShowTimelineAction_toggled(bool)));

        optionsConfigureiCubGUIAction = new QAction(MainWindow);
        optionsConfigureiCubGUIAction->setName(QString::fromUtf8("optionsConfigureiCubGUIAction"));
        optionsConfigureiCubGUIAction->setText("Configure iCubGUI...");
		MainWindow->connect(optionsConfigureiCubGUIAction,SIGNAL(activated()),MainWindow,SLOT(on_optionsConfigureiCubGUIAction_triggered()));

		helpAboutAction = new QAction(MainWindow);
        helpAboutAction->setName(QString::fromUtf8("helpAboutAction"));
        helpAboutAction->setText("About...");
		MainWindow->connect(helpAboutAction,SIGNAL(activated()),MainWindow,SLOT(on_helpAboutAction_triggered()));

        resetCameraAction = new QAction(MainWindow);
        resetCameraAction->setName(QString::fromUtf8("resetCameraAction"));
        QIconSet icon10;
		icon10.setPixmap(QPixmap(config.findPath("icons/resetcamera.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        resetCameraAction->setIconSet(icon10);
		resetCameraAction->setText("Reset Camera");
        resetCameraAction->setShortcut("Ctrl+0");
		MainWindow->connect(resetCameraAction,SIGNAL(activated()),MainWindow,SLOT(on_resetCameraAction_triggered()));

        centralwidget = new QWidget(MainWindow);
        centralwidget->setName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setName(QString::fromUtf8("gridLayout"));
        gridLayout1 = new QGridLayout();
        gridLayout1->setName(QString::fromUtf8("gridLayout1"));
        animationView = new AnimationView(centralwidget,config);
        animationView->setName(QString::fromUtf8("animationView"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorStretch(0);
        sizePolicy.setVerStretch(0);
        sizePolicy.setHeightForWidth(animationView->sizePolicy().hasHeightForWidth());
        animationView->setSizePolicy(sizePolicy);
		gridLayout1->addMultiCellWidget(animationView, 0, 0, 0, 1);
		gridLayout->addMultiCellLayout(gridLayout1, 0, 1, 0, 0);

        MainWindow->setCentralWidget(centralwidget);
        MainWindow->menuBar()->setName(QString::fromUtf8("menubar"));
        MainWindow->menuBar()->setGeometry(QRect(0, 0, 703, 25));
		
        menuFile = new QPopupMenu(MainWindow);
		menuFile->setName(QString::fromUtf8("menuFile"));
		menuFile->setCaption("File");
		MainWindow->menuBar()->insertItem("File",menuFile);
		fileNewAction->addTo(menuFile);
		fileOpenAction->addTo(menuFile);
		fileSaveAction->addTo(menuFile);
		fileSaveAsAction->addTo(menuFile);
		fileExitAction->addTo(menuFile);
		
        menuOptions = new QPopupMenu(MainWindow);
        menuOptions->setName(QString::fromUtf8("menuOptions"));
		menuOptions->setCaption("Options");
        MainWindow->menuBar()->insertItem("Options",menuOptions);
		optionsJointLimitsAction->addTo(menuOptions);
		optionsShowTimelineAction->addTo(menuOptions);
		optionsConfigureiCubGUIAction->addTo(menuOptions);
		
		menuHelp = new QPopupMenu(MainWindow);
        menuHelp->setName(QString::fromUtf8("menuHelp"));
		menuHelp->setCaption("Help");
		MainWindow->menuBar()->insertItem("Help",menuHelp);
		helpAboutAction->addTo(menuHelp);
	
		toolBar = new QToolBar(MainWindow);
        toolBar->setName(QString::fromUtf8("toolBar"));
		toolBar->setGeometry(QRect(0,0,703,64));
		toolBar->setFixedHeight(64);
		MainWindow->addToolBar(toolBar);
		fileNewAction->addTo(toolBar);
		fileOpenAction->addTo(toolBar);
		toolBar->addSeparator();
		fileSaveAction->addTo(toolBar);
		fileSaveAsAction->addTo(toolBar);
		toolBar->addSeparator();
		resetCameraAction->addTo(toolBar);
		toolBar->setCaption("toolBar");
        #ifndef QT_NO_TOOLTIP
        toolBar->setLabel("Reset camera view to default position");
		#endif // QT_NO_TOOLTIP
        toolBar->addSeparator();

        QSizePolicy sizePolicy2(QSizePolicy::Fixed,QSizePolicy::Expanding);
        sizePolicy2.setHorStretch(0);
        sizePolicy2.setVerStretch(0);
        fpsLabel = new QLabel(toolBar);
		fpsLabel->setText("FPS:");
        fpsLabel->setName(QString::fromUtf8("fpsLabel"));
        sizePolicy2.setHeightForWidth(fpsLabel->sizePolicy().hasHeightForWidth());
        fpsLabel->setSizePolicy(sizePolicy2);

        QSizePolicy sizePolicy4(QSizePolicy::Fixed,QSizePolicy::Fixed);
        sizePolicy4.setHorStretch(0);
        sizePolicy4.setVerStretch(0);
        fpsSpin = new QSpinBox(toolBar);
        fpsSpin->setName(QString::fromUtf8("fpsSpin"));
        sizePolicy4.setHeightForWidth(fpsSpin->sizePolicy().hasHeightForWidth());
        fpsSpin->setSizePolicy(sizePolicy4);
        fpsSpin->setRange(1,50);
		fpsSpin->setValue(10);
		MainWindow->connect(fpsSpin,SIGNAL(valueChanged(int)),MainWindow,SLOT(on_fpsSpin_valueChanged(int)));

        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy1.setHorStretch(0);
        sizePolicy1.setVerStretch(0);
        playButton = new QPushButton(toolBar);
        playButton->setName(QString::fromUtf8("playButton"));
		playButton->setText(QString());
        QIconSet icon11;
        icon11.setPixmap(QPixmap(config.findPath("icons/play.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        playButton->setIconSet(icon11);
		MainWindow->connect(playButton,SIGNAL(clicked()),MainWindow,SLOT(on_playButton_clicked()));
    } // setupUi
};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

//QT_END_NAMESPACE

#endif // UI_MAINAPPLICATIONFORM_H

