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

#include <qlayout.h>
#include <qtabwidget.h>
#include <qspinbox.h>
#include <qiconset.h>
#include <qlabel.h>
#include <qlineedit.h>
#include <qpushbutton.h>
#include <qmenubar.h>

#ifdef ICUB_USE_QT4_QT3_SUPPORT
#include <q3popupmenu.h>
#include <q3boxlayout.h>
#include <q3toolbar.h>
#include <q3mainwindow.h>
#include <q3action.h>
#include <q3combobox.h>
#include <q3groupbox.h>
#else // ICUB_USE_QT4_QT3_SUPPORT
#include <qpopupmenu.h>
#include <qlayout.h>
#include <qtoolbar.h>
#include <qaction.h>
#include <qcombobox.h>
#include <qgroupbox.h>
#include <qmainwindow.h>
#endif // ICUB_USE_QT4_QT3_SUPPORT

#include "animationview.h"

//QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
#ifdef ICUB_USE_QT4_QT3_SUPPORT
    Q3Action *fileNewAction;
    Q3Action *fileOpenAction;
    Q3Action *fileAddAction;
    Q3Action *fileSaveAction;
    Q3Action *fileSaveAsAction;
    Q3Action *fileExitAction;

    Q3Action *optionsJointLimitsAction;
    Q3Action *optionsShowTimelineAction;
    Q3Action *optionsConfigureiCubGUIAction;

    Q3Action *helpAboutAction;

    Q3Action *resetCameraAction;

    Q3Action *playAction;
    Q3Action *fpsAction;
#else // ICUB_USE_QT4_QT3_SUPPORT
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

    QAction *playAction;
    QAction *fpsAction;
#endif // ICUB_USE_QT4_QT3_SUPPORT

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

    QLabel *fpsLabel;
    QSpinBox *fpsSpin;
    QWidget *fpsWidget;

#ifdef ICUB_USE_QT4_QT3_SUPPORT
    Q3HBoxLayout *fpsLayout;
    Q3PopupMenu *menuFile;
    Q3PopupMenu *menuOptions;
    Q3PopupMenu *menuHelp;
    Q3ToolBar *toolBar;
#else // ICUB_USE_QT4_QT3_SUPPORT
    QHBoxLayout *fpsLayout;
    QPopupMenu *menuFile;
    QPopupMenu *menuOptions;
    QPopupMenu *menuHelp;
    QToolBar *toolBar;
#endif // ICUB_USE_QT4_QT3_SUPPORT

#define setShortcut(s) setAccel(QKeySequence(QString(s)))

#ifdef ICUB_USE_QT4_QT3_SUPPORT
    void setupUi(Q3MainWindow *MainWindow,yarp::os::ResourceFinder& config)
#else // ICUB_USE_QT4_QT3_SUPPORT
    void setupUi(QMainWindow *MainWindow,yarp::os::ResourceFinder& config)
#endif // ICUB_USE_QT4_QT3_SUPPORT
    {
        MainWindow->setName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(703, 818);

        // default size
        int width=850;
        int height=600;

        if (config.check("name")) GUI_NAME=std::string(config.find("name").asString().c_str());
        if (GUI_NAME[0]!='/') GUI_NAME=std::string("/")+GUI_NAME;
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

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        fileNewAction = new Q3Action(MainWindow);
        fileOpenAction = new Q3Action(MainWindow);
        fileSaveAction = new Q3Action(MainWindow);
        fileSaveAsAction = new Q3Action(MainWindow);
        fileExitAction = new Q3Action(MainWindow);
        optionsJointLimitsAction = new Q3Action(MainWindow);
        optionsShowTimelineAction = new Q3Action(MainWindow);
        optionsConfigureiCubGUIAction = new Q3Action(MainWindow);
        helpAboutAction = new Q3Action(MainWindow);
        resetCameraAction = new Q3Action(MainWindow);
        playAction = new Q3Action(MainWindow);
#else // ICUB_USE_QT4_QT3_SUPPORT
        fileNewAction = new QAction(MainWindow);
        fileOpenAction = new QAction(MainWindow);
        fileSaveAction = new QAction(MainWindow);
        fileSaveAsAction = new QAction(MainWindow);
        fileExitAction = new QAction(MainWindow);
        optionsJointLimitsAction = new QAction(MainWindow);
        optionsShowTimelineAction = new QAction(MainWindow);
        optionsConfigureiCubGUIAction = new QAction(MainWindow);
        helpAboutAction = new QAction(MainWindow);
        resetCameraAction = new QAction(MainWindow);
        playAction = new QAction(MainWindow);
#endif // ICUB_USE_QT4_QT3_SUPPORT

        fileNewAction->setName(QString::fromUtf8("fileNewAction"));
        QIconSet icon;
        icon.setPixmap(QPixmap(config.findPath("icons/filenew.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileNewAction->setIconSet(icon);
        fileNewAction->setText("New");
        fileNewAction->setShortcut("Ctrl+N");
        MainWindow->connect(fileNewAction,SIGNAL(activated()),MainWindow,SLOT(on_fileNewAction_triggered()));

        fileOpenAction->setName(QString::fromUtf8("fileOpenAction"));
        QIconSet icon1;
        icon1.setPixmap(QPixmap(config.findPath("icons/fileopen.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileOpenAction->setIconSet(icon1);
        fileOpenAction->setText("Open ...");
        fileOpenAction->setShortcut("Ctrl+O");
        MainWindow->connect(fileOpenAction,SIGNAL(activated()),MainWindow,SLOT(on_fileOpenAction_triggered()));

        fileSaveAction->setName(QString::fromUtf8("fileSaveAction"));
        QIconSet icon3;
        icon3.setPixmap(QPixmap(config.findPath("icons/filesave.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileSaveAction->setIconSet(icon3);
        fileSaveAction->setText("Save");
        fileSaveAction->setShortcut("Ctrl+S");
        MainWindow->connect(fileSaveAction,SIGNAL(activated()),MainWindow,SLOT(on_fileSaveAction_triggered()));

        fileSaveAsAction->setName(QString::fromUtf8("fileSaveAsAction"));
        QIconSet icon4;
        icon4.setPixmap(QPixmap(config.findPath("icons/filesaveas.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        fileSaveAsAction->setIconSet(icon4);
        fileSaveAsAction->setText("Save As...");
        fileSaveAsAction->setShortcut("Ctrl+A");
        MainWindow->connect(fileSaveAsAction,SIGNAL(activated()),MainWindow,SLOT(on_fileSaveAsAction_triggered()));

        fileExitAction->setName(QString::fromUtf8("fileExitAction"));        
        fileExitAction->setText("Exit");
        fileExitAction->setShortcut("Ctrl+Q");
        MainWindow->connect(fileExitAction,SIGNAL(activated()),MainWindow,SLOT(on_fileExitAction_triggered()));

        optionsJointLimitsAction->setName(QString::fromUtf8("optionsJointLimitsAction"));
        optionsJointLimitsAction->setToggleAction(true);
        optionsJointLimitsAction->setText("Joint Limits");
        MainWindow->connect(optionsJointLimitsAction,SIGNAL(toggled(bool)),MainWindow,SLOT(on_optionsJointLimitsAction_toggled(bool)));

        optionsShowTimelineAction->setName(QString::fromUtf8("optionsShowTimelineAction"));
        optionsShowTimelineAction->setToggleAction(true);
        optionsShowTimelineAction->setText("Show Timeline");
        MainWindow->connect(optionsShowTimelineAction,SIGNAL(toggled(bool)),MainWindow,SLOT(on_optionsShowTimelineAction_toggled(bool)));

        optionsConfigureiCubGUIAction->setName(QString::fromUtf8("optionsConfigureiCubGUIAction"));
        optionsConfigureiCubGUIAction->setText("Configure iCubGUI...");
        MainWindow->connect(optionsConfigureiCubGUIAction,SIGNAL(activated()),MainWindow,SLOT(on_optionsConfigureiCubGUIAction_triggered()));

        helpAboutAction->setName(QString::fromUtf8("helpAboutAction"));
        helpAboutAction->setText("About...");
        MainWindow->connect(helpAboutAction,SIGNAL(activated()),MainWindow,SLOT(on_helpAboutAction_triggered()));

        resetCameraAction->setName(QString::fromUtf8("resetCameraAction"));
        QIconSet icon10;
        icon10.setPixmap(QPixmap(config.findPath("icons/resetcamera.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        resetCameraAction->setIconSet(icon10);
        resetCameraAction->setText("Reset Camera");
        resetCameraAction->setShortcut("Ctrl+0");
        MainWindow->connect(resetCameraAction,SIGNAL(activated()),MainWindow,SLOT(on_resetCameraAction_triggered()));

        playAction->setName(QString::fromUtf8("playAction"));
        QIconSet icon11;
        icon11.setPixmap(QPixmap(config.findPath("icons/play.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        playAction->setIconSet(icon11);
        playAction->setText(QString());
        MainWindow->connect(playAction,SIGNAL(triggered()),MainWindow,SLOT(on_playAction_clicked()));

        fpsWidget = new QWidget;

        QSizePolicy sizePolicy2(QSizePolicy::Fixed,QSizePolicy::Expanding);
        sizePolicy2.setHorStretch(0);
        sizePolicy2.setVerStretch(0);
        fpsLabel = new QLabel(fpsWidget);
        fpsLabel->setText("FPS:");
        fpsLabel->setName(QString::fromUtf8("fpsLabel"));
        sizePolicy2.setHeightForWidth(fpsLabel->sizePolicy().hasHeightForWidth());
        fpsLabel->setSizePolicy(sizePolicy2);

        QSizePolicy sizePolicy4(QSizePolicy::Fixed,QSizePolicy::Fixed);
        sizePolicy4.setHorStretch(0);
        sizePolicy4.setVerStretch(0);
        fpsSpin = new QSpinBox(fpsWidget);
        fpsSpin->setName(QString::fromUtf8("fpsSpin"));
        sizePolicy4.setHeightForWidth(fpsSpin->sizePolicy().hasHeightForWidth());
        fpsSpin->setSizePolicy(sizePolicy4);
        fpsSpin->setRange(1,50);
        fpsSpin->setValue(10);
        MainWindow->connect(fpsSpin,SIGNAL(valueChanged(int)),MainWindow,SLOT(on_fpsSpin_valueChanged(int)));

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        fpsLayout = new Q3HBoxLayout;
#else // ICUB_USE_QT4_QT3_SUPPORT
        fpsLayout = new QHBoxLayout;
#endif // ICUB_USE_QT4_QT3_SUPPORT
        fpsLayout->addWidget(fpsLabel);
        fpsLayout->addWidget(fpsSpin);
        fpsWidget->setLayout(fpsLayout);

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

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        menuFile = new Q3PopupMenu(MainWindow);
#else // ICUB_USE_QT4_QT3_SUPPORT
        menuFile = new QPopupMenu(MainWindow);
#endif // ICUB_USE_QT4_QT3_SUPPORT
        menuFile->setName(QString::fromUtf8("menuFile"));
        menuFile->setCaption("File");
        MainWindow->menuBar()->insertItem("File",menuFile);
        fileNewAction->addTo(menuFile);
        fileOpenAction->addTo(menuFile);
        fileSaveAction->addTo(menuFile);
        fileSaveAsAction->addTo(menuFile);
        fileExitAction->addTo(menuFile);

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        menuOptions = new Q3PopupMenu(MainWindow);
#else // ICUB_USE_QT4_QT3_SUPPORT
        menuOptions = new QPopupMenu(MainWindow);
#endif // ICUB_USE_QT4_QT3_SUPPORT
        menuOptions->setName(QString::fromUtf8("menuOptions"));
        menuOptions->setCaption("Options");
        MainWindow->menuBar()->insertItem("Options",menuOptions);
        optionsJointLimitsAction->addTo(menuOptions);
        optionsShowTimelineAction->addTo(menuOptions);
        optionsConfigureiCubGUIAction->addTo(menuOptions);

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        menuHelp = new Q3PopupMenu(MainWindow);
#else // ICUB_USE_QT4_QT3_SUPPORT
        menuHelp = new QPopupMenu(MainWindow);
#endif // ICUB_USE_QT4_QT3_SUPPORT
        menuHelp->setName(QString::fromUtf8("menuHelp"));
        menuHelp->setCaption("Help");
        MainWindow->menuBar()->insertItem("Help",menuHelp);
        helpAboutAction->addTo(menuHelp);

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        toolBar = new Q3ToolBar(MainWindow);
#else // ICUB_USE_QT4_QT3_SUPPORT
        toolBar = new QToolBar(MainWindow);
#endif // ICUB_USE_QT4_QT3_SUPPORT
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
        playAction->addTo(toolBar);
        toolBar->setWidget(fpsWidget);
    } // setupUi
};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

//QT_END_NAMESPACE

#endif // UI_MAINAPPLICATIONFORM_H

