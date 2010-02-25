/*
 * ui_mainapplicationform.h
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
    QTabWidget *avatarPropsTab;
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
		//gridLayout->setSpacing(8);
        gridLayout->setName(QString::fromUtf8("gridLayout"));
        gridLayout1 = new QGridLayout();
		//gridLayout1->setSpacing(8);
        gridLayout1->setName(QString::fromUtf8("gridLayout1"));
        animationView = new AnimationView(centralwidget,config);
        animationView->setName(QString::fromUtf8("animationView"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorStretch(0);
        sizePolicy.setVerStretch(0);
        sizePolicy.setHeightForWidth(animationView->sizePolicy().hasHeightForWidth());
        animationView->setSizePolicy(sizePolicy);

        //gridLayout1->addWidget(animationView, 0, 0, 1, 2);
		gridLayout1->addMultiCellWidget(animationView, 0, 0, 0, 1);


        //gridLayout->addLayout(gridLayout1, 0, 0, 2, 1);
		gridLayout->addMultiCellLayout(gridLayout1, 0, 1, 0, 0);

        avatarPropsTab = new QTabWidget(centralwidget);
        avatarPropsTab->setName(QString::fromUtf8("avatarPropsTab"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy1.setHorStretch(0);
        sizePolicy1.setVerStretch(0);
        sizePolicy1.setHeightForWidth(avatarPropsTab->sizePolicy().hasHeightForWidth());
        avatarPropsTab->setSizePolicy(sizePolicy1);
        avatarTab = new QWidget();
        avatarTab->setName(QString::fromUtf8("avatarTab"));
        gridLayout2 = new QGridLayout(avatarTab);
		//gridLayout2->setSpacing(8);
        gridLayout2->setName(QString::fromUtf8("gridLayout2"));
        figureLabel = new QLabel(avatarTab);
        figureLabel->setName(QString::fromUtf8("figureLabel"));
		figureLabel->setText("Figure:");
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Preferred);
		sizePolicy2.setHorStretch(0);
        sizePolicy2.setVerStretch(0);
        sizePolicy2.setHeightForWidth(figureLabel->sizePolicy().hasHeightForWidth());
        figureLabel->setSizePolicy(sizePolicy2);

        //gridLayout2->addWidget(figureLabel, 0, 0, 1, 1);
		gridLayout2->addMultiCellWidget(figureLabel, 0, 0, 0, 0);

        figureCombo = new QComboBox(avatarTab);
        figureCombo->setName(QString::fromUtf8("figureCombo"));
		figureCombo->clear();
        figureCombo->insertStringList(QStringList() << "iCub",0);
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy3.setHorStretch(0);
        sizePolicy3.setVerStretch(0);
        sizePolicy3.setHeightForWidth(figureCombo->sizePolicy().hasHeightForWidth());
        figureCombo->setSizePolicy(sizePolicy3);

        //gridLayout2->addWidget(figureCombo, 0, 1, 1, 1);
		gridLayout2->addMultiCellWidget(figureCombo, 0, 0, 1, 1);
        
		scaleLabel = new QLabel(avatarTab);
        scaleLabel->setName(QString::fromUtf8("scaleLabel"));
		scaleLabel->setText("Size:");
        sizePolicy2.setHeightForWidth(scaleLabel->sizePolicy().hasHeightForWidth());
        scaleLabel->setSizePolicy(sizePolicy2);

        //gridLayout2->addWidget(scaleLabel, 0, 2, 1, 1);
		gridLayout2->addMultiCellWidget(scaleLabel, 0, 0, 2, 2);

        scaleSpin = new QSpinBox(avatarTab);
        scaleSpin->setName(QString::fromUtf8("scaleSpin"));
		scaleSpin->setSuffix("%");
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorStretch(0);
        sizePolicy4.setVerStretch(0);
        sizePolicy4.setHeightForWidth(scaleSpin->sizePolicy().hasHeightForWidth());
        scaleSpin->setSizePolicy(sizePolicy4);
        scaleSpin->setMinValue(30);
        scaleSpin->setMaxValue(200);
        scaleSpin->setValue(100);

        //gridLayout2->addWidget(scaleSpin, 0, 3, 1, 1);
		gridLayout2->addMultiCellWidget(scaleSpin, 0, 0, 3, 3);

        editPartLabel = new QLabel(avatarTab);
        editPartLabel->setName(QString::fromUtf8("editPartLabel"));
		editPartLabel->setText("Edit Part:");
        sizePolicy2.setHeightForWidth(editPartLabel->sizePolicy().hasHeightForWidth());
        editPartLabel->setSizePolicy(sizePolicy2);

        //gridLayout2->addWidget(editPartLabel, 1, 0, 1, 1);
		gridLayout2->addMultiCellWidget(editPartLabel, 1, 1, 0, 0);

        editPartCombo = new QComboBox(avatarTab);
        editPartCombo->setName(QString::fromUtf8("editPartCombo"));
		editPartCombo->clear();
		editPartCombo->insertStringList(animationView->partNames(),0);
        sizePolicy3.setHeightForWidth(editPartCombo->sizePolicy().hasHeightForWidth());
        editPartCombo->setSizePolicy(sizePolicy3);

        //gridLayout2->addWidget(editPartCombo, 1, 1, 1, 3);
		gridLayout2->addMultiCellWidget(editPartCombo, 1, 1, 1, 3);

        rotationGroupBox = new QGroupBox(avatarTab);
        rotationGroupBox->setName(QString::fromUtf8("rotationGroupBox"));
		rotationGroupBox->setTitle("Rotation");
        gridLayout3 = new QGridLayout(rotationGroupBox);
		//gridLayout3->setSpacing(8);
        gridLayout3->setName(QString::fromUtf8("gridLayout3"));
        xRotationLabel = new QLabel(rotationGroupBox);
        xRotationLabel->setName(QString::fromUtf8("xRotationLabel"));
		xRotationLabel->setText("X Rotation:");
        sizePolicy2.setHeightForWidth(xRotationLabel->sizePolicy().hasHeightForWidth());
        xRotationLabel->setSizePolicy(sizePolicy2);

        //gridLayout3->addWidget(xRotationLabel, 0, 0, 1, 1);
		gridLayout3->addMultiCellWidget(xRotationLabel, 0, 0, 0, 0);

        xRotationEdit = new QLineEdit(rotationGroupBox);
        xRotationEdit->setName(QString::fromUtf8("xRotationEdit"));
        QSizePolicy sizePolicy5(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy5.setHorStretch(0);
        sizePolicy5.setVerStretch(0);
        sizePolicy5.setHeightForWidth(xRotationEdit->sizePolicy().hasHeightForWidth());
        xRotationEdit->setSizePolicy(sizePolicy5);

        //gridLayout3->addWidget(xRotationEdit, 0, 1, 1, 1);
		gridLayout3->addMultiCellWidget(xRotationEdit, 0, 0, 1, 1);

        shiftLabel = new QLabel(rotationGroupBox);
        shiftLabel->setName(QString::fromUtf8("shiftLabel"));
		shiftLabel->setText("Shift");
        sizePolicy2.setHeightForWidth(shiftLabel->sizePolicy().hasHeightForWidth());
		shiftLabel->setSizePolicy(sizePolicy2);
        shiftLabel->setMinimumSize(QSize(40, 0));
        //shiftLabel->setStyleSheet(QString::fromUtf8("color: #ab0000;\n"));
		shiftLabel->setPaletteForegroundColor(QColor(0xAB,0x00,0x00));
        shiftLabel->setAlignment(Qt::AlignRight|Qt::AlignVCenter);
		//gridLayout3->addWidget(shiftLabel, 0, 1, 2, 1);
        gridLayout3->addMultiCellWidget(shiftLabel, 0, 0, 2, 2);

        xRotationSlider = new QSlider(rotationGroupBox);
        xRotationSlider->setName(QString::fromUtf8("xRotationSlider"));
        sizePolicy5.setHeightForWidth(xRotationSlider->sizePolicy().hasHeightForWidth());
        xRotationSlider->setSizePolicy(sizePolicy5);
        xRotationSlider->setOrientation(Qt::Horizontal);

        //gridLayout3->addWidget(xRotationSlider, 1, 0, 1, 3);
		gridLayout3->addMultiCellWidget(xRotationSlider, 1, 1, 0, 2);

        yRotationLabel = new QLabel(rotationGroupBox);
        yRotationLabel->setName(QString::fromUtf8("yRotationLabel"));
		yRotationLabel->setText("Y Rotation:");
        sizePolicy2.setHeightForWidth(yRotationLabel->sizePolicy().hasHeightForWidth());
        yRotationLabel->setSizePolicy(sizePolicy2);

        //gridLayout3->addWidget(yRotationLabel, 2, 0, 1, 1);
		gridLayout3->addMultiCellWidget(yRotationLabel, 2, 2, 0, 0);

        yRotationEdit = new QLineEdit(rotationGroupBox);
        yRotationEdit->setName(QString::fromUtf8("yRotationEdit"));
        sizePolicy5.setHeightForWidth(yRotationEdit->sizePolicy().hasHeightForWidth());
        yRotationEdit->setSizePolicy(sizePolicy5);

        //gridLayout3->addWidget(yRotationEdit, 2, 1, 1, 1);
		gridLayout3->addMultiCellWidget(yRotationEdit, 2, 2, 1, 1);

        altLabel = new QLabel(rotationGroupBox);
        altLabel->setName(QString::fromUtf8("altLabel"));
		altLabel->setText("Alt");
        sizePolicy2.setHeightForWidth(altLabel->sizePolicy().hasHeightForWidth());
        altLabel->setSizePolicy(sizePolicy2);
        altLabel->setMinimumSize(QSize(40, 0));
        //altLabel->setStyleSheet(QString::fromUtf8("color: #20af00;\n"));
		altLabel->setPaletteForegroundColor(QColor(0x20,0xAF,0x00));
        altLabel->setAlignment(Qt::AlignRight|Qt::AlignVCenter);

        //gridLayout3->addWidget(altLabel, 2, 2, 1, 1);
		gridLayout3->addMultiCellWidget(altLabel, 2, 2, 2, 2);

        yRotationSlider = new QSlider(rotationGroupBox);
        yRotationSlider->setName(QString::fromUtf8("yRotationSlider"));
        sizePolicy5.setHeightForWidth(yRotationSlider->sizePolicy().hasHeightForWidth());
        yRotationSlider->setSizePolicy(sizePolicy5);
        yRotationSlider->setOrientation(Qt::Horizontal);

        //gridLayout3->addWidget(yRotationSlider, 3, 0, 1, 3);
		gridLayout3->addMultiCellWidget(yRotationSlider, 3, 3, 0, 2);

        zRotationLabel = new QLabel(rotationGroupBox);
        zRotationLabel->setName(QString::fromUtf8("zRotationLabel"));
		zRotationLabel->setText("Z Rotation:");
        sizePolicy2.setHeightForWidth(zRotationLabel->sizePolicy().hasHeightForWidth());
        zRotationLabel->setSizePolicy(sizePolicy2);

        //gridLayout3->addWidget(zRotationLabel, 4, 0, 1, 1);
		gridLayout3->addMultiCellWidget(zRotationLabel, 4, 4, 0, 0);

        zRotationEdit = new QLineEdit(rotationGroupBox);
        zRotationEdit->setName(QString::fromUtf8("zRotationEdit"));
        sizePolicy5.setHeightForWidth(zRotationEdit->sizePolicy().hasHeightForWidth());
        zRotationEdit->setSizePolicy(sizePolicy5);

        //gridLayout3->addWidget(zRotationEdit, 4, 1, 1, 1);
		gridLayout3->addMultiCellWidget(zRotationEdit, 4, 4, 1, 1);

        ctrlLabel = new QLabel(rotationGroupBox);
        ctrlLabel->setName(QString::fromUtf8("ctrlLabel"));
		ctrlLabel->setText("Ctrl");
        sizePolicy2.setHeightForWidth(ctrlLabel->sizePolicy().hasHeightForWidth());
        ctrlLabel->setSizePolicy(sizePolicy2);
        ctrlLabel->setMinimumSize(QSize(40, 0));
        //ctrlLabel->setStyleSheet(QString::fromUtf8("color: #0000e2;\n"));
        ctrlLabel->setPaletteForegroundColor(QColor(0x00,0x00,0xE2));
		ctrlLabel->setAlignment(Qt::AlignRight|Qt::AlignVCenter);

        //gridLayout3->addWidget(ctrlLabel, 4, 2, 1, 1);
		gridLayout3->addMultiCellWidget(ctrlLabel, 4, 4, 2, 2);

		zRotationSlider = new QSlider(rotationGroupBox);
        zRotationSlider->setName(QString::fromUtf8("zRotationSlider"));
        sizePolicy5.setHeightForWidth(zRotationSlider->sizePolicy().hasHeightForWidth());
        zRotationSlider->setSizePolicy(sizePolicy5);
        zRotationSlider->setOrientation(Qt::Horizontal);

        //gridLayout3->addWidget(zRotationSlider, 5, 0, 1, 3);
		gridLayout3->addMultiCellWidget(zRotationSlider, 5, 5, 0, 2);

        //gridLayout2->addWidget(rotationGroupBox, 2, 0, 1, 4);
		gridLayout2->addMultiCellWidget(rotationGroupBox, 2, 2, 0, 3);

        positionGroupBox = new QGroupBox(avatarTab);
        positionGroupBox->setName(QString::fromUtf8("positionGroupBox"));
		positionGroupBox->setTitle("Position");
        gridLayout4 = new QGridLayout(positionGroupBox);
		//gridLayout4->setSpacing(8);
        gridLayout4->setName(QString::fromUtf8("gridLayout4"));
        xPositionLabel = new QLabel(positionGroupBox);
        xPositionLabel->setName(QString::fromUtf8("xPositionLabel"));
		xPositionLabel->setText("X Position:");
        sizePolicy2.setHeightForWidth(xPositionLabel->sizePolicy().hasHeightForWidth());
        xPositionLabel->setSizePolicy(sizePolicy2);

        //gridLayout4->addWidget(xPositionLabel, 0, 0, 1, 1);
		gridLayout4->addMultiCellWidget(xPositionLabel, 0, 0, 0, 0);

        xPositionEdit = new QLineEdit(positionGroupBox);
        xPositionEdit->setName(QString::fromUtf8("xPositionEdit"));
        sizePolicy5.setHeightForWidth(xPositionEdit->sizePolicy().hasHeightForWidth());
        xPositionEdit->setSizePolicy(sizePolicy5);

        //gridLayout4->addWidget(xPositionEdit, 0, 1, 1, 1);
		gridLayout4->addMultiCellWidget(xPositionEdit, 0, 0, 1, 1);

        xPositionSlider = new QSlider(positionGroupBox);
        xPositionSlider->setName(QString::fromUtf8("xPositionSlider"));
        sizePolicy5.setHeightForWidth(xPositionSlider->sizePolicy().hasHeightForWidth());
        xPositionSlider->setSizePolicy(sizePolicy5);
        xPositionSlider->setMinValue(-19600);
        xPositionSlider->setMaxValue(19600);
        xPositionSlider->setOrientation(Qt::Horizontal);

        //gridLayout4->addWidget(xPositionSlider, 1, 0, 1, 2);
		gridLayout4->addMultiCellWidget(xPositionSlider, 1, 1, 0, 1);

        ypositionLabel = new QLabel(positionGroupBox);
        ypositionLabel->setName(QString::fromUtf8("ypositionLabel"));
		ypositionLabel->setText("Y Position:");
        sizePolicy2.setHeightForWidth(ypositionLabel->sizePolicy().hasHeightForWidth());
        ypositionLabel->setSizePolicy(sizePolicy2);

        //gridLayout4->addWidget(ypositionLabel, 2, 0, 1, 1);
		gridLayout4->addMultiCellWidget(ypositionLabel, 2, 2, 0, 0);

        yPositionEdit = new QLineEdit(positionGroupBox);
        yPositionEdit->setName(QString::fromUtf8("yPositionEdit"));
        sizePolicy5.setHeightForWidth(yPositionEdit->sizePolicy().hasHeightForWidth());
        yPositionEdit->setSizePolicy(sizePolicy5);

        //gridLayout4->addWidget(yPositionEdit, 2, 1, 1, 1);
		gridLayout4->addMultiCellWidget(yPositionEdit, 2, 2, 1, 1);

        yPositionSlider = new QSlider(positionGroupBox);
        yPositionSlider->setName(QString::fromUtf8("yPositionSlider"));
        sizePolicy5.setHeightForWidth(yPositionSlider->sizePolicy().hasHeightForWidth());
        yPositionSlider->setSizePolicy(sizePolicy5);
        yPositionSlider->setMinValue(-19600);
        yPositionSlider->setMaxValue(19600);
        yPositionSlider->setOrientation(Qt::Horizontal);

        //gridLayout4->addWidget(yPositionSlider, 3, 0, 1, 2);
		gridLayout4->addMultiCellWidget(yPositionSlider, 3, 3, 0, 1);

        zPositionLabel = new QLabel(positionGroupBox);
        zPositionLabel->setName(QString::fromUtf8("zPositionLabel"));
		zPositionLabel->setText("Z Position:");
        sizePolicy2.setHeightForWidth(zPositionLabel->sizePolicy().hasHeightForWidth());
        zPositionLabel->setSizePolicy(sizePolicy2);

        //gridLayout4->addWidget(zPositionLabel, 4, 0, 1, 1);
		gridLayout4->addMultiCellWidget(zPositionLabel, 4, 4, 0, 0);

        zPositionEdit = new QLineEdit(positionGroupBox);
        zPositionEdit->setName(QString::fromUtf8("zPositionEdit"));
        sizePolicy5.setHeightForWidth(zPositionEdit->sizePolicy().hasHeightForWidth());
        zPositionEdit->setSizePolicy(sizePolicy5);

        //gridLayout4->addWidget(zPositionEdit, 4, 1, 1, 1);
		gridLayout4->addMultiCellWidget(zPositionEdit, 4, 4, 1, 1);

        zPositionSlider = new QSlider(positionGroupBox);
        zPositionSlider->setName(QString::fromUtf8("zPositionSlider"));
        sizePolicy5.setHeightForWidth(zPositionSlider->sizePolicy().hasHeightForWidth());
        zPositionSlider->setSizePolicy(sizePolicy5);
        zPositionSlider->setMinValue(-19600);
        zPositionSlider->setMaxValue(19600);
        zPositionSlider->setOrientation(Qt::Horizontal);

        //gridLayout4->addWidget(zPositionSlider, 5, 0, 1, 2);
		gridLayout4->addMultiCellWidget(zPositionSlider, 5, 5, 0, 1);

        //gridLayout2->addWidget(positionGroupBox, 3, 0, 1, 4);
		gridLayout2->addMultiCellWidget(positionGroupBox, 3, 3, 0, 3);

        spacerItem = new QSpacerItem(20, 31, QSizePolicy::Minimum, QSizePolicy::Expanding);

        //gridLayout2->addItem(spacerItem, 4, 1, 1, 2);
		gridLayout2->addMultiCell(spacerItem, 4, 4, 1, 2);

        avatarPropsTab->addTab(avatarTab,"iCub");

        //gridLayout->addWidget(avatarPropsTab, 0, 1, 1, 1);
		gridLayout->addMultiCellWidget(avatarPropsTab, 0, 0, 1, 1);

        gridLayout5 = new QGridLayout();
		//gridLayout5->setSpacing(8);
        gridLayout5->setName(QString::fromUtf8("gridLayout5"));
        hboxLayout = new QHBoxLayout();
        hboxLayout->setName(QString::fromUtf8("hboxLayout"));
        spacerItem1 = new QSpacerItem(0, 20, QSizePolicy::Preferred, QSizePolicy::Minimum);

        hboxLayout->addItem(spacerItem1);

        playButton = new QPushButton(centralwidget);
        playButton->setName(QString::fromUtf8("playButton"));
		playButton->setText(QString());
        QIconSet icon11;
        icon11.setPixmap(QPixmap(config.findPath("icons/play.png").c_str()), QIconSet::Automatic, QIconSet::Normal, QIconSet::Off);
        playButton->setIconSet(icon11);
		MainWindow->connect(playButton,SIGNAL(clicked()),MainWindow,SLOT(on_playButton_clicked()));
        hboxLayout->addWidget(playButton);

        fpsLabel = new QLabel(centralwidget);
		fpsLabel->setText("FPS:");
        fpsLabel->setName(QString::fromUtf8("fpsLabel"));
        sizePolicy2.setHeightForWidth(fpsLabel->sizePolicy().hasHeightForWidth());
        fpsLabel->setSizePolicy(sizePolicy2);
        hboxLayout->addWidget(fpsLabel);

        fpsSpin = new QSpinBox(centralwidget);
        fpsSpin->setName(QString::fromUtf8("fpsSpin"));
        sizePolicy4.setHeightForWidth(fpsSpin->sizePolicy().hasHeightForWidth());
        fpsSpin->setSizePolicy(sizePolicy4);
        fpsSpin->setRange(1,50);
		fpsSpin->setValue(10);
		MainWindow->connect(fpsSpin,SIGNAL(valueChanged(int)),MainWindow,SLOT(on_fpsSpin_valueChanged(int)));


        hboxLayout->addWidget(fpsSpin);
        gridLayout5->addMultiCell(hboxLayout, 0, 0, 0, 1);
        gridLayout->addMultiCell(gridLayout5, 1, 1, 1, 1);


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

		/*
        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuOptions->menuAction());
        menubar->addAction(menuHelp->menuAction());

        menuFile->addAction(fileNewAction);
        menuFile->addAction(fileOpenAction);
        menuFile->addAction(fileAddAction);
        menuFile->addSeparator();
        menuFile->addAction(fileSaveAction);
        menuFile->addAction(fileSaveAsAction);
        menuFile->addSeparator();
        menuFile->addSeparator();
        menuFile->addAction(fileExitAction);
        menuOptions->addAction(optionsJointLimitsAction);
        menuOptions->addAction(optionsShowTimelineAction);
        menuOptions->addAction(optionsConfigureiCubGUIAction);
        menuHelp->addAction(helpContentsAction);
        menuHelp->addAction(helpIndexAction);
        menuHelp->addSeparator();
        menuHelp->addAction(helpAboutAction);
        toolBar->addAction(fileNewAction);
        toolBar->addAction(fileOpenAction);
        toolBar->addAction(fileAddAction);
        toolBar->addSeparator();
        toolBar->addAction(fileSaveAction);
        toolBar->addAction(fileSaveAsAction);
        toolBar->addSeparator();
        toolBar->addAction(resetCameraAction);
		*/

		retranslateUi(MainWindow);

        avatarPropsTab->setCurrentPage(0);

        //QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi
	
    void retranslateUi(QMainWindow *MainWindow)
    {                
        
        
        
        
        
        
        
        
        
        

    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

//QT_END_NAMESPACE

#endif // UI_MAINAPPLICATIONFORM_H