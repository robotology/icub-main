/*
* ui_settingsdialog.h
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

#ifndef UI_SETTINGSDIALOGFORM_H
#define UI_SETTINGSDIALOGFORM_H

#include <qpushbutton.h>
#include <qtabwidget.h>
#include <qcheckbox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <qdialog.h>
#include <qmetaobject.h>
#include <qapplication.h>

#ifdef ICUB_USE_QT4_QT3_SUPPORT
#include <q3gridlayout.h>
#include <q3boxlayout.h>
#else // ICUB_USE_QT4_QT3_SUPPORT
#include <qlayout.h>
#endif // ICUB_USE_QT4_QT3_SUPPORT


//QT_BEGIN_NAMESPACE

class Ui_SettingsDialogForm
{
public:
    QPushButton *okButton;
    QPushButton *cancelButton;
    QPushButton *applyButton;
    QTabWidget *tabWidget4;
    QWidget *tab;
    QSpacerItem *spacerItem;
    QCheckBox *useFogCheckbox;
    QLabel *floorTranslucencyLabel;
    QSpinBox *floorTranslucencySpin;
    QSpacerItem *spacerItem1;
    QSpacerItem *spacerItem2;

#ifdef ICUB_USE_QT4_QT3_SUPPORT
    Q3GridLayout *gridLayout;
    Q3GridLayout *gridLayout1;
    Q3HBoxLayout *hboxLayout;
    Q3VBoxLayout *vboxLayout;
    Q3HBoxLayout *hboxLayout1;
#else // ICUB_USE_QT4_QT3_SUPPORT
    QGridLayout *gridLayout;
    QGridLayout *gridLayout1;
    QHBoxLayout *hboxLayout;
    QVBoxLayout *vboxLayout;
    QHBoxLayout *hboxLayout1;
#endif // ICUB_USE_QT4_QT3_SUPPORT

    void setupUi(QDialog *SettingsDialogForm)
    {
        if (QString(SettingsDialogForm->name()).isEmpty())
            SettingsDialogForm->setName(QString::fromUtf8("SettingsDialogForm"));
        //SettingsDialogForm->setWindowModality(Qt::WindowModal);
        SettingsDialogForm->setModal(true);
        SettingsDialogForm->resize(337, 200);
#ifdef ICUB_USE_QT4_QT3_SUPPORT
        gridLayout = new Q3GridLayout(SettingsDialogForm);
#else // ICUB_USE_QT4_QT3_SUPPORT
        gridLayout = new QGridLayout(SettingsDialogForm);
#endif // ICUB_USE_QT4_QT3_SUPPORT
        gridLayout->setSpacing(6);
        gridLayout->setMargin(11);
        gridLayout->setName(QString::fromUtf8("gridLayout"));
        okButton = new QPushButton(SettingsDialogForm);
        okButton->setName(QString::fromUtf8("okButton"));

        gridLayout->addMultiCellWidget(okButton, 1, 1, 1, 1);

        cancelButton = new QPushButton(SettingsDialogForm);
        cancelButton->setName(QString::fromUtf8("cancelButton"));

        //gridLayout->addWidget(cancelButton, 1, 3, 1, 1);
        gridLayout->addMultiCellWidget(cancelButton, 1, 1, 3, 3);

        applyButton = new QPushButton(SettingsDialogForm);
        applyButton->setName(QString::fromUtf8("applyButton"));

        //gridLayout->addWidget(applyButton, 1, 2, 1, 1);
        gridLayout->addMultiCellWidget(applyButton, 1, 1, 2, 2);

        tabWidget4 = new QTabWidget(SettingsDialogForm);
        tabWidget4->setName(QString::fromUtf8("tabWidget4"));
        tab = new QWidget();
        tab->setName(QString::fromUtf8("tab"));
#ifdef ICUB_USE_QT4_QT3_SUPPORT
        gridLayout1 = new Q3GridLayout(tab);
#else // ICUB_USE_QT4_QT3_SUPPORT
        gridLayout1 = new QGridLayout(tab);
#endif // ICUB_USE_QT4_QT3_SUPPORT
        gridLayout1->setSpacing(6);
        gridLayout1->setMargin(11);
        gridLayout1->setName(QString::fromUtf8("gridLayout1"));
        spacerItem = new QSpacerItem(20, 70, QSizePolicy::Minimum, QSizePolicy::Expanding);

        //gridLayout1->addItem(spacerItem, 1, 0, 1, 1);
        gridLayout1->addMultiCell(spacerItem, 1, 1, 0, 0);

#ifdef ICUB_USE_QT4_QT3_SUPPORT
        hboxLayout = new Q3HBoxLayout();
        vboxLayout = new Q3VBoxLayout();
        hboxLayout1 = new Q3HBoxLayout();
#else // ICUB_USE_QT4_QT3_SUPPORT
        hboxLayout = new QHBoxLayout();
        vboxLayout = new QVBoxLayout();
        hboxLayout1 = new QHBoxLayout();
#endif // ICUB_USE_QT4_QT3_SUPPORT
        hboxLayout->setSpacing(6);
        hboxLayout->setName(QString::fromUtf8("hboxLayout"));
        vboxLayout->setSpacing(6);
        vboxLayout->setName(QString::fromUtf8("vboxLayout"));
        useFogCheckbox = new QCheckBox(tab);
        useFogCheckbox->setName(QString::fromUtf8("useFogCheckbox"));

        vboxLayout->addWidget(useFogCheckbox);

        hboxLayout1->setSpacing(6);
        hboxLayout1->setName(QString::fromUtf8("hboxLayout1"));
        floorTranslucencyLabel = new QLabel(tab);
        floorTranslucencyLabel->setName(QString::fromUtf8("floorTranslucencyLabel"));
        floorTranslucencyLabel->setAlignment(Qt::BreakAnywhere);

        hboxLayout1->addWidget(floorTranslucencyLabel);

        floorTranslucencySpin = new QSpinBox(tab);
        floorTranslucencySpin->setName(QString::fromUtf8("floorTranslucencySpin"));
        floorTranslucencySpin->setMaxValue(100);

        hboxLayout1->addWidget(floorTranslucencySpin);


        vboxLayout->addLayout(hboxLayout1);


        hboxLayout->addLayout(vboxLayout);

        spacerItem1 = new QSpacerItem(406, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        hboxLayout->addItem(spacerItem1);


        //gridLayout1->addLayout(hboxLayout, 0, 0, 1, 1);
        gridLayout1->addMultiCellLayout(hboxLayout, 0, 0, 0, 0);

        tabWidget4->addTab(tab, QString());

        //gridLayout->addWidget(tabWidget4, 0, 0, 1, 4);
        gridLayout->addMultiCellWidget(tabWidget4, 0, 0, 0, 3);

        spacerItem2 = new QSpacerItem(61, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        //gridLayout->addItem(spacerItem2, 1, 0, 1, 1);
        gridLayout->addMultiCell(spacerItem2, 1, 1, 0, 0);

        retranslateUi(SettingsDialogForm);

        tabWidget4->setCurrentPage(0);

        //SettingsDialogForm->connect(floorTranslucencySpin,SIGNAL(valueChanged(int)),SettingsDialogForm,SLOT(on_floorTranslucencySpin_valueChanged(int)));

        SettingsDialogForm->connect(okButton,SIGNAL(clicked()),SettingsDialogForm,SLOT(on_okButton_clicked()));
        SettingsDialogForm->connect(cancelButton,SIGNAL(clicked()),SettingsDialogForm,SLOT(on_cancelButton_clicked()));
        SettingsDialogForm->connect(applyButton,SIGNAL(clicked()),SettingsDialogForm,SLOT(on_applyButton_clicked()));
        //SettingsDialogForm->connect(floorTranslucencySpin,SIGNAL(valueChanged(int)),SettingsDialogForm,SLOT(on_floorTranslucencySpin_valueChanged(int)));
        //QMetaObject::connectSlotsByName(SettingsDialogForm);
    } // setupUi

    void retranslateUi(QDialog *SettingsDialogForm)
    {
        SettingsDialogForm->setCaption("Configure iCubGUI");
        okButton->setText("&OK");
        okButton->setAccel(QKeySequence("Alt+O"));
        cancelButton->setText("&Cancel");
        cancelButton->setAccel(QKeySequence("Alt+C"));
        applyButton->setText("&Apply");
        applyButton->setAccel(QKeySequence("Alt+A"));
        useFogCheckbox->setText("Use Fog");
        floorTranslucencyLabel->setText("Floor Translucency:");
        floorTranslucencySpin->setSuffix("%");
        //tabWidget4->setTabLabel(tabWidget4->indexOf(tab), "OpenGL");
        tabWidget4->setTabLabel(tab, "OpenGL");
        Q_UNUSED(SettingsDialogForm);
    } // retranslateUi

};

namespace Ui {
    class SettingsDialogForm: public Ui_SettingsDialogForm {};
} // namespace Ui

//QT_END_NAMESPACE

#endif // UI_SETTINGSDIALOGFORM_H

