/*
 * settingsdialog.cpp
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


#include "settingsdialog.h"
#include "ui_settingsdialog.h"
#include "settings.h"

SettingsDialog::SettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingsDialog)
{
    ui->setupUi(this);
    ui->fogCheck->setChecked(Settings::fog());
    ui->spinBox->setValue(Settings::floorTranslucency());
}

SettingsDialog::~SettingsDialog()
{
    delete ui;
}

void SettingsDialog::on_btnApply_clicked()
{
    Settings::setFog(ui->fogCheck->isChecked());
    Settings::setFloorTranslucency(ui->spinBox->value());
    configChanged();
    //qApp->processEvents();
}

void SettingsDialog::on_btnCancel_clicked()
{
    reject();
}

void SettingsDialog::on_btnOk_clicked()
{
    on_btnApply_clicked();
    accept();
}
