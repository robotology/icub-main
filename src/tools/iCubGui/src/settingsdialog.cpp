/*
 * settingsdialog.cpp
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

#include "settings.h"
#include "settingsdialog.h"

SettingsDialog::SettingsDialog(QWidget* parent) : QDialog(parent)
{
  qDebug("SettingsDialog::SettingsDialog()");

  setupUi(this);

  useFogCheckbox->setChecked(Settings::fog());
  floorTranslucencySpin->setValue(Settings::floorTranslucency());
}

SettingsDialog::~SettingsDialog()
{
  qDebug("SettingsDialog::~SettingsDialog()");
}

void SettingsDialog::on_applyButton_clicked()
{
  qDebug("accept()");

  Settings::setFog(useFogCheckbox->isChecked());
  Settings::setFloorTranslucency(floorTranslucencySpin->value());
  emit configChanged();
  qApp->processEvents();
}

void SettingsDialog::on_okButton_clicked()
{
  qDebug("acceptOk()");
  on_applyButton_clicked();
  accept();
}

void SettingsDialog::on_cancelButton_clicked()
{
  qDebug("reject()");
  reject();
}

void SettingsDialog::on_useFogCheckbox_toggled(bool state)
{
  qDebug("useFogToggled(%d)",state);
  //Settings::setFog(state);
}

void SettingsDialog::on_floorTranslucencySpin_valueChanged(int value)
{
  qDebug("floorTranslucencyChanged(%d)",value);
  //Settings::setFloorTranslucency(value);
}

