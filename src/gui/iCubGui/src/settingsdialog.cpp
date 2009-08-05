/***************************************************************************
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*
 * settingsdialog.cpp
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
}

void SettingsDialog::on_floorTranslucencySpin_valueChanged(int value)
{
  qDebug("floorTranslucencyChanged(%d)",value);
}

