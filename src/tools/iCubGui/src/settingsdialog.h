/*
 * settingsdialog.h
 */

#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include "ui_settingsdialogform.h"

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

class SettingsDialog : public QDialog, Ui::SettingsDialogForm
{
  Q_OBJECT

  public:
    SettingsDialog(QWidget* parent=0);
    ~SettingsDialog();

  signals:
    void configChanged();

  protected slots:
    void on_applyButton_clicked();
    void on_okButton_clicked();
    void on_cancelButton_clicked();

    void on_useFogCheckbox_toggled(bool state);
    void on_floorTranslucencySpin_valueChanged(int value);

    //void on_easeInCheckbox_toggled(bool state);
    //void on_easeOutCheckbox_toggled(bool state);
};

#endif


