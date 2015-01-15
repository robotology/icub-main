/*
 * settingsdialog.h
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

#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QDialog>

namespace Ui {
class SettingsDialog;
}

class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SettingsDialog(QWidget *parent = 0);
    ~SettingsDialog();

private:
    Ui::SettingsDialog *ui;

signals:
    void configChanged();

private slots:
    void on_btnApply_clicked();
    void on_btnCancel_clicked();
    void on_btnOk_clicked();
};

#endif // SETTINGSDIALOG_H

