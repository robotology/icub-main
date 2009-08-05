/*
 * settingsdialog.h
 */

#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include "ui_settingsdialogform.h"

/*
  @author Zi Ree <Zi Ree @ Second Life>
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
