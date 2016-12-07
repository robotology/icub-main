#ifndef CHANGEINFODIALOG_H
#define CHANGEINFODIALOG_H

#include <QDialog>

namespace Ui {
class ChangeInfoDialog;
}

class ChangeInfoDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ChangeInfoDialog(QWidget *parent = 0);
    ~ChangeInfoDialog();
    void setOldInfo(QString info);
    QString getNewInfo();

private:
    Ui::ChangeInfoDialog *ui;


};

#endif // CHANGEINFODIALOG_H
