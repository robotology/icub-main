#ifndef CHANGEADDRESSDIALOG_H
#define CHANGEADDRESSDIALOG_H

#include <QDialog>

namespace Ui {
class ChangeAddressDialog;
}

class ChangeAddressDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ChangeAddressDialog(QWidget *parent = 0);
    ~ChangeAddressDialog();
    void setOldAddress(QString address);
    QString getNewAddress();

private:
    Ui::ChangeAddressDialog *ui;
};

#endif // CHANGEADDRESSDIALOG_H
