#include "changeaddressdialog.h"
#include "ui_changeaddressdialog.h"

ChangeAddressDialog::ChangeAddressDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ChangeAddressDialog)
{
    ui->setupUi(this);
}

ChangeAddressDialog::~ChangeAddressDialog()
{
    delete ui;
}


void ChangeAddressDialog::setOldAddress(QString address)
{
   ui->editOld->setText(address);
}

QString ChangeAddressDialog::getNewAddress()
{
    return ui->editNew->text();
}
