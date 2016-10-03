#include "changeinfodialog.h"
#include "ui_changeinfodialog.h"

ChangeInfoDialog::ChangeInfoDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ChangeInfoDialog)
{
    ui->setupUi(this);
}

ChangeInfoDialog::~ChangeInfoDialog()
{
    delete ui;
}

void ChangeInfoDialog::setOldInfo(QString info)
{
    ui->oldEdit->setText(info);
}

QString ChangeInfoDialog::getNewInfo()
{
    return ui->newEdit->text();
}
