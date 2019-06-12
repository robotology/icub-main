#ifndef CUSTOMSPINBOX_H
#define CUSTOMSPINBOX_H

#include <QObject>
#include <QSpinBox>
#include "downloader.h"

class CustomSpinBox : public QSpinBox
{
    Q_OBJECT
public:
    CustomSpinBox(icubCanProto_boardType_t boardType, QWidget *parent = nullptr);

    void setData(QVariant v);
    QVariant getData();
    void clear();
    void setCurrentValue(int v);

private:
    QVariant data;
    int previousValue;
    bool modified;

private slots:
    void onValueChanged(int);
};

#endif // CUSTOMSPINBOX_H
