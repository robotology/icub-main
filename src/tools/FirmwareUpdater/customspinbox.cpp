#include "customspinbox.h"

CustomSpinBox::CustomSpinBox(icubCanProto_boardType_t boardType,QWidget *parent) : QSpinBox(parent)
{
    setMinimum(0);
    if(boardType == icubCanProto_boardType__strain2){
        setMaximum(0xFFFF);
    } else {
        setMaximum(0x3FF);
    }
    setStyleSheet("font: 9pt");
    modified = false;
    connect(this,SIGNAL(valueChanged(int)),this,SLOT(onValueChanged(int)));
}


void CustomSpinBox::setData(QVariant v)
{
    data = v;
}

QVariant CustomSpinBox::getData()
{
    return data;
}

void CustomSpinBox::clear()
{
    modified = false;
    //setValue(previousValue);
    setStyleSheet("font: 9pt");
}

void CustomSpinBox::setCurrentValue(int v)
{
    if(v == value() || modified){
        return;
    }

    blockSignals(true);
    setValue(v);
    previousValue = v;
    blockSignals(false);
}

void CustomSpinBox::onValueChanged(int v)
{
    modified = true;
    setStyleSheet("color: rgb(239, 41, 41);"
                            "font: bold 9pt");

}
