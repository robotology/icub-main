#include "customcombobox.h"
#include "downloader.h"

CustomComboBox::CustomComboBox(QWidget *parent) : QComboBox(parent), mutex(QMutex::Recursive)
{
    initialized = false;

    modified = false;
    setStyleSheet("font: 9pt");
    previousIndex = -1;
    connect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));

    initItems();
}


void CustomComboBox::clear()
{
    mutex.lock();
    if(previousIndex < 0){
        mutex.unlock();
        return;
    }
    disconnect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    //setCurrentIndex(previousIndex);
    previousIndex = -1;
    modified = false;
    setStyleSheet("font: 9pt");
    connect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    mutex.unlock();
}

void CustomComboBox::setCurrIndex(int v)
{
    mutex.lock();
    if(modified || v == currentIndex()){
        mutex.unlock();
        return;
    }
    disconnect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    setCurrentIndex(v);
    previousIndex = v;
    connect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    mutex.unlock();
}

void CustomComboBox::onCurrentIndexChanged(int v)
{
    mutex.lock();
    if(previousIndex < 0){
        mutex.unlock();
        return;
    }

    if(v == count() -1 && itemData(count() - 1,GAINAMPROLE).toInt() < 0){
        removeItem(count() - 1);
        disconnect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
        setCurrentIndex(0);
        previousIndex = 0;
        connect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    } else if(itemData(count() - 1,GAINAMPROLE).toInt() < 0){
        removeItem(count() - 1);
    }

    modified = true;
    setStyleSheet("color: rgb(239, 41, 41);"
                  "font: bold 9pt");
    mutex.unlock();
}

void CustomComboBox::clearItems()
{
    mutex.lock();
    while (count() > 0) {
        removeItem(count() - 1);
    }
    initialized = false;
    mutex.unlock();
}
void CustomComboBox::initItems()
{
    mutex.lock();
    if(initialized){
        mutex.unlock();
        return;
    }
    addItem("48,000",48);
    setItemData(count() - 1,ampl_gain48,GAINAMPROLE);
    addItem("36,000",36);
    setItemData(count() - 1,ampl_gain36,GAINAMPROLE);
    addItem("24,000",24);
    setItemData(count() - 1,ampl_gain24,GAINAMPROLE);
    addItem("20,000",20);
    setItemData(count() - 1,ampl_gain20,GAINAMPROLE);
    addItem("16,000",16);
    setItemData(count() - 1,ampl_gain16,GAINAMPROLE);
    addItem("10,000",10);
    setItemData(count() - 1,ampl_gain10,GAINAMPROLE);
    addItem("8,000",8);
    setItemData(count() - 1,ampl_gain08,GAINAMPROLE);
    addItem("6,000",6);
    setItemData(count() - 1,ampl_gain06,GAINAMPROLE);
    addItem("4,000",4);
    setItemData(count() - 1,ampl_gain04,GAINAMPROLE);

    initialized = true;
    mutex.unlock();
}

void CustomComboBox::addCustomValue(float val)
{
    int c = count();
    QString s = QString("%1").arg(val);
    QString s1 = itemText(c - 1);


    if(modified || s1 == s){
        return;
    }
    if(itemData(c - 1,GAINAMPROLE ).toInt() < 0){
        removeItem(c - 1);
    }
    addItem(s,val);
    setItemData(c, -100, GAINAMPROLE);
    disconnect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    setCurrentIndex(c);
    connect(this,SIGNAL(currentIndexChanged(int)),this,SLOT(onCurrentIndexChanged(int)));
    previousIndex = c;
}

