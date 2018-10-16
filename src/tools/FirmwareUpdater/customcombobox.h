#ifndef CUSTOMCOMBOBOX_H
#define CUSTOMCOMBOBOX_H

#include <QObject>
#include <QComboBox>
#include <QMutex>

#define GAINAMPROLE Qt::UserRole + 1

class CustomComboBox : public QComboBox
{
    Q_OBJECT

public:
    CustomComboBox(QWidget *parent = nullptr);
    void clear();
    void setCurrIndex(int v);
    void initItems();
    void clearItems();
    void addCustomValue(float val);
    void setIndexFromAmpGain(int g);

private:
    int previousIndex;
    bool modified;
    bool initialized;
    QMutex mutex;

private slots:
    void onCurrentIndexChanged(int v);
};

#endif // CUSTOMCOMBOBOX_H
