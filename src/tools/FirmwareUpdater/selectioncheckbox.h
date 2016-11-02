#ifndef SELECTIONCHECKBOX_H
#define SELECTIONCHECKBOX_H

#include <QObject>
#include <QWidget>
#include <QCheckBox>
#include <QTreeWidgetItem>
#include "firmwareupdatercore.h"

#define INDEX_OF_BOARD          Qt::UserRole
#define EMPTY_NODE              Qt::UserRole + 1
#define CONNECTED               Qt::UserRole + 2
#define DEVICE_LEVEL            Qt::UserRole + 3
#define CAN_TYPE                Qt::UserRole + 4
#define REFRESH_NEED            Qt::UserRole + 5
#define CAN_UPLOAD_LOADER       Qt::UserRole + 6
#define CAN_UPLOAD_APP          Qt::UserRole + 7
#define CAN_UPLOAD_UPDATER      Qt::UserRole + 8
#define CAN_JUMP_UPDATER        Qt::UserRole + 9


#define DEVICE  1
#define ID      2
#define ADDRESS 3
#define PROCESS 4
#define VERSION 5
#define INFO    6


class SelectionCheckBox : public QCheckBox
{
    Q_OBJECT
    
public:
    SelectionCheckBox(QWidget * = NULL);
    SelectionCheckBox(FirmwareUpdaterCore *core,QTreeWidgetItem *it,QWidget *parent=0);
    //~SelectionCheckBox();
    bool isSelected();
//    QTreeWidgetItem *getTreeNode();
//    QString getBoardType();
//    int getBoardIndex();
//    bool boardIsEth();
//    bool boardIsCan();
    void setSelected(bool);
    
    

private:
//    FirmwareUpdaterCore *core;
//    QTreeWidgetItem *treeNode;
//    int boardIndex;
    bool selected;
//    QString boardType;
//    bool isEth;
//    bool isCan;
public slots:
    void onSelectEnded();

private slots:
    void onSelectionChanged(bool);

    
signals:
    void needChangeSelection(bool);
    void selectedChanged(bool);
    void needLoading(bool,bool);
};

#endif // SELECTIONCHECKBOX_H


