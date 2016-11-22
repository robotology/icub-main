#ifndef CUSTOMTREEWIDGETITEM_H
#define CUSTOMTREEWIDGETITEM_H

#include <QObject>
#include "firmwareupdatercore.h"
#include "selectioncheckbox.h"
#include <QTreeWidgetItem>

#define ETH_TREE_NODE   QTreeWidgetItem::UserType + 1
#define CAN_TREE_NODE   QTreeWidgetItem::UserType + 2

#define ETH_TREE_ROOT_NODE   QTreeWidgetItem::UserType + 3
#define CAN_TREE_ROOT_NODE   QTreeWidgetItem::UserType + 4



class CustomTreeWidgetItem : public QObject, public QTreeWidgetItem
{
    Q_OBJECT

public:
    CustomTreeWidgetItem(QTreeWidgetItem*parent, QStringList fields, int indexOfBoard, FirmwareUpdaterCore *core,int type);

    int getIndexOfBoard();
    void setCheckEnabled(bool enable);
    bool checkIsEnabled();
    void setCheckSelected(bool selected);
    bool isCheckSelected();
    QTreeWidgetItem *getParentNode();
    void setCanBoards(QList <sBoard> boards);
    virtual void refresh();
    QString getBoardType();



    QString retrieveCanBoards(bool force = true);
    EthBoard getBoard();
    QList <sBoard> getCanBoards();
    sBoard getCanBoard(int index);


protected:

    int m_indexOfBoard;
    FirmwareUpdaterCore *core;
    SelectionCheckBox *check;
    QTreeWidgetItem *parentNode;
    QList <sBoard> canBoards;

signals:
    void selectedChanged(bool);
    void needLoading(bool,bool);
    void selectionCheckDestroy(QObject*);

private slots:
    void onSelectedChanged(bool);

};

class EthTreeWidgetItem : public CustomTreeWidgetItem
{
    Q_OBJECT

public:
    EthTreeWidgetItem(QTreeWidgetItem*parent, FirmwareUpdaterCore *core, int indexOfBoard);
    void setSelectedBoard(bool, int);
    void refresh();





};

class CanTreeWidgetItem : public CustomTreeWidgetItem
{
    Q_OBJECT

public:
    CanTreeWidgetItem(QTreeWidgetItem*,FirmwareUpdaterCore *core,int indexOfBoard);

    sBoard getBoard();
    void refresh();

};

#endif // CUSTOMTREEWIDGETITEM_H
