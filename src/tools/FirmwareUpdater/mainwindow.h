#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMutex>
#include <QMainWindow>
#include "firmwareupdatercore.h"
#include "selectioncheckbox.h"
#include <QTreeWidgetItem>
#include <QProgressBar>
#include <QLabel>
#include <QPair>
#include <QFutureWatcher>

namespace Ui {
class MainWindow;
}

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

    virtual void refresh();
    QString getBoardType();


    QString retrieveCanBoards();
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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(FirmwareUpdaterCore *core,bool adminMode,QWidget *parent = 0);
    ~MainWindow();

     void refreshDevices();

private:
     void removeChildren(QTreeWidgetItem *);

     void checkConnectionButton(QTreeWidgetItem*);
     void emptyNode(QTreeWidgetItem *it);
     void populateInfo(int boardNum);
     void getCanBoards(QTreeWidgetItem *it, bool refresh = false);
     void checkEnableButtons();
     void setEthBoardInfo(int index,QString info, QTreeWidgetItem *refreshNode = NULL);
     void setCanBoardInfo   (QPair<int, int> canData, QString info, QString address = "", int deviceId = -1, QTreeWidgetItem *refreshNode = NULL);
     void setCanBoardAddress(QPair<int, int> canData, int canType, QPair<QString,QString>addr, int deviceId = -1, QTreeWidgetItem *refreshNode = NULL);
     void setEthBoardAddress(int index,QString address, QTreeWidgetItem *refreshNode = NULL);

     //void loading(bool, bool disableAll = false);
     void setNodeRestartNeed(QTreeWidgetItem *refreshNode, bool need = true);
     void uploadEthApplication(QString);
     void uploadCanApplication(QString filename, QString address = "", int deviceId = -1, CustomTreeWidgetItem *node = NULL);
     void uploadLoader(QString);
     void uploadUpdater(QString);

private:
    Ui::MainWindow *ui;
    FirmwareUpdaterCore *core;
    QProgressBar *progress;
    QLabel *infoResult;
    QMutex loadingMutex;
    QMutex mutex1;
    QList<CustomTreeWidgetItem *> selectedNodes;
    bool isLoading;
    QTreeWidget *infoTreeWidget;
    int loadCounter;
    QFutureWatcher<bool> watcher;

signals:
    void appendInfo(boardInfo2_t,eOipv4addr_t);
    void appendInfo(QString = "");
    void canBoardsRetrieved(QTreeWidgetItem *it, bool refresh);
    void refreshEthBoardsNode(QTreeWidgetItem*, bool refresh = false, bool refreshAll = false);
    void setInfoRes(QString);
    void deviceSelectionChanged();
    void needSetRestartOnSelected();
    void refreshCanBoardsFromEth(QTreeWidgetItem*);
    void needLoading(bool,bool,QString = "");


private slots:
    void onFutureFinished();
    void loading(bool, bool disableAll = false, QString msg = "");
    void onNeedSetRestartOnSelected();
    void onUpdateProgressBar(float fraction);
    void onUploadApplication(bool);
    void onUploadLoader(bool);
    void onUploadUpdater(bool);
    void onBootUpdater(bool);
    void onJumpToUpdater(bool click);
    void onGoToMaintenance(bool click);
    void onGoToApplication(bool click);
    void onEraseEprom(bool click);
    void onBootApplication(bool);
    void onCalibrate(bool);
    void populateEthBoardsNode(QTreeWidgetItem*, bool refreshSingleNode = false, bool refreshAll = false);
    void onDeviceSelectionChanged();
    void onConnect();
    void onAppendInfo(QString);
    void onAppendInfo(boardInfo2_t, eOipv4addr_t address);
    void onSelectionChanged(bool selected);
    void onDeviceExpanded(QTreeWidgetItem*);
    void onCanBoardsRetrieved(QTreeWidgetItem *it, bool refresh);
    void onBlinkPressed(bool);
    void onChangeInfo(bool);
    void onSelectionCheckDestroy(QObject*);
    void onChangeAddress(bool);
    void onRestartBoards(bool click);
    //void onRestartBoards5Secs(bool click);
    void onSetInfoRes(QString);
};

#endif // MAINWINDOW_H
