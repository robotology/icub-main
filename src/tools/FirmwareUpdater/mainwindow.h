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
#include "calibrationwindow.h"
#include "customtreewidgetitem.h"
namespace Ui {
class MainWindow;
}


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
     void checkSelButton(QTreeWidgetItem*);
     void emptyNode(QTreeWidgetItem *it);
     void populateInfo(int boardNum);
     void populateCANinfo(sBoard canboard);
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
     void checkSelection(bool selected, CustomTreeWidgetItem *c);


private:
    Ui::MainWindow *ui;
    CalibrationWindow *calibDlg;
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
    icubCanProto_boardType_t sgboardtype;

signals:
    void appendInfo(boardInfo2_t,eOipv4addr_t);
    void appendInfo(QString = "");
    void appendInfo(sBoard canboard);
    void canBoardsRetrieved(QTreeWidgetItem *it, bool refresh);
    void refreshEthBoardsNode(QTreeWidgetItem*, bool refresh = false, bool refreshAll = false);
    void setInfoRes(QString);
    void deviceSelectionChanged();
    void needSetRestartOnSelected();
    void refreshCanBoardsFromEth(QTreeWidgetItem*);
    void needLoading(bool,bool,QString = "",bool = true);



private slots:
    void onFutureFinished();
    void loading(bool, bool disableAll = false, QString msg = "", bool infiniteLoad = true);
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
    void onSel(void);
    void onDes(void);
    void onAppendInfo(sBoard);
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
    bool getCANaddress(CustomTreeWidgetItem *child, int &cbus, int &adr, QString &cbustr, QString &cadrstr);
    bool getDeviceID(QTreeWidgetItem *child, QString &idstr, QString &devicestr);
    bool getDeviceID(QString devicefullstring, QString &idstr, QString &devicestr);
};

#endif // MAINWINDOW_H
