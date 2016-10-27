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

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(FirmwareUpdaterCore *core,bool adminMode,QWidget *parent = 0);
    ~MainWindow();

     void refreshDevices(bool refresh = false, int msecs = 3000);

private:
     void removeChildren(QTreeWidgetItem *);

     void checkConnectionButton(QTreeWidgetItem*);
     void emptyNode(QTreeWidgetItem *it);
     void populateInfo(int boardNum);
     void getCanBoards(QTreeWidgetItem *it, bool refresh = false);
     void checkEnableButtons();
     void setEthBoardInfo(int index,QString info, QTreeWidgetItem *refreshNode = NULL);
     void setCanBoardInfo   (int bus, int id, QString info, QString ethAddress = "", QTreeWidgetItem *refreshNode = NULL);
     void setCanBoardAddress(int bus, int id, int canType, QPair<QString,QString>addr, QTreeWidgetItem *refreshNode = NULL);
     void setEthBoardAddress(int index,QString address, QTreeWidgetItem *refreshNode = NULL);

     //void loading(bool, bool disableAll = false);
     void setNodeRestartNeed(QTreeWidgetItem *refreshNode, bool need = true);
     void uploadEthApplication(QString);
     void uploadCanApplication(QString filename,QString ethAddress = "");
     void uploadLoader(QString);
     void uploadUpdater(QString);

private:
    Ui::MainWindow *ui;
    FirmwareUpdaterCore *core;
    QProgressBar *progress;
    QLabel *infoResult;
    QMutex loadingMutex;
    QMutex mutex1;
    QList<SelectionCheckBox *> selectedNodes;
    bool isLoading;
    QTreeWidget *infoTreeWidget;

signals:
    void appendInfo(boardInfo2_t,eOipv4addr_t);
    void appendInfo(QString = "");
    void canBoardsRetrieved(QTreeWidgetItem *it,QList <sBoard>, bool refresh);
    void refreshEthBoardsNode(QTreeWidgetItem*, bool refresh = false, bool refreshAll = false);
    void setInfoRes(QString);
    void deviceSelectionChanged();
    void needSetRestartOnSelected();
    void refreshCanBoardsFromEth(QTreeWidgetItem*);
    void needLoading(bool,bool);


private slots:
    void loading(bool, bool disableAll = false);
    void onNeedSetRestartOnSelected();
    void onUpdateProgressBar(float fraction);
    void onUploadApplication(bool);
    void onUploadLoader(bool);
    void onUploadUpdater(bool);
    void onBootUpdater(bool);
    void onJumpToUpdater(bool click);
    void onGoToMaintenance(bool click);
    void onGoToApplication(bool click);
    void onBootApplication(bool);
    void onCalibrate(bool);
    void populateEthBoardsNode(QTreeWidgetItem*, bool refreshSingleNode = false, bool refreshAll = false);
    void onDeviceSelectionChanged();
    void onConnect();
    void onAppendInfo(QString);
    void onAppendInfo(boardInfo2_t, eOipv4addr_t address);
    void onSelectionChanged(bool selected);
    void onDeviceExpanded(QTreeWidgetItem*);
    void onCanBoardsRetrieved(QTreeWidgetItem *it, QList <sBoard> canBoards, bool refresh);
    void onBlinkPressed(bool);
    void onChangeInfo(bool);
    void onSelectionCheckDestroy(QObject*);
    void onChangeAddress(bool);
    void onRestartBoards(bool click);
    //void onRestartBoards5Secs(bool click);
    void onSetInfoRes(QString);
};

#endif // MAINWINDOW_H
