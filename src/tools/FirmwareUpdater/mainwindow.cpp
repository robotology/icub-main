#include "mainwindow.h"
#include "changeinfodialog.h"
#include "changeaddressdialog.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <QCheckBox>
#include <QtConcurrent/QtConcurrent>
#include <QFileDialog>
#include <QMessageBox>


MainWindow::MainWindow(FirmwareUpdaterCore *core, bool adminMode, QWidget *parent) :
    QMainWindow(parent), /*mutex(QMutex::Recursive)*/
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType <QList<sBoard> > ("QList<sBoard>");
    qRegisterMetaType <QVector<int> > ("QVector<int>");
    qRegisterMetaType <boardInfo2_t> ("boardInfo2_t");
    qRegisterMetaType <eOipv4addr_t> ("eOipv4addr_t");
    isLoading = false;
    loadCounter = 0;


    QWidget *container = new QWidget(ui->statusBar);
    QHBoxLayout *layout = new QHBoxLayout(container);
    container->setLayout(layout);
    container->setFixedHeight(40);
    //container->setFixedWidth(300);


    progress = new QProgressBar(container);
    progress->setEnabled(false);
    progress->setFixedWidth(100);
    layout->addWidget(progress);

    infoResult = new QLabel(container);
    infoResult->setVisible(false);
    layout->addWidget(infoResult);


    QSpacerItem *spacer = new QSpacerItem(20,40,QSizePolicy::MinimumExpanding,QSizePolicy::Fixed);
    layout->addSpacerItem(spacer);

    ui->statusBar->addWidget(container,100);
    //ui->statusBar->layout()->addWidget(container);
    ui->splitter->setStretchFactor(0,80);
    ui->splitter->setStretchFactor(0,20);

    infoTreeWidget = new QTreeWidget(ui->groupBox_2);
    infoTreeWidget->setVisible(false);
    ui->groupBox_2->layout()->addWidget(infoTreeWidget);
    infoTreeWidget->setColumnCount(2);
    infoTreeWidget->setHeaderLabels(QStringList() << "Name" << "Value");
    infoTreeWidget->addTopLevelItem(new QTreeWidgetItem(infoTreeWidget,QStringList() << "Board"));
    infoTreeWidget->addTopLevelItem(new QTreeWidgetItem(infoTreeWidget,QStringList() << "Bootstrap Processes"));
    infoTreeWidget->addTopLevelItem(new QTreeWidgetItem(infoTreeWidget,QStringList() << "Properties of the Processes"));



    this->core = core;

    QList < QPair<QString,QVariant> > devices = core->getDevices();

    //ui->devicesTree->setSelectionMode(QTreeWidget::MultiSelection);
    for(int i=0;i<devices.count();i++){
        int type = devices.at(i).first.contains("ETH") ? ETH_TREE_ROOT_NODE : CAN_TREE_ROOT_NODE;

        CustomTreeWidgetItem *item = new CustomTreeWidgetItem(NULL,QStringList() << "" << devices.at(i).first << devices.at(i).second.toString(),-1,core,type);
        ui->devicesTree->addTopLevelItem(item);

        QTreeWidgetItem *empty = new QTreeWidgetItem(item,QStringList() << "" << "?");
        empty->setData(0,EMPTY_NODE,true);

        item->setTextColor(DEVICE,QColor(Qt::red));
    }

    if(!adminMode){
        ui->advancedGroup->setVisible(false);
    }

    connect(core,SIGNAL(updateProgress(float)),
            this,SLOT(onUpdateProgressBar(float)),Qt::QueuedConnection);

    connect(ui->devicesTree,SIGNAL(itemSelectionChanged()),
            this,SLOT(onDeviceSelectionChanged()));
    connect(this,SIGNAL(deviceSelectionChanged()),
                this,SLOT(onDeviceSelectionChanged()),Qt::QueuedConnection);

    connect(ui->devicesTree,SIGNAL(itemExpanded(QTreeWidgetItem*)),
            this,SLOT(onDeviceExpanded(QTreeWidgetItem*)));

    connect(this,SIGNAL(refreshCanBoardsFromEth(QTreeWidgetItem*)),
            this,SLOT(onDeviceExpanded(QTreeWidgetItem*)),Qt::QueuedConnection);


    connect(ui->connectButton,SIGNAL(clicked(bool)),
            this,SLOT(onConnect()));

    connect(this,SIGNAL(appendInfo(QString)),
            this,SLOT(onAppendInfo(QString)),Qt::QueuedConnection);

    connect(this,SIGNAL(appendInfo(boardInfo2_t,eOipv4addr_t)),
            this,SLOT(onAppendInfo(boardInfo2_t,eOipv4addr_t)),Qt::QueuedConnection);

    connect(this,SIGNAL(canBoardsRetrieved(QTreeWidgetItem*,bool )),
            this,SLOT(onCanBoardsRetrieved(QTreeWidgetItem*, bool)),Qt::QueuedConnection);

    connect(this,SIGNAL(refreshEthBoardsNode(QTreeWidgetItem*,bool,bool)),
            this,SLOT(populateEthBoardsNode(QTreeWidgetItem*,bool,bool)),Qt::QueuedConnection);

    connect(this,SIGNAL(setInfoRes(QString)),
            this,SLOT(onSetInfoRes(QString)),Qt::QueuedConnection);

    connect(this,SIGNAL(needSetRestartOnSelected()),
            this,SLOT(onNeedSetRestartOnSelected()),Qt::QueuedConnection);

    connect(this,SIGNAL(needLoading(bool,bool,QString)),
                this,SLOT(loading(bool,bool,QString)),Qt::QueuedConnection);

    connect(ui->btnBlink,SIGNAL(clicked(bool)),this,SLOT(onBlinkPressed(bool)));
    connect(ui->btnCahngeInfo,SIGNAL(clicked(bool)),this,SLOT(onChangeInfo(bool)));
    connect(ui->btnChangeIp,SIGNAL(clicked(bool)),this,SLOT(onChangeAddress(bool)));
    connect(ui->btnChangeCanAddr,SIGNAL(clicked(bool)),this,SLOT(onChangeAddress(bool)));
    connect(ui->btnRestart,SIGNAL(clicked(bool)),this,SLOT(onRestartBoards(bool)));
    //connect(ui->btnRestartSecs,SIGNAL(clicked(bool)),this,SLOT(onRestartBoards5Secs(bool)));
    connect(ui->btnCalibrate,SIGNAL(clicked(bool)),this,SLOT(onCalibrate(bool)));
    connect(ui->btnBootApp,SIGNAL(clicked(bool)),this,SLOT(onBootApplication(bool)));
    connect(ui->btnBootUpdater,SIGNAL(clicked(bool)),this,SLOT(onBootUpdater(bool)));
    connect(ui->btnUploadApp,SIGNAL(clicked(bool)),this,SLOT(onUploadApplication(bool)));
    connect(ui->btnUploadLoader,SIGNAL(clicked(bool)),this,SLOT(onUploadLoader(bool)));
    connect(ui->btnUploadUpdater,SIGNAL(clicked(bool)),this,SLOT(onUploadUpdater(bool)));
    connect(ui->btnJumpUpdater,SIGNAL(clicked(bool)),this,SLOT(onJumpToUpdater(bool)));
    connect(ui->btnGoToMaintenance,SIGNAL(clicked(bool)),this,SLOT(onGoToMaintenance(bool)));
    connect(ui->btnGoToApplication,SIGNAL(clicked(bool)),this,SLOT(onGoToApplication(bool)));
    connect(ui->btnEraseEeprom,SIGNAL(clicked(bool)),this,SLOT(onEraseEprom(bool)));
    connect(&watcher, SIGNAL(finished()), this, SLOT(onFutureFinished()));

    ui->devicesTree->sortItems(ADDRESS,Qt::AscendingOrder);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onUploadLoader(bool click)
{
    QString filename = QFileDialog::getOpenFileName(this,"Choose File",QDir::home().absolutePath());

    if(filename.isEmpty()){
        return;
    }

    QtConcurrent::run(this,&MainWindow::uploadLoader,filename);


}
void MainWindow::onUploadUpdater(bool click)
{
    QString filename = QFileDialog::getOpenFileName(this,"Choose File",QDir::home().absolutePath());

    if(filename.isEmpty()){
        return;
    }

    QtConcurrent::run(this,&MainWindow::uploadUpdater,filename);


}
void MainWindow::onUploadApplication(bool click)
{
    QString filename = QFileDialog::getOpenFileName(this,"Choose File",QDir::home().absolutePath());

    if(filename.isEmpty()){
        return;
    }

    if(!selectedNodes.isEmpty()){
        if(selectedNodes.first()->type() == ETH_TREE_NODE){
            QtConcurrent::run(this,&MainWindow::uploadEthApplication,filename);
        } else if(selectedNodes.first()->type() == CAN_TREE_NODE){
            if(selectedNodes.first()->getParentNode()->type() == ETH_TREE_NODE){
                QString address = selectedNodes.first()->getParentNode()->text(ADDRESS);
                QtConcurrent::run(this,&MainWindow::uploadCanApplication,filename,address,-1);
            }else if(selectedNodes.first()->getParentNode()->type() == CAN_TREE_ROOT_NODE){
                QString driver = selectedNodes.first()->getParentNode()->text(DEVICE);
                int deviceId = selectedNodes.first()->getParentNode()->text(ID).toInt();
                QtConcurrent::run(this,&MainWindow::uploadCanApplication,filename,driver,deviceId);
            }
        }
    }




}

void MainWindow::uploadLoader(QString filename)
{
    QString result;
    core->uploadLoader(filename,&result);
    setInfoRes(result);
    needSetRestartOnSelected();
}

void MainWindow::uploadUpdater(QString filename)
{
    QString result;
    core->uploadUpdater(filename,&result);
    setInfoRes(result);
    needSetRestartOnSelected();
}

void MainWindow::uploadEthApplication(QString filename)
{
    QString result;
    core->uploadEthApplication(filename,&result);
    setInfoRes(result);
    needSetRestartOnSelected();
}

void MainWindow::uploadCanApplication(QString filename,QString address,int deviceId)
{
    QString result;
    core->uploadCanApplication(filename,&result,address,deviceId);
    setInfoRes(result);

//    if(!ethAddress.isEmpty()){
//        refreshCanBoardsFromEth(selectedNodes.first()->getTreeNode()->parent());
//    }
}

void MainWindow::onNeedSetRestartOnSelected()
{
//    foreach (SelectionCheckBox *c, selectedNodes) {
//        setNodeRestartNeed(c->getTreeNode());
//    }
}


void MainWindow::onBootApplication(bool click)
{
    core->bootFromApplication();
    if(ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt() == 2 && ui->devicesTree->currentItem()->parent()->text(DEVICE) == "ETH"){
        QtConcurrent::run(this,&MainWindow::populateInfo,ui->devicesTree->currentItem()->data(0,INDEX_OF_BOARD).toInt());
    }

}

void MainWindow::onBootUpdater(bool click)
{
    core->bootFromUpdater();
    if(ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt() == 2 && ui->devicesTree->currentItem()->parent()->text(DEVICE) == "ETH"){
        QtConcurrent::run(this,&MainWindow::populateInfo,ui->devicesTree->currentItem()->data(0,INDEX_OF_BOARD).toInt());
    }
}

void MainWindow::onJumpToUpdater(bool click)
{
    loading(true,true);
    QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::jumpToUpdater);
    watcher.setFuture(future);
}

void MainWindow::onGoToMaintenance(bool click)
{
    loading(true,true);
    QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::goToMaintenance);
    watcher.setFuture(future);

}

void MainWindow::onGoToApplication(bool click)
{
    loading(true,true);
    QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::goToApplication);
    watcher.setFuture(future);

}

void MainWindow::onEraseEprom(bool click)
{
    if(selectedNodes.count() > 0){
        if(selectedNodes.first()->type() == ETH_TREE_NODE){
            loading(true,true);
            QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::eraseEthEprom);
            watcher.setFuture(future);
        } else if(selectedNodes.first()->type() == CAN_TREE_NODE){
            
        }
    }
}

void MainWindow::onFutureFinished()
{
    if(watcher.result()){
        foreach (CustomTreeWidgetItem *it, selectedNodes) {
            it->refresh();
        }
    }
    loading(false);
    checkEnableButtons();
}

void MainWindow::onBlinkPressed(bool click)
{
    core->blinkEthBoards();
}

void MainWindow::onChangeAddress(bool click)
{
    if(selectedNodes.count() != 1){
        return;
    }
    CustomTreeWidgetItem *child = selectedNodes.first();

    QString oldAddress;
    if(child->type() == ETH_TREE_NODE){
        oldAddress = core->getEthBoardAddress(child->getIndexOfBoard());
    } else if(child->type() == CAN_TREE_NODE){
        oldAddress = child->text(ID);
    }else{
        return;
    }

    ChangeAddressDialog dlg;
    dlg.setOldAddress(oldAddress);
    if(dlg.exec() == QDialog::Accepted){
        if(dlg.getNewAddress() != oldAddress){
            if(child->type() == ETH_TREE_NODE){
                emptyNode(child);
                QtConcurrent::run(this,&MainWindow::setEthBoardAddress,
                                  child->getIndexOfBoard(),
                                  dlg.getNewAddress(),
                                  child);
            } else if(child->type() == CAN_TREE_NODE){
                QString address = child->getParentNode()->text(ADDRESS);
                for(int i=0;i<child->childCount();i++){
                    child->child(i)->setDisabled(true);
                }
                int bus = child->text(ADDRESS).remove("CAN_").toInt();
                int id  = child->text(ID).toInt();
                int canType = child->data(0,CAN_TYPE).toInt();


                if(child->data(0,DEVICE_LEVEL).toInt() == 3){   

                    QPair <QString,QString> addr;
                    addr.second = dlg.getNewAddress();
                    addr.first = address;

                    QtConcurrent::run(this,&MainWindow::setCanBoardAddress,
                                      QPair<int,int>(bus,
                                      id),
                                      canType,
                                      addr,
                                      -1,
                                      child->getParentNode());



                }else if(child->data(0,DEVICE_LEVEL).toInt() == 1){

                    QString driver = child->getParentNode()->text(DEVICE);
                    int networkId = child->getParentNode()->text(ID).toInt();

                    QPair <QString,QString> addr;
                    addr.second = dlg.getNewAddress();
                    addr.first = driver;

                    QtConcurrent::run(this,&MainWindow::setCanBoardAddress,
                                      QPair<int,int>(bus,
                                      id),
                                      canType,
                                      addr,
                                      networkId,
                                      child->getParentNode());
                }



            }
        }
    }
}

void MainWindow::onChangeInfo(bool click)
{
    if(selectedNodes.count() != 1){
        return;
    }

    CustomTreeWidgetItem *child = selectedNodes.first();

    QString oldInfo;
    if(child->type() == ETH_TREE_NODE){
        oldInfo = core->getEthBoardInfo(child->getIndexOfBoard());
    } else if(child->type() == CAN_TREE_NODE){
        oldInfo = child->text(INFO);
    }else{
        return;
    }

    ChangeInfoDialog dlg;
    dlg.setOldInfo(oldInfo);
    if(dlg.exec() == QDialog::Accepted){
        if(dlg.getNewInfo() != oldInfo){
            if(child->type() == ETH_TREE_NODE){
                QtConcurrent::run(this,&MainWindow::setEthBoardInfo,child->getIndexOfBoard(),dlg.getNewInfo(),child);
            } else if(child->type() == CAN_TREE_NODE){
                QString address = child->getParentNode()->text(ADDRESS);

                for(int i=0;i<child->childCount();i++){
                    child->child(i)->setDisabled(true);
                }

                if(child->data(0,DEVICE_LEVEL).toInt() == 3){

                    QtConcurrent::run(this,&MainWindow::setCanBoardInfo,
                                      QPair<int,int>(child->text(ADDRESS).remove("CAN_").toInt(),
                                      child->text(ID).toInt()),
                                      dlg.getNewInfo(),
                                      address,
                                      -1,
                                      child->getParentNode());

                } else if(child->data(0,DEVICE_LEVEL).toInt() == 1){
                    QString driver = child->getParentNode()->text(DEVICE);
                    int networkId = child->getParentNode()->text(ID).toInt();

                    QtConcurrent::run(this,&MainWindow::setCanBoardInfo,
                                      QPair<int,int>(child->text(ADDRESS).remove("CAN_").toInt(),
                                      child->text(ID).toInt()),
                                      dlg.getNewInfo(),
                                      driver,
                                      networkId,
                                      child->getParentNode());

                }



            }
        }
    }

}

void MainWindow::onCalibrate(bool click)
{

}

void MainWindow::setCanBoardInfo(QPair <int,int> canData, QString info, QString address, int deviceId,QTreeWidgetItem *refreshNode)
{
    needLoading(true,true);
    int bus = canData.first;
    int id  = canData.second;
    QString result;
    CustomTreeWidgetItem *ethNode =(CustomTreeWidgetItem*)refreshNode;

    core->setSelectedCanBoards(ethNode->getCanBoards(),address,deviceId);
    core->setCanBoardInfo(bus,id,info,address,deviceId,&result);

    result = ethNode->retrieveCanBoards();
    canBoardsRetrieved(refreshNode, true);
    needLoading(false,false,result);

}

void MainWindow::setCanBoardAddress(QPair <int,int> canData, int canType, QPair<QString, QString> addr, int deviceId,QTreeWidgetItem *refreshNode)
{
    needLoading(true,true);
    int bus = canData.first;
    int id  = canData.second;
    QString result;
    CustomTreeWidgetItem *ethNode =(CustomTreeWidgetItem*)refreshNode;
    core->setSelectedCanBoards(ethNode->getCanBoards(),addr.first,deviceId);
    core->setCanBoardAddress(bus,id,canType,addr.second,addr.first,deviceId,&result);
    result = ethNode->retrieveCanBoards();
    canBoardsRetrieved(refreshNode, false);
    needLoading(false,false,result);

}

void MainWindow::setEthBoardInfo(int index,QString info, QTreeWidgetItem *refreshNode)
{
    loading(true,true);
    QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::setEthBoardInfo,index,info);
    watcher.setFuture(future);

//    loading(true);
//    core->setEthBoardInfo(index,info);
//    refreshEthBoardsNode(refreshNode,true);
//    loading(false);
}

void MainWindow::setEthBoardAddress(int index,QString address, QTreeWidgetItem *refreshNode)
{
    loading(true,true);
    QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::setEthBoardAddress,index,address);
    watcher.setFuture(future);

//    if(core->setEthBoardAddress(index,address)){
//        qDebug() << "SUCCESS";
//        //refreshEthBoardsNode(refreshNode,true);
//        ((CustomTreeWidgetItem*)refreshNode)->refresh();;
//        checkEnableButtons();
//    }else{
//        qDebug() << "FAILED";
//        //TODO ERROR
//    }
//    loading(false);
}

void MainWindow::setNodeRestartNeed(QTreeWidgetItem *refreshNode, bool need)
{
    if(need){
        refreshNode->setIcon(DEVICE,QIcon(":/images/restart-needed.png"));
    }else{
        refreshNode->setIcon(DEVICE,QIcon());
    }

    refreshNode->setData(0,REFRESH_NEED,need);
    for(int i=0;i<refreshNode->childCount();i++){
        QTreeWidgetItem *child = refreshNode->child(i);
        child->setDisabled(need);
        child->setData(0,REFRESH_NEED,need);
    }

}

void MainWindow::onRestartBoards(bool click)
{
    core->restartEthBoards();
    //QtConcurrent::run(this,&MainWindow::refreshDevices,true,8000);
}

//void MainWindow::onRestartBoards5Secs(bool click)
//{
//    core->restartEthBoards();
//    QtConcurrent::run(this,&MainWindow::refreshDevices,true,3000);
//}

void MainWindow::checkConnectionButton(QTreeWidgetItem *it)
{
    if(it->parent() == NULL || it->data(0,DEVICE_LEVEL) == 2){
        ui->connectButton->setEnabled(true);
//        if(it->data(0,CONNECTED).toBool() == true){
//            ui->connectButton->setText("Disconnect");
//        }else{
//            ui->connectButton->setText("Connect");
//        }

    }else{
        ui->connectButton->setEnabled(false);
    }
}


void MainWindow::onDeviceSelectionChanged()
{
    if(ui->devicesTree->selectedItems().count() > 0){
        checkConnectionButton(ui->devicesTree->currentItem());

        // If it is a child node
        if(ui->devicesTree->currentItem() && ui->devicesTree->currentItem()->parent()){
            appendInfo();
            if(ui->devicesTree->currentItem()->parent()->text(DEVICE).contains("ETH")){
                for(int i=0; i < ui->devicesTree->currentItem()->parent()->childCount();i++){
                    if(ui->devicesTree->currentItem()->parent()->child(i)->isSelected() &&
                            ui->devicesTree->currentItem()->parent()->child(i)->data(0,EMPTY_NODE).toBool() == false){
                        int boardNum =  ui->devicesTree->currentItem()->parent()->child(i)->data(0,INDEX_OF_BOARD).toInt();

                        QtConcurrent::run(this,&MainWindow::populateInfo,boardNum);
                    }
                }
            }


        }else{
            appendInfo();
        }
    }else{
        ui->connectButton->setEnabled(false);
        appendInfo();
    }
    checkEnableButtons();

}

void MainWindow::populateInfo(int boardNum)
{
    loading(true);
    QString details;
    eOipv4addr_t address;
    boardInfo2_t info = core->getMoreDetails(boardNum,&details,&address);
    if(!details.isEmpty()){
        appendInfo(details);
    }else{
        appendInfo(info,address);
    }
    loading(false);
}

void MainWindow::onAppendInfo(boardInfo2_t info,eOipv4addr_t address)
{
    ui->detailsText->setVisible(false);
    infoTreeWidget->setVisible(true);

    infoTreeWidget->clear();
    QTreeWidgetItem *boardNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "Board");
    QTreeWidgetItem *bootStrapNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "Bootstrap Processes");
    QTreeWidgetItem *propertiesNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "Properties of the Processes");
    infoTreeWidget->addTopLevelItem(boardNode);
    infoTreeWidget->addTopLevelItem(bootStrapNode);
    infoTreeWidget->addTopLevelItem(propertiesNode);

    QString type;
    switch (info.boardtype) {
    case eobrd_ethtype_ems4:
        type = "EMS4";
        break;
    case eobrd_ethtype_mc4plus:
        type = "MC4PLUS";
        break;
    case eobrd_ethtype_mc2plus:
        type = "MC2PLUS";
        break;
    case eobrd_ethtype_none:
        type = "NONE";
        break;
    case eobrd_ethtype_unknown:
        type = "UNKNOWN";
        break;
    default:
        break;
    }

    QTreeWidgetItem *typeNode = new QTreeWidgetItem(boardNode, QStringList() << "Type" << type);
    boardNode->addChild(typeNode);
    boardNode->setExpanded(true);

    ACE_UINT64 mac = info.macaddress;
    char board_mac[32];

    snprintf(board_mac, sizeof(board_mac), "%02X-%02X-%02X-%02X-%02X-%02X",
                        (uint8_t)(mac >> 40) & 0xff,
                        (uint8_t)(mac >> 32) & 0xff,
                        (uint8_t)(mac >> 24) & 0xff,
                        (uint8_t)(mac >> 16) & 0xff,
                        (uint8_t)(mac >> 8 ) & 0xff,
                        (uint8_t)(mac      ) & 0xff);

    QTreeWidgetItem *macNode = new QTreeWidgetItem(boardNode, QStringList() << "Mac" << board_mac);
    boardNode->addChild(macNode);

    QTreeWidgetItem *ipNode = new QTreeWidgetItem(boardNode, QStringList() << "Ip" << ipv4tostring(address).c_str());
    boardNode->addChild(ipNode);

    QTreeWidgetItem *statusNode = new QTreeWidgetItem(boardNode, QStringList() << "Status" << (info.maintenanceIsActive ? "maintenance" : "application"));
    boardNode->addChild(statusNode);

    /*******************************************************************************/

    QTreeWidgetItem *startUpNode = new QTreeWidgetItem(bootStrapNode, QStringList() << "Startup" << core->getProcessFromUint(info.processes.startup));
    bootStrapNode->addChild(startUpNode);
    bootStrapNode->setExpanded(true);

    QTreeWidgetItem *defaultNode = new QTreeWidgetItem(bootStrapNode, QStringList() << "Default" << core->getProcessFromUint(info.processes.def2run));
    bootStrapNode->addChild(defaultNode);

    QTreeWidgetItem *runningNode = new QTreeWidgetItem(bootStrapNode, QStringList() << "Running" << core->getProcessFromUint(info.processes.runningnow));
    bootStrapNode->addChild(runningNode);

    /*******************************************************************************/


    propertiesNode->setExpanded(true);
    for(int i= 0; i<(int)info.processes.numberofthem;i++){
        QTreeWidgetItem *processNode = new QTreeWidgetItem(propertiesNode, QStringList() << QString("Process %1").arg(i));
        propertiesNode->addChild(processNode);
        processNode->setExpanded(true);

        QTreeWidgetItem *processType = new QTreeWidgetItem(processNode, QStringList() << "Type" << core->getProcessFromUint(info.processes.info[i].type));
        processNode->addChild(processType);

        QTreeWidgetItem *processVersion = new QTreeWidgetItem(processNode, QStringList() << "Version" << QString("%1,%2").arg(info.processes.info[i].version.major).arg(info.processes.info[i].version.minor));
        processNode->addChild(processVersion);

        QTreeWidgetItem *processDate = new QTreeWidgetItem(processNode, QStringList() << "Date" << QDateTime(QDate(info.processes.info[i].date.year,info.processes.info[i].date.month,info.processes.info[i].date.day),
                                                                                                             QTime(info.processes.info[i].date.hour,info.processes.info[i].date.min)).toString("yyyy/MM/dd - hh:mm"));
        processNode->addChild(processDate);

        QTreeWidgetItem *processBuilt = new QTreeWidgetItem(processNode, QStringList() << "Date" << QDateTime(QDate(info.processes.info[i].compilationdate.year,info.processes.info[i].compilationdate.month,info.processes.info[i].compilationdate.day),
                                                                                                      QTime(info.processes.info[i].compilationdate.hour,info.processes.info[i].compilationdate.min)).toString("yyyy/MM/dd - hh:mm"));
        processNode->addChild(processBuilt);

        QTreeWidgetItem *processRom= new QTreeWidgetItem(processNode, QStringList() << "Rom" << QString("[%1 - %2] kb").arg(info.processes.info[i].rom_addr_kb).arg(info.processes.info[i].rom_size_kb));
        processNode->addChild(processRom);



    }


}



void MainWindow::onAppendInfo(QString text)
{

    ui->detailsText->clear();
    if(!text.isEmpty()){
        ui->detailsText->appendPlainText(text);
    }
    infoTreeWidget->setVisible(false);
    ui->detailsText->setVisible(true);
}

void MainWindow::onConnect()
{
    setInfoRes("");
    foreach (QTreeWidgetItem *it, ui->devicesTree->selectedItems()) {
        if(it->data(0,DEVICE_LEVEL).toInt() == 0){

             if(it->data(0,CONNECTED).toBool() == true){
                 emptyNode(it);
                 it->setData(0,CONNECTED,false);
                 it->setTextColor(DEVICE,QColor(Qt::red));
             }

             if(core->connectTo(it->text(DEVICE),it->text(ID)) > 0){
                 it->setData(0,CONNECTED,true);
                 it->setExpanded(true);
                 it->setTextColor(DEVICE,QColor(Qt::green));
             }else{
                 it->setData(0,CONNECTED,false);
                 it->setTextColor(DEVICE,QColor(Qt::red));
             }

             QtConcurrent::run(this,&MainWindow::refreshDevices);
        }

        if(it->data(0,DEVICE_LEVEL).toInt() == 2){
            if(it->text(PROCESS).contains("eUpdater" )){
                QtConcurrent::run(this,&MainWindow::getCanBoards,it,false);
            }else{
                QMessageBox msgBox;
                msgBox.setText("You have to put the device in maintenace mode to perform this operation.");
                msgBox.setStandardButtons(QMessageBox::Ok );
                msgBox.setDefaultButton(QMessageBox::Ok);
                msgBox.exec();
            }
        }

        checkConnectionButton(it);
    }


}

void MainWindow::checkEnableButtons()
{
    bool isEth = false;
    bool needRestart = false;
    bool canUploadLoader = true;
    bool canUploadApp = true;
    bool canUploadUpdater = true;
    bool canJumpUpdater = true;

    for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
        QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
        for(int j=0;j<topLevel->childCount();j++){

            QTreeWidgetItem *widgItem = topLevel->child(j);
            if(widgItem->type() != ETH_TREE_ROOT_NODE  && widgItem->type() != CAN_TREE_ROOT_NODE && widgItem->type() != QTreeWidgetItem::Type){
                CustomTreeWidgetItem *child = (CustomTreeWidgetItem *)widgItem;
                if(child->isCheckSelected()){
                    if(child->type() == ETH_TREE_NODE){
                        isEth = true;


                        canUploadLoader &= child->data(0,CAN_UPLOAD_LOADER).toBool();
                        canUploadApp &= child->data(0,CAN_UPLOAD_APP).toBool();
                        canUploadUpdater &= child->data(0,CAN_UPLOAD_UPDATER).toBool();
                        canJumpUpdater &= child->data(0,CAN_JUMP_UPDATER).toBool();
                    }
                }
            }



//            for(int k=0; k<topLevel->child(j)->childCount();k++ ){
//                SelectionCheckBox *check1 = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j)->child(k),0);
//                if(check1 && check1->isEnabled() && check1->isSelected()){
//                    selectedCount++;
//                }
//            }
        }
    }

    if(selectedNodes.isEmpty()){
        ui->btnBlink->setEnabled(false);
        ui->btnBootApp->setEnabled(false);
        ui->btnBootUpdater->setEnabled(false);
        ui->btnCahngeInfo->setEnabled(false);
        ui->btnCalibrate->setEnabled(false);
        ui->btnChangeCanAddr->setEnabled(false);
        ui->btnChangeIp->setEnabled(false);
        ui->btnEraseEeprom->setEnabled(false);
        ui->btnJumpUpdater->setEnabled(false);
        ui->btnRestart->setEnabled(false);
        //ui->btnRestartSecs->setEnabled(false);
        ui->btnGoToApplication->setEnabled(false);
        ui->btnGoToMaintenance->setEnabled(false);
        ui->btnUploadApp->setEnabled(false);
        ui->btnUploadLoader->setEnabled(false);
        ui->btnUploadUpdater->setEnabled(false);
        return;
    }


    if(isEth){
        if(needRestart){
            ui->btnGoToApplication->setEnabled(true);
            ui->btnGoToMaintenance->setEnabled(true);
            ui->btnRestart->setEnabled(true);
            //ui->btnRestartSecs->setEnabled(true);
            ui->btnBlink->setEnabled(false);
            ui->btnBootApp->setEnabled(false);
            ui->btnBootUpdater->setEnabled(false);
            ui->btnCalibrate->setEnabled(false);

            ui->btnChangeCanAddr->setEnabled(false);
            ui->btnCahngeInfo->setEnabled(false);

            ui->btnChangeIp->setEnabled(false);
            ui->btnEraseEeprom->setEnabled(false);
            ui->btnJumpUpdater->setEnabled(false);
            ui->btnUploadApp->setEnabled(false);
            ui->btnUploadLoader->setEnabled(false);
            ui->btnUploadUpdater->setEnabled(false);
        }else{
            ui->btnBlink->setEnabled(true);
            ui->btnBootApp->setEnabled(true);
            ui->btnBootUpdater->setEnabled(true);

            ui->btnCalibrate->setEnabled(false);
            ui->btnChangeCanAddr->setEnabled(false);
            if(selectedNodes.count() == 1){
                ui->btnChangeIp->setEnabled(true);
                ui->btnCahngeInfo->setEnabled(true);
            }else{
                ui->btnChangeIp->setEnabled(false);
                ui->btnCahngeInfo->setEnabled(false);
            }
            ui->btnEraseEeprom->setEnabled(true);
            if(canJumpUpdater){
                ui->btnJumpUpdater->setEnabled(true);
            }else{
                ui->btnJumpUpdater->setEnabled(false);
            }
            ui->btnGoToApplication->setEnabled(true);
            ui->btnGoToMaintenance->setEnabled(true);
            ui->btnRestart->setEnabled(true);
            //ui->btnRestartSecs->setEnabled(true);
            if(canUploadApp){
                ui->btnUploadApp->setEnabled(true);
            }else{
                ui->btnUploadApp->setEnabled(false);
            }
            if(canUploadLoader){
                ui->btnUploadLoader->setEnabled(true);
            }else{
                ui->btnUploadLoader->setEnabled(false);
            }
            if(canUploadUpdater){
                ui->btnUploadUpdater->setEnabled(true);
            }else{
                ui->btnUploadUpdater->setEnabled(false);
            }
        }

    }else{
        ui->btnBlink->setEnabled(false);
        ui->btnBootApp->setEnabled(false);
        ui->btnBootUpdater->setEnabled(false);

        if(selectedNodes.count() == 1){
            ui->btnChangeCanAddr->setEnabled(true);
            ui->btnCahngeInfo->setEnabled(true);
            int canType = selectedNodes.first()->data(0,CAN_TYPE).toInt();
            if(canType == icubCanProto_boardType__strain || canType == icubCanProto_boardType__6sg){
                ui->btnCalibrate->setEnabled(true);
            }else{
                ui->btnCalibrate->setEnabled(false);
            }
        }else{
            ui->btnChangeCanAddr->setEnabled(false);
            ui->btnCahngeInfo->setEnabled(false);
            ui->btnCalibrate->setEnabled(false);
        }
        ui->btnChangeIp->setEnabled(false);
        ui->btnEraseEeprom->setEnabled(true);
        ui->btnJumpUpdater->setEnabled(false);
        ui->btnRestart->setEnabled(false);
        //ui->btnRestartSecs->setEnabled(false);
        ui->btnGoToApplication->setEnabled(false);
        ui->btnGoToMaintenance->setEnabled(false);
        ui->btnUploadApp->setEnabled(true);
        ui->btnUploadLoader->setEnabled(false);
        ui->btnUploadUpdater->setEnabled(false);
    }


}

void MainWindow::emptyNode(QTreeWidgetItem *it)
{
    removeChildren(it);
    QTreeWidgetItem *empty = new QTreeWidgetItem(it,QStringList() << "" << "?");
    empty->setData(0,EMPTY_NODE,true);
}

void MainWindow::refreshDevices()
{
    needLoading(true,true);
    QString ret;
    for (int i = 0; i<ui->devicesTree->selectedItems().count(); i++) {
        QTreeWidgetItem *it = ui->devicesTree->selectedItems().at(i);
        if(!it->parent() == NULL){
            continue;
        }

        if(it->type() == ETH_TREE_ROOT_NODE){
            refreshEthBoardsNode(it);



        }else if(it->type() == CAN_TREE_ROOT_NODE){
            CustomTreeWidgetItem *canRootNode = (CustomTreeWidgetItem*)it;
            ret = canRootNode->retrieveCanBoards();
            setInfoRes(ret);
        }
    }
    needLoading(false,false,ret);
}

void MainWindow::removeChildren(QTreeWidgetItem *it)
{
    for(int i=it->childCount() - 1; i >= 0; i--){
        QTreeWidgetItem *childItem = it->takeChild(i);

        if(childItem->type() != QTreeWidgetItem::Type){
            CustomTreeWidgetItem *child = (CustomTreeWidgetItem*)childItem;
            if(child->isCheckSelected()){
                child->setCheckSelected(false);
            }
            selectedNodes.removeOne(child);
        }



        delete childItem;
    }
}


void MainWindow::populateEthBoardsNode(QTreeWidgetItem *it, bool refreshSingleNode, bool refreshAll)
{

    if(refreshSingleNode){
        CustomTreeWidgetItem *itemToRefresh = (CustomTreeWidgetItem*)it;
        itemToRefresh->refresh();
        return;
    }

    QTreeWidgetItem *parentNode = it->parent();


    if(!refreshAll && !refreshSingleNode && core->getEthBoardList().size() > 0){
        // Remove the first child node that has text "?"
        removeChildren(it);
    }


    for (int i=0; i<core->getEthBoardList().size(); ++i)
    {
        if(refreshSingleNode && it->data(0,INDEX_OF_BOARD) != i){
            continue;
        }

        if(refreshSingleNode){
            int count = it->childCount();
            for (int i=0;i<count;i++) {
                if(!parentNode){
                    break;
                }
                QTreeWidgetItem *c = parentNode->child(i);
                foreach (CustomTreeWidgetItem *s, selectedNodes) {
                    if(s == c || s == it){
                        selectedNodes.removeOne(s);
                        s->setCheckSelected(false);
                    }
                }
            }
            it->parent()->removeChild(it);
        }

        if(!refreshAll){
            EthTreeWidgetItem *child = new EthTreeWidgetItem(it,core,i);


            connect(child,SIGNAL(selectedChanged(bool)),
                    this,SLOT(onSelectionChanged(bool)),Qt::QueuedConnection);
            connect(child,SIGNAL(destroyed(QObject*)),
                    this,SLOT(onSelectionCheckDestroy(QObject*)));
            connect(child,SIGNAL(needLoading(bool,bool)),
                    this,SLOT(loading(bool,bool)),Qt::DirectConnection);

            if(selectedNodes.count() > 0){
                if(selectedNodes.first()->getBoardType() != child->getBoardType()){
                    child->setCheckEnabled(false);
                }
            }
        }

    }

}

void MainWindow::onSelectionCheckDestroy(QObject *obj)
{
    mutex1.lock();
    selectedNodes.removeOne((CustomTreeWidgetItem*)obj);
    mutex1.unlock();
}

void MainWindow::onCanBoardsRetrieved(QTreeWidgetItem *it, bool refresh)
{

    if(it->type() != ETH_TREE_NODE){
        return;
    }

    EthTreeWidgetItem *ethNode = (EthTreeWidgetItem*)it;
    ethNode->setExpanded(true);

    if(!refresh && ethNode->getCanBoards().count() > 0){
        removeChildren(it);
        it->setData(0,EMPTY_NODE,false);
    }

    if(!refresh){
        for(int i=0;i<ethNode->getCanBoards().count();i++){

            CanTreeWidgetItem *canNode = new CanTreeWidgetItem(ethNode,core,i);

            connect(canNode,SIGNAL(selectedChanged(bool)),
                    this,SLOT(onSelectionChanged(bool)),Qt::QueuedConnection);
            connect(canNode,SIGNAL(destroyed(QObject*)),
                    this,SLOT(onSelectionCheckDestroy(QObject*)),Qt::DirectConnection);
            connect(canNode,SIGNAL(needLoading(bool,bool)),
                    this,SLOT(loading(bool,bool)),Qt::DirectConnection);


            if(selectedNodes.count() > 0){
                if(selectedNodes.first()->getBoardType() != canNode->getBoardType()){
                    canNode->setCheckEnabled(false);
                }
            }


        }
    }else{
        for(int i=0;i<ethNode->childCount();i++){
            CanTreeWidgetItem *canNode = (CanTreeWidgetItem*)ethNode->child(i);
            canNode->refresh();
        }
    }

    checkEnableButtons();

//    needLoading(false,false);
}

void MainWindow::getCanBoards(QTreeWidgetItem *it, bool refresh)
{

    if(it->type() == ETH_TREE_NODE){
        needLoading(true,true);
        QString result;
        EthTreeWidgetItem *canNode = (EthTreeWidgetItem *)it;
        result = canNode->retrieveCanBoards();
        canBoardsRetrieved(it, refresh);
        needLoading(false,false,result);
    }


}

void MainWindow::onDeviceExpanded(QTreeWidgetItem *it)
{
//    if(it->data(0,DEVICE_LEVEL).toInt() != 2){
//        return;
//    }

//    if(!it->parent()->text(DEVICE).contains("ETH")){
//        return;
//    }

//    if(it->data(0,REFRESH_NEED).toBool()){
//        return;
//    }

//    for(int i=0;i<it->childCount();i++){
//        it->child(i)->setDisabled(true);
//    }
//    QtConcurrent::run(this,&MainWindow::getCanBoards,it,false);
}


void MainWindow::onSetInfoRes(QString result)
{
    mutex1.lock();
    infoResult->setVisible(true);
    infoResult->setText(result);
    mutex1.unlock();
}

void MainWindow::onUpdateProgressBar(float fraction)
{

    loadingMutex.lock();

    if (fraction < 0.0f){
        fraction=0.0f;
    }else if (fraction>1.0f){
        fraction=1.0f;
    }
    int val = fraction*100;

    if(progress->maximum() == 0){
        progress->setMaximum(100);
    }

    if(val < 100){
        if(!progress->isEnabled()){
            progress->setEnabled(true);
        }
        ui->advancedGroup->setEnabled(false);
        ui->controlsGroup->setEnabled(false);
        infoResult->setVisible(true);
        infoResult->setText("Updating...");
        ui->devicesTree->setEnabled(false);
    }else{
        progress->setEnabled(false);
        ui->advancedGroup->setEnabled(true);
        ui->controlsGroup->setEnabled(true);
        infoResult->setText("Update Done");
        ui->devicesTree->setEnabled(true);
    }
    progress->setValue(val);
    loadingMutex.unlock();

}

void MainWindow::loading(bool load, bool disableAll,QString msg)
{

    if(load/* && !isLoading*/){
        loadCounter++;
        isLoading = true;
        progress->setMaximum(0);
        progress->setEnabled(true);
        ui->advancedGroup->setEnabled(false);
        ui->controlsGroup->setEnabled(false);
        infoResult->setVisible(true);
        if(disableAll){
            ui->devicesTree->setEnabled(false);
        }
        infoResult->setText("Loading...");
    }else /*if(!load && isLoading)*/{
        loadCounter--;
        if(loadCounter == 0){
            progress->setMaximum(100);
            progress->setEnabled(false);
            ui->advancedGroup->setEnabled(true);
            ui->controlsGroup->setEnabled(true);
            if(!ui->devicesTree->isEnabled()){
                ui->devicesTree->setEnabled(true);
            }
            infoResult->setVisible(true);
            if(!msg.isEmpty()){
                infoResult->setText(msg);
            }else{
                infoResult->setText("Operation Done");
            }
            isLoading = false;
        }

    }
}



void MainWindow::onSelectionChanged(bool selected)
{
    loading(true);
    CustomTreeWidgetItem *c = (CustomTreeWidgetItem*)sender();

    if(selected){
        if(selectedNodes.contains(c)){
            loading(false);
            checkEnableButtons();
            return;
        }

        selectedNodes.append(c);

        for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
            QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
            for(int j=0;j<topLevel->childCount();j++){

                QTreeWidgetItem *widgItem = topLevel->child(j);
                if(widgItem->type() != QTreeWidgetItem::Type){
                    CustomTreeWidgetItem *child = (CustomTreeWidgetItem*)widgItem;
                    if(child->checkIsEnabled()){
                        if(child->getBoardType() != c->getBoardType() || c->getParentNode() != child->getParentNode()){
                            child->setCheckEnabled(false);
                        }
                    }

                    for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                        QTreeWidgetItem *widgItem1 = topLevel->child(j)->child(k);
                        if(widgItem1->type() != QTreeWidgetItem::Type){
                            CustomTreeWidgetItem *child1 = (CustomTreeWidgetItem*)widgItem1;

                            if(child1->checkIsEnabled()){
                                if(child1->getBoardType() != c->getBoardType() || c->getParentNode() != child1->getParentNode()){
                                    child1->setCheckEnabled(false);
                                }
                            }
                        }

                    }
                }




            }

        }
    }else{
//        if(!selectedNodes.contains(c)){
//            loading(false);
//            checkEnableButtons();
//            return;
//        }
        selectedNodes.removeOne(c);
        bool found = false;
        for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
            QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
            for(int j=0;j<topLevel->childCount();j++){
                QTreeWidgetItem *widgItem = topLevel->child(j);
                if(widgItem->type() != QTreeWidgetItem::Type){
                    CustomTreeWidgetItem *child = (CustomTreeWidgetItem*)widgItem;
                    if(child->isCheckSelected() || found){
                        found = true;
                        break;
                    }

                    for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                        QTreeWidgetItem *widgItem1 = topLevel->child(j)->child(k);
                        if(widgItem1->type() != QTreeWidgetItem::Type){
                            CustomTreeWidgetItem *child1 = (CustomTreeWidgetItem*)widgItem1;
                            if(child1->isCheckSelected()){
                                found = true;
                                break;
                            }
                        }


                    }

                }
            }

            if(found){
                break;
            }
        }

        if(!found){
            for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
                QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
                for(int j=0;j<topLevel->childCount();j++){
                    QTreeWidgetItem *widgItem = topLevel->child(j);
                    if(widgItem->type() != QTreeWidgetItem::Type){
                        CustomTreeWidgetItem *child = (CustomTreeWidgetItem*)widgItem;

                        if(!topLevel->child(j)->isDisabled()){
                            child->setCheckEnabled(true);
                        }
                        for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                            if(topLevel->child(j)->data(0,REFRESH_NEED) == true){
                                continue;
                            }
                            QTreeWidgetItem *widgItem1 = topLevel->child(j)->child(k);
                            if(widgItem1->type() != QTreeWidgetItem::Type){
                                CustomTreeWidgetItem *child1 = (CustomTreeWidgetItem*)widgItem1;
                                if(!topLevel->child(j)->child(k)->isDisabled()){
                                    child1->setCheckEnabled(true);
                                }
                            }

                        }
                    }



                }

            }
        }

    }
    loading(false);
    checkEnableButtons();
}







/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

CustomTreeWidgetItem::CustomTreeWidgetItem(QTreeWidgetItem *parent, QStringList fields, int indexOfBoard, FirmwareUpdaterCore *core, int type) : QTreeWidgetItem(parent,fields,type), QObject()
{
    m_indexOfBoard = indexOfBoard;
    this->core = core;

    if(type != ETH_TREE_ROOT_NODE && type != CAN_TREE_ROOT_NODE){
        check = new SelectionCheckBox(core,this);
        connect(check,SIGNAL(selectedChanged(bool)),
                this,SIGNAL(selectedChanged(bool)),Qt::QueuedConnection);
        connect(check,SIGNAL(needChangeSelection(bool)),
                this,SLOT(onSelectedChanged(bool)));
        connect(check,SIGNAL(destroyed(QObject*)),
                this,SIGNAL(selectionCheckDestroy(QObject*)));
        connect(check,SIGNAL(needLoading(bool,bool)),
                this,SIGNAL(needLoading(bool,bool)),Qt::DirectConnection);
        connect(core,SIGNAL(selectedEnded()),check,SLOT(onSelectEnded()),Qt::QueuedConnection);
        this->treeWidget()->setItemWidget(this,0,check);
    }else{
        check = NULL;
    }
    parentNode = parent;
}
QTreeWidgetItem *CustomTreeWidgetItem::getParentNode()
{
    return parentNode;
}

int CustomTreeWidgetItem::getIndexOfBoard()
{
    return m_indexOfBoard;
}

void CustomTreeWidgetItem::setCheckEnabled(bool enable)
{
    check->setEnabled(enable);
}

bool CustomTreeWidgetItem::checkIsEnabled()
{
    if(!check){
        return true;
    }
    return check->isEnabled();
}

void CustomTreeWidgetItem::setCheckSelected(bool selected)
{
    check->setChecked(selected);//setSelected(selected);

}


void CustomTreeWidgetItem::refresh()
{

}

void CustomTreeWidgetItem::onSelectedChanged(bool selected)
{
    if(type() == ETH_TREE_NODE){
        core->setSelectedEthBoard(m_indexOfBoard,selected);
    }else if(type() == CAN_TREE_NODE){
        EthTreeWidgetItem *ethNode = (EthTreeWidgetItem*)parentNode;
        ethNode->setSelectedBoard(selected,m_indexOfBoard);
        selectedChanged(selected);
//        QString ethAddress;
//        if(data(0,DEVICE_LEVEL).toInt() == 3){
//            ethAddress = parentNode->text(ADDRESS);
//        }

        //QtConcurrent::run(core,&FirmwareUpdaterCore::setSelectedCanBoard,m_indexOfBoard,selected,ethAddress);
    }

}

bool CustomTreeWidgetItem::isCheckSelected()
{
    if(!check){
        return false;
    }
    return check->isSelected();
}

QString CustomTreeWidgetItem::getBoardType()
{
    return text(DEVICE);
}

EthBoard CustomTreeWidgetItem::getBoard()
{
    return core->getEthBoardList()[m_indexOfBoard];
}

QList<sBoard> CustomTreeWidgetItem::getCanBoards()
{
    return canBoards;
}

sBoard CustomTreeWidgetItem::getCanBoard(int index)
{
    sBoard board;
    if(index < canBoards.count()){
        board = canBoards.at(index);
    }
    return board;

}

QString CustomTreeWidgetItem::retrieveCanBoards()
{
    QString result;
    if(type() == ETH_TREE_NODE){
        canBoards = core->getCanBoardsFromEth(text(ADDRESS),&result,CanPacket::everyCANbus,true);
    } else if(type() == CAN_TREE_ROOT_NODE){
        canBoards = core->getCanBoardsFromDriver(text(DEVICE),text(ID).toInt(),&result,true);
    }

    return result;
}

/***************************************************************************/

EthTreeWidgetItem::EthTreeWidgetItem(QTreeWidgetItem *parent, FirmwareUpdaterCore *core, int indexOfBoard) : CustomTreeWidgetItem(parent,QStringList(),indexOfBoard,core,ETH_TREE_NODE)
{
    refresh();
    QTreeWidgetItem *empty = new QTreeWidgetItem(this,QStringList() << "" << "?");
    empty->setData(0,DEVICE_LEVEL,3);
    empty->setData(0,EMPTY_NODE,true);
}


void EthTreeWidgetItem::setSelectedBoard(bool selected, int index)
{
    sBoard b = canBoards.at(index);
    b.selected = selected;
    canBoards.replace(index,b);
}

void EthTreeWidgetItem::refresh()
{
    char board_ipaddr[16];
    char board_mac[32];

    char board_version[16];
    char board_date[24];
    char board_built[24];
    char board_type[24];
    char running_process[24];
    char board_info[32];

    memset(board_ipaddr,0,sizeof(board_ipaddr));
    memset(board_mac,0,sizeof(board_mac));
    memset(board_version,0,sizeof(board_version));
    memset(board_date,0,sizeof(board_date));
    memset(board_built,0,sizeof(board_built));
    memset(board_type,0,sizeof(board_type));
    memset(running_process,0,sizeof(running_process));
    memset(board_info,0,sizeof(board_info));

    EthBoard board = getBoard();

    snprintf(board_ipaddr, sizeof(board_ipaddr), "%s", board.getIPV4string().c_str());

    ACE_UINT64 mac = board.getInfo().macaddress;


    snprintf(board_mac, sizeof(board_mac), "%02X-%02X-%02X-%02X-%02X-%02X",
                        (uint8_t)(mac >> 40) & 0xff,
                        (uint8_t)(mac >> 32) & 0xff,
                        (uint8_t)(mac >> 24) & 0xff,
                        (uint8_t)(mac >> 16) & 0xff,
                        (uint8_t)(mac >> 8 ) & 0xff,
                        (uint8_t)(mac      ) & 0xff
            );


    snprintf(board_version, sizeof(board_version), "%s", board.getVersionfRunning().c_str());
    snprintf(board_type, sizeof(board_type), "%s", eoboards_type2string2(eoboards_ethtype2type(board.getInfo().boardtype), eobool_true));
    snprintf(running_process, sizeof(running_process), "%s", eouprot_process2string((eOuprot_process_t)board.getInfo().processes.runningnow));
    snprintf(board_info, sizeof(board_info), "%s", board.getInfoOnEEPROM().c_str());
    snprintf(board_date, sizeof(board_date), "%s", board.getDatefRunning().c_str());
    snprintf(board_built, sizeof(board_date), "%s", board.getCompilationDateOfRunning().c_str());

    QStringList myFields;
    myFields.append("");
    myFields.append(board_type);
    myFields.append("");
    myFields.append(board_ipaddr);
    myFields.append(running_process);
    myFields.append(board_version);
    myFields.append(board_info);

    setData(0,INDEX_OF_BOARD,m_indexOfBoard);
    setData(0,DEVICE_LEVEL,2);

    for(int i=0;i<myFields.count();i++) {
        setText(i,myFields.at(i));
    }

    if((eOuprot_process_t)board.getInfo().processes.runningnow == uprot_proc_Updater){
        setData(0,CAN_UPLOAD_LOADER,true);
        setData(0,CAN_UPLOAD_APP,true);
        setData(0,CAN_UPLOAD_UPDATER,false);
        setData(0,CAN_JUMP_UPDATER,false);
    } else if((eOuprot_process_t)board.getInfo().processes.runningnow == uprot_proc_ApplPROGupdater){
        setData(0,CAN_UPLOAD_LOADER,true);
        setData(0,CAN_UPLOAD_APP,false);
        setData(0,CAN_UPLOAD_UPDATER,true);
        setData(0,CAN_JUMP_UPDATER,true);
    }else{
        setData(0,CAN_UPLOAD_LOADER,false);
        setData(0,CAN_UPLOAD_APP,false);
        setData(0,CAN_UPLOAD_UPDATER,false);
        setData(0,CAN_JUMP_UPDATER,false);
    }


}



/***************************************************************************/

CanTreeWidgetItem::CanTreeWidgetItem(QTreeWidgetItem *parent, FirmwareUpdaterCore *core, int indexOfBoard) : CustomTreeWidgetItem(parent,QStringList(),indexOfBoard,core,CAN_TREE_NODE)
{
    refresh();
    //selectedChanged(getBoard().selected);
}

sBoard CanTreeWidgetItem::getBoard()
{
    return ((EthTreeWidgetItem*)parentNode)->getCanBoard(m_indexOfBoard);
}

void CanTreeWidgetItem::refresh()
{
    char board_type        [50]; memset (board_type,0,50);
    char board_process     [50]; memset (board_process,0,50);
    char board_status      [50]; memset (board_status,0,50);
    char board_add_info    [50]; memset (board_add_info,0,50);
    char board_firmware_version  [10]; memset (board_firmware_version,0,10);
    char board_appl_minor  [10]; memset (board_appl_minor,0,10);
    char board_appl_build  [10]; memset (board_appl_build,0,10);
    char board_serial      [10]; memset (board_serial,0,10);
    char board_protocol    [10]; memset (board_protocol,0,10);

    sBoard board = getBoard();

    snprintf(board_type, sizeof(board_type), "%s", eoboards_type2string((eObrd_type_t)board.type));
    switch (board.status)
    {
    case BOARD_RUNNING:
        strcpy(board_status, "RUNNING");
        break;
    case BOARD_WAITING:
        strcpy(board_status, "WAITING");
        break;
    case BOARD_WAITING_ACK:
        strcpy(board_status, "WAITING_ACK");
        break;
    case BOARD_DOWNLOADING:
        strcpy(board_status, "DOWNLOADING");
        break;
    case BOARD_OK :
        strcpy(board_status, "OK");
        break;
    case BOARD_ERR:
        strcpy(board_status, "ERR");
        break;
    default:
        strcpy(board_status, "UNKNOWN");
        break;
    }

    if(true == board.applicationisrunning){
        strcpy(board_process, "canApplication");
    } else {
        strcpy(board_process, "canBootloader");
    }

    strncpy  (board_add_info, board.add_info,32);

    if(-1 == board.appl_vers_build){
        sprintf (board_firmware_version, "%d.0x%x", board.appl_vers_major, board.appl_vers_minor);
    } else {
        sprintf (board_firmware_version, "%d.0x%x.%d", board.appl_vers_major, board.appl_vers_minor, board.appl_vers_build);
    }

    sprintf (board_appl_minor,"%d",board.appl_vers_minor);
    sprintf (board_appl_build,"%d",board.appl_vers_build);
    sprintf (board_serial,"%s",board.serial);

    if((0 == board.prot_vers_major) && (0 == board.prot_vers_minor))
    {
        snprintf (board_protocol, sizeof(board_protocol), "N/A");
    }
    else
    {
        snprintf (board_protocol, sizeof(board_protocol), "%d.%d", board.prot_vers_major, board.prot_vers_minor);
    }

    QStringList myFields;
    myFields.append("");
    myFields.append(board_type);
    myFields.append(QString("%1").arg(board.pid));
    myFields.append(QString("CAN_%1").arg(board.bus));
    myFields.append(board_process);
    myFields.append(board_firmware_version);
    myFields.append(board_add_info);


    for(int i=0;i<myFields.count();i++) {
        setText(i,myFields.at(i));
    }

    setData(0,INDEX_OF_BOARD,m_indexOfBoard);
    setData(0,DEVICE_LEVEL,parentNode->type() == ETH_TREE_NODE ? 3 : 2);
    setData(0,EMPTY_NODE,false);
    setData(0,CAN_TYPE,board.type);

    if(isCheckSelected() != board.selected){
        setCheckSelected(board.selected);
        selectedChanged(board.selected);
    }



}

