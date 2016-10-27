#include "mainwindow.h"
#include "changeinfodialog.h"
#include "changeaddressdialog.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <QCheckBox>
#include <QtConcurrent/QtConcurrent>
#include <QFileDialog>



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
        QTreeWidgetItem *item = new QTreeWidgetItem(QStringList() << "" << devices.at(i).first << devices.at(i).second.toString());
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

    connect(this,SIGNAL(canBoardsRetrieved(QTreeWidgetItem*,QList<sBoard>, bool )),
            this,SLOT(onCanBoardsRetrieved(QTreeWidgetItem*,QList<sBoard>, bool)),Qt::QueuedConnection);

    connect(this,SIGNAL(refreshEthBoardsNode(QTreeWidgetItem*,bool,bool)),
            this,SLOT(populateEthBoardsNode(QTreeWidgetItem*,bool,bool)),Qt::QueuedConnection);

    connect(this,SIGNAL(setInfoRes(QString)),
            this,SLOT(onSetInfoRes(QString)),Qt::QueuedConnection);

    connect(this,SIGNAL(needSetRestartOnSelected()),
            this,SLOT(onNeedSetRestartOnSelected()),Qt::QueuedConnection);

    connect(this,SIGNAL(needLoading(bool,bool)),
                this,SLOT(loading(bool,bool)),Qt::QueuedConnection);

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
        if(selectedNodes.first()->boardIsEth()){
            QtConcurrent::run(this,&MainWindow::uploadEthApplication,filename);
        }else{
            QtConcurrent::run(this,&MainWindow::uploadCanApplication,filename,selectedNodes.first()->getTreeNode()->parent()->text(ADDRESS));
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

void MainWindow::uploadCanApplication(QString filename,QString ethAddress)
{
    QString result;
    core->uploadCanApplication(filename,&result,ethAddress);
    setInfoRes(result);

    if(!ethAddress.isEmpty()){
        refreshCanBoardsFromEth(selectedNodes.first()->getTreeNode()->parent());
    }
}

void MainWindow::onNeedSetRestartOnSelected()
{
    foreach (SelectionCheckBox *c, selectedNodes) {
        setNodeRestartNeed(c->getTreeNode());
    }
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
    core->jumpToUpdater();

    QtConcurrent::run(this,&MainWindow::refreshDevices,true,3000);
}

void MainWindow::onGoToMaintenance(bool click)
{
    core->goToMaintenance();
    QtConcurrent::run(this,&MainWindow::refreshDevices,true,0);
}

void MainWindow::onGoToApplication(bool click)
{
    bool ret = core->goToApplication();
    QtConcurrent::run(this,&MainWindow::refreshDevices,true,0);
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
    
    SelectionCheckBox *check = selectedNodes.first();
    QString oldAddress;
    if(check->boardIsEth()){
        oldAddress = core->getEthBoardAddress(check->getBoardIndex());
    } else {
        oldAddress = check->getTreeNode()->text(ID);
    }
    
    ChangeAddressDialog dlg;
    dlg.setOldAddress(oldAddress);
    if(dlg.exec() == QDialog::Accepted){
        if(dlg.getNewAddress() != oldAddress){
            if(check->boardIsEth()){
                QtConcurrent::run(this,&MainWindow::setEthBoardAddress,
                                  check->getBoardIndex(),
                                  dlg.getNewAddress(),
                                  check->getTreeNode());
            }else{
                QString address;
                if(check->getTreeNode()->data(0,DEVICE_LEVEL).toInt() == 3){
                    address = check->getTreeNode()->parent()->text(ADDRESS);
                    for(int i=0;i<check->getTreeNode()->childCount();i++){
                        check->getTreeNode()->child(i)->setDisabled(true);
                    }
                    int bus = check->getTreeNode()->text(ADDRESS).remove("CAN_").toInt();
                    int id  = check->getTreeNode()->text(ID).toInt();
                    int canType = check->getTreeNode()->data(0,CAN_TYPE).toInt();
                    QPair <QString,QString> addr;
                    addr.second = dlg.getNewAddress();
                    addr.first = address;
                    QtConcurrent::run(this,&MainWindow::setCanBoardAddress,
                                      bus,
                                      id,
                                      canType,
                                      addr,
                                      check->getTreeNode()->parent());



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

    SelectionCheckBox *check = selectedNodes.first();
    QString oldInfo;
    if(check->boardIsEth()){
        oldInfo = core->getEthBoardInfo(check->getBoardIndex());
    } else {
        oldInfo = check->getTreeNode()->text(INFO);   
    }
    ChangeInfoDialog dlg;
    dlg.setOldInfo(oldInfo);
    if(dlg.exec() == QDialog::Accepted){
        if(dlg.getNewInfo() != oldInfo){
            if(check->boardIsEth()){
                QtConcurrent::run(this,&MainWindow::setEthBoardInfo,check->getBoardIndex(),dlg.getNewInfo(),check->getTreeNode());
            }else{
                QString address;
                if(check->getTreeNode()->data(0,DEVICE_LEVEL).toInt() == 3){
                    address = check->getTreeNode()->parent()->text(ADDRESS);
                    for(int i=0;i<check->getTreeNode()->childCount();i++){
                        check->getTreeNode()->child(i)->setDisabled(true);
                    }
                    QtConcurrent::run(this,&MainWindow::setCanBoardInfo,
                                      check->getTreeNode()->text(ADDRESS).remove("CAN_").toInt(),
                                      check->getTreeNode()->text(ID).toInt(),
                                      dlg.getNewInfo(),
                                      address,
                                      check->getTreeNode()->parent());

                }



            }
        }
    }

}

void MainWindow::onCalibrate(bool click)
{

}

void MainWindow::setCanBoardInfo(int bus, int id, QString info, QString ethAddress, QTreeWidgetItem *refreshNode)
{
    loading(true);
    QString result;
    core->setCanBoardInfo(bus,id,info,ethAddress,&result);
    QList <sBoard> canBoards = core->getCanBoardsFromEth(refreshNode->text(ADDRESS),&result);
    canBoardsRetrieved(refreshNode,canBoards, true);
    loading(false);
    setInfoRes(result);
}

void MainWindow::setCanBoardAddress(int bus, int id, int canType, QPair<QString, QString> addr, QTreeWidgetItem *refreshNode)
{
    loading(true);
    QString result;
    core->setCanBoardAddress(bus,id,canType,addr.second,addr.first,&result);
    QList <sBoard> canBoards = core->getCanBoardsFromEth(refreshNode->text(ADDRESS),&result);
    canBoardsRetrieved(refreshNode,canBoards, false);
    loading(false);
    setInfoRes(result);
}

void MainWindow::setEthBoardInfo(int index,QString info, QTreeWidgetItem *refreshNode)
{
    loading(true);
    core->setEthBoardInfo(index,info);
    refreshEthBoardsNode(refreshNode,true);
    loading(false);
}

void MainWindow::setEthBoardAddress(int index,QString address, QTreeWidgetItem *refreshNode)
{
    loading(true);
    if(core->setEthBoardAddress(index,address)){
        qDebug() << "SUCCESS";
        //refreshEthBoardsNode(refreshNode,true);
        //refreshNode->setIcon(ADDRESS,QIcon(":/images/restart-needed.png"));
        //setNodeRestartNeed(refreshNode);
        refreshEthBoardsNode(refreshNode,true);
        checkEnableButtons();
    }else{
        qDebug() << "FAILED";
        //TODO ERROR
    }
    loading(false);
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
    if(it->parent() == NULL){
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
    foreach (QTreeWidgetItem *it, ui->devicesTree->selectedItems()) {
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

        checkConnectionButton(it);
    }
    refreshDevices();

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

            SelectionCheckBox *check = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j),0);
            if(check && check->isEnabled() && check->isSelected()){
                if(check->getTreeNode()->data(0,DEVICE_LEVEL).toInt() == 2){
                    if(check->getTreeNode()->parent()->text(DEVICE).contains("ETH")){
                        isEth = true;
                    }
                    if(check->getTreeNode()->data(0,REFRESH_NEED).toBool()){
                        needRestart = true;
                    }
                    canUploadLoader &= check->getTreeNode()->data(0,CAN_UPLOAD_LOADER).toBool();
                    canUploadApp &= check->getTreeNode()->data(0,CAN_UPLOAD_APP).toBool();
                    canUploadUpdater &= check->getTreeNode()->data(0,CAN_UPLOAD_UPDATER).toBool();
                    canJumpUpdater &= check->getTreeNode()->data(0,CAN_JUMP_UPDATER).toBool();
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
            int canType = selectedNodes.first()->getTreeNode()->data(0,CAN_TYPE).toInt();
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

void MainWindow::refreshDevices(bool refresh, int msecs)
{
    loading(true,refresh);
    for (int i = 0; i<ui->devicesTree->topLevelItemCount(); i++) {
        QTreeWidgetItem *it = ui->devicesTree->topLevelItem(i);
        if(it->text(DEVICE).contains("ETH")){
            if(refresh){
                //core->disconnectFrom(it->text(DEVICE),it->text(ID));
//                it->setData(0,CONNECTED,false);
//                it->setTextColor(DEVICE,QColor(Qt::red));
//                qDebug() << "WAITING " << msecs << " msecs";
//                QThread::msleep(msecs);

//                int found = core->connectTo(it->text(DEVICE),it->text(ID));
//                if(found > 0){
//                    it->setData(0,CONNECTED,true);
//                    it->setExpanded(true);
//                    it->setTextColor(DEVICE,QColor(Qt::green));
//                    if(found != it->childCount()){
//                        refreshEthBoardsNode(it);
//                    }else{
//                        refreshEthBoardsNode(it,false,refresh);
//                    }

//                }else{
//                    it->setData(0,CONNECTED,false);
//                    it->setTextColor(DEVICE,QColor(Qt::red));
//                    emptyNode(it);
//                }
                refreshEthBoardsNode(it,false,refresh);

                deviceSelectionChanged();
            }else{
                refreshEthBoardsNode(it);
            }



        }
    }
    loading(false);
}

void MainWindow::removeChildren(QTreeWidgetItem *it)
{
    for(int i=it->childCount() - 1; i >= 0; i--){

        SelectionCheckBox *c = ((SelectionCheckBox*)ui->devicesTree->itemWidget(it->child(i),0));
        if(c){
            if(c->isSelected()){
                c->setSelected(false);
            }

            selectedNodes.removeOne(c);
        }
        QTreeWidgetItem *child = it->takeChild(i);
        delete child;
    }
}


void MainWindow::populateEthBoardsNode(QTreeWidgetItem *it, bool refreshSingleNode, bool refreshAll)
{
    //loading(true);
    char board_ipaddr[16];
    char board_mac[32];

    char board_version[16];
    char board_date[24];
    char board_built[24];
    char board_type[24];
    char running_process[24];
    char board_info[32];

    QTreeWidgetItem *parentNode = it->parent();


    if(!refreshAll && !refreshSingleNode && core->getEthBoardList().size() > 0){
        removeChildren(it);
    }

    QList <int> notFoundIndexes;
    for (int i=0; i<core->getEthBoardList().size(); ++i){
        notFoundIndexes.append(i);
    }

    QList <QTreeWidgetItem*> notRemoveNodes;
    for (int i=0; i<core->getEthBoardList().size(); ++i)
    {
        EthBoard board = core->getEthBoardList()[i];
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
                foreach (SelectionCheckBox *s, selectedNodes) {
                    if(s->getTreeNode() == c || s->getTreeNode() == it){
                        selectedNodes.removeOne(s);
                        s->setSelected(false);
                    }
                }
            }
            it->parent()->removeChild(it);
        }

        memset(board_ipaddr,0,sizeof(board_ipaddr));
        memset(board_mac,0,sizeof(board_mac));
        memset(board_version,0,sizeof(board_version));
        memset(board_date,0,sizeof(board_date));
        memset(board_built,0,sizeof(board_built));
        memset(board_type,0,sizeof(board_type));
        memset(running_process,0,sizeof(running_process));
        memset(board_info,0,sizeof(board_info));

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

        /*if(refreshSingleNode){
            for(int j=0;j<it->parent()->childCount();j++){
                QTreeWidgetItem *child = it->parent()->child(j);
                if(child->text(ADDRESS) == QString("%1").arg(board_ipaddr)){
                    child->setText(INFO,board_info);
                    child->setText(VERSION,board_version);
                    child->setText(PROCESS,running_process);
                    child->setData(0,INDEX_OF_BOARD,i);
                    if(child->data(0,REFRESH_NEED).toBool()){
                        setNodeRestartNeed(child,false);
                    }
                    notRemoveNodes.append(child);
                    notFoundIndexes.removeOne(i);
                    if(QString("%1").arg(running_process).contains("eUpdater")){
                        child->setData(0,CAN_UPLOAD_LOADER,true);
                        child->setData(0,CAN_UPLOAD_APP,true);
                        child->setData(0,CAN_UPLOAD_UPDATER,false);
                        child->setData(0,CAN_JUMP_UPDATER,false);
                    } else if(QString("%1").arg(running_process).contains("eApplPROGupdater")){
                        child->setData(0,CAN_UPLOAD_LOADER,true);
                        child->setData(0,CAN_UPLOAD_APP,false);
                        child->setData(0,CAN_UPLOAD_UPDATER,true);
                        child->setData(0,CAN_JUMP_UPDATER,true);
                    }else{
                        child->setData(0,CAN_UPLOAD_LOADER,false);
                        child->setData(0,CAN_UPLOAD_APP,false);
                        child->setData(0,CAN_UPLOAD_UPDATER,false);
                        child->setData(0,CAN_JUMP_UPDATER,false);
                    }
                }
                if(child->isExpanded()){
                    onDeviceExpanded(child);
                }
            }
        }else */if(refreshAll){
            for(int j=0;j<it->childCount();j++){
                QTreeWidgetItem *child = it->child(j);
                if(child->text(ADDRESS) == QString("%1").arg(board_ipaddr)){
                    child->setText(INFO,board_info);
                    child->setText(VERSION,board_version);
                    child->setText(PROCESS,running_process);
                    child->setData(0,INDEX_OF_BOARD,i);
                    if(child->data(0,REFRESH_NEED).toBool()){
                        setNodeRestartNeed(child,false);
                    }
                    notRemoveNodes.append(child);
                    notFoundIndexes.removeOne(i);
                    if(QString("%1").arg(running_process).contains("eUpdater")){
                        child->setData(0,CAN_UPLOAD_LOADER,true);
                        child->setData(0,CAN_UPLOAD_APP,true);
                        child->setData(0,CAN_UPLOAD_UPDATER,false);
                        child->setData(0,CAN_JUMP_UPDATER,false);
                    } else if(QString("%1").arg(running_process).contains("eApplPROGupdater")){
                        child->setData(0,CAN_UPLOAD_LOADER,true);
                        child->setData(0,CAN_UPLOAD_APP,false);
                        child->setData(0,CAN_UPLOAD_UPDATER,true);
                        child->setData(0,CAN_JUMP_UPDATER,true);
                    }else{
                        child->setData(0,CAN_UPLOAD_LOADER,false);
                        child->setData(0,CAN_UPLOAD_APP,false);
                        child->setData(0,CAN_UPLOAD_UPDATER,false);
                        child->setData(0,CAN_JUMP_UPDATER,false);
                    }
                    if(selectedNodes.contains((SelectionCheckBox*)ui->devicesTree->itemWidget(child,0))){
                        core->setSelectedEthBoard(child->data(0,INDEX_OF_BOARD).toInt(),true);
                    }
                }
                if(child->isExpanded()){
                    onDeviceExpanded(child);
                }
            }
        }else {

            QStringList fields;
            fields.append("");
            fields.append(board_type);
            fields.append("");
            fields.append(board_ipaddr);
            fields.append(running_process);
            fields.append(board_version);
            fields.append(board_info);

            notFoundIndexes.removeOne(i);


            QTreeWidgetItem *child = new QTreeWidgetItem(!refreshSingleNode ? it : parentNode ,fields);
            child->setData(0,INDEX_OF_BOARD,i);
            child->setData(0,DEVICE_LEVEL,2);
            if(QString("%1").arg(running_process).contains("eUpdater")){
                child->setData(0,CAN_UPLOAD_LOADER,true);
                child->setData(0,CAN_UPLOAD_APP,true);
                child->setData(0,CAN_UPLOAD_UPDATER,false);
                child->setData(0,CAN_JUMP_UPDATER,false);
            } else if(QString("%1").arg(running_process).contains("eApplPROGupdater")){
                child->setData(0,CAN_UPLOAD_LOADER,true);
                child->setData(0,CAN_UPLOAD_APP,false);
                child->setData(0,CAN_UPLOAD_UPDATER,true);
                child->setData(0,CAN_JUMP_UPDATER,true);
            }else{
                child->setData(0,CAN_UPLOAD_LOADER,false);
                child->setData(0,CAN_UPLOAD_APP,false);
                child->setData(0,CAN_UPLOAD_UPDATER,false);
                child->setData(0,CAN_JUMP_UPDATER,false);
            }

            SelectionCheckBox *check = new SelectionCheckBox(core,child);

            connect(check,SIGNAL(selectedChanged(bool)),
                    this,SLOT(onSelectionChanged(bool)),Qt::QueuedConnection);
            connect(check,SIGNAL(destroyed(QObject*)),
                    this,SLOT(onSelectionCheckDestroy(QObject*)));
            connect(check,SIGNAL(needLoading(bool,bool)),
                    this,SLOT(loading(bool,bool)),Qt::DirectConnection);

            ui->devicesTree->setItemWidget(child,0,check);


            QTreeWidgetItem *empty = new QTreeWidgetItem(child,QStringList() << "" << "?");
            empty->setData(0,DEVICE_LEVEL,3);
            empty->setData(0,EMPTY_NODE,true);

            if(selectedNodes.count() > 0){
                if(selectedNodes.first()->getBoardType() != QString("%1").arg(board_type)){
                    check->setEnabled(false);
                }
            }
        }




    }
    //loading(false);


    /*if(notFoundIndexes.count() > 0){

        int count = it->parent()->childCount();
        for (int i=0;i<count;i++) {
            QTreeWidgetItem *c = it->parent()->child(i);
            if(!notRemoveNodes.contains(c)){
                foreach (SelectionCheckBox *s, selectedNodes) {
                    if(s->getTreeNode() == c){
                        selectedNodes.removeOne(s);
                        s->setSelected(false);
                    }

                }
                removeChildren(c);
                it->parent()->removeChild(c);
            }
        }


        foreach (int i, notFoundIndexes) {



            EthBoard board = core->getEthBoardList()[i];

            memset(board_ipaddr,0,sizeof(board_ipaddr));
            memset(board_mac,0,sizeof(board_mac));
            memset(board_version,0,sizeof(board_version));
            memset(board_date,0,sizeof(board_date));
            memset(board_built,0,sizeof(board_built));
            memset(board_type,0,sizeof(board_type));
            memset(running_process,0,sizeof(running_process));
            memset(board_info,0,sizeof(board_info));



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


            QStringList fields;
            fields.append("");
            fields.append(board_type);
            fields.append("");
            fields.append(board_ipaddr);
            fields.append(running_process);
            fields.append(board_version);
            fields.append(board_info);


            QTreeWidgetItem *child = new QTreeWidgetItem(it->parent(),fields);
            child->setData(0,INDEX_OF_BOARD,i);
            child->setData(0,DEVICE_LEVEL,2);
            if(QString("%1").arg(running_process).contains("eUpdater")){
                child->setData(0,CAN_UPLOAD_LOADER,true);
                child->setData(0,CAN_UPLOAD_APP,true);
                child->setData(0,CAN_UPLOAD_UPDATER,false);
                child->setData(0,CAN_JUMP_UPDATER,false);
            } else if(QString("%1").arg(running_process).contains("eApplPROGupdater")){
                child->setData(0,CAN_UPLOAD_LOADER,true);
                child->setData(0,CAN_UPLOAD_APP,false);
                child->setData(0,CAN_UPLOAD_UPDATER,true);
                child->setData(0,CAN_JUMP_UPDATER,true);
            }else{
                child->setData(0,CAN_UPLOAD_LOADER,false);
                child->setData(0,CAN_UPLOAD_APP,false);
                child->setData(0,CAN_UPLOAD_UPDATER,false);
                child->setData(0,CAN_JUMP_UPDATER,false);
            }

            SelectionCheckBox *check = new SelectionCheckBox(core,child);

            connect(check,SIGNAL(selectedChanged(bool)),
                    this,SLOT(onSelectionChanged(bool)),Qt::QueuedConnection);
            connect(check,SIGNAL(destroyed(QObject*)),
                    this,SLOT(onSelectionCheckDestroy(QObject*)));
            connect(check,SIGNAL(needLoading(bool,bool)),
                    this,SLOT(loading(bool,bool)),Qt::DirectConnection);

            ui->devicesTree->setItemWidget(child,0,check);


            QTreeWidgetItem *empty = new QTreeWidgetItem(child,QStringList() << "" << "?");
            empty->setData(0,DEVICE_LEVEL,3);
            empty->setData(0,EMPTY_NODE,true);

            if(selectedNodes.count() > 0){
                if(selectedNodes.first()->getBoardType() != QString("%1").arg(board_type)){
                    check->setEnabled(false);
                }
            }
        }
    }*/
}

void MainWindow::onSelectionCheckDestroy(QObject *obj)
{
    mutex1.lock();
    selectedNodes.removeOne((SelectionCheckBox*)obj);
    mutex1.unlock();
}

void MainWindow::onCanBoardsRetrieved(QTreeWidgetItem *it,QList <sBoard> canBoards, bool refresh)
{


    if(!refresh && canBoards.count() > 0){
        removeChildren(it);
        it->setData(0,EMPTY_NODE,false);
    }

    for(int i=0;i<canBoards.count();i++){
        sBoard board = canBoards.at(i);

        char board_type        [50]; memset (board_type,0,50);
        char board_process     [50]; memset (board_process,0,50);
        char board_status      [50]; memset (board_status,0,50);
        char board_add_info    [50]; memset (board_add_info,0,50);
        char board_firmware_version  [10]; memset (board_firmware_version,0,10);
        char board_appl_minor  [10]; memset (board_appl_minor,0,10);
        char board_appl_build  [10]; memset (board_appl_build,0,10);
        char board_serial      [10]; memset (board_serial,0,10);
        char board_protocol    [10]; memset (board_protocol,0,10);

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


        if(refresh){
            for(int j=0;j<it->childCount();j++){
                QTreeWidgetItem *child = it->child(j);
                if(child->text(DEVICE) == QString("%1").arg(board_type) &&
                   child->text(ID) == QString("%1").arg(board.pid) &&
                   child->text(ADDRESS) == QString("CAN_%1").arg(board.bus)){
                    child->setText(DEVICE,board_type);
                    child->setText(ID,QString("%1").arg(board.pid));
                    child->setText(ADDRESS,QString("CAN_%1").arg(board.bus));
                    child->setText(VERSION,board_firmware_version);
                    child->setText(INFO,board_add_info);
                    child->setText(PROCESS,board_process);
                }
            }
        }else{
            QStringList fields;
            fields.append("");
            fields.append(board_type);
            fields.append(QString("%1").arg(board.pid));
            fields.append(QString("CAN_%1").arg(board.bus));
            fields.append(board_process);
            fields.append(board_firmware_version);
            fields.append(board_add_info);

            QTreeWidgetItem *child = new QTreeWidgetItem(it,fields);
            child->setData(0,INDEX_OF_BOARD,i);
            child->setData(0,DEVICE_LEVEL,3);
            child->setData(0,EMPTY_NODE,false);
            child->setData(0,CAN_TYPE,board.type);

            SelectionCheckBox *check = new SelectionCheckBox(core,child);

            connect(check,SIGNAL(selectedChanged(bool)),
                    this,SLOT(onSelectionChanged(bool)),Qt::QueuedConnection);
            connect(check,SIGNAL(destroyed(QObject*)),
                    this,SLOT(onSelectionCheckDestroy(QObject*)),Qt::DirectConnection);
            connect(check,SIGNAL(needLoading(bool,bool)),
                    this,SLOT(loading(bool,bool)),Qt::DirectConnection);

            ui->devicesTree->setItemWidget(child,0,check);

            if(selectedNodes.count() > 0){
                if(selectedNodes.first()->getBoardType() != QString("%1").arg(board_type)){
                    check->setEnabled(false);
                }
            }
        }
    }

    checkEnableButtons();

    needLoading(false,false);
}

void MainWindow::getCanBoards(QTreeWidgetItem *it, bool refresh)
{
    needLoading(true,false);
    QString result;
    QList <sBoard> canBoards = core->getCanBoardsFromEth(it->text(ADDRESS),&result);
    //loading(false);

    canBoardsRetrieved(it,canBoards, refresh);
    setInfoRes(result);

}

void MainWindow::onDeviceExpanded(QTreeWidgetItem *it)
{
    if(it->data(0,DEVICE_LEVEL).toInt() != 2){
        return;
    }

//    if(it->data(0,EMPTY_NODE).toBool() == false){
//        return;
//    }

    if(!it->parent()->text(DEVICE).contains("ETH")){
        return;
    }

    if(it->data(0,REFRESH_NEED).toBool()){
        return;
    }

    for(int i=0;i<it->childCount();i++){
        it->child(i)->setDisabled(true);
    }
    QtConcurrent::run(this,&MainWindow::getCanBoards,it,false);
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

void MainWindow::loading(bool load, bool disableAll)
{

//    if(load/* && !isLoading*/){
//        loadingMutex.lock();
//        isLoading = true;
//        progress->setMaximum(0);
//        progress->setEnabled(true);
//        ui->advancedGroup->setEnabled(false);
//        ui->controlsGroup->setEnabled(false);
//        infoResult->setVisible(true);
//        if(disableAll){
//            ui->devicesTree->setEnabled(false);
//        }
//        infoResult->setText("Loading...");
//    }else /*if(!load && isLoading)*/{
//        progress->setMaximum(100);
//        progress->setEnabled(false);
//        ui->advancedGroup->setEnabled(true);
//        ui->controlsGroup->setEnabled(true);
//        if(!ui->devicesTree->isEnabled()){
//            ui->devicesTree->setEnabled(true);
//        }
//        infoResult->setVisible(true);
//        infoResult->setText("Operation Done");
//        isLoading = false;
//        loadingMutex.unlock();
//    }
}



void MainWindow::onSelectionChanged(bool selected)
{
    loading(true);
    SelectionCheckBox *c = (SelectionCheckBox*)sender();




    if(selected){
        selectedNodes.append(c);

        for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
            QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
            for(int j=0;j<topLevel->childCount();j++){

                SelectionCheckBox *check = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j),0);
                if(check && check->isEnabled()){
                    if(check->getBoardType() != c->getBoardType() || c->getTreeNode()->parent() != check->getTreeNode()->parent()){
                        check->setEnabled(false);
                    }
                }

                for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                    SelectionCheckBox *check1 = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j)->child(k),0);
                    if(check1 && check1->isEnabled()){
                        if(check1->getBoardType() != c->getBoardType() || c->getTreeNode()->parent() != check1->getTreeNode()->parent()){
                            check1->setEnabled(false);
                        }
                    }
                }
            }

        }
    }else{
        selectedNodes.removeOne(c);
        bool found = false;
        for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
            QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
            for(int j=0;j<topLevel->childCount();j++){
                SelectionCheckBox *check = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j),0);
                if((check && check->isSelected()) || found){
                    found = true;
                    break;
                }

                for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                    SelectionCheckBox *check1 = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j)->child(k),0);
                    if(check1 && check1->isSelected()){
                        found = true;
                        break;
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
//                    if(topLevel->child(j)->data(0,REFRESH_NEED) == true){
//                        continue;
//                    }
                    SelectionCheckBox *check = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j),0);
                    if(check && !topLevel->child(j)->isDisabled()){
                        check->setEnabled(true);
                    }
                    for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                        if(topLevel->child(j)->data(0,REFRESH_NEED) == true){
                            continue;
                        }
                        SelectionCheckBox *check1 = (SelectionCheckBox*)ui->devicesTree->itemWidget(topLevel->child(j)->child(k),0);
                        if(check1 && !topLevel->child(j)->child(k)->isDisabled()){
                            check1->setEnabled(true);
                        }
                    }
                }

            }
        }

    }
    loading(false);
    checkEnableButtons();
}
