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

    connect(this,SIGNAL(canBoardsRetrieved(QTreeWidgetItem*,QList<sBoard>, bool )),
            this,SLOT(onCanBoardsRetrieved(QTreeWidgetItem*,QList<sBoard>, bool)),Qt::QueuedConnection);

    connect(this,SIGNAL(refreshEthBoardsNode(QTreeWidgetItem*,bool,bool)),
            this,SLOT(populateEthBoardsNode(QTreeWidgetItem*,bool,bool)),Qt::QueuedConnection);

    connect(this,SIGNAL(setInfoRes(QString)),
            this,SLOT(onSetInfoRes(QString)),Qt::QueuedConnection);

    connect(this,SIGNAL(needSetRestartOnSelected()),
            this,SLOT(onNeedSetRestartOnSelected()),Qt::QueuedConnection);

    connect(ui->btnBlink,SIGNAL(clicked(bool)),this,SLOT(onBlinkPressed(bool)));
    connect(ui->btnCahngeInfo,SIGNAL(clicked(bool)),this,SLOT(onChangeInfo(bool)));
    connect(ui->btnChangeIp,SIGNAL(clicked(bool)),this,SLOT(onChangeAddress(bool)));
    connect(ui->btnChangeCanAddr,SIGNAL(clicked(bool)),this,SLOT(onChangeAddress(bool)));
    connect(ui->btnRestart,SIGNAL(clicked(bool)),this,SLOT(onRestartBoards(bool)));
    connect(ui->btnRestartSecs,SIGNAL(clicked(bool)),this,SLOT(onRestartBoards5Secs(bool)));
    connect(ui->btnCalibrate,SIGNAL(clicked(bool)),this,SLOT(onCalibrate(bool)));
    connect(ui->btnBootApp,SIGNAL(clicked(bool)),this,SLOT(onBootApplication(bool)));
    connect(ui->btnBootUpdater,SIGNAL(clicked(bool)),this,SLOT(onBootUpdater(bool)));
    connect(ui->btnUploadApp,SIGNAL(clicked(bool)),this,SLOT(onUploadApplication(bool)));
    connect(ui->btnUploadLoader,SIGNAL(clicked(bool)),this,SLOT(onUploadLoader(bool)));
    connect(ui->btnUploadUpdater,SIGNAL(clicked(bool)),this,SLOT(onUploadUpdater(bool)));
    connect(ui->btnJumpUpdater,SIGNAL(clicked(bool)),this,SLOT(onJumpToUpdater(bool)));
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

    QtConcurrent::run(this,&MainWindow::refreshDevices,true,8000);
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
        qDebug() << "SECCESS";
        //refreshEthBoardsNode(refreshNode,true);
        //refreshNode->setIcon(ADDRESS,QIcon(":/images/restart-needed.png"));
        setNodeRestartNeed(refreshNode);
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
    QtConcurrent::run(this,&MainWindow::refreshDevices,true,8000);
}

void MainWindow::onRestartBoards5Secs(bool click)
{
    core->restartEthBoards();
    QtConcurrent::run(this,&MainWindow::refreshDevices,true,3000);
}

void MainWindow::checkConnectionButton(QTreeWidgetItem *it)
{
    if(it->parent() == NULL){
        ui->connectButton->setEnabled(true);
        if(it->data(0,CONNECTED).toBool() == true){
            ui->connectButton->setText("Disconnect");
        }else{
            ui->connectButton->setText("Connect");
        }

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
            ui->detailsText->clear();
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
            ui->detailsText->clear();
        }
    }else{
        ui->connectButton->setEnabled(false);
        ui->detailsText->clear();
    }
    checkEnableButtons();

}

void MainWindow::populateInfo(int boardNum)
{
    loading(true);
    QString details = core->getMoreDetails((core->getEthBoardList())[boardNum].mAddress);
    appendInfo(details);
    loading(false);
}

void MainWindow::onAppendInfo(QString text)
{

    ui->detailsText->clear();
    if(!text.isEmpty()){
        ui->detailsText->appendPlainText(text);
    }
}

void MainWindow::onConnect()
{
    foreach (QTreeWidgetItem *it, ui->devicesTree->selectedItems()) {
        if(it->data(0,CONNECTED).toBool() == false){
            if(core->connectTo(it->text(DEVICE),it->text(ID)) > 0){
                it->setData(0,CONNECTED,true);
                it->setExpanded(true);
                it->setTextColor(DEVICE,QColor(Qt::green));
            }else{
                it->setData(0,CONNECTED,false);
                it->setTextColor(DEVICE,QColor(Qt::red));
            }
        }else{
            emptyNode(it);
            core->disconnectFrom(it->text(DEVICE),it->text(ID));
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
        ui->btnRestartSecs->setEnabled(false);
        ui->btnUploadApp->setEnabled(false);
        ui->btnUploadLoader->setEnabled(false);
        ui->btnUploadUpdater->setEnabled(false);
        return;
    }


    if(isEth){
        if(needRestart){
            ui->btnRestart->setEnabled(true);
            ui->btnRestartSecs->setEnabled(true);
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
            ui->btnRestart->setEnabled(true);
            ui->btnRestartSecs->setEnabled(true);
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
        ui->btnRestartSecs->setEnabled(false);
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
                core->disconnectFrom(it->text(DEVICE),it->text(ID));
                it->setData(0,CONNECTED,false);
                it->setTextColor(DEVICE,QColor(Qt::red));
                qDebug() << "WAITING " << msecs << " msecs";
                QThread::msleep(msecs);

                int found = core->connectTo(it->text(DEVICE),it->text(ID));
                if(found > 0){
                    it->setData(0,CONNECTED,true);
                    it->setExpanded(true);
                    it->setTextColor(DEVICE,QColor(Qt::green));
                    if(found != it->childCount()){
                        refreshEthBoardsNode(it);
                    }else{
                        refreshEthBoardsNode(it,false,refresh);
                    }

                }else{
                    it->setData(0,CONNECTED,false);
                    it->setTextColor(DEVICE,QColor(Qt::red));
                    emptyNode(it);
                }

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
        ACE_UINT32 ip=core->getEthBoardList()[i].mAddress;
        sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);

        ACE_UINT64 mac=core->getEthBoardList()[i].mMac;
        //ACE_UINT32 macL=(ACE_UINT32)(mac&0xFFFFFFFF);
        //ACE_UINT32 macH=(ACE_UINT32)((mac>>32)&0xFFFF);

        //sprintf(board_mac,"%04X%08X",macH,macL);

        snprintf(board_mac, sizeof(board_mac), "%02X-%02X-%02X-%02X-%02X-%02X",
                            (uint8_t)(mac >> 40) & 0xff,
                            (uint8_t)(mac >> 32) & 0xff,
                            (uint8_t)(mac >> 24) & 0xff,
                            (uint8_t)(mac >> 16) & 0xff,
                            (uint8_t)(mac >> 8 ) & 0xff,
                            (uint8_t)(mac      ) & 0xff
                );

        snprintf(board_version, sizeof(board_version), "%d.%d", core->getEthBoardList()[i].mVersionMajor, core->getEthBoardList()[i].mVersionMinor);
        snprintf(board_type, sizeof(board_type), "%s", core->getEthBoardList()[i].mBoardType.c_str());
        snprintf(running_process, sizeof(running_process), "%s", core->getEthBoardList()[i].mRunningProcess.c_str());
        snprintf(board_info, sizeof(board_info), "%s", core->getEthBoardList()[i].mInfo32.c_str());
        snprintf(board_date, sizeof(board_date), "%s", core->getEthBoardList()[i].mReleasedOn.c_str());
        snprintf(board_built, sizeof(board_date), "%s", core->getEthBoardList()[i].mBuiltOn.c_str());


        if(refreshSingleNode){
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
        }else if(refreshAll){
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


            QTreeWidgetItem *child = new QTreeWidgetItem(it,fields);
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


    if(notFoundIndexes.count() > 0){
        for (int i=0;i<it->childCount();i++) {
            QTreeWidgetItem *c = it->child(i);
            if(!notRemoveNodes.contains(c)){
                foreach (SelectionCheckBox *s, selectedNodes) {
                    if(s->getTreeNode() == c){
                        selectedNodes.removeOne(s);
                        s->setSelected(false);
                    }

                }
                removeChildren(c);
                delete c;
            }
        }

        foreach (int i, notFoundIndexes) {
            ACE_UINT32 ip=core->getEthBoardList()[i].mAddress;
            sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);

            ACE_UINT64 mac=core->getEthBoardList()[i].mMac;
            //ACE_UINT32 macL=(ACE_UINT32)(mac&0xFFFFFFFF);
            //ACE_UINT32 macH=(ACE_UINT32)((mac>>32)&0xFFFF);

            //sprintf(board_mac,"%04X%08X",macH,macL);

            snprintf(board_mac, sizeof(board_mac), "%02X-%02X-%02X-%02X-%02X-%02X",
                                (uint8_t)(mac >> 40) & 0xff,
                                (uint8_t)(mac >> 32) & 0xff,
                                (uint8_t)(mac >> 24) & 0xff,
                                (uint8_t)(mac >> 16) & 0xff,
                                (uint8_t)(mac >> 8 ) & 0xff,
                                (uint8_t)(mac      ) & 0xff
                    );

            snprintf(board_version, sizeof(board_version), "%d.%d", core->getEthBoardList()[i].mVersionMajor, core->getEthBoardList()[i].mVersionMinor);
            snprintf(board_type, sizeof(board_type), "%s", core->getEthBoardList()[i].mBoardType.c_str());
            snprintf(running_process, sizeof(running_process), "%s", core->getEthBoardList()[i].mRunningProcess.c_str());
            snprintf(board_info, sizeof(board_info), "%s", core->getEthBoardList()[i].mInfo32.c_str());
            snprintf(board_date, sizeof(board_date), "%s", core->getEthBoardList()[i].mReleasedOn.c_str());
            snprintf(board_built, sizeof(board_date), "%s", core->getEthBoardList()[i].mBuiltOn.c_str());

            QStringList fields;
            fields.append("");
            fields.append(board_type);
            fields.append("");
            fields.append(board_ipaddr);
            fields.append(running_process);
            fields.append(board_version);
            fields.append(board_info);


            QTreeWidgetItem *child = new QTreeWidgetItem(it,fields);
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
}

void MainWindow::getCanBoards(QTreeWidgetItem *it, bool refresh)
{
    loading(true);
    QString result;
    QList <sBoard> canBoards = core->getCanBoardsFromEth(it->text(ADDRESS),&result);
    loading(false);
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
    if(load/* && !isLoading*/){
        loadingMutex.lock();
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
        progress->setMaximum(100);
        progress->setEnabled(false);
        ui->advancedGroup->setEnabled(true);
        ui->controlsGroup->setEnabled(true);
        if(!ui->devicesTree->isEnabled()){
            ui->devicesTree->setEnabled(true);
        }
        infoResult->setVisible(true);
        infoResult->setText("Operation Done");
        isLoading = false;
        loadingMutex.unlock();
    }
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
