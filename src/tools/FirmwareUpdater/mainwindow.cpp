#include "mainwindow.h"
#include "changeinfodialog.h"
#include "changeaddressdialog.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <QCheckBox>
#include <QtConcurrent/QtConcurrent>
#include <QFileDialog>
#include <QMessageBox>
#include <QToolButton>
#include "straincalibgui.h"

MainWindow::MainWindow(FirmwareUpdaterCore *core, bool adminMode, QWidget *parent) :
    QMainWindow(parent), /*mutex(QMutex::Recursive)*/
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //core->strainCalibMode = strainCalibMode;
    qRegisterMetaType <QList<sBoard> > ("QList<sBoard>");
    qRegisterMetaType <QVector<int> > ("QVector<int>");
    qRegisterMetaType <boardInfo2_t> ("boardInfo2_t");
    qRegisterMetaType <eOipv4addr_t> ("eOipv4addr_t");
    qRegisterMetaType <sBoard> ("sBoard");
    sgboardtype = icubCanProto_boardType__strain;
    isLoading = false;
    loadCounter = 0;
    calibDlg = NULL;


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
//    ui->splitter->setStretchFactor(0,80);
//    ui->splitter->setStretchFactor(0,20);

    infoTreeWidget = new QTreeWidget(ui->groupBox_2);
    infoTreeWidget->setVisible(false);
    ui->groupBox_2->layout()->addWidget(infoTreeWidget);
    ui->groupBox_2->setTitle("Board Properties");
    infoTreeWidget->setColumnCount(2);
    infoTreeWidget->setHeaderLabels(QStringList() << "Name" << "Value");
    infoTreeWidget->addTopLevelItem(new QTreeWidgetItem(infoTreeWidget,QStringList() << "Board"));
    infoTreeWidget->addTopLevelItem(new QTreeWidgetItem(infoTreeWidget,QStringList() << "Bootstrap Processes"));
    infoTreeWidget->addTopLevelItem(new QTreeWidgetItem(infoTreeWidget,QStringList() << "Properties of the Processes"));


    ui->checkBoxEE->setCheckState(Qt::Unchecked); //(Qt::Checked); Qt::Unchecked
    ui->checkBoxEE->setEnabled(false);
    //Qt::CheckState eestate = ui->checkBoxEE->checkState();


    ui->actionSel->setIcon(QIcon(":/images/sel-bw.png"));
    ui->actionDes->setIcon(QIcon(":/images/des-bw.png"));

    //ui->actionSel->setText("SS");

    connect(ui->actionSel, SIGNAL(triggered()), this, SLOT(onSel()));
    connect(ui->actionDes, SIGNAL(triggered()), this, SLOT(onDes()));


    this->core = core;

    QList < QPair<QString,QVariant> > devices = core->getDevices();

    //ui->devicesTree->setSelectionMode(QTreeWidget::MultiSelection);
    for(int i=0;i<devices.count();i++){
        int type = devices.at(i).first.contains("ETH") ? ETH_TREE_ROOT_NODE : CAN_TREE_ROOT_NODE;

        QString dev = devices.at(i).first + "<" + devices.at(i).second.toString() + ">";
        //CustomTreeWidgetItem *item = new CustomTreeWidgetItem(NULL,QStringList() << "" << devices.at(i).first << devices.at(i).second.toString(),-1,core,type);
        CustomTreeWidgetItem *item = new CustomTreeWidgetItem(NULL,QStringList() << "" << dev << "",-1,core,type);
        ui->devicesTree->addTopLevelItem(item);

        QTreeWidgetItem *empty = new QTreeWidgetItem(item,QStringList() << "" << "" << "?");
        empty->setData(0,EMPTY_NODE,true);

        item->setTextColor(DEVICEID,QColor(Qt::red));
    }
    if(!adminMode){
        ui->advancedGroup->setVisible(false);
        ui->strainGroup->setVisible(false);
    }

//    if(!strainCalibMode){

//        ui->btnStrainCalib->setVisible(false);
//    }else{
//        ui->strainGroup->setVisible(false);
//        ui->advancedGroup->setVisible(false);
//        ui->btnCahngeInfo->setVisible(false);
//        ui->btnUploadApp->setVisible(false);
//    }

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

    connect(this,SIGNAL(appendInfo(sBoard)),
            this,SLOT(onAppendInfo(sBoard)),Qt::QueuedConnection);


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

    connect(this,SIGNAL(needLoading(bool,bool,QString,bool)),
                this,SLOT(loading(bool,bool,QString,bool)),Qt::QueuedConnection);

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
    connect(ui->btnStrainCalib,SIGNAL(clicked(bool)),this,SLOT(onStrainCalib(bool)));
    connect(&watcher, SIGNAL(finished()), this, SLOT(onFutureFinished()));

    ui->devicesTree->sortItems(ADDRESS,Qt::AscendingOrder);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onSel(void)
{

    if(selectedNodes.isEmpty())
    {
        return;
    }

#if 0
    test che mette tutti a sel
    for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
        QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
        for(int j=0;j<topLevel->childCount();j++){

            QTreeWidgetItem *widgItem = topLevel->child(j);

            if((widgItem->type() != ETH_TREE_ROOT_NODE)  && (widgItem->type() != CAN_TREE_ROOT_NODE) && (widgItem->type() != QTreeWidgetItem::Type))
            {
                CustomTreeWidgetItem *child = (CustomTreeWidgetItem *)widgItem;
                if(!child->isCheckSelected()){
                    if(child->type() == ETH_TREE_NODE){
                        child->setCheckSelected(true);
                        printf("sel-eth\n");
                    }
                    if(child->type() == CAN_TREE_NODE)
                    {
                        child->setCheckSelected(true);
                        printf("sel-can\n");
                    }
                }
            }
        }
    }
#else

    CustomTreeWidgetItem *c = selectedNodes.first();

    QString targetboard = c->getBoardType();

    if(true){

        for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
            QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
            int mmm = topLevel->childCount();
            for(int j=0;j<topLevel->childCount();j++){

                QTreeWidgetItem *widgItem = topLevel->child(j);
                if(widgItem->type() != QTreeWidgetItem::Type){
                    CustomTreeWidgetItem *child = (CustomTreeWidgetItem*)widgItem;
                    QString brd = child->getBoardType();
                    if(child->checkIsEnabled()){
                        if((child->getBoardType() == c->getBoardType()) && (c->getParentNode() == child->getParentNode())){
                            child->setCheckSelected(true);
                        }
                    }

                    int nnn = topLevel->child(j)->childCount();
                    for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                        QTreeWidgetItem *widgItem1 = topLevel->child(j)->child(k);
                        if(widgItem1->type() != QTreeWidgetItem::Type){
                            CustomTreeWidgetItem *child1 = (CustomTreeWidgetItem*)widgItem1;

                            if(child1->checkIsEnabled()){
                                if((child1->getBoardType() == c->getBoardType()) && (c->getParentNode() == child1->getParentNode())){
                                    child1->setCheckSelected(true);
                                }
                            }
                        }

                    }
                }




            }

        }
    }



#endif

//    int size = selectedNodes.size();
//    for(int i=0;i<size;i++){

//                CustomTreeWidgetItem *child = selectedNodes[i];
//                if(child->isCheckSelected()){
//                    if(child->type() == ETH_TREE_NODE){
//                        child->setCheckSelected(false);
//                        printf("des-eth\n");
//                    }
//                    if(child->type() == CAN_TREE_NODE)
//                    {
//                        child->setCheckSelected(false);
//                        printf("des-can\n");
//                    }
//                }
//    }

    checkEnableButtons();
}


void MainWindow::onDes(void)
{

    if(selectedNodes.isEmpty())
    {
        return;
    }

//    for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
//        QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
//        for(int j=0;j<topLevel->childCount();j++){

//            QTreeWidgetItem *widgItem = topLevel->child(j);
//            if(selectedNodes.first()->type() != widgItem->type())
//            {
//                continue;
//            }

//            if((widgItem->type() != ETH_TREE_ROOT_NODE)  && (widgItem->type() != CAN_TREE_ROOT_NODE) && (widgItem->type() != QTreeWidgetItem::Type))
//            {
//                CustomTreeWidgetItem *child = (CustomTreeWidgetItem *)widgItem;
//                if(child->isCheckSelected()){
//                    if(child->type() == ETH_TREE_NODE){
//                        child->setCheckSelected(false);
//                        printf("des-eth\n");
//                    }
//                    if(child->type() == CAN_TREE_NODE)
//                    {
//                        child->setCheckSelected(false);
//                        printf("des-can\n");
//                    }
//                }
//            }
//        }
//    }

    int size = selectedNodes.size();
    for(int i=0;i<size;i++){

                CustomTreeWidgetItem *child = selectedNodes[i];
                if(child->isCheckSelected()){
                    if(child->type() == ETH_TREE_NODE){
                        child->setCheckSelected(false);
                        //printf("des-eth\n");
                    }
                    if(child->type() == CAN_TREE_NODE)
                    {
                        child->setCheckSelected(false);
                        //printf("des-can\n");
                    }
                }
    }

    checkEnableButtons();

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
                CustomTreeWidgetItem *parentNode = (CustomTreeWidgetItem*)selectedNodes.first()->getParentNode();
                QtConcurrent::run(this,&MainWindow::uploadCanApplication,filename,address,-1,parentNode);
            }else if(selectedNodes.first()->getParentNode()->type() == CAN_TREE_ROOT_NODE){
//                QString driver = selectedNodes.first()->getParentNode()->text(DEVICE);
//                int deviceId = selectedNodes.first()->getParentNode()->text(ID).toInt();
                QString driver;
                QString deviceIdstr;
                getDeviceID(selectedNodes.first()->getParentNode(), deviceIdstr, driver);
                int deviceId = deviceIdstr.toInt();
                CustomTreeWidgetItem *parentNode = (CustomTreeWidgetItem*)selectedNodes.first()->getParentNode();
                QtConcurrent::run(this,&MainWindow::uploadCanApplication,filename,driver,deviceId,parentNode);
            }
        }
    }




}

void MainWindow::uploadLoader(QString filename)
{
    QString result;
    core->uploadLoader(filename,&result);
    if(selectedNodes.count() > 0){
        CustomTreeWidgetItem *node = selectedNodes.first();
        if(node->getParentNode()->type() == ETH_TREE_ROOT_NODE){
            refreshEthBoardsNode(node->getParentNode());
        }
    }
    setInfoRes(result);
   // needSetRestartOnSelected();
}

void MainWindow::uploadUpdater(QString filename)
{
    QString result;
    core->uploadUpdater(filename,&result);
    if(selectedNodes.count() > 0){
        CustomTreeWidgetItem *node = selectedNodes.first();
        if(node->getParentNode()->type() == ETH_TREE_ROOT_NODE){
            refreshEthBoardsNode(node->getParentNode());
        }
    }
    setInfoRes(result);
    //needSetRestartOnSelected();
}

void MainWindow::uploadEthApplication(QString filename)
{
    QString result;
    core->uploadEthApplication(filename,&result);
    if(selectedNodes.count() > 0){
        CustomTreeWidgetItem *node = selectedNodes.first();
        if(node->getParentNode()->type() == ETH_TREE_ROOT_NODE){
            refreshEthBoardsNode(node->getParentNode());
        }
    }
    setInfoRes(result);
    //needSetRestartOnSelected();
}

void MainWindow::uploadCanApplication(QString filename,QString address,int deviceId,CustomTreeWidgetItem *node)
{
    bool bEraseEEPROM = false;
    if(ui->checkBoxEE->isEnabled())
    {
        Qt::CheckState eestate = ui->checkBoxEE->checkState();
        bEraseEEPROM = (Qt::Checked == eestate) ? true : false;
    }

    needLoading(true,true,"",false);
    QString result;
    if(node){
        core->setSelectedCanBoards(node->getCanBoards(),address,deviceId);
    }
    QList <sBoard> resultCanBoards;
    core->uploadCanApplication(filename, &result, bEraseEEPROM, address, deviceId, &resultCanBoards);
    needLoading(false,false,"",false);
    needLoading(true,true);
    node->setCanBoards(resultCanBoards);
    setInfoRes(result);
    canBoardsRetrieved(node,false);
    needLoading(false,false);
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
    QString devicestr;
    QString idstr;
    getDeviceID(ui->devicesTree->currentItem()->parent()->text(DEVICEID), idstr, devicestr);
    if(ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt() == 2 && devicestr == "ETH"){
//    if(ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt() == 2 && ui->devicesTree->currentItem()->parent()->text(DEVICE) == "ETH"){
        QtConcurrent::run(this,&MainWindow::populateInfo,ui->devicesTree->currentItem()->data(0,INDEX_OF_BOARD).toInt());
    }

}

void MainWindow::onBootUpdater(bool click)
{
    core->bootFromUpdater();
    QString devicestr;
    QString idstr;
    getDeviceID(ui->devicesTree->currentItem()->parent()->text(DEVICEID), idstr, devicestr);
    if(ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt() == 2 && devicestr == "ETH"){
//    if(ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt() == 2 && ui->devicesTree->currentItem()->parent()->text(DEVICE) == "ETH"){
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
//    foreach (CustomTreeWidgetItem *node, selectedNodes) {
//        node->setCheckSelected(false);
//    }
    QFuture<bool> future = QtConcurrent::run(core,&FirmwareUpdaterCore::goToMaintenance);
    watcher.setFuture(future);

}

void MainWindow::onGoToApplication(bool click)
{
    loading(true,true);
    foreach (CustomTreeWidgetItem *node, selectedNodes) {
        emptyNode(node); 
    }
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
//            CustomTreeWidgetItem *node = selectedNodes.first();
//            ((CanTreeWidgetItem*)node)->erasEeprom(!node->data(0,CAN_ERASE_EEPROM).toBool());
//            node->refresh();

//            checkSelection(true,node);
        }
    }
}

void MainWindow::onFutureFinished()
{
    if(watcher.result()){
        foreach (CustomTreeWidgetItem *it, selectedNodes) {
            it->refresh();
            it->setCheckSelected(false);
        }
    }
    loading(false);
    checkEnableButtons();
    onDeviceSelectionChanged();
}

void MainWindow::onStrainCalib(bool click)
{
    if(selectedNodes.isEmpty()){
        return;
    }
    CanTreeWidgetItem *it = (CanTreeWidgetItem*)selectedNodes.first();

    sBoard board = it->getBoard();
    yDebug() << "onStrainCalib() has device = " << device.toStdString();
    yDebug() << "onStrainCalib() has bus = " << board.bus;
    yDebug() << "onStrainCalib() has id = " << board.pid;
    StrainCalibGui strainCalibGui(device,board.bus,board.pid,core);
    strainCalibGui.exec();
}

void MainWindow::onBlinkPressed(bool click)
{
    core->blinkEthBoards();
}

bool MainWindow::getDeviceID(QTreeWidgetItem *child, QString &idstr, QString &devicestr)
{
    QString tmp = child->text(DEVICEID);
    QByteArray ba = tmp.toLatin1();
    const char *c_str2 = ba.data();
#if 0
    char device_cstr[64] = {0};
    char id_cstr[64] = {0};
    sscanf(c_str2, "%s %s", device_cstr, id_cstr);
    devicestr = device_cstr;
    idstr = id_cstr;
#else
    string ttmp = string(c_str2);
    std::size_t ff = ttmp.find("<");
    std::size_t ll = ttmp.find(">");
    string t1;
    t1.assign(ttmp, 0, ff);
    string t2;
    t2.assign(ttmp, ff+1, ll-ff-1);
    devicestr = t1.c_str();
    idstr = t2.c_str();
#endif

    return true;
}

bool MainWindow::getDeviceID(QString devicefullstring, QString &idstr, QString &devicestr)
{
    QString tmp = devicefullstring;
    QByteArray ba = tmp.toLatin1();
    const char *c_str2 = ba.data();
#if 0
    char device_cstr[64] = {0};
    char id_cstr[64] = {0};
    sscanf(c_str2, "%s %s", device_cstr, id_cstr);
    devicestr = device_cstr;
    idstr = id_cstr;
#else
    string ttmp = string(c_str2);
    std::size_t ff = ttmp.find("<");
    std::size_t ll = ttmp.find(">");
    string t1;
    t1.assign(ttmp, 0, ff);
    string t2;
    t2.assign(ttmp, ff+1, ll-ff-1);
    devicestr = t1.c_str();
    idstr = t2.c_str();
#endif
    return true;
}


void MainWindow::onChangeAddress(bool click)
{
    if(selectedNodes.count() != 1){
        return;
    }
    CustomTreeWidgetItem *child = selectedNodes.first();


    int cbus = 0;
    int cadr = 0;
    QString oldAddress;
    if(child->type() == ETH_TREE_NODE){
        oldAddress = core->getEthBoardAddress(child->getIndexOfBoard());
    } else if(child->type() == CAN_TREE_NODE){
        QString canBus;
        getCANaddress(child, cbus, cadr, canBus, oldAddress);
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
                int canType = child->data(0,CAN_TYPE).toInt();
                CustomTreeWidgetItem *parentNode = (CustomTreeWidgetItem*)child->getParentNode();

                if(parentNode->type() == ETH_TREE_NODE){

                    QPair <QString,QString> addr;
                    addr.second = dlg.getNewAddress();
                    addr.first = address;

                    QtConcurrent::run(this,&MainWindow::setCanBoardAddress,
                                      QPair<int,int>(cbus, cadr),
                                      canType,
                                      addr,
                                      -1,
                                      parentNode);



                }else if(parentNode->type() == CAN_TREE_ROOT_NODE){

                    //QString driver = parentNode->text(DEVICE);
                    //int networkId = parentNode->text(ID).toInt();
                    QString driver;
                    QString networkIdstr;
                    getDeviceID(child->getParentNode(), networkIdstr, driver);
                    int networkId = networkIdstr.toInt();

                    QPair <QString,QString> addr;
                    addr.second = dlg.getNewAddress();
                    addr.first = driver;

                    QtConcurrent::run(this,&MainWindow::setCanBoardAddress,
                                      QPair<int,int>(cbus, cadr),
                                      canType,
                                      addr,
                                      networkId,
                                      parentNode);
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
                CustomTreeWidgetItem *parentNode = (CustomTreeWidgetItem*)child->getParentNode();


                for(int i=0;i<child->childCount();i++){
                    child->child(i)->setDisabled(true);
                }

                int cbus = 0;
                int cadr = 0;
                QString cbustring;
                QString cadrstring;
                getCANaddress(child, cbus, cadr, cbustring, cadrstring);

                if(parentNode->type() == ETH_TREE_NODE){
                    QString address = parentNode->text(ADDRESS);
                    QtConcurrent::run(this,&MainWindow::setCanBoardInfo,
                                      QPair<int,int>(cbus, cadr),
                                      dlg.getNewInfo(),
                                      address,
                                      -1,
                                      parentNode);

                } else if(parentNode->type() == CAN_TREE_ROOT_NODE){
                    //QString driver = child->getParentNode()->text(DEVICE);
                    //int networkId = child->getParentNode()->text(ID).toInt();
                    QString driver;
                    QString networkIdstr;
                    getDeviceID(child->getParentNode(), networkIdstr, driver);
                    int networkId = networkIdstr.toInt();

                    QtConcurrent::run(this,&MainWindow::setCanBoardInfo,
                                      QPair<int,int>(cbus, cadr),
                                      dlg.getNewInfo(),
                                      driver,
                                      networkId,
                                      parentNode);

                }



            }
        }
    }

}

void MainWindow::onCalibrate(bool click)
{
    if(calibDlg){
        delete calibDlg;
        calibDlg = NULL;
    }

    calibDlg = new CalibrationWindow(core,sgboardtype, selectedNodes.first(),this);
    calibDlg->show();

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
        refreshNode->setIcon(DEVICEID,QIcon(":/images/restart-needed.png"));
    }else{
        refreshNode->setIcon(DEVICEID,QIcon());
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
            if(ui->devicesTree->currentItem()->parent()->text(DEVICEID).contains("ETH")){
                for(int i=0; i < ui->devicesTree->currentItem()->parent()->childCount();i++){
                    if(ui->devicesTree->currentItem()->parent()->child(i)->isSelected() &&
                            ui->devicesTree->currentItem()->parent()->child(i)->data(0,EMPTY_NODE).toBool() == false){
                        int boardNum =  ui->devicesTree->currentItem()->parent()->child(i)->data(0,INDEX_OF_BOARD).toInt();

                        QtConcurrent::run(this,&MainWindow::populateInfo,boardNum);
                    }
                }
            }
            else
            {
                // it is a can board either under an ETH board or under a different device
                // must be able to detect if it is can node under an ETH board or under CFW2...

                int level = ui->devicesTree->currentItem()->data(0,DEVICE_LEVEL).toInt();
                level = level;

                for(int i=0; i < ui->devicesTree->currentItem()->parent()->childCount();i++){
                    if(ui->devicesTree->currentItem()->parent()->child(i)->isSelected() &&
                            ui->devicesTree->currentItem()->parent()->child(i)->data(0,EMPTY_NODE).toBool() == false){
                        //int boardNum = 0;
                         int boardNum =  ui->devicesTree->currentItem()->parent()->child(i)->data(0,INDEX_OF_BOARD).toInt();

                        sBoard canBoard;
                        if(3 == level)
                        {
                            canBoard = ((EthTreeWidgetItem*)ui->devicesTree->currentItem()->parent())->getCanBoard(boardNum);
                        }
                        else
                        {   // on cfw2 or others
                            canBoard = ((CanTreeWidgetItem*)ui->devicesTree->currentItem())->getBoard();
                        }

                        QtConcurrent::run(this,&MainWindow::populateCANinfo,canBoard);
                    }
                }
            }


        }else{
            appendInfo();
        }
    }else{
        ui->connectButton->setEnabled(false);
        ui->actionSel->setEnabled(false);
        ui->actionDes->setEnabled(false);
        appendInfo();
    }
    checkEnableButtons();

}

void MainWindow::populateInfo(int boardNum)
{
    needLoading(true,true);
    QString details;
    eOipv4addr_t address;
    boardInfo2_t info = core->getMoreDetails(boardNum,&details,&address);
    if(!details.isEmpty()){
        appendInfo(details);
    }else{
        appendInfo(info,address);
    }
    needLoading(false,false);
}

void MainWindow::populateCANinfo(sBoard canboard)
{
    needLoading(true,true);
    appendInfo(canboard);
    needLoading(false,false);
}

void MainWindow::onAppendInfo(boardInfo2_t info,eOipv4addr_t address)
{
    ui->detailsText->setVisible(false);
    infoTreeWidget->setVisible(true);

    infoTreeWidget->clear();
    QTreeWidgetItem *boardNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "ETH board");
    QTreeWidgetItem *bootStrapNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "Bootstrap Processes");
    QTreeWidgetItem *propertiesNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "Properties of the Processes");
    infoTreeWidget->addTopLevelItem(boardNode);
    infoTreeWidget->addTopLevelItem(bootStrapNode);
    infoTreeWidget->addTopLevelItem(propertiesNode);

    QString type;
    switch (info.boardtype) {
    case eobrd_ethtype_ems4:
        type = "ems";
        break;
    case eobrd_ethtype_mc4plus:
        type = "mc4plus";
        break;
    case eobrd_ethtype_mc2plus:
        type = "mc2plus";
        break;
    case eobrd_ethtype_none:
        type = "none";
        break;
    case eobrd_ethtype_unknown:
        type = "unknown";
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

    QTreeWidgetItem *macNode = new QTreeWidgetItem(boardNode, QStringList() << "MAC" << board_mac);
    boardNode->addChild(macNode);

    QTreeWidgetItem *ipNode = new QTreeWidgetItem(boardNode, QStringList() << "IP" << ipv4tostring(address).c_str());
    boardNode->addChild(ipNode);

    QString mode = (info.maintenanceIsActive ? "maintenance" : "application");
    if(eApplPROGupdater == info.processes.runningnow)
    {
        mode = "special application for programming the updater";
    }
    else if((eApplication == info.processes.runningnow) && (0xff != info.applicationdetails))
    {
        if(0x01 == (0x01 & info.applicationdetails))
        {
            mode = "application (RUNNING)";
        }
        else
        {
            mode = "application (IDLE)";
        }
    }

    QTreeWidgetItem *statusNode = new QTreeWidgetItem(boardNode, QStringList() << "Status" << mode);
    
    {   // changing color to the mode 
        QFont ff = statusNode->font(1);
        ff.setBold(true);
        statusNode->setFont(1, ff);

        if(eApplPROGupdater == info.processes.runningnow)
        {
            statusNode->setForeground(1, Qt::magenta);
        }
        else if(eUpdater == info.processes.runningnow)
        {
            statusNode->setForeground(1, Qt::blue);
        }
        else
        {
            statusNode->setForeground(1, Qt::black);
        }
    } 
    
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

        QTreeWidgetItem *processVersion = new QTreeWidgetItem(processNode, QStringList() << "Version" << QString("%1.%2").arg(info.processes.info[i].version.major).arg(info.processes.info[i].version.minor));
        processNode->addChild(processVersion);

        QTreeWidgetItem *processDate = new QTreeWidgetItem(processNode, QStringList() << "Date" << QDateTime(QDate(info.processes.info[i].date.year,info.processes.info[i].date.month,info.processes.info[i].date.day),
                                                                                                             QTime(info.processes.info[i].date.hour,info.processes.info[i].date.min)).toString("yyyy/MM/dd - hh:mm"));
        processNode->addChild(processDate);

        QTreeWidgetItem *processBuilt = new QTreeWidgetItem(processNode, QStringList() << "Built On" << QDateTime(QDate(info.processes.info[i].compilationdate.year,info.processes.info[i].compilationdate.month,info.processes.info[i].compilationdate.day),
                                                                                                      QTime(info.processes.info[i].compilationdate.hour,info.processes.info[i].compilationdate.min)).toString("yyyy/MM/dd - hh:mm"));
        processNode->addChild(processBuilt);

        QTreeWidgetItem *processRom= new QTreeWidgetItem(processNode, QStringList() << "ROM" << QString("[%1, %1+%2) kb").arg(info.processes.info[i].rom_addr_kb).arg(info.processes.info[i].rom_size_kb));
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


void MainWindow::onAppendInfo(sBoard canboard)
{
    ui->detailsText->setVisible(false);
    infoTreeWidget->setVisible(true);

    infoTreeWidget->clear();
    QTreeWidgetItem *boardNode = new QTreeWidgetItem(infoTreeWidget,QStringList() << "CAN board");

    infoTreeWidget->addTopLevelItem(boardNode);

#if 0
    QString type;
    type = eoboards_type2string2((eObrd_type_t)canboard.type, eobool_true);

    QTreeWidgetItem *typeNode = new QTreeWidgetItem(boardNode, QStringList() << "Type" << type);
    boardNode->addChild(typeNode);


    QString addr;
    char aaa[16] = {0};
    snprintf(aaa, sizeof(aaa), "CAN%d:%d", canboard.bus, canboard.pid);
    addr = aaa;
    QTreeWidgetItem *adrNode = new QTreeWidgetItem(boardNode, QStringList() << "Address" << addr);
    boardNode->addChild(adrNode);
#else

    char str[64] = {0};

    const char *name = eoboards_type2string2((eObrd_type_t)canboard.type, eobool_true);
    QTreeWidgetItem *type = new QTreeWidgetItem(boardNode, QStringList() << "Type" << name);
    boardNode->addChild(type);


    snprintf(str, sizeof(str), "CAN%d:%d", canboard.bus, canboard.pid);
    QTreeWidgetItem *adr = new QTreeWidgetItem(boardNode, QStringList() << "Address" << str);
    boardNode->addChild(adr);

    if(true == canboard.applicationisrunning)
    {
        snprintf(str, sizeof(str), "canApplication");
    }
    else
    {
        snprintf(str, sizeof(str), "canBootloader");
    }
    QTreeWidgetItem *pro = new QTreeWidgetItem(boardNode, QStringList() << "Running process" << str);
    boardNode->addChild(pro);

    if(-1 == canboard.appl_vers_build)
    {
        snprintf(str, sizeof(str), "%d.%d", canboard.appl_vers_major, canboard.appl_vers_minor);
    }
    else
    {
        snprintf(str, sizeof(str), "%d.%d.%d", canboard.appl_vers_major, canboard.appl_vers_minor, canboard.appl_vers_build);
    }
    QTreeWidgetItem *fwvers = new QTreeWidgetItem(boardNode, QStringList() << "Firmware version" << str);
    boardNode->addChild(fwvers);

    if((0 == canboard.prot_vers_major) && (0 == canboard.prot_vers_minor))
    {
        snprintf(str, sizeof(str), "N/A");
    }
    else
    {
        snprintf(str, sizeof(str), "%d.%d", canboard.prot_vers_major, canboard.prot_vers_minor);
    }
    QTreeWidgetItem *prvers = new QTreeWidgetItem(boardNode, QStringList() << "CAN protocol version" << str);
    boardNode->addChild(prvers);

    snprintf(str, sizeof(str), "%s", canboard.add_info);
    QTreeWidgetItem *inf = new QTreeWidgetItem(boardNode, QStringList() << "Info" << str);
    boardNode->addChild(inf);

    if((eobrd_strain == canboard.type) || (eobrd_strain2 == canboard.type))
    {
        QTreeWidgetItem *sn = new QTreeWidgetItem(boardNode, QStringList() << "Serial Number" << canboard.serial);
        boardNode->addChild(sn);

        snprintf(str, sizeof(str), "at boot = %d, in use = %d", canboard.strainregsetatboot, canboard.strainregsetinuse);
        QTreeWidgetItem *rs = new QTreeWidgetItem(boardNode, QStringList() << "Regulations Set" << str);
        boardNode->addChild(rs);

//        QTreeWidgetItem *rsu = new QTreeWidgetItem(boardNode, QStringList() << "Regulations Set in use" << canboard.strainregsetinuse);
//        boardNode->addChild(rsu);
    }

//    QTreeWidgetItem *basic = new QTreeWidgetItem(boardNode, QStringList() << "Details" << "See left panel");
//    boardNode->addChild(basic);

#endif

    boardNode->setExpanded(true);

}



void MainWindow::onConnect()
{
    setInfoRes("");
    foreach (QTreeWidgetItem *it, ui->devicesTree->selectedItems()) {
        if(it->data(0,DEVICE_LEVEL).toInt() == 0){

             if(it->data(0,CONNECTED).toBool() == true){
                 //emptyNode(it);
                 it->setData(0,CONNECTED,false);
                 it->setTextColor(DEVICEID,QColor(Qt::red));
             }
             //QString device;
             QString deviceIdstr;
             getDeviceID(it, deviceIdstr, device);

             if(core->connectTo(device,deviceIdstr) > 0){
                 it->setData(0,CONNECTED,true);
                 it->setExpanded(true);
                 it->setTextColor(DEVICEID,QColor(Qt::green));
             }else{
                 it->setData(0,CONNECTED,false);
                 it->setTextColor(DEVICEID,QColor(Qt::red));
             }

             QtConcurrent::run(this,&MainWindow::refreshDevices);
        }

        if(it->data(0,DEVICE_LEVEL).toInt() == 2){
            QString sss = it->text(PROCESS);
            if(it->text(PROCESS).contains("eUpdater" )){
                QtConcurrent::run(this,&MainWindow::getCanBoards,it,false);
            }
            else{
                QMessageBox msgBox;
                if(it->text(PROCESS).contains("eApplPROGupdater" ))
                {
                    msgBox.setText("The executing process is the " + sss + " which does not allow CAN discovery but only the programming of the eUpdater. You have to put the board in maintenance mode");
                }
                else
                {
                     msgBox.setText("You have to put the board in maintenance mode to perform this operation. Now it is running the " + sss);
                }

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
    bool canChangeInfo =  true;
    bool canUploadUpdater = true;
    bool canJumpUpdater = true;
    bool canEraseEthEEPROM =  true;
    bool canChangeIP =  true;

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
                        canChangeInfo &= child->data(0,CAN_UPLOAD_APP).toBool();
                        canEraseEthEEPROM &= child->data(0,CAN_UPLOAD_APP).toBool();
                        canChangeIP &= child->data(0,CAN_UPLOAD_APP).toBool();
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
        ui->btnStrainCalib->setEnabled(false);
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
        ui->actionSel->setEnabled(false);
        ui->actionDes->setEnabled(false);
        ui->checkBoxEE->setCheckState(Qt::Unchecked);
        ui->checkBoxEE->setEnabled(false);
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
            ui->actionSel->setEnabled(false);
            ui->actionDes->setEnabled(false);
            ui->checkBoxEE->setCheckState(Qt::Unchecked);
            ui->checkBoxEE->setEnabled(false);
        }else{
            ui->btnBlink->setEnabled(true);
            ui->btnBootApp->setEnabled(true);
            ui->btnBootUpdater->setEnabled(true);

            ui->btnCalibrate->setEnabled(false);
            ui->btnChangeCanAddr->setEnabled(false);
            if(selectedNodes.count() == 1){
                // only if we have the updater running
                ui->btnChangeIp->setEnabled(canChangeIP);
                ui->btnCahngeInfo->setEnabled(canChangeInfo);
                ui->btnEraseEeprom->setEnabled(canEraseEthEEPROM);
            }else{
                ui->btnChangeIp->setEnabled(false);
                ui->btnCahngeInfo->setEnabled(false);
                ui->btnEraseEeprom->setEnabled(false);
            }

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
            ui->actionSel->setEnabled(true);
            ui->actionDes->setEnabled(true);
            ui->checkBoxEE->setCheckState(Qt::Unchecked);
            ui->checkBoxEE->setEnabled(false);
        }

    }else{
        ui->btnBlink->setEnabled(false);
        ui->btnBootApp->setEnabled(false);
        ui->btnBootUpdater->setEnabled(false);

        if(selectedNodes.count() == 1){
            ui->btnChangeCanAddr->setEnabled(true);
            // ok, we enablke it also for bootloader ...
            ui->btnCahngeInfo->setEnabled(true);

            sBoard canBoard = ((EthTreeWidgetItem*)selectedNodes.first()->getParentNode())->getCanBoard(selectedNodes.first()->getIndexOfBoard());
            if(/*core->strainCalibMode && */(canBoard.type == icubCanProto_boardType__strain) || (canBoard.type == icubCanProto_boardType__strain2)){
                ui->btnStrainCalib->setEnabled(true);
            }else{
                ui->btnStrainCalib->setEnabled(false);
            }
            if(((canBoard.type == icubCanProto_boardType__strain) || (canBoard.type == icubCanProto_boardType__strain2) || (canBoard.type == icubCanProto_boardType__6sg)) && (canBoard.status == BOARD_RUNNING) ){
                sgboardtype = static_cast<icubCanProto_boardType_t>(canBoard.type);
                ui->btnCalibrate->setEnabled(true);
                //ui->btnEraseEeprom->setEnabled(true);
                ui->btnEraseEeprom->setEnabled(false);
                ui->checkBoxEE->setCheckState(Qt::Unchecked); //(Qt::Checked); Qt::Unchecked
                ui->checkBoxEE->setEnabled(true);
            }else{
                ui->btnCalibrate->setEnabled(false);
                ui->btnEraseEeprom->setEnabled(false);
                ui->checkBoxEE->setCheckState(Qt::Unchecked); //(Qt::Checked); Qt::Unchecked
                ui->checkBoxEE->setEnabled(false);
            }
        }else{
            ui->btnChangeCanAddr->setEnabled(false);
            ui->btnCahngeInfo->setEnabled(false);
            ui->btnCalibrate->setEnabled(false);
            ui->btnEraseEeprom->setEnabled(false);
            ui->checkBoxEE->setCheckState(Qt::Unchecked);
            ui->checkBoxEE->setEnabled(false);
            ui->btnStrainCalib->setEnabled(false);
        }
        ui->btnChangeIp->setEnabled(false);

        ui->btnJumpUpdater->setEnabled(false);
        ui->btnRestart->setEnabled(false);
        //ui->btnRestartSecs->setEnabled(false);
        ui->btnGoToApplication->setEnabled(false);
        ui->btnGoToMaintenance->setEnabled(false);
        ui->btnUploadApp->setEnabled(true);
        ui->btnUploadLoader->setEnabled(false);
        ui->btnUploadUpdater->setEnabled(false);
        ui->actionSel->setEnabled(true);
        ui->actionDes->setEnabled(true);
    }


}

void MainWindow::emptyNode(QTreeWidgetItem *it)
{
    removeChildren(it);
    QTreeWidgetItem *empty = new QTreeWidgetItem(it,QStringList() << "" << "" << "?");
    empty->setData(0,EMPTY_NODE,true);
}

void MainWindow::refreshDevices()
{
    needLoading(true,true);
    QString ret;
    for (int i = 0; i<ui->devicesTree->selectedItems().count(); i++) {
        QTreeWidgetItem *it = ui->devicesTree->selectedItems().at(i);
        if(it->parent() != NULL){
            continue;
        }

        if(it->type() == ETH_TREE_ROOT_NODE){
            refreshEthBoardsNode(it);



        }else if(it->type() == CAN_TREE_ROOT_NODE){
            CustomTreeWidgetItem *canRootNode = (CustomTreeWidgetItem*)it;
            ret = canRootNode->retrieveCanBoards();
            setInfoRes(ret);
            canBoardsRetrieved(canRootNode,false);
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
    }else{
        emptyNode(it);
        return;
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
    CustomTreeWidgetItem *node = (CustomTreeWidgetItem*)it;

    int type = node->type();
    if(type != ETH_TREE_NODE && type != CAN_TREE_ROOT_NODE){
       return;
    }


    node->setExpanded(true);

    if(!refresh){
        if(node->getCanBoards().count() > 0){
            removeChildren(it);
            it->setData(0,EMPTY_NODE,false);
        }else{
            emptyNode(it);
            return;
        }
    }

    if(!refresh){
        for(int i=0;i<node->getCanBoards().count();i++){

            CanTreeWidgetItem *canNode = new CanTreeWidgetItem(node,core,i);


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
        for(int i=0;i<node->childCount();i++){
            CanTreeWidgetItem *canNode = (CanTreeWidgetItem*)node->child(i);
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
    onDeviceSelectionChanged();
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
//        ui->advancedGroup->setEnabled(false);
//        ui->controlsGroup->setEnabled(false);
        infoResult->setVisible(true);
        infoResult->setText("Updating...");
//        ui->devicesTree->setEnabled(false);
    }else{
        progress->setEnabled(false);
//        ui->advancedGroup->setEnabled(true);
//        ui->controlsGroup->setEnabled(true);
        infoResult->setText("Update Done");
//        ui->devicesTree->setEnabled(true);
    }
    progress->setValue(val);
    loadingMutex.unlock();

}

void MainWindow::loading(bool load, bool disableAll,QString msg,bool infiniteLoad)
{

    if(load/* && !isLoading*/){
        loadCounter++;
        isLoading = true;
        if(infiniteLoad){
            progress->setMaximum(0);
            progress->setEnabled(true);
        }

        ui->strainGroup->setEnabled(false);
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
            if(infiniteLoad){
                progress->setMaximum(100);
                progress->setEnabled(false);
            }
            ui->strainGroup->setEnabled(true);
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

void MainWindow::checkSelection(bool selected,CustomTreeWidgetItem *c)
{
    if(selected){

        for(int i=0;i<ui->devicesTree->topLevelItemCount();i++){
            QTreeWidgetItem *topLevel = ui->devicesTree->topLevelItem(i);
            for(int j=0;j<topLevel->childCount();j++){

                QTreeWidgetItem *widgItem = topLevel->child(j);
                if(widgItem->type() != QTreeWidgetItem::Type){
                    CustomTreeWidgetItem *child = (CustomTreeWidgetItem*)widgItem;
// marco.accame: must remove this code because otherwise, if i select an eth board w/ can beneath, the can boards are not check-deselectde
//                    if(child == c){
//                        continue;
//                    }
                    if(child->checkIsEnabled()){
                        if(child->getBoardType() != c->getBoardType() || c->getParentNode() != child->getParentNode() ){
                            child->setCheckEnabled(false);
                        }
                    }

                    for(int k=0; k<topLevel->child(j)->childCount();k++ ){
                        QTreeWidgetItem *widgItem1 = topLevel->child(j)->child(k);
                        if(widgItem1->type() != QTreeWidgetItem::Type){
                            CustomTreeWidgetItem *child1 = (CustomTreeWidgetItem*)widgItem1;
                            if(child1 == c){
                                continue;
                            }
                            if(child1->checkIsEnabled()){
                                if(child1->getBoardType() != c->getBoardType() || c->getParentNode() != child1->getParentNode() ){
                                    child1->setCheckEnabled(false);
                                }
                            }
                        }

                    }
                }

            }

        }

    }else{

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


    }else{

        selectedNodes.removeOne(c);

    }

    checkSelection(selected,c);

    loading(false);
    checkEnableButtons();
}


bool MainWindow::getCANaddress(CustomTreeWidgetItem *child, int &cbus, int &cadr, QString &cbustr, QString &cadrstr)
{
#if defined(_MAIN_WINDOW_SHOW_CAN_ADDRESS_IN_ADDRESS_COLUMN)

    QString tmp = child->text(ADDRESS);
    QByteArray ba = tmp.toLatin1();
    const char *c_str2 = ba.data();

    bool bIPprefix = false;
#if defined(_MAIN_WINDOW_USE_IP_PREFIX_FOR_CAN_ADDRESS)
    bIPprefix = (child->getParentNode()->type() == ETH_TREE_NODE) ? true : false;
#endif

    if(false == bIPprefix)
    {   // e.g., CAN2:7
        sscanf(c_str2, "CAN%d:%d", &cbus, &cadr);
    }
    else
    {   // e.g., 10.0.1.21:CAN2:7
        int ip1, ip2, ip3, ip4;
        sscanf(c_str2, "%d.%d.%d.%d:CAN%d:%d", &ip1, &ip2, &ip3, &ip4, &cbus, &cadr);
    }

    char strtmp[8];
    snprintf(strtmp, sizeof(strtmp), "%d", cadr);
    cadrstr = strtmp;

    snprintf(strtmp, sizeof(strtmp), "CAN%d", cbus);
    cbustr = strtmp;

#else

    cbus = child->text(ADDRESS).remove("CAN").toInt();
    cadr = child->text(ID).toInt();

    char strtmp[8];
    snprintf(strtmp, sizeof(strtmp), "%d", cadr);
    cadrstr = strtmp;

    snprintf(strtmp, sizeof(strtmp), "CAN%d", cbus);
    cbustr = strtmp;

#endif

    return true;
}




/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

