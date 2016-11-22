#include "customtreewidgetitem.h"

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

QString CustomTreeWidgetItem::retrieveCanBoards(bool force)
{
    QString result;
    if(type() == ETH_TREE_NODE){
        canBoards = core->getCanBoardsFromEth(text(ADDRESS),&result,CanPacket::everyCANbus,force);
    } else if(type() == CAN_TREE_ROOT_NODE){
        canBoards = core->getCanBoardsFromDriver(text(DEVICE),text(ID).toInt(),&result,force);
    }

    return result;
}

void CustomTreeWidgetItem::setCanBoards(QList<sBoard> boards)
{
    canBoards = boards;
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
    if(index >= canBoards.count()){
        return;
    }
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
        setData(0,CAN_UPLOAD_LOADER,false);
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

