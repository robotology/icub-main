#include "firmwareupdatercore.h"
#include <qdebug.h>
#include <EoUpdaterProtocol.h>

FirmwareUpdaterCore *self = NULL;


static void updateProgressCallback(float fraction)
{
    self->updateProgress(fraction);
}

FirmwareUpdaterCore::FirmwareUpdaterCore(QObject *parent) : QObject(parent), mutex(QMutex::Recursive)
{
    self = this;
}

bool FirmwareUpdaterCore::init(Searchable& config, int port, QString address, int VerbositY)
{
    verbosity = VerbositY;

    mutex.lock();
    if(!gMNT.open()){
        if(verbosity>0) qDebug("Can't open socket, aborting.");
        mutex.unlock();
        return false;
    }

    setVerbosity(verbosity);


    Bottle sensorSetConfig=config.findGroup("DRIVERS").tail();

    for (int t=0; t<sensorSetConfig.size(); ++t){
        yarp::os::Bottle sensorConfig(sensorSetConfig.get(t).toString());

        QString type = QString("%1").arg(sensorConfig.get(0).asString().c_str());
        QString line = QString("%1").arg(sensorConfig.get(1).asString().c_str());
        if(verbosity>0) qDebug() << type << "-" << line;


        bool ok;
        int num = QString("%1").arg(line).toInt(&ok);
        if(ok){
            devices.append(QPair<QString,QVariant>(type,num));
        }else{
            devices.append(QPair<QString,QVariant>(type,line));
        }
    }
    mutex.unlock();
    return true;
}

bool FirmwareUpdaterCore::setVerbosity(int verb)
{
    verbosity = verb;
    bool lowers_are_verbose = (verb >= 1) ? true : false;
    gMNT.verbose(lowers_are_verbose);
    downloader.set_verbose(lowers_are_verbose);

    return true;
}

QStringList FirmwareUpdaterCore::getDevicesName()
{
    mutex.lock();
    QStringList names;
    for(int i=0;i<devices.count();i++){
        QPair<QString,QVariant> p = devices.at(i);
        names.append(p.first);
    }
    mutex.unlock();

    return names;
}

bool FirmwareUpdaterCore::goToMaintenance()
{
    mutex.lock();
    bool ret = gMNT.go2maintenance(EthMaintainer::ipv4OfAllSelected, true, 5, 1.0);;
    mutex.unlock();
    return ret;
}

bool FirmwareUpdaterCore::goToApplication()
{
    return gMNT.go2application(EthMaintainer::ipv4OfAllSelected, true, 10, true);
}

bool FirmwareUpdaterCore::eraseEthEprom()
{
    return gMNT.command_eeprom_erase(EthMaintainer::ipv4OfAllSelected);
}



bool FirmwareUpdaterCore::jumpToUpdater()
{
    mutex.lock();
    bool ret = gMNT.command_jump2updater(EthMaintainer::ipv4OfAllSelected);
    mutex.unlock();
    return ret;
}

QList<QPair<QString,QVariant> > FirmwareUpdaterCore::getDevices()
{
    return devices;
}

void FirmwareUpdaterCore::disconnectFrom(QString device, QString id)
{
    //    if(!device.isEmpty() && !id.isEmpty() && device.contains("ETH")){
    //        gMNT.boards_get().clear();
    //        yDebug() << "empty eth devices";
    //        qDebug() << "empty eth devices";
    //    }
    //    //connectTo(device,id);
}

int FirmwareUpdaterCore::connectTo(QString device, QString id)
{
    mutex.lock();
    if(!device.isEmpty() && !id.isEmpty()){
        if(device.contains("ETH")){
            int num = gMNT.discover(true, 2, 1.0).size();
            if(verbosity>0)
            {
                yDebug() << "FirmwareUpdaterCore::connectTo() has found " << num << " ETH boards";
            }
            mutex.unlock();
            return num;
        }else{
            QString result;
            QList<sBoard> s = getCanBoardsFromDriver(device,id.toInt(),&result,true);
            mutex.unlock();
            return s.count();
        }
    }
    mutex.unlock();
    return 0;
}

bool FirmwareUpdaterCore::isBoardInMaintenanceMode(QString ip)
{
    for(int i=0;i<gMNT.boards_get().size();i++){
        QString boardIp = QString("%1").arg(gMNT.boards_get()[i].getIPV4string().c_str());
        if(boardIp == ip){
            return gMNT.boards_get()[i].isInMaintenance();
        }
    }
    return false;
}

EthBoardList FirmwareUpdaterCore::getEthBoardList()
{
    return gMNT.boards_get();
}

void FirmwareUpdaterCore::setSelectedEthBoard(int index,bool selected)
{
    mutex.lock();
    if(gMNT.boards_get().size() > index){
        gMNT.boards_get()[index].setSelected(selected);
    }
    mutex.unlock();
    selectedEnded();
}

void FirmwareUpdaterCore::setSelectedEthBoard(QString boardIp,bool selected)
{
    mutex.lock();
    for(int i=0;i<gMNT.boards_get().size();i++){
        if(QString("%1").arg(gMNT.boards_get()[i].getIPV4string().c_str()) == boardIp){
            gMNT.boards_get()[i].setSelected(selected);
            break;
        }
    }
    mutex.unlock();
    selectedEnded();
}

void FirmwareUpdaterCore::setSelectedCanBoards(QList <sBoard> selectedBoards,QString address, int deviceId)
{
    mutex.lock();
    QString res;
    if(!address.isEmpty() && deviceId == -1){
        getCanBoardsFromEth(address,&res);
    }else{
        getCanBoardsFromDriver(address,deviceId,&res);
    }

    this->canBoards = selectedBoards;

    foreach (sBoard b, selectedBoards) {
        for(int i=0;i<downloader.board_list_size;i++){
            if(downloader.board_list[i].bus == b.bus &&
                    downloader.board_list[i].pid == b.pid){
                downloader.board_list[i].selected = b.selected;
                downloader.board_list[i].eeprom = b.eeprom;
            }
        }
    }

    mutex.unlock();
    selectedEnded();
}

void FirmwareUpdaterCore::eraseCanEprom()
{

}

void FirmwareUpdaterCore::setSelectedCanBoard(int index,bool selected,QString address, int deviceId)
{
    mutex.lock();
//    if(!ethAddress.isEmpty()){
//        if(currentAddress != ethAddress){
//            if(downloader.connected){
//                downloader.stopdriver();
//            }
//            QString res;
//            getCanBoardsFromEth(ethAddress,&res);
//        }
//    }
    QString res;
    if(!address.isEmpty() && deviceId == -1){
        getCanBoardsFromEth(address,&res);
    }else{
        getCanBoardsFromDriver(address,deviceId,&res);
    }
    downloader.board_list[index].selected=selected;
    mutex.unlock();
    selectedEnded();
}


boardInfo2_t FirmwareUpdaterCore::getMoreDetails(int boardNum,QString *infoString,eOipv4addr_t *address)
{
    mutex.lock();
    boardInfo2_t info = gMNT.boards_get()[boardNum].getInfo();
    if(address){
        *address = gMNT.boards_get()[boardNum].getIPV4();
        if(infoString && info.protversion == 0){
            *infoString =  QString("%1").arg(gMNT.moreinformation(*address, false).c_str());
        }
    }
    mutex.unlock();

    return info;

}

QString FirmwareUpdaterCore::getProcessFromUint(uint8_t id)
{
    switch (id) {
    case uprot_proc_Loader:
        return "eLoader";
    case uprot_proc_Updater:
        return "eUpdater";
    case uprot_proc_Application:
        return "eApplication";
    case uprot_proc_ApplPROGupdater:
        return "eApplPROGupdater";
    default:
        return "None";
        break;
    }
}

QList<sBoard> FirmwareUpdaterCore::getCanBoardsFromDriver(QString driver, int networkId, QString *retString, bool force)
{
    mutex.lock();

    if(force){
        downloader.stopdriver();
    }else{
        if(downloader.connected && (!currentAddress.isEmpty() || (currentDriver != driver || currentId != networkId ))){
            downloader.stopdriver();
        }
        if(currentDriver == driver && currentId == networkId){
            mutex.unlock();
            return canBoards;
        }
    }

    canBoards.clear();
    yarp::os::Property params;
    QString networkType;
    if(driver.contains("CFW2",Qt::CaseInsensitive)){
        networkType="cfw2can";
    } else if(driver.contains("ECAN",Qt::CaseInsensitive)){
        networkType = "ecan";
    } else if(driver.contains("PCAN",Qt::CaseInsensitive)){
        networkType = "pcan";
    } else if(driver.contains("SOCKET",Qt::CaseInsensitive)){
        networkType="socketcan";
    }
    params.put("device", networkType.toLatin1().data());
    params.put("canDeviceNum", networkId);
    params.put("canTxQueue", 64);
    params.put("canRxQueue", 64);
    params.put("canTxTimeout", 2000);
    params.put("canRxTimeout", 2000);

    //try to connect to the driver
    int ret = downloader.initdriver(params, (verbosity>1) ? true : false);

    if (0 != ret){
        if(-2 == ret){
            if(verbosity>0) qDebug() << "FirmwareUpdaterCore::getCanBoardsFromDriver(): Init ETH driver - The ETH board has just jumped to eUpdater\n Connect again";
            *retString = "FirmwareUpdaterCore::getCanBoardsFromDriver(): Init ETH driver - The ETH board has just jumped to eUpdater\n Connect again";
            // TODO DIALOG
        } else {
            if(verbosity>0) qDebug() << "FirmwareUpdaterCore::getCanBoardsFromDriver(): Init driver failed - Hardware busy or not connected?!";
            *retString = "Cannot init driver " + driver + "<" +  QString::number(networkId) + "> ... HW is busy or not connected";
            // TODO DIALOG
        }
        mutex.unlock();
        return canBoards;
    }


    ret = downloader.initschede();

    if (ret == -1)
    {
        if(verbosity>0) qDebug()  << "FirmwareUpdaterCore::getCanBoardsFromDriver(): No answer received from CAN boards after a successful driver init.";
        *retString = "No CAN boards found beneath " + driver + "<" + QString::number(networkId) + ">";
        downloader.stopdriver();
        currentAddress = "";
        //not_connected_status();
        mutex.unlock();
        return canBoards;
    }

    for(int i=0; i<downloader.board_list_size;i++){
        canBoards.append(downloader.board_list[i]);
    }
    currentAddress = "";
    currentDriver = driver;
    currentId = networkId;

    //downloader.stopdriver();
    mutex.unlock();
    return canBoards;

}

QList<sBoard > FirmwareUpdaterCore::getCanBoardsFromEth(QString address, QString *retString, int canID, bool force)
{
    mutex.lock();

    if(force){
        downloader.stopdriver();
    }else{
        if(downloader.connected && address != currentAddress && !currentAddress.isEmpty() || (currentAddress.isEmpty() && !currentDriver.isEmpty())){
            downloader.stopdriver();
        }

        if(currentAddress == address){
            mutex.unlock();
            return canBoards;
        }
    }


    canBoards.clear();
    unsigned int remoteAddr;
    unsigned int localAddr;
    


    if (!compile_ip_addresses(address.toLatin1().data(),&remoteAddr,&localAddr)){
        if(verbosity>0) qDebug() << "FirmwareUpdaterCore::getCanBoardsFromEth(): Init driver failed - Could not find network interface";
        // TODO DIALOG
        *retString = "Init driver failed - Could not find network interface";
        address = "";
        mutex.unlock();
        return canBoards;
    }


    yarp::os::Property params;
    params.put("device", "ETH");
    params.put("local", int( localAddr));
    params.put("remote",int(remoteAddr));
    params.put("canid",canID);


    //try to connect to the driver
    int ret = downloader.initdriver(params, (verbosity>1) ? true : false);

    if (0 != ret){
        if(-2 == ret){
            if(verbosity>0) qDebug() << "FirmwareUpdaterCore::getCanBoardsFromEth((): Init ETH driver - The ETH board has just jumped to eUpdater\n Connect again";
            *retString = "FirmwareUpdaterCore::getCanBoardsFromEth((): Init ETH driver - The ETH board has just jumped to eUpdater\n Connect again";
            // TODO DIALOG
        } else {
            if(verbosity>0) qDebug() << "FirmwareUpdaterCore::getCanBoardsFromEth((): Init driver failed - Hardware busy or not connected?!";
            *retString = "FirmwareUpdaterCore::getCanBoardsFromEth(): Init driver failed - Hardware busy or not connected?!";
            // TODO DIALOG
        }
        mutex.unlock();
        return canBoards;
    }


    ret = downloader.initschede();

    if (ret == -1)
    {
        if(verbosity>0) qDebug()  << "FirmwareUpdaterCore::getCanBoardsFromEth(): No CAN boards found beneath " << address << " after a successful driver init.";
        *retString = "No CAN boards found beneath " + address;
        downloader.stopdriver();
        address = "";
        //not_connected_status();
        mutex.unlock();
        return canBoards;
    }

    for(int i=0; i<downloader.board_list_size;i++){
        canBoards.append(downloader.board_list[i]);
    }
    currentAddress = address;

    //downloader.stopdriver();
    mutex.unlock();
    return canBoards;

}



bool FirmwareUpdaterCore::compile_ip_addresses(const char* addr,unsigned int *remoteAddr,unsigned int *localAddr)
{
    ACE_UINT32 ip1,ip2,ip3,ip4;
    sscanf(addr,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    *remoteAddr=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;

    size_t count=0;
    ACE_INET_Addr* addr_array=NULL;
    int ret=ACE::get_ip_interfaces(count,addr_array);

    if (ret || count<=0)
    {
        mutex.unlock();
        return false;
    }

    *localAddr=addr_array[0].get_ip_address();

    for (unsigned int a=1; a<count; ++a)
    {
        if ((*remoteAddr & 0xFFFF0000)==(addr_array[a].get_ip_address() & 0xFFFF0000))
        {
            *localAddr=addr_array[a].get_ip_address();
            break;
        }
    }

    return true;
}

void FirmwareUpdaterCore::blinkEthBoards()
{
    mutex.lock();
    gMNT.command_blink(EthMaintainer::ipv4OfAllSelected);
    mutex.unlock();
}

QString FirmwareUpdaterCore::getEthBoardInfo(int index)
{
    mutex.lock();
    QString ret = QString("%1").arg(gMNT.boards_get()[index].getInfoOnEEPROM().c_str());
    mutex.unlock();
    return ret;
}

QString FirmwareUpdaterCore::getEthBoardAddress(int index)
{
    mutex.lock();
    char board_ipaddr[16];
    ACE_UINT32 ip = ipv4toace(gMNT.boards_get()[index].getIPV4());
    sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
    mutex.unlock();
    return QString("%1").arg(board_ipaddr);
}

bool FirmwareUpdaterCore::setEthBoardInfo(int index, QString newInfo)
{
    mutex.lock();
    eOipv4addr_t address = gMNT.boards_get()[index].getIPV4();
    bool ret = gMNT.command_info32_set(address, newInfo.toLatin1().data());
    if(!ret){
        if(verbosity>0) qDebug() << "setEthBoardInfo failed";
    }


    vector<string> vv = gMNT.command_info32_get(address);
    foreach (string v, vv) {
        if(verbosity>0) qDebug() << v.c_str();
    }
    // TODO chiedere
    // it already sets it internally to commandInfo32Get()
    if(vv.size() > 0){
        if(verbosity>0) qDebug() << gMNT.boards_get()[index].getInfoOnEEPROM().c_str();
    }
    mutex.unlock();
    return true;
}

void FirmwareUpdaterCore::setCanBoardInfo(int bus, int id, QString newInfo,QString address, int deviceId,QString *resultString)
{
    mutex.lock();
    QString res;
    if(!address.isEmpty() && deviceId == -1){
        getCanBoardsFromEth(address,&res);
    }else{
        getCanBoardsFromDriver(address,deviceId,&res);
    }
    downloader.change_board_info(bus, id, newInfo.toLatin1().data());

//    if(!ethAddress.isEmpty()){
//        if(currentAddress != ethAddress){
//            if(downloader.connected){
//                downloader.stopdriver();
//            }
//            getCanBoardsFromEth(ethAddress,resultString);
//        }
//        downloader.change_board_info(bus, id, newInfo.toLatin1().data());
//    }
    mutex.unlock();
}

bool FirmwareUpdaterCore::setCanBoardAddress(int bus, int id, int canType,QString newAddress,QString address,int deviceId,QString *resultString)
{
    mutex.lock();
    QString res;
    if(!address.isEmpty() && deviceId == -1){
        getCanBoardsFromEth(address,&res);
    }else{
        getCanBoardsFromDriver(address,deviceId,&res);
    }

    int new_val = newAddress.toInt();
    if (new_val <=0 || new_val> 15){
        if(verbosity>0) qDebug() << "Error, new address out of range 0 - 15";
        return false;
    }

    if (new_val == id){
        if(verbosity>0) qDebug() << "Error, same address set";
        return false;
    }

    downloader.change_card_address(bus, id, new_val,canType);

//    if(!ethAddress.isEmpty()){
//        if(currentAddress != ethAddress){
//            if(downloader.connected){
//                downloader.stopdriver();
//            }
//            getCanBoardsFromEth(ethAddress,resultString);
//        }

//        int new_val = newAddress.toInt();
//        if (new_val <=0 || new_val> 15){
//            if(verbosity>0) qDebug() << "Error, new address out of range 0 - 15";
//            return false;
//        }

//        if (new_val == id){
//            if(verbosity>0) qDebug() << "Error, same address set";
//            return false;
//        }

//        downloader.change_card_address(bus, id, new_val,canType);
//    }
    mutex.unlock();

    return true;
}

bool FirmwareUpdaterCore::setEthBoardAddress(int index, QString newAddress)
{

    mutex.lock();
    int ip1,ip2,ip3,ip4;
    sscanf(newAddress.toLatin1().data(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    if (ip1<0 || ip1>255 || ip2<0 || ip2>255 || ip3<0 || ip3>255 || ip4<0 || ip4>255){
        mutex.unlock();
        return false;
    }
    ACE_UINT32 iNewAddress=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;


    ACE_UINT32 address = ipv4toace(gMNT.boards_get()[index].getIPV4());
    ACE_UINT32 mask = 0xFFFFFF00;

    if(iNewAddress == (iNewAddress & mask)){ // checks new ip address is not a network address . For example x.y.z.w/24 x.y.z.0
        if(verbosity>0) qDebug() << "Error Setting address";
        mutex.unlock();
        return false;
    }

    if((~mask) == (iNewAddress & (~mask))){ // checks new ip address is not a broadcast address . For example x.y.z.w/24 x.y.z.255
        if(verbosity>0) qDebug() << "Error Setting address";
        mutex.unlock();
        return false;
    }

    if (iNewAddress == address){
        if(verbosity>0) qDebug() << "Error, same address set";
        mutex.unlock();
        return false;
    }
    char old_addr[16];
    sprintf(old_addr,"%d.%d.%d.%d",(address>>24)&0xFF,(address>>16)&0xFF,(address>>8)&0xFF,address&0xFF);

    bool ret = gMNT.command_changeaddress(acetoipv4(address), acetoipv4(iNewAddress), true, true, true, true);
    mutex.unlock();
    return ret;



}


bool FirmwareUpdaterCore::uploadLoader(QString filename,QString *resultString)
{
    mutex.lock();
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        if(verbosity>0) qDebug() << "Error opening the selected file!";
        mutex.unlock();
        return false;
    }
    eOipv4addr_t ipv4 = 0; // all selected
    eObrd_ethtype_t type = eobrd_ethtype_none;
    eOversion_t ver;
    ver.major = 0;
    ver.minor = 0;
    std::string result;
    bool ok = gMNT.program(ipv4, type, eLoader, ver, programFile, false, updateProgressCallback, false);

    fclose(programFile);

    *resultString = QString("%1").arg(result.c_str());
    if(ok){
        mutex.unlock();
        return true;
    }
    mutex.unlock();
    return false;
}


bool FirmwareUpdaterCore::uploadUpdater(QString filename,QString *resultString)
{
    mutex.lock();
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        if(verbosity>0) qDebug() << "Error opening the selected file!";
        mutex.unlock();
        return false;
    }
    eOipv4addr_t ipv4 = 0; // all selected
    eObrd_ethtype_t type = eobrd_ethtype_none;
    eOversion_t ver;
    ver.major = 0;
    ver.minor = 0;
    std::string result;
    bool ok = gMNT.program(ipv4, type, eUpdater, ver, programFile, false, updateProgressCallback, false);

    fclose(programFile);

    *resultString = QString("%1").arg(result.c_str());
    if(ok){
        mutex.unlock();
        return true;
    }
    mutex.unlock();
    return false;
}


#ifdef SERIALMETHOD
bool FirmwareUpdaterCore::uploadCanApplication(QString filename,QString *resultString, QString ethAddress)
{
    mutex.lock();
    if(!ethAddress.isEmpty()){
        if(currentAddress != ethAddress){
            if(downloader.connected){
                downloader.stopdriver();
            }
            getCanBoardsFromEth(ethAddress,resultString);
        }

    }
    double timer_start =0;
    double timer_end   =0;

    if (downloader.connected == false){
        *resultString ="Driver not running";
        mutex.unlock();
        return false;
    }

    //check if at least one board was selected
    bool at_least_one_board_selected = false;

    for (int i=0; i<downloader.board_list_size; i++){
        if (downloader.board_list[i].status==BOARD_RUNNING &&
                downloader.board_list[i].selected==true)
            at_least_one_board_selected = true;
    }

    if (!at_least_one_board_selected){
        *resultString = "No Boards selected! - Select one or more boards to update the firmware";
        mutex.unlock();
        return false;
    }

    QMap<int,int> canDevices;
    for (int i=0; i<downloader.board_list_size; i++)
    {
        if (downloader.board_list[i].selected==true)
        {
            canDevices.insertMulti(downloader.board_list[i].bus,i);
            downloader.board_list[i].selected = false;
            if(verbosity>0) qDebug() << "FOUND SELECTED SCHEDA " << i << " ON BUS " << downloader.board_list[i].bus;
        }
    }

    int ret      = 0;
    int finished = 0;
    int busCount = canDevices.uniqueKeys().count();
    for(int k=0;k<busCount;k++){

        int bus = canDevices.uniqueKeys().at(k);

        if (downloader.open_file(filename.toLatin1().data())!=0){
            *resultString = "Error opening the selected file!";
            mutex.unlock();
            return false;
        }
        if(verbosity>0) qDebug() << "FILE " << filename << " OPENED";
        //TODO
        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        //        if (strstr (buffer, "calibrationDataSN") != 0)
        //        {
        //            load_calibration (buffer);
        //            return ALL_OK;
        //        }
        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        // Get an identification of the firmware fot the file that you have selected
        //    int firmware_board_type=0;
        //    int firmware_version=0;
        //    int firmware_revision=0;

        //indentify download type from the type of the selected boards
        int download_type = icubCanProto_boardType__unknown;
        bool download_eeprom =false;


        QList<int> indexes = canDevices.values(bus);
//        foreach (int index, indexes) {
//            downloader.board_list[index].selected = true;
//            if(verbosity>0) qDebug() << "SELECTING BOARD " << index << " OF BUS " << k;
//        }


        for(int i=0;i<indexes.count();i++){
            int index = indexes.at(i);
            downloader.board_list[index].selected = true;
            if(verbosity>0) qDebug() << "SELECTING BOARD " << index << " OF BUS " << bus;
            if (downloader.board_list[index].status==BOARD_RUNNING){
                if (downloader.startscheda(bus,
                                           downloader.board_list[index].pid,
                                           downloader.board_list[index].eeprom,
                                           downloader.board_list[index].type)!=0){
                    *resultString = "Unable to start the board - Unable to send message 'start' or no answer received";
                    mutex.unlock();
                    return false;
                } else {
                    if(verbosity>0) qDebug() << "START SCHEDA  " << index << " ON BUS " << bus << " OK";
                    downloader.board_list[index].status=BOARD_WAITING;
                }
                download_type = downloader.board_list[index].type;
                download_eeprom = downloader.board_list[index].eeprom;
            }
        }



//        // Start the download for the selected boards
//        for (int i=0; i<downloader.board_list_size; i++){
//            if (downloader.board_list[i].status==BOARD_RUNNING && downloader.board_list[i].selected==true){
//                if (downloader.startscheda(downloader.board_list[i].bus,
//                                           downloader.board_list[i].pid,
//                                           downloader.board_list[i].eeprom,
//                                           downloader.board_list[i].type)!=0){
//                    *resultString = "Unable to start the board - Unable to send message 'start' or no answer received";
//                    return false;
//                } else {
//                    if(verbosity>0) qDebug() << "START SCHEDA  " << i << " OK";
//                    downloader.board_list[i].status=BOARD_WAITING;
//                }
//                download_type = downloader.board_list[i].type;
//                download_eeprom = downloader.board_list[i].eeprom;
//            }
//        }



        timer_start= yarp::os::Time::now();
        finished = 0;
        bool print00 = false, print25 = false, print50 = false, print75 = false, print99 = false;
        // Start the download for the selected boards
        do
        {
            ret = downloader.download_file(bus, 0x0F, download_type,download_eeprom);
            if (float(downloader.progress)/downloader.file_length/busCount >0.0  && print00==false)    {if(verbosity>0) qDebug("downloading %s, 1%% done\n",filename.toLatin1().data()); print00=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.25 && print25==false)    {if(verbosity>0) qDebug("downloading %s, 25%% done\n",filename.toLatin1().data()); print25=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.50 && print50==false)    {if(verbosity>0) qDebug("downloading %s, 50%% done\n",filename.toLatin1().data()); print50=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.75 && print75==false)    {if(verbosity>0) qDebug("downloading %s, 75%% done\n",filename.toLatin1().data()); print75=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.99 && print99==false)    {if(verbosity>0) qDebug("downloading %s, finished!\n",filename.toLatin1().data()); print99=true;}

            if (ret==1){
                updateProgress(float(downloader.progress)/downloader.file_length/busCount);
            }
            if (ret==-1){
                if(verbosity>0) qDebug() << "Fatal Error during download, terminate";
                *resultString = "Fatal Error during download, terminate";
                finished = 1;
            }
            if (ret==0){
                if(verbosity>0) qDebug() << "Download terminated";
                *resultString = "Download terminated";
                finished = 1;
            }

        }
        while (finished!=1);

        // End the download for the selected boards

        if(downloader.stopscheda(bus, 15) != 0){
            if(verbosity>0) qDebug() << "ERROR STOPPING SCHEDA";
        }else{
            if(verbosity>0) qDebug() << "scheda stopped";
        }

        foreach (int index, indexes) {
            downloader.board_list[index].selected = false;
            if(verbosity>0) qDebug() << "DE-SELECTING BOARD " << index << " OF BUS " << bus;
        }



    }
    timer_end= yarp::os::Time::now();




    //Display result message
    if (ret == 0)
    {
        char time_text [50];
        double download_time = (timer_end-timer_start) ;
        sprintf (time_text, "All Board OK! Download Time (s): %.2f", download_time);

        *resultString = QString("Download Finished. %1").arg(time_text);

        updateProgress(1.0);
        mutex.unlock();
        return true;
    }
    else
    {
        //*resultString = "Error during file transfer";

        updateProgress(1.0);
        mutex.unlock();
        return false;
    }

    mutex.unlock();
    return true;
}

#else
bool FirmwareUpdaterCore::uploadCanApplication(QString filename,QString *resultString, bool ee, QString address,int deviceId,QList <sBoard> *resultCanBoards)
{
//    if(!address.isEmpty()){
//        if(currentAddress != address){
//            if(downloader.connected){
//                downloader.stopdriver();
//            }
//            getCanBoardsFromEth(address,resultString);
//        }

//    }
    QString res;
    if(!address.isEmpty() && deviceId == -1){
        getCanBoardsFromEth(address,&res);
    }else{
        getCanBoardsFromDriver(address,deviceId,&res);
    }


    double timer_start =0;
    double timer_end   =0;

    if (downloader.connected == false){
        *resultString ="Driver not running";
        return false;
    }

    //check if at least one board was selected
    bool at_least_one_board_selected = false;
    int i = 0;

    for (i=0; i<downloader.board_list_size; i++){
        if (downloader.board_list[i].status==BOARD_RUNNING &&
                downloader.board_list[i].selected==true)
            at_least_one_board_selected = true;
    }

    if (!at_least_one_board_selected){
        *resultString = "No Boards selected! - Select one or more boards to update the firmware";
        return false;
    }
    if (downloader.open_file(filename.toLatin1().data())!=0){
        *resultString = "Error opening the selected file!";
        return false;
    }
    //TODO
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //        if (strstr (buffer, "calibrationDataSN") != 0)
    //        {
    //            load_calibration (buffer);
    //            return ALL_OK;
    //        }
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    // Get an identification of the firmware fot the file that you have selected
    int firmware_board_type=0;
    int firmware_version=0;
    int firmware_revision=0;

    //indentify download type from the type of the selected boards
    int download_type = icubCanProto_boardType__unknown;
    bool download_eeprom =false;
    for (i=0; i<downloader.board_list_size; i++)
    {
        if (downloader.board_list[i].selected==true)
        {
            download_type = downloader.board_list[i].type;
            download_eeprom = downloader.board_list[i].eeprom;
        }

    }
    download_eeprom = ee;

    // Start the download for the selected boards
    for (i=0; i<downloader.board_list_size; i++){
        if (downloader.board_list[i].status==BOARD_RUNNING && downloader.board_list[i].selected==true){
            //bool EE = downloader.board_list[i].eeprom;
            bool EE = ee;
            if (downloader.startscheda(downloader.board_list[i].bus, downloader.board_list[i].pid, EE, downloader.board_list[i].type)!=0){
                *resultString = "Unable to start the board - Unable to send message 'start' or no answer received";
                return false;
            } else {
                downloader.board_list[i].status=BOARD_WAITING;
            }
        }
    }

    int ret      = 0;
    int finished = 0;

    timer_start= yarp::os::Time::now();

//    bool acemortest_notyetstopped = true;
//    const float acemortest_progress = 0.10;

    bool print00 = false, print25 = false, print50 = false, print75 = false, print99 = false;
    // Start the download for the selected boards
    do
    {
        ret = downloader.download_file(CanPacket::everyCANbus, 0x0F, download_type,download_eeprom);
        if (float(downloader.progress)/downloader.file_length >0.0  && print00==false)    {if(verbosity>0) qDebug("programming %s: 1%% done",filename.toLatin1().data()); print00=true;}
        if (float(downloader.progress)/downloader.file_length >0.25 && print25==false)    {if(verbosity>0) qDebug("programming %s: 25%% done",filename.toLatin1().data()); print25=true;}
        if (float(downloader.progress)/downloader.file_length >0.50 && print50==false)    {if(verbosity>0) qDebug("programming %s: 50%% done",filename.toLatin1().data()); print50=true;}
        if (float(downloader.progress)/downloader.file_length >0.75 && print75==false)    {if(verbosity>0) qDebug("programming %s: 75%% done",filename.toLatin1().data()); print75=true;}
        if (float(downloader.progress)/downloader.file_length >0.99 && print99==false)    {if(verbosity>0) qDebug("programming %s: finished!",filename.toLatin1().data()); print99=true;}

//        if ((float(downloader.progress)/downloader.file_length > acemortest_progress) && (acemortest_notyetstopped))
//        {
//            // place you breakpoint in here.
//            acemortest_notyetstopped = false;
//        }

        if (ret==1)
        {
            updateProgress(float(downloader.progress)/downloader.file_length);
        }
        if (ret==-1)
        {
            *resultString = "Fatal Error during download, terminate";
            finished = 1;
        }
        if (ret==0)
        {
            *resultString = "Download terminated";
            finished = 1;
        }

    }
    while (finished!=1);
    timer_end= yarp::os::Time::now();

    // End the download for the selected boards
    int errors =0;
    downloader.stopscheda(CanPacket::everyCANbus, 15);



    yarp::os::Time::delay(3.0);
    //Display result message
    if (ret == 0)
    {
        char time_text [50];
        double download_time = (timer_end-timer_start) ;
        sprintf (time_text, "All Board OK! Download Time (s): %.2f", download_time);

        *resultString = QString("Download Finished. %1").arg(time_text);

        updateProgress(1.0);
        downloader.initschede();

        canBoards.clear();
        for(int i=0; i<downloader.board_list_size;i++){
            canBoards.append(downloader.board_list[i]);
        }
        if(resultCanBoards){
            *resultCanBoards = canBoards;
        }
        return true;
    }
    else
    {
        //*resultString = "Error during file transfer";

        updateProgress(1.0);
        downloader.initschede();
        return false;
    }
    downloader.initschede();

    canBoards.clear();
    for(int i=0; i<downloader.board_list_size;i++){
        canBoards.append(downloader.board_list[i]);
    }
    *resultCanBoards = canBoards;
    return true;
}
#endif

bool FirmwareUpdaterCore::uploadEthApplication(QString filename,QString *resultString)
{
    mutex.lock();
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
//    if(verbosity>0) qDebug() << "attempting opening the selected file:" << filename << "or .." << filename.toLatin1().data();
    if (!programFile){
        //TODO ERROR
        if(verbosity>0) qDebug() << "Error opening the selected file:" << filename << "or .." << filename.toLatin1().data();
        mutex.unlock();
        return false;
    }
    eOipv4addr_t ipv4 = 0; // all selected
    eObrd_ethtype_t type = eobrd_ethtype_none;
    eOversion_t ver;
    ver.major = 0;
    ver.minor = 0;
    std::string result;
    bool ok = gMNT.program(ipv4, type, eApplication, ver, programFile, false, updateProgressCallback, false);

    fclose(programFile);

    *resultString = QString("%1").arg(result.c_str());
    if(ok){
        mutex.unlock();
        return true;
    }
    mutex.unlock();
    return false;
}



cDownloader *FirmwareUpdaterCore::getDownloader()
{
    return &downloader;
}



void FirmwareUpdaterCore::restartEthBoards()
{
    mutex.lock();
    gMNT.command_restart(EthMaintainer::ipv4OfAllSelected);
    mutex.unlock();
}


void FirmwareUpdaterCore::bootFromApplication()
{
    mutex.lock();
    gMNT.command_def2run(EthMaintainer::ipv4OfAllSelected, eApplication, false, false);
    mutex.unlock();
}

void FirmwareUpdaterCore::bootFromUpdater()
{
    mutex.lock();
    gMNT.command_def2run(EthMaintainer::ipv4OfAllSelected, eUpdater, false, false);
    mutex.unlock();
}
