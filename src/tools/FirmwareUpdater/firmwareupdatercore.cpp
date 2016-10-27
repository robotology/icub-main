#include "firmwareupdatercore.h"
#include <qdebug.h>
#include <EoUpdaterProtocol.h>

FirmwareUpdaterCore *self = NULL;


static void updateProgressCallback(float fraction)
{
    self->updateProgress(fraction);
}

FirmwareUpdaterCore::FirmwareUpdaterCore(QObject *parent) : QObject(parent)
{
    self = this;
}

bool FirmwareUpdaterCore::init(Searchable& config, int port, QString address)
{


    if(!gMNT.open()){
        qDebug("Can't open socket, aborting.");

        return false;
    }

    Bottle sensorSetConfig=config.findGroup("DRIVERS").tail();

    for (int t=0; t<sensorSetConfig.size(); ++t){
        yarp::os::Bottle sensorConfig(sensorSetConfig.get(t).toString());

        QString type = QString("%1").arg(sensorConfig.get(0).asString().c_str());
        QString line = QString("%1").arg(sensorConfig.get(1).asString().c_str());
        qDebug() << type << "-" << line;

        bool ok;
        int num = QString("%1").arg(line).toInt(&ok);
        if(ok){
            devices.append(QPair<QString,QVariant>(type,num));
        }else{
            devices.append(QPair<QString,QVariant>(type,line));
        }
    }

    return true;
}

QStringList FirmwareUpdaterCore::getDevicesName()
{
    QStringList names;
    for(int i=0;i<devices.count();i++){
        QPair<QString,QVariant> p = devices.at(i);
        names.append(p.first);
    }


    return names;
}

bool FirmwareUpdaterCore::goToMaintenance()
{
    return gMNT.go2maintenance(EthMaintainer::ipv4OfAllSelected, true, 5, 1.0);
}

bool FirmwareUpdaterCore::goToApplication()
{
    return gMNT.go2application(EthMaintainer::ipv4OfAllSelected, true, 10, true);
}

bool FirmwareUpdaterCore::jumpToUpdater()
{
    return gMNT.command_jump2updater(EthMaintainer::ipv4OfAllSelected);
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
    if(!device.isEmpty() && !id.isEmpty() && device.contains("ETH")){
        int num = gMNT.discover(true, 2, 1.0).size();
        yDebug() << "Found " << num << " devices";
        qDebug() << "Found " << num << " devices";
        return num;
    }
    return 0;
}

EthBoardList FirmwareUpdaterCore::getEthBoardList()
{
    return gMNT.boards_get();
}

void FirmwareUpdaterCore::setSelectedEthBoard(int index,bool selected)
{

    if(gMNT.boards_get().size() > index){
        gMNT.boards_get()[index].setSelected(selected);
    }
    selectedEnded();
}

void FirmwareUpdaterCore::setSelectedCanBoard(int index,bool selected,QString ethAddress)
{
    if(!ethAddress.isEmpty()){
        if(currentAddress != ethAddress){
            if(downloader.connected){
                downloader.stopdriver();
            }
            QString res;
            getCanBoardsFromEth(ethAddress,&res);
        }
    }
    downloader.board_list[index].selected=selected;
    selectedEnded();
}


boardInfo2_t FirmwareUpdaterCore::getMoreDetails(int boardNum,QString *infoString,eOipv4addr_t *address)
{
    boardInfo2_t info = gMNT.boards_get()[boardNum].getInfo();
    if(address){
        *address = gMNT.boards_get()[boardNum].getIPV4();
        if(infoString && info.protversion == 0){
            *infoString =  QString("%1").arg(gMNT.moreinformation(*address, false).c_str());
        }
    }

    return info;

}

QString FirmwareUpdaterCore::getProcessFromUint(uint8_t id)
{
    switch (id) {
    case uprot_proc_Loader:
        return "Loader";
    case uprot_proc_Updater:
        return "Updater";
    case uprot_proc_Application:
        return "Application";
    case uprot_proc_ApplPROGupdater:
        return "Application Program Updater";
    default:
        return "None";
        break;
    }
}

QList<sBoard > FirmwareUpdaterCore::getCanBoardsFromEth(QString address, QString *retString, int canID)
{
    mutex.lock();
    QList <sBoard> canBoards;
    if(downloader.connected && address == currentAddress && !currentAddress.isEmpty()){
        downloader.stopdriver();
    }

    unsigned int remoteAddr;
    unsigned int localAddr;
    


    if (!compile_ip_addresses(address.toLatin1().data(),&remoteAddr,&localAddr)){
        qDebug() << "Init driver failed - Could not find network interface";
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
    int ret = downloader.initdriver(params);

    if (0 != ret){
        if(-2 == ret){
            qDebug() << "Init ETH driver - The ETH board has just jumped to eUpdater\n Connect again";
            *retString = "Init ETH driver - The ETH board has just jumped to eUpdater\n Connect again";
            // TODO DIALOG
        } else {
            qDebug() << "Init driver failed - Hardware busy or not connected?!";
            *retString = "Init driver failed - Hardware busy or not connected?!";
            // TODO DIALOG
        }
        mutex.unlock();
        return canBoards;
    }


    ret = downloader.initschede();

    if (ret == -1)
    {
        qDebug()  << "Communication error - No answers received (no boards found).";
        *retString = "Communication error - No answers received (no boards found).";
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
    gMNT.command_blink(EthMaintainer::ipv4OfAllSelected);
}

QString FirmwareUpdaterCore::getEthBoardInfo(int index)
{
    return QString("%1").arg(gMNT.boards_get()[index].getInfoOnEEPROM().c_str());
}

QString FirmwareUpdaterCore::getEthBoardAddress(int index)
{
    char board_ipaddr[16];
    ACE_UINT32 ip = ipv4toace(gMNT.boards_get()[index].getIPV4());
    sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
    return QString("%1").arg(board_ipaddr);
}

void FirmwareUpdaterCore::setEthBoardInfo(int index, QString newInfo)
{
    eOipv4addr_t address = gMNT.boards_get()[index].getIPV4();
    bool ret = gMNT.command_info32_set(address, newInfo.toLatin1().data());
    if(!ret){
        qDebug() << "setEthBoardInfo failed";
    }


    vector<string> vv = gMNT.command_info32_get(address);
    foreach (string v, vv) {
        qDebug() << v.c_str();
    }
    // TODO chiedere
    // it already sets it internally to commandInfo32Get()
    if(vv.size() > 0)
    {
        qDebug() << gMNT.boards_get()[index].getInfoOnEEPROM().c_str();
    }
}

void FirmwareUpdaterCore::setCanBoardInfo(int bus, int id, QString newInfo,QString ethAddress,QString *resultString)
{
    if(!ethAddress.isEmpty()){
        if(currentAddress != ethAddress){
            if(downloader.connected){
                downloader.stopdriver();
            }
            getCanBoardsFromEth(ethAddress,resultString);
        }
        downloader.change_board_info(bus, id, newInfo.toLatin1().data());
    }
}

bool FirmwareUpdaterCore::setCanBoardAddress(int bus, int id, int canType,QString newAddress,QString ethAddress,QString *resultString)
{
    if(!ethAddress.isEmpty()){
        if(currentAddress != ethAddress){
            if(downloader.connected){
                downloader.stopdriver();
            }
            getCanBoardsFromEth(ethAddress,resultString);
        }

        int new_val = newAddress.toInt();
        if (new_val <=0 || new_val> 15){
            qDebug() << "Error, new address out of range 0 - 15";
            return false;
        }

        if (new_val == id){
            qDebug() << "Error, same address set";
            return false;
        }

        downloader.change_card_address(bus, id, new_val,canType);
    }
}

bool FirmwareUpdaterCore::setEthBoardAddress(int index, QString newAddress)
{

    int ip1,ip2,ip3,ip4;
    sscanf(newAddress.toLatin1().data(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    if (ip1<0 || ip1>255 || ip2<0 || ip2>255 || ip3<0 || ip3>255 || ip4<0 || ip4>255){
        return false;
    }
    ACE_UINT32 iNewAddress=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;


    ACE_UINT32 address = ipv4toace(gMNT.boards_get()[index].getIPV4());
    ACE_UINT32 mask = 0xFFFFFF00;

    if(iNewAddress == (iNewAddress & mask)){ // checks new ip address is not a network address . For example x.y.z.w/24 x.y.z.0
        qDebug() << "Error Setting address";
        return false;
    }

    if((~mask) == (iNewAddress & (~mask))){ // checks new ip address is not a broadcast address . For example x.y.z.w/24 x.y.z.255
        qDebug() << "Error Setting address";
        return false;
    }

    if (iNewAddress == address){
        qDebug() << "Error, same address set";
        return false;
    }
    char old_addr[16];
    sprintf(old_addr,"%d.%d.%d.%d",(address>>24)&0xFF,(address>>16)&0xFF,(address>>8)&0xFF,address&0xFF);

    bool ret = gMNT.command_changeaddress(acetoipv4(address), acetoipv4(iNewAddress), true, true, true, true);
    return ret;



}


bool FirmwareUpdaterCore::uploadLoader(QString filename,QString *resultString)
{
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        qDebug() << "Error opening the selected file!";
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
        return true;
    }
    return false;
}


bool FirmwareUpdaterCore::uploadUpdater(QString filename,QString *resultString)
{
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        qDebug() << "Error opening the selected file!";
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
        return true;
    }
    return false;
}


bool FirmwareUpdaterCore::uploadCanApplication(QString filename,QString *resultString, QString ethAddress)
{
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
        return false;
    }

    QMap<int,int> canDevices;
    for (int i=0; i<downloader.board_list_size; i++)
    {
        if (downloader.board_list[i].selected==true)
        {
            canDevices.insertMulti(downloader.board_list[i].bus,i);
            downloader.board_list[i].selected = false;
            qDebug() << "FOUND SELECTED SCHEDA " << i << " ON BUS " << downloader.board_list[i].bus;
        }
    }

    int ret      = 0;
    int finished = 0;
    int busCount = canDevices.uniqueKeys().count();
    for(int k=0;k<busCount;k++){

        int bus = canDevices.uniqueKeys().at(k);

        if (downloader.open_file(filename.toLatin1().data())!=0){
            *resultString = "Error opening the selected file!";
            return false;
        }
        qDebug() << "FILE " << filename << " OPENED";
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
//            qDebug() << "SELECTING BOARD " << index << " OF BUS " << k;
//        }


        for(int i=0;i<indexes.count();i++){
            int index = indexes.at(i);
            downloader.board_list[index].selected = true;
            qDebug() << "SELECTING BOARD " << index << " OF BUS " << bus;
            if (downloader.board_list[index].status==BOARD_RUNNING){
                if (downloader.startscheda(bus,
                                           downloader.board_list[index].pid,
                                           downloader.board_list[index].eeprom,
                                           downloader.board_list[index].type)!=0){
                    *resultString = "Unable to start the board - Unable to send message 'start' or no answer received";
                    return false;
                } else {
                    qDebug() << "START SCHEDA  " << index << " ON BUS " << bus << " OK";
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
//                    qDebug() << "START SCHEDA  " << i << " OK";
//                    downloader.board_list[i].status=BOARD_WAITING;
//                }
//                download_type = downloader.board_list[i].type;
//                download_eeprom = downloader.board_list[i].eeprom;
//            }
//        }



        timer_start= yarp::os::Time::now();

        bool print00 = false, print25 = false, print50 = false, print75 = false, print99 = false;
        // Start the download for the selected boards
        do
        {
            ret = downloader.download_file(CanPacket::everyCANbus, 0x0F, download_type,download_eeprom);
            if (float(downloader.progress)/downloader.file_length/busCount >0.0  && print00==false)    {qDebug("downloading %s, 1%% done\n",filename.toLatin1().data()); print00=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.25 && print25==false)    {qDebug("downloading %s, 25%% done\n",filename.toLatin1().data()); print25=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.50 && print50==false)    {qDebug("downloading %s, 50%% done\n",filename.toLatin1().data()); print50=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.75 && print75==false)    {qDebug("downloading %s, 75%% done\n",filename.toLatin1().data()); print75=true;}
            if (float(downloader.progress)/downloader.file_length/busCount >0.99 && print99==false)    {qDebug("downloading %s, finished!\n",filename.toLatin1().data()); print99=true;}

            if (ret==1){
                updateProgress(float(downloader.progress)/downloader.file_length/busCount);
            }
            if (ret==-1){
                qDebug() << "Fatal Error during download, terminate";
                *resultString = "Fatal Error during download, terminate";
                finished = 1;
            }
            if (ret==0){
                qDebug() << "Download terminated";
                *resultString = "Download terminated";
                finished = 1;
            }

        }
        while (finished!=1);

        // End the download for the selected boards
        int errors =0;
        if(downloader.stopscheda(CanPacket::everyCANbus, 15) != 0){
            qDebug() << "ERROR STOPPING SCHEDA";
        }else{
            qDebug() << "scheda stopped";
        }

        foreach (int index, indexes) {
            downloader.board_list[index].selected = false;
            qDebug() << "DE-SELECTING BOARD " << index << " OF BUS " << bus;
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
        return true;
    }
    else
    {
        //*resultString = "Error during file transfer";

        updateProgress(1.0);
        return false;
    }

    return true;
}

bool FirmwareUpdaterCore::uploadEthApplication(QString filename,QString *resultString)
{
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        qDebug() << "Error opening the selected file!";
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
        return true;
    }
    return false;
}






void FirmwareUpdaterCore::restartEthBoards()
{
    gMNT.command_restart(EthMaintainer::ipv4OfAllSelected);
}


void FirmwareUpdaterCore::bootFromApplication()
{
    gMNT.command_def2run(EthMaintainer::ipv4OfAllSelected, eApplication, false, false);
}

void FirmwareUpdaterCore::bootFromUpdater()
{
    gMNT.command_def2run(EthMaintainer::ipv4OfAllSelected, eUpdater, false, false);
}
