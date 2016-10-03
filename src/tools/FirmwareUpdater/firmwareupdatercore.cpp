#include "firmwareupdatercore.h"
#include <qdebug.h>

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

    std::string addr(address.toLatin1().data());
    if (!gUpdater.create((quint16)port,addr)){
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

void FirmwareUpdaterCore::jumpToUpdater()
{
    gUpdater.cmdJumpUpd();
}

QList<QPair<QString,QVariant> > FirmwareUpdaterCore::getDevices()
{
    return devices;
}

void FirmwareUpdaterCore::disconnectFrom(QString device, QString id)
{
    if(!device.isEmpty() && !id.isEmpty() && device.contains("ETH")){
        gUpdater.getBoardList().empty();
        yDebug() << "empty eth devices";
        qDebug() << "empty eth devices";
    }
}

int FirmwareUpdaterCore::connectTo(QString device, QString id)
{
    if(!device.isEmpty() && !id.isEmpty() && device.contains("ETH")){
        int num = gUpdater.cmdDiscover();
        yDebug() << "Found " << num << " devices";
        qDebug() << "Found " << num << " devices";
        return num;
    }
    return 0;
}

BoardList &FirmwareUpdaterCore::getEthBoardList()
{
    return gUpdater.getBoardList();
}

void FirmwareUpdaterCore::setSelectedEthBoard(int index,bool selected)
{
    if(gUpdater.getBoardList().size() > index){
        gUpdater.getBoardList()[index].mSelected = selected;
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


QString FirmwareUpdaterCore::getMoreDetails(ACE_UINT32 address)
{
    return QString("%1").arg(gUpdater.cmdGetMoreInfo(false,address).c_str());
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
    gUpdater.cmdBlink();
}

QString FirmwareUpdaterCore::getEthBoardInfo(int index)
{
    return QString("%1").arg(gUpdater.getBoardList()[index].mInfo32.c_str());
}

QString FirmwareUpdaterCore::getEthBoardAddress(int index)
{
    char board_ipaddr[16];
    ACE_UINT32 ip = gUpdater.getBoardList()[index].mAddress;
    sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
    return QString("%1").arg(board_ipaddr);
}

void FirmwareUpdaterCore::setEthBoardInfo(int index, QString newInfo)
{
    ACE_UINT32 address=gUpdater.getBoardList()[index].mAddress;
    gUpdater.cmdInfo32Set(newInfo.toLatin1().data(), address);
    vector<string> vv = gUpdater.cmdInfo32Get(address);
    if(vv.size() > 0){
        gUpdater.getBoardList()[index].mInfo32 = vv[0];
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


    ACE_UINT32 address=gUpdater.getBoardList()[index].mAddress;
    ACE_UINT32 mask=gUpdater.getBoardList()[index].mMask;

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

    gUpdater.cmdChangeAddress(iNewAddress, address);
    return true;



}


bool FirmwareUpdaterCore::uploadLoader(QString filename,QString *resultString)
{
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        qDebug() << "Error opening the selected file!";
        return false;
    }
    uint32_t addr = 0; // all selected
    //addr = (10 << 24) | (0 << 16) | (1 << 8) | (2); test only one board w/ address 10.0.1.2
    std::string result=gUpdater.cmdProgram(programFile,*(int*)&EthUpdater::partition_LOADER,updateProgressCallback, addr);

    fclose(programFile);
    *resultString = QString("%1").arg(result.c_str());
    return true;
}


bool FirmwareUpdaterCore::uploadUpdater(QString filename,QString *resultString)
{
    FILE *programFile=fopen(filename.toLatin1().data(),"r");
    if (!programFile){
        //TODO ERROR
        qDebug() << "Error opening the selected file!";
        return false;
    }
    uint32_t addr = 0; // all selected
    //addr = (10 << 24) | (0 << 16) | (1 << 8) | (2); test only one board w/ address 10.0.1.2
    std::string result=gUpdater.cmdProgram(programFile,*(int*)&EthUpdater::partition_UPDATER,updateProgressCallback, addr);

    fclose(programFile);
    *resultString = QString("%1").arg(result.c_str());
    return true;
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

    // Start the download for the selected boards
    for (i=0; i<downloader.board_list_size; i++){
        if (downloader.board_list[i].status==BOARD_RUNNING && downloader.board_list[i].selected==true){
            if (downloader.startscheda(downloader.board_list[i].bus, downloader.board_list[i].pid, downloader.board_list[i].eeprom, downloader.board_list[i].type)!=0){
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

    bool print00 = false, print25 = false, print50 = false, print75 = false, print99 = false;
    // Start the download for the selected boards
    do
    {
        ret = downloader.download_file(CanPacket::everyCANbus, 0x0F, download_type,download_eeprom);
        if (float(downloader.progress)/downloader.file_length >0.0  && print00==false)    {qDebug("downloading %s, 1%% done\n",filename.toLatin1().data()); print00=true;}
        if (float(downloader.progress)/downloader.file_length >0.25 && print25==false)    {qDebug("downloading %s, 25%% done\n",filename.toLatin1().data()); print25=true;}
        if (float(downloader.progress)/downloader.file_length >0.50 && print50==false)    {qDebug("downloading %s, 50%% done\n",filename.toLatin1().data()); print50=true;}
        if (float(downloader.progress)/downloader.file_length >0.75 && print75==false)    {qDebug("downloading %s, 75%% done\n",filename.toLatin1().data()); print75=true;}
        if (float(downloader.progress)/downloader.file_length >0.99 && print99==false)    {qDebug("downloading %s, finished!\n",filename.toLatin1().data()); print99=true;}

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

        //        // Update the progress bar
        //        if (prompt_version==false && downloader.progress % 50 == 0)
        //        {
        //            gtk_progress_bar_set_fraction ((GtkProgressBar*) progress_bar, 0);
        //            gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
        //            gtk_widget_draw(treeview, NULL);
        //            gtk_main_iteration_do (false);
        //        }
    }
    while (finished!=1);
    timer_end= yarp::os::Time::now();

    // End the download for the selected boards
    int errors =0;
    downloader.stopscheda(CanPacket::everyCANbus, 15);



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
    uint32_t addr = 0; // all selected
    //addr = (10 << 24) | (0 << 16) | (1 << 8) | (2); test only one board w/ address 10.0.1.2
    std::string result=gUpdater.cmdProgram(programFile,*(int*)&EthUpdater::partition_APPLICATION,updateProgressCallback, addr);

    fclose(programFile);
    *resultString = QString("%1").arg(result.c_str());
    return true;
}






void FirmwareUpdaterCore::restartEthBoards()
{
    gUpdater.cmdRestart();
}


void FirmwareUpdaterCore::bootFromApplication()
{
    qDebug() << "PRE bootFromApplication -- Actual Selected devices";
    for(int i=0;i<gUpdater.getBoardList().size();i++){
        qDebug() << " -" << i <<  " " << gUpdater.getBoardList()[i].mSelected;
    }
    gUpdater.cmdSetDEF2RUN(eApplication);

    qDebug() << "POST bootFromApplication -- Actual Selected devices";
    for(int i=0;i<gUpdater.getBoardList().size();i++){
        qDebug() << " -" << i <<  " " << gUpdater.getBoardList()[i].mSelected;
    }

}

void FirmwareUpdaterCore::bootFromUpdater()
{
    gUpdater.cmdSetDEF2RUN(eUpdater);
}
