#include "mainwindow.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QFileInfo>
#include <qdebug.h>

#include "firmwareupdatercore.h"

#ifdef Q_OS_WIN
#include <Windows.h>
#endif

#define MY_ADDR "10.0.1.104"
#define MY_PORT 3333

using namespace yarp::os;


bool checkApplicationLock();
void removeApplicationLock();
void printSecondLevelDevices(FirmwareUpdaterCore*,QString device,QString id);
void printThirdLevelDevices(FirmwareUpdaterCore*,QString device,QString id,QString board);
void programSecondLevelDevice(FirmwareUpdaterCore*,QString device,QString id,QString board,QString file);
void programThirdLevelDevice(FirmwareUpdaterCore*,QString device,QString id,QString board,QString canLine, QString canId,QString file);

int main(int argc, char *argv[])
{
    Network yarp;
    QApplication a(argc, argv);

    FirmwareUpdaterCore core;

#ifdef UPDATER_RELEASE
    if(!checkApplicationLock()){
        yDebug() << "The application is busy";
        qDebug() << "The application is busy";
        return 0;
    }
#endif

    QCommandLineParser parser;
    parser.addVersionOption();
    parser.setApplicationDescription("Firmware Updater Help");
    parser.addHelpOption();

    QCommandLineOption noGuiOption(QStringList() << "" << "nogui", "The application starts in console mode");
    QCommandLineOption adminOption(QStringList() << "" << "admin", "The application starts in admin mode");
    QCommandLineOption iniFileOption(QStringList() << "" << "from", "Override the default ini file","config","firmwareupdater.ini");
    QCommandLineOption addressOption(QStringList() << "" << "address", "Override the default address","address",MY_ADDR);
    QCommandLineOption portOption(QStringList() << "" << "port", "Override the default port","port","3333");
    QCommandLineOption discoverOption(QStringList() << "" << "discover", "Discover devices");
    QCommandLineOption deviceOption(QStringList() << "" << "device", "Choose Device (i.e. ETH)","device","ETH");
    QCommandLineOption idOption(QStringList() << "" << "id", "Choose a device id (i.e. eth1)","id","eth1");
    QCommandLineOption boardOption(QStringList() << "" << "board", "Choose a device board (i.e. 10.0.0.1)","board","");
    QCommandLineOption programOption(QStringList() << "" << "program", "Program devices");
    QCommandLineOption fileOption(QStringList() << "" << "file", "Path to a firmware file","file","");
    QCommandLineOption ethCanLineOption(QStringList() << "" << "eth_can_line", "Select a can line","eth_can_line","");
    QCommandLineOption ethCanIdOption(QStringList() << "" << "eth_can_id", "Select a can id","eth_can_id","");


    parser.addOption(noGuiOption);
    parser.addOption(adminOption);
    parser.addOption(iniFileOption);
    parser.addOption(addressOption);
    parser.addOption(portOption);
    parser.addOption(discoverOption);
    parser.addOption(deviceOption);
    parser.addOption(idOption);
    parser.addOption(boardOption);
    parser.addOption(programOption);
    parser.addOption(fileOption);
    parser.addOption(ethCanLineOption);
    parser.addOption(ethCanIdOption);

    parser.process(a);

    bool noGui = parser.isSet(noGuiOption);
    bool adminMode = parser.isSet(adminOption);
    QString iniFile = parser.value(iniFileOption);
    QString address = MY_ADDR;
    bool bPrintUsage=false;
    int port = MY_PORT;


    if(parser.isSet(addressOption)){
        address = parser.value(addressOption);
    }else{
        qDebug() << "Using default address " << MY_ADDR;
        yDebug() << "Using default address " << MY_ADDR;
        bPrintUsage=true;
    }

    if(parser.isSet(portOption)){
        port = parser.value(portOption).toInt();
    }else{
        qDebug() << "Using default port " << MY_PORT;
        yDebug() << "Using default port " << MY_PORT;
        bPrintUsage=true;
    }

    if (bPrintUsage){
        qDebug() << "Usage: " << argv[0] << " --port n --address xxx.xxx.xxx.xxx\n";
    }





    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("firmwareUpdater");
    rf.setDefaultConfigFile(iniFile.toLatin1().data());

    if(!rf.configure(argc, argv)){
        return false;
    }

    if(!core.init(rf, port, address)){
        return -1;
    }
    int ret = 0;
    MainWindow w(&core,adminMode);
    if(!noGui){
        w.show();
        ret = a.exec();
    }else{
        bool discover = parser.isSet(discoverOption);
        bool program = parser.isSet(programOption);
        QString device = parser.value(deviceOption);
        QString id = parser.value(idOption);
        QString board = parser.value(boardOption);
        QString file = parser.value(fileOption);
        QString canLine = parser.value(ethCanLineOption);
        QString canId = parser.value(ethCanIdOption);

        if(discover){
            if(device.isEmpty()){
                qDebug() << "Need a device to be set";
            }else if(id.isEmpty()){
                qDebug() << "Need an id to be set";
            }else{

                if(board.isEmpty()){
                    printSecondLevelDevices(&core,device,id);
                }else{
                    printThirdLevelDevices(&core,device,id,board);
                }
            }
        }else if(program){
            if(device.isEmpty()){
                qDebug() << "Need a device to be set";
            }else if(id.isEmpty()){
                qDebug() << "Need an id to be set";
            }else if(board.isEmpty()){
                qDebug() << "Need a board to be set";
            }else if(file.isEmpty()){
                qDebug() << "Need a file path to be set";
            }else if(canLine.isEmpty() && canId.isEmpty()){
                programSecondLevelDevice(&core,device,id,board,file);
            }else{
                if(canLine.isEmpty()){
                    qDebug() << "Need a can line to be set";
                } else if(canId.isEmpty()){
                    qDebug() << "Need a can id to be set";
                }else{
                    programThirdLevelDevice(&core,device,id,board,canLine,canId,file);
                }
            }
        }
    }



#ifdef UPDATER_RELEASE
    removeApplicationLock();
#endif

    return ret;
}


/**************************************************/
void programThirdLevelDevice(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,QString file)
{
    QString retString;
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            char board_ipaddr[16];
            for(int i=0;i<core->getEthBoardList().size();i++){
                ACE_UINT32 ip=core->getEthBoardList()[i].mAddress;
                sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);

                if(board.contains(board_ipaddr)){
                    core->setSelectedEthBoard(i,true);
                    QList <sBoard> canBoards = core->getCanBoardsFromEth(board,&retString,canLine.toInt());
                    if(canBoards.count() > 0){
                        for(int j=0;j<canBoards.count();j++){
                            sBoard b = canBoards.at(j);
                            if(b.bus == canLine.toInt() && b.pid == canId.toInt()){
                                 core->setSelectedCanBoard(j,true,board);
                                 core->uploadCanApplication(file,&retString,board);
                                 qDebug() << retString;
                                 break;
                            }
                        }
                    }else{
                        qDebug() << retString;
                    }

                }
            }
        }
    }
}


void programSecondLevelDevice(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString file)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            char board_ipaddr[16];
            for(int i=0;i<core->getEthBoardList().size();i++){
                ACE_UINT32 ip=core->getEthBoardList()[i].mAddress;
                sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);

                if(board.contains(board_ipaddr)){
                    core->setSelectedEthBoard(i,true);
                    QString resultString;
                    bool b = core->uploadEthApplication(file,&resultString);
                    if(!b){
                        qDebug() << resultString;
                    }else{
                        qDebug() << "Update Done";
                    }
                    break;
                }
            }
        }

    }

}

void printSecondLevelDevices(FirmwareUpdaterCore *core,QString device,QString id)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            char board_ipaddr[16];
            char board_mac[32];

            char board_version[16];
            char board_date[24];
            char board_built[24];
            char board_type[24];
            char running_process[24];
            char board_info[32];

            qDebug() << "-------------------------------------------------------------";
            for(int i=0;i<core->getEthBoardList().size();i++){
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

                qDebug() << "************** Device " << i << " ******************";
                qDebug() << "Ip:        "<< board_ipaddr;
                qDebug() << "Mac:       "<< board_mac;
                qDebug() << "Version:   "<< board_version;
                qDebug() << "Type:      "<< board_type;
                qDebug() << "Process:   "<< running_process;
                qDebug() << "Info:      "<< board_info;
                qDebug() << "Date:      "<< board_date;
                qDebug() << "Built:     "<< board_built;
                qDebug() << "\n";

            }
            qDebug() << "-------------------------------------------------------------";
        }
    }
    core->disconnectFrom(device,id);
}

void printThirdLevelDevices(FirmwareUpdaterCore *core,QString device,QString id,QString board)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            QString retString;
            QList <sBoard> canBoards = core->getCanBoardsFromEth(board,&retString);
            if(canBoards.count() <= 0){
                qDebug() <<  retString;
            }else{
                qDebug() << "-------------------------------------------------------------";
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

                    qDebug() << "************** Board " << i << " ******************";
                    qDebug() << "Type:              " << board_type;
                    qDebug() << "Id:                " << board.pid;
                    qDebug() << "Address:           " << "CAN_" << board.bus;
                    qDebug() << "Process:           " << board_process;
                    qDebug() << "Status:            " << board_status;
                    qDebug() << "Info:              " << board_add_info;
                    qDebug() << "Firmware Version:  " << board_firmware_version;
                    qDebug() << "Serial:            " << board_serial;
                    qDebug() << "Protocol:          " << board_protocol;
                    qDebug() << "\n";
                }
                qDebug() << "-------------------------------------------------------------";
            }

        }
    }else{
        qDebug() << "No boards Found";
    }
    core->disconnectFrom(device,id);
}

bool checkApplicationLock()
{
    QString tempFile;
#ifdef Q_OS_WIN
    tempFile = QApplication::applicationDirPath() + "/firmwareUpdater.singletone";
#else
    tempFile = QApplication::applicationDirPath() + "/.firmwareUpdater.singletone";
#endif

    QFileInfo fInfo(tempFile);
    if(fInfo.exists()){
        return false;
    }

    QFile f(tempFile);
    f.open(QIODevice::WriteOnly);
    f.write("busy");
    f.flush();
    f.close();

#ifdef Q_OS_WIN
    LPCWSTR a = (const wchar_t*)fInfo.filePath().utf16();
    BOOL b = SetFileAttributes(a,FILE_ATTRIBUTE_HIDDEN);
#endif

    return true;

}


void removeApplicationLock()
{
    QString tempFile;
#ifdef Q_OS_WIN
    tempFile = QApplication::applicationDirPath() + "/firmwareUpdater.singletone";
#else
    tempFile = QApplication::applicationDirPath() + "/.firmwareUpdater.singletone";
#endif

    QFileInfo fInfo(tempFile);
    if(!fInfo.exists()){
        return;
    }

    QFile f(tempFile);
    f.remove();

}
