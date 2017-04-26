#include "mainwindow.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QFileInfo>
#include <qdebug.h>
#include <QDir>

#include "firmwareupdatercore.h"

#undef UPDATER_RELEASE

#ifdef Q_OS_WIN
#include <Windows.h>
#endif

#define MY_ADDR "10.0.1.104"
#define MY_PORT 3333

int verbosity = 1;

using namespace yarp::os;


bool checkApplicationLock();
void removeApplicationLock();
void printCanDevices(QList<sBoard> canBoards);
int printSecondLevelDevices(FirmwareUpdaterCore*,QString device,QString id);
int printThirdLevelDevices(FirmwareUpdaterCore*,QString device,QString id,QString board, bool forceMaintenance, bool forceApplication);
int programEthDevice(FirmwareUpdaterCore*,QString device,QString id,QString board,QString file);
int programCanDevice(FirmwareUpdaterCore*, QString device, QString id, QString board, QString canLine, QString canId, QString file, bool eraseEEprom);
int setBoardToApplication(FirmwareUpdaterCore *core,QString device,QString id,QString board);
int setBoardToMaintenance(FirmwareUpdaterCore *core,QString device,QString id,QString board);
int eraseEthEEprom(FirmwareUpdaterCore *core,QString device,QString id,QString board);

int main(int argc, char *argv[])
{
    Network yarp;
    QApplication a(argc, argv);

    FirmwareUpdaterCore core;

#ifdef UPDATER_RELEASE
    if(!checkApplicationLock()){
        yDebug() << "The application is busy";
        qDebug() << "The application is busy. Check if an instance is already running, or if it is in zombie state. If none of them ... remove the file .firmwareUpdater.singleton";
        return 0;
    }
#endif

    QCommandLineParser parser;
    parser.addVersionOption();
    parser.setApplicationDescription("Firmware Updater Help");
    parser.addHelpOption();

    QCommandLineOption noGuiOption(QStringList() << "g" << "nogui", "The application starts in console mode");
    QCommandLineOption adminOption(QStringList() << "a" << "admin", "The application starts in admin mode");
    QCommandLineOption iniFileOption(QStringList() << "f" << "from", "Override the default ini file","config","firmwareupdater.ini");
    QCommandLineOption addressOption(QStringList() << "s" << "address", "Override the default address","address",MY_ADDR);
    QCommandLineOption portOption(QStringList() << "p" << "port", "Override the default port","port","3333");
    QCommandLineOption discoverOption(QStringList() << "d" << "discover", "Discover devices");
    QCommandLineOption deviceOption(QStringList() << "e" << "device", "Choose Device (i.e. ETH or CFW2_CAN, ESD_CAN...)","device","");
    QCommandLineOption idOption(QStringList() << "i" << "id", "Choose a device id (i.e. eth1 or 1-2-3...)","id","");
    QCommandLineOption boardOption(QStringList() << "t" << "eth_board", "Choose a device board (i.e. 10.0.0.1)","eth_board","");
    QCommandLineOption programOption(QStringList() << "r" << "program", "Program devices");
    QCommandLineOption fileOption(QStringList() << "l" << "file", "Path to a firmware file","file","");
    QCommandLineOption ethCanLineOption(QStringList() << "c" << "can_line", "Select a can line","can_line","");
    QCommandLineOption ethCanIdOption(QStringList() << "n" << "can_id", "Select a can id","can_id","");
    QCommandLineOption ethForceMaintenance(QStringList() << "m" << "force-eth-maintenance", "Force the board to go in maintenace mode","");
    QCommandLineOption ethForceApplication(QStringList() << "o" << "force-eth-application", "Force the board to go in application mode","");
    QCommandLineOption eraseEEpromOption(QStringList() << "p" << "erase_eeprom" << "Erase EEprom","");
    QCommandLineOption verbosityOption(QStringList() << "x" << "verbosity", "Choose a verbosity level [0, 1]","verbosity","");


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
    parser.addOption(ethForceMaintenance);
    parser.addOption(ethForceApplication);
    parser.addOption(eraseEEpromOption);
    parser.addOption(verbosityOption);

    parser.process(a);

    bool noGui = parser.isSet(noGuiOption);
    bool adminMode = parser.isSet(adminOption);
    QString iniFile = parser.value(iniFileOption);
    QString address = MY_ADDR;
    bool bPrintUsage=false;
    int port = MY_PORT;

    if(parser.isSet(verbosityOption))
    {
        QString vv = parser.value(verbosityOption);
        verbosity = vv.toInt();
    }


    if(parser.isSet(addressOption)){
        address = parser.value(addressOption);
    }else{
        if(verbosity >= 1) qDebug() << "Using default address " << MY_ADDR;
        //yDebug() << "Using default address " << MY_ADDR;
        bPrintUsage=true;
    }

    if(parser.isSet(portOption)){
        port = parser.value(portOption).toInt();
    }else{
        if(verbosity >= 1) qDebug() << "Using default port " << MY_PORT;
        //yDebug() << "Using default port " << MY_PORT;
        bPrintUsage=true;
    }

    if (bPrintUsage){
        //qDebug() << "Usage: " << argv[0] << " --port n --address xxx.xxx.xxx.xxx\n";
    }





    ResourceFinder rf;
    if(verbosity >= 1)
    {
        rf.setVerbose();
    }
    rf.setDefaultContext("firmwareUpdater");
    rf.setDefaultConfigFile(iniFile.toLatin1().data());

    if(!rf.configure(argc, argv)){
        return false;
    }

    if(!core.init(rf, port, address, verbosity)){
        return -1;
    }
    int ret = 1;
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


        bool forceMaintenance = parser.isSet(ethForceMaintenance);
        bool forceApplication = parser.isSet(ethForceApplication);
        bool eraseEEprom = parser.isSet(eraseEEpromOption);

        core.setVerbosity(verbosity);


        if(discover){
            if(device.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need a device to be set";
            }else if(id.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need an id to be set";
            }else{

                if(board.isEmpty()){
                    ret = printSecondLevelDevices(&core,device,id);
                }else{
                    ret = printThirdLevelDevices(&core,device,id,board,forceMaintenance,forceApplication);
                }
            }
        }else if(program){
            if(device.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need a device to be set";
            }else if(id.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need an id to be set";
            }else if(board.isEmpty() && device.contains("ETH")){
                if(verbosity >= 1) qDebug() << "Need a board to be set";
            }else if(file.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need a file path to be set";
            }else if(canLine.isEmpty() && canId.isEmpty()){
                ret = programEthDevice(&core,device,id,board,file);
                if(eraseEEprom && ret == 0){
                    ret = eraseEthEEprom(&core,device,id,board);
                }
            }else{
                if(canLine.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a can line to be set";
                } else if(canId.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a can id to be set";
                }else{
                    ret = programCanDevice(&core,device,id,board,canLine,canId,file,eraseEEprom);
                }
            }
        }else if(forceApplication || forceMaintenance){
            if(device.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need a device to be set";
            }else if(id.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need an id to be set";
            }else if(board.isEmpty()){
                if(verbosity >= 1) qDebug() << "Need a board to be set";
            }else{
                if(forceApplication){
                    ret = setBoardToApplication(&core,device,id,board);
                }else {
                    ret = setBoardToMaintenance(&core,device,id,board);
                }

            }
            if(eraseEEprom && ret == 0){
                ret = eraseEthEEprom(&core,device,id,board);
            }
        }
    }



#ifdef UPDATER_RELEASE
    removeApplicationLock();
#endif

    return ret;
}


/**************************************************/

int eraseEthEEprom(FirmwareUpdaterCore *core,QString device,QString id,QString board)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            core->setSelectedEthBoard(board,true);
            core->eraseEthEprom();
        }
    }
    return 0;
}

int setBoardToApplication(FirmwareUpdaterCore *core,QString device,QString id,QString board)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            core->setSelectedEthBoard(board,true);
            core->goToApplication();
        }
    }
    return 0;
}

int setBoardToMaintenance(FirmwareUpdaterCore *core,QString device,QString id,QString board)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            core->setSelectedEthBoard(board,true);
            core->goToMaintenance();
        }
    }

    return 0;
}


int programCanDevice(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,QString file,bool eraseEEprom)
{
    QString retString;
    if(device.contains("ETH")){
        int boards = core->connectTo(device,id);
        if(boards > 0){
            char board_ipaddr[16];
            for(int i=0;i<core->getEthBoardList().size();i++){
                EthBoard ethBoard = core->getEthBoardList()[i];
                snprintf(board_ipaddr, sizeof(board_ipaddr), "%s", ethBoard.getIPV4string().c_str());

                if(board.contains(board_ipaddr)){
                    core->setSelectedEthBoard(i,true);
                    QList <sBoard> canBoards = core->getCanBoardsFromEth(board,&retString,canLine.toInt(),true);
                    if(canBoards.count() > 0){
                        int selectedCount = 0;
                        for(int j=0;j<canBoards.count();j++){
                            sBoard b = canBoards.at(j);
                            if(b.bus == canLine.toInt() && b.pid == canId.toInt()){
                                b.selected = true;
                                b.eeprom = eraseEEprom;
                                canBoards.replace(j,b);
                                selectedCount++;
                            }

                        }
                        if(selectedCount > 0){
                            core->setSelectedCanBoards(canBoards,board);
                            bool ret = core->uploadCanApplication(file, &retString, eraseEEprom, board);
                            if(verbosity >= 1) qDebug() << retString;
                            return ret ? 0 : -1;
                        }else{
                            if(verbosity >= 1) qDebug() << "No board selected";
                            return -1;
                        }
                    }else{
                        if(verbosity >= 1) qDebug() << retString;
                        return -1;
                    }

                }
            }

        }
    }else{
        QList <sBoard> canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        if(canBoards.count() > 0){
            int selectedCount = 0;
            for(int j=0;j<canBoards.count();j++){
                sBoard b = canBoards.at(j);
                if(b.bus == canLine.toInt() && b.pid == canId.toInt()){
                    b.selected = true;
                    b.eeprom = eraseEEprom;
                    canBoards.replace(j,b);
                    selectedCount++;
                }
            }
            if(selectedCount > 0){
                core->setSelectedCanBoards(canBoards,device,id.toInt());
                bool ret = core->uploadCanApplication(file, &retString, eraseEEprom, device, id.toInt());
                if(verbosity >= 1) qDebug() << retString;
                return ret ? 0 : -1;
            }else{
                if(verbosity >= 1) qDebug() << "No board selected";
                return -1;
            }
        }else{
            if(verbosity >= 1) qDebug() << retString;
            return -1;
        }
    }
    return -1;
}


int programEthDevice(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString file)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            char board_ipaddr[16];
            for(int i=0;i<core->getEthBoardList().size();i++){
                EthBoard ethBoard = core->getEthBoardList()[i];
                snprintf(board_ipaddr, sizeof(board_ipaddr), "%s", ethBoard.getIPV4string().c_str());

                if(board.contains(board_ipaddr)){
                    core->setSelectedEthBoard(i,true);
                    QString resultString;
                    bool b = core->uploadEthApplication(file,&resultString);
                    if(!b){
                        if(verbosity >= 1) qDebug() << resultString;
                        return -1;
                    }else{
                        if(verbosity >= 1) qDebug() << "Update Done";
                        return 0;
                    }
                    break;
                }
            }
        }

    }else{
        if(verbosity >= 1) qDebug() << "No boards found";
    }
    return -1;

}

int printSecondLevelDevices(FirmwareUpdaterCore *core,QString device,QString id)
{
    if(device.contains("ETH")){
        int boards = core->connectTo(device,id);
        if(boards > 0){

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


            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";
            for(int i=0;i<core->getEthBoardList().size();i++){
                EthBoard board = core->getEthBoardList()[i];

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

                if(verbosity >= 1) qDebug() << "************** Device " << i << " ******************";
                if(verbosity >= 1) qDebug() << "Ip:        "<< board_ipaddr;
                if(verbosity >= 1) qDebug() << "Mac:       "<< board_mac;
                if(verbosity >= 1) qDebug() << "Version:   "<< board_version;
                if(verbosity >= 1) qDebug() << "Type:      "<< board_type;
                if(verbosity >= 1) qDebug() << "Process:   "<< running_process;
                if(verbosity >= 1) qDebug() << "Info:      "<< board_info;
                if(verbosity >= 1) qDebug() << "Date:      "<< board_date;
                if(verbosity >= 1) qDebug() << "Built:     "<< board_built;
                if(verbosity >= 1) qDebug() << "\n";

            }
            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

        }
    }else{
        QString retString;
        QList <sBoard> canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        if(canBoards.count() <= 0){
            if(verbosity >= 1) qDebug() <<  retString;
        }else{
            printCanDevices(canBoards);
        }
    }
    return 0;
}

int printThirdLevelDevices(FirmwareUpdaterCore *core,QString device,QString id,QString board, bool forceMaintenance, bool forceApplication)
{
    int boards = core->connectTo(device,id);
    if(boards > 0){
        if(device.contains("ETH")){
            if(forceMaintenance){
                core->setSelectedEthBoard(board,true);
                core->goToMaintenance();
            }else if(forceApplication){
                core->setSelectedEthBoard(board,true);
                core->goToApplication();
            }
            if(core->isBoardInMaintenanceMode(board)){
                QString retString;
                QList <sBoard> canBoards = core->getCanBoardsFromEth(board,&retString);
                if(canBoards.count() <= 0){
                    if(verbosity >= 1) qDebug() <<  retString;
                }else{
                    printCanDevices(canBoards);
                }

            }else{
                if(verbosity >= 1) qDebug() << "You have to put the device in maintenace mode to perform this operation.";
            }

        }
    }else{
        if(verbosity >= 1) qDebug() << "No boards Found";
    }

    return 0;
}

void printCanDevices(QList<sBoard> canBoards)
{
    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";
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

        snprintf(board_type, sizeof(board_type), "%s", eoboards_type2string2((eObrd_type_t)board.type, eobool_true));

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

        if(verbosity >= 1) qDebug() << "************** Board " << i << " ******************";
        if(verbosity >= 1) qDebug() << "Type:              " << board_type;
        if(verbosity >= 1) qDebug() << "Id:                " << board.pid;
        if(verbosity >= 1) qDebug() << "Address:           " << "CAN_" << board.bus;
        if(verbosity >= 1) qDebug() << "Process:           " << board_process;
        if(verbosity >= 1) qDebug() << "Status:            " << board_status;
        if(verbosity >= 1) qDebug() << "Info:              " << board_add_info;
        if(verbosity >= 1) qDebug() << "Firmware Version:  " << board_firmware_version;
        if(verbosity >= 1) qDebug() << "Serial:            " << board_serial;
        if(verbosity >= 1) qDebug() << "Protocol:          " << board_protocol;
        if(verbosity >= 1) qDebug() << "\n";
    }
    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";
}

bool checkApplicationLock()
{
    QString tempFile;
#ifdef Q_OS_WIN
    tempFile = QDir::homePath() + "/firmwareUpdater.singleton";
#else
    tempFile = QDir::homePath() + "/.firmwareUpdater.singleton";
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
    LPCSTR a = (const char*)fInfo.filePath().utf16();
    BOOL b = SetFileAttributes(a,FILE_ATTRIBUTE_HIDDEN);
#endif

    return true;

}


void removeApplicationLock()
{
    QString tempFile;
#ifdef Q_OS_WIN
    tempFile = QDir::homePath() + "/firmwareUpdater.singleton";
#else
    tempFile = QDir::homePath() + "/.firmwareUpdater.singleton";
#endif

    QFileInfo fInfo(tempFile);
    if(!fInfo.exists()){
        return;
    }

    QFile f(tempFile);
    f.remove();

}
