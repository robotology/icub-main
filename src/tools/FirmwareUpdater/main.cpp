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

enum action_t
{
    action_impossible = -1,
    action_none = 0,
    action_discover = 1,
    action_verify = 2,
    action_program = 3,
    action_forcemaintenance = 4,
    action_forceapplication = 5,
    action_query = 6,
    action_loaddatfile = 7,
    action_setstrainsn = 8,
    action_setstraingainsoffsets = 9,
    action_getcanboardversion = 10,
    action_savedatfile = 11

};


bool checkApplicationLock();
void removeApplicationLock();
void printCanDevices(QList<sBoard> canBoards, QString onIPboard, bool slimprint);
int printSecondLevelDevices(FirmwareUpdaterCore*,QString device,QString id, bool slimprint);
int printThirdLevelDevices(FirmwareUpdaterCore*,QString device,QString id,QString board, bool forceMaintenance, bool forceApplication, bool slimprint);
int programEthDevice(FirmwareUpdaterCore*,QString device,QString id,QString board,QString file);
int programCanDevice(FirmwareUpdaterCore*, QString device, QString id, QString board, QString canLine, QString canId, QString file, bool eraseEEprom);
int setBoardToApplication(FirmwareUpdaterCore *core,QString device,QString id,QString board);
int setBoardToMaintenance(FirmwareUpdaterCore *core,QString device,QString id,QString board);
//int eraseEthEEprom(FirmwareUpdaterCore *core,QString device,QString id,QString board);

int verifyOnSecondLevel(FirmwareUpdaterCore *core,QString device,QString id, const QString &targetIPaddr, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers);
int verifyOnSecondLevel_ETHboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetIPaddr, const QString &targetFWvers);
int verifyOnSecondLevel_CANboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers);
int verifyCanDevices(QList<sBoard> canBoards, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers);
int verifyOnThirdLevel_CANunderETH(FirmwareUpdaterCore *core, QString device, QString id, QString board, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers);

//int queryOnSecondLevel(FirmwareUpdaterCore *core,QString device,QString id, const QString &targetIPaddr, const QString &targetCANline, const QString &targetCANaddr);
int queryOnSecondLevel_ETHboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetIPaddr);
int queryOnSecondLevel_CANboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetCANline, const QString &targetCANaddr);
int queryCanDevices(QList<sBoard> canBoards, const QString onIPboard, const QString &targetCANline, const QString &targetCANaddr);
int queryOnThirdLevel_CANunderETH(FirmwareUpdaterCore *core, QString device, QString id, const QString board, const QString &targetCANline, const QString &targetCANaddr);
int loadDatFileStrain2(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,QString file,bool eraseEEprom);
int saveDatFileStrain2(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,bool eraseEEprom);
int setStrainSn(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId, QString serialNumber);
int setStrainGainsOffsets(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId);
int getCanBoardVersion(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,bool save);


int main(int argc, char *argv[])
{
    Network yarp;
    QApplication a(argc, argv);

    QApplication::setStyle("motif");


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
    //QCommandLineOption strainCalibOption(QStringList() << "k" << "strain-acquisition", "The application starts the STRAIN acquisition mode");
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
    QCommandLineOption eraseEEpromOption(QStringList() << "p" << "erase_eeprom" << "Erase EEPROM of STRAIN during FW update","");
    QCommandLineOption verbosityOption(QStringList() << "x" << "verbosity", "Choose a verbosity level [0, 1]","verbosity","");
    QCommandLineOption verifyOption(QStringList() << "y" << "verify", "Verify FW version [ma.mi / ma.mi.re]. returns 0 if address and FW both match, 1 if board is found but FW does not match, 2 if board is not even found","verify","");
    QCommandLineOption queryOption(QStringList() << "q" << "query", "Queries a given address for its type and FW version [ma.mi / ma.mi.re]. prints a result on stdout. it returns 1 if it does not find a board at address");
    QCommandLineOption loadDatFileOption(QStringList() << "z" << "load-dat-file", "Loads the calibration .dat file into STRAIN2 eeprom (pass the file.dat with -l or --file option)","","");
    QCommandLineOption setStrainSnOption(QStringList() << "w" << "set-strain-sn", "Sets the passed serialNumber (i.e. SN001) on STRAIN2","sn","");
    QCommandLineOption setStrainGainsOffsetOption(QStringList() << "j" << "set-strain-gains", "Sets default gains (8,24,24,10,10,24) on STRAIN2","","");
    QCommandLineOption getCanBoardVersionOption(QStringList() << "b" << "get-canboard-version", "Gets Bootloader or Application version","saveFile","");
    QCommandLineOption saveDatFileOption(QStringList() << "u" << "save-dat-file", "Saves the calibration .dat file from STRAIN2 eeprom","","");


    parser.addOption(noGuiOption);
    //parser.addOption(strainCalibOption);
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
    parser.addOption(verifyOption);
    parser.addOption(queryOption);
    parser.addOption(loadDatFileOption);
    parser.addOption(setStrainSnOption);
    parser.addOption(setStrainGainsOffsetOption);
    parser.addOption(getCanBoardVersionOption);
    parser.addOption(saveDatFileOption);



    parser.process(a);

    bool noGui = parser.isSet(noGuiOption);
    bool adminMode = parser.isSet(adminOption);
    //bool strainCalibMode = parser.isSet(strainCalibOption);
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
    rf.setDefaultContext("firmwareUpdater");
    rf.setDefaultConfigFile(iniFile.toLatin1().data());

    if(!rf.configure(argc, argv)){
        return false;
    }

    if(!core.init(rf, port, address, verbosity)){
        return -1;
    }
    int ret = 1;
    MainWindow w(&core,adminMode/*,strainCalibMode*/);
    if(!noGui){
        w.show();
        ret = a.exec();
    }else{
        bool discover = parser.isSet(discoverOption);
        bool program = parser.isSet(programOption);
        bool verify = parser.isSet(verifyOption);
        bool query = parser.isSet(queryOption);
        QString device = parser.value(deviceOption);
        QString id = parser.value(idOption);
        QString board = parser.value(boardOption);
        QString file = parser.value(fileOption);
        QString canLine = parser.value(ethCanLineOption);
        QString canId = parser.value(ethCanIdOption);

        QString targetFW = parser.value(verifyOption);

        bool forceMaintenance = parser.isSet(ethForceMaintenance);
        bool forceApplication = parser.isSet(ethForceApplication);
        bool eraseEEprom = parser.isSet(eraseEEpromOption);
        bool loadDatFile = parser.isSet(loadDatFileOption);
        bool saveDatFile = parser.isSet(saveDatFileOption);
        bool setSn = parser.isSet(setStrainSnOption);
        QString serialNumber = parser.value(setStrainSnOption);
        bool setGains = parser.isSet(setStrainGainsOffsetOption);
        QString saveVersion = parser.value(getCanBoardVersionOption);
        bool getVersion = parser.isSet(getCanBoardVersionOption);

        core.setVerbosity(verbosity);

        action_t action = action_none;

        // check mutual exclusive actions: discover or query or verify or program or forceapplication or forcemaintenance
        // to do: move code into functions

        if((discover) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_discover;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((query) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_query;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((verify) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_verify;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((program) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_program;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((forceMaintenance) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_forcemaintenance;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((forceApplication) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_forceapplication;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((loadDatFile) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_loaddatfile;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((setSn) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_setstrainsn;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((setGains) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_setstraingainsoffsets;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((getVersion) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_getcanboardversion;
            }
            else
            {
                action = action_impossible;
            }
        }

        if((saveDatFile) && (action_impossible != action))
        {
            if(action == action_none)
            {
                action = action_savedatfile;
            }
            else
            {
                action = action_impossible;
            }
        }

        // now use a switch case

        switch(action)
        {
            default:
            case action_none:
            {
                ret = 1;

                if(verbosity >= 1) qDebug() << "specify at least one option amongst discover / verify / program / forcemaintenance / forceapplication";

            } break;

            case action_impossible:
            {
                ret = 1;

                if(verbosity >= 1) qDebug() << "specify only one option amongst discover / verify / program / forcemaintenance / forceapplication";

            } break;

            case action_discover:
            {
                ret = 1;
                //yDebug() << "discover";

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else{

                    if(board.isEmpty()){
                        ret = printSecondLevelDevices(&core,device,id, true);
                    }else{
                        ret = printThirdLevelDevices(&core,device,id,board,true,false, true);
                    }
                }

            } break;

            case action_query:
            {
                ret = 1;
                //yDebug() << "query";

                if(device.isEmpty())
                {
                    if(verbosity >= 1) qDebug() << "Need a device";
                }
                else if (device.contains("ETH"))
                {
                    // second level eth (ipaddr + ethfwversion) or third level can_under_eth (ipaddr+canaddr + canfwversion) or ..
                    if(board.isEmpty())
                    {
                        if(verbosity >= 1) qDebug() << "Need an IP address";
                    }
                    else if((canLine.isEmpty()) && (canId.isEmpty()))
                    {
                        // we query the fw version of an eth board
                        ret = queryOnSecondLevel_ETHboard(&core, device, id, board);
                    }
                    else if((!canLine.isEmpty()) && (!canId.isEmpty()))
                    {
                        // we query the fw version of a can board below eth
                        // FirmwareUpdater --nogui --verbosity 1 --device ETH --id eth1 --eth_board 10.0.1.1 --can_line 2 --can_id 2 --query
                        ret = queryOnThirdLevel_CANunderETH(&core, device, id, board, canLine, canId);
                    }
                    else
                    {
                        if(verbosity >= 1) qDebug() << "Must have both can line and address";
                    }
                }
                else
                {
                    // second level cfw2 or other can driver
                    if((canLine.isEmpty()) || (canId.isEmpty()))
                    {
                        if(verbosity >= 1) qDebug() << "Must have both can line and address";
                    }
                    else
                    {
                        // we query the fw version of a can board below cfw2
                        ret = queryOnSecondLevel_CANboard(&core, device, id, canLine, canId);
                    }

                }

            } break;


            case action_verify:
            {
                ret = 2;
                //yDebug() << "verify";

                if(device.isEmpty())
                {
                    if(verbosity >= 1) qDebug() << "Need a device";
                }
                else if (device.contains("ETH"))
                {
                    // second level eth (ipaddr + ethfwversion) or third level can_under_eth (ipaddr+canaddr + canfwversion) or ..
                    if(board.isEmpty())
                    {
                        if(verbosity >= 1) qDebug() << "Need an ip address";
                    }
                    else if(targetFW.isEmpty())
                    {
                        if(verbosity >= 1) qDebug() << "Need a target fw version";
                    }
                    else if((canLine.isEmpty()) && (canId.isEmpty()))
                    {
                        // we evaluate the fw version of an eth board
                        ret = verifyOnSecondLevel_ETHboard(&core, device, id, board, targetFW);
                    }
                    else if((!canLine.isEmpty()) && (!canId.isEmpty()))
                    {
                        // we evaluate the fw version of a can board below eth
                        // FirmwareUpdater --nogui --verbosity 1 --device ETH --id eth1 --eth_board 10.0.1.1 --can_line 2 --can_id 2 --verify 1.3.7
                        ret = verifyOnThirdLevel_CANunderETH(&core, device, id, board, canLine, canId, targetFW);
                    }
                    else
                    {
                        if(verbosity >= 1) qDebug() << "Must have both can line and address";
                    }
                }
                else
                {
                    // second level cfw2 or other can driver
                    if(targetFW.isEmpty())
                    {
                        if(verbosity >= 1) qDebug() << "Need a target fw version";
                    }
                    else if((canLine.isEmpty()) || (canId.isEmpty()))
                    {
                        if(verbosity >= 1) qDebug() << "Must have both can line and address";
                    }
                    else
                    {
                        // we evaluate the fw version of a can board below eth
                        ret = verifyOnSecondLevel_CANboard(&core, device, id, canLine, canId, targetFW);
                    }

                }

            } break;

            case action_program:
            {
                ret = 1;
                //yDebug() << "program";

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
//                    if(eraseEEprom && ret == 0){
//                        ret = eraseEthEEprom(&core,device,id,board);
//                    }
                }else{
                    if(canLine.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can line to be set";
                    } else if(canId.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can id to be set";
                    }else{
                        ret = programCanDevice(&core,device,id,board,canLine,canId,file,eraseEEprom);
                    }
                }

            } break;

            case action_forcemaintenance:
            {
                ret = 1;
                //yDebug() << "forcemaintenance";

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else if(board.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a board to be set";
                }else{

                    ret = setBoardToMaintenance(&core,device,id,board);
                }

            } break;

            case action_forceapplication:
            {
                ret = 1;
                //yDebug() << "forceapplication";

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else if(board.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a board to be set";
                }else{

                    ret = setBoardToApplication(&core,device,id,board);
                }

            } break;

            case action_loaddatfile:
            {
                ret = 1;

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else if(file.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a file path to be set";
                }else{
                    if(!device.contains("ETH") && canLine.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can line to be set";
                    } else if(!device.contains("ETH") && canId.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can id to be set";
                    }else{
                      ret = loadDatFileStrain2(&core,device,id,board,canLine,canId,file,eraseEEprom);
                    }
                }

            } break;

            case action_setstrainsn:
            {
                ret = 1;

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else{
                    if(!device.contains("ETH") && canLine.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can line to be set";
                    } else if(!device.contains("ETH") && canId.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can id to be set";
                    }else if(!device.contains("ETH") && serialNumber.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a serial number to be set";
                    }else{
                      ret = setStrainSn(&core,device,id,board,canLine,canId,serialNumber);
                    }
                }

            } break;

            case action_setstraingainsoffsets:
            {
                ret = 1;

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else{
                    if(!device.contains("ETH") && canLine.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can line to be set";
                    } else if(!device.contains("ETH") && canId.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can id to be set";
                    }else{
                      ret = setStrainGainsOffsets(&core,device,id,board,canLine,canId);
                    }
                }

            } break;

            case action_getcanboardversion:
            {
                ret = 1;

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else{
                    if(!device.contains("ETH") && canLine.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can line to be set";
                    } else if(!device.contains("ETH") && canId.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can id to be set";
                    }else{
                      bool save;
                      if(saveVersion == "y") save = true;
                      else save = false;
                      ret = getCanBoardVersion(&core,device,id,board,canLine,canId,save);
                    }
                }

            } break;

            case action_savedatfile:
            {
                ret = 1;

                if(device.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need a device to be set";
                }else if(id.isEmpty()){
                    if(verbosity >= 1) qDebug() << "Need an id to be set";
                }else{
                    if(!device.contains("ETH") && canLine.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can line to be set";
                    } else if(!device.contains("ETH") && canId.isEmpty()){
                        if(verbosity >= 1) qDebug() << "Need a can id to be set";
                    }else{
                      ret = saveDatFileStrain2(&core,device,id,board,canLine,canId,eraseEEprom);
                    }
                }

            } break;

        };
#if 0
// old code now substituted by the switch-case
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
//                if(eraseEEprom && ret == 0){
//                    ret = eraseEthEEprom(&core,device,id,board);
//                }
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
//            if(eraseEEprom && ret == 0){
//                ret = eraseEthEEprom(&core,device,id,board);
//            }
        }
#endif

    }



#ifdef UPDATER_RELEASE
    removeApplicationLock();
#endif

    return ret;
}


/**************************************************/

//int eraseEthEEprom(FirmwareUpdaterCore *core,QString device,QString id,QString board)
//{
//    int boards = core->connectTo(device,id);
//    if(boards > 0){
//        if(device.contains("ETH")){
//            core->setSelectedEthBoard(board,true);
//            core->eraseEthEprom();
//        }
//    }
//    return 0;
//}

/*  // the chosen gains:
        const strain2_ampl_discretegain_t ampsets[NUMofCHANNELS] =
        {
            ampl_gain08, ampl_gain24, ampl_gain24,
            ampl_gain10, ampl_gain10, ampl_gain24
        };

        yDebug() << "strain2-amplifier-tuning: STEP-1. imposing gains which are different of each channel";

        for(int channel=0; channel<NUMofCHANNELS; channel++)
        {
            yDebug() << "strain2-amplifier-tuning: STEP-1. on channel" << channel << "we impose gain =" << strain_amplifier_discretegain2float(ampsets[channel]);

            strain_set_amplifier_discretegain(bus, target_id, channel, ampsets[channel], regset, errorstring);

            // i wait some time
            yarp::os::Time::delay(1.0);
        }

        */   

int getCanBoardVersion(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,bool save)
{
    QList <sBoard> canBoards;
    QString retString;
    int ret;
    string msg;
      
    if(device.contains("SOCKETCAN"))
    {
        if (canId.toInt() <1 || canId.toInt() >= 15){
        yError("Invalid board address!\n");
        return false;
        }

        canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        
        
    }
    else if(device.contains("ETH"))
    {
        QString result, ret;
        ret = setBoardToMaintenance(core,device,id,board);
        if(!core->isBoardInMaintenanceMode(board)){
            yError("ETH board is not present or not in maintenace mode!!\n");
            return false;
        }
        canBoards = core->getCanBoardsFromEth(board,&result,canLine.toInt(),true);
    }

        if(canBoards.count() > 0 && icubCanProto_boardType__strain2 == canBoards[0].type)
        {
            ofstream myfile;
            string prefix = "Application ";

            if(!canBoards[0].applicationisrunning && save)
            {   
                try{
                    myfile.open ("firmware-info.txt", std::ios_base::app);
                    prefix = " Bootloader ";
                    myfile << canBoards[0].appl_vers_major << "." << canBoards[0].appl_vers_minor << "\n";
                    myfile.close();
                    yInfo() << prefix << " version : " << canBoards[0].appl_vers_major << "." << canBoards[0].appl_vers_minor;
                }
                catch (std::ifstream::failure e) {
                    yError() << "Exception opening file";
                    return false;
                }               
            }else if(canBoards[0].applicationisrunning && save)
            {
                try{
                    myfile.open ("firmware-info.txt", std::ios_base::app);
                    prefix = " Application ";
                    myfile << canBoards[0].appl_vers_major << "." << canBoards[0].appl_vers_minor << "." << canBoards[0].appl_vers_build << "\n";
                    myfile.close();
                    yInfo() << prefix << " version : " << canBoards[0].appl_vers_major << "." << canBoards[0].appl_vers_minor << "." << canBoards[0].appl_vers_build;
                }
                catch (std::ifstream::failure e) {
                    yError() << "Exception opening file";
                    return false;
                }     
            }
            
        } else {
            yError() << "No CAN board found, stopped!";
            return false;
        }
   
    return -1;
}

int setStrainGainsOffsets(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId)
{
    //10-2020 - davide.tome@iit.it
    //This method is used to set the PGA gains to 
    // i.e. FirmwareUpdater -g -e ETH -i eth1 -t 10.0.1.1 -c 1 -n 13 -z -w SN001

    QList <sBoard> canBoards;
    QString retString;
    int ret;
    string msg;
    std::vector<strain2_ampl_discretegain_t> gains(0);
    std::vector<int16_t> targets(0);
    const strain2_ampl_discretegain_t ampsets[6] =
            {
                ampl_gain08, ampl_gain24, ampl_gain24,
                ampl_gain10, ampl_gain10, ampl_gain24
            };
    

    for(int i = 0; i < 6; i++){ targets.push_back(0); gains.push_back(ampsets[i]);}
  
    if(device.contains("SOCKETCAN"))
    {
        if (canId.toInt() <1 || canId.toInt() >= 15){
        yError("Invalid board address!\n");
        return false;
        }

        canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        
        
    }
    else if(device.contains("ETH"))
    {
        QString result, ret;
        ret = setBoardToMaintenance(core,device,id,board);
        if(!core->isBoardInMaintenanceMode(board)){
            yError("ETH board is not present or not in maintenace mode!!\n");
            return false;
        }
        canBoards = core->getCanBoardsFromEth(board,&result,canLine.toInt(),true);
    }

        if(canBoards.count() > 0 && icubCanProto_boardType__strain2 == canBoards[0].type)
        {
            string error = "e";

            yDebug() << "strain2-amplifier-tuning: STEP-1. imposing gains which are different of each channel";

            core->getDownloader()->strain_calibrate_offset2(canLine.toInt(), canId.toInt(), icubCanProto_boardType__strain2, gains, targets, &msg);
            yarp::os::Time::delay(0.2);
            core->getDownloader()->strain_save_to_eeprom(canLine.toInt(),canId.toInt(), &msg);
            yInfo() << "Gains Saved!"; 

        } else {
            yError() << "No STRAIN2 board found, stopped!";
            return false;
        }

        unsigned int adc[6];
        char tempbuf [250];
        bool failCh = false;

        for(int i=0; i<6; i++){

            if(i==0)ret  = core->getDownloader()->strain_get_adc (canLine.toInt(),canId.toInt(), i, adc[i], 0, &msg);
            else  ret  |= core->getDownloader()->strain_get_adc (canLine.toInt(),canId.toInt(), i, adc[i], 0, &msg);
            
            unsigned int z = static_cast<int>(adc[i])-32768;
            sprintf(tempbuf,"%d",z);
            int t = std::stoi(tempbuf);
            if(t < -500 || t > 500) failCh = true;
            yDebug() << i << " " << std::stoi(tempbuf);
            yarp::os::Time::delay(0.2);
        }

        if(failCh){
            yError() << "Strange value on Channels ADC readings...";
            return false;
        }else{
            yInfo() << "Good values in ADC channels reading!";
        }
       
       /*  if(device.contains("ETH")){
            ret = setBoardToApplication(core,device,id,board);
            if(core->isBoardInMaintenanceMode(board)){
            yError("ETH board not switched to application mode!!\n");
            return false;
            } else yInfo() << "ETH board ready!";
        } */

   
    return -1;
}

int setStrainSn(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId, QString serialNumber)
{
    //10-2020 - davide.tome@iit.it
    //This method is used to set the SN in the STRAIN EEPROM
    // i.e. FirmwareUpdater -g -e ETH -i eth1 -t 10.0.1.1 -c 1 -n 13 -z -w SN001
    
    QList <sBoard> canBoards;
    QString retString;
    int ret;
    string msg;

    QByteArray string = serialNumber.toLatin1();
    char * sn = string.data();

    if(device.contains("SOCKETCAN"))
    {
        if (canId.toInt() <1 || canId.toInt() >= 15){
        yError("Invalid board address!\n");
        return false;
        }

        canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
               
    }
    else if(device.contains("ETH"))
    {
        QString result, ret;
        ret = setBoardToMaintenance(core,device,id,board);
        if(!core->isBoardInMaintenanceMode(board)){
            yError("ETH board is not present or not in maintenace mode!!\n");
            return false;
        }
        canBoards = core->getCanBoardsFromEth(board,&result,canLine.toInt(),true);
    }
    

    if(canBoards.count() > 0 && icubCanProto_boardType__strain2 == canBoards[0].type)
        {
            
            
            core->getDownloader()->strain_set_serial_number(canLine.toInt(),canId.toInt(), sn);
            core->getDownloader()->strain_save_to_eeprom(canLine.toInt(),canId.toInt(), &msg);

            yInfo() << "Serial Number Saved!";
            

        } else {
            yError() << "No STRAIN2 board found, stopped!";
            return false;
        }

        /* if(device.contains("ETH")){
            ret = setBoardToApplication(core,device,id,board);
            if(core->isBoardInMaintenanceMode(board)){
            yError("ETH board not switched to application mode!!\n");
            return false;
            } else yInfo() << "ETH board ready!";
        } */

    return -1;
}

int loadDatFileStrain2(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,QString file,bool eraseEEprom)
{
    //09-2020 - davide.tome@iit.it
    //This method is used to load a calibration file into the eeprom of the STRAIN2 using the dedicated CLI option -z/--load-dat-file (and -l/--file option to specify the file to be loaded)
    // i.e. FirmwareUpdater -g -e SOCKETCAN -i 0 -c 1 -n 13 -z -l calibrationDataSN003.dat
    // i.e. FirmwareUpdater -g -e ETH -i eth1 -t 10.0.1.1 -c 1 -n 13 -z -l calibrationDataSN003.dat
    
    QList <sBoard> canBoards;
    QString retString;
    int ret;
    char sn[256];
    int index = 0;
    unsigned int CHANNEL_COUNT = 6; 
    strain2_ampl_regs_t amp_registers[6]; 
    unsigned int offset[6];
    unsigned int calib_matrix[3][6][6];
    int calib_bias[6];
    unsigned int full_scale_const[3][6];
    string msg;


    if (file==NULL){
        yError("File not found!\n");
        return false;
    }
    

    if(device.contains("SOCKETCAN"))
    {
        if (canId.toInt() <1 || canId.toInt() >= 15){
        yError("Invalid board address!\n");
        return false;
        }

        canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        
        
    }
    else if(device.contains("ETH"))
    {
        QString result, ret;
        ret = setBoardToMaintenance(core,device,id,board);
        if(!core->isBoardInMaintenanceMode(board)){
            yError("ETH board is not present or not in maintenace mode!!\n");
            return false;
        }
        canBoards = core->getCanBoardsFromEth(board,&result,canLine.toInt(),true);
    }
    
    //Flash the .dat file   
    if(canBoards.count() > 0 && icubCanProto_boardType__strain2 == canBoards[0].type)
        {
            int ret = core->getDownloader()->get_serial_no(canLine.toInt(),canId.toInt(),sn);
            if(canBoards.count() > 0)
            {
                int boardtype = canBoards[0].type;
                int regset = canBoards[0].strainregsetinuse;

                int file_version=0;
                fstream filestr;
                filestr.open (file.toLatin1().data(), fstream::in);
                if (!filestr.is_open()){
                    yError("Error opening calibration file!\n");
                    return false;
                }

                int i=0;
                char buffer[256];

                //file version
                filestr.getline (buffer,256);
                filestr.getline (buffer,256);
                sscanf (buffer,"%d",&file_version);


                if((icubCanProto_boardType__strain2 == boardtype) && (3 != file_version))
                {
                    yError("Wrong file. Calibration version not supported for strain2: %d\n", file_version);
                    return false;
                }
                else if((icubCanProto_boardType__strain == boardtype) && (2 != file_version))
                {
                    yError("Wrong file. Calibration version not supported: %d\n", file_version);
                    return false;
                }

                if(3 == file_version)
                {
                    // Board type:
                    filestr.getline (buffer,256);
                    filestr.getline (buffer,256);
                    if(0 != strcmp(buffer, "strain2"))
                    {
                        yError("Wrong file. Board type not supported: %s\n", buffer);
                        return false;
                    }

                    // Serial number:
                    filestr.getline (buffer,256);
                    filestr.getline (buffer,256);
                    sprintf(sn,"%s", buffer);
                    core->getDownloader()->strain_set_serial_number(canLine.toInt(),canId.toInt(), sn);
                    //yDebug() << buffer;

                    // Amplifier registers:
                    filestr.getline (buffer,256);
                    for (i=0;i<CHANNEL_COUNT; i++)
                    {
                        filestr.getline (buffer,256);
                        yDebug() << buffer;
                        unsigned int t08[6] = {0};
                        sscanf  (buffer,"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", &t08[0], &t08[1], &t08[2], &t08[3], &t08[4], &t08[5]);
                        for(int j=0; j<6; j++) amp_registers[i].data[j] = t08[j];

                        core->getDownloader()->strain_set_amplifier_regs(canLine.toInt(),canId.toInt(), i, amp_registers[i], regset);

                        // downloader.strain_set_offset (downloader.board_list[selected].bus, downloader.board_list[selected].pid, i, offset[i]);
                        //core->getDownloader()->strain_set_offset (bus,id, i, offset[i]);
                        //printf("0X%02x, 0X%02x, 0X%02x, 0X%02x, 0X%02x,0X%02x", amp_registers[i].data[0], amp_registers[i].data[1], amp_registers[i].data[2], amp_registers[i].data[3], amp_registers[i].data[4], amp_registers[i].data[5]);
                        //fflush(stdout);
                        drv_sleep(10);
                    }

                }
                else
                {

                    //serial number
                    filestr.getline (buffer,256);
                    filestr.getline (buffer,256);
                    sprintf(sn,"%s", buffer);
                    core->getDownloader()->strain_set_serial_number(canLine.toInt(),canId.toInt(), sn);

                    //offsets
                    filestr.getline (buffer,256);
                    for (i=0;i<CHANNEL_COUNT; i++)
                    {
                        filestr.getline (buffer,256);
                        sscanf  (buffer,"%d",&offset[i]);
                        // downloader.strain_set_offset (downloader.board_list[selected].bus, downloader.board_list[selected].pid, i, offset[i]);
                        core->getDownloader()->strain_set_offset (canLine.toInt(),canId.toInt(), i, offset[i], regset);
                        drv_sleep(200);
                    }
                }

                //calibration matrix
                filestr.getline (buffer,256);
                for (i=0;i<36; i++){
                    int ri=i/6;
                    int ci=i%6;
                    filestr.getline (buffer,256);
                    sscanf (buffer,"%x",&calib_matrix[index][ri][ci]);
                    //printf("%d %x\n", calib_matrix[index][ri][ci],calib_matrix[index][ri][ci]);
                    core->getDownloader()->strain_set_matrix_rc(canLine.toInt(),canId.toInt(), ri, ci, calib_matrix[index][ri][ci], regset);
                }

                //matrix gain
                filestr.getline (buffer,256);
                filestr.getline (buffer,256);
                int cc=0;
                sscanf (buffer,"%d",&cc);
                core->getDownloader()->strain_set_matrix_gain(canLine.toInt(),canId.toInt(), cc, regset);

                //tare
                filestr.getline (buffer,256);
                for (i=0;i<CHANNEL_COUNT; i++){
                    filestr.getline (buffer,256);
                    sscanf  (buffer,"%d",&calib_bias[i]);
                    core->getDownloader()->strain_set_calib_bias(canLine.toInt(),canId.toInt(), i, calib_bias[i], regset);
                }

                //full scale values
                filestr.getline (buffer,256);
                for (i=0;i<CHANNEL_COUNT; i++){
                    filestr.getline (buffer,256);
                    sscanf  (buffer,"%d",&full_scale_const[index][i]);
                    core->getDownloader()->strain_set_full_scale(canLine.toInt(),canId.toInt(), i, full_scale_const[index][i], regset);
                }
                filestr.close();
                filestr.clear();

                core->getDownloader()->strain_save_to_eeprom(canLine.toInt(),canId.toInt(), &msg);

                yInfo() << "Calibration file loaded!";
            }

        } else {
            yError() << "No STRAIN2 board found, stopped!";
            return false;
        }

        /* if(device.contains("ETH")){
            ret = setBoardToApplication(core,device,id,board);
            if(core->isBoardInMaintenanceMode(board)){
            yError("ETH board not switched to application mode!!\n");
            return false;
            } else yInfo() << "ETH board ready!";
        } */

    return -1;
}

int saveDatFileStrain2(FirmwareUpdaterCore *core,QString device,QString id,QString board,QString canLine,QString canId,bool eraseEEprom)
{
    //09-2020 - davide.tome@iit.it
    //This method is used to saves the calibration of the STRAIN2 
    // i.e. FirmwareUpdater -g -e ETH -i eth1 -t 10.0.1.1 -c 1 -n 13 -u
    
    QList <sBoard> canBoards;
    QString retString;
    int ret;
    char sn[256];
    unsigned int CHANNEL_COUNT = 6; 
    strain2_ampl_regs_t amp_registers[6]; 
    unsigned int offset[6];
    unsigned int calib_matrix[3][6][6];
    int calib_bias[6];
    float amp_gains[6];
    uint16_t amp_offsets[6];
    unsigned int full_scale_const[3][6];
    unsigned int matrix[3][6][6];
    unsigned int full_scale_calib[3][6];
    unsigned int calib_const[3];
    char serial_no[8];
    string msg;

    calib_const[0] = 1;
    calib_const[1] = 1;
    calib_const[2] = 1;

    
    int index = 0;

    char path[256] = { 0 };
    std::string filename;

    
    int i=0;
    char buffer[256];

    if(device.contains("SOCKETCAN"))
    {
        if (canId.toInt() <1 || canId.toInt() >= 15){
        yError("Invalid board address!\n");
        return false;
        }

        canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        
    }
    else if(device.contains("ETH"))
    {
        QString result, ret;
        ret = setBoardToMaintenance(core,device,id,board);
        if(!core->isBoardInMaintenanceMode(board)){
            yError("ETH board is not present or not in maintenace mode!!\n");
            return false;
        }
        canBoards = core->getCanBoardsFromEth(board,&result,canLine.toInt(),true);
    }
    
    //Flash the .dat file   
    if(canBoards.count() > 0 && icubCanProto_boardType__strain2 == canBoards[0].type)
    {
        core->getDownloader()->strain_get_serial_number(1, 13, serial_no);

        filename += "calibrationData";
        filename += serial_no;
        filename += ".dat";
        fstream filestr;
        filestr.open (filename.c_str(), fstream::out);

        for(int i=0; i<6; i++)
        {            
            core->getDownloader()->strain_get_amplifier_regs(1, 13, i, amp_registers[i], cDownloader::strain_regset_inuse, &msg);
            core->getDownloader()->strain_get_amplifier_gain_offset(1, 13, i, amp_gains[i], amp_offsets[i], cDownloader::strain_regset_inuse, &msg);   
            core->getDownloader()->strain_get_offset (1, 13, i, offset[i], cDownloader::strain_regset_inuse, &msg);  
        }

        for(int mi=0;mi<1;mi++){

            for (int ri=0;ri<CHANNEL_COUNT;ri++){
                for (int ci=0;ci<CHANNEL_COUNT;ci++){
                    core->getDownloader()->strain_get_matrix_rc(1, 13, ri, ci, matrix[mi][ri][ci], cDownloader::strain_regset_inuse, &msg);
                    core->getDownloader()->strain_get_full_scale(1, 13, ri, full_scale_const[mi][ri], cDownloader::strain_regset_inuse, &msg);
                }
            }
        }
    
        if(icubCanProto_boardType__strain2 == canBoards[0].type)
        {
                // file version
                filestr<<"File version:"<<endl;
                filestr<<"3"<<endl;
                // board type
                filestr<<"Board type:"<<endl;
                filestr<<"strain2"<<endl;
                // serial number
                filestr<<"Serial number:"<<endl;
                sprintf (buffer,"%s",serial_no);
                filestr<<buffer<<endl;
                // amplifier registers
                filestr<<"Amplifier registers:"<<endl;
                for (i=0;i<CHANNEL_COUNT; i++){
                    sprintf (buffer,"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                            amp_registers[i].data[0], amp_registers[i].data[1], amp_registers[i].data[2],
                            amp_registers[i].data[3], amp_registers[i].data[4], amp_registers[i].data[5]);
                    filestr<<buffer<<endl;
                }
            }
            else
            {
                //file version
                filestr<<"File version:"<<endl;
                filestr<<"2"<<endl;

                //serial number
                filestr<<"Serial number:"<<endl;
                sprintf (buffer,"%s",serial_no);
                filestr<<buffer<<endl;

                //offsets
                filestr<<"Offsets:"<<endl;
                for (i=0;i<CHANNEL_COUNT; i++){
                    sprintf (buffer,"%d",offset[i]);
                    filestr<<buffer<<endl;
                }
            }



            //calibration matrix
            filestr<<"Calibration matrix:"<<endl;
            for (i=0;i<36; i++){
                sprintf (buffer,"%x",matrix[index][i/6][i%6]);
                filestr<<buffer<<endl;
            }


            //matrix gain
            filestr<<"Matrix gain:"<<endl;
            sprintf (buffer,"%d",calib_const[index]);
            filestr<<buffer<<endl;


            //tare
            filestr<<"Tare:"<<endl;
            for (i=0;i<CHANNEL_COUNT; i++){
                sprintf (buffer,"%d",calib_bias[i]);
                filestr<<buffer<<endl;
            }

            //full scale values
            filestr<<"Full scale values:"<<endl;
            for (i=0;i<CHANNEL_COUNT; i++){
                sprintf (buffer,"%d",full_scale_const[index][i]);
                filestr<<buffer<<endl;
            }
            
            yInfo() << "Calibration file saved!";
            filestr.close();    
        }
    
    return -1;
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

int printSecondLevelDevices(FirmwareUpdaterCore *core,QString device,QString id, bool slimprint)
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
            char appl_version[32] = {0};

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
                snprintf(appl_version, sizeof(appl_version), "%d.%d", board.getInfo().processes.info[2].version.major, board.getInfo().processes.info[2].version.minor);

                if(true == slimprint)
                {
                    char IPslimstring[512] = {0};
                    snprintf(IPslimstring, sizeof(IPslimstring), "%s: type = %s, application = %d.%d, updater = %d.%d, loader = %d.%d",
                                board.getIPV4string().c_str(),
                                eoboards_type2string2(eoboards_ethtype2type(board.getInfo().boardtype), eobool_true),
                                board.getInfo().processes.info[2].version.major, board.getInfo().processes.info[2].version.minor,
                                board.getInfo().processes.info[1].version.major, board.getInfo().processes.info[1].version.minor,
                                board.getInfo().processes.info[0].version.major, board.getInfo().processes.info[0].version.minor);

                    qDebug() << IPslimstring;
                }
                else
                {
                    if(verbosity >= 1) qDebug() << "************** Device " << i << " ******************";
                    if(verbosity >= 1) qDebug() << "Ip:        "<< board_ipaddr;
                    if(verbosity >= 1) qDebug() << "Mac:       "<< board_mac;
                    if(verbosity >= 1) qDebug() << "Version:   "<< board_version;
                    if(verbosity >= 1) qDebug() << "Appl Ver:  "<< appl_version;
                    if(verbosity >= 1) qDebug() << "Type:      "<< board_type;
                    if(verbosity >= 1) qDebug() << "Process:   "<< running_process;
                    if(verbosity >= 1) qDebug() << "Info:      "<< board_info;
                    if(verbosity >= 1) qDebug() << "Date:      "<< board_date;
                    if(verbosity >= 1) qDebug() << "Built:     "<< board_built;
                    if(verbosity >= 1) qDebug() << "\n";
                }

            }
            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

        }
    }else{
        QString retString;
        QList <sBoard> canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        if(canBoards.count() <= 0){
            if(verbosity >= 1) qDebug() <<  retString;
        }else{
            QString empty;
            printCanDevices(canBoards, empty, slimprint);
        }
    }
    return 0;
}

int printThirdLevelDevices(FirmwareUpdaterCore *core,QString device,QString id,QString board, bool forceMaintenance, bool forceApplication, bool slimprint)
{

//    if(forceMaintenance)
//    {
//        yDebug() << "printThirdLevelDevices() is sending in manteinance mode:" << board.toStdString();
//        int ret = setBoardToMaintenance(core,device,id,board);
//        yDebug() << "ret is " << ret;
//    }

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
                    printCanDevices(canBoards, board, slimprint);
                }

            }else{
                if(verbosity >= 1) qDebug() << "for board" << board << "You have to put the device in maintenace mode to perform this operation.";
            }

        }
    }else{
        if(verbosity >= 1) qDebug() << "No boards Found";
    }

    return 0;
}

void printCanDevices(QList<sBoard> canBoards, QString onIPboard, bool slimprint)
{
    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";
    for(int i=0;i<canBoards.count();i++){
        sBoard board = canBoards.at(i);

        char board_type        [50];        memset (board_type, 0, sizeof(board_type));
        char board_process     [50];        memset (board_process, 0, sizeof(board_process));
        char board_status      [50];        memset (board_status, 0, sizeof(board_status));
        char board_add_info    [50];        memset (board_add_info, 0, sizeof(board_add_info));
        char board_firmware_version  [32];  memset (board_firmware_version, 0, sizeof(board_firmware_version));
        char board_appl_minor  [10];        memset (board_appl_minor, 0, sizeof(board_appl_minor));
        char board_appl_build  [10];        memset (board_appl_build, 0, sizeof(board_appl_build));
        char board_serial      [50];        memset (board_serial, 0, sizeof(board_serial));
        char board_protocol    [10];        memset (board_protocol, 0, sizeof(board_protocol));

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
            snprintf (board_firmware_version, sizeof(board_firmware_version), "%d.%d", board.appl_vers_major, board.appl_vers_minor);
        } else {
            snprintf (board_firmware_version, sizeof(board_firmware_version), "%d.%d.%d", board.appl_vers_major, board.appl_vers_minor, board.appl_vers_build);
        }

        snprintf (board_appl_minor, sizeof(board_appl_minor), "%d",board.appl_vers_minor);
        snprintf (board_appl_build, sizeof(board_appl_build), "%d",board.appl_vers_build);
        snprintf (board_serial, sizeof(board_serial), "%s", board.serial);

        if((0 == board.prot_vers_major) && (0 == board.prot_vers_minor))
        {
            snprintf (board_protocol, sizeof(board_protocol), "N/A");
        }
        else
        {
            snprintf (board_protocol, sizeof(board_protocol), "%d.%d", board.prot_vers_major, board.prot_vers_minor);
        }

        if(true == slimprint)
        {
            char CANslimstring[512] = {0};
            char IPstr[24] = {0};
            if(false == onIPboard.isEmpty())
            {
                snprintf(IPstr, sizeof(IPstr), "%s:", onIPboard.toStdString().c_str());
            }
            snprintf(CANslimstring, sizeof(CANslimstring), "%sCAN%d:%d: type = %s, application = %s",
                        IPstr,
                        board.bus, board.pid,
                        //board.getIPV4string().c_str(),
                        eoboards_type2string2((eObrd_type_t)board.type, eobool_true),
                        board_firmware_version);

            qDebug() << CANslimstring;
        }
        else
        {
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

int verifyOnSecondLevel(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetIPaddr, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers)
{
    int ret = 1;

    if(device.contains("ETH"))
    {
        int boards = core->connectTo(device, id);
        if(boards > 0)
        {

            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

            bool found =  false;
            for(int i=0;i<core->getEthBoardList().size();i++)
            {
                EthBoard board = core->getEthBoardList()[i];

                if(targetIPaddr.toStdString() == board.getIPV4string())
                {
                    found = true;

                    char board_ipaddr[16] = {0};
                    char appl_version[32] = {0};

                    snprintf(board_ipaddr, sizeof(board_ipaddr), "%s", board.getIPV4string().c_str());
                    snprintf(appl_version, sizeof(appl_version), "%d.%d", board.getInfo().processes.info[2].version.major, board.getInfo().processes.info[2].version.minor);

                    if(targetFWvers == appl_version)
                    {
                        ret = 0;
                        if(verbosity >= 1)  qDebug() << "MATCHED";
                    }
                    else
                    {
                        ret = 1;
                        if(verbosity >= 1)  qDebug() << "NOT MATCHED";
                    }

                    break;
                }



            }

            if(!found)
            {
                ret = 1;
                if(verbosity >= 1)  qDebug() << "NOT MATCHED (IP = " << targetIPaddr << " not in found boards)";
            }
            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

        }
        else
        {
            ret = 1;
            if(verbosity >= 1)  qDebug() << "NOT MATCHED (found no board)";
        }
    }
    else
    {
        QString retString;
        QList <sBoard> canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
        if(canBoards.count() <= 0)
        {
            ret = 1;
            if(verbosity >= 1) qDebug() <<  retString;
        }
        else
        {
            ret = verifyCanDevices(canBoards, targetCANline, targetCANaddr,  targetFWvers);
        }
    }

    return ret;
}

int verifyOnSecondLevel_ETHboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetIPaddr, const QString &targetFWvers)
{
    int ret = 2;

//    if(device.contains("ETH"))
//    {
        int boards = core->connectTo(device, id);
        if(boards > 0)
        {

            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

            bool found =  false;
            for(int i=0;i<core->getEthBoardList().size();i++)
            {
                EthBoard board = core->getEthBoardList()[i];

                if(targetIPaddr.toStdString() == board.getIPV4string())
                {
                    found = true;
                    ret = 1;

                    char board_ipaddr[16] = {0};
                    char appl_version[32] = {0};

                    snprintf(board_ipaddr, sizeof(board_ipaddr), "%s", board.getIPV4string().c_str());
                    snprintf(appl_version, sizeof(appl_version), "%d.%d", board.getInfo().processes.info[2].version.major, board.getInfo().processes.info[2].version.minor);

                    if(targetFWvers == appl_version)
                    {
                        ret = 0;
                        if(verbosity >= 1)  qDebug() << "ETH FOUND + FW MATCHED";
                    }
                    else
                    {
                        ret = 1;
                        if(verbosity >= 1)  qDebug() << "ETH FOUND + FW NOT MATCHED";
                    }

                    break;
                }



            }

            if(!found)
            {
                ret = 2;
                if(verbosity >= 1)  qDebug() << "NOT MATCHED (IP = " << targetIPaddr << " not in found boards)";
            }
            if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

        }
        else
        {
            ret = 2;
            if(verbosity >= 1)  qDebug() << "NOT MATCHED (found no board)";
        }

//    }
//    else
//    {
//        QString retString;
//        QList <sBoard> canBoards = core->getCanBoardsFromDriver(device,id.toInt(),&retString,true);
//        if(canBoards.count() <= 0)
//        {
//            ret = 1;
//            if(verbosity >= 1) qDebug() <<  retString;
//        }
//        else
//        {
//            ret = verifyCanDevices(canBoards, targetCANline, targetCANaddr,  targetFWvers);
//        }
//    }

    return ret;
}

int verifyOnSecondLevel_CANboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers)
{
    int ret = 2; // not found, not matched

    QString retString;
    QList <sBoard> canBoards = core->getCanBoardsFromDriver(device, id.toInt(), &retString, true);
    if(canBoards.count() <= 0)
    {
        ret = 2;
        if(verbosity >= 1) qDebug() <<  retString;
    }
    else
    {
        ret = verifyCanDevices(canBoards, targetCANline, targetCANaddr,  targetFWvers);
    }


    return ret;
}

int verifyCanDevices(QList<sBoard> canBoards, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers)
{
    int ret = 2; // not found, not matched

    bool found = false;

    for(int i=0;i<canBoards.count();i++)
    {
        sBoard board = canBoards.at(i);

        if(verbosity >= 1) qDebug() << "---------------------------------------------------------";

        // see if address matches
        char line[8] = {0};
        snprintf(line, sizeof(line), "%d", board.bus);
        char addr[8] = {0};
        snprintf(addr, sizeof(addr), "%d", board.pid);

        //if(verbosity >= 1) qDebug() << targetCANline << targetCANaddr;
        //printf("line+addr = %s %s", line, addr);

        if((targetCANline.toStdString() == string(line)) && (targetCANaddr.toStdString() == string(addr)) )
        {
            found =  true;
            ret = 1; // found, maybe not yet fw match

            if(verbosity >= 1) qDebug() << "CAN ADDRESS: FOUND";

            //snprintf(board_type, sizeof(board_type), "%s", eoboards_type2string2((eObrd_type_t)board.type, eobool_true));

            if(true == board.applicationisrunning)
            {
                char board_firmware_version  [32] = {0};
                snprintf (board_firmware_version, sizeof(board_firmware_version), "%d.%d.%d", board.appl_vers_major, board.appl_vers_minor, board.appl_vers_build);
                //printf("ss %s", board_firmware_version);

                if(string(board_firmware_version) == targetFWvers.toStdString())
                {
                    ret = 0; // match!
                    if(verbosity >= 1) qDebug() << "CAN FW VERSION: MATCHES";
                }
            }

            break;
        }
    }

    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

    return ret;
}


int verifyOnThirdLevel_CANunderETH(FirmwareUpdaterCore *core, QString device, QString id, QString board, const QString &targetCANline, const QString &targetCANaddr, const QString &targetFWvers)
{
    int ret = 2;

    const bool forceMaintenance = true;

    int boards = core->connectTo(device, id);

    if(boards > 0)
    {
        if(device.contains("ETH"))
        {
            if(forceMaintenance)
            {
                core->setSelectedEthBoard(board, true);
                core->goToMaintenance();
            }

            if(core->isBoardInMaintenanceMode(board))
            {
                QString retString;
                QList <sBoard> canBoards = core->getCanBoardsFromEth(board, &retString);
                if(canBoards.count() <= 0)
                {
                    if(verbosity >= 1) qDebug() <<  retString;
                }
                else
                {
                    ret = verifyCanDevices(canBoards, targetCANline, targetCANaddr, targetFWvers);
                }

            }
            else
            {
                if(verbosity >= 1) qDebug() << "You have to put the device in maintenace mode to perform this operation.";
            }

        }
    }
    else
    {
        if(verbosity >= 1) qDebug() << "No boards Found";
    }

    return ret;
}


int queryOnSecondLevel_ETHboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetIPaddr)
{
    int ret = 1;

    const bool slimprint = true;

    bool found =  false;

    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

    int boards = core->connectTo(device, id);
    if(boards > 0)
    {
        for(int i=0;i<core->getEthBoardList().size();i++)
        {
            EthBoard board = core->getEthBoardList()[i];

            if(targetIPaddr.toStdString() == board.getIPV4string())
            {
                found = true;
                ret = 0;

                if(true == slimprint)
                {
                    char IPslimstring[512] = {0};
                    snprintf(IPslimstring, sizeof(IPslimstring), "%s: type = %s, application = %d.%d, updater = %d.%d, loader = %d.%d",
                                board.getIPV4string().c_str(),
                                eoboards_type2string2(eoboards_ethtype2type(board.getInfo().boardtype), eobool_true),
                                board.getInfo().processes.info[2].version.major, board.getInfo().processes.info[2].version.minor,
                                board.getInfo().processes.info[1].version.major, board.getInfo().processes.info[1].version.minor,
                                board.getInfo().processes.info[0].version.major, board.getInfo().processes.info[0].version.minor);

                    qDebug() << IPslimstring;
                }

                break;
            }


        }


    }

    if(!found)
    {
        ret = 1;
        char notfound[512] = {0};
        snprintf(notfound, sizeof(notfound), "%s: not found",
                    targetIPaddr.toStdString().c_str());

        qDebug() << notfound;
    }
    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";


    return ret;
}


int queryOnThirdLevel_CANunderETH(FirmwareUpdaterCore *core, QString device, QString id, const QString board, const QString &targetCANline, const QString &targetCANaddr)
{
    int ret = 1;

    const bool forceMaintenance = true;

    char notfound[256] = {0};

    int boards = core->connectTo(device, id);

    if(boards > 0)
    {
        if(device.contains("ETH"))
        {
            if(forceMaintenance)
            {
                core->setSelectedEthBoard(board, true);
                core->goToMaintenance();
            }

            if(core->isBoardInMaintenanceMode(board))
            {
                QString retString;
                QList <sBoard> canBoards = core->getCanBoardsFromEth(board, &retString);
                if(canBoards.count() <= 0)
                {
                    if(verbosity >= 1) qDebug() <<  retString;
                    snprintf(notfound, sizeof(notfound), "%s: no can board beneath", board.toStdString().c_str());
                    qDebug() << notfound;
                }
                else
                {
                    ret = queryCanDevices(canBoards, board, targetCANline, targetCANaddr);
                }

            }
            else
            {
                if(verbosity >= 1) qDebug() << "You have to put the device in maintenace mode to perform this operation.";
            }

        }
    }
    else
    {
        if(verbosity >= 1) qDebug() << "No boards Found";
        snprintf(notfound, sizeof(notfound), "%s: cannot find it", board.toStdString().c_str());
        qDebug() << notfound;
    }

    return ret;
}


int queryCanDevices(QList<sBoard> canBoards, const QString onIPboard, const QString &targetCANline, const QString &targetCANaddr)
{
    int ret = 1; // not found

    bool found = false;
    const bool slimprint = true;

    for(int i=0;i<canBoards.count();i++)
    {
        sBoard board = canBoards.at(i);

        if(verbosity >= 1) qDebug() << "---------------------------------------------------------";

        // see if address matches
        char line[8] = {0};
        snprintf(line, sizeof(line), "%d", board.bus);
        char addr[8] = {0};
        snprintf(addr, sizeof(addr), "%d", board.pid);


        if((targetCANline.toStdString() == string(line)) && (targetCANaddr.toStdString() == string(addr)) )
        {
            found =  true;
            ret = 0; // found

            char board_firmware_version[32] = {0};

            if(true == board.applicationisrunning)
            {
                snprintf (board_firmware_version, sizeof(board_firmware_version), "%d.%d.%d", board.appl_vers_major, board.appl_vers_minor, board.appl_vers_build);
            }
            else
            {
                snprintf (board_firmware_version, sizeof(board_firmware_version), "%d.%d", board.appl_vers_major, board.appl_vers_minor);
            }

            if(true == slimprint)
            {
                char CANslimstring[512] = {0};
                char IPstr[24] = {0};
                if(false == onIPboard.isEmpty())
                {
                    snprintf(IPstr, sizeof(IPstr), "%s:", onIPboard.toStdString().c_str());
                }
                snprintf(CANslimstring, sizeof(CANslimstring), "%sCAN%d:%d: type = %s, application = %s",
                            IPstr,
                            board.bus, board.pid,
                            eoboards_type2string2((eObrd_type_t)board.type, eobool_true),
                            board_firmware_version);

                qDebug() << CANslimstring;
            }

            break;
        }
    }

    if(!found)
    {
        ret = 1;
        char notfound[512] = {0};
        char IPstr[24] = {0};
        if(false == onIPboard.isEmpty())
        {
            snprintf(IPstr, sizeof(IPstr), "%s:", onIPboard.toStdString().c_str());
        }
        snprintf(notfound, sizeof(notfound), "%sCAN%s:%s: not found",
                 IPstr,
                 targetCANline.toStdString().c_str(), targetCANaddr.toStdString().c_str());

        qDebug() << notfound;
    }

    if(verbosity >= 1) qDebug() << "-------------------------------------------------------------";

    return ret;
}


int queryOnSecondLevel_CANboard(FirmwareUpdaterCore *core, QString device, QString id, const QString &targetCANline, const QString &targetCANaddr)
{
    int ret = 1; // not found

    char notfound[256] = {0};

    QString retString;
    QList <sBoard> canBoards = core->getCanBoardsFromDriver(device, id.toInt(), &retString, true);
    if(canBoards.count() <= 0)
    {
        ret = 1;
        if(verbosity >= 1) qDebug() <<  retString;
        snprintf(notfound, sizeof(notfound), "<%s>: no can boards beneath", device.toStdString().c_str());
        qDebug() << notfound;
    }
    else
    {
        QString none;
        ret = queryCanDevices(canBoards, targetCANline, none, targetCANaddr);
    }


    return ret;
}


