#ifndef FIRMWAREUPDATERCORE_H
#define FIRMWAREUPDATERCORE_H

#include <QObject>
#include <QVariantMap>
#include <yarp/os/all.h>
//#include <EthUpdater.h>
#include <EthMaintainer.h>
#include <downloader.h>
#include <yarp/dev/all.h>
#include <QMutex>

using namespace yarp::os;

class FirmwareUpdaterCore : public QObject
{
    Q_OBJECT
public:
    explicit FirmwareUpdaterCore(QObject *parent = 0);

    bool init(Searchable& config, int port, QString address, int VerbositY);
    bool setVerbosity(int verb);
    QStringList getDevicesName();
    QList<QPair<QString,QVariant> > getDevices();
    int connectTo(QString device, QString id);
    bool isBoardInMaintenanceMode(QString ip);
    void disconnectFrom(QString device, QString id);
    EthBoardList getEthBoardList();
    void setSelectedEthBoard(int index,bool selected);
    void setSelectedEthBoard(QString boardIp,bool selected);
    void setSelectedCanBoard(int index, bool selected, QString ethAddress = "", int deviceId = -1);
    void setSelectedCanBoards(QList <sBoard> selectedBoards, QString address, int deviceId = -1);
    boardInfo2_t getMoreDetails(int boardNum = EthMaintainer::ipv4OfAllSelected, QString *infoString = NULL, eOipv4addr_t *address = NULL);
    QList<sBoard> getCanBoardsFromEth(QString address, QString *retString, int canID = CanPacket::everyCANbus, bool force = false);
    QList<sBoard> getCanBoardsFromDriver(QString driver, int networkId, QString *retString, bool force = false);
    void blinkEthBoards();
    QString getEthBoardInfo(int index);
    QString getEthBoardAddress(int index);
    bool setEthBoardInfo(int index, QString newInfo);
    void setCanBoardInfo(int bus, int id, QString newInfo, QString ethAddress = "", int deviceId = -1, QString *resultString = NULL);
    bool setEthBoardAddress(int index, QString newAddress);
    bool setCanBoardAddress(int bus, int id, int canType, QString newAddress, QString ethAddress = "", int deviceId = -1, QString *resultString = NULL);
    void restartEthBoards();
    void bootFromApplication();
    void bootFromUpdater();
    bool uploadEthApplication(QString filename, QString *resultString);
    bool uploadCanApplication(QString filename, QString *resultString, bool ee, QString address = "", int deviceId = -1, QList<sBoard> *resultCanBoards = NULL);
    bool uploadLoader(QString filename, QString *resultString);
    bool uploadUpdater(QString filename, QString *resultString);
    //void updateProgressCallback(float);
    bool jumpToUpdater();
    bool goToApplication();
    bool goToMaintenance();
    bool eraseEthEprom();
    void eraseCanEprom();
    QString getProcessFromUint(uint8_t id);
    cDownloader *getDownloader();


private:
    bool compile_ip_addresses(const char* addr,unsigned int *remoteAddr,unsigned int *localAddr);
private:
    QList < QPair<QString,QVariant> > devices;
    //EthUpdater  gUpdater;
    EthMaintainer gMNT;
    cDownloader downloader;
    QMutex mutex;
    QString currentAddress;
    QString currentDriver;
    int currentId;
    QList <sBoard> canBoards;
    int verbosity;

signals:
    void updateProgress(float);
    void selectedEnded();

public slots:
};

#endif // FIRMWAREUPDATERCORE_H
