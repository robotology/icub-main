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

    bool init(Searchable& config, int port, QString address);
    QStringList getDevicesName();
    QList<QPair<QString,QVariant> > getDevices();
    int connectTo(QString device, QString id);
    void disconnectFrom(QString device, QString id);
    EthBoardList getEthBoardList();
    void setSelectedEthBoard(int index,bool selected);
    void setSelectedCanBoard(int index, bool selected, QString ethAddress = "");
    boardInfo2_t getMoreDetails(int boardNum = EthMaintainer::ipv4OfAllSelected, QString *infoString = NULL, eOipv4addr_t *address = NULL);
    QList<sBoard> getCanBoardsFromEth(QString address, QString *retString, int canID = CanPacket::everyCANbus);
    void blinkEthBoards();
    QString getEthBoardInfo(int index);
    QString getEthBoardAddress(int index);
    void setEthBoardInfo(int index, QString newInfo);
    void setCanBoardInfo(int bus, int id, QString newInfo, QString ethAddress = "", QString *resultString = NULL);
    bool setEthBoardAddress(int index, QString newAddress);
    bool setCanBoardAddress(int bus, int id, int canType, QString newAddress, QString ethAddress = "",QString *resultString = NULL);
    void restartEthBoards();
    void bootFromApplication();
    void bootFromUpdater();
    bool uploadEthApplication(QString filename, QString *resultString);
    bool uploadCanApplication(QString filename,QString *resultString, QString ethAddress = "");
    bool uploadLoader(QString filename, QString *resultString);
    bool uploadUpdater(QString filename, QString *resultString);
    //void updateProgressCallback(float);
    bool jumpToUpdater();
    bool goToApplication();
    bool goToMaintenance();
    QString getProcessFromUint(uint8_t id);


private:
    bool compile_ip_addresses(const char* addr,unsigned int *remoteAddr,unsigned int *localAddr);
private:
    QList < QPair<QString,QVariant> > devices;
    //EthUpdater  gUpdater;
    EthMaintainer gMNT;
    cDownloader downloader;
    QMutex mutex;
    QString currentAddress;

signals:
    void updateProgress(float);
    void selectedEnded();

public slots:
};

#endif // FIRMWAREUPDATERCORE_H
