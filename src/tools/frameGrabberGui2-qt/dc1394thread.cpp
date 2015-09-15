#include "dc1394thread.h"
#include <QDebug>
#include "dc1394SliderBase.h"

DC1394Thread::DC1394Thread(char *loc, char *rem,QObject *parent) :
    QThread(parent)
{
    this->rem = rem;
    this->loc = loc;

    opCounter = 0;
}


void DC1394Thread::run()
{

    // create firewire thread
    //DC1394Control = new yarp::dev::RemoteFrameGrabberControlsDC1394();

    grabberControl = new yarp::dev::PolyDriver;
    // create usb thread

    yarp::os::Property config;
    config.put("device", "remote_grabber");
    config.put("remote",rem.toLatin1().data());
    config.put("local",loc.toLatin1().data());

    bool opened = grabberControl->open(config);  //TODO --> if false??????

    keepRunning = true;
    while(keepRunning){

        QVariantList list = taskList.takeFirst();

        threadFunction func = (threadFunction)list.takeFirst().toInt();

        switch(func){
        case _unknown:{
            done();
            break;
        }
        case _initFormatTab:{
            initFormatTab();
            break;
        }
        case _init:{
            init();
            break;
        }
        case _reload:{
            reload();
            break;
        }
        case _reset:{
            reset();
            break;
        }
        case _loadDefault:{
            loadDefualt();
            break;
        }
        case _setTransmissionDC1394:{
            setTransmissionDC1394(list);
            break;
        }
        case _setPowerDC1394:{
            setPowerDC1394(list);
            break;
        }
        case _setFormat7WindowDC1394:{
            setFormat7WindowDC1394(list);
            break;
        }
        case _setVideoModeDC1394:{
            setVideoModeDC1394(list);
            break;
        }
        case _setColorCodingDC1394:{
            setColorCodingDC1394(list);
            break;
        }
        case _setFPSDC1394:{
            setFPSDC1394(list);
            break;
        }
        case _setISOSpeedDC1394:{
            setISOSpeedDC1394(list);
            break;
        }
        case _setBytesPerPacketDC1394:{
            setBytesPerPacketDC1394(list);
            break;
        }
        case _setOperationModeDC1394:{
            setOperationModeDC1394(list);
            break;
        }
        case _sliderRefresh:{
            sliderRefresh(list);
            break;
        }
        case _sliderWBRefresh:{
            sliderWBRefresh(list);
            break;
        }
        case _sliderPropagate:{
            sliderPropagate(list);
            break;
        }
        case _sliderWBPropagate:{
            sliderWBPropagate(list);
            break;
        }
        case _sliderSetFeature:{
            sliderSetFeatureDC1394(list);
            break;
        }
        case _sliderWBSetFeature:{
            sliderWBSetFeatureDC1394(list);
            break;
        }
        case _sliderOnePush:{
            sliderOnePush(list);
            break;
        }
        case _sliderWBOnePush:{
            sliderWBOnePush(list);
            break;
        }
        case _sliderRadioAuto:{
            sliderRadioAuto(list);
            break;
        }
        case _sliderPower:{
            sliderPower(list);
            break;
        }
        case _sliderHasFeature:{
            sliderHasFeature(list);
            break;
        }
        }


        while(keepRunning && taskList.isEmpty()){
            mutex.lock();
            waitCond.wait(&mutex,200);
            mutex.unlock();
        }
    }

    DC1394Control->close();
    delete DC1394Control;
}

void DC1394Thread::stop()
{
    keepRunning = false;
    waitCond.wakeAll();
    while(isRunning()){
        QThread::msleep(100);
    }
}

void DC1394Thread::doTask(threadFunction func, QVariantList args)
{
    mutex1.lock();
    QVariant vaFunc = (int)func;

    if(func != _sliderSetFeature && func != _sliderWBSetFeature){
        if(opCounter == 0){
            startLoading();
        }
        opCounter++;
    }

    QVariantList list;
    list.append(vaFunc);
    list.append(args);

    mutex.lock();
    taskList.enqueue(list);
    waitCond.wakeAll();
    mutex.unlock();

    mutex1.unlock();
}

void DC1394Thread::doTask(threadFunction func)
{
    mutex1.lock();
    QVariant vaFunc = (int)func;

    if(opCounter == 0){
        startLoading();
    }
    opCounter++;

    QVariantList list;
    list.append(vaFunc);
    mutex.lock();
    taskList.enqueue(list);
    waitCond.wakeAll();
    mutex.unlock();

    mutex1.unlock();
}



void DC1394Thread::initFormatTab()
{
    uint videoModeMaskDC1394 = DC1394Control->getVideoModeMaskDC1394();
    uint fPSMaskDC1394 = DC1394Control->getFPSMaskDC1394();
    uint colorCodingMaskDC1394 = DC1394Control->getColorCodingMaskDC1394(DC1394Control->getVideoModeDC1394());
    initFormatTabDone(videoModeMaskDC1394,fPSMaskDC1394,colorCodingMaskDC1394);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::init()
{
    uint videoModeDC1394 = DC1394Control->getVideoModeDC1394();
    uint fPSDC1394 = DC1394Control->getFPSDC1394();
    uint iSOSpeedDC1394 = DC1394Control->getISOSpeedDC1394();
    bool operationModeDC1394 = DC1394Control->getOperationModeDC1394();
    uint colorCodingDC1394 = DC1394Control->getColorCodingDC1394();
    unsigned int xmax,ymax,xstep,ystep,xoffstep,yoffstep;
    DC1394Control->getFormat7MaxWindowDC1394(xmax,ymax,xstep,ystep,xoffstep,yoffstep);
    QSize max = QSize(xmax,ymax);
    QSize step = QSize(xstep,ystep);
    QSize offset = QSize(xoffstep,yoffstep);

    unsigned int xdim,ydim;
    int x0,y0;
    DC1394Control->getFormat7WindowDC1394(xdim,ydim,x0,y0);
    QSize dim = QSize(xdim,ydim);
    QSize pos = QSize(x0,y0);

    uint bytesPerPacketDC1394 = DC1394Control->getBytesPerPacketDC1394();
    bool transmissionDC1394 = DC1394Control->getTransmissionDC1394();

    initDone(videoModeDC1394,
             fPSDC1394,
             iSOSpeedDC1394,
             operationModeDC1394,
             colorCodingDC1394,
             max,
             step,
             offset,
             dim,
             pos,
             bytesPerPacketDC1394,
             transmissionDC1394
             );
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::reload()
{
    uint videoModeDC1394 = DC1394Control->getVideoModeDC1394();
    uint colorCodingMaskDC1394 = DC1394Control->getColorCodingMaskDC1394(videoModeDC1394);

    unsigned int xmax,ymax,xstep,ystep,xoffstep,yoffstep;
    DC1394Control->getFormat7MaxWindowDC1394(xmax,ymax,xstep,ystep,xoffstep,yoffstep);
    QSize max = QSize(xmax,ymax);
    QSize step = QSize(xstep,ystep);
    QSize offset = QSize(xoffstep,yoffstep);

    unsigned int xdim,ydim;
    int x0,y0;
    DC1394Control->getFormat7WindowDC1394(xdim,ydim,x0,y0);
    QSize dim = QSize(xdim,ydim);
    QSize pos = QSize(x0,y0);

    uint colorCodingDC1394 = DC1394Control->getColorCodingDC1394();
    uint bytesPerPacketDC1394 = DC1394Control->getBytesPerPacketDC1394();
    uint fPSMaskDC1394 = DC1394Control->getFPSMaskDC1394();
    uint fPSDC1394 = DC1394Control->getFPSDC1394();
    uint iSOSpeedDC1394 = DC1394Control->getISOSpeedDC1394();

    reloadDone(videoModeDC1394,
               colorCodingMaskDC1394,
               max,
               step,
               offset,
               dim,
               pos,
               colorCodingDC1394,
               bytesPerPacketDC1394,
               fPSMaskDC1394,
               fPSDC1394,
               iSOSpeedDC1394);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}

void DC1394Thread::loadDefualt()
{
    DC1394Control->setDefaultsDC1394();
    opCounter++;
    reload();

    uint bytesPerPacketDC1394 = DC1394Control->getBytesPerPacketDC1394();
    uint colorCodingMaskDC1394 = DC1394Control->getColorCodingMaskDC1394(DC1394Control->getVideoModeDC1394());
    DC1394Control->setTransmissionDC1394(true);
    bool transmissionDC1394 = DC1394Control->getTransmissionDC1394();

    loadDefaultDone(bytesPerPacketDC1394,colorCodingMaskDC1394,transmissionDC1394);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::reset()
{
    DC1394Control->setResetDC1394();
    opCounter++;
    reload();

    uint bytesPerPacketDC1394 = DC1394Control->getBytesPerPacketDC1394();
    uint colorCodingMaskDC1394 = DC1394Control->getColorCodingMaskDC1394(DC1394Control->getVideoModeDC1394());
    bool transmissionDC1394 = DC1394Control->getTransmissionDC1394();

    resetDone(bytesPerPacketDC1394,colorCodingMaskDC1394,transmissionDC1394);
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setTransmissionDC1394(QVariantList arg)
{
    qDebug() << "setTransmissionDC1394";
    bool value = arg.at(0).toBool();
    DC1394Control->setTransmissionDC1394(value);
    setTransmissionDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setPowerDC1394(QVariantList arg)
{
    qDebug() << "setPowerDC1394";
    bool value = arg.at(0).toBool();
    DC1394Control->setPowerDC1394(value);
    setPowerDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}
void DC1394Thread::setFormat7WindowDC1394(QVariantList arg)
{
    qDebug() << "setFormat7WindowDC1394";
    unsigned int xdim=arg.at(0).toUInt();
    unsigned int ydim=arg.at(1).toUInt();
    int x0=arg.at(2).toInt();
    int y0=arg.at(3).toInt();

    DC1394Control->setFormat7WindowDC1394(xdim,ydim,x0,y0);
    setFormat7WindowDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setVideoModeDC1394(QVariantList arg)
{
    qDebug() << "setVideoModeDC1394";
    unsigned int video_mode=arg.at(0).toUInt();
    DC1394Control->setVideoModeDC1394(video_mode);
    setVideoModeDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setColorCodingDC1394(QVariantList arg)
{
    qDebug() << "setColorCodingDC1394";

    int value =arg.at(0).toInt();
    DC1394Control->setColorCodingDC1394(value);

    setColorCodingDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setFPSDC1394(QVariantList arg)
{
    qDebug() << "setFPSDC1394";
    int value = arg.at(0).toInt();
    DC1394Control->setFPSDC1394(value);

    setFPSDC1394Done();

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setISOSpeedDC1394(QVariantList arg)
{
    qDebug() << "setISOSpeedDC1394";
    int value = arg.at(0).toInt();
    DC1394Control->setISOSpeedDC1394(value);

    setISOSpeedDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setBytesPerPacketDC1394(QVariantList arg)
{
    qDebug() << "setBytesPerPacketDC1394";
    int value = arg.at(0).toUInt();
    DC1394Control->setBytesPerPacketDC1394(value);

    setBytesPerPacketDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::setOperationModeDC1394(QVariantList arg)
{
    qDebug() << "setOperationModeDC1394";
    int value = arg.at(0).toInt();
    DC1394Control->setOperationModeDC1394(value);

    setOperationModeDC1394Done();
    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}

void DC1394Thread::sliderRefresh(QVariantList arg)
{
    qDebug() << "sliderRefresh";
    cameraFeature_id_t feature;

    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

//    bool bON=DC1394Control->getActiveDC1394(feature);
//    bool bAuto=DC1394Control->getModeDC1394(feature);
//    bool bHasOnOff=DC1394Control->hasOnOffDC1394(feature);
//    bool bHasAuto=DC1394Control->hasAutoDC1394(feature);
//    bool bHasManual=DC1394Control->hasManualDC1394(feature);
//    bool bHasOnePush=DC1394Control->hasOnePushDC1394(feature);
//    double val=DC1394Control->getFeatureDC1394(feature);

    bool bON=true;   //DC1394Control->getActiveDC1394(feature);
    bool bAuto=true;  //DC1394Control->getModeDC1394(feature);
    bool bHasOnOff=true; //DC1394Control->hasOnOffDC1394(feature);
    bool bHasAuto= true; //DC1394Control->hasAutoDC1394(feature);
    bool bHasManual=true; //DC1394Control->hasManualDC1394(feature);
    bool bHasOnePush=true; //DC1394Control->hasOnePushDC1394(feature);
    double val=0.5; //DC1394Control->getFeatureDC1394(feature);

    sliderRefreshDone(ptr,bON,bAuto,bHasOnOff,bHasAuto,bHasManual,bHasOnePush,val);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}

void DC1394Thread::sliderWBRefresh(QVariantList arg)
{
    qDebug() << "sliderWBRefresh";
    cameraFeature_id_t feature;

    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

    bool bON=DC1394Control->getActiveDC1394(feature);
    bool bAuto=DC1394Control->getModeDC1394(feature);
    bool bHasOnOff=DC1394Control->hasOnOffDC1394(feature);
    bool bHasAuto=DC1394Control->hasAutoDC1394(feature);
    bool bHasManual=DC1394Control->hasManualDC1394(feature);
    bool bHasOnePush=DC1394Control->hasOnePushDC1394(feature);
    double redVal;
    double blueVal;
    DC1394Control->getWhiteBalanceDC1394(blueVal,redVal);



    sliderWBRefreshDone(ptr,bON,bAuto,bHasOnOff,bHasAuto,bHasManual,bHasOnePush,redVal,blueVal);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}

void DC1394Thread::sliderPropagate(QVariantList arg)
{
    qDebug() << "sliderPropagate";

    cameraFeature_id_t feature;
    int value = arg.at(0).toInt();
    feature = (cameraFeature_id_t)value;


    double val = arg.at(1).toDouble();
    bool bRBa = arg.at(2).toBool();
    bool bPwr = arg.at(3).toBool();

    DC1394Control->setFeatureDC1394(feature,val);
    DC1394Control->setModeDC1394(feature,bRBa);
    DC1394Control->setActiveDC1394(feature,bPwr);

    sliderPropagateDone();

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}

void DC1394Thread::sliderWBPropagate(QVariantList arg)
{
    qDebug() << "sliderWBPropagate";

    cameraFeature_id_t feature;
    int value = arg.at(0).toInt();
    feature = (cameraFeature_id_t)value;


    double redVal = arg.at(1).toDouble();
    double blueVal = arg.at(2).toDouble();
    bool bRBa = arg.at(3).toBool();
    bool bPwr = arg.at(4).toBool();

    DC1394Control->setWhiteBalanceDC1394(blueVal,redVal);
    DC1394Control->setModeDC1394(feature,bRBa);
    DC1394Control->setActiveDC1394(feature,bPwr);

    sliderWBPropagateDone();

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}


void DC1394Thread::sliderSetFeatureDC1394(QVariantList arg)
{
    qDebug() << "sliderSetFeatureDC1394";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

    double val = arg.at(2).toDouble();

    DC1394Control->setFeatureDC1394(feature,val);

    sliderSetFeatureDC1394Done(ptr,val);

//    opCounter--;
//    if(opCounter == 0){
//        stopLoading();
//    }
}

void DC1394Thread::sliderWBSetFeatureDC1394(QVariantList arg)
{
    qDebug() << "sliderWBSetFeatureDC1394";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

    double redVal = arg.at(2).toDouble();
    double blueVal = arg.at(3).toDouble();

    DC1394Control->setWhiteBalanceDC1394(blueVal,redVal);

    sliderWBSetFeatureDC1394Done(ptr,redVal,blueVal);

//    opCounter--;
//    if(opCounter == 0){
//        stopLoading();
//    }
}

void DC1394Thread::sliderOnePush(QVariantList arg)
{
    qDebug() << "sliderOnePush";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

    DC1394Control->setOnePushDC1394(feature);

    double val = DC1394Control->getFeatureDC1394(feature);

    sliderOnePushDone(ptr,val);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}

void DC1394Thread::sliderWBOnePush(QVariantList arg)
{
    qDebug() << "sliderWBOnePush";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

    double redVal;
    double blueVal;

    DC1394Control->setOnePushDC1394(feature);
    DC1394Control->getWhiteBalanceDC1394(blueVal,redVal);

    sliderWBOnePushDone(ptr,redVal,blueVal);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}


void DC1394Thread::sliderRadioAuto(QVariantList arg)
{
    qDebug() << "sliderRadioAuto";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    bool bVal = arg.at(2).toBool();
    feature = (cameraFeature_id_t)value;

    bool bON = DC1394Control->getActiveDC1394(feature);
    DC1394Control->setModeDC1394(feature,bVal);

    sliderRadioAutoDone(ptr,bON,bVal);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

}

void DC1394Thread::sliderPower(QVariantList arg)
{
    qDebug() << "sliderPower";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    bool bON = arg.at(2).toBool();
    feature = (cameraFeature_id_t)value;

    DC1394Control->setActiveDC1394(feature,bON);

    bool hasAuto = DC1394Control->hasAutoDC1394(feature);
    bool hasManual = DC1394Control->hasManualDC1394(feature);
    bool hasOnePush = DC1394Control->hasOnePushDC1394(feature);

    sliderPowerDone(ptr,bON,hasAuto,hasManual,hasOnePush);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }
}


void DC1394Thread::sliderHasFeature(QVariantList arg)
{
    qDebug() << "sliderHasFeature";

    cameraFeature_id_t feature;
    QObject *ptr = (QObject*)arg.at(0).value<void*>();
    int value = arg.at(1).toInt();
    feature = (cameraFeature_id_t)value;

    bool hasFeature = DC1394Control->hasFeatureDC1394(feature);
    hasFeature = true;// DC1394Control->hasFeatureDC1394(feature);

    opCounter--;
    if(opCounter == 0){
        stopLoading();
    }

    sliderHasFeatureDone(ptr,hasFeature);

}
