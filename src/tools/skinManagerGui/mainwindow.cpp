#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <math.h>
#include <yarp/os/Log.h>

MainWindow::MainWindow(ResourceFinder *rf,QWidget *parent) :
    QMainWindow(parent),loadingWidget(this),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->rf = rf;
    prevFreq = 0;
    currentSampleFreq = 5;
    smoothSliderPressed = false;

    connect(&portThread,SIGNAL(portCreated()),
            this,SLOT(onPortCreated()));
    connect(&portThread,SIGNAL(openErrorDialog(QString)),
            this,SLOT(onOpenErrorDialog(QString)));

    portThread.start();

    calibratingTimer.setInterval(100);
    calibratingTimer.setSingleShot(false);
    connect(&calibratingTimer,SIGNAL(timeout()),this,SLOT(onCalibratingTimer()),Qt::QueuedConnection);

    connect(ui->btnCalibrate,SIGNAL(clicked()),this,SLOT(onCalibrate()));
    connect(ui->btnShowTouchThres,SIGNAL(clicked()),this,SLOT(onThreashold()));
    connect(ui->btnBinarization,SIGNAL(toggled(bool)),this,SLOT(onBinarization(bool)));
    connect(ui->btnSmooth,SIGNAL(toggled(bool)),this,SLOT(onSmooth(bool)));
    connect(ui->sliderScaleSmooth,SIGNAL(valueChanged(int)),this,SLOT(onSmoothValueChanged(int)));
    connect(ui->sliderScaleSmooth,SIGNAL(sliderPressed()),this,SLOT(onSmoothValuePressed()));
    connect(ui->sliderScaleSmooth,SIGNAL(sliderReleased()),this,SLOT(onSmoothValueReleased()));

    connect(ui->spinNeighbor,SIGNAL(valueChanged(double)),this,SLOT(onSpinNeighborChanged(double)));
    connect(ui->spinCompContactGain,SIGNAL(valueChanged(double)),this,SLOT(onSpinCompContactGainChanged(double)));
    connect(ui->spinCompGain,SIGNAL(valueChanged(double)),this,SLOT(onSpinCompGainChanged(double)));
    connect(ui->spinThreashold,SIGNAL(valueChanged(int)),this,SLOT(onSpinThresholdChanged(int)));

    connect(ui->spinSampleFreq,SIGNAL(valueChanged(int)),this,SLOT(onSampleFreqChanged(int)));

    updateTimer.setSingleShot(false);
    updateTimer.setInterval(1000/currentSampleFreq);
    connect(&updateTimer,SIGNAL(timeout()),this,SLOT(onUpdateTimer()));


}

MainWindow::~MainWindow()
{
    delete ui;

    portThread.closePorts();
    portThread.stop();
}

void MainWindow::onOpenErrorDialog(QString msg)
{
    QMessageBox::critical(this,"Error",msg);
}

void MainWindow::onPortCreated()
{
    QString guiName                  = rf->check("name", Value("skinManGui")).asString().c_str();
    QString guiRpcPortName           = "/" + guiName + "/rpc:o";
    QString guiMonitorPortName       = "/" + guiName + "/monitor:i";
    QString guiInfoPortName          = "/" + guiName + "/info:i";

    if (!portThread.guiRpcPort->open(guiRpcPortName.toLatin1().data())) {
        QString msg = QString("Unable to open port %1").arg(guiRpcPortName);
        QMessageBox::critical(this,"Error",msg);
        return; //TODO EXIT
    }
    if (!portThread.driftCompMonitorPort->open(guiMonitorPortName.toLatin1().data())){
        QString msg = QString("Unable to open port %1").arg(guiMonitorPortName);
        QMessageBox::critical(this,"Error",msg);
        return; //TODO EXIT
    }
    if (!portThread.driftCompInfoPort->open(guiInfoPortName.toLatin1().data())){
        QString msg = QString("Unable to open port %1").arg(guiInfoPortName);
        QMessageBox::critical(this,"Error",msg);
        return; //TODO EXIT
    }

    portThread.driftCompInfoPort->setStrict();

    initGui();

}

void MainWindow::initGui()
{
    currentSampleFreq = 5;
    currentSampleNum = 100;

    // if the rpc port is connected, then initialize the gui status
    initDone = false;
    int outPorts = portThread.guiRpcPort->getOutputCount();
    if(outPorts>0){
        initDone = initGuiStatus();
    }

    if(initDone){
        printLog("GUI connected!");
    } else {
        printLog("GUI not connected. Connect it to the module to make it work.");
    }
    // otherwise the gui will try to initialize every timeout (i.e. 1 sec)

    updateTimer.start();
}



bool MainWindow::initGuiStatus(){
    Bottle reply = portThread.sendRpcCommand(true, get_binarization);
    if(string(reply.toString().c_str()).compare("on") == 0){
        ui->btnBinarization->setEnabled(true);
        ui->btnBinarization->setText("ON");
        ui->btnBinarization->setChecked(true);
    }else{
        ui->btnBinarization->setEnabled(true);
        ui->btnBinarization->setText("OFF");
        ui->btnBinarization->setChecked(false);
    }

    reply = portThread.sendRpcCommand(true, get_smooth_filter);
    if(string(reply.toString().c_str()).compare("on") == 0){
        ui->btnSmooth->setText("ON");
        ui->sliderContainer->setEnabled(true);
        ui->btnSmooth->setChecked(true);
    }else{
        ui->btnSmooth->setText("OFF");
        ui->sliderContainer->setEnabled(false);
        ui->btnSmooth->setChecked(false);
    }

    reply = portThread.sendRpcCommand(true, get_smooth_factor);
    currentSmoothFactor = reply.get(0).asDouble();
    ui->sliderScaleSmooth->setValue(currentSmoothFactor * 10);

    onSmoothValueChanged(currentSmoothFactor * 10);

    reply = portThread.sendRpcCommand(true, get_threshold);
    if(reply.isNull() || reply.size()==0 || !reply.get(0).isInt()){
        printLog("Error while getting the safety threshold");
        return false;
    }else{
        currentThreshold = reply.get(0).asInt();
        ui->spinThreashold->setValue(currentThreshold);
    }

    reply = portThread.sendRpcCommand(true, get_gain);
    if(reply.isNull() || reply.size()==0 || (!reply.get(0).isDouble() && !reply.get(0).isInt())){
        printLog("Error while getting the compensation gain");
        return false;
    }else{
        currentCompGain = reply.get(0).asDouble();
        ui->spinCompGain->setValue(currentCompGain);
    }

    reply = portThread.sendRpcCommand(true,get_cont_gain);
    if(reply.isNull() || reply.size()==0 || (!reply.get(0).isDouble() && !reply.get(0).isInt())){
        printLog("Error while getting the contact compensation gain");
        return false;
    }else{
        currentContCompGain = reply.get(0).asDouble();
        ui->spinCompContactGain->setValue(currentContCompGain);
    }

    reply = portThread.sendRpcCommand(true,get_max_neigh_dist);
    if(reply.isNull() || reply.size()==0 || (!reply.get(0).isDouble() && !reply.get(0).isInt())){
        printLog("Error while getting the max neighbor distance");
        return false;
    }else{
        currentMaxNeighDist = reply.get(0).asDouble();
        ui->spinNeighbor->setValue(currentMaxNeighDist * 100);
    }

    // get module information
    reply = portThread.sendRpcCommand(true, get_info);
    if(reply.isNull() || reply.size()!=3){
        printLog("Error while reading the module information");
        return false;
    }
    stringstream ss;
    ss<< reply.get(0).toString().c_str()<< endl;
    ss<< reply.get(1).toString().c_str()<< "\nInput ports:";
    Bottle* portList = reply.get(2).asList();
    portNames.resize(portList->size()/2);
    portDim.resize(portList->size()/2);
    //int numTaxels = 0;
    for(unsigned int i=0;i<portDim.size();i++){
        portNames[i] = portList->get(i*2).toString().c_str();
        portDim[i] = portList->get(i*2+1).asInt();
        //numTaxels += portDim[i];
        ss<< "\n - "<< portNames[i]<< " ("<< portDim[i]<< " taxels)";
    }
    ui->infoPanel->setPlainText(QString("%1").arg(ss.str().c_str()));


    // check whether the skin calibration is in process
    reply = portThread.sendRpcCommand(true, is_calibrating);
    if(string(reply.toString().c_str()).compare("yes")==0){
        loadingWidget.start();
        calibratingTimer.start();
    }

    ui->controlsWidget->setEnabled(true);
    ui->sampleFreqContainer->setEnabled(true);
    ui->treeCompensation->setEnabled(true);
    return true;
}


void MainWindow::onCalibratingTimer()
{
    Bottle reply = portThread.sendRpcCommand(true,is_calibrating);
    if(string(reply.toString().c_str()).compare("yes")==0){
        return;
    }

    ui->controlsWidget->setEnabled(true);
    calibratingTimer.stop();
    loadingWidget.stop();
    printLog("Calibrating Done!");
    QString msg = "Calibrating Done!";
    ui->statusBar->showMessage(msg);
    ui->infoPanel->appendPlainText(msg);

}

void MainWindow::onCalibrate()
{
    portThread.sendRpcCommand(false,calibrate);
    loadingWidget.start("Calibrating...don't touch the skin!!");
    calibratingTimer.start();
    ui->controlsWidget->setEnabled(false);
}

void MainWindow::onSpinThresholdChanged(int value)
{
    int safetyThr = value;
    if(safetyThr == currentThreshold)
        return;

    // set the threshold
    Bottle b, setReply;
    b.addInt(set_threshold);
    b.addInt(safetyThr);
    portThread.guiRpcPort->write(b, setReply);

    // read the threshold
    Bottle getReply = portThread.sendRpcCommand(true, get_threshold);
    currentThreshold = getReply.get(0).asInt();

    if(safetyThr==currentThreshold){
        QString msg = QString("Safety threshold changed: %1").arg(safetyThr);
        ui->statusBar->showMessage(msg);
        ui->infoPanel->appendPlainText(msg);
        return;
    }

    QString msg = QString("Unable to set the threshold to: %1 m.\nSet command reply: %2").arg(safetyThr).arg(setReply.toString().c_str());
    QMessageBox::critical(this,"Error",msg);

    // setting the old value
    ui->spinThreashold->setValue(currentThreshold);
}

void MainWindow::onSpinCompGainChanged(double value)
{
    double compGain = value;
    if(compGain == currentCompGain)
        return;

    // set the gain
    Bottle b, setReply;
    b.addInt(set_gain);
    b.addDouble(compGain);
    portThread.guiRpcPort->write(b, setReply);

    // read the gain
    Bottle getReply = portThread.sendRpcCommand(true, get_gain);
    currentCompGain = getReply.get(0).asDouble();

    if(compGain==currentCompGain){
        QString msg = QString("Compensation gain changed: %1").arg(compGain);
        ui->statusBar->showMessage(msg);
        ui->infoPanel->appendPlainText(msg);
        return;
    }
    QString msg = QString("Unable to set the compensation gain to: %1 m.\nSet command reply: %2").arg(compGain).arg(setReply.toString().c_str());
    QMessageBox::critical(this,"Error",msg);

    // setting the old value
    ui->spinCompGain->setValue(currentCompGain);
}

void MainWindow::onSpinCompContactGainChanged(double value)
{
    double contCompGain = value;
    if(contCompGain == currentContCompGain){
        return;
    }

    // set the gain
    Bottle b, setReply;
    b.addInt(set_cont_gain);
    b.addDouble(contCompGain);
    portThread.guiRpcPort->write(b, setReply);

    // read the gain
    Bottle getReply = portThread.sendRpcCommand(true, get_cont_gain);
    currentContCompGain = getReply.get(0).asDouble();

    if(contCompGain==currentContCompGain){
        QString msg = QString("Contact compensation gain changed: %1").arg(contCompGain);
        ui->statusBar->showMessage(msg);
        ui->infoPanel->appendPlainText(msg);
        return;
    }

    QString msg = QString("Unable to set the contact compensation gain to: %1 m.\nSet command reply: %2").arg(contCompGain).arg(setReply.toString().c_str());
    QMessageBox::critical(this,"Error",msg);

    // setting the old value
    ui->spinCompContactGain->setValue(currentContCompGain);
}

void MainWindow::onSpinNeighborChanged(double value)
{
    double maxNeighDist = 1e-2*value; // TODO ????????????????????????????????????

    if(maxNeighDist == currentMaxNeighDist){
        return;
    }

    // set the value
    Bottle b, setReply;
    b.addInt(set_max_neigh_dist);
    b.addDouble(maxNeighDist);
    portThread.guiRpcPort->write(b, setReply);

    // read the value
    Bottle getReply = portThread.sendRpcCommand(true, get_max_neigh_dist);
    currentMaxNeighDist = getReply.get(0).asDouble();

    if(maxNeighDist==currentMaxNeighDist){
        QString msg = QString("Max neighbor distance changed: %1 m").arg(maxNeighDist);
        ui->statusBar->showMessage(msg);
        ui->infoPanel->appendPlainText(msg);
        return;
    }

    QString msg = QString("Unable to set the max neighbor distance to: %1 m.\nSet command reply: %2").arg(maxNeighDist).arg(setReply.toString().c_str());
    QMessageBox::critical(this,"Error",msg);
    // setting the old value
    ui->spinNeighbor->setValue(currentMaxNeighDist);

}

void MainWindow::onSmoothValuePressed()
{
    smoothSliderPressed = true;
}

void MainWindow::onSmoothValueReleased()
{
    if(!smoothSliderPressed){
        return;
    }

    smoothSliderPressed = false;

    changeSmooth(ui->sliderScaleSmooth->value());


}

void MainWindow::changeSmooth(int val)
{
    // check whether the smooth factor has changed
    double value = (double)val/10;
    double smoothFactor =  round(value, 1); //double(int((value*10)+0.5))/10.0;
    if(smoothFactor==currentSmoothFactor){
        return;
    }


    // set the smooth factor
    Bottle b, setReply;
    b.addInt(set_smooth_factor);
    b.addDouble(smoothFactor);
    portThread.guiRpcPort->write(b, setReply);

    // read the smooth factor
    Bottle getReply = portThread.sendRpcCommand(true, get_smooth_factor);
    currentSmoothFactor = getReply.get(0).asDouble();
    currentSmoothFactor = round(currentSmoothFactor, 1); //double(int((currentSmoothFactor*10)+0.5))/10.0;

    if(smoothFactor==currentSmoothFactor){
        QString msg = QString("Smooth factor changed: %1").arg(smoothFactor);
        ui->statusBar->showMessage(msg);
        ui->infoPanel->appendPlainText(msg);
        return;
    }
    QString msg = QString("Unable to set the smooth factor to: %1\nSet command reply: ").arg(smoothFactor).arg(setReply.toString().c_str());
    QMessageBox::critical(this,"Error",msg);
    // setting the old value

    ui->sliderScaleSmooth->setValue(currentSmoothFactor*10);
}

void MainWindow::onThreashold()
{
    Bottle touchThr = portThread.sendRpcCommand(true, get_touch_thr);
    int index = 0;
    for(unsigned int j=0; j<portDim.size(); j++){
        stringstream msg;
        msg<< "Thresholds for port "<< portNames[j]<< ":\n";
        for(unsigned int i=0; i<portDim[j]; i++){
            if(i%12==0){
                if(i!=0){
                    msg<< "\n";
                }
                msg<< "TR"<< i/12<< ":\t";
            }
            msg << int(touchThr.get(index).asDouble())<< ";\t";
            index++;
        }
        QMessageBox::information(this,"Information",QString("%1").arg(msg.str().c_str()));
    }
}

void MainWindow::onSmoothValueChanged(int val)
{
    QString d = QString("%L1").arg((double)val/10,0,'f',1);
    ui->lblSmoothValue->setText(d);

    int w = ui->sliderScaleSmooth->width() - 30;
    double newX = ((double)w/(double)10) * (double)val;
    ui->lblSmoothValue->setGeometry(newX,0,30,20);
}

void MainWindow::onSmooth(bool btnState)
{
    // if the button is off it means it is going to be turned on
    if (btnState){
        Bottle b, setReply;
        b.addInt(set_smooth_filter);
        b.addString("on");
        portThread.guiRpcPort->write(b, setReply);
        Bottle reply = portThread.sendRpcCommand(true, get_smooth_filter);
        if(string(reply.toString().c_str()).compare("on") == 0){
            ui->btnSmooth->setText("ON");
            ui->smoothControlContainer->setEnabled(true);
            QString msg = "Smooth filter turned on";
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }else{
            QString msg = QString("Error! Unable to turn the smooth filter on: %1").arg(reply.toString().c_str());
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }
    } else {
        Bottle b, setReply;
        b.addInt(set_smooth_filter); b.addString("off");
        portThread.guiRpcPort->write(b, setReply);
        Bottle reply = portThread.sendRpcCommand(true, get_smooth_filter);
        if(string(reply.toString().c_str()).compare("off") == 0){
            ui->btnSmooth->setText("OFF");
            ui->smoothControlContainer->setEnabled(false);
            QString msg = "Smooth filter turned off";
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }else{
            QString msg = QString("Error! Unable to turn the smooth filter off: %1").arg(reply.toString().c_str());
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }
    }
}

void MainWindow::onBinarization(bool btnState)
{
    if (btnState){
        Bottle b, setReply;
        b.addInt(set_binarization);
        b.addString("on");
        portThread.guiRpcPort->write(b, setReply);
        Bottle reply = portThread.sendRpcCommand(true, get_binarization);
        if(string(reply.toString().c_str()).compare("on")==0){
            ui->btnBinarization->setText("ON");
            QString msg = "Binarization filter turned on";
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }else{
            QString msg = QString("Error! Unable to turn the binarization filter on: %1").arg( reply.toString().c_str());
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }
    } else {
        Bottle b, setReply;
        b.addInt(set_binarization);
        b.addString("off");
        portThread.guiRpcPort->write(b, setReply);
        Bottle reply = portThread.sendRpcCommand(true, get_binarization);
        if(string(reply.toString().c_str()).compare("off")==0){
            ui->btnBinarization->setText("OFF");
            QString msg = "Binarization filter turned off";
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }else{
            QString msg = QString("Error! Unable to turn the binarization filter off: %1").arg(reply.toString().c_str());
            ui->statusBar->showMessage(msg);
            ui->infoPanel->appendPlainText(msg);
        }
    }
}


void MainWindow::printLog(QString text)
{
    QString msg = text + "\n";
    //ui->logPanel->appendPlainText(msg);
    //qDebug() << msg;
    yDebug("%s", msg.toLatin1().data());
}

double MainWindow::round(double value, int decimalDigit){
    double q = pow(10.0, decimalDigit);
    return double(int((value*q)+0.5))/q;
}


void MainWindow::onUpdateTimer()
{
    // if the gui has not been initialized yet
    if(!initDone){
        // if the rpc port is connected, then initialize the gui status
        if(portThread.guiRpcPort->getOutputCount()>0){
            initDone = initGuiStatus();
        }

        if(initDone){
            printLog("GUI connected!");
        } else {
            return;
        }
    }

    if(portThread.driftCompMonitorPort->getInputCount()==0){
        setMsgFreq(false, 0);
    }else{
        Vector* b = portThread.driftCompMonitorPort->read(false);
        if(!b || b->size()==0){
            setMsgFreq(false, 0);
        }else{
            //set the frequency
            double freq = (*b)[0];
            setMsgFreq(true, freq);

            // set the drift
            int numTax = 0;
            for(unsigned int i=0; i<portDim.size(); i++){
                numTax += portDim[i];
            }
            const int numTr = numTax/12;
            if(numTax>0){

                // *** UPDATE DRIFT TREE VIEW (2ND TAB)
                vector<float> driftPerTr(numTr);
                double sumTr;
                for(int i=0; i<numTr; i++){
                    sumTr=0;
                    for(int j=0; j<12; j++){
                        sumTr += (float) (*b)(i*12+j+1);
                    }
                    driftPerTr[i] = (float)round(sumTr/12.0, 2);
                }

                if(ui->treeCompensation->topLevelItemCount() <= 0){
                    for(size_t i=0; i<portNames.size();i++){
                        QTreeWidgetItem *portItem = new QTreeWidgetItem();
                        ui->treeCompensation->addTopLevelItem(portItem);

                        for(unsigned int j=0; j<portDim[i]/12; j++){
                            QTreeWidgetItem *trItem = new QTreeWidgetItem();
                            portItem->addChild(trItem);

                            for(int k=0; k<12; k++){
                                QTreeWidgetItem *taxelItem = new QTreeWidgetItem();
                                trItem->addChild(taxelItem);
                            }

                        }
                    }
                }

                double sumPort, meanPort;
                stringstream trS, taxS;
                int index=1;
                int portIndex=0;
                double meanTr;

                for(size_t i=0; i<portNames.size();i++){
                    sumPort = 0;

                    QTreeWidgetItem *portItem = ui->treeCompensation->topLevelItem(i);

                    for(unsigned int j=0; j<portDim[i]/12; j++){
                        meanTr = driftPerTr[portIndex];
                        portIndex++;
                        sumPort += meanTr;
                        trS.str("");
                        trS<<j;

                        QTreeWidgetItem *trItem = portItem->child(j);
                        trItem->setText(1,trS.str().c_str());
                        trItem->setText(3,QString("%L1").arg(meanTr,0,'f',3));

                        for(int k=0; k<12; k++){
                            double drift = round((*b)(index), 2);
                            index++;
                            taxS.str("");
                            taxS<<k;
                            QTreeWidgetItem *taxelItem = trItem->child(k);
                            if(taxelItem){
                                taxelItem->setText(2,taxS.str().c_str());
                                taxelItem->setText(3,QString("%L1").arg(drift,0,'f',3));
                            }
                        }
                    }

                    meanPort = sumPort/(portDim[i]/12);
                    portItem->setText(0, portNames[i].c_str());
                    portItem->setText(3, QString("%L1").arg(meanPort,0,'f',3));
                }

            }
        }
    }

    // check if there are messages on the info port
    Bottle* infoMsg = portThread.driftCompInfoPort->read(false);
    while(infoMsg){

        printLog(infoMsg->toString().c_str());
        infoMsg = portThread.driftCompInfoPort->read(false);
    }
}

void MainWindow::onSampleFreqChanged(int value)
{
    if (value == currentSampleFreq || !updateTimer.isActive()){
        return;
    }
    currentSampleFreq = value;
    updateTimer.stop();
    updateTimer.setInterval(1000/currentSampleFreq);
    updateTimer.start();

}

void MainWindow::setMsgFreq(bool freqUpdated, double freq){
    QString text;
    double auxFreq = round(freq, 2);
    if(freqUpdated){
        text = QString("SkinDriftCompensation frequency: %L1").arg(freq,0,'f',3);
    }else{
        auxFreq = -1;
        text = "Cannot read the frequency. Probably the skinDriftCompensation module has stopped working.";
    }
    if(auxFreq != prevFreq){
        //ui->statusBar->showMessage(text);
        ui->infoPanel->appendPlainText(text);
        prevFreq = auxFreq;
    }
}
