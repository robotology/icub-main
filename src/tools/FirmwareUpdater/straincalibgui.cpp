#include "straincalibgui.h"
#include "ui_straincalibgui.h"
#include <QDebug>
#include <QtConcurrent/QtConcurrent>
#include <QFileDialog>


//#define NSAMPLES 1000

StrainCalibGui::StrainCalibGui(QString device, int bus, int pid, FirmwareUpdaterCore *core, QWidget *parent) :
    QDialog(parent), mutexacquisitionofdata(QMutex::Recursive), mutexdriver(QMutex::Recursive),
    ui(new Ui::StrainCalibGui)
{
    progressvalue = 0;
    acquiringdata = false;
    enableacquisitions = true;
    ui->setupUi(this);
    this->core = core;
    fp = NULL;
    trial=-1;
    current_trial = -1;
    connect(ui->btnAcquireData,SIGNAL(clicked(bool)),this,SLOT(onAcquireData(bool)));
    buttons.append(ui->btn1);
    buttons.append(ui->btn2);
    buttons.append(ui->btn3);
    buttons.append(ui->btn4);
    buttons.append(ui->btn5);
    buttons.append(ui->btn6);
    buttons.append(ui->btn7);
    buttons.append(ui->btn8);
    buttons.append(ui->btn9);
    buttons.append(ui->btn10);
    buttons.append(ui->btn11);
    buttons.append(ui->btn12);
    buttons.append(ui->btn13);

    readValues.append(ui->cur1);
    readValues.append(ui->cur2);
    readValues.append(ui->cur3);
    readValues.append(ui->cur4);
    readValues.append(ui->cur5);
    readValues.append(ui->cur6);

    expValues.append(ui->exp1);
    expValues.append(ui->exp2);
    expValues.append(ui->exp3);
    expValues.append(ui->exp4);
    expValues.append(ui->exp5);
    expValues.append(ui->exp6);

    errValues.append(ui->err1);
    errValues.append(ui->err2);
    errValues.append(ui->err3);
    errValues.append(ui->err4);
    errValues.append(ui->err5);
    errValues.append(ui->err6);

    ui->freeEdit1->setValidator(&validator);
    ui->freeEdit2->setValidator(&validator);
    ui->freeEdit3->setValidator(&validator);
    ui->freeEdit4->setValidator(&validator);
    ui->freeEdit5->setValidator(&validator);
    ui->freeEdit6->setValidator(&validator);


    int bi[13]={8020,1001,8010,8030,8040,8050,8060,8070,8080,3001,4001,5001,6001};

    for (int i=0; i<buttons.count();i++) {
        buttons.at(i)->setProperty("btnIndex",bi[i]);
        connect(buttons.at(i),SIGNAL(clicked(bool)),this,SLOT(onButtonClick(bool)));
    }

    connect(ui->freeAcqModeGroup,SIGNAL(toggled(bool)),this,SLOT(onFreeAcqMode(bool)));
    //connect(ui->buttonBox,SIGNAL(accepted()),this,SLOT(close()));
    connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(onClose()));


    std::string filenameOfexpectedValues = std::string("firmwareupdater.strain.expectedvalues.txt");
    bool b = expected_values_handler.init(filenameOfexpectedValues.c_str());
    if(!b){
        yError() << "ERROR in opening file" << filenameOfexpectedValues << ".... using default values for expected value handler";
        expected_values_handler.init();
    }
    else
    {
        yDebug() << "loaded file of expected values =" << filenameOfexpectedValues;
    }

    int low = 0;
    int high = 0;
    expected_values_handler.get_thresholds(low, high);
    yDebug() << "expected values for" << expected_values_handler.get_serialnumber() << "with thresholds = " << low << high;

//    QString tt = QString("Current Error [%1,  %2]").arg(low, high);
//    ui->label_16->setText(tt);

    connect(&watcher, SIGNAL(finished()), this, SLOT(onFutureFinished()));

#ifdef ATI_SENS
    ATIsens = new devMeasurementFns();
#endif


    if((device == "ETH") || (device == "eth"))
    {
        config.network = strainInterface::Network::ETH;
        yDebug() << "strainInterface::Network::ETH";
    } 
    else if ((device == "SOCKETCAN") || (device == "socketcan"))
    {
        yDebug() << "strainInterface::Network::socketcan";
        config.network = strainInterface::Network::socketcan;
    } 
    else if((device == "ECAN") || (device == "ecan"))
    {
        yDebug() << "strainInterface::Network::ecan";
        config.network = strainInterface::Network::ecan;
    }
    else
    {
        config.network = strainInterface::Network::unknown;
        yDebug() << "unknown strainInterface::Network =" << device.toStdString();
    }

    config.canbus = (strainInterface::CanBus)bus;
    config.canaddress = (strainInterface::CanAddress)pid;
    config.txrate = 2;
//    config.load_default();
//    if(!sI.open(config)){
//        qDebug() << "Error opening device";
//    }


    connect(&timer,SIGNAL(timeout()),this,SLOT(onTimerTimeout()));
    timer.setInterval(500);
    timer.setSingleShot(false);
    timer.start();

    enabledebugprints = false;


    if(false == checkDirectory())
    {
        yError() << "CANNOT find the data folder. Please create it!";
        ui->labelInstructions->setText("You can select an acquisition from the menu ... but:"
                                       "\nCANNOT find the data folder."
                                       "\nIf you want your acquisition data saved, please create it!");
    }
//    else
//    {
//         ui->labelInstructions->setText("Select an acquisition from the menu");
//    }

}

StrainCalibGui::~StrainCalibGui()
{
    //sI.close();


    stopacquisitions();

    delete ui;
}

void StrainCalibGui::stopacquisitions(bool forcestop)
{
    // wait all acquisitions to be over:
    // - stop timer so that tick_acquisition() is not called anymore
    // - wait pending acquisitions to be over by calling two lockdriver() true / false

    static bool alreadystopped = false;
    if((false == alreadystopped) || (true == forcestop))
    {
        // we must be sure that no tick_acquisition() is executed: we stop the timer so that no future
        // callback onTimerTimeout() is called. but it may be executing rigth now.
        // hence we first set enableacquisitions = false for furthe security
        enableacquisitions = false;
        timer.stop();

        // we now wait until any acquisition is over and we force stop of strain tx. twice!
        while(false == mutexdriver.tryLock(250))
        {
            ui->progress->setValue(progressvalue);
            ui->labelInstructions->setText("Received the QUIT command: waiting for the acquisition to terminate");
        }
        //lockdriver(true);
        // marco.accame: on windows, esdcan driver cannot manage streaming mode in reception.... sic
        cDownloader::strain_acquisition_mode_t acqmode = (strainInterface::Network::ETH == config.network) ? (cDownloader::strain_acquisition_mode_streaming) : (cDownloader::strain_acquisition_mode_polling);
        core->getDownloader()->strain_acquire_stop(config.get_canbus(), config.get_canaddress(), acqmode);
        core->getDownloader()->strain_acquire_stop(config.get_canbus(), config.get_canaddress(), acqmode);
        lockdriver(false);

        alreadystopped = true;
    }
}

void StrainCalibGui::onClose()
{
    stopacquisitions(true);
    stopacquisitions(true);

    close();
}

void StrainCalibGui::onFreeAcqMode(bool b)
{
    if(b){
        ui->spinSamples->setEnabled(true);
        ui->btnContainer1->setEnabled(false);
        ui->btnContainer2->setEnabled(false);
        ui->btnAcquireData->setEnabled(true);
        ui->labelInstructions->setText("Free Acquisiton Mode");
    } else {
        ui->btnContainer1->setEnabled(true);
        ui->btnContainer2->setEnabled(true);
        ui->btnAcquireData->setEnabled(false);

        if(false == checkDirectory())
        {
            yError() << "CANNOT find the data folder. Please create it!";
            ui->labelInstructions->setText("You can select an acquisition from the menu ... but:"
                                           "\nCANNOT find the data folder."
                                           "\nIf you want your acquisition data saved, please create it!");
        }
        else
        {
             ui->labelInstructions->setText("Select an acquisition from the menu");
        }
    }
}

void StrainCalibGui::onFutureFinished()
{
    ui->progress->setMaximum(100);

    if(!watcher.result()){
        ui->labelInstructions->setText("Not enough data acquired. Repeat acquisition.");
        close_files();
        return;
    }


    if(ui->freeAcqModeGroup->isChecked())
    {
        close_files();
        return;
    }
    else
    {
        int rtrial=remap_trials();

        if (rtrial==-1){
            trial_bias=last_value;
        }

        trial++;
        qDebug("%d\n",trial); //debug only
        showMenu();
    }

}

void StrainCalibGui::onAcquireData(bool b)
{

    if(ui->freeAcqModeGroup->isChecked())
    {
        QString fileName = QFileDialog::getSaveFileName(NULL,"Choose a file");
        if(fileName.isEmpty())
        {
            yWarning() << "choose another file";
            return;
        }
        fp = fopen(fileName.toLatin1().data(),"w");
        if(nullptr == fp)
        {
            yWarning() << "cannot open file" << fileName.toLatin1().data() << "hence i will not perform acquisition";
            ui->labelInstructions->setText("Cannot open file. Try once more.");
            return;
        }
    }

    QFuture<bool> future = QtConcurrent::run(this,&StrainCalibGui::acquire_samples,ui->spinSamples->value());
    watcher.setFuture(future);
    ui->progress->setMaximum(100);

}


void StrainCalibGui::onButtonClick(bool b)
{
    QPushButton *btn = (QPushButton*)sender();
    int btnIndex = btn->property("btnIndex").toInt();
    currentBtnIndex = buttons.indexOf(btn);
    trial = btnIndex;
    //ui->btn3->setStyleSheet("background-color: rgb(0, 175, 0);");
    // 4/9/2018 Tolto con Marco Accame perchè non ritenuto comprensibile
    turnOffButtons();
    showMenu();

}

#ifdef ATI_SENS
void acquire_1000_samples()
{
    char temp[255];
    float data[6]={0,0,0,0,0,0};
    printf("\n   press return to acquire ***ATI sensor*** data or 'r' to reset\n");
    cin >> temp;

    if (strcmp(temp,"r")==0)
    {
        ATIsens->BiasForceTorqueSensor();
        printf("\n   bias done. press return to acquire ***ATI sensor*** data\n");
        cin >> temp;
    }

    int i=0;
    int j=0;
    float data_mean[6] = {0,0,0,0,0,0};
    for (i=0; i<1000; i++)
    {
        Sleep(5);
        ATIsens->ReadFTsensorData();
        ATIsens->GetFTData(data);
        fprintf(fp,"%3d %+f %+f %+f %+f %+f %+f\n",i,data[0],data[1],data[2],data[3],data[4],data[5]);
        printf("samples:%3d [0]:%+6.2f [1]:%+6.2f [2]:%+6.2f [3]:%+6.2f [4]:%+6.2f [5]:%+6.2f\r\n",i,data[0],data[1],data[2],data[3],data[4],data[5]);
        for (j=0; j<6; j++)
        {
            data_mean[j]=data_mean[j]+data[j];
        }
    }
    for (j=0; j<6; j++)
        {
            data_mean[j]/=1000;
        }
    //fprintf(fp2,"%3d %+f %+f %+f %+f %+f %+f\n",i,data_mean[0],data_mean[1],data_mean[2],data_mean[3],data_mean[4],data_mean[5]);
    printf("means:%3d [0]:%+6.2f [1]:%+6.2f [2]:%+6.2f [3]:%+6.2f [4]:%+6.2f [5]:%+6.2f\r\n",i,data_mean[0],data_mean[1],data_mean[2],data_mean[3],data_mean[4],data_mean[5]);


    trial++;
}

#else

void StrainCalibGui::progressbar(void *owner, float percent)
{
    if(nullptr == owner)
    {
        return;
    }

    int value = static_cast<int>(percent * 100.0);
//    value /= 10;
//    value *= 10;

    StrainCalibGui *thegui = reinterpret_cast<StrainCalibGui*>(owner);

    if(value != thegui->progressvalue)
    {
        thegui->progressvalue = value;
    }
}

bool StrainCalibGui::get(const unsigned int number, vector<cDownloader::strain_value_t> &values, bool debugprint, bool updateprogress)
{

    double t0 = yarp::os::SystemClock::nowSystem();

    if(debugprint)
    {
        yDebug() << "strainInterface::get(): is acquiring" << number << "from the 6 channels. Please wait ...";
    }

    void * param = nullptr;
    void (*updateProgressBar)(void*, float) = nullptr;

    if(true == updateprogress)
    {
        updateProgressBar = progressbar;
        param = reinterpret_cast<void*>(this);
    }

    const bool calibmode = false;
    // marco.accame: on windows, esdcan driver cannot manage streaming mode in reception....sic
    cDownloader::strain_acquisition_mode_t acqmode = (strainInterface::Network::ETH == config.network) ? (cDownloader::strain_acquisition_mode_streaming) : (cDownloader::strain_acquisition_mode_polling);
    core->getDownloader()->strain_acquire_start(config.get_canbus(), config.get_canaddress(), config.get_txrate(), calibmode, acqmode);
    int ret = core->getDownloader()->strain_acquire_get(config.get_canbus(), config.get_canaddress(), values, number, updateProgressBar, param, acqmode);
    //yarp::os::SystemClock::delaySystem(0.100); i used it to test the flush operation of strain_acquire_stop()....
    core->getDownloader()->strain_acquire_stop(config.get_canbus(), config.get_canaddress(), acqmode);
    core->getDownloader()->strain_acquire_stop(config.get_canbus(), config.get_canaddress(), acqmode);

    double t1 = yarp::os::SystemClock::nowSystem();

    if(debugprint)
    {
        if (0 == ret)
        {
            yDebug() << "strainInterface::get() has succesfully acquired" << values.size() << "values of the 6 channels in" << (t1 - t0) << "seconds";
            yDebug() << "the values are:";
            for (int i = 0; i < values.size(); i++)
            {
                yDebug() << "#" << i + 1 << "=" << values[i].channel[0] << values[i].channel[1] << values[i].channel[2] << values[i].channel[3] <<
                    values[i].channel[4] << values[i].channel[5] << values[i].valid;
            }
        }
        else
        {
            yDebug() << "strainInterface::get() has failed acquisitions";
        }
    }

    return (0 == ret) ? true : false;
}

bool StrainCalibGui::print(const vector<cDownloader::strain_value_t> &values, FILE *fp, QList<float> ft)
{

    if(nullptr == fp)
    {
        yError("the file pointer is invalid: I cannot print my %lu values", values.size());
        return false;
    }

    if(!ft.isEmpty() && (ft.count() == 6)){
        fprintf(fp,"- %f %f %f %f %f %f\n\n",ft.at(0),ft.at(1),ft.at(2),ft.at(3),ft.at(4),ft.at(5));
        fflush(fp);
    }

    for(size_t n=0; n<values.size(); n++)
    {
        unsigned short unsigned_gaugeData[6] = {0};
        signed short signed_gaugeData[6] = {0};

        for(size_t c=0; c<6; c++)
        {
            unsigned_gaugeData[c] = values[values.size()-1-n].channel[c];
            signed_gaugeData[c] = unsigned_gaugeData[c]-0x7fff;
        }

        if(NULL != fp)
        {
            fprintf(fp,"%d %d %d %d %d %d %d\n",static_cast<int>(n),signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],
                    signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
            fflush(fp);
        }

        if(enabledebugprints)
        {
            yDebug("samples:%3d [0]:%+6d [1]:%+6d [2]:%+6d [3]:%+6d [4]:%+6d [5]:%+6d\r\n",static_cast<int>(n),signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
        }
    }

    return true;
}

bool StrainCalibGui::acquire_samples(int samples)
{
    // marco.accame: now we acquire with the strainInterface class in one shot

    const bool enabledebugprints = false;

    if(false == enableacquisitions)
    {
        return false;
    }


    lockdriver(true);
    setAcquisitionOfDataActive(true);

    vector<cDownloader::strain_value_t> values;
    bool ret = get(samples, values, enabledebugprints, true);

    if (ret)
    {

        if (samples != values.size())
        {
            yError() << "cannot acquire enough strain samples. Only:" << values.size() << "instead of" << samples;
        }
        else
        {
            if (enabledebugprints)
            {
                yDebug("Acquired %lu strain samples. Ready! \n", values.size());
            }
        }

        if (ui->freeAcqModeGroup->isChecked()) {
            QList <float> v;
            v.append(ui->freeEdit1->text().toFloat());
            v.append(ui->freeEdit2->text().toFloat());
            v.append(ui->freeEdit3->text().toFloat());
            v.append(ui->freeEdit4->text().toFloat());
            v.append(ui->freeEdit5->text().toFloat());
            v.append(ui->freeEdit6->text().toFloat());
            print(values, fp, v);
        }
        else {
            print(values, fp);
        }


        // now i need to retrieve the most recent value from values but in a particular format and fill variable last_value

        cDownloader::strain_value_t lastvalue = values.at(values.size() - 1);

        lastvalue.extract(last_value.dat);

    }
    else
    {
        fprintf(fp, "acquisition error. please check log on terminal.\n");
        fflush(fp);
    }

    setAcquisitionOfDataActive(false);
    lockdriver(false);

    return true;
}

#endif

void StrainCalibGui::showMenu()
{
    switch (trial) {

    case 0:
        fp = fopen("./data/output0.dat","w");
        ui->groupBox->setTitle("Beginning calibration ...");
        ui->labelInstructions->setText("Remove all loads from the sensor\n"
                                       "Orient the sensor with the flat surface facing upwards\n"
                                       "Turn the sensor on (CAN message 205 2 7 0)");
        terminate_section();
        break;
    case  1001:
        fp = fopen("./data/output1.dat","w");
        ui->groupBox->setTitle("(2) z+ pointing DOWNwards 5kg torques - file: output1.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 nut\n"
                                       "2. orient the assembly with the z+ axis pointing downwards\n\n"
                                       "3. remove loads");
        break;
    case  1002:
        ui->labelInstructions->setText("4.  apply 5.00 kg on the y- axis\n");
        break;
    case  1003:
        ui->labelInstructions->setText("5.   apply 5.00 kg on the x+ axis\n");
        break;
    case  1004:
        ui->labelInstructions->setText("6.   apply 5.00 kg on the y+ axis\n");
        break;
    case  1005:
        ui->labelInstructions->setText("7.   apply 5.00 kg on the x- axis\n");
        break;
    case  1006:
        terminate_section();
        break;
    case 3001:
        fp = fopen("./data/output3.dat","w");
        ui->groupBox->setTitle("(10) x+ axis pointing UPwards 5kg laterals - file: output3.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the structure with the x+ axis pointing upwards\n"
                                       "3. remove loads\n");
        break;
    case  3002:
        ui->labelInstructions->setText("4.   apply 5.00 kg on the y- axis\n");
        break;
    case  3003:
        ui->labelInstructions->setText("5.   apply 5.00 kg on the z+ axis\n");
        break;
    case  3004:
        ui->labelInstructions->setText("6.   apply 5.00 kg on the y+ axis\n");
        break;
    case 3005:
        terminate_section();
        break;
    case 4001:
        fp = fopen("./data/output4.dat","w");
        ui->groupBox->setTitle("(11) y+ axis pointing UPwards 5kg laterals - file: output4.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the structure with the y+ axis pointing upwards\n"
                                       "3. remove loads\n");
        break;
    case  4002:
        ui->labelInstructions->setText("4.   apply 5.00 kg on the x+ axis\n");
        break;
    case  4003:
        ui->labelInstructions->setText("5.   apply 5.00 kg on the z+ axis\n");
        break;
    case  4004:
        ui->labelInstructions->setText("6.   apply 5.00 kg on the x- axis\n");
        break;
    case 4005:
        terminate_section();
        break;

    case 5001:
        fp = fopen("./data/output5.dat","w");
        ui->groupBox->setTitle("(12) x- axis pointing UPwards 5kg laterals - file: output5.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the structure with the x- axis pointing upwards\n"
                                       "3. remove loads\n");
        break;
    case  5002:
        ui->labelInstructions->setText("4.   apply 5.00 kg on the y+ axis\n");
        break;
    case  5003:
        ui->labelInstructions->setText("5.   apply 5.00 kg on the z+ axis\n");
        break;
    case  5004:
        ui->labelInstructions->setText("6.   apply 5.00 kg on the y- axis\n");
        break;
    case 5005:
        terminate_section();
        break;


    case 6001:
        fp = fopen("./data/output6.dat","w");
        ui->groupBox->setTitle("(13) y- axis pointing UPwards 5kg laterals - file: output6.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the structure with the y- axis pointing upwards\n"
                                       "3. remove loads\n");
        break;
    case  6002:
        ui->labelInstructions->setText("4.   apply 5.00 kg on the x- axis\n");
        break;
    case  6003:
        ui->labelInstructions->setText("5.   apply 5.00 kg on the z+ axis\n");
        break;
    case  6004:
        ui->labelInstructions->setText("6.   apply 5.00 kg on the x+ axis\n");
        break;
    case 6005:
        terminate_section();
        break;


    case 8010:
        fp = fopen("./data/output81.dat","w");
        ui->groupBox->setTitle("(3) z+ pointing UPwards 25kg compression - file: output81.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 nut\n"
                                       "2. orient the assembly with the z+ axis pointing upwards\n\n"
                                       "3. remove loads\n");
        break;

    case  8011:
        ui->labelInstructions->setText("4. apply 25 kg in the z- direction (compression)\n");
        break;
    case 8012:
        terminate_section();
        break;


    case 8020:
        fp = fopen("./data/output82.dat","w");
        ui->groupBox->setTitle("(1) z+ pointing DOWNwards 25kg traction - file: output82.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 nut\n"
                                       "2. screw the M10 ring on the top of the assembly\n"
                                       "3. orient the assembly with the z+ axis pointing downwards\n\n"
                                       "4. remove loads\n");
        break;
    case  8021:
        ui->labelInstructions->setText("5. apply a 25 kg load in the z+ direction (traction)\n");
        break;
    case 8022:
        terminate_section();
        break;


    case 8030:
        fp = fopen("./data/output83.dat","w");
        ui->groupBox->setTitle("(4) x+ pointing UPwards 25kg - file: output83.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the plate with the x+ axis pointing upwards\n"
                                       "3. remove loads\n");
        break;
    case 8031:
        ui->labelInstructions->setText("4. apply a 25 kg load in the x- direction\n");
        break;
    case 8032:
        terminate_section();
        break;


    case 8040:
        fp = fopen("./data/output84.dat","w");
        ui->groupBox->setTitle("(5) x- pointing UPwards 25kg - file: output84.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the plate with the x- axis pointing upwards\n\n"
                                       "3. remove loads\n");
        break;
    case 8041:
        ui->labelInstructions->setText("4. apply a 25 kg load in the x+ direction\n");
        break;
    case 8042:
        terminate_section();
        break;

    case 8050:
        fp = fopen("./data/output85.dat","w");
        ui->groupBox->setTitle("(6) label 1 pointing DOWNwards 25kg - file: output85.dat");
        ui->labelInstructions->setText("1. assemble the assembly with label 1 pointing DOWN\n\n"
                                       "3. remove loads\n");
        break;
    case 8051:
        ui->labelInstructions->setText("4. apply 25kg\n");
        break;
    case 8052:
        terminate_section();
        break;


    case 8060:
        fp = fopen("./data/output86.dat","w");
        ui->groupBox->setTitle("(7) label 2 pointing DOWNwards 25kg - file: output86.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the plate with label 2 pointing DOWN\n\n"
                                       "3. remove loads\n");
        break;
    case 8061:
        ui->labelInstructions->setText("4. apply 25kg\n");
        break;
    case 8062:
        terminate_section();
        break;
    case 8070:
        fp = fopen("./data/output87.dat","w");
        ui->groupBox->setTitle("(8) label 3 pointing DOWNwards 25kg - file: output87.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the plate with label 3 pointing DOWN\n\n"
                                       "3. remove loads\n");
        break;
    case 8071:
        ui->labelInstructions->setText("4. apply 25kg\n");
        break;
    case 8072:
        terminate_section();
        break;

    case 8080:
        fp = fopen("./data/output88.dat","w");
        ui->groupBox->setTitle("(9) label 4 pointing DOWNwards 25kg - file: output88.dat");
        ui->labelInstructions->setText("1. assemble the assembly with a M10 knob\n"
                                       "2. orient the plate with label 4 pointing DOWN\n\n"
                                       "3. remove loads\n");
        break;
    case 8081:
        ui->labelInstructions->setText("4. apply 25kg\n");
        break;
    case 8082:
        terminate_section();
        break;
    case 9999:
        bias_sensor();
        trial=-1;
        break;
    default:
        trial++;
        break;
    }
}

void StrainCalibGui::close_files()
{
    if (fp!=0){
        fclose(fp);
        fp=NULL;
    }
}

void StrainCalibGui::terminate_section()
{
    trial=-1;

    close_files();
    turnOnButtons();
    if(!ui->freeAcqModeGroup->isChecked()){
        buttons.at(currentBtnIndex)->setStyleSheet("background-color: rgb(0, 175, 0);");
    }

}


void StrainCalibGui::turnOffButtons()
{
    if(!ui->freeAcqModeGroup->isChecked()){
        foreach (QPushButton *btn, buttons) {
            btn->setEnabled(false);
        }

        ui->btnAcquireData->setEnabled(true);
        ui->btnAcquireData->setText("Start Acquisition");
        ui->freeAcqModeGroup->setEnabled(false);
    }
    ui->spinSamples->setEnabled(true);

}

void StrainCalibGui::turnOnButtons()
{
    ui->groupBox->setTitle("Instructions");

    if(!ui->freeAcqModeGroup->isChecked()){
        foreach (QPushButton *btn, buttons) {
            btn->setEnabled(true);
        }
        ui->btnAcquireData->setEnabled(false);
        ui->freeAcqModeGroup->setEnabled(true);
        if(false == checkDirectory())
        {
            yError() << "CANNOT find the data folder. Please create it!";
            ui->labelInstructions->setText("You can select an acquisition from the menu ... but:"
                                           "\nCANNOT find the data folder."
                                           "\nIf you want your acquisition data saved, please create it!");
        }
        else
        {
             ui->labelInstructions->setText("Select an acquisition from the menu");
        }
    }else{
        ui->labelInstructions->setText("Free Acquisiton Mode");
    }

    ui->spinSamples->setEnabled(false);

}

void StrainCalibGui::bias_sensor()
{
    #ifdef ATI_SENS
        ATIsens->BiasForceTorqueSensor();
    #endif
}


int StrainCalibGui::remap_trials()
{
    //{8020,1001,8010,8030,8040,8050,8060,8070,8080,3001,4001,5001,6001};

    switch (trial)
    {
        case 1001: return -1;
        case 1002: return 0;
        case 1003: return 1;
        case 1004: return 2;
        case 1005: return 3;

        case 3001: return -1;
        case 3002: return 4;
        case 3003: return 5;
        case 3004: return 6;

        case 4001: return -1;
        case 4002: return 7;
        case 4003: return 8;
        case 4004: return 9;

        case 5001: return -1;
        case 5002: return 10;
        case 5003: return 11;
        case 5004: return 12;

        case 6001: return -1;
        case 6002: return 13;
        case 6003: return 14;
        case 6004: return 15;

        case 8010: return -1;
        case 8011: return 16;

        case 8020: return -1;
        case 8021: return 17;

        case 8030: return -1;
        case 8031: return 18;

        case 8040: return -1;
        case 8041: return 19;

        case 8050: return -1;
        case 8051: return 20;

        case 8060: return -1;
        case 8061: return 21;

        case 8070: return -1;
        case 8071: return 22;

        case 8080: return -1;
        case 8081: return 23;

        case -1:
            //printf("DEBUG: section completed, back to the menu\n");
        return -1;

        default:
            qDebug("DEBUG: **** ERROR, invalid trial (%d) selected! ****\n", trial);
        return -1;
    }
}


void StrainCalibGui::onTimerTimeout()
{
    int  i=0;

    current_trial=remap_trials();
    signed_elem_class exp_vals;
    signed_elem_class err_vals;
    signed_elem_class in_bound;

    if (current_trial==-1){
        for (i=0; i<6;i++){

                readValues.at(i)->setText(QString("%1").arg(last_value.dat[i]));
                expValues.at(i)->setText("---");
                errValues.at(i)->setText("---");
                expValues.at(i)->setStyleSheet("");
                errValues.at(i)->setStyleSheet("");

            }

            if(true == isAcquisitionOfDataActive())
            {
                ui->btnAcquireData->setEnabled(false);
                ui->btnAcquireData->setText("Acquiring Data Now");
                ui->spinSamples->setEnabled(false);
                ui->progress->setValue(progressvalue);
            }
            else
            {
                if(ui->freeAcqModeGroup->isChecked())
                {
                    ui->btnAcquireData->setEnabled(true);
                    ui->btnAcquireData->setText("Start Acquiring Data");
                    ui->spinSamples->setEnabled(true);
                }
                else
                {
                    if(!ui->btnAcquireData->isEnabled())
                    {
                        ui->btnAcquireData->setText("Choose an Acquisition Mode");
                    }
                }

                ui->progress->setValue(0);
            }

    } else {

        if(true == isAcquisitionOfDataActive())
        {
            ui->btnAcquireData->setEnabled(false);
            ui->btnAcquireData->setText("Acquiring Data Now");
            ui->progress->setValue(progressvalue);
        }
        else
        {

            ui->progress->setValue(0);

            tick_acquisition(1);

            expected_values_handler.get_current_expected_values(exp_vals, current_trial);
            for (i=0; i<6;i++){
                expValues.at(i)->setText(QString("%1").arg(exp_vals.dat[i]));
            }

            signed_elem_class last;
            last = last_value;
            last.remove_bias(trial_bias);
            for (i=0; i<6;i++){
                readValues.at(i)->setText(QString("%1").arg(last.dat[i]));
            }
            bool in_boundary = expected_values_handler.check_vals(last,current_trial,err_vals,in_bound);
            for (i=0; i<6; i++){
                errValues.at(i)->setText(QString("%1").arg(err_vals.dat[i]));

                if (in_bound.dat[i]==0){
                    errValues.at(i)->setStyleSheet("color: rgb(255, 0, 0);");
                } else if (in_bound.dat[i]==1) {
                    errValues.at(i)->setStyleSheet("color: rgb(0, 255, 0);");
                } else if (in_bound.dat[i]==2) {
                    errValues.at(i)->setStyleSheet("color: orange;");
                }
            }


            ui->btnAcquireData->setEnabled(true);
            ui->spinSamples->setEnabled(false);

            if (in_boundary==false){
             ui->btnAcquireData->setText("Strange values detected.\n Acquire anyway? ");
            } else {
             ui->btnAcquireData->setText("Regular Values: Acquire Data");
            }
        }
    }
}


bool StrainCalibGui::tick_acquisition(int samples)
{
    // marco.accame: now we acquire with the strainInterface class in one shot

    if(false == enableacquisitions)
    {
        return false;
    }

    lockdriver(true);

    vector<cDownloader::strain_value_t> values;
    get(samples, values, enabledebugprints, false);
    // yDebug() << "simple acquisition of " << values.size() << "strain samples";

    // now i need to retrieve the most recent value from values but in a particular format and fill variable last_value

    if(values.size() > 0)
    {
        cDownloader::strain_value_t lastvalue = values.at(values.size()-1);
        lastvalue.extract(last_value.dat);
    }

    lockdriver(false);

    return true;

}

void StrainCalibGui::setAcquisitionOfDataActive(const bool active)
{
    mutexacquisitionofdata.lock();
    acquiringdata = active;
    mutexacquisitionofdata.unlock();
}

bool StrainCalibGui::isAcquisitionOfDataActive()
{
    bool active = false;
    mutexacquisitionofdata.lock();
    active = acquiringdata;
    mutexacquisitionofdata.unlock();
    return active;
}

void StrainCalibGui::lockdriver(const bool on)
{
    if(on)
    {
        mutexdriver.lock();
    }
    else
    {
        mutexdriver.unlock();
    }
}

bool StrainCalibGui::checkDirectory()
{
    QFileInfo dir =  QFileInfo("./data");
    return dir.isDir();
}

