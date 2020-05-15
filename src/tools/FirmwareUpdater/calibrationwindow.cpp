#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"
#include <qdebug.h>
#include <QLabel>
#include <QHBoxLayout>
#include <QtConcurrent/QtConcurrent>
#include <QFileDialog>
#include <QMessageBox>
#include <strain.h>

#define CHANNEL_COUNT   6
#define HEX_VALC 0x8000
#define RANGE32K 32768

#define     COL_CURRMEASURE     0

#define     COL_GAIN            0
#define     COL_OFFSET          1
#define     COL_TARGET          2
#define     COL_REGISTERS       3

#define     COL_CALIBBIAS       0

#define     COL_FULLSCALE       0

#define     COL_MAXMEASURE      0
#define     COL_MINMEASURE      1
#define     COL_DIFFMEASURE     2
#define     COL_NEWTONMEASURE   3

#define     MATRIX_COUNT        1


using namespace strain::dsp;

int convert_to_signed32k(unsigned int v)
{
    return static_cast<int>(v)-32768; // it is the 0x8000 used in q15 transformation
}

int showasQ15(int v)
{
    return static_cast<std::int16_t>(v&0xffff);
}

// bias, tare, offset. in here we mean what we add to the six values before or after matyrix multiplication.
int showBias(unsigned int v)
{
    // marco.accame:
    // the gui has always shown the value at it is.
    // it should however show the value w/ showasQ15()
    return v;
}



unsigned int q15_from(int v)
{
    return v+0x8000;
}

const strain2_ampl_discretegain_t CalibrationWindow::defaultStrain2AmplGains[6] =
{
    ampl_gain08, ampl_gain24, ampl_gain24, ampl_gain10, ampl_gain10, ampl_gain24
};

const uint16_t CalibrationWindow::defaultStrain2AmplOffsets[6] =
{
    32767, 32767, 32767, 32767, 32767, 32767 
};

const uint16_t CalibrationWindow::defaultStrain1DACoffsets[6] =
{
    511, 511, 511, 511, 511, 511 
}; 


CalibrationWindow::CalibrationWindow(FirmwareUpdaterCore *core, icubCanProto_boardType_t b, CustomTreeWidgetItem *item, QWidget *parent) :
    QMainWindow(parent),mutex(QMutex::Recursive),
    ui(new Ui::CalibrationWindow)
{
    ui->setupUi(this);
    setWindowModality(Qt::ApplicationModal);

    this->core = core;
    this->selected = item->getIndexOfBoard();
    this->item = item;
    bus = ((CustomTreeWidgetItem*)item->getParentNode())->getCanBoard(selected).bus;
    id  = ((CustomTreeWidgetItem*)item->getParentNode())->getCanBoard(selected).pid;
    calibration_value = 32767;
#if defined(MARCO_ACCAME_19SEP2018)
    calib_const[0] = 1;
    calib_const[1] = 1;
    calib_const[2] = 1;
#else
    calib_const[0] = 0;
    calib_const[1] = 0;
    calib_const[2] = 0;
#endif
    serial_number_changed = false;
    matrix_changed[0] = false;
    matrix_changed[1] = false;
    matrix_changed[2] = false;
    fullScaleChanged = false;

    boardtype = b; // we can have either a icubCanProto_boardType__strain or a icubCanProto_boardType__strain2 ...
    eeprom_saved_status=true;
    first_time = true;
    refresh_serialnumber = true;
    clearStats = false;
    for(int i=0;i<CHANNEL_COUNT;i++){
        ch[i]               = i;
        adc[i]              = 0;
        calib_bias[i]       = 0;
        curr_bias[i]        = 0;
        // marco.accame: we want to show the maximum / minimum values in the same way the adc values are displayed: in range [-32k, +32k)
        maxadc[i]           = -32768;
        minadc[i]           = +32767;

        maxft[i]           = -32768;
        minft[i]           = +32767;

        offsetSliderPressed[i]    = false;
        amp_gain1[i]        = 0;
        amp_gain2[i]        = 0;
    }
    strncpy(serial_no,"UNDEF",8);

    for(int m=0;m<MATRIX_COUNT;m++){
        for(int i=0;i<CHANNEL_COUNT;i++){
            full_scale_const[m][i] = 0;
        }
    }

    if(icubCanProto_boardType__strain == boardtype){
        ui->tableParamters->hideColumn(COL_GAIN);
    }



    for (int i=0;i <CHANNEL_COUNT;i++) {
        QWidget *container = new QWidget(ui->tableParamters);
        /*****************************************************/
        CustomSpinBox *spinner = new CustomSpinBox(boardtype,container);
        slider_offset.append(spinner);

        ui->tableParamters->setCellWidget(i,COL_OFFSET,spinner);
        ui->tableParamters->cellWidget(i,COL_OFFSET)->setEnabled(false);

        CustomComboBox *gainCombo = new CustomComboBox(container);

        ui->tableParamters->setCellWidget(i,COL_GAIN,gainCombo);
        ui->tableParamters->cellWidget(i,COL_GAIN)->setEnabled(false);

        /*****************************************************/

        QSpinBox *spinnerTarget = new QSpinBox(container);
        spinnerTarget->setMinimum(0);
        spinnerTarget->setMaximum(32767);
        spinnerTarget->setMinimum(-32768);
        spinnerTarget->setValue(0);
        ui->tableParamters->setCellWidget(i,COL_TARGET,spinnerTarget);

        if(icubCanProto_boardType__strain == boardtype && i > 0){
            spinnerTarget->setEnabled(false);
        } else if(icubCanProto_boardType__strain == boardtype && i == 0){
            connect(spinnerTarget,SIGNAL(valueChanged(int)),this,SLOT(onTargetValueChanged(int)));
        }



        /*****************************************************/

        QTableWidgetItem *item = new QTableWidgetItem("-");
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        ui->tableCurr->setItem(i,COL_CURRMEASURE,item);

        ui->tableCurr->setHorizontalHeaderItem(0, new QTableWidgetItem("ADC"));


        QTableWidgetItem *item1 = new QTableWidgetItem("-");
        item1->setFlags(item1->flags() ^ Qt::ItemIsEditable);
        ui->tableUseMatrix->setItem(i,COL_MAXMEASURE,item1);


        QTableWidgetItem *item2 = new QTableWidgetItem("-");
        item2->setFlags(item2->flags() ^ Qt::ItemIsEditable);
        ui->tableUseMatrix->setItem(i,COL_MINMEASURE,item2);


        QTableWidgetItem *item3 = new QTableWidgetItem("-");
        item3->setFlags(item3->flags() ^ Qt::ItemIsEditable);
        ui->tableUseMatrix->setItem(i,COL_DIFFMEASURE,item3);


        QTableWidgetItem *item4 = new QTableWidgetItem("-");
        item4->setFlags(item4->flags() ^ Qt::ItemIsEditable);
        ui->tableUseMatrix->setItem(i,COL_NEWTONMEASURE,item4);

        for(int j=0;j<CHANNEL_COUNT;j++){
            QTableWidgetItem *item = new QTableWidgetItem("-");
            item->setFlags(item->flags() ^ Qt::ItemIsEditable);
            ui->matrixA->setItem(i,j,item);


//            QTableWidgetItem *item1 = new QTableWidgetItem("-");
//            ui->matrixB->setItem(i,j,item1);

//            QTableWidgetItem *item2 = new QTableWidgetItem("-");
//            ui->matrixC->setItem(i,j,item2);
        }


        QTableWidgetItem *item001 = new QTableWidgetItem("-");
        item001->setFlags(item001->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_GAIN,item001);


        QTableWidgetItem *item5 = new QTableWidgetItem("-");
        item5->setFlags(item5->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_TARGET,item5);


//        QTableWidgetItem *item6 = new QTableWidgetItem("-");
//        item6->setFlags(item6->flags() ^ Qt::ItemIsEditable);
//        ui->tableParamters->setItem(i,COL_CURRBIAS,item6);

        QTableWidgetItem *item6 = new QTableWidgetItem("-");
        item6->setFlags(item6->flags() ^ Qt::ItemIsEditable);
        ui->tableBias->setItem(i,COL_CALIBBIAS,item6);

        QTableWidgetItem *item7 = new QTableWidgetItem("-");
        item7->setFlags(item7->flags() ^ Qt::ItemIsEditable);
        ui->tableFullScaleA->setItem(i,COL_FULLSCALE,item7);

//        QTableWidgetItem *item8 = new QTableWidgetItem("-");
//        item8->setFlags(item8->flags() ^ Qt::ItemIsEditable);
//        ui->tableFullScaleB->setItem(i,COL_FULLSCALE,item8);

//        QTableWidgetItem *item9 = new QTableWidgetItem("-");
//        item9->setFlags(item9->flags() ^ Qt::ItemIsEditable);
//        ui->tableFullScaleC->setItem(i,COL_FULLSCALE,item9);

        fullScales.append(ui->tableFullScaleA);
//        fullScales.append(ui->tableFullScaleB);
//        fullScales.append(ui->tableFullScaleC);

    }

    ui->tableParamters->setColumnWidth(COL_OFFSET,150);
    //ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);



    progress = new QProgressBar(this);
    progress->setFixedWidth(100);
    progress->setMinimum(0);
    progress->setMaximum(100);
    progress->setVisible(false);
    ui->statusbar->addWidget(progress);

//    matrixGain.append(ui->editMatrixGainA);
//    matrixGain.append(ui->editMatrixGainB);
//    matrixGain.append(ui->editMatrixGainC);

    matrices.append(ui->matrixA);
//    matrices.append(ui->matrixB);
//    matrices.append(ui->matrixC);

    if(boardtype == icubCanProto_boardType__strain){
        ui->comboRegSet->setEnabled(false);
        ui->comboRegSetBoot->setEnabled(false);
    }


    //connect(ui->comboUseMatrix,SIGNAL(currentIndexChanged(int)),this,SLOT(onUseMatrixChanged(int)),Qt::QueuedConnection);
    connect(ui->comboRegSet,SIGNAL(currentIndexChanged(int)),this,SLOT(onChangeRegSet(int)));
    connect(ui->comboRegSetBoot,SIGNAL(currentIndexChanged(int)),this,SLOT(onChangeRegSetBoot(int)));
    connect(this,SIGNAL(applyDone()),this,SLOT(onApplyDone()));
    connect(ui->btnClose,SIGNAL(clicked(bool)),this,SLOT(close()));
    connect(ui->edit_serial,SIGNAL(textEdited(QString)),this,SLOT(onSerialChanged(QString)));
    connect(this,SIGNAL(loading(bool)),this,SLOT(onLoading(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setText(QLineEdit*,QString)),this,SLOT(onSetText(QLineEdit*,QString)),Qt::QueuedConnection);
    connect(this,SIGNAL(setText(QTableWidgetItem*,QString)),this,SLOT(onSetText(QTableWidgetItem*,QString)),Qt::QueuedConnection);
    connect(this,SIGNAL(setTableVisible(QTableWidget*,bool)),this,SLOT(onSetTableVisible(QTableWidget*,bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setOffsetSliderValue(CustomSpinBox*,int)),this,SLOT(onOffsetSpinnerValue(CustomSpinBox*,int)),Qt::BlockingQueuedConnection);

    connect(ui->btnParametersClear,SIGNAL(clicked(bool)),this,SLOT(onParametersClear(bool)));
    connect(ui->btnParametersApply,SIGNAL(clicked(bool)),this,SLOT(onParametersApply(bool)));
    connect(ui->checkAutoTune,SIGNAL(toggled(bool)),this,SLOT(onCheckAutoTune(bool)));
    connect(ui->btnSetCalibBias,SIGNAL(clicked(bool)),this,SLOT(onSetCalibBias(bool)));
    connect(ui->btnResetCalibBias,SIGNAL(clicked(bool)),this,SLOT(onResetCalibBias(bool)));

    connect(ui->btnSetSerial,SIGNAL(clicked(bool)),this,SLOT(onSetSerial(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setSerialChanged(bool)),this,SLOT(onSetSerialChanged(bool)));
    //connect(ui->btnResetCalib,SIGNAL(clicked(bool)),this,SLOT(onResetCalibMatrix(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setFullScale()),this,SLOT(onSetFullScale()),Qt::QueuedConnection);
    connect(this,SIGNAL(setMatrix(int)),this,SLOT(onSetMatrix(int)),Qt::QueuedConnection);
    connect(ui->actionSave_To_Eproom,SIGNAL(triggered(bool)),this,SLOT(onSaveToEeprom(bool)),Qt::QueuedConnection);
    connect(ui->actionClear_Statistics,SIGNAL(triggered(bool)),this,SLOT(onClear_Statistics(bool)),Qt::QueuedConnection);
    connect(ui->actionClear_the_full_regulation,SIGNAL(triggered(bool)),this,SLOT(onClear_FullRegulation(bool)),Qt::QueuedConnection);
    connect(ui->actionClear_the_regulation_set_in_use,SIGNAL(triggered(bool)),this,SLOT(onClear_Regulation(bool)),Qt::QueuedConnection);
    connect(ui->btnSetCalibration,SIGNAL(clicked(bool)),this,SLOT(onSetCalibration(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(resetMatrices(int)),this,SLOT(resetMatricesState(int)),Qt::QueuedConnection);
    connect(this,SIGNAL(updateTitle()),this,SLOT(onUpdateTitle()),Qt::QueuedConnection);
    connect(this,SIGNAL(appendLogMsg(QString)),this,SLOT(onAppendLogMsg(QString)),Qt::QueuedConnection);
    connect(ui->btnClearLog,SIGNAL(clicked(bool)),this,SLOT(onClearLog()));
    connect(ui->btnLoadCalibFile,SIGNAL(clicked(bool)),this,SLOT(onLoadCalibrationFile(bool)));
    connect(ui->btnSaveCalibFile,SIGNAL(clicked(bool)),this,SLOT(onSaveCalibrationFile(bool)));
    connect(ui->btnImportCalibMatrix,SIGNAL(clicked(bool)),this,SLOT(onImportCalibrationFile(bool)));
    //connect(ui->checkDigital,SIGNAL(toggled(bool)),this,SLOT(onDigitalRegulation(bool)));


    ui->actionImport_Calib_Matrix->setEnabled(false);
    ui->actionLoad_Calibration_File->setEnabled(false);
    ui->actionSave_Calibration_File->setEnabled(false);
    //    timer = new QTimer();
    //    timer->setInterval(500);
    //    timer->setSingleShot(false);
    //    connect(timer,SIGNAL(timeout()),this,SLOT(onTimeout()),Qt::DirectConnection);
    //    timer->start();

    QtConcurrent::run(this,&CalibrationWindow::onTimeout);

}

CalibrationWindow::~CalibrationWindow()
{
    //    timer->stop();
    //    delete timer;

    mutex.lock();
    keepRunning = false;
    delete ui;
    mutex.unlock();
}

void CalibrationWindow::onTargetValueChanged(int v)
{


    if(boardtype == icubCanProto_boardType__strain){
        for(int i=1;i<CHANNEL_COUNT;i++){
            QSpinBox *spinnerTarget = (QSpinBox*)ui->tableParamters->cellWidget(i,COL_TARGET);
            spinnerTarget->setValue(v);
        }

    }
}

void CalibrationWindow::onChangeRegSet(int regSet)
{
    QtConcurrent::run(this,&CalibrationWindow::useMatrix,regSet,false);
}

void CalibrationWindow::onChangeRegSetBoot(int regSet)
{
    QtConcurrent::run(this,&CalibrationWindow::useMatrix,regSet,true);
}


void CalibrationWindow::onDigitalRegulation(bool checked)
{
    ui->digitalContainer->setEnabled(checked);
    if(checked){
        ui->tableUseMatrix->horizontalHeaderItem(COL_NEWTONMEASURE)->setText("FT");
//        ui->tableUseMatrix->showColumn(COL_NEWTONMEASURE);
//        ui->tableUseMatrix->setColumnWidth(0,60);
//        ui->tableUseMatrix->setColumnWidth(1,60);
//        ui->tableUseMatrix->setColumnWidth(2,60);
    }else{
        //ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);
        ui->tableUseMatrix->horizontalHeaderItem(COL_NEWTONMEASURE)->setText("Empty");

    }

}
void CalibrationWindow::onCheckAutoTune(bool checked)
{
    if(checked){
        for(int i=0; i<CHANNEL_COUNT;i++){
            if(icubCanProto_boardType__strain == boardtype){
                if(i == 0){
                    ui->tableParamters->cellWidget(i,COL_TARGET)->setEnabled(true);
                }
            } else {
                ui->tableParamters->cellWidget(i,COL_TARGET)->setEnabled(true);
            }
            ui->tableParamters->cellWidget(i,COL_OFFSET)->setEnabled(false);
            ui->tableParamters->cellWidget(i,COL_GAIN)->setEnabled(false);
        }
    } else {
        for(int i=0; i<CHANNEL_COUNT;i++){
            ui->tableParamters->cellWidget(i,COL_TARGET)->setEnabled(false);
            ui->tableParamters->cellWidget(i,COL_OFFSET)->setEnabled(true);
            ui->tableParamters->cellWidget(i,COL_GAIN)->setEnabled(true);
        }
    }
}

void CalibrationWindow::onParametersClear(bool click)
{

    for(int i=0; i<CHANNEL_COUNT;i++){
        CustomComboBox *combo = ((CustomComboBox*)ui->tableParamters->cellWidget(i,COL_GAIN));

        if(icubCanProto_boardType__strain2 == boardtype)
        {
            combo->setIndexFromAmpGain(defaultStrain2AmplGains[i]);
            ((CustomSpinBox*)ui->tableParamters->cellWidget(i,COL_OFFSET))->setValue(defaultStrain2AmplOffsets[i]);
        }
        else
        {
            ((CustomSpinBox*)ui->tableParamters->cellWidget(i,COL_OFFSET))->setValue(defaultStrain1DACoffsets[i]);
        }

    }
}

void CalibrationWindow::onParametersApply(bool click)
{
    if(ui->checkAutoTune->isChecked()){
        QtConcurrent::run(this,&CalibrationWindow::autoAdjust);
    } else {
        QtConcurrent::run(this,&CalibrationWindow::applyParameters);
    }
}

void CalibrationWindow::onOffsetEditingFinished()
{
    qDebug() << "EDIT FINISH FOR " << sender();
}

void CalibrationWindow::onTabMatrixCahnged(int index)
{
    // This will be removed in the future
    switch (index) {
    case 0:
        ui->btnLoadCalibFile->setEnabled(true);
        ui->btnSaveCalibFile->setEnabled(true);
        ui->btnImportCalibMatrix->setEnabled(true);
        //ui->btnResetCalib->setEnabled(true);
        ui->btnSetCalibration->setEnabled(true);
        break;
    default:
        ui->btnLoadCalibFile->setEnabled(false);
        ui->btnSaveCalibFile->setEnabled(false);
        ui->btnImportCalibMatrix->setEnabled(false);
        //ui->btnResetCalib->setEnabled(false);
        ui->btnSetCalibration->setEnabled(false);
        break;
    }

}

void CalibrationWindow::onOffsetSliderPressed()
{
    int index = slider_offset.indexOf((CustomSpinBox*)sender());
    offsetSliderPressed[index] = true;
}

void CalibrationWindow::onOffsetSliderReleased()
{
    int index = slider_offset.indexOf((CustomSpinBox*)sender());

    QtConcurrent::run(this,&CalibrationWindow::setOffset,index,((CustomSpinBox*)sender())->value());

    offsetSliderPressed[index] = false;

}

void CalibrationWindow::setCalibBias()
{
    mutex.lock();
    loading();
    string msg;
    core->getDownloader()->strain_set_calib_bias(bus,id,&msg);
    appendLogMsg(msg.c_str());
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::resetCalibBias()
{
    mutex.lock();
    loading();
    string msg;
    core->getDownloader()->strain_reset_calib_bias(bus,id,&msg);
    appendLogMsg(msg.c_str());
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::resetCurrBias()
{
    mutex.lock();
    loading();
    string msg;
    core->getDownloader()->strain_reset_curr_bias(bus,id,&msg);
    appendLogMsg(msg.c_str());
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::setCurrBias()
{
    mutex.lock();
    loading();
    string msg;
    core->getDownloader()->strain_set_curr_bias(bus,id,&msg);
    appendLogMsg(msg.c_str());
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::autoAdjust()
{
    mutex.lock();
    loading();
    string msg;


    std::vector<strain2_ampl_discretegain_t> gains(0);
    std::vector<int16_t> targets(0);
    if(icubCanProto_boardType__strain == boardtype)
    {
        CustomSpinBox *spin = (CustomSpinBox*)ui->tableParamters->cellWidget(0,COL_TARGET);
        std::int16_t target = spin->value();
        gains.resize(0);
        targets.push_back(target);
    }
    else
    {
        for(int i=0;i<CHANNEL_COUNT;i++)
        {
            QComboBox *combo = (QComboBox*)ui->tableParamters->cellWidget(i,COL_GAIN);
            CustomSpinBox *spin = (CustomSpinBox*)ui->tableParamters->cellWidget(i,COL_TARGET);
            gains.push_back(static_cast<strain2_ampl_discretegain_t>(combo->itemData(combo->currentIndex(),GAINAMPROLE).toInt()));
            targets.push_back(spin->value());
        }

    }
    core->getDownloader()->set_external_logger(this, logger);
    core->getDownloader()->strain_calibrate_offset2(bus, id, boardtype, gains, targets, &msg);
    core->getDownloader()->set_external_logger(NULL, NULL);

    loading(false);
    applyDone();
    mutex.unlock();
}

void CalibrationWindow::applyParameters()
{
    mutex.lock();
    loading();
    string msg;

    for(int i=0;i<CHANNEL_COUNT;i++)
    {
        QComboBox *combo = (QComboBox*)ui->tableParamters->cellWidget(i,COL_GAIN);
        CustomSpinBox *spin = (CustomSpinBox*)ui->tableParamters->cellWidget(i,COL_OFFSET);

        if(icubCanProto_boardType__strain2 == boardtype)
        {
            int lastOffset = spin->value();
            uint16_t offset = lastOffset; // 32*1024 - 1;
            int g = combo->itemData(combo->currentIndex(),GAINAMPROLE).toInt();
            strain2_ampl_discretegain_t dg = static_cast<strain2_ampl_discretegain_t>(g);
            float gain = core->getDownloader()->strain_amplifier_discretegain2float(dg);
            if(0 != gain){
                core->getDownloader()->strain_set_amplifier_gain_offset(bus, id, i, gain, offset, cDownloader::strain_regset_inuse, &msg);
            }
        }
        else
        {
            int lastOffset = spin->value();
            uint16_t offset = lastOffset;
            core->getDownloader()->strain_set_offset(bus,id, i, offset, cDownloader::strain_regset_inuse, &msg);
        }
    }
    //connect(this,SIGNAL(setOffsetSliderValue(CustomSpinBox*,int)),this,SLOT(onOffsetSliderValue(CustomSpinBox*,int)),Qt::BlockingQueuedConnection);
    loading(false);
    applyDone();
    mutex.unlock();
}


void CalibrationWindow::setSerial()
{
    mutex.lock();
    loading();
    string msg;


    core->getDownloader()->strain_set_serial_number(bus,id,ui->edit_serial->text().toLatin1().data(),&msg);
    appendLogMsg(msg.c_str());
    serial_number_changed = false;
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::saveToEeprom()
{
    mutex.lock();
    loading();
    drv_sleep (1000);
    string msg;
    core->getDownloader()->strain_save_to_eeprom(bus,id,&msg);
    appendLogMsg(msg.c_str());
    drv_sleep (1000);
    //resetMatrices();
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::Clear_Statistics()
{
    mutex.lock();
    clearStats = true;
    mutex.unlock();
}

void CalibrationWindow::Clear_AllRegulations()
{
    mutex.lock();

    const bool alsoserialnumber = true;
    if((icubCanProto_boardType__strain2 == boardtype))
    {
        SetDefaultRegulationSet(cDownloader::strain_regset_one, alsoserialnumber);
        SetDefaultRegulationSet(cDownloader::strain_regset_two);
        SetDefaultRegulationSet(cDownloader::strain_regset_three);
        useMatrix(0, true);
        useMatrix(0, false);
    }
    else
    {
        SetDefaultRegulationSet(cDownloader::strain_regset_inuse, alsoserialnumber);
    }

    refresh_serialnumber = alsoserialnumber;

    mutex.unlock();
}

void CalibrationWindow::Clear_Regulation()
{
    mutex.lock();

    const bool alsoserialnumber = false;
    SetDefaultRegulationSet(cDownloader::strain_regset_inuse, alsoserialnumber);
    refresh_serialnumber = alsoserialnumber;

    mutex.unlock();
}


void CalibrationWindow::resetMatricesState(int index)
{
    
    for(int i=0;i<MATRIX_COUNT;i++){
        matrix_changed[i] = false;
        if(index >=0){
            if(i != index){
                continue;
            }
        }
        QTableWidget *table = matrices.at(i);
        //QLineEdit *gain = matrixGain.at(i);

        //disconnect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)));
        for(int r=0;r<CHANNEL_COUNT;r++){
            for(int c=0;c<CHANNEL_COUNT;c++){
                table->item(r,c)->setTextColor("");
            }
        }
        //connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)),Qt::UniqueConnection);
        //gain->setStyleSheet("");
    }
}

void CalibrationWindow::setCalibration()
{
    mutex.lock();
    loading();

    int index = 0;
    //QTableWidget *table = matrices.at(index);

    // marco.accame: the regulation set to be sent to the board is the one in use in its inside


    if(matrix_changed[0]){
        for (int ri=0; ri<CHANNEL_COUNT; ri++){
            for (int ci=0; ci<CHANNEL_COUNT; ci++){
                string msg;
                core->getDownloader()->strain_set_matrix_rc(bus,id,ri,ci,calib_matrix[index][ri][ci], cDownloader::strain_regset_inuse, &msg);
                appendLogMsg(msg.c_str());
            }
        }
    }


    if(fullScaleChanged){
        for (int i=0;i<CHANNEL_COUNT; i++){
            string msg;
            core->getDownloader()->strain_set_full_scale(bus,id,i,full_scale_calib[index][i], cDownloader::strain_regset_inuse, &msg);
            appendLogMsg(msg.c_str());
        }
    }

//    string msg;
//    core->getDownloader()->strain_set_matrix_gain(bus,id,calib_const[index], regset, &msg);
//    appendLogMsg(msg.c_str());



    int ri=0;
    int ci=0;

    drv_sleep (1000);
    for (ri=0;ri<CHANNEL_COUNT;ri++){
        for (ci=0;ci<CHANNEL_COUNT;ci++){
            string msg;
            core->getDownloader()->strain_get_matrix_rc(bus,id,ri,ci,matrix[index][ri][ci], cDownloader::strain_regset_inuse, &msg);
            appendLogMsg(msg.c_str());
        }
    }
    setMatrix(index);
    setFullScale();
    //resetMatrices(index);

//    int count_ok=0;
//    for (int i=0;i<36; i++){
//        ri=i/6;
//        ci=i%6;
//        if (calib_matrix[index][ri][ci]==matrix[index][ri][ci]) {
//            count_ok++;
//        } else {
//            printf ("Found 1 error on element %d,%d !!\n",ri, ci);
//            appendLogMsg("Found 1 error on element %d,%d !!");
//        }
//    }

//    if (count_ok==36){
//        printf ("Calibration file applied with no errors\n");
//        appendLogMsg(QString("Calibration file applied with no errors"));
//        matrix_changed[0]=false;
//    } else {
//        printf ("Found %d errors applying the calibration file!!\n",36-count_ok);
//        appendLogMsg(QString("Found %1 errors applying the calibration file!!").arg(36-count_ok));
//    }

    printf ("Calibration file applied with no errors\n");
    appendLogMsg(QString("Calibration file applied with no errors"));
    matrix_changed[0]=false;
    fullScaleChanged = false;
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::loadCalibrationFile(QString fileName)
{
    // marco.accame: the regulation set to be sent to the board is the one in use in its inside

    mutex.lock();
    int index = 0;//ui->tabWidget->currentIndex();
    loading();
    //load data file
    if(!calibration_load_v3 (fileName.toLatin1().data(), bus, id, index, cDownloader::strain_regset_inuse)){
        loading(false);
        mutex.unlock();
    }

    //update windows graphics
    int i=0;
    int ri=0;
    int ci=0;
    string msg;
    char buffer[256];

    drv_sleep (500);
    core->getDownloader()->strain_get_serial_number(bus,id, buffer, &msg);

    setText(ui->edit_serial,QString(buffer));
    serial_number_changed=false;

    drv_sleep (500);
    for (ri=0;ri<CHANNEL_COUNT;ri++){
        for (ci=0;ci<CHANNEL_COUNT;ci++){
            core->getDownloader()->strain_get_matrix_rc(bus,id, ri, ci, matrix[index][ri][ci], cDownloader::strain_regset_inuse, &msg);
            appendLogMsg(msg.c_str());
            sprintf(buffer,"%x",matrix[index - 1][ri][ci]);
            setMatrix(index);
            //resetMatrices(index);
        }
    }


    drv_sleep (500);

    int count_ok=0;
    for (i=0;i<36; i++){
        ri=i/6;
        ci=i%6;
        if (calib_matrix[index][ri][ci]==matrix[index][ri][ci]){
            count_ok++;
        } else {
            QString s = QString("Found 1 error on element %1,%2 !!").arg(ri).arg(ci);
            appendLogMsg(s);
        }
    }

    if (count_ok==36){
        QString s = QString("Calibration file applied with no errors");
        appendLogMsg(s);
        matrix_changed[index]=false;
    } else {
        QString s = QString("Found %1 errors applying the calibration file!!").arg(36-count_ok);
        appendLogMsg(s);
    }




    fullScaleChanged = false;
    loading(false);
    mutex.unlock();

}

void CalibrationWindow::onLoadCalibrationFile(bool click)
{
    QString filename = QFileDialog::getOpenFileName(this,"Choose File",QDir::home().absolutePath(),"dat(*.dat)");

    if(filename.isEmpty()){
        return;
    }

    QtConcurrent::run(this,&CalibrationWindow::loadCalibrationFile,filename);
}

void CalibrationWindow::saveCalibrationFile(QString filePath)
{
    mutex.lock();
    loading();
    int index = 0;

    char path[256] = { 0 };
    snprintf(path, sizeof(path), "%s", filePath.toLatin1().data());
    std::string filename = std::string(path);

    filename += "/calibrationData";
    filename += serial_no;
    filename += ".dat";
    fstream filestr;
    filestr.open (filename.c_str(), fstream::out);
    int i=0;
    char buffer[256];


    if(icubCanProto_boardType__strain2 == boardtype)
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



    printf ("Calibration file saved!\n");
    appendLogMsg("Calibration file saved!");
    filestr.close();
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::onSaveCalibrationFile(bool click)
{
    QString filePath = QFileDialog::getExistingDirectory(this,"Choose File",QDir::home().absolutePath());

    if(filePath.isEmpty()){
        return;
    }

    QtConcurrent::run(this,&CalibrationWindow::saveCalibrationFile,filePath);
}

void CalibrationWindow::importCalibrationFile(QString fileName)
{
    // marco.accame: the regulation set to be sent to the board is the one in use in its inside

    mutex.lock();
    loading();
    int index = 0;//ui->tabWidget->currentIndex();
    char* buff = fileName.toLatin1().data();

    if (buff==NULL){
        yError("File not found!\n");
        appendLogMsg("File not found!");
        loading(false);
        mutex.unlock();
        return;
    }

    //fstream filestr;
    QFile filestr(fileName);

    if (!filestr.open (QIODevice::ReadOnly)){
        yError("Error opening calibration file!\n");
        appendLogMsg("Error opening calibration file!");
        loading(false);
        mutex.unlock();
        return;
    }

    int i=0;

    QByteArray line = filestr.readLine();
#if defined(MARCO_ACCAME_19SEP2018)    
    if(line.at(0) != '#')
    {
        // it must be a hex file: close it and call importCalibrationFileHEX()
        filestr.close();
        loading(false);
        mutex.unlock();
        importCalibrationFileHEX(fileName);
        return;
    }
#endif
    while(!line.isEmpty() && line.at(0) == '#' && !filestr.atEnd()){
        line = filestr.readLine();
        continue;
    }

    if(filestr.atEnd()){
        filestr.close();

        printf ("No calib file loaded!\n");
        appendLogMsg("No calib file loaded!");
        loading(false);
        mutex.unlock();
        return;

    }

    QString s = QString(line);

    //for (i=0;i<36; i++){
    for (i=0;i<CHANNEL_COUNT; i++){

        QTextStream t(&s);
        float val[6];//1,val2,val3,val4,val5,val6;
        t >> val[0] >> val[1] >> val[2] >> val[3] >> val[4] >> val[5];

        bool saturated;
        //sscanf (line.data(),"%f",&val);
        for (int j=0;j<CHANNEL_COUNT; j++){
            Q15 value = q15::convert(val[j],saturated);
            calib_matrix[index][i][j] = value;
            matrix[index][i][j] = value;
        }

        line = filestr.readLine();
        s = QString(line);

    }


    matrix_changed[index]=true;
    setMatrix(index);

    while(!line.isEmpty() && line.at(0) == '#' && !filestr.atEnd()){
        line = filestr.readLine();
        continue;
    }

    s = QString(line);

    if(filestr.atEnd()){
        filestr.close();

        something_changed=true;
        matrix_changed[index]=true;
        setMatrix(index);
        printf ("Calibration Matrix loaded! - No Fullscale found\n");
        appendLogMsg("Calibration Matrix loaded! - No Fullscale found!");
        loading(false);
        mutex.unlock();
        return;

    }


    QTextStream t(&s);

    float val[6];//1,val2,val3,val4,val5,val6;
    t >> val[0] >> val[1] >> val[2] >> val[3] >> val[4] >> val[5];

    bool saturated;

    for (i=0;i<CHANNEL_COUNT; i++){
        FSC value = fsc::convert(val[i],saturated);
        full_scale_calib[index][i] = value;
//        core->getDownloader()->strain_set_full_scale(bus,id,i,value, regset, &msg);
//        appendLogMsg(msg.c_str());

    }

    fullScaleChanged = true;
    filestr.close();

    something_changed=true;

    setFullScale();
    printf ("Calibration file loaded!\n");
    appendLogMsg("Calibration file loaded!");


    /***********************************/

//    int ri=0;
//    int ci=0;

//    drv_sleep (1000);
//    for (ri=0;ri<CHANNEL_COUNT;ri++){
//        for (ci=0;ci<CHANNEL_COUNT;ci++){
//            core->getDownloader()->strain_get_matrix_rc(bus,id,ri,ci,matrix[index][ri][ci], regset, &msg);
//            appendLogMsg(msg.c_str());
//        }
//    }
//    setMatrix(index);
//    resetMatrices(index);

//    int count_ok=0;
//    for (i=0;i<36; i++){
//        ri=i/6;
//        ci=i%6;
//        if (calib_matrix[index][ri][ci]==matrix[index][ri][ci]) {
//            count_ok++;
//        } else {
//            printf ("Found 1 error on element %d,%d !!\n",ri, ci);
//            appendLogMsg("Found 1 error on element %d,%d !!");
//        }
//    }

//    if (count_ok==36){
//        printf ("Calibration file %s applied with no errors\n", buff);
//        appendLogMsg(QString("Calibration file %1 applied with no errors").arg(buff));
//        matrix_changed[0]=false;
//    } else {
//        printf ("Found %d errors applying the calibration file!!\n",36-count_ok);
//        appendLogMsg(QString("Found %1 errors applying the calibration file!!").arg(36-count_ok));
//    }






    loading(false);
    mutex.unlock();
}

void CalibrationWindow::onImportCalibrationFile(bool click)
{
    QString fileName = QFileDialog::getOpenFileName(this,"Choose File",QDir::home().absolutePath());

    if(fileName.isEmpty()){
        return;
    }

    QtConcurrent::run(this,&CalibrationWindow::importCalibrationFile,fileName);
}


void CalibrationWindow::onSetCalibration(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::setCalibration);
}

void CalibrationWindow::onSaveToEeprom(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::saveToEeprom);
}

void CalibrationWindow::onClear_Statistics(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::Clear_Statistics);
}

void CalibrationWindow::onClear_FullRegulation(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::Clear_AllRegulations);
}

void CalibrationWindow::onClear_Regulation(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::Clear_Regulation);
}

void CalibrationWindow::onSetCalibBias(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::setCalibBias);
}

void CalibrationWindow::onResetCalibBias(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::resetCalibBias);
}

void CalibrationWindow::onSetCurrBias(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::setCurrBias);
}

void CalibrationWindow::onResetCurrBias(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::resetCurrBias);
}


void CalibrationWindow::onSetSerial(bool)
{
    QtConcurrent::run(this,&CalibrationWindow::setSerial);
}

void CalibrationWindow::onLoading(bool loading)
{
    if(loading){
        ui->centralwidget->setEnabled(false);
        progress->setVisible(true);
        progress->setMaximum(0);
    }else{
        ui->centralwidget->setEnabled(true);
        progress->setVisible(false);
        progress->setMaximum(100);
    }
}

void CalibrationWindow::onUpdateTitle()
{
    QString title = "Calibration";
    if(!eeprom_saved_status){
        title += " - ****** Not saved *****";
    }
    setWindowTitle(title);
}

void CalibrationWindow::closeEvent(QCloseEvent *e)
{
    //timer->stop();
    if(!eeprom_saved_status){
        if(QMessageBox::warning(this, "The regulation set changed on the RAM of the board  ", "Do you want to save the values to EEPROM?", QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes){
            onSaveToEeprom(true);
            e->ignore();
            return;
        }
    }
    keepRunning = false;
    QMainWindow::closeEvent(e);
}

void CalibrationWindow::onMatrixChanged(QTableWidgetItem *item)
{
    Q_UNUSED(item);
    if(first_time){
        return;
    }
    if(item->text() == "-"){
        return;
    }
    QTableWidget *table = (QTableWidget*)sender();
    matrix_changed[matrices.indexOf(table)] = true;
    for(int i=0;i<CHANNEL_COUNT;i++){
        for(int j=0;j<CHANNEL_COUNT;j++){
            table->item(i,j)->setTextColor("red");
        }
    }
}


void CalibrationWindow::onSerialChanged(QString text)
{
    if(first_time){
        return;
    }
    serial_number_changed = true;
}

void CalibrationWindow::onSetSerialChanged(bool changed)
{
    if(changed){
        ui->edit_serial->setStyleSheet("background-color: rgb(255,0,0)");
    }else{
        ui->edit_serial->setStyleSheet("");
    }
}

void CalibrationWindow::onClearLog()
{
    ui->logText->clear();
}

void CalibrationWindow::onAppendLogMsg(QString msg)
{
    if(!msg.isEmpty()){
        ui->logText->appendPlainText(msg);
    }
}

void CalibrationWindow::onResetCalibMatrix(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::resetCalibration);

}

void CalibrationWindow::resetCalibration()
{
    mutex.lock();
    int index = 0;//ui->tabWidget->currentIndex();
    //QLineEdit *editGain = matrixGain.at(index);

    for (int ri=0; ri<6; ri++){
        for (int ci=0; ci<6; ci++){
            if (ri==ci){
                matrix[index][ri][ci] = 32767;
                calib_matrix[index][ri][ci] = 32767;
            } else {
                matrix[index][ri][ci] = 0;
                calib_matrix[index][ri][ci] = 0;
            }
        }
    }
    calib_const[index]=1;
    //setText(editGain,QString("%1").arg(calib_const[index]));
    setMatrix(index);
    matrix_changed[index] = true;
    mutex.unlock();
}


//void CalibrationWindow::onUseMatrixChanged(int value)
//{
//    if(value != 0){
////        ui->tableUseMatrix->hideColumn(COL_MAXMEASURE);
////        ui->tableUseMatrix->hideColumn(COL_MINMEASURE);
////        ui->tableUseMatrix->hideColumn(COL_DIFFMEASURE);
//        ui->tableUseMatrix->showColumn(COL_NEWTONMEASURE);
//        ui->actionImport_Calib_Matrix->setEnabled(true);
//        ui->actionLoad_Calibration_File->setEnabled(true);
//        ui->actionSave_Calibration_File->setEnabled(true);
//    }else{
////        ui->tableUseMatrix->showColumn(COL_MAXMEASURE);
////        ui->tableUseMatrix->showColumn(COL_MINMEASURE);
////        ui->tableUseMatrix->showColumn(COL_DIFFMEASURE);
//        ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);
//        ui->actionImport_Calib_Matrix->setEnabled(false);
//        ui->actionLoad_Calibration_File->setEnabled(false);
//        ui->actionSave_Calibration_File->setEnabled(false);
//    }


//    QtConcurrent::run(this,&CalibrationWindow::useMatrix,value);

//}

void CalibrationWindow::useMatrix(int index, bool boot)
{
    mutex.lock();
    loading();
    string msg;
    if(index >= 0){
        // marco.accame: use the new method strain_set_regulationset w/ strain_regset_one / strain_regset_two / strain_regset_three
        // index = 0 means no calibration used. index = 1 means first set, etc. hence
        int regsetInUseTMP = index + 1;
        if(!boot){
            core->getDownloader()->strain_set_regulationset(bus, id, regsetInUseTMP, cDownloader::strain_regsetmode_temporary, &msg);
        } else {
            core->getDownloader()->strain_set_regulationset(bus, id, regsetInUseTMP, cDownloader::strain_regsetmode_permanent, &msg);
        }
        appendLogMsg(msg.c_str());
    }
    //currentMatrixIndex = index;
    loading(false);
    mutex.unlock();
}




void CalibrationWindow::setOffset(int chan, int value)
{
    // marco.accame: the regulation set to be sent to the board is the one in use in its inside

    mutex.lock();
    loading();
    offset[chan] = value;
    string msg;
    core->getDownloader()->strain_set_offset(bus,id, chan, offset[chan], cDownloader::strain_regset_inuse, &msg);
    appendLogMsg(msg.c_str());
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::onSetText(QLineEdit *edit,QString text)
{
    edit->setText(text);
}

void CalibrationWindow::onSetText(QTableWidgetItem *item ,QString text)
{
    item->setText(text);
}

void CalibrationWindow::onSetTableVisible(QTableWidget *item,bool visible)
{
    item->setVisible(visible);
}

void CalibrationWindow::onApplyDone()
{
    for(int i=0; i<CHANNEL_COUNT;i++){
        ((CustomComboBox*)ui->tableParamters->cellWidget(i,COL_GAIN))->clear();
        ((CustomSpinBox*)ui->tableParamters->cellWidget(i,COL_OFFSET))->clear();
    }
}

void CalibrationWindow::onOffsetSpinnerValue(CustomSpinBox *spinner, int value)
{
    spinner->setCurrentValue(value);
}

void CalibrationWindow::onSetFullScale()
{
    QTableWidget *table = fullScales.first();
    //disconnect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)));

    char tempbuf [250];
    for(int ri=0;ri<CHANNEL_COUNT;ri++){
        double v = fsc::convert(full_scale_calib[0][ri]);
        //qDebug() << v;
        sprintf(tempbuf,"%f",v);
        QTableWidgetItem *item = table->item(ri,0);
        item->setText(tempbuf);
    }




    //connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)),Qt::UniqueConnection);
}
void CalibrationWindow::onSetMatrix(int index)
{
    QTableWidget *table = matrices.at(index);

    //disconnect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)));
    char tempbuf [250];
    for(int ri=0;ri<CHANNEL_COUNT;ri++){
        for(int ci=0;ci<CHANNEL_COUNT;ci++){
            double v = q15::convert(matrix[index][ri][ci]);
            //qDebug() << v;
            sprintf(tempbuf,"%f",v);
            QTableWidgetItem *item = table->item(ri,ci);
            item->setText(tempbuf);
        }


    }

    //connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)),Qt::UniqueConnection);
}

void CalibrationWindow::onTimeout()
{
    // marco.accame: the regulation set for now is the one in use inside the strain2


    qDebug() << "STARTING....";
    keepRunning = true;
    while(keepRunning){
        mutex.lock();

        int bUseCalibration = ui->checkDigital->isChecked();
        string msg;
        bool skip_display_calib=false;

#if 0
        // non va bene discriminare sulla base di busecalibration. lo si deve fare su strain1 / strain2
        if(bUseCalibration) {
            // marco.accame.todo: must use strain_get_regulationset(). attention the values returned will be 1, 2, 3
            // previously strain_get_matrix(currmatrix) returned currmatrix = 0 to mean the first one. hence it was used setCurrentIndex(currmatrix+1)
            core->getDownloader()->strain_get_regulationset(bus, id, regsetInUse, cDownloader::strain_regsetmode_temporary, &msg);

            int bootRegset = cDownloader::strain_regset_inuse;
            core->getDownloader()->strain_get_regulationset(bus, id, bootRegset, cDownloader::strain_regsetmode_permanent, &msg);
            // must now transform usedregulationset.
            // for now, until we have gui support to the three regulation sets ..
            // it is forced to 0 to keep former behaviour of strain_get_matrix()
            //usedregulationset = 0;


            ui->comboRegSet->blockSignals(true);
            ui->comboRegSet->setCurrentIndex(regsetInUse - 1);
            ui->comboRegSet->blockSignals(false);

            ui->comboRegSetBoot->blockSignals(true);
            ui->comboRegSetBoot->setCurrentIndex(bootRegset - 1);
            ui->comboRegSetBoot->blockSignals(false);
        }else{
//            ui->comboUseMatrix->blockSignals(true);
//            ui->comboUseMatrix->setCurrentIndex(0);
//            ui->comboUseMatrix->blockSignals(false);
//            currentMatrixIndex = 0;
        }

#else

        if(icubCanProto_boardType__strain2 == boardtype)
        {
            int regsetInUseTMP = cDownloader::strain_regset_one;

            // marco.accame.todo: must use strain_get_regulationset(). attention the values returned will be 1, 2, 3
            // previously strain_get_matrix(currmatrix) returned currmatrix = 0 to mean the first one. hence it was used setCurrentIndex(currmatrix+1)
            core->getDownloader()->strain_get_regulationset(bus, id, regsetInUseTMP, cDownloader::strain_regsetmode_temporary, &msg);

            int bootRegsetTMP = cDownloader::strain_regset_one;
            core->getDownloader()->strain_get_regulationset(bus, id, bootRegsetTMP, cDownloader::strain_regsetmode_permanent, &msg);
            // must now transform usedregulationset.
            // for now, until we have gui support to the three regulation sets ..
            // it is forced to 0 to keep former behaviour of strain_get_matrix()
            //usedregulationset = 0;


            ui->comboRegSet->blockSignals(true);
            ui->comboRegSet->setCurrentIndex(regsetInUseTMP - 1);
            ui->comboRegSet->blockSignals(false);

            ui->comboRegSetBoot->blockSignals(true);
            ui->comboRegSetBoot->setCurrentIndex(bootRegsetTMP - 1);
            ui->comboRegSetBoot->blockSignals(false);
        }

#endif


        if(first_time) {
            loading();
            ((CustomTreeWidgetItem*)item->getParentNode())->retrieveCanBoards(false);
            for (int i=0;i<((CustomTreeWidgetItem*)item->getParentNode())->getCanBoards().count();i++ ) {
                sBoard b = ((CustomTreeWidgetItem*)item->getParentNode())->getCanBoards().at(i);
                if(b.bus == bus && b.pid == id){
                    selected = i;
                    break;
                }
            }

        }

        if(refresh_serialnumber)
        {
            refresh_serialnumber = false;

            core->getDownloader()->strain_get_serial_number(core->getDownloader()->board_list[selected].bus,
                                                            core->getDownloader()->board_list[selected].pid, serial_no,&msg);
            appendLogMsg(msg.c_str());
            setText(ui->edit_serial,serial_no);

        }

        int ret=0;
        ret = core->getDownloader()->strain_get_eeprom_saved(core->getDownloader()->board_list[selected].bus,
                                                             core->getDownloader()->board_list[selected].pid,
                                                             &eeprom_saved_status,&msg);
        appendLogMsg(msg.c_str());
        if (ret!=0){
            qDebug() << "debug: message 'strain_get_eeprom_saved' lost";
        }
        updateTitle();

        for (int i=0;i<CHANNEL_COUNT;i++){
            if(i==0){
                ret  = core->getDownloader()->strain_get_offset (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, offset[i], cDownloader::strain_regset_inuse, &msg);
            }else{
                ret |= core->getDownloader()->strain_get_offset (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, offset[i], cDownloader::strain_regset_inuse, &msg);
            }
            appendLogMsg(msg.c_str());
        }
        if (ret!=0){
            qDebug() <<"debug: message 'strain_get_offset' lost.";
        }


        for (int i=0;i<CHANNEL_COUNT;i++){
            if(i==0){
                ret  = core->getDownloader()->strain_get_adc (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, adc[i], bUseCalibration,&msg);
            }else{
                ret |= core->getDownloader()->strain_get_adc (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, adc[i], bUseCalibration,&msg);
            }
            appendLogMsg(msg.c_str());
        }

        if (ret!=0){
            qDebug() <<"debug: message 'strain_get_adc' lost.";
        }

        int ri,ci=0;
        char tempbuf [250];




        setSerialChanged(serial_number_changed);

        // marco.accame: maybe it is better to retrieve only one matrix: the selected one, not all three of them

        for(int mi=0;mi<MATRIX_COUNT;mi++){
            if (matrix_changed[mi] == false){
                for (ri=0;ri<CHANNEL_COUNT;ri++){
                    for (ci=0;ci<CHANNEL_COUNT;ci++){
                        core->getDownloader()->strain_get_matrix_rc(core->getDownloader()->board_list[selected].bus,
                                                                    core->getDownloader()->board_list[selected].pid,
                                                                    ri, ci, matrix[mi][ri][ci], cDownloader::strain_regset_inuse, &msg);
                        appendLogMsg(msg.c_str());
                    }
                }
                setMatrix(mi);
//                core->getDownloader()->strain_get_matrix_gain(core->getDownloader()->board_list[selected].bus,
//                                                              core->getDownloader()->board_list[selected].pid,
//                                                              calib_const[mi], regset, &msg);
//                appendLogMsg(msg.c_str());
//                sprintf(tempbuf,"%d",calib_const[mi]);
//                setText(matrixGain.at(mi),tempbuf);

//                for (ri=0;ri<CHANNEL_COUNT;ri++){
//                    core->getDownloader()->strain_get_full_scale(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, full_scale_const[ri],&msg);
//                    appendLogMsg(msg.c_str());
//                    sprintf(tempbuf,"%d",full_scale_const[ri]);
//                    QTableWidgetItem *item2 = ui->tableParamters->item(ri,COL_FULLSCALE);
//                    setText(item2,tempbuf);
//                }

                QTableWidget *table = matrices.at(mi);
                for(int i=0;i<CHANNEL_COUNT;i++){
                    for(int j=0;j<CHANNEL_COUNT;j++){
                        table->item(i,j)->setTextColor("");
                    }
                }
            }else{
                QTableWidget *table = matrices.first();
                for(int i=0;i<CHANNEL_COUNT;i++){
                    for(int j=0;j<CHANNEL_COUNT;j++){
                        table->item(i,j)->setTextColor("red");
                    }
                }
            }
            if(!fullScaleChanged){
                for (ri=0;ri<CHANNEL_COUNT;ri++){
                    fullScales.at(mi)->item(ri,0)->setTextColor("");
                    core->getDownloader()->strain_get_full_scale(bus,id, ri, full_scale_const[mi][ri], cDownloader::strain_regset_inuse, &msg);
                    appendLogMsg(msg.c_str());
                    sprintf(tempbuf,"%d",full_scale_const[mi][ri]);
                    QTableWidgetItem *item2 = fullScales.at(mi)->item(ri,COL_FULLSCALE);
                    setText(item2,tempbuf);


                }
            } else {
                for(int i=0;i<CHANNEL_COUNT;i++){
                    fullScales.first()->item(i,0)->setTextColor("red");
                }
            }

        }


        if(clearStats)
        {
            clearStats = false;
            for(int i=0;i<CHANNEL_COUNT;i++)
            {
                maxadc[i]           = -32768;
                minadc[i]           = +32767;
                maxft[i]            = -32768;
                minft[i]            = +32767;
            }
        }

        for (int i=0;i<CHANNEL_COUNT;i++){
            if (!bUseCalibration){
                if(convert_to_signed32k(adc[i])>maxadc[i]){
                    maxadc[i]=convert_to_signed32k(adc[i]);
                }
                if (convert_to_signed32k(adc[i])<minadc[i]){
                    minadc[i]=convert_to_signed32k(adc[i]);
                }
                maxft[i]=-32768;
                minft[i]=+32767;
            } else {
                if(convert_to_signed32k(adc[i])>maxft[i]){
                    maxft[i]=convert_to_signed32k(adc[i]);
                }
                if (convert_to_signed32k(adc[i])<minft[i]){
                    minft[i]=convert_to_signed32k(adc[i]);
                }

                maxadc[i]=-32768;
                minadc[i]=+32767;
            }
        }


        if(bUseCalibration)
        {
            if(ui->tableCurr->horizontalHeaderItem(0)->text() != "ForceTorque"){
                setText(ui->tableCurr->horizontalHeaderItem(0),"ForceTorque");

                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_MAXMEASURE),"Max FT");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_MINMEASURE),"Min FT");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_DIFFMEASURE),"Delta FT");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_NEWTONMEASURE),"FT");



                for (int i=0;i<CHANNEL_COUNT;i++){
                    setText(ui->tableUseMatrix->verticalHeaderItem(i),QString("%1").arg(i==0 ?"Fx"
                                                                                             : i==1 ? "Fy"
                                                                                             : i==2 ? "Fz"
                                                                                             : i==3 ? "Mx"
                                                                                             : i==4 ? "My"
                                                                                             : "Mz" ));
                    setText(ui->tableCurr->verticalHeaderItem(i),QString("%1").arg(i==0 ?"Fx"
                                                                                             : i==1 ? "Fy"
                                                                                             : i==2 ? "Fz"
                                                                                             : i==3 ? "Mx"
                                                                                             : i==4 ? "My"
                                                                                             : "Mz" ));
                }

                setTableVisible(ui->tableCurr,false);
                setTableVisible(ui->tableCurr,true);
                setTableVisible(ui->tableUseMatrix,false);
                setTableVisible(ui->tableUseMatrix,true);
            }
        }
        else
        {
            if(ui->tableCurr->horizontalHeaderItem(0)->text() != "ADC"){
                setText(ui->tableCurr->horizontalHeaderItem(0),"ADC");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_MAXMEASURE),"Max ADC");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_MINMEASURE),"Min ADC");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_DIFFMEASURE),"Delta ADC");
                setText(ui->tableUseMatrix->horizontalHeaderItem(COL_NEWTONMEASURE),"Empty");

                for (int i=0;i<CHANNEL_COUNT;i++){
                    setText(ui->tableUseMatrix->verticalHeaderItem(i),QString("Ch:%1").arg(i));
                    setText(ui->tableCurr->verticalHeaderItem(i),QString("Ch:%1").arg(i));
                }

                setTableVisible(ui->tableCurr,false);
                setTableVisible(ui->tableCurr,true);
                setTableVisible(ui->tableUseMatrix,false);
                setTableVisible(ui->tableUseMatrix,true);
            }
        }


        for (int i=0;i<CHANNEL_COUNT;i++){

            if(icubCanProto_boardType__strain2 == boardtype)
            {
                core->getDownloader()->strain_get_amplifier_regs(core->getDownloader()->board_list[selected].bus,
                                                                 core->getDownloader()->board_list[selected].pid, i,
                                                                 amp_registers[i], cDownloader::strain_regset_inuse, &msg);
                core->getDownloader()->strain_get_amplifier_gain_offset(core->getDownloader()->board_list[selected].bus,
                                                                        core->getDownloader()->board_list[selected].pid, i,
                                                                        amp_gains[i], amp_offsets[i], cDownloader::strain_regset_inuse, &msg);
//#warning TEST di ricezione di un valore di gain inconsueto... dove viene cambiato il valore del combo sull base del valore ricevuto dall strain?
                appendLogMsg(msg.c_str());


                CustomComboBox *combo = (CustomComboBox *)ui->tableParamters->cellWidget(i,COL_GAIN);
                bool found = false;
                for(int k=0;k<combo->count();k++){
                    if(combo->itemData(k).toInt() == amp_gains[i]){

                        combo->setCurrIndex(k);
                        found = true;
                        break;
                    }
                }
                if(!found){
                    combo->addCustomValue(amp_gains[i]);
                }

            }
            else
            {
                amp_gains[i] = 1.0f;
                amp_offsets[i] = 0;
            }


            core->getDownloader()->strain_get_calib_bias(core->getDownloader()->board_list[selected].bus,
                                                         core->getDownloader()->board_list[selected].pid, i,
                                                         calib_bias[i], cDownloader::strain_regset_inuse, &msg);
            appendLogMsg(msg.c_str());
            //sprintf(tempbuf,"%d (%d)",showasQ15(calib_bias[i]), showBias(calib_bias[i]));
            sprintf(tempbuf,"%d", showasQ15(calib_bias[i]));

            QTableWidgetItem *item = ui->tableBias->item(i,COL_CALIBBIAS);
            setText(item,tempbuf);


//            core->getDownloader()->strain_get_curr_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, curr_bias[i], &msg);
//            appendLogMsg(msg.c_str());
//            sprintf(tempbuf,"%d", showasQ15(curr_bias[i]));
//            QTableWidgetItem *item1 = ui->tableParamters->item(i,COL_CURRBIAS);
//            setText(item1,tempbuf);

//            sprintf (tempbuf,"%d",full_scale_const[i]);
//            QTableWidgetItem *item2 = ui->tableParamters->item(i,COL_FULLSCALE);
//            //item2->setText(tempbuf);
//            setText(item2,tempbuf);

            CustomSpinBox *slider = slider_offset.at(i);
            setOffsetSliderValue(slider,offset[i]);

            // marco.accame: we always show the received value in range [-32k, +32k).
            // in case of 0 == bUseCalibration: it is the adc value
            // in case of 1 == bUseCalibration: it is = M * (adc+calibtare) + currtare
            sprintf(tempbuf,"%d",convert_to_signed32k(adc[i]));

            QTableWidgetItem *item3 = NULL;
            if(bUseCalibration){
                item3 = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
            }else{
                item3 = ui->tableCurr->item(i,COL_CURRMEASURE);
            }
            setText(item3,tempbuf);




            if(bUseCalibration){
                /******************************  Use digital ***************************/
                sprintf(tempbuf,"%d",maxft[i]);
                QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_MAXMEASURE);
                setText(item,tempbuf);

                sprintf(tempbuf,"%d",minft[i]);
                QTableWidgetItem *item1 = ui->tableUseMatrix->item(i,COL_MINMEASURE);
                setText(item1,tempbuf);

                sprintf(tempbuf,"%d",maxft[i]-minft[i]);
                QTableWidgetItem *item2 = ui->tableUseMatrix->item(i,COL_DIFFMEASURE);
                setText(item2,tempbuf);

                if (full_scale_const[currentMatrixIndex][i]==0){
                    qDebug() << "Error getting the full scale "<< i << " from the sensor";
                    skip_display_calib=true;
                }



                if (skip_display_calib==false){
                    if(i<=2){
                        sprintf(tempbuf,"%+.3f N",(convert_to_signed32k(adc[i]))/float(RANGE32K)*full_scale_const[currentMatrixIndex][i]);
                    }else{
                        sprintf(tempbuf,"%+.3f Nm",(convert_to_signed32k(adc[i]))/float(RANGE32K)*full_scale_const[currentMatrixIndex][i]);
                    }
                    //QTableWidgetItem *item3 = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    QTableWidgetItem *item3 = ui->tableCurr->item(i,COL_CURRMEASURE);
                    setText(item3,tempbuf);
                }else{
                    //QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    QTableWidgetItem *item3 = ui->tableCurr->item(i,COL_CURRMEASURE);
                    setText(item3,"ERROR");
                }
            } else {
                /****************************** Don't Use digital ***************************/
                sprintf(tempbuf,"%d",maxadc[i]);
                QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_MAXMEASURE);
                setText(item,tempbuf);


                sprintf(tempbuf,"%d",minadc[i]);
                QTableWidgetItem *item1 = ui->tableUseMatrix->item(i,COL_MINMEASURE);
                setText(item1,tempbuf);


                sprintf(tempbuf,"%d",maxadc[i]-minadc[i]);
                QTableWidgetItem *item2 = ui->tableUseMatrix->item(i,COL_DIFFMEASURE);
                setText(item2,tempbuf);

                QTableWidgetItem *item3 = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                setText(item3,"");

//                if(i<=2){
//                    QTableWidgetItem *item = ui->tableCurr->item(i,COL_CURRMEASURE);
//                    setText(item,"--- N");
//                }else{
//                    QTableWidgetItem *item = ui->tableCurr->item(i,COL_CURRMEASURE);
//                    setText(item,"--- Nm");
//                }
            }
        }



        if(first_time){
            first_time = false;
            loading(false);

        }

        mutex.unlock();



        QThread::msleep(500);

    }

}

bool CalibrationWindow::calibration_load_v3 (char* filename, int selected_bus, int selected_id, int index, int regset)
{
    if (filename==NULL){
        yError("File not found!\n");
        appendLogMsg("File not found!");
        return false;
    }
    if (selected_id <1 || selected_id >= 15){
        yError("Invalid board address!\n");
        appendLogMsg("Invalid board address!");
        return false;
    }

    int file_version=0;
    fstream filestr;
    filestr.open (filename, fstream::in);
    if (!filestr.is_open()){
        yError("Error opening calibration file!\n");
        appendLogMsg("Error opening calibration file!");
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
        appendLogMsg("Wrong file. Calibration version not supported for strain2");
        return false;
    }
    else if((icubCanProto_boardType__strain == boardtype) && (2 != file_version))
    {
        yError("Wrong file. Calibration version not supported: %d\n", file_version);
        appendLogMsg("Wrong file. Calibration version not supported");
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
            appendLogMsg("Wrong file. Board type not supported");
            return false;
        }

        // Serial number:
        filestr.getline (buffer,256);
        filestr.getline (buffer,256);
        sprintf(serial_no,"%s", buffer);
        core->getDownloader()->strain_set_serial_number(bus,id, serial_no);
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

            core->getDownloader()->strain_set_amplifier_regs(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, amp_registers[i], regset);

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
        sprintf(serial_no,"%s", buffer);
        core->getDownloader()->strain_set_serial_number(bus,id, serial_no);

        //offsets
        filestr.getline (buffer,256);
        for (i=0;i<CHANNEL_COUNT; i++)
        {
            filestr.getline (buffer,256);
            sscanf  (buffer,"%d",&offset[i]);
            // downloader.strain_set_offset (downloader.board_list[selected].bus, downloader.board_list[selected].pid, i, offset[i]);
            core->getDownloader()->strain_set_offset (bus,id, i, offset[i], regset);
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
        printf("%d %x\n", calib_matrix[index][ri][ci],calib_matrix[index][ri][ci]);
        core->getDownloader()->strain_set_matrix_rc(bus,id, ri, ci, calib_matrix[index][ri][ci], regset);
    }



    //matrix gain
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    core->getDownloader()->strain_set_matrix_gain(bus,id, cc, regset);

    //tare
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&calib_bias[i]);
        core->getDownloader()->strain_set_calib_bias(bus,id, i, calib_bias[i], regset);
    }

    //full scale values
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&full_scale_const[index][i]);
        core->getDownloader()->strain_set_full_scale(bus,id, i, full_scale_const[index][i], regset);
    }



    filestr.close();
    filestr.clear();

    matrix_changed[0]=true;
    matrix_changed[1]=true;
    matrix_changed[2]=true;
    something_changed=true;
    printf ("Calibration file loaded!\n");
    appendLogMsg("Calibration file loaded!");

    return true;
}

#if 0
bool CalibrationWindow::calibration_load_v2 (char* filename, int selected_bus, int selected_id, int index)
{
    const int regset = cDownloader::strain_regset_inuse;

    if (filename==NULL){
        yError("File not found!\n");
        appendLogMsg("File not found!");
        return false;
    }
    if (selected_id <1 || selected_id >= 15){
        yError("Invalid board address!\n");
        appendLogMsg("Invalid board address!");
        return false;
    }

    int file_version=0;
    fstream filestr;
    filestr.open (filename, fstream::in);
    if (!filestr.is_open()){
        yError("Error opening calibration file!\n");
        appendLogMsg("Error opening calibration file!");
        return false;
    }

    int i=0;
    char buffer[256];

    //file version
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sscanf (buffer,"%d",&file_version);
    if (file_version!=2){
        yError("Wrong file. Calibration version != 2\n");
        appendLogMsg("Wrong file. Calibration version != 2");
        return false;
    }

    //serial number
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sprintf(serial_no,"%s", buffer);
    core->getDownloader()->strain_set_serial_number(bus,id, serial_no);

    //offsets
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&offset[i]);
        // downloader.strain_set_offset (downloader.board_list[selected].bus, downloader.board_list[selected].pid, i, offset[i]);
        core->getDownloader()->strain_set_offset (bus,id, i, offset[i], regset);
        drv_sleep(200);
    }

    //calibration matrix
    filestr.getline (buffer,256);
    for (i=0;i<36; i++){
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[index][ri][ci]);
        printf("%d %x\n", calib_matrix[index][ri][ci],calib_matrix[index][ri][ci]);
        core->getDownloader()->strain_set_matrix_rc(bus,id, ri, ci, calib_matrix[index][ri][ci], regset);
    }



    //matrix gain
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    core->getDownloader()->strain_set_matrix_gain(bus,id, cc, regset);

    //tare
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&calib_bias[i]);
        core->getDownloader()->strain_set_calib_bias(bus,id, i, calib_bias[i], regset);
    }

    //full scale values
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&full_scale_const[index][i]);
        core->getDownloader()->strain_set_full_scale(bus,id, i, full_scale_const[index][i], regset);
    }



    filestr.close();
    filestr.clear();

    matrix_changed[0]=true;
    matrix_changed[1]=true;
    matrix_changed[2]=true;
    something_changed=true;
    printf ("Calibration file loaded!\n");
    appendLogMsg("Calibration file loaded!");

    return true;
}
#endif


#if defined(MARCO_ACCAME_19SEP2018)
void CalibrationWindow::importCalibrationFileHEX(QString fileName)
{
    // marco.accame: the regulation set to be sent to the board is the one in use in its inside
    const int regset = cDownloader::strain_regset_inuse;
    const int index = 0;

    mutex.lock();
    loading();
    char* buff = fileName.toLatin1().data();
    string msg;

    if (buff==NULL){
        yError("File not found!\n");
        appendLogMsg("File not found!");
        loading(false);
        mutex.unlock();
        return;
    }

    fstream filestr;
    filestr.open (buff, fstream::in);
    if (!filestr.is_open()){
        yError("Error opening MATLAB matrix-fullscale file!\n");
        appendLogMsg("Error opening MATLAB matrix-fullscale file!");
        loading(false);
        mutex.unlock();
        return;
    }

    printf("Importing MATLAB matrix-fullscale file.\n");

    printf("matrix[6][6] = \n");
    unsigned int mat0[6][6] = {0};
    int i=0;
    char buffer[256];
    for (i=0;i<36; i++){
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&mat0[ri][ci]);
        printf("%f (0x%x)\n", strain::dsp::q15::convert(mat0[ri][ci]), mat0[ri][ci]);
        core->getDownloader()->strain_set_matrix_rc(bus,id,ri,ci,mat0[ri][ci], regset, &msg);
        appendLogMsg(msg.c_str());
    }


    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    core->getDownloader()->strain_set_matrix_gain(bus,id,cc, regset, &msg);
    appendLogMsg(msg.c_str());

    printf("gain = %d [BUT IT IS UNUSED]\n", cc);

    printf("fullscales[6] = \n");
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf (buffer,"%d",&cc);
        printf("%d\n", cc);
        core->getDownloader()->strain_set_full_scale(bus,id,i,cc, regset, &msg);
        appendLogMsg(msg.c_str());
    }

    filestr.close();

//    something_changed=true;
    printf ("MATLAB matrix-fullscale file loaded!\n");
    appendLogMsg("MATLAB matrix-fullscale file loaded!");

    int ri=0;
    int ci=0;

    unsigned int mat1[6][6] = {0};
    drv_sleep (1000);
    for (ri=0;ri<CHANNEL_COUNT;ri++){
        for (ci=0;ci<CHANNEL_COUNT;ci++){
            core->getDownloader()->strain_get_matrix_rc(bus,id,ri,ci,mat1[ri][ci], regset, &msg);
            appendLogMsg(msg.c_str());
        }
    }
//    setMatrix(index);
//    resetMatrices(index);

    int count_ok=0;
    for (i=0;i<36; i++){
        ri=i/6;
        ci=i%6;
        if (mat0[ri][ci]==mat1[ri][ci]) {
            count_ok++;
        } else {
            printf ("Found 1 error on element %d,%d !!\n",ri, ci);
            appendLogMsg("Found 1 error on element %d,%d !!");
        }
    }

    if (count_ok==36){
        printf ("MATLAB matrix-fullscale file %s applied with no errors\n", buff);
        appendLogMsg(QString("MATLAB matrix-fullscale file %1 applied with no errors").arg(buff));
//        matrix_changed[0]=false;
    } else {
        printf ("Found %d errors applying the MATLAB matrix-fullscale file!!\n",36-count_ok);
        appendLogMsg(QString("Found %1 errors applying the MATLAB matrix-fullscale file!!").arg(36-count_ok));
    }

    matrix_changed[index] = false;
    fullScaleChanged = false;
    something_changed = false;

    loading(false);
    mutex.unlock();
}
#endif // (MARCO_ACCAME_19SEP2018)

void CalibrationWindow::logger(void *caller, const std::string &msg)
{
    if(NULL != caller)
    {
        CalibrationWindow *cw = reinterpret_cast<CalibrationWindow*>(caller);
        cw->appendLogMsg(QString::fromStdString(msg));
    }

}

bool CalibrationWindow::SetDefaultRegulationSet(int regset, bool alsoserialnumber)
{
    int i=0;

    if(alsoserialnumber)
    {
        // serial number
        char my_serial_no[8] = "SN0000";
        core->getDownloader()->strain_set_serial_number(bus,id, my_serial_no);
    }

    if((icubCanProto_boardType__strain2 == boardtype))
    {
        // amplifier registers:
        for (i=0;i<CHANNEL_COUNT; i++)
        {
            core->getDownloader()->strain_set_amplifier_gain_offset(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, core->getDownloader()->strain_amplifier_discretegain2float(defaultStrain2AmplGains[i]), defaultStrain2AmplOffsets[i], regset);
            drv_sleep(10);
        }

    }
    else
    {
        // offsets
        for (i=0;i<CHANNEL_COUNT; i++)
        {
            core->getDownloader()->strain_set_offset (bus,id, i, defaultStrain1DACoffsets[i], regset);
            drv_sleep(200);
        }
    }

    // calibration matrix
    for (i=0;i<36; i++)
    {
        int ri=i/6;
        int ci=i%6;
        unsigned int value = 0;
        if(ri == ci)
        {
            value = 32767;
        }
        core->getDownloader()->strain_set_matrix_rc(bus,id, ri, ci, value, regset);
    }



    // matrix gain
    core->getDownloader()->strain_set_matrix_gain(bus,id, 1, regset);

    // tare
    for (i=0;i<CHANNEL_COUNT; i++)
    {
        core->getDownloader()->strain_set_calib_bias(bus,id, i, 0, regset);
    }

    // full scale values
    for (i=0;i<CHANNEL_COUNT; i++)
    {
        unsigned int fs =
        core->getDownloader()->strain_set_full_scale(bus,id, i, 32767, regset);
    }

    return true;
}
