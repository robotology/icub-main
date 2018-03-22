#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"
#include <qdebug.h>
#include <QLabel>
#include <QHBoxLayout>
#include <QtConcurrent/QtConcurrent>
#include <QFileDialog>
#include <QMessageBox>

#define CHANNEL_COUNT   6
#define HEX_VALC 0x8000
#define RANGE32K 32768

#define     COL_CURRMEASURE     0

#define     COL_GAIN            0
#define     COL_OFFSET          1
#define     COL_CALIBBIAS       2
#define     COL_CURRBIAS        3

#define     COL_FULLSCALE       0

#define     COL_MAXMEASURE      0
#define     COL_MINMEASURE      1
#define     COL_DIFFMEASURE     2
#define     COL_NEWTONMEASURE   3

#define     MATRIX_COUNT        3

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
    currentMatrixIndex = 0;

    calibration_value = 32767;
    calib_const[0] = 0;
    calib_const[1] = 0;
    calib_const[2] = 0;
    serial_number_changed = false;
    matrix_changed[0] = false;
    matrix_changed[1] = false;
    matrix_changed[2] = false;

    boardtype = b; // we can have either a icubCanProto_boardType__strain or a icubCanProto_boardType__strain2 ...
    eeprom_saved_status=true;
    first_time = true;
    for(int i=0;i<CHANNEL_COUNT;i++){
        ch[i]               = i;
        adc[i]              = 0;
        calib_bias[i]       = 0;
        curr_bias[i]        = 0;
        // marco.accame: we want to show the maximum / minimum values in the same way the adc values are displayed: in range [-32k, +32k)
        maxadc[i]           = -32768;
        minadc[i]           = +32767;

        sliderPressed[i]    = false;
        amp_gain1[i]        = 0;
        amp_gain2[i]        = 0;
    }
    strncpy(serial_no,"UNDEF",8);

    for(int m=0;m<MATRIX_COUNT;m++){
        for(int i=0;i<CHANNEL_COUNT;i++){
            full_scale_const[m][i] = 0;
        }
    }




    for (int i=0;i <CHANNEL_COUNT;i++) {
        QWidget *container = new QWidget(ui->tableParamters);
        QSlider *slider = new QSlider(container);
        slider->setMinimum(0);
        if(icubCanProto_boardType__strain2 == boardtype)
        {
            slider->setMaximum(0xFFFF);
        }
        else
        {
            slider->setMaximum(0x3FF);
        }
        slider_gain.append(slider);
        slider->setOrientation(Qt::Horizontal);

        QLabel *lbl = new QLabel(container);
        lbl->setText("--");

        QHBoxLayout *l = new QHBoxLayout(container);
        l->addWidget(slider);
        l->addWidget(lbl);
        container->setLayout(l);
        ui->tableParamters->setCellWidget(i,COL_OFFSET,container);


        connect(slider,SIGNAL(valueChanged(int)),this,SLOT(onOffsetSliderValue(int)),Qt::DirectConnection);
        connect(slider,SIGNAL(sliderPressed()),this,SLOT(onSliderPressed()));
        connect(slider,SIGNAL(sliderReleased()),this,SLOT(onSliderReleased()));


        QTableWidgetItem *item = new QTableWidgetItem("-");
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        ui->tableCurr->setItem(i,COL_CURRMEASURE,item);

        //QTableWidgetItem *header2 = new QTableWidgetItem();
        //header2->setText("CIAO");
        //tableWidget->setHorizontalHeaderItem(1,header2);
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
            ui->matrixA->setItem(i,j,item);

            QTableWidgetItem *item1 = new QTableWidgetItem("-");
            ui->matrixB->setItem(i,j,item1);

            QTableWidgetItem *item2 = new QTableWidgetItem("-");
            ui->matrixC->setItem(i,j,item2);
        }


        QTableWidgetItem *item001 = new QTableWidgetItem("-");
        item001->setFlags(item001->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_GAIN,item001);


        QTableWidgetItem *item5 = new QTableWidgetItem("-");
        item5->setFlags(item5->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_CALIBBIAS,item5);


        QTableWidgetItem *item6 = new QTableWidgetItem("-");
        item6->setFlags(item6->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_CURRBIAS,item6);

        QTableWidgetItem *item7 = new QTableWidgetItem("-");
        item7->setFlags(item7->flags() ^ Qt::ItemIsEditable);
        ui->tableFullScaleA->setItem(i,COL_FULLSCALE,item7);

        QTableWidgetItem *item8 = new QTableWidgetItem("-");
        item8->setFlags(item8->flags() ^ Qt::ItemIsEditable);
        ui->tableFullScaleB->setItem(i,COL_FULLSCALE,item8);

        QTableWidgetItem *item9 = new QTableWidgetItem("-");
        item9->setFlags(item9->flags() ^ Qt::ItemIsEditable);
        ui->tableFullScaleC->setItem(i,COL_FULLSCALE,item9);

        fullScales.append(ui->tableFullScaleA);
        fullScales.append(ui->tableFullScaleB);
        fullScales.append(ui->tableFullScaleC);

    }

    ui->tableParamters->setColumnWidth(COL_OFFSET,150);
    ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);



// marco.accame: it was...
//    ui->slider_zero->setMinimum(0);
//    ui->slider_zero->setMaximum(65535);
//    ui->slider_zero->setValue(32767);
     //marco.accame: is ...
    ui->slider_zero->setMinimum(-32768);
    ui->slider_zero->setMaximum(32767);
    ui->slider_zero->setValue(0);
    ui->zeroLbl->setText(QString("%1").arg(ui->slider_zero->value()));

    progress = new QProgressBar(this);
    progress->setFixedWidth(100);
    progress->setMinimum(0);
    progress->setMaximum(100);
    progress->setVisible(false);
    ui->statusbar->addWidget(progress);

    matrixGain.append(ui->editMatrixGainA);
    matrixGain.append(ui->editMatrixGainB);
    matrixGain.append(ui->editMatrixGainC);

    matrices.append(ui->matrixA);
    matrices.append(ui->matrixB);
    matrices.append(ui->matrixC);


    connect(ui->comboUseMatrix,SIGNAL(currentIndexChanged(int)),this,SLOT(onUseMatrixChanged(int)),Qt::QueuedConnection);

    connect(ui->btnClose,SIGNAL(clicked(bool)),this,SLOT(close()));
    connect(ui->edit_serial,SIGNAL(textEdited(QString)),this,SLOT(onSerialChanged(QString)));
    connect(this,SIGNAL(loading(bool)),this,SLOT(onLoading(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setText(QLineEdit*,QString)),this,SLOT(onSetText(QLineEdit*,QString)),Qt::QueuedConnection);
    connect(this,SIGNAL(setText(QTableWidgetItem*,QString)),this,SLOT(onSetText(QTableWidgetItem*,QString)),Qt::QueuedConnection);
    connect(this,SIGNAL(setSliderValue(QSlider*,int)),this,SLOT(onSliderValue(QSlider*,int)),Qt::BlockingQueuedConnection);
    connect(ui->btnSetCalibBias,SIGNAL(clicked(bool)),this,SLOT(onSetCalibBias(bool)));
    connect(ui->btnResetCalibBias,SIGNAL(clicked(bool)),this,SLOT(onResetCalibBias(bool)));
    connect(ui->btnSetCurrBias,SIGNAL(clicked(bool)),this,SLOT(onSetCurrBias(bool)));
    connect(ui->btnResetCurrBias,SIGNAL(clicked(bool)),this,SLOT(onResetCurrBias(bool)));
    connect(ui->btnAutoAdjust,SIGNAL(clicked(bool)),this,SLOT(onAutoAdjust(bool)));
    connect(ui->slider_zero,SIGNAL(valueChanged(int)),this,SLOT(onSliderZeroChanged(int)));
    connect(ui->btnSetSerial,SIGNAL(clicked(bool)),this,SLOT(onSetSerial(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setSerialChanged(bool)),this,SLOT(onSetSerialChanged(bool)));
    connect(ui->btnResetCalib,SIGNAL(clicked(bool)),this,SLOT(onResetCalibMatrix(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(setMatrix(int)),this,SLOT(onSetMatrix(int)),Qt::QueuedConnection);
    connect(ui->actionSave_To_Eproom,SIGNAL(triggered(bool)),this,SLOT(onSaveToEeprom(bool)),Qt::QueuedConnection);
    connect(ui->btnSetCalibration,SIGNAL(clicked(bool)),this,SLOT(onSetCalibration(bool)),Qt::QueuedConnection);
    connect(this,SIGNAL(resetMatrices(int)),this,SLOT(resetMatricesState(int)),Qt::QueuedConnection);
    connect(this,SIGNAL(updateTitle()),this,SLOT(onUpdateTitle()),Qt::QueuedConnection);
    connect(this,SIGNAL(appendLogMsg(QString)),this,SLOT(onAppendLogMsg(QString)),Qt::QueuedConnection);
    connect(ui->btnClearLog,SIGNAL(clicked(bool)),this,SLOT(onClearLog()));
    connect(ui->tabWidget,SIGNAL(currentChanged(int)),this,SLOT(onTabMatrixCahnged(int)));
    connect(ui->btnLoadCalibFile,SIGNAL(clicked(bool)),this,SLOT(onLoadCalibrationFile(bool)));
    connect(ui->btnSaveCalibFile,SIGNAL(clicked(bool)),this,SLOT(onSaveCalibrationFile(bool)));
    connect(ui->btnImportCalibMatrix,SIGNAL(clicked(bool)),this,SLOT(onImportCalibrationFile(bool)));


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

void CalibrationWindow::onTabMatrixCahnged(int index)
{
    // This will be removed in the future
    switch (index) {
    case 0:
        ui->btnLoadCalibFile->setEnabled(true);
        ui->btnSaveCalibFile->setEnabled(true);
        ui->btnImportCalibMatrix->setEnabled(true);
        ui->btnResetCalib->setEnabled(true);
        ui->btnSetCalibration->setEnabled(true);
        break;
    default:
        ui->btnLoadCalibFile->setEnabled(false);
        ui->btnSaveCalibFile->setEnabled(false);
        ui->btnImportCalibMatrix->setEnabled(false);
        ui->btnResetCalib->setEnabled(false);
        ui->btnSetCalibration->setEnabled(false);
        break;
    }

}

void CalibrationWindow::onSliderPressed()
{
    int index = slider_gain.indexOf((QSlider*)sender());
    sliderPressed[index] = true;
}

void CalibrationWindow::onSliderReleased()
{
    int index = slider_gain.indexOf((QSlider*)sender());

    QtConcurrent::run(this,&CalibrationWindow::setOffset,index,((QSlider*)sender())->value());

    sliderPressed[index] = false;

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
    // marco.accame: transform [-32K, +32K) into [0, +64K) as required by strain_calibrate_offset()
    unsigned int middlevalue = 32768 + ui->slider_zero->value();
    core->getDownloader()->strain_calibrate_offset(bus,id, boardtype, middlevalue, &msg);
    loading(false);
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
    resetMatrices();
    loading(false);
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
        QLineEdit *gain = matrixGain.at(i);

        disconnect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)));
        for(int r=0;r<CHANNEL_COUNT;r++){
            for(int c=0;c<CHANNEL_COUNT;c++){
                table->item(r,c)->setBackgroundColor("white");
            }
        }
        connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)),Qt::UniqueConnection);
        gain->setStyleSheet("");
    }
}

void CalibrationWindow::setCalibration()
{
    mutex.lock();
    int index = ui->tabWidget->currentIndex();
    QTableWidget *table = matrices.at(index);


    loading();
    for (int ri=0; ri<CHANNEL_COUNT; ri++){
        for (int ci=0; ci<CHANNEL_COUNT; ci++){
            const char* temp2 = table->item(ri,ci)->text().toLatin1().data();
            sscanf (temp2,"%x",&matrix[index][ri][ci]);
            string msg;
            core->getDownloader()->strain_set_matrix_rc(bus,id,ri,ci,matrix[index][ri][ci],index,&msg);
            appendLogMsg(msg.c_str());
        }
    }

    string msg;
    core->getDownloader()->strain_set_matrix_gain(bus,id,calib_const[index],index,&msg);
    appendLogMsg(msg.c_str());
    qDebug("Calibration matrix updated");

    resetMatrices(index);

    loading(false);
    mutex.unlock();
}

void CalibrationWindow::loadCalibrationFile(QString fileName)
{
    mutex.lock();
    int index = ui->tabWidget->currentIndex();
    loading();
    //load data file
    if(!calibration_load_v2 (fileName.toLatin1().data(), bus, id,index)){
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
    core->getDownloader()->strain_get_serial_number(bus,id, buffer,&msg);

    setText(ui->edit_serial,QString(buffer));
    serial_number_changed=false;

    drv_sleep (500);
    for (ri=0;ri<CHANNEL_COUNT;ri++){
        for (ci=0;ci<CHANNEL_COUNT;ci++){
            core->getDownloader()->strain_get_matrix_rc(bus,id, ri, ci, matrix[index][ri][ci],index,&msg);
            appendLogMsg(msg.c_str());
            sprintf(buffer,"%x",matrix[index - 1][ri][ci]);
            setMatrix(index);
            resetMatrices(index);
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
    int index = ui->tabWidget->currentIndex();
    char *c = filePath.toLatin1().data();
    std::string filename = c;
    filename += "/calibrationData";
    filename += serial_no;
    filename += ".dat";
    fstream filestr;
    filestr.open (filename.c_str(), fstream::out);
    int i=0;
    char buffer[256];

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
    mutex.lock();
    loading();
    int index = ui->tabWidget->currentIndex();
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
        yError("Error opening calibration file!\n");
        appendLogMsg("Error opening calibration file!");
        loading(false);
        mutex.unlock();
        return;
    }

    int i=0;
    char buffer[256];
    for (i=0;i<36; i++){
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[index][ri][ci]);
        printf("%d %x\n", calib_matrix[index][ri][ci],calib_matrix[index][ri][ci]);
        core->getDownloader()->strain_set_matrix_rc(bus,id,ri,ci,calib_matrix[index][ri][ci],index,&msg);
        appendLogMsg(msg.c_str());
    }


    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    core->getDownloader()->strain_set_matrix_gain(bus,id,cc,0,&msg);
    appendLogMsg(msg.c_str());

    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf (buffer,"%d",&cc);
        core->getDownloader()->strain_set_full_scale(bus,id,i,cc,index,&msg);
        appendLogMsg(msg.c_str());
    }

    filestr.close();

    something_changed=true;
    printf ("Calibration file loaded!\n");
    appendLogMsg("Calibration file loaded!");

    int ri=0;
    int ci=0;

    drv_sleep (1000);
    for (ri=0;ri<CHANNEL_COUNT;ri++){
        for (ci=0;ci<CHANNEL_COUNT;ci++){
            core->getDownloader()->strain_get_matrix_rc(bus,id,ri,ci,matrix[index][ri][ci],0,&msg);
            appendLogMsg(msg.c_str());
        }
    }
    setMatrix(index);
    resetMatrices(index);

    int count_ok=0;
    for (i=0;i<36; i++){
        ri=i/6;
        ci=i%6;
        if (calib_matrix[index][ri][ci]==matrix[index][ri][ci]) {
            count_ok++;
        } else {
            printf ("Found 1 error on element %d,%d !!\n",ri, ci);
            appendLogMsg("Found 1 error on element %d,%d !!");
        }
    }

    if (count_ok==36){
        printf ("Calibration file %s applied with no errors\n", buff);
        appendLogMsg(QString("Calibration file %1 applied with no errors").arg(buff));
        matrix_changed[0]=false;
    } else {
        printf ("Found %d errors applying the calibration file!!\n",36-count_ok);
        appendLogMsg(QString("Found %1 errors applying the calibration file!!").arg(36-count_ok));
    }






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

void CalibrationWindow::onAutoAdjust(bool click)
{
    QtConcurrent::run(this,&CalibrationWindow::autoAdjust);
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
        if(QMessageBox::warning(this,"The Calibration has not been saved","Do you want to save this calibration to eeprom?",QMessageBox::Yes,QMessageBox::No) == QMessageBox::Yes){
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
            table->item(i,j)->setBackgroundColor("red");
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
    int index = ui->tabWidget->currentIndex();
    QLineEdit *editGain = matrixGain.at(index);

    for (int ri=0; ri<6; ri++){
        for (int ci=0; ci<6; ci++){
            if (ri==ci){
                matrix[index][ri][ci] = 32767;
            } else {
                matrix[index][ri][ci] = 0;
            }
        }
    }
    calib_const[index]=1;
    setText(editGain,QString("%1").arg(calib_const[index]));
    setMatrix(index);
    matrix_changed[index] = true;
    mutex.unlock();
}


void CalibrationWindow::onUseMatrixChanged(int value)
{
    if(value != 0){
        ui->tableUseMatrix->hideColumn(COL_MAXMEASURE);
        ui->tableUseMatrix->hideColumn(COL_MINMEASURE);
        ui->tableUseMatrix->hideColumn(COL_DIFFMEASURE);
        ui->tableUseMatrix->showColumn(COL_NEWTONMEASURE);
        ui->actionImport_Calib_Matrix->setEnabled(true);
        ui->actionLoad_Calibration_File->setEnabled(true);
        ui->actionSave_Calibration_File->setEnabled(true);
    }else{
        ui->tableUseMatrix->showColumn(COL_MAXMEASURE);
        ui->tableUseMatrix->showColumn(COL_MINMEASURE);
        ui->tableUseMatrix->showColumn(COL_DIFFMEASURE);
        ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);
        ui->actionImport_Calib_Matrix->setEnabled(false);
        ui->actionLoad_Calibration_File->setEnabled(false);
        ui->actionSave_Calibration_File->setEnabled(false);
    }


    QtConcurrent::run(this,&CalibrationWindow::useMatrix,value);

}

void CalibrationWindow::useMatrix(int index)
{
    mutex.lock();
    loading();
    string msg;
    if(index > 0){
        core->getDownloader()->strain_set_matrix(bus,id,index - 1,&msg);
        appendLogMsg(msg.c_str());
    }
    currentMatrixIndex = index;
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::onSliderZeroChanged(int val)
{
    ui->zeroLbl->setText(QString("%1").arg(val));
}

void CalibrationWindow::onOffsetSliderValue(int val)
{
    QSlider *slider = (QSlider*)sender();
    QLabel *lbl = (QLabel*)((QHBoxLayout*)((QWidget*)slider->parent())->layout())->itemAt(1)->widget();
    lbl->setText(QString("%1").arg(val));


}

void CalibrationWindow::setOffset(int chan, int value)
{
    mutex.lock();
    loading();
    offset[chan] = value;
    string msg;
    core->getDownloader()->strain_set_offset(bus,id, chan, offset[chan],&msg);
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

void CalibrationWindow::onSliderValue(QSlider *slider, int value)
{
    int chan = slider_gain.indexOf(slider);

    if(sliderPressed[chan] || slider->value() == value){
        return;
    }
    slider->setValue(value);

}

void CalibrationWindow::onSetMatrix(int index)
{
    QTableWidget *table = matrices.at(index);

    char tempbuf [250];
    for(int ri=0;ri<CHANNEL_COUNT;ri++){
        for(int ci=0;ci<CHANNEL_COUNT;ci++){
            sprintf(tempbuf,"%x",matrix[index][ri][ci]);
            QTableWidgetItem *item = table->item(ri,ci);
            item->setText(tempbuf);
        }
    }
    connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixChanged(QTableWidgetItem*)),Qt::UniqueConnection);
}

void CalibrationWindow::onTimeout()
{
    qDebug() << "STARTING....";
    keepRunning = true;
    while(keepRunning){
        mutex.lock();
        string msg;

        if(first_time){
            loading();
            ((CustomTreeWidgetItem*)item->getParentNode())->retrieveCanBoards(false);
            for (int i=0;i<((CustomTreeWidgetItem*)item->getParentNode())->getCanBoards().count();i++ ) {
                sBoard b = ((CustomTreeWidgetItem*)item->getParentNode())->getCanBoards().at(i);
                if(b.bus == bus && b.pid == id){
                    selected = i;
                    break;
                }
            }

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
                ret  = core->getDownloader()->strain_get_offset (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, offset[i],&msg);
            }else{
                ret |= core->getDownloader()->strain_get_offset (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, offset[i],&msg);
            }
            appendLogMsg(msg.c_str());
        }
        if (ret!=0){
            qDebug() <<"debug: message 'strain_get_offset' lost.";
        }

        int bUseCalibration = currentMatrixIndex != 0 ? 1 : 0;
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



        if (eeprom_saved_status==false) {
            /*gtk_widget_modify_bg (save_button, GTK_STATE_NORMAL,      &r_color);
                gtk_widget_modify_bg (save_button, GTK_STATE_ACTIVE,      &r_color);
                gtk_widget_modify_bg (save_button, GTK_STATE_PRELIGHT,    &r_color);
                gtk_widget_modify_bg (save_button, GTK_STATE_SELECTED,    &r_color);
                gtk_widget_modify_bg (save_button, GTK_STATE_INSENSITIVE, &r_color);
                gtk_button_set_label     (GTK_BUTTON(save_button), "Not saved.\nSave to eeprom?"); */
        } else {
            //gtk_button_set_label     (GTK_BUTTON(save_button), "Save to eeprom");
        }


        setSerialChanged(serial_number_changed);

        for(int mi=0;mi<MATRIX_COUNT;mi++){
            if (matrix_changed[mi] == false){
                for (ri=0;ri<CHANNEL_COUNT;ri++){
                    for (ci=0;ci<CHANNEL_COUNT;ci++){
                        core->getDownloader()->strain_get_matrix_rc(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, ci, matrix[mi][ri][ci],mi,&msg);
                        appendLogMsg(msg.c_str());
                    }
                }
                setMatrix(mi);
                core->getDownloader()->strain_get_matrix_gain(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, calib_const[mi],mi,&msg);
                appendLogMsg(msg.c_str());
                sprintf(tempbuf,"%d",calib_const[mi]);
                setText(matrixGain.at(mi),tempbuf);

                //                for (ri=0;ri<CHANNEL_COUNT;ri++){
                //                    core->getDownloader()->strain_get_full_scale(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, full_scale_const[ri],&msg);
                //                    appendLogMsg(msg.c_str());
                //                    sprintf(tempbuf,"%d",full_scale_const[ri]);
                //                    QTableWidgetItem *item2 = ui->tableParamters->item(ri,COL_FULLSCALE);
                //                    setText(item2,tempbuf);
                //                }
            }

            for (ri=0;ri<CHANNEL_COUNT;ri++){
                core->getDownloader()->strain_get_full_scale(bus,id, ri, full_scale_const[mi][ri],mi,&msg);
                appendLogMsg(msg.c_str());
                sprintf(tempbuf,"%d",full_scale_const[mi][ri]);
                QTableWidgetItem *item2 = fullScales.at(mi)->item(ri,COL_FULLSCALE);
                setText(item2,tempbuf);
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
            } else {
                maxadc[i]=-32768;
                minadc[i]=+32767;
            }
        }

        bool skip_display_calib=false;

        if(bUseCalibration){
            int currentMatrix;
            core->getDownloader()->strain_get_matrix(bus,id,currentMatrix,&msg);
            ui->comboUseMatrix->blockSignals(true);
            ui->comboUseMatrix->setCurrentIndex(currentMatrix+1);
            ui->comboUseMatrix->blockSignals(false);
        }else{
            ui->comboUseMatrix->blockSignals(true);
            ui->comboUseMatrix->setCurrentIndex(0);
            currentMatrixIndex = 0;
            ui->comboUseMatrix->blockSignals(false);
        }

        if(bUseCalibration)
        {
            ui->tableCurr->horizontalHeaderItem(0)->setText("FT");
        }
        else
        {
            ui->tableCurr->horizontalHeaderItem(0)->setText("ADC");
        }

        for (int i=0;i<CHANNEL_COUNT;i++){

            if(icubCanProto_boardType__strain2 == boardtype)
            {
                core->getDownloader()->strain_get_amplifier_gain_offset(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, amp_gains[i], amp_offsets[i], &msg);
                appendLogMsg(msg.c_str());
                sprintf(tempbuf,"%6.3f",amp_gains[i]);
                QTableWidgetItem *item00 = ui->tableParamters->item(i,COL_GAIN);
                setText(item00,tempbuf);
            }
            else
            {
                amp_gains[i] = 1.0f;
                amp_offsets[i] = 0;
                sprintf(tempbuf,"%s", "N/A");
                QTableWidgetItem *item00 = ui->tableParamters->item(i,COL_GAIN);
                setText(item00, tempbuf);
            }


            core->getDownloader()->strain_get_calib_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, calib_bias[i],&msg);
            appendLogMsg(msg.c_str());
            sprintf(tempbuf,"%d (%d)",showasQ15(calib_bias[i]), showBias(calib_bias[i]));
            QTableWidgetItem *item = ui->tableParamters->item(i,COL_CALIBBIAS);
            //item->setText(tempbuf);
            setText(item,tempbuf);

            core->getDownloader()->strain_get_curr_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, curr_bias[i],&msg);
            appendLogMsg(msg.c_str());
            sprintf(tempbuf,"%d", showasQ15(curr_bias[i]));
            QTableWidgetItem *item1 = ui->tableParamters->item(i,COL_CURRBIAS);
            //item1->setText(tempbuf);
            setText(item1,tempbuf);

//            sprintf (tempbuf,"%d",full_scale_const[i]);
//            QTableWidgetItem *item2 = ui->tableParamters->item(i,COL_FULLSCALE);
//            //item2->setText(tempbuf);
//            setText(item2,tempbuf);

            QSlider *slider = slider_gain.at(i);
            //slider->setValue(offset[i]);
            setSliderValue(slider,offset[i]);

            // marco.accame: we always show the received value in range [-32k, +32k).
            // in case of 0 == bUseCalibration: it is the adc value
            // in case of 1 == bUseCalibration: it is = M * (adc+calibtare) + currtare
            sprintf(tempbuf,"%d",convert_to_signed32k(adc[i]));
            QTableWidgetItem *item3 = ui->tableCurr->item(i,COL_CURRMEASURE);
            //item3->setText(tempbuf);
            setText(item3,tempbuf);

            if(bUseCalibration){
                QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_MAXMEASURE);
                setText(item,"---");

                QTableWidgetItem *item1 = ui->tableUseMatrix->item(i,COL_MINMEASURE);
                setText(item1,"---");

                QTableWidgetItem *item2 = ui->tableUseMatrix->item(i,COL_DIFFMEASURE);
                setText(item2,"---");

                if (full_scale_const[currentMatrixIndex - 1][i]==0){
                    qDebug() << "Error getting the full scale "<< i << " from the sensor";
                    skip_display_calib=true;
                }



                if (skip_display_calib==false){
                    if(i<=2){
                        sprintf(tempbuf,"%+.3f N",(convert_to_signed32k(adc[i]))/float(RANGE32K)*full_scale_const[currentMatrixIndex - 1][i]);
                    }else{
                        sprintf(tempbuf,"%+.3f Nm",(convert_to_signed32k(adc[i]))/float(RANGE32K)*full_scale_const[currentMatrixIndex - 1][i]);
                    }
                    QTableWidgetItem *item3 = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    setText(item3,tempbuf);
                }else{
                    QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    //item->setText("ERROR");
                    setText(item,"ERROR");
                }
            } else {
                sprintf(tempbuf,"%d",maxadc[i]);
                QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_MAXMEASURE);
                setText(item,tempbuf);


                sprintf(tempbuf,"%d",minadc[i]);
                QTableWidgetItem *item1 = ui->tableUseMatrix->item(i,COL_MINMEASURE);
                setText(item1,tempbuf);


                sprintf(tempbuf,"%d",maxadc[i]-minadc[i]);
                QTableWidgetItem *item2 = ui->tableUseMatrix->item(i,COL_DIFFMEASURE);
                setText(item2,tempbuf);


                if(i<=2){
                    QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    setText(item,"--- N");
                }else{
                    QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    setText(item,"--- Nm");
                }
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

bool CalibrationWindow::calibration_load_v2 (char* filename, int selected_bus, int selected_id, int index)
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
        core->getDownloader()->strain_set_offset (bus,id, i, offset[i]);
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
        core->getDownloader()->strain_set_matrix_rc(bus,id, ri, ci, calib_matrix[index][ri][ci]);
    }



    //matrix gain
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    core->getDownloader()->strain_set_matrix_gain(bus,id, cc);

    //tare
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&calib_bias[i]);
        core->getDownloader()->strain_set_calib_bias(bus,id, i, calib_bias[i]);
    }

    //full scale values
    filestr.getline (buffer,256);
    for (i=0;i<CHANNEL_COUNT; i++){
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&full_scale_const[index][i]);
        core->getDownloader()->strain_set_full_scale(bus,id, i, full_scale_const[index][i]);
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

