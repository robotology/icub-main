#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"
#include <qdebug.h>
#include <QLabel>
#include <QHBoxLayout>
#include <QtConcurrent/QtConcurrent>

#define MAX_COUNT   6
#define HEX_VALC 0x8000

#define     COL_CURRMEASURE     0

#define     COL_OFFSET          0
#define     COL_CALIBBIAS       1
#define     COL_CURRBIAS        2
#define     COL_FULLSCALE       3

#define     COL_MAXMEASURE      0
#define     COL_MINMEASURE      1
#define     COL_DIFFMEASURE     2
#define     COL_NEWTONMEASURE   3


CalibrationWindow::CalibrationWindow(FirmwareUpdaterCore *core, CustomTreeWidgetItem *item, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CalibrationWindow)
{
    ui->setupUi(this);
    setWindowModality(Qt::ApplicationModal);

    this->core = core;
    this->selected = item->getIndexOfBoard();
    this->item = item;
    bus = ((CustomTreeWidgetItem*)item->getParentNode())->getCanBoard(selected).bus;
    id  = ((CustomTreeWidgetItem*)item->getParentNode())->getCanBoard(selected).pid;

    calibration_value=32767;
    calib_const[0] = 0;
    calib_const[1] = 0;
    calib_const[2] = 0;
    serial_number_changed = false;
    matrixA_changed = false;
    matrixB_changed = false;
    matrixC_changed = false;
    eeprom_saved_status=false;
    first_time = true;
    for(int i=0;i<MAX_COUNT;i++){
        ch[i]               = i;
        adc[i]              = 0;
        calib_bias[i]       = 0;
        curr_bias[i]        = 0;
        maxadc[i]           = 0;
        minadc[i]           = 65535;
        full_scale_const[i] = 0;
        sliderPressed[i]    = false;
        amp_gain1[i]        = 0;
        amp_gain2[i]        = 0;
    }
    strncpy(serial_no,"UNDEF",8);





    for (int i=0;i <MAX_COUNT;i++) {
        QWidget *container = new QWidget(ui->tableParamters);
        QSlider *slider = new QSlider(container);
        slider->setMinimum(0);
        slider->setMaximum(0x3FF);
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

        for(int j=0;j<MAX_COUNT;j++){
            QTableWidgetItem *item = new QTableWidgetItem("-");
            ui->matrixA->setItem(i,j,item);

            QTableWidgetItem *item1 = new QTableWidgetItem("-");
            ui->matrixB->setItem(i,j,item1);

            QTableWidgetItem *item2 = new QTableWidgetItem("-");
            ui->matrixC->setItem(i,j,item2);
        }

        QTableWidgetItem *item5 = new QTableWidgetItem("-");
        item5->setFlags(item5->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_CALIBBIAS,item5);


        QTableWidgetItem *item6 = new QTableWidgetItem("-");
        item6->setFlags(item6->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_CURRBIAS,item6);

        QTableWidgetItem *item7 = new QTableWidgetItem("-");
        item7->setFlags(item7->flags() ^ Qt::ItemIsEditable);
        ui->tableParamters->setItem(i,COL_FULLSCALE,item7);
    }

    ui->tableParamters->setColumnWidth(COL_OFFSET,150);
    ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);




    ui->slider_zero->setMinimum(0);
    ui->slider_zero->setMaximum(65535);
    ui->slider_zero->setValue(32767);
    ui->zeroLbl->setText(QString("%1").arg(ui->slider_zero->value()));

    progress = new QProgressBar(this);
    progress->setFixedWidth(100);
    progress->setMinimum(0);
    progress->setMaximum(100);
    progress->setVisible(false);
    ui->statusbar->addWidget(progress);



    connect(ui->comboUseMatrix,SIGNAL(currentIndexChanged(int)),this,SLOT(onUseMatrixChanged(int)));

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
    connect(ui->btnSetSerial,SIGNAL(clicked(bool)),this,SLOT(onSetSerial(bool)));
    connect(this,SIGNAL(setSerialChanged(bool)),this,SLOT(onSetSerialChanged(bool)));
    connect(ui->btnResetCalib,SIGNAL(clicked(bool)),this,SLOT(onResetCalibMatrix(bool)));
    connect(this,SIGNAL(setMatrix(int)),this,SLOT(onSetMatrix(int)),Qt::QueuedConnection);
    connect(ui->actionSave_To_Eproom,SIGNAL(triggered(bool)),this,SLOT(onSaveToEeprom(bool)));
    connect(ui->btnSetCalibration,SIGNAL(clicked(bool)),this,SLOT(onSetCalibration(bool)));
    connect(this,SIGNAL(resetMatrices(int)),this,SLOT(resetMatricesState(int)),Qt::QueuedConnection);
    connect(this,SIGNAL(updateTitle()),this,SLOT(onUpdateTitle()),Qt::QueuedConnection);

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
    core->getDownloader()->strain_set_calib_bias(bus,id);
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::resetCalibBias()
{
    mutex.lock();
    loading();
    core->getDownloader()->strain_reset_calib_bias(bus,id);
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::resetCurrBias()
{
    mutex.lock();
    loading();
    core->getDownloader()->strain_reset_curr_bias(bus,id);
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::setCurrBias()
{
    mutex.lock();
    loading();
    core->getDownloader()->strain_set_curr_bias(bus,id);
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::autoAdjust()
{
    mutex.lock();
    loading();
    core->getDownloader()->strain_calibrate_offset(bus,id,ui->slider_zero->value());
    loading(false);
    mutex.unlock();
}
void CalibrationWindow::setSerial()
{
    mutex.lock();
    loading();
    core->getDownloader()->strain_set_serial_number(bus,id,ui->edit_serial->text().toLatin1().data());
    serial_number_changed = false;
    loading(false);
    mutex.unlock();
}

void CalibrationWindow::saveToEeprom()
{
    mutex.lock();
    loading();
    drv_sleep (1000);
    core->getDownloader()->strain_save_to_eeprom(bus,id);
    drv_sleep (1000);
    resetMatrices();
    loading(false);
    mutex.unlock();
}


void CalibrationWindow::resetMatricesState(int index)
{
    matrixA_changed = false;
    matrixB_changed = false;
    matrixC_changed = false;

    for(int i=0;i<3;i++){
        if(index >=0){
            if(i != index){
                continue;
            }
        }
        QTableWidget *table;
        QLineEdit *gain;
        switch (i) {
        case 0:
            table = ui->matrixA;
            gain = ui->editMatrixGainA;
            break;
        case 1:
            table = ui->matrixB;
            gain = ui->editMatrixGainB;
            break;
        default:
            table = ui->matrixC;
            gain = ui->editMatrixGainC;
            break;
        }
        disconnect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixAChanged(QTableWidgetItem*)));
        for(int r=0;r<MAX_COUNT;r++){
            for(int c=0;c<MAX_COUNT;c++){
                table->item(r,c)->setBackgroundColor("white");
            }
        }
        connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixAChanged(QTableWidgetItem*)),Qt::UniqueConnection);
        gain->setStyleSheet("");
    }
}

void CalibrationWindow::setCalibration()
{
    mutex.lock();
    int index = ui->tabWidget->currentIndex();
    QTableWidget *table;
    QLineEdit *gain;
    switch (index) {
    case 0:
        table = ui->matrixA;
        gain = ui->editMatrixGainA;
        break;
    case 1:
        table = ui->matrixB;
        gain = ui->editMatrixGainB;
        break;
    default:
        table = ui->matrixC;
        gain = ui->editMatrixGainC;
        break;
    }

    loading();
    for (int ri=0; ri<MAX_COUNT; ri++){
        for (int ci=0; ci<MAX_COUNT; ci++){
            const char* temp2 = table->item(ri,ci)->text().toLatin1().data();
            sscanf (temp2,"%x",&matrix[index][ri][ci]);
            core->getDownloader()->strain_set_matrix_rc(bus,id,ri,ci,matrix[index][ri][ci]);
        }
    }

    core->getDownloader()->strain_set_matrix_gain(bus,id,calib_const[index]);
    qDebug("Calibration matrix updated");

    resetMatrices(index);

    loading(false);
    mutex.unlock();
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
    keepRunning = false;
    QMainWindow::closeEvent(e);
}

void CalibrationWindow::onMatrixAChanged(QTableWidgetItem *item)
{
    Q_UNUSED(item);
    if(first_time){
        return;
    }
    if(item->text() == "-"){
        return;
    }
    matrixA_changed = true;
    for(int i=0;i<MAX_COUNT;i++){
        for(int j=0;j<MAX_COUNT;j++){
            ui->matrixA->item(i,j)->setBackgroundColor("red");
        }
    }
}

void CalibrationWindow::onMatrixBChanged(QTableWidgetItem *item)
{
    Q_UNUSED(item);
    if(first_time){
        return;
    }
    if(item->text() == "-"){
        return;
    }
    matrixB_changed = true;
    for(int i=0;i<MAX_COUNT;i++){
        for(int j=0;j<MAX_COUNT;j++){
            ui->matrixB->item(i,j)->setBackgroundColor("red");
        }
    }
}

void CalibrationWindow::onMatrixCChanged(QTableWidgetItem *item)
{
    Q_UNUSED(item);
    if(first_time){
        return;
    }
    if(item->text() == "-"){
        return;
    }
    matrixC_changed = true;
    for(int i=0;i<MAX_COUNT;i++){
        for(int j=0;j<MAX_COUNT;j++){
            ui->matrixC->item(i,j)->setBackgroundColor("red");
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

void CalibrationWindow::onResetCalibMatrix(bool click)
{
    mutex.lock();
    int index = ui->tabWidget->currentIndex();
    QTableWidget *table = NULL;
    QLineEdit *editGain;
    bool *changed;
    switch (index) {
    case 0:
        table = ui->matrixA;
        editGain = ui->editMatrixGainA;
        changed = &matrixA_changed;
        break;
    case 1:
        table = ui->matrixB;
        editGain = ui->editMatrixGainB;
        changed = &matrixB_changed;
        break;
    default:
        table = ui->matrixC;
        editGain = ui->editMatrixGainC;
        changed = &matrixC_changed;
        break;
    }

    for (int ri=0; ri<6; ri++){
        for (int ci=0; ci<6; ci++){
            if (ri==ci){
                matrix[index][ri][ci] = 32767;
                //table->item(ri,ci)->setText("7fff");
            } else {
                matrix[index][ri][ci] = 0;
                //table->item(ri,ci)->setText("0");
            }
        }
    }
    calib_const[index]=1;
    editGain->setText(QString("%1").arg(calib_const[index]));
    setMatrix(index);
    *changed = true;
    mutex.unlock();

}


void CalibrationWindow::onUseMatrixChanged(int value)
{
    if(value != 0){
        ui->tableUseMatrix->hideColumn(COL_MAXMEASURE);
        ui->tableUseMatrix->hideColumn(COL_MINMEASURE);
        ui->tableUseMatrix->hideColumn(COL_DIFFMEASURE);
        ui->tableUseMatrix->showColumn(COL_NEWTONMEASURE);
    }else{
        ui->tableUseMatrix->showColumn(COL_MAXMEASURE);
        ui->tableUseMatrix->showColumn(COL_MINMEASURE);
        ui->tableUseMatrix->showColumn(COL_DIFFMEASURE);
        ui->tableUseMatrix->hideColumn(COL_NEWTONMEASURE);
    }
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
    core->getDownloader()->strain_set_offset(bus,id, chan, offset[chan]);
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
    QTableWidget *table = NULL;
    switch (index) {
    case 0:
        table = ui->matrixA;
        break;
    case 1:
        table = ui->matrixB;
        break;
    default:
        table = ui->matrixC;
        break;
    }
    char tempbuf [250];
    for(int ri=0;ri<MAX_COUNT;ri++){
        for(int ci=0;ci<MAX_COUNT;ci++){
            sprintf(tempbuf,"%x",matrix[index][ri][ci]);
            QTableWidgetItem *item = table->item(ri,ci);
            item->setText(tempbuf);
        }
    }
    connect(table,SIGNAL(itemChanged(QTableWidgetItem*)), this,SLOT( onMatrixAChanged(QTableWidgetItem*)),Qt::UniqueConnection);




}

void CalibrationWindow::onTimeout()
{
    qDebug() << "STARTING....";
    keepRunning = true;
    while(keepRunning){
        mutex.lock();


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
                                                            core->getDownloader()->board_list[selected].pid, serial_no);
            //ui->edit_serial->setText(serial_no);
            setText(ui->edit_serial,serial_no);
        }
        int ret=0;
        ret = core->getDownloader()->strain_get_eeprom_saved(core->getDownloader()->board_list[selected].bus,
                                                             core->getDownloader()->board_list[selected].pid,
                                                             &eeprom_saved_status);
        if (ret!=0){
            qDebug() << "debug: message 'strain_get_eeprom_saved' lost";
        }
        updateTitle();

        for (int i=0;i<MAX_COUNT;i++){
            if(i==0){
                ret  = core->getDownloader()->strain_get_offset (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, offset[i]);
            }else{
                ret |= core->getDownloader()->strain_get_offset (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, offset[i]);
            }
        }
        if (ret!=0){
            qDebug() <<"debug: message 'strain_get_offset' lost.";
        }

        int bool_raw = ui->comboUseMatrix->currentIndex() != 0 ? 1 : 0;
        for (int i=0;i<MAX_COUNT;i++){
            if(i==0){
                ret  = core->getDownloader()->strain_get_adc (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, adc[i], bool_raw);
            }else{
                ret |= core->getDownloader()->strain_get_adc (core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, adc[i], bool_raw);
            }
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

        if (matrixA_changed == false){
            for (ri=0;ri<MAX_COUNT;ri++){
                for (ci=0;ci<MAX_COUNT;ci++){
                    core->getDownloader()->strain_get_matrix_rc(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, ci, matrix[0][ri][ci]);
                }
            }
            setMatrix(0);
            core->getDownloader()->strain_get_matrix_gain(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, calib_const[0]);
            sprintf(tempbuf,"%d",calib_const[0]);
            setText(ui->editMatrixGainA,tempbuf);

            for (ri=0;ri<MAX_COUNT;ri++){
                core->getDownloader()->strain_get_full_scale(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, full_scale_const[ri]);
                sprintf(tempbuf,"%d",full_scale_const[ri]);
                QTableWidgetItem *item2 = ui->tableParamters->item(ri,COL_FULLSCALE);
                setText(item2,tempbuf);
            }
        } /*else {
            for (ri=0;ri<MAX_COUNT;ri++){
                for (ci=0;ci<MAX_COUNT;ci++){
                    //gtk_widget_modify_base (edit_matrix[ri][ci],GTK_STATE_NORMAL, &r_color );
                }
            }
        }*/

        for (int i=0;i<MAX_COUNT;i++){
            if (!bool_raw){
                if(adc[i]>maxadc[i]){
                    maxadc[i]=adc[i];
                }
                if (adc[i]<minadc[i]){
                    minadc[i]=adc[i];
                }
            } else {
                maxadc[i]=0;
                minadc[i]=65535;
            }
        }

        bool skip_display_calib=false;
        for (int i=0;i<MAX_COUNT;i++){
            core->getDownloader()->strain_get_calib_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, calib_bias[i]);
            sprintf(tempbuf,"%d",calib_bias[i]);
            QTableWidgetItem *item = ui->tableParamters->item(i,COL_CALIBBIAS);
            //item->setText(tempbuf);
            setText(item,tempbuf);

            core->getDownloader()->strain_get_curr_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, curr_bias[i]);
            sprintf(tempbuf,"%d",curr_bias[i]);
            QTableWidgetItem *item1 = ui->tableParamters->item(i,COL_CURRBIAS);
            //item1->setText(tempbuf);
            setText(item1,tempbuf);

            sprintf (tempbuf,"%d",full_scale_const[i]);
            QTableWidgetItem *item2 = ui->tableParamters->item(i,COL_FULLSCALE);
            //item2->setText(tempbuf);
            setText(item2,tempbuf);

            QSlider *slider = slider_gain.at(i);
            //slider->setValue(offset[i]);
            setSliderValue(slider,offset[i]);

            sprintf(tempbuf,"%d",adc[i]-HEX_VALC);
            QTableWidgetItem *item3 = ui->tableCurr->item(i,COL_CURRMEASURE);
            //item3->setText(tempbuf);
            setText(item3,tempbuf);

            if(bool_raw){
                QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_MAXMEASURE);
                //item->setText("---");
                setText(item,"---");

                QTableWidgetItem *item1 = ui->tableUseMatrix->item(i,COL_MINMEASURE);
                //item1->setText("---");
                setText(item1,"---");

                QTableWidgetItem *item2 = ui->tableUseMatrix->item(i,COL_DIFFMEASURE);
                //item2->setText("---");
                setText(item2,"---");

                if (full_scale_const[i]==0){
                    qDebug() << "Error getting the full scale "<< i << " from the sensor";
                    skip_display_calib=true;
                }

                if (skip_display_calib==false){
                    if(i<=2){
                        sprintf(tempbuf,"%+.3f N",(int(adc[i])-HEX_VALC)/float(HEX_VALC)*full_scale_const[i]);
                    }else{
                        sprintf(tempbuf,"%+.3f N/m",(int(adc[i])-HEX_VALC)/float(HEX_VALC)*full_scale_const[i]);
                    }
                    QTableWidgetItem *item3 = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    //item3->setText(tempbuf);
                    setText(item3,tempbuf);
                }else{
                    QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    //item->setText("ERROR");
                    setText(item,"ERROR");
                }
            } else {
                sprintf(tempbuf,"%d",maxadc[i]);
                QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_MAXMEASURE);
                //item->setText(tempbuf);
                setText(item,tempbuf);


                sprintf(tempbuf,"%d",minadc[i]);
                QTableWidgetItem *item1 = ui->tableUseMatrix->item(i,COL_MINMEASURE);
                //item1->setText(tempbuf);
                setText(item1,tempbuf);


                sprintf(tempbuf,"%d",maxadc[i]-minadc[i]);
                QTableWidgetItem *item2 = ui->tableUseMatrix->item(i,COL_DIFFMEASURE);
                //item2->setText(tempbuf);
                setText(item2,tempbuf);


                if(i<=2){
                    QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    //item->setText("--- N");
                    setText(item,"--- N");
                }else{
                    QTableWidgetItem *item = ui->tableUseMatrix->item(i,COL_NEWTONMEASURE);
                    setText(item,"--- N/m");
                    //item->setText("--- N/m");
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

