#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"
#include <qdebug.h>
#include <QLabel>
#include <QHBoxLayout>

#define MAX_COUNT   6
#define HEX_VALC 0x8000

#define     COL_OFFSET          0
#define     COL_CURRMEASURE     1
#define     COL_MAXMEASURE      2
#define     COL_MINMEASURE      3
#define     COL_DIFFMEASURE     4
#define     COL_NEWTONMEASURE   5
#define     COL_CALIBBIAS       6
#define     COL_CURRBIAS        7

CalibrationWindow::CalibrationWindow(FirmwareUpdaterCore *core, int selected, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CalibrationWindow)
{
    this->core = core;
    this->selected = selected;
    ui->setupUi(this);
    calibration_value=32767;
    calib_const = 0;
    serial_number_changed = false;
    matrix_changed = false;

    for(int i=0;i<MAX_COUNT;i++){
        ch[i]               = i;
        adc[i]              = 0;
        calib_bias[i]       = 0;
        curr_bias[i]        = 0;
        maxadc[i]           = 0;
        minadc[i]           = 65535;
        full_scale_const[i] = 0;
        first_time[i]       = true;
        amp_gain1[i]        = 0;
        amp_gain2[i]        = 0;
    }
    strncpy(serial_no,"UNDEF",8);





    for (int i=0;i <MAX_COUNT;i++) {
        QWidget *container = new QWidget(ui->tableWidget);
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
        ui->tableWidget->setCellWidget(i,COL_OFFSET,container);


        connect(slider,SIGNAL(valueChanged(int)),this,SLOT(onOffsetSliderValue(int)));

        QLineEdit *lineEdit = new QLineEdit(ui->tableWidget);
        lineEdit->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_CURRMEASURE,lineEdit);
        curr_measure.append(lineEdit);

        QLineEdit *lineEdit1 = new QLineEdit(ui->tableWidget);
        lineEdit1->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_MAXMEASURE,lineEdit1);
        max_measure.append(lineEdit1);

        QLineEdit *lineEdit2 = new QLineEdit(ui->tableWidget);
        lineEdit2->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_MINMEASURE,lineEdit2);
        min_measure.append(lineEdit2);

        QLineEdit *lineEdit3 = new QLineEdit(ui->tableWidget);
        lineEdit3->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_DIFFMEASURE,lineEdit3);
        diff_measure.append(lineEdit3);

        QLineEdit *lineEdit4 = new QLineEdit(ui->tableWidget);
        lineEdit4->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_NEWTONMEASURE,lineEdit4);
        newton_measure.append(lineEdit4);

        for(int j=0;j<MAX_COUNT;j++){
            QLineEdit *lineEdit5 = new QLineEdit(ui->matrix);
            ui->matrix->setCellWidget(i,j,lineEdit5);
        }

        QLineEdit *lineEdit6 = new QLineEdit(ui->tableWidget);
        lineEdit6->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_CALIBBIAS,lineEdit6);
        calibbias.append(lineEdit6);

        QLineEdit *lineEdit7 = new QLineEdit(ui->tableWidget);
        lineEdit7->setReadOnly(true);
        ui->tableWidget->setCellWidget(i,COL_CURRBIAS,lineEdit7);
        currbias.append(lineEdit7);
    }

    ui->tableWidget->setColumnWidth(COL_OFFSET,200);
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->verticalHeader()->setDefaultSectionSize(40);

    ui->slider_zero->setMinimum(0);
    ui->slider_zero->setMaximum(65535);



    timer.setInterval(500);
    timer.setSingleShot(false);
    connect(&timer,SIGNAL(timeout()),this,SLOT(onTimeout()));
    timer.start();

}

CalibrationWindow::~CalibrationWindow()
{
    timer.stop();
    mutex.lock();
    delete ui;
    mutex.unlock();
}


void CalibrationWindow::onOffsetSliderValue(int val)
{
    QSlider *slider = (QSlider*)sender();
    QLabel *lbl = (QLabel*)((QHBoxLayout*)((QWidget*)slider->parent())->layout())->itemAt(1)->widget();
    lbl->setText(QString("%1").arg(val));
}

void CalibrationWindow::onTimeout()
{
    if(mutex.tryLock()){
        int ret=0;
        ret = core->getDownloader()->strain_get_eeprom_saved(core->getDownloader()->board_list[selected].bus,
                                                  core->getDownloader()->board_list[selected].pid,
                                                  &eeprom_saved_status);
        if (ret!=0){
            qDebug() << "debug: message 'strain_get_eeprom_saved' lost";
        }

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

        int bool_raw = ui->check_raw_vals->isChecked();
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
        QColor r_color,y_color;
        r_color.setRed(65535/256);
        r_color.setGreen(39000/256);
        r_color.setBlue(39000/256);
        y_color.setRed(65535/256);
        y_color.setGreen(65535/256);
        y_color.setBlue(38000/256);

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

        if (serial_number_changed==false) {
            //gtk_widget_modify_base (edit_serial_number,GTK_STATE_NORMAL, NULL );
        } else  {
            //gtk_widget_modify_base (edit_serial_number,GTK_STATE_NORMAL, &r_color );
        }

        if (matrix_changed==false){
            for (ri=0;ri<MAX_COUNT;ri++){
                for (ci=0;ci<MAX_COUNT;ci++){
                    core->getDownloader()->strain_get_matrix_rc(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, ci, matrix[ri][ci]);
                    sprintf(tempbuf,"%x",matrix[ri][ci]);
                    ((QLineEdit*)ui->matrix->cellWidget(ri,ci))->setText(tempbuf);
                }
            }
            core->getDownloader()->strain_get_matrix_gain(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, calib_const);
            sprintf(tempbuf,"%d",calib_const);
            ui->editMatrixGain->setText(tempbuf);

            for (ri=0;ri<MAX_COUNT;ri++){
                core->getDownloader()->strain_get_full_scale(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, ri, full_scale_const[ri]);
                sprintf(tempbuf,"%d",full_scale_const[ri]);
                //gtk_label_set_text (GTK_LABEL(full_scale_label[ri]), tempbuf);
            }
        } else {
            for (ri=0;ri<MAX_COUNT;ri++){
                for (ci=0;ci<MAX_COUNT;ci++){
                    //gtk_widget_modify_base (edit_matrix[ri][ci],GTK_STATE_NORMAL, &r_color );
                }
            }
        }

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

        for (int i=0;i<MAX_COUNT;i++){
            core->getDownloader()->strain_get_calib_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, calib_bias[i]);
            sprintf(tempbuf,"%d",calib_bias[i]);
            calibbias.at(i)->setText(tempbuf);

            core->getDownloader()->strain_get_curr_bias(core->getDownloader()->board_list[selected].bus, core->getDownloader()->board_list[selected].pid, i, curr_bias[i]);
            sprintf(tempbuf,"%d",curr_bias[i]);
            currbias.at(i)->setText(tempbuf);
        }

        for (int i=0; i<slider_gain.count();i++) {
            QSlider *slider = slider_gain.at(i);
            slider->setValue(offset[i]);
        }

        for (int i=0; i<curr_measure.count();i++) {
            sprintf(tempbuf,"%d",adc[i]-HEX_VALC);
            QLineEdit *edit = curr_measure.at(i);
            edit->setText(tempbuf);
        }

        if(bool_raw){
            for (int i=0; i<max_measure.count();i++) {
                QLineEdit *edit = max_measure.at(i);
                edit->setText("---");
            }

            for (int i=0; i<min_measure.count();i++) {
                QLineEdit *edit = min_measure.at(i);
                edit->setText("---");
            }

            for (int i=0; i<diff_measure.count();i++) {
                QLineEdit *edit = diff_measure.at(i);
                edit->setText("---");
            }

            bool skip_display_calib=false;
            for (int i=0; i<MAX_COUNT; i++){
                if (full_scale_const[i]==0){
                    qDebug() << "Error getting the full scale "<< i << " from the sensor";
                    skip_display_calib=true;
                }
            }
            if (skip_display_calib==false){
                for (int i=0; i<newton_measure.count();i++) {
                    QLineEdit *edit = newton_measure.at(i);
                    if(i<=2){
                        sprintf(tempbuf,"%+.3f N",(int(adc[i])-HEX_VALC)/float(HEX_VALC)*full_scale_const[i]);
                        edit->setText(tempbuf);
                    }else{
                        sprintf(tempbuf,"%+.3f N/m",(int(adc[i])-HEX_VALC)/float(HEX_VALC)*full_scale_const[i]);
                        edit->setText(tempbuf);
                    }
                }
            }else{
                for (int i=0; i<newton_measure.count();i++) {
                    QLineEdit *edit = newton_measure.at(i);
                    edit->setText("ERROR");
                }
            }


        }else{
            for (int i=0; i<max_measure.count();i++) {
                sprintf(tempbuf,"%d",maxadc[i]);
                QLineEdit *edit = max_measure.at(i);
                edit->setText(tempbuf);
            }

            for (int i=0; i<min_measure.count();i++) {
                sprintf(tempbuf,"%d",minadc[i]);
                QLineEdit *edit = min_measure.at(i);
                edit->setText(tempbuf);
            }

            for (int i=0; i<diff_measure.count();i++) {
                sprintf(tempbuf,"%d",maxadc[i]-minadc[i]);
                QLineEdit *edit = diff_measure.at(i);
                edit->setText(tempbuf);
            }

            for (int i=0; i<newton_measure.count();i++) {
                QLineEdit *edit = newton_measure.at(i);
                if(i<=2){
                    edit->setText("--- N");
                }else{
                    edit->setText("--- N/m");
                }
            }
        }
        mutex.unlock();

    }

}
