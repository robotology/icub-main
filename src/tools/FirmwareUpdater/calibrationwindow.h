#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QLineEdit>
#include <QTimer>
#include <downloader.h>
#include "firmwareupdatercore.h"
#include <QTableWidgetItem>
#include <QProgressBar>
#include "customtreewidgetitem.h"

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CalibrationWindow(FirmwareUpdaterCore *core, icubCanProto_boardType_t b, CustomTreeWidgetItem *item, QWidget *parent = 0);
    ~CalibrationWindow();


protected:
    void closeEvent(QCloseEvent *e);
    void setCalibBias();
    void resetCalibBias();
    void setCurrBias();
    void resetCurrBias();
    void autoAdjust();
    void setSerial();
    void saveToEeprom();
    void setOffset(int chan, int value);
    void setCalibration();
    void loadCalibrationFile(QString fileName);
    void saveCalibrationFile(QString filePath);
    void importCalibrationFile(QString fileName);
    bool calibration_load_v2 (char* filename, int selected_bus, int selected_id, int index);
    void useMatrix(int);
    void resetCalibration();

private:
    int currentMatrixIndex;
    Ui::CalibrationWindow *ui;
    CustomTreeWidgetItem *item;
    FirmwareUpdaterCore *core;
    icubCanProto_boardType_t boardtype;
    bool sliderPressed[6];
    unsigned int adc[6];
    signed int maxadc[6];
    signed int minadc[6];
    bool first_time;
    unsigned int offset[6];
    unsigned int amp_gain1[6];
    unsigned int amp_gain2[6];
    int ch[6];
    int calib_bias[6];
    int curr_bias[6];
    uint16_t amp_offsets[6];
    float amp_gains[6];
    unsigned int matrix[3][6][6];
    unsigned int calib_matrix[3][6][6];
    unsigned int calibration_value;
    unsigned int calib_const[3];
    unsigned int full_scale_const[3][6];
    char serial_no[8];
    int selected;
    bool matrix_changed[3];
    bool serial_number_changed;
    bool something_changed;
    bool eeprom_saved_status;
    QList<QSlider*>slider_gain;
    QList<QLineEdit*>matrixGain;
    QList<QTableWidget*>matrices;
    QList<QTableWidget*>fullScales;
    QTimer *timer;
    int bus;
    int id;
    QMutex mutex;
    QMutex sliderMutex;
    QProgressBar *progress;
    bool keepRunning;
private slots:
    void onTabMatrixCahnged(int);
    void onTimeout();
    void onSliderZeroChanged(int);
    void onOffsetSliderValue(int);
    void onMatrixChanged(QTableWidgetItem*);
    void onSerialChanged(QString);
    void onLoading(bool);
    void onSetText(QLineEdit*,QString text);
    void onSetText(QTableWidgetItem*,QString text);
    void onSliderValue(QSlider*, int value);
    void onSetCalibBias(bool click);
    void onResetCalibBias(bool click);
    void onSetCurrBias(bool click);
    void onResetCurrBias(bool click);
    void onAutoAdjust(bool click);
    void onSetSerial(bool click);
    void onSetSerialChanged(bool);
    void onResetCalibMatrix(bool);
    void onUseMatrixChanged(int);
    void onSetMatrix(int index);
    void onSaveToEeprom(bool click);
    void onSetCalibration(bool click);
    void onLoadCalibrationFile(bool click);
    void onSaveCalibrationFile(bool click);
    void onImportCalibrationFile(bool click);
    void resetMatricesState(int index = -1);
    void onUpdateTitle();
    void onSliderPressed();
    void onSliderReleased();
    void onClearLog();
    void onAppendLogMsg(QString);
signals:
    void loading(bool = true);
    void setText(QLineEdit*,QString text);
    void setText(QTableWidgetItem*,QString text);
    void setSliderValue(QSlider*, int value);
    void setSerialChanged(bool);
    void setMatrix(int index);
    void resetMatrices(int index = -1);
    void updateTitle();
    void appendLogMsg(QString);



};

#endif // CALIBRATIONWINDOW_H
