#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <QDialog>
#include <QSlider>
#include <QLineEdit>
#include <QTimer>
#include <downloader.h>
#include "firmwareupdatercore.h"

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrationWindow(FirmwareUpdaterCore *core, int selected, QWidget *parent = 0);
    ~CalibrationWindow();

private:
    Ui::CalibrationWindow *ui;
    FirmwareUpdaterCore *core;
    unsigned int adc[6];
    unsigned int maxadc[6];
    unsigned int minadc[6];
    bool first_time[6];
    unsigned int offset[6];
    unsigned int amp_gain1[6];
    unsigned int amp_gain2[6];
    int ch[6];
    int calib_bias[6];
    int curr_bias[6];
    unsigned int matrix[6][6];
    unsigned int calib_matrix[6][6];
    unsigned int calibration_value;
    unsigned int calib_const;
    unsigned int full_scale_const[6];
    char serial_no[8];
    int selected;
    bool matrix_changed;
    bool serial_number_changed;
    bool something_changed;
    bool eeprom_saved_status;

    QList<QLineEdit*> calibbias;
    QList<QLineEdit*> currbias;
    QList<QLineEdit*> max_measure;
    QList<QLineEdit*> min_measure;
    QList<QLineEdit*> diff_measure;
    QList<QLineEdit*> newton_measure;
    QList<QLineEdit*>curr_measure;
    QList<QSlider*>slider_gain;
    QTimer timer;

    QMutex mutex;

private slots:
    void onTimeout();
    void onOffsetSliderValue(int);

};

#endif // CALIBRATIONWINDOW_H
