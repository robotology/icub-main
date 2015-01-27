#ifndef DC1394SLIDERBASE_H
#define DC1394SLIDERBASE_H

#include <QWidget>
#include <mainwindow.h>
#include <yarp/dev/RemoteFrameGrabberDC1394.h>

typedef enum {
  DC1394_FEATURE_BRIGHTNESS=0,
  DC1394_FEATURE_EXPOSURE,
  DC1394_FEATURE_SHARPNESS,
  DC1394_FEATURE_WHITE_BALANCE,
  DC1394_FEATURE_HUE,
  DC1394_FEATURE_SATURATION,
  DC1394_FEATURE_GAMMA,
  DC1394_FEATURE_SHUTTER,
  DC1394_FEATURE_GAIN,
  DC1394_FEATURE_IRIS,
  DC1394_FEATURE_FOCUS,
  DC1394_FEATURE_TEMPERATURE,
  DC1394_FEATURE_TRIGGER,
  DC1394_FEATURE_TRIGGER_DELAY,
  DC1394_FEATURE_WHITE_SHADING,
  DC1394_FEATURE_FRAME_RATE,
  DC1394_FEATURE_ZOOM,
  DC1394_FEATURE_PAN,
  DC1394_FEATURE_TILT,
  DC1394_FEATURE_OPTICAL_FILTER,
  DC1394_FEATURE_CAPTURE_SIZE,
  DC1394_FEATURE_CAPTURE_QUALITY
} dc1394feature_id_t;

#define SLIDER      0
#define SLIDERWB    1

class DC1394SliderBase : public QWidget
{
    Q_OBJECT

public:
    DC1394SliderBase(QWidget *parent) : QWidget(parent){}
    virtual ~DC1394SliderBase(){}
    static int GetHeight(){ return m_Height; }
    virtual void Refresh()=0;
    virtual void Propagate()=0;
    virtual void updateSliders()=0;
    int getSliderType(){return type;}
protected:
    static int m_Height;
    bool m_bInactive;
    int type;


signals:
    void reload();
    void featureDisabled(QObject*);
};



#endif // DC1394SLIDERBASE_H
