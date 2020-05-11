#ifndef DC1394SLIDERBASE_H
#define DC1394SLIDERBASE_H

#include <QWidget>
#include <mainwindow.h>
#include <yarp/dev/FrameGrabberInterfaces.h>


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
