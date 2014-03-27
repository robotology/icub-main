/*
* animationview.h
*/

/*
* Copyright (C) 2009 RobotCub Consortium
* Author: Alessandro Scalzo alessandro.scalzo@iit.it
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* Based on:
*
*   Qavimator
*   Copyright (C) 2006 by Zi Ree   *
*   Zi Ree @ SecondLife   *
*   Released under the terms of the GNU GPL v2.0.
*/

#ifndef ANIMATIONVIEW_H
#define ANIMATIONVIEW_H

#include <qgl.h>
#include <qtimer.h>

#include "camera.h"
#include "bvh.h"
#include "objectsthread.h"
#include "subtitilessthread.h"

//#include "rotation.h"
//#include "prop.h"

// defines where we start counting opengl ids for parts with multiple animations
// first animation counts 0-ANIMATION_INCREMENT-1, next ANIMATION_INCREMENT++
#define ANIMATION_INCREMENT 100

#define OBJECT_START      8000

#define DRAG_HANDLE_START OBJECT_START+1000
#define DRAG_HANDLE_X     DRAG_HANDLE_START
#define DRAG_HANDLE_Y     DRAG_HANDLE_START+1
#define DRAG_HANDLE_Z     DRAG_HANDLE_START+2
#define SCALE_HANDLE_X    DRAG_HANDLE_START+3
#define SCALE_HANDLE_Y    DRAG_HANDLE_START+4
#define SCALE_HANDLE_Z    DRAG_HANDLE_START+5
#define ROTATE_HANDLE_X   DRAG_HANDLE_START+6
#define ROTATE_HANDLE_Y   DRAG_HANDLE_START+7
#define ROTATE_HANDLE_Z   DRAG_HANDLE_START+8

class QMouseEvent;

class AnimationView : public QGLWidget
{
    Q_OBJECT

public:
    AnimationView(QWidget* parent,yarp::os::ResourceFinder& config);
    ~AnimationView();

    // This function clears the animations
    void clear();

    void startTimer(int msec)
    {
        mTimer.start(msec); 
    }
    void stopTimer()
    {
        mTimer.stop();
    }

    QStringList& partNames(){ return pBVH->partNames; }

signals:
    void backgroundClicked();

    public slots:
        void resetCamera();
        void timerTimeout(){ repaint(); }

        protected slots:
            void draw();

protected:
    //enum { MODE_PARTS, MODE_SKELETON, MODE_ROT_AXES };

    virtual void paintGL();
    virtual void paintOverlayGL();
    virtual void initializeGL();
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseDoubleClickEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    virtual void resizeEvent(QResizeEvent* newSize);

    // remove annyoing debug message from QT (only on windows)
    // Checked on qt3: on Windows updateOverlayGL() does nothing
    // but prints an annooying debug message, looks safe to remove. 
    // On Linux updateOverlayGL() calls other functions so it is 
    // better to keep calling it.
    // WARNING: check carefully againe in case of porting to Qt4
    virtual void updateOverlayGL ()
    {
        #ifdef WIN32
            //do nothing
        #else
            QGLWidget::updateOverlayGL();
        #endif
    }

    void drawFloor();

    bool leftMouseButton;
    char modifier;

    bool m_bInitialized;

    QTimer mTimer;
    BVH* pBVH;
    ObjectsManager* mObjectsManager;
    SubtitlesManager* mSubtitlesManager; 

    QPoint clickPos;           // holds the mouse click position for dragging
    QPoint returnPos;          // holds the mouse position to return to after dragging

    Camera camera;
    double changeX, changeY, changeZ;

    int dragX, dragY;           // holds the last mouse drag offset
    int oldDragX, oldDragY;     // holds the mouse position before the last drag

    int drawMode;
    bool xSelect, ySelect, zSelect;

    void setProjection();
    void setModelView();

    void clearSelected();
    void drawCircle(int axis, float radius, int width);    

};

#endif


