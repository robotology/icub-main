#ifndef SUBTITLESTHREAD_H
#define SUBTITLESTHREAD_H

#include <qthread.h>
#include <yarp/os/BufferedPort.h>
#include <qgl.h>


#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <string.h>
extern std::string GUI_NAME;

class SubtitlesManager
{
    yarp::os::BufferedPort<yarp::os::Bottle> mTxtMsgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> mTxtDbgPort;

public:
    SubtitlesManager(const char *objPortName,const char *texPortName)
    {
        mTxtMsgPort.open((GUI_NAME+objPortName).c_str());
        mTxtDbgPort.open((GUI_NAME+texPortName).c_str());

        mTxtMsgPort.setStrict();
        mTxtDbgPort.setStrict();

        /*
        mPx=mPy=mPz=0.0;
        mRx=mRy=mRz=0.0;
        for (int i=0; i<3; ++i)
            for (int j=0; j<3; ++j)
                R[i][j]=(double)(i==j);
        */
    }

    ~SubtitlesManager()
    {
        mTxtMsgPort.interrupt();
        mTxtMsgPort.close();
        
        mTxtDbgPort.interrupt();
        mTxtDbgPort.close();
    }

    void draw()
    {
        static double txtWatchDogStart = yarp::os::Time::now();
        static double dbgWatchDogStart = yarp::os::Time::now();
        static std::string txtMsg;
        static std::string dbgMsg;

        Bottle* msg = mTxtMsgPort.read(false);
        Bottle* dbg = mTxtDbgPort.read(false);

        if (msg !=0)
        {
            txtMsg= msg->get(0).asString().c_str();
            txtWatchDogStart = yarp::os::Time::now();
        }
        if (dbg !=0)
        {
            dbgMsg = dbg->get(0).asString().c_str();
            dbgWatchDogStart = yarp::os::Time::now();
        }

        if (yarp::os::Time::now()-txtWatchDogStart < 3.0 && !txtMsg.empty())
        {
            drawText("iCub: ",txtMsg.c_str(),1.0f,1.0f,1.0f);
        }

        if (yarp::os::Time::now()-dbgWatchDogStart < 3.0 && !dbgMsg.empty())
        {
            drawText("Dbg: ",dbgMsg.c_str(),1.0f,1.0f,1.0f);
        }

    }

    void drawText(const char* speaker, const char* text, float r, float g, float b)
    {
        glLoadIdentity();
        glColor3f(r, g, b);
        glTranslatef(-4.0f, -2.5f, -5.0f);
        glRasterPos2i(0,0);

        glDisable(GL_LIGHTING);
        //glRasterPos3f(0,0,0);

        static char tmp[1024];
        strcpy (tmp,speaker);
        strcat (tmp,text);
        for (int i=0; tmp[i]!='\0'; ++i)
        {
            glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,tmp[i]);
        }
        glEnable(GL_LIGHTING);
    }

};


#endif


