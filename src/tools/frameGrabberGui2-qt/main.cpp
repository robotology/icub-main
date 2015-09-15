#include "mainwindow.h"
#include <QApplication>

#include <yarp/os/Property.h>
#include <yarp/os/Network.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    QApplication a(argc, argv);

    MainWindow *window = NULL;
    if(argc == 3){
        window = new MainWindow(argv[1],argv[2]);
        window->show();
    }else if (argc<3) {
        printf("usage:\n");
        printf("%s --local <localportname> --remote <grabberport> [--width <gui width>] [--height <gui height>] [--x <gui x pos>] [--y <gui y pos>]\n",argv[0]);
        return 1;
    }else if (argc>3){

        yarp::os::Property options;
        options.fromCommand(argc,argv);

        printf("%s\n",options.toString().c_str());

        // switch to subsections if available
        yarp::os::Searchable *Network = &options.findGroup("NETWORK");
        yarp::os::Searchable *Window = &options.findGroup("WINDOW");
        //yarp::os::Searchable *Program = &options.findGroup("PROGRAM");

        if (Network->isNull()) { Network = &options; }
        if (Window->isNull()) { Window = &options; }
        //if (Program->isNull()) { Program = &options; }

        yarp::os::Value *val;

        char portName[256]="",outPortName[256]="";

        if (Network->check("PortName",val)||Network->check("local",val)){
            strcpy(portName, val->asString().c_str());
        }

        if (Network->check("OutPortName",val)||Network->check("remote",val)){
            strcpy(outPortName, val->asString().c_str());
        }

        //printf("using local=%s remote=%s x=%d y=%d\n",portName,outPortName,posX,posY);
        //fflush(stdout);

        window = new MainWindow(portName,outPortName);
        int posX=0,posY=0;
        int width,height;
        width = window->width();
        height = window->height();

        if (Window->check("PosX",val)||Window->check("x",val)){
            posX = val->asInt();
        }
        if (Window->check("PosY",val)||Window->check("y",val)){
                posY = val->asInt();
        }
        if (Window->check("width",val)){
            width = val->asInt();
        }
        if (Window->check("height",val)){
            height = val->asInt();
        }

        window->resize(width,height);
        window->move(posX,posY);
        window->show();
    }

     int ret = a.exec();
     delete window;

     return (ret!=0?1:0);
}
