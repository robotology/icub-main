/*
 * main.cpp
 */

#include "qavimator.h"
#include <qapplication.h>

int main( int argc, char ** argv )
{
    QApplication a(argc,argv);
    qavimator* mw=new qavimator();
    mw->show();
    a.connect(&a,SIGNAL(lastWindowClosed()),&a,SLOT(quit()));
    return a.exec();
}
