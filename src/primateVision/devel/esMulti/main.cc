
/*esLog project*/

#include <stdio.h>
#include <string>
//#include <iostream>
#include <qapplication.h>
#include <q3vbox.h>

#include "world3d.h"
#include "proc.h"


using namespace std;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  QObject::connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));

  //Create 3D World widget in a blank window:
  Q3VBox *mainwin = new Q3VBox;
  mainwin->setCaption( "EgoSphere World" );
  world3d* world = new world3d(&a,mainwin);
  usleep(100000);
  
  PROC *p = new PROC(&a,world);
  
  
  return a.exec();

}
