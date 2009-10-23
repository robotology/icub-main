
/*esLog project*/

#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <q3vbox.h>

#include "world3d.h"
#include "proc.h"


using namespace std;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);



  string fname=string("prova.jpg");

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--imagefile")==0)||(strcmp(argv[i],"-i")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-i/--imagefile file] " <<endl;
      exit(0);
    }
  }



  QObject::connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));

  //Create 3D World widget in a blank window:
  Q3VBox *mainwin = new Q3VBox;
  mainwin->setCaption( "EgoSphere World" );
  world3d* world = new world3d(&a,mainwin);
  usleep(100000);
  
  PROC *p = new PROC(&a,world,&fname);
  
  
  return a.exec();

}
