
/*esLog project*/


#ifndef PROC_H
#define PROC_H

#include <qthread.h>
#include <qapplication.h>
#include "world3d.h"

using namespace std;

class PROC : public QThread
{
    
 public:
  PROC(QApplication* a_,world3d*w_);
  ~PROC();
  void run();
  
 private:
  world3d *world;
  QApplication *a;

};
#endif
