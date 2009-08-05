
#include "partMover.h"

partMover::partMover()
{

}

partMover::~partMover()
{
  fprintf(stderr, "Closing partMover thread \n");
}

void partMover::setPolyDriver(PolyDriver *deviceDriver)
{
  dd=deviceDriver;

  bool ok;
  ok  = dd->view(pos);
  ok &= dd->view(vel);
  ok &= dd->view(enc);
  ok &= dd->view(pid);
  ok &= dd->view(amp);
  ok &= dd->view(lim);

  if (!ok)
    fprintf(stderr, "Problems acquiring interfaces\n");
  else
    fprintf(stderr, "Control board was accessed succesfully!\n");

  pos->getAxes(&numberOfJoints);
  fprintf(stderr, "Number of axes is: %d \n", numberOfJoints);

}


