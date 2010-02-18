#ifndef CARTESIANMOVER_H
#define CARTESIANMOVER_H

#include <math.h>
///////////YARP//////////
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/String.h>
///////////ICUB///////////
#include <iCub/ctrlMath.h>
///////////GTK///////////
#include <gtk/gtk.h>
#include <gtk/gtkmain.h>
#include "gtkMessages.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace ctrl;
using namespace yarp::math;
using namespace yarp::os::impl;

#define NUMBER_OF_CARTESIAN_COORDINATES 6
#define UPDATE_TIME 500

struct gtkClassData;

class cartesianMover{
 private:
  GtkWidget *vbox;
  PolyDriver *partDd;

  //cartesian interface
  ICartesianControl *crt;

  int *index;

  GtkWidget **frame_slider1;
  GtkWidget **currPosArray;
  GtkWidget **sliderArray;
  GtkWidget** po;
  guint *entry_id;

  static bool display_cartesian_pose(cartesianMover *currentPart);
  static bool display_axis_pose(cartesianMover *currentPart);
  static void position_slider_changed(GtkRange *range, cartesianMover *currentPart);

 public:
  ResourceFinder *finder;

  char *partLabel;
  bool interfaceError;
  cartesianMover(GtkWidget *vbox_d, PolyDriver *partDd_d, char * labelPart, ResourceFinder *fnd);
  ~cartesianMover();
  void releaseDriver();

};

struct gtkClassCartesianData
{
  int *indexPointer;
  cartesianMover* cartesianPointer;
};

#endif
