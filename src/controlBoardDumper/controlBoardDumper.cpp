#include "controlBoardDumper.h"


void controlBoardDumper::setDevice(PolyDriver *Arm_d, int rate, ConstString portPrefix, ConstString dataToDump)
{
  // open ports
  Arm_dd=Arm_d;
  //getter = NULL;

  bool ok;
  ok  = Arm_dd->view(pos);
  ok &= Arm_dd->view(vel);
  ok &= Arm_dd->view(enc);
  ok &= Arm_dd->view(pid);
  ok &= Arm_dd->view(amp);
  ok &= Arm_dd->view(lim);

  if (!ok)
    ACE_OS::printf("Problems acquiring interfaces\n");
  else
    ACE_OS::printf("Control board was accessed succesfully!\n");

  pos->getAxes(&numberOfJoints);
  fprintf(stderr, "Number of axes is: %d \n", numberOfJoints);

  //initialize used variables
  data = new double [numberOfJoints];

  port = new Port;

  ConstString portName = portPrefix + dataToDump;
  port->open(portName.c_str());

  this->setRate(rate);
}

controlBoardDumper::~controlBoardDumper()
{    

}

void controlBoardDumper::setThetaMap(int *map, int n)
{ 
  fprintf(stderr, "Setting the map dimension %d \n", n);
  numberOfJointsRead = n;
  dataRead = new double [numberOfJointsRead];
  dataMap = new int [numberOfJointsRead];
  for (int i = 0; i < numberOfJointsRead; i++)
    {
      //fprintf(stderr, "map is %d \n", map[i]);
      dataMap[i] = map[i];
      dataRead[i] = 1.0;
    }
}

void controlBoardDumper::setGetter(GetData *g)
{
  getter = g;
}

bool controlBoardDumper::threadInit()
{

  return 1;
}

controlBoardDumper::controlBoardDumper():RateThread(500)
{

}

void controlBoardDumper::threadRelease()
{
  fprintf(stderr, "Closing thread \n");
  //Arm_dd->close();
  Time::delay(.1);
  fprintf(stderr, "Closing ports \n");
  port->close();
}

void controlBoardDumper::run()
{
  //ACE_OS::printf("Entering the main thread\n");
  //enc->getEncoders(data);
  //Bottle bData;
  //for (int i = 0; i < numberOfJointsRead; i++)
  //  {
  //    dataRead[i] = data[dataMap[i]];
  //    bData.addDouble(dataRead[i]);
  //  }
  //port->write(bData);

  if (getter)
    {
      //ACE_OS::printf("Getter is getting something\n");
      getter -> getData(data);

      //fprintf(stderr, "Time is %lf \n", stmp.getTime());

      Bottle bData;
      for (int i = 0; i < numberOfJointsRead; i++)
	{
	  //ACE_OS::printf("%.2f \n", data[dataMap[i]]);
	  dataRead[i] = data[dataMap[i]];
	  bData.addDouble(dataRead[i]);
	}
      if (getter->getStamp(stmp)) 
	{
	  if (stmp.isValid())
	    port->setEnvelope(stmp);
	  else
	    {
            //stmp.update();
            stmp=Stamp(0,-1);
	      port->setEnvelope(stmp);
	    }
	}
      else
	fprintf(stderr, "controlBoardDumper::warning. Trying to get a stamp without a proper IPreciselyTimed defined. \n");
      
      port->write(bData);
    }
}  
