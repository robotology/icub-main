#include "statCollector.h"

statCollector::statCollector(int r):RateThread(r)
{

}

statCollector::~statCollector()
{
 
}

void statCollector::setDeviceDriver(PolyDriver *deviceDriver)
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

  vStat = new FpsStats[numberOfJoints];
  pStat = new FpsStats[numberOfJoints];
  out = new double[numberOfJoints];
  q = new double[numberOfJoints];
}

void statCollector::run()
{
  static int iter = 0;
  //get applied voltages
  while(!pid->getOutputs(out));
  //get positions
  while(!enc->getEncoders(q));
  iter++;

  if (iter%100==0)
    fprintf(stderr, "***\n");

  for (int joint = 0; joint < numberOfJoints; joint++)
    {  
      vStat[joint].update(out[joint]);
      pStat[joint].update(q[joint]);

      if((iter%100)==0)
	{
	  double vMean, vStd, vMin, vMax;
	  double qMean, qStd, qMin, qMax;
	  vStat[joint].getStats(vMean, vStd, vMin, vMax);
	  pStat[joint].getStats(qMean, qStd, qMin, qMax);
	  if (vMin != vMax)
	    {
	      fprintf(stderr, "v[%d] doesn't seem to be constant: [mean, std, min, max] = [%.2f, %.4f, %.2f, %.2f]", joint, vMean, vStd, vMin, vMax);
	      if ((qMin != qMax))
		fprintf(stderr, "and q[%d]: [mean, std, min, max] = [%.2f, %.4f, %.2f, %.2f]", joint, qMean, qStd, qMin, qMax);
	      fprintf(stderr, "\n");
	    }
	  else if ((qMin != qMax))
	    fprintf(stderr, "q[%d] is varying with constant v: [mean, std, min, max] = [%.2f, %.4f, %.2f, %.2f]\n", joint, qMean, qStd, qMin, qMax);
	  vStat[joint].reset();
	  pStat[joint].reset();
	  
	}
    }
}

void statCollector::threadRelease()
{
  fprintf(stderr, "Closing thread \n");
  delete[] vStat;
  delete[] pStat;
  delete[] out;
  delete[] q;
  //Arm_dd->close();
  //Time::delay(.1);
  //fprintf(stderr, "Closing ports \n");
  //port->close();
}
