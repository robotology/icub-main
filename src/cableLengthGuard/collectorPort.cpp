#include "collectorPort.h"

collectorPort::collectorPort(int n)
{
  nJnts = n;
  theta = new double[n];
  received = new bool[n];
  for (int i = 0; i < n; i++)
    received[i] = false;
  fprintf(stderr, "Collector port has initialized the vals \n");
}

void collectorPort::open(const char *name)
{
  port = new BufferedPort<Bottle>;
  port->open(name);
}
    
void collectorPort::close()
{
  port->interrupt();
  fprintf(stderr, "Port was interrupted\n");
  //port.close();
}

void collectorPort::run()
{
  while(!Thread::isStopping())
    {
      Bottle *in, lastCommand;
      
      //mutex.wait();
      //for(int i=0; i < nJnts; i++)
      //if (received[i] == true)
      //  {
      //    fprintf(stderr, "[%d]:%.1f ", i, theta[i]);
      //    fprintf(stderr, "\n");
      //  }
      //mutex.post();

      //fprintf(stderr, "reading...");
      in = port->read(false);
      //fprintf(stderr, "...done \n");
      if (in!=0)
	{
	  lastCommand = *in;
	  //fprintf(stderr, "Collector --> received %s\n", lastCommand.toString().c_str());
	  Value v1 = lastCommand.pop();
	  Value v2 = lastCommand.pop();
	  Value v3 = lastCommand.pop();
	  Value v4 = lastCommand.pop();
	  if (v1.isDouble() && v2.isInt() && v3.isVocab() && v4.isVocab())
	      if (v3.asVocab() == Vocab::encode("pos"))
		if (v4.asVocab() == Vocab::encode("set"))
		    {
		      if (v2.asInt()<nJnts)
			{
			  theta[v2.asInt()] = v1.asDouble();
			  received[v2.asInt()] = true;
			}
		      else
			fprintf(stderr, "Refusing the value cause out of range");
		    }
	  
	  if (v1.isList() && v2.isList())
	    {
	      Bottle *b1 = v1.asList();
	      Bottle *b2 = v2.asList();
	      if (b2->pop().asVocab() == Vocab::encode("poss"))
		{
		  int n = b1->size();
		  if (n == nJnts)
		    {
		      mutex.wait();
		      for (int i = 0; i < n; i++)
			{
			  theta[i] = b1->get(i).asDouble();
			  received[i] = true;
			}
		      mutex.post();
		    }
		  else
		    fprintf(stderr, "Refusing values cause of wrong dimensions");
		  //fprintf(stderr, "First value is: %.1f\n", v1.asString().c_str());
		  //ConstString stringVocab = Vocab::decode(v2.asVocab());
		  //fprintf(stderr, "Second value is: %s\n", stringVocab.c_str());
		}	    
	    }
	}
    }
  fprintf(stderr, "Exiting the portCollector thread\n");
  delete port;
  delete[] theta;
  delete[] received;
}
	
bool collectorPort::get(double &th, int i)
{
  if(i < nJnts)
    if (received[i] == true)
      {
	th = theta[i];
	return true;
      }

  return false;

}

void collectorPort::reset()
{
  for (int i = 0; i < nJnts; i++)
    received[i] = false;
}
