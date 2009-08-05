#include "collectorPort.h"

collectorPort::collectorPort()
{
  middlePortName = new char[80];
  clientPortName = new char[80];
  serverPortName = new char[80];
}

void collectorPort::open(const char *middle,
			 const char *client,
			 const char *server,
			 const char *output)
{
  portMiddle = new Port;
  portMiddle->open(middle);
  
  portOutput = new Port;
  portOutput ->open(output);

  Network::connect   (middle, server);
  Network::disconnect(client, server);
  Network::connect   (client, middle);

  strcpy(middlePortName, middle);
  strcpy(serverPortName, server);
  strcpy(clientPortName, client);

  continueCommunication = true;
}
    
void collectorPort::close()
{
  fprintf(stderr, "Closing the collectorPort thread. First disconnecting the client to middle...\n");
  Network::disconnect(clientPortName, middlePortName);
  fprintf(stderr, "...Done! \n Now disconnecting middle to server...");
  Network::disconnect(middlePortName, serverPortName);
  fprintf(stderr, "Done! \n Now reconnecting the client to server...");
  Network::connect(clientPortName, serverPortName);
  fprintf(stderr, "Done!");

  continueCommunication = false;
  portMiddle->interrupt();
  fprintf(stderr, "Port was interrupted\n");
  //port.close();
}

void collectorPort::run()
{

  while(!Thread::isStopping() && continueCommunication)
    {
      Bottle in, out;
      
      //fprintf(stderr, "reading...");
      portMiddle->read(in, true);
      //fprintf(stderr, "Received a message: %s", in.toString().c_str());
      if (continueCommunication)
	{
	  //fprintf(stderr, "...writing...");
	  portMiddle->write(in, out);
	  //fprintf(stderr, "in [%s] out [%s]\n", in.toString().c_str(), 
	  //         out.toString().c_str());
	  //fprintf(stderr, "...replying \n");
	  portMiddle->reply(out);
	}
      else
	fprintf(stderr, "...quitting communication \n");
      
      //write input to output
      portOutput->write(in);
    }
  fprintf(stderr, "Exiting the portCollector thread\n");
  delete portMiddle;
}
