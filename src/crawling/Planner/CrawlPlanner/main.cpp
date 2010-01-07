#include <stdio.h>
#include <yarp/os/all.h>
using namespace yarp::os;

#include "CrawlPlanner.h"

int main(int argc, char* argv[])
{
    Network yarp;

    //create and run tracker module
    CrawlPlanner CrawlPlanner;
  
    return CrawlPlanner.runModule(argc, argv);
}