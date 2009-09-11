
To launch the crawling, you need to do the following:
(

With the yarp run interface (from $ICUB_DIR/app/default/scripts)

1. manager.py ../../Crawling/scripts/vel_cont.xml  
2. manager.py ../../Crawling/scripts/gen.xml
3. manager.py ../../Crawling/scripts/man.xml

(the scripts are made so that the processes run on a node called /janine)

With Linux scripts (from $ICUB_DIR/app/Crawling/scripts)

1. ./velocity.sh start  or 
2. ./generator.sh start or 
3. CrawlManager

You can stop the processes using ./velocity.sh stop (or kill) or ./generator.sh stop (or kill) 

The config files used by the application are located in ICUB_DIR/app/Crawling/config. 

