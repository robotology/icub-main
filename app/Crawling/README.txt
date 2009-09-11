
To launch the crawling, you need to do the following:
(either in $ICUB_DIR/app/Crawling/scripts or in $ICUB_DIR/app/default/scripts)

With the yarp run interface

1. manager.py vel_cont.xml  
2. manager.py gen.xml
3. manager.py man.xml

(the script are made so that the processes run on a node called /janine)

With Linux scripts

1. ./velocity.sh start  or 
2. ./generator.sh start or 
3. CrawlManager

You can stop the processes using ./velocity.sh stop (or kill) or ./generator.sh stop (or kill) 

The config files used by the application are located in ICUB_DIR/app/Crawling/config. 
Only the managerConfig.ini file should be modified. 
