echo off

cd DAT

start /B CrawlGeneratorModule.exe --part right_arm --file ..\..\config\right_armConfig.ini --period 50 &
start /B CrawlGeneratorModule.exe --part left_arm --file ..\..\config\left_armConfig.ini --period 50 &
start /B CrawlGeneratorModule.exe --part right_leg --file ..\..\config\right_legConfig.ini --period 50 &
start /B CrawlGeneratorModule.exe --part left_leg --file ..\..\config\left_legConfig.ini --period 50 &
start /B CrawlGeneratorModule.exe --part head --file ..\..\config\headConfig.ini --period 50 &
start /B CrawlGeneratorModule.exe --part torso --file ..\..\config\torsoConfig.ini --period 50 &
