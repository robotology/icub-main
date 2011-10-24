/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Carlo Ciliberto
 * email:  carlo.ciliberto@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Rand.h>


#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>


#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>
#include <vector>
#include <list>


#include <iCub/boostMIL/ClassifierFactory.h>
#include <iCub/boostMIL/OnlineBoost.h>
#include <iCub/boostMIL/MILClassifier.h>

#include <yarp/os/ResourceFinder.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::boostMIL;



//register the Weak Classifiers in the classifier factory.
void register_classifiers(ResourceFinder &rf)
{
    ClassifierFactory::instance().registerClassifier(new MILClassifier("mil",rf));
}



//Reads the features (related to a single image) from the file 'stream' and concatenates them in a vector.
//Returns the inputs containing this long vector associated with the image label.
//
//The system assumption is that the first line in 'stream' contains the image label
//while every other line in the file contains a single image feataure.
Inputs *extractInputFromFile(ifstream &stream)
{
    std::stringstream strstr;
    strstr << stream.rdbuf();

    yarp::os::Bottle bot;
    bot.fromString(strstr.str().c_str());

    int label = bot.get(0).asInt();

    if((bot.size()-1) % 2 != 0)
    {
        cout << "Error! wrong bottle size!" << endl;
        exit(1);
    }

    int n_features = (bot.size()-1)/2;

    int feature_size = 64;

    Vector *vec = new Vector(n_features*bot.get(2).asList()->size());


    int idx = 0;
    int i = 0;
    for(i = 1; i <= n_features; i++)
    {
        //odd elements in the bottle are not taken as they contain information on the 2D position of 
        //the feature. We are interested in the 64 (for SURFs) dimensional vector.
        yarp::os::Bottle *b = bot.get(2*i).asList();
        if(b->size() != feature_size)
        {
            cout << "wrong feature size: " << b->size() << endl;
        }
        for(int j = 0; j < b->size(); j++)
        {
            (*vec)[idx] = b->get(j).asDouble();
            idx++;
        }
    }
    
    if(idx != vec->size())
    {
        cout << "Error! Filled " << idx << "elements but the vector has size " << vec->size() << endl;
        exit(1);
    }

    return new Inputs(label,"mil",vec);
}

//Extracts a list of samples from a given folder
void readFolders(const char *folder_name, list<Inputs*> &dataset, int n_start, int n_files)
{
    //load bags
    int cnt = n_start;
    int counter = 0;

    char loadPath[255];

    sprintf(loadPath,"%s%.08d.txt",folder_name,cnt);

    //open the first file
    ifstream fin(loadPath);

    fprintf(stdout,"0%%      100%% \n");
    //read all files
    while(fin.is_open())
    {
        if((counter*100/n_files)%10)
            fprintf(stdout,"*");

        if(counter > n_files) break;

        dataset.push_back(extractInputFromFile(fin));

        counter++;

        sprintf(loadPath,"%s%.08d.txt",folder_name,cnt);

        fin.close();
        fin.open(loadPath);
    }
}


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("boostMILExample/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    //register classifiers
    register_classifiers(rf);

    //load data into a list of inputs
    string path = rf.find("path").asString().c_str();

    int n_files = rf.find("n_files").asInt();
    int n_train = rf.find("n_train").asInt();
    int n_start = rf.find("n_start").asInt();

    list<Inputs*> dataset;

    fprintf(stdout,"Reading files from %s\n",path.c_str());
    readFolders(path.c_str(),dataset,n_start,n_files);
    fprintf(stdout,"Files Read\n");

    //initialize the Online Strong Classifier
    OnlineBoost online_boost(rf);

    int cnt = 0;
    list<Inputs*>::iterator d_itr = dataset.begin();

    int fa,fr,ta,tr;
    fa=fr=ta=tr=0;

    for(list<Inputs*>::iterator itr=dataset.begin(); itr!=dataset.end(); itr++)
    {
        if(cnt<n_train)
            online_boost.train(*itr);
        else
        {
            int output=online_boost.classify(*itr);
            if(output==0)
            {
                fprintf(stdout,"Error! Online Boost does not seem to be ready to classify!\n");
                return 0;
            }

            if(output*(*itr)->getLabel()>0)
                output==+1?ta++:tr++;
            else
                output==+1?fa++:fr++;
        }
        cnt++;
    }

    fprintf(stdout,"truly   accepted: %d\n",ta);
    fprintf(stdout,"truly   rejected: %d\n",tr);
    fprintf(stdout,"falsely accepted: %d\n",fa);
    fprintf(stdout,"falsely rejected: %d\n",fr);
    fprintf(stdout,"recognition performance: %f%%",(double) (ta+tr)/(ta+tr+fa+fr)*100);

    while(dataset.size())
    {
        delete dataset.front();
        dataset.pop_front();
    }
}

