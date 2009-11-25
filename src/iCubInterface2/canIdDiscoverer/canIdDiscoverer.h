#ifndef __CANIDDISCOVERER__
#define __CANIDDISCOVERER__

// Probe canbus network for control cards
// return a list of pairs <KEY, ID>
// KEY=a string identifying the board type (HEAD, RIGHTARM...)
// ID=integer, can id of the board(s)
// 
// At the moment chcecks only the first board found.
//
// Warning, code "as it is" written for demo...
// Oct 07, Lorenzo Natale
//

#include "downloader.h"

#include <yarp/os/Searchable.h>

#include <string>
#include <list>

const int NNETS=10;
static const char *POSSIBLE_NETWORKS[]={"LEGS", "LEFTHAND", "RIGHTHAND", "TORSO", "HEAD", "RIGHTARM", "LEFTARM", "HEADTORSO", "LEFTLEG", "RIGHTLEG"};

const int ICUB_CAN_IDS_NUMDEVICES=4;

struct canIdEntry
{
    std::string key;
    int id;

    canIdEntry(std::string &k, int i)
    {
        key=k;
        id=i;
    }
};

class ICUB_CAN_IDS
{
public:
    typedef std::list<canIdEntry> CanIdList;
    typedef CanIdList::iterator CanIdIterator;

    CanIdList canIdList;

    int operator [](const std::string &key)
    {
        CanIdIterator it=canIdList.begin();
        bool found=false;
        while(it!=canIdList.end())
        {
            if ((*it).key==key)
                return (*it).id;

            it++;
        }

        return -1;
    }

    void push(const canIdEntry &v)
    {
        canIdList.push_back(v);
    }

    const ICUB_CAN_IDS &operator=(const ICUB_CAN_IDS &r)
    {
        canIdList=r.canIdList;
        return *this;
    }

    ICUB_CAN_IDS(const ICUB_CAN_IDS &r)
    {
        canIdList=r.canIdList;
    }

    ICUB_CAN_IDS(){}

    void print()
    {
        CanIdIterator it=canIdList.begin();
        bool found=false;
        while(it!=canIdList.end())
        {
            printf("%s: %d\n", (*it).key.c_str(), (*it).id);
            it++;
        }
    }

    int size()
    {
        return canIdList.size();
    }
};

class CanIdDiscoverer
{
private:
    cDownloader downloader;
    bool wasInvokedF;

public:
    CanIdDiscoverer()
    {
        wasInvokedF=false;
    }

    ~CanIdDiscoverer()
    {}

    ICUB_CAN_IDS discover(const char *devName)
    {
        ICUB_CAN_IDS ret;

        for(int i=0;i<ICUB_CAN_IDS_NUMDEVICES;i++)
        {
            yarp::os::Property params;
            params.put("device", devName);
            params.put("CanDeviceNum", i);
            params.put("CanTxQueue", 64);
            params.put("CanRxQueue", 64);
            params.put("CanTxTimeout", 2000);
            params.put("CanRxTimeout", 2000);
            int err=0;
            err=downloader.initdriver(params);

            if(err==-1)
            {
                fprintf(stderr, "No id %d device on can network\n", i);
                downloader.stopdriver();
            }
            else 
            {
                int err=downloader.initschede();

                if (err==-1)
                {
                    fprintf(stderr, "Error reading from can network %d\n", i);
                }
                else if (downloader.board_list_size<=0)
                {
                    fprintf(stderr, "Id %d device replied, but no boards found\n",i);
                }
                else
                {    
                    printf("Found %d boards on net %d\n", downloader.board_list_size, i);
                    char *addInfo=downloader.board_list[0].add_info;

                    //searching for first non-empty additional info
                    int b=0;
                    while((strncmp(addInfo, "", 1)==0)&&(b<downloader.board_list_size))
                    {
                        b++;
                        //	fprintf(stderr, "Warning checking board %d\n",b);
                        addInfo=downloader.board_list[b].add_info;
                    }
                    /////////////////

                    // now match addInfo with list of IDs
                    bool found=false;
                    for(int n=0;n<NNETS;n++)
                    {
                        std::string k=POSSIBLE_NETWORKS[n];
                        if (strncmp(addInfo, k.c_str(),k.length())==0)
                        {
                            //    fprintf(stderr, "Writing %s\n", k.c_str());
                            ret.push(canIdEntry(k,i));
                            found=true;
                        }
                    }
                    if (!found)
                        {
                            printf("Warning: on network %d additional info contains unrecognized network identifier (%s)\n", i, addInfo);
                        }
                }
                downloader.stopdriver();
            }
        }

        wasInvokedF=true;
        return ret;
    }

    void printList(ICUB_CAN_IDS &v)
    {
        v.print();
    }

    bool wasInvoked()
    {
        return wasInvokedF;
    }
};

#endif
