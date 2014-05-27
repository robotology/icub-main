#ifndef ROBOT_INTERFACE_STATUS_MSG
#define ROBOT_INTERFACE_STATUS_MSG
#include <string>
#include <vector>
#include <yarp/os/Bottle.h>

class robotInterfaceStatusMsg
{
public:
    std::vector<int> crashed_boards;
    std::vector<std::pair<int,std::string>> faulted_boards;
    std::vector<int> currents;
    int total_connected_boards;
    yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle temp;
        yarp::os::Bottle& list= temp.addList();
        //temp.addInt(crashed_boards.size());
        list.addString("crashed_boards");
        for (int i=0;i<crashed_boards.size();i++)
        {
            list.addInt(crashed_boards[i]);
        }
        yarp::os::Bottle& list1= temp.addList();
        list1.addString("faulted_boards");        
        for (int i=0;i<faulted_boards.size();i++)
        {
            list1.addInt(faulted_boards[i].first);
            list1.addString(faulted_boards[i].second);
        }
//        temp.add(total_connected_boards);
        return temp;
    }
    void fromBottle(yarp::os::Bottle& temp)
    {
        yarp::os::Bottle* list=temp.get(0).asList();
        for (int i=1;i<list->size();i++)
            crashed_boards.push_back(list->get(i).asInt());
        yarp::os::Bottle* list1=temp.get(1).asList();
        for (int i=1;i<list1->size();i=i+2)
            faulted_boards.push_back(std::make_pair(list1->get(i).asInt(),list1->get(i+1).asString()));
  //      total_connected_boards=temp.get(num+2).asInt();
        return;
    }
};

#endif //ROBOT_INTERFACE_STATUS_MSG
