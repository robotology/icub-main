/* 
 * Copyright (C)2014  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <yarp/os/Thread.h>

#include <list>
#include <string>

using namespace yarp::os;

struct message_entry
{
    std::string messeges;
    std::string timestamp;
};

class log_entry
{
    public:
    std::list<message_entry> entry_list;

    public:
    std::string  port;
    std::string  process_name;
    std::string  process_pid;
    void clear();
    void append(message_entry entry);
};

class logger_thread;

class logger
{
    private:
    logger_thread* log_updater;

    public:
    logger(std::string portName);
    ~logger();
    bool start();
    void stop();

    void get_messages_by_port    (std::string  port,    std::list<message_entry>& messages);
    void get_messages_by_process (std::string  process, std::list<message_entry>& messages);
    void get_messages_by_pid     (std::string  pid,     std::list<message_entry>& messages);
    
};
