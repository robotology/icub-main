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

#include "logger.h"
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <yarp/os/Semaphore.h>

//private classes
class logger_thread : public Thread
{
    public:
    yarp::os::Semaphore mutex;
    std::list<log_entry> log_list;
    Port logger_port;
    std::string portName;
    
    public:
    void run();
};

void log_entry::clear()
{
    entry_list.clear();
}

void log_entry::append(message_entry entry)
{
    entry_list.push_back(entry);
}

void logger_thread::run()
{
    while (1)
    {
        yarp::os::Time::delay(0.1);
        this->mutex.wait();

        Bottle b;
        logger_port.read(b);
        if (b.size()!=2) 
        {
            fprintf (stderr, "unkwnon log format!\n");
            continue;
        }

        std::string header = b.get(0).asString();
        message_entry body;
        body.messeges = b.get(1).asString();
        body.timestamp = "";

        log_entry entry;
        std::istringstream iss(header);
        std::string token;
        getline(iss, token, '/');
        getline(iss, token, '/');
        getline(iss, token, '/'); entry.port = token;
        getline(iss, token, '/'); entry.process_name = token;
        getline(iss, token, '/'); entry.process_pid = token.erase(token.size()-1);
        
        std::list<log_entry>::iterator it;
        for (it = log_list.begin(); it != log_list.end(); it++)
        {
            if (it->process_pid==entry.process_pid)
            {
                it->append(body);
                break;
            }
        }
        if (it == log_list.end())
        {
            entry.append(body);
            log_list.push_back(entry);
        }

        this->mutex.post();
    }
}

//public methods
bool logger::start()
{
    if (log_updater->logger_port.open(log_updater->portName.c_str())==true)
    {
        fprintf(stdout,"Logger successfully started, listening on port %s\n", log_updater->portName.c_str());
    }
    else
    {
        fprintf(stderr,"Unable to start logger: port %s is unavailable\n", log_updater->portName.c_str());
        return false;
    }

    log_updater->start();
    return true;
}

void logger::stop()
{
    log_updater->stop();
    log_updater->logger_port.close();
}

logger::logger(std::string portName)
{
    log_updater=new logger_thread();
    log_updater->portName = portName;
}

logger::~logger()
{
    if (log_updater!=0)
    {
        delete log_updater;
        log_updater = 0;
    }
}

void logger::get_messages_by_port    (std::string  port,  std::list<message_entry>& messages)
{
    log_updater->mutex.wait();
    std::list<log_entry>::iterator it;
    for (it = log_updater->log_list.begin(); it != log_updater->log_list.end(); it++)
    {
        if (it->port == port)
        {
            messages = (it->entry_list);
            break;
        }
    }
    log_updater->mutex.post();
}

void logger::get_messages_by_process (std::string  process,  std::list<message_entry>& messages)
{
    log_updater->mutex.wait();
    std::list<log_entry>::iterator it;
    for (it = log_updater->log_list.begin(); it != log_updater->log_list.end(); it++)
    {
        if (it->process_name == process)
        {
            messages = (it->entry_list);
            break;
        }
    }
    log_updater->mutex.post();
}

void logger::get_messages_by_pid     (std::string pid, std::list<message_entry>& messages)
{
    log_updater->mutex.wait();
    std::list<log_entry>::iterator it;
    for (it = log_updater->log_list.begin(); it != log_updater->log_list.end(); it++)
    {
        if (it->process_pid == pid)
        {
            messages = (it->entry_list);
            break;
        }
    }
    log_updater->mutex.post();
}
