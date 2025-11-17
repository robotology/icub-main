/* 
 * Copyright (C)2025  iCub Facility - Istituto Italiano di Tecnologia
 * Author: SATHISH KUMAR S
 * email:  sathish.subramani@iit.it
 * github: https://github.com/sksubiit
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details
*/
#ifndef __TEST_H__
#define __TEST_H__

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <algorithm> 
#include <fstream>
#include <sstream>
#include <cctype>
#include <limits>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <set>
#include <regex>
#include <chrono>
#include <iomanip>
#include <limits.h>   
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>

#include "EoBoards.h"  
#include "EoUpdaterProtocol.h"


class simpleEthClient
{
public:

    simpleEthClient(): sock_(-1) {}
    ~simpleEthClient() { closeSocket(); } 

    //bool open(const char *ip, uint16_t port = 7777, double rx_timeout_sec = 1.0);
    bool open(const char *ip, double rx_timeout_sec = 5.0);
    void closeSocket();
    bool sendRaw(const void *buf, size_t len);

    // Board operations
    bool discover();
    bool jump2updater();
    bool def2run_application();
    bool restart();
    bool blink();

    // normal programming mode
    bool program(); 

    // Ensure the target at `ip` is in maintenance (eUpdater). Logs progress to `log`.
    // Returns: 0 => entered maintenance and needs programming,
    //          2 => already up-to-date (skip programming),
    //          1 => failure.
    int ensureMaintenance(const char *ip, int max_retries, int retry_delay_sec, std::ostream &log);

    // find firmware entry and optional version in firmware.info.xml
    bool findFirmwareForBoardWithVersion(const std::string &boardname, std::string &out_hexpath, int &out_major, int &out_minor);

    // Orchestrator entry: parse network file, prepare all boards in parallel, then program prepared ones.
    // Returns 0 on full success, non-zero otherwise.
    static int orchestrateParallelProgram();

private:
    int sock_;
    struct sockaddr_in dest_;
    struct sockaddr_in src_;

    // helpers for programming
    bool recvReplyForIP(uint8_t expected_opc, int timeout_ms, eOuprot_result_t &out_res);
    bool findFirmwareForBoard(const std::string &boardname, std::string &out_hexpath);
    bool sendPROG_START(eOuprot_partition2prog_t partition, eOuprot_result_t &out_res);
    bool sendPROG_DATA_chunk(uint32_t address, const uint8_t *data, size_t len, eOuprot_result_t &out_res);
    bool sendPROG_END(uint16_t numberofpkts, eOuprot_result_t &out_res);

    //discover reply printers
    static void print_discover_reply(const eOuprot_cmd_DISCOVER_REPLY_t *reply, const char *srcip);
    static void print_legacy_scan_reply(const eOuprot_cmd_LEGACY_SCAN_REPLY_t *scan, const char *srcip);
    


    // Orchestrator helpers (tagged to the class)
    static std::vector<std::string> parseIPsFromNetworkFile(const char *xmlpath);
    static std::string logname(const std::string &ip);
    static pid_t spawn_and_log(const std::string &exe_path, const std::vector<std::string> &args, const std::string &logpath, bool append);
};
#endif // __TEST_H__