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

#include "yafu.h"

// helper: receive a 4-byte eOuprot_cmdREPLY_t for dest_ ip and check opc
bool simpleEthClient::recvReplyForIP(uint8_t expected_opc, int timeout_ms, eOuprot_result_t &out_res)
{
    (void)timeout_ms; // socket timeout already set in open()
    unsigned char buf[1500];
    socklen_t sl = sizeof(src_);

    // loop until timeout or until we receive an expected reply from dest_
    while (true) {
        ssize_t r = recvfrom(sock_, buf, sizeof(buf), 0, (struct sockaddr*)&src_, &sl);
        if (r < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // socket read timed out -> no reply
                return false;
            }
            perror("recvfrom");
            return false;
        }
        // ignore packets from other senders, keep waiting until timeout
        if (src_.sin_addr.s_addr != dest_.sin_addr.s_addr) {
            continue;
        }
        if ((size_t)r < sizeof(eOuprot_cmdREPLY_t)) continue;
        auto *reply = reinterpret_cast<eOuprot_cmdREPLY_t*>(buf);
        if (reply->opc != expected_opc) continue;
        out_res = static_cast<eOuprot_result_t>(reply->res);
        return true;
    }
}

// search firmware.info.xml for <board type="boardname"> and return the file path (relative resolved)
bool simpleEthClient::findFirmwareForBoard(const std::string &boardname, std::string &out_hexpath)
{
    // path hardcoded relative to repo; adapt if needed
    const char *xmlpath = "../../../../../icub-firmware-build/info/firmware.info.xml";
    std::ifstream f(xmlpath);
    if (!f.is_open()) return false;
    std::string line;
    bool inboard = false;
    std::string foundfile;
    while (std::getline(f, line)) {
        // trim whitespace
        std::string s = line;
        // lowercase for safer matching (board types in xml are lowercase)
        auto tolower_copy = [](std::string t){ for(char &c: t) c = static_cast<char>(std::tolower((unsigned char)c)); return t; };
        std::string tl = tolower_copy(s);
        std::string key = "board type=\"";
        size_t pos = tl.find(key);
        if (!inboard && pos != std::string::npos) {
            size_t start = pos + key.size();
            size_t end = tl.find('"', start);
            if (end != std::string::npos) {
                std::string bt = tl.substr(start, end - start);
                if (bt == tolower_copy(boardname)) {
                    inboard = true;
                    continue;
                }
            }
        }
        if (inboard) {
            size_t p1 = s.find("<file>");
            if (p1 != std::string::npos) {
                size_t p2 = s.find("</file>", p1);
                if (p2 != std::string::npos) {
                    foundfile = s.substr(p1 + strlen("<file>"), p2 - (p1 + strlen("<file>")));
                    break;
                }
            }
            // board close without file -> stop
            if (s.find("</board>") != std::string::npos) break;
        }
    }
    f.close();
    if (foundfile.empty()) return false;
    // resolve relative path: xml path parent + foundfile
    std::string xmls(xmlpath);
    size_t slash = xmls.rfind('/');
    std::string dir = (slash == std::string::npos) ? std::string(".") : xmls.substr(0, slash+1);
    std::string candidate = dir + foundfile;
    // normalize simple ../
    // try candidate as-is
    struct stat st;
    if (stat(candidate.c_str(), &st) == 0) {
        out_hexpath = candidate;
        return true;
    }
    // fallback: try the raw foundfile as absolute or relative to cwd
    if (stat(foundfile.c_str(), &st) == 0) {
        out_hexpath = foundfile;
        return true;
    }
    return false;
}

// new helper: find firmware file and optional version attribute (version="MAJOR.MINOR") in the <file> tag
bool simpleEthClient::findFirmwareForBoardWithVersion(const std::string &boardname, std::string &out_hexpath, int &out_major, int &out_minor)
{
    out_major = out_minor = -1;
    const char *xmlpath = "../../../../../icub-firmware-build/info/firmware.info.xml";
    std::ifstream f(xmlpath);
    if (!f.is_open()) return false;

    std::string line;
    bool in_correct_board_block = false;
    std::string foundfile;

    auto tolower_copy = [](std::string s) {
        for (char &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };

    while (std::getline(f, line)) {
        std::string lower_line = tolower_copy(line);

        if (!in_correct_board_block) {
            // Look for the start of a board block that matches our board name
            std::string key = "<board type=\"" + tolower_copy(boardname) + "\"";
            if (lower_line.find(key) != std::string::npos) {
                in_correct_board_block = true;
            }
        } else {
            // We are inside the correct board block, look for file and version
            std::smatch match;
            std::regex file_re(R"(<\s*file\s*>([^<]+)</file>)");
            if (std::regex_search(line, match, file_re)) {
                foundfile = match[1].str();
            }

            std::regex version_re(R"(<\s*version\s+major\s*=\s*['"](\d+)['"]\s+minor\s*=\s*['"](\d+)['"])");
            if (std::regex_search(line, match, version_re)) {
                out_major = std::stoi(match[1].str());
                out_minor = std::stoi(match[2].str());
            }

            // If we find the closing board tag, we are done with this block
            if (lower_line.find("</board>") != std::string::npos) {
                break;
            }
        }
    }
    f.close();

    if (foundfile.empty()) {
        return false;
    }

    // resolve relative path: xml path parent + foundfile
    std::string xmls(xmlpath);
    size_t slash = xmls.rfind('/');
    std::string dir = (slash == std::string::npos) ? std::string(".") : xmls.substr(0, slash + 1);
    std::string candidate = dir + foundfile;

    struct stat st;
    if (stat(candidate.c_str(), &st) == 0) {
        out_hexpath = candidate;
        return true;
    }
    if (stat(foundfile.c_str(), &st) == 0) {
        out_hexpath = foundfile;
        return true;
    }
    out_hexpath = candidate;
    return true;
}

bool simpleEthClient::sendPROG_START(eOuprot_partition2prog_t partition, eOuprot_result_t &out_res)
{
    eOuprot_cmd_PROG_START_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_PROG_START;
    cmd.partition = static_cast<uint8_t>(partition);
    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto PROG_START"); return false; }
    if (!recvReplyForIP(uprot_OPC_PROG_START, 0, out_res)) return false;
    return true;
}

bool simpleEthClient::sendPROG_DATA_chunk(uint32_t address, const uint8_t *data, size_t len, eOuprot_result_t &out_res)
{
    if (len == 0 || len > uprot_PROGmaxsize) return false;
    // prepare command in a dynamic buffer to send only HEAD+data
    const size_t HEAD_SIZE = 7; // opc (1) + address(4) + size(2)
    std::vector<uint8_t> packet(HEAD_SIZE + len, EOUPROT_VALUE_OF_UNUSED_BYTE);
    packet[0] = static_cast<uint8_t>(uprot_OPC_PROG_DATA);
    // address little-endian
    packet[1] = static_cast<uint8_t>(address & 0xFF);
    packet[2] = static_cast<uint8_t>((address >> 8) & 0xFF);
    packet[3] = static_cast<uint8_t>((address >> 16) & 0xFF);
    packet[4] = static_cast<uint8_t>((address >> 24) & 0xFF);
    // size little-endian (2 bytes)
    packet[5] = static_cast<uint8_t>(len & 0xFF);
    packet[6] = static_cast<uint8_t>((len >> 8) & 0xFF);
    // data
    memcpy(&packet[HEAD_SIZE], data, len);
    if (!sendRaw(packet.data(), packet.size())) { perror("sendto PROG_DATA"); return false; }
    if (!recvReplyForIP(uprot_OPC_PROG_DATA, 0, out_res)) return false;
    return true;
}

bool simpleEthClient::sendPROG_END(uint16_t numberofpkts, eOuprot_result_t &out_res)
{
    eOuprot_cmd_PROG_END_t end_cmd;
    memset(&end_cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(end_cmd));
    end_cmd.opc = uprot_OPC_PROG_END;
    end_cmd.numberofpkts[0] = numberofpkts & 0xFF;
    end_cmd.numberofpkts[1] = (numberofpkts >> 8) & 0xFF;
    if (!sendRaw(&end_cmd, sizeof(end_cmd))) { perror("sendto PROG_END"); return false; }
    if (!recvReplyForIP(uprot_OPC_PROG_END, 0, out_res)) return false;
    return true;
}

// program workflow: discover -> xml lookup -> jump2updater if needed -> parse intel hex -> PROG_START/DATA/END -> def2run + restart
bool simpleEthClient::program()
{
    if (sock_ < 0) return false;

    // 1) discover the board (unicast to dest_)
    eOuprot_cmd_DISCOVER_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_LEGACY_SCAN;
    cmd.opc2 = uprot_OPC_DISCOVER;
    cmd.jump2updater = 0;
    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto discover"); return false; }

    // wait for reply from dest_
    unsigned char buf[1500];
    bool got = false;
    eOuprot_cmd_DISCOVER_REPLY_t discovered = {0};
    socklen_t sl;
    while (true)
    {
        sl = sizeof(src_);
        ssize_t r = recvfrom(sock_, buf, sizeof(buf), 0, (struct sockaddr*)&src_, &sl);
        if (r < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            perror("recvfrom");
            return false;
        }
        if (src_.sin_addr.s_addr != dest_.sin_addr.s_addr) continue;
        // pick the best available discover info
        if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY2_t)) {
            auto *rep2 = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY2_t*>(buf);
            discovered = rep2->discoveryreply;
            got = true;
            break;
        } else if ((size_t)r >= sizeof(eOuprot_cmd_MOREINFO_REPLY_t)) {
            auto *m = reinterpret_cast<eOuprot_cmd_MOREINFO_REPLY_t*>(buf);
            discovered = m->discover;
            got = true;
            break;
        } else if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY_t)) {
            auto *rep = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY_t*>(buf);
            discovered = *rep;
            got = true;
            break;
        } else {
            // ignore legacy limited replies for programming (we need boardtype/process info)
        }
    }
    if (!got) {
        std::cerr << "No discover reply from target IP" << std::endl;
        return false;
    }

    // derive board name used in firmware.info.xml (strip eobrd_ prefix if present)
    const char *raw_board_name = eoboards_type2string2(static_cast<eObrd_type_t>(discovered.boardtype), static_cast<eObool_t>(0));
    std::string board_name;
    if (raw_board_name) {
        board_name = raw_board_name;
        const char prefix[] = "eobrd_";
        if (board_name.rfind(prefix, 0) == 0) {
            board_name = board_name.substr(strlen(prefix));
        }
    } else {
        std::cerr << "Cannot resolve board type string from reply" << std::endl;
        return false;
    }
    std::cout << "Discovered board type: " << board_name << std::endl;

    // 2) find firmware file from firmware.info.xml
    std::string hexpath;
    if (!findFirmwareForBoard(board_name, hexpath)) {
        std::cerr << "Firmware entry not found for board '" << board_name << "' in firmware.info.xml" << std::endl;
        return false;
    }
    std::cout << "Found firmware file: " << hexpath << std::endl;

    // 3) ensure maintenance mode
    bool inmaintenance = (discovered.processes.runningnow == eUpdater);
    if (!inmaintenance) {
        std::cout << "Board not in maintenance, requesting jump2updater..." << std::endl;
        int max_retries = 3;
        for (int attempt = 0; attempt < max_retries; ++attempt) {
            if (!jump2updater()) {
                std::cerr << "Failed to send jump2updater, attempt " << (attempt + 1) << " of " << max_retries << std::endl;
                continue;
            }
            sleep(5); // Wait longer for the board to transition
            // Re-run discover and verify
            eOuprot_cmd_DISCOVER_t cmd2;
            memset(&cmd2, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd2));
            cmd2.opc = uprot_OPC_LEGACY_SCAN;
            cmd2.opc2 = uprot_OPC_DISCOVER;
            cmd2.jump2updater = 0;
            if (!sendRaw(&cmd2, sizeof(cmd2))) {
                perror("sendto discover2");
                return false;
            }
            bool got2 = false;
            while (true) {
                socklen_t sl = sizeof(src_);
                ssize_t r = recvfrom(sock_, buf, sizeof(buf), 0, (struct sockaddr*)&src_, &sl);
                if (r < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    perror("recvfrom");
                    return false;
                }
                if (src_.sin_addr.s_addr != dest_.sin_addr.s_addr) continue;
                if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY_t)) {
                    auto *rep = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY_t*>(buf);
                    discovered = *rep;
                    got2 = true;
                    break;
                }
            }
            if (got2 && discovered.processes.runningnow == eUpdater) {
                std::cout << "Board entered maintenance mode" << std::endl;
                break;
            }
            std::cerr << "Board did not enter eUpdater (maintenance) mode, attempt " << (attempt + 1) << " of " << max_retries << std::endl;
        }
        if (discovered.processes.runningnow != eUpdater) {
            std::cerr << "Board failed to enter maintenance mode after " << max_retries << " attempts" << std::endl;
            return false;
        }
    } else {
        std::cout << "Board already in maintenance" << std::endl;
    }

    // 4) open hex file and stream intel-hex
    std::ifstream hexf(hexpath);
    if (!hexf.is_open()) {
        std::cerr << "Cannot open hex file: " << hexpath << std::endl;
        return false;
    }

    // send PROG_START (partition = APPLICATION)
    eOuprot_result_t resa;
    if (!sendPROG_START(uprot_partitionAPPLICATION, resa)) {
        std::cerr << "PROG_START no reply or failed" << std::endl;
        return false;
    }
    if (resa != uprot_RES_OK) {
        std::cerr << "PROG_START returned error: " << (int)resa << std::endl;
        return false;
    }
    std::cout << "PROG_START acknowledged" << std::endl;

    // parse intel hex: accumulate contiguous data up to uprot_PROGmaxsize, send chunks
    std::string line;
    uint32_t upper16 = 0;
    std::vector<uint8_t> chunk;
    uint32_t chunk_base = 0;
    int chunks_sent = 0;
    auto flush_chunk = [&](bool force)->bool {
        if (chunk.empty()) return true;
        // send with retry
        const int MAX_RETRIES = 3;
        eOuprot_result_t rr;
        bool ok = false;
        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            if (sendPROG_DATA_chunk(chunk_base, chunk.data(), chunk.size(), rr)) {
                if (rr == uprot_RES_OK) { ok = true; break; }
                else if (rr == uprot_RES_ERR_TRYAGAIN) { usleep(200000); continue; }
                else { break; }
            } else {
                usleep(200000);
            }
        }
        if (!ok) return false;
        ++chunks_sent;
        chunk.clear();
        return true;
    };

    while (std::getline(hexf, line)) {
        if (line.empty()) continue;
        if (line[0] != ':') continue;
        // parse
        auto hexbyte = [](char hi, char lo)->int {
            auto val = [](char c)->int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                return 0;
            };
            return (val(hi) << 4) | val(lo);
        };
        size_t idx = 1;
        int bytecount = hexbyte(line[idx], line[idx+1]); idx += 2;
        int addr16 = (hexbyte(line[idx], line[idx+1]) << 8) | hexbyte(line[idx+2], line[idx+3]); idx += 4;
        int rectype = hexbyte(line[idx], line[idx+1]); idx += 2;
        std::vector<uint8_t> data;
        for (int i=0;i<bytecount;i++) {
            int b = hexbyte(line[idx], line[idx+1]); idx += 2;
            data.push_back(static_cast<uint8_t>(b));
        }
        // checksum ignored here (could be validated)
        if (rectype == 0x00) { // data
            uint32_t absaddr = (upper16 << 16) | static_cast<uint32_t>(addr16);

            // Skip records that target RAM (non-flash). The board will drop these;
            // avoid sending them from host (prevents extra "non flash chunk" packet).
            // so it follows the same logic as in the case of GUI by removing non flash chunk
            if ((absaddr & 0xFF000000u) == 0x20000000u) {
                // flush any pending flash chunk before skipping this RAM record
                if (!chunk.empty()) {
                    if (!flush_chunk(true)) { std::cerr << "Failed to send data chunk\n"; return false; }
                }
                std::cout << "Skipping Intel HEX record targeted to RAM at 0x"
                          << std::hex << absaddr << std::dec << " size=" << data.size() << std::endl;
                continue;
            }

            // if chunk empty initialize
            if (chunk.empty()) {
                chunk_base = absaddr;
            }
            // if not contiguous or would overflow, flush
            if (absaddr != chunk_base + chunk.size() || (chunk.size() + data.size() > uprot_PROGmaxsize)) {
                if (!flush_chunk(true)) { std::cerr << "Failed to send data chunk\n"; return false; }
                chunk_base = absaddr;
            }
            // append
            for (uint8_t b : data) chunk.push_back(b);
            // if chunk full flush
            if (chunk.size() >= uprot_PROGmaxsize) {
                if (!flush_chunk(true)) { std::cerr << "Failed to send data chunk\n"; return false; }
            }
        } else if (rectype == 0x01) { // EOF
            // flush any pending data
            if (!flush_chunk(true)) { std::cerr << "Failed to send final data chunk\n"; return false; }
            break;
        } else if (rectype == 0x04) { // extended linear address
            // value is two bytes in data
            if (data.size() >= 2) {
                uint32_t newUpper = (static_cast<uint32_t>(data[0]) << 8) | static_cast<uint32_t>(data[1]);
                // flush pending chunk before changing upper
                if (!flush_chunk(true)) { std::cerr << "Failed to send chunk before extended linear\n"; return false; }
                upper16 = newUpper;
            }
        } else {
            // other record types: treat as boundary -> flush
            if (!flush_chunk(true)) { std::cerr << "Failed to send chunk at record boundary\n"; return false; }
        }
    }

    // in case file ended without EOF record, flush
    if (!chunk.empty()) {
        if (!flush_chunk(true)) { std::cerr << "Failed to send trailing chunk\n"; return false; }
    }

    hexf.close();

    // send PROG_END: numberofpkts = chunks_sent + 1  (protocol requirement)
    uint16_t numberofpkts = static_cast<uint16_t>(chunks_sent + 1);
    std::cout << "Sending PROG_END, chunks_sent=" << chunks_sent << " numberofpkts=" << numberofpkts << std::endl;

    eOuprot_result_t r_end;
    if (!sendPROG_END(numberofpkts, r_end)) {
        std::cerr << "PROG_END send/recv failed (no reply)" << std::endl;
        return false;
    }
    if (r_end != uprot_RES_OK) {
        std::cerr << "PROG_END returned error " << (int)r_end << std::endl;
        return false;
    }
    std::cout << "PROG_END acknowledged, chunks sent: " << chunks_sent << std::endl;

    // restart the board to run the new application
    // if (!def2run_application()) {
    //     std::cerr << "Warning: def2run_application failed" << std::endl;
    // }
    // sleep(1);
    // if (!restart()) {
    //     std::cerr << "Warning: restart failed" << std::endl;
    // }

    return true;
}

bool simpleEthClient::open(const char *ip, double rx_timeout_sec)
{
    closeSocket();
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) { perror("socket"); return false; }

    int on = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
        perror("setsockopt(SO_REUSEADDR)");
    }
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    const char *local_ip_env = getenv("LOCAL_IP");
    if (local_ip_env && inet_pton(AF_INET, local_ip_env, &local.sin_addr) <= 0) {
        perror("inet_pton(LOCAL_IP)");
        closeSocket();
        return false;
    }
    if (!local_ip_env) local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(0); // Bind to an ephemeral port
    if (bind(sock_, (struct sockaddr*)&local, sizeof(local)) < 0) {
        perror("bind");
        closeSocket();
        return false;
    }

    memset(&dest_, 0, sizeof(dest_));
    dest_.sin_family = AF_INET;
    // Remote/receiver port used by the target remains 3333
    dest_.sin_port = htons(3333);
    if (inet_pton(AF_INET, ip, &dest_.sin_addr) <= 0) { perror("inet_pton"); closeSocket(); return false; }

    struct timeval tv;
    tv.tv_sec = static_cast<time_t>(rx_timeout_sec);
    tv.tv_usec = static_cast<suseconds_t>((rx_timeout_sec - tv.tv_sec) * 1e6);
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) { perror("setsockopt"); closeSocket(); return false; }

    return true;
}

// bool simpleEthClient::open(const char *ip, uint16_t port, double rx_timeout_sec)
// {
//     closeSocket();
//     sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
//     if (sock_ < 0) { perror("socket"); return false; }

//     int on = 1;
//     if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
//         perror("setsockopt(SO_REUSEADDR)");
//     }
//     struct sockaddr_in local;
//     memset(&local, 0, sizeof(local));
//     local.sin_family = AF_INET;
//     const char *local_ip_env = getenv("LOCAL_IP");
//     if (local_ip_env && inet_pton(AF_INET, local_ip_env, &local.sin_addr) <= 0) {
//         perror("inet_pton(LOCAL_IP)");
//         closeSocket();
//         return false;
//     }
//     if (!local_ip_env) local.sin_addr.s_addr = htonl(INADDR_ANY);
//     local.sin_port = htons(port);
//     if (bind(sock_, (struct sockaddr*)&local, sizeof(local)) < 0) {
//         perror("bind");
//         closeSocket();
//         return false;
//     }

//     memset(&dest_, 0, sizeof(dest_));
//     dest_.sin_family = AF_INET;
//     // Remote/receiver port used by the target remains 3333
//     dest_.sin_port = htons(3333);
//     if (inet_pton(AF_INET, ip, &dest_.sin_addr) <= 0) { perror("inet_pton"); closeSocket(); return false; }

//     struct timeval tv;
//     tv.tv_sec = static_cast<time_t>(rx_timeout_sec);
//     tv.tv_usec = static_cast<suseconds_t>((rx_timeout_sec - tv.tv_sec) * 1e6);
//     if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) { perror("setsockopt"); closeSocket(); return false; }

//     return true;
// }

void simpleEthClient::closeSocket()
{
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
}

bool simpleEthClient::sendRaw(const void *buf, size_t len)
{
    if (sock_ < 0) return false;
    ssize_t s = sendto(sock_, buf, len, 0, (struct sockaddr*)&dest_, sizeof(dest_));
    return (s == (ssize_t)len);
}

bool simpleEthClient::discover()
{
    if (sock_ < 0) return false;
    eOuprot_cmd_DISCOVER_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_LEGACY_SCAN;
    cmd.opc2 = uprot_OPC_DISCOVER;
    cmd.jump2updater = 0;

    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto"); return false; }
    std::cout << "DISCOVER sent, awaiting replies..." << std::endl;

    unsigned char buf[1500];
    while (true)
    {
        socklen_t sl = sizeof(src_);
        ssize_t r = recvfrom(sock_, buf, sizeof(buf), 0, (struct sockaddr*)&src_, &sl);
        if (r < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::cout << "Receive timed out, no more replies." << std::endl;
                break;
            }
            perror("recvfrom");
            return false;
        }

        const char *srcip = inet_ntoa(src_.sin_addr);
        if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY2_t))
        {
            auto *rep2 = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY2_t*>(buf);
            simpleEthClient::print_discover_reply(&rep2->discoveryreply, srcip);
            for (int i = 0; i < 2; ++i)
            {
                const eOuprot_procinfo_t &p = rep2->extraprocs[i];
                std::cout << " ExtraProc[" << i << "] type=" << (int)p.type
                          << " ver=" << (int)p.version.major << "." << (int)p.version.minor
                          << " rom_addr_kb=" << p.rom_addr_kb << " rom_size_kb=" << p.rom_size_kb << std::endl;
            }
        }
        else if ((size_t)r >= sizeof(eOuprot_cmd_MOREINFO_REPLY_t))
        {
            auto *more = reinterpret_cast<eOuprot_cmd_MOREINFO_REPLY_t*>(buf);
            simpleEthClient::print_discover_reply(&more->discover, srcip);
        }
        else if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY_t))
        {
            auto *rep = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY_t*>(buf);
            simpleEthClient::print_discover_reply(rep, srcip);
        }
        else if ((size_t)r >= sizeof(eOuprot_cmd_LEGACY_SCAN_REPLY_t))
        {
            auto *scan = reinterpret_cast<eOuprot_cmd_LEGACY_SCAN_REPLY_t*>(buf);
            print_legacy_scan_reply(scan, srcip);
        }
        else
        {
            std::cout << "Ignored/unknown packet of size " << r << " from " << srcip << " -- raw:";
            for (ssize_t i = 0; i < r; ++i)
            {
                printf(" %02X", buf[i]);
            }
            std::cout << std::endl;
        }
    }
    return true;
}

bool simpleEthClient::jump2updater()
{
    eOuprot_cmd_DISCOVER_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc  = uprot_OPC_LEGACY_SCAN;
    cmd.opc2 = uprot_OPC_DISCOVER;
    cmd.jump2updater = 1;

    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto"); return false; }
    std::cout << "DISCOVER (jump2updater=1) sent to request maintenance mode." << std::endl;
    return true;
}

bool simpleEthClient::def2run_application()
{
    eOuprot_cmd_DEF2RUN_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_DEF2RUN;
    cmd.proc = static_cast<uint8_t>(eApplication);
    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto"); return false; }
    std::cout << "DEF2RUN -> Application sent." << std::endl;
    return true;
}

bool simpleEthClient::restart()
{
    eOuprot_cmd_RESTART_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_RESTART;
    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto"); return false; }
    std::cout << "RESTART sent." << std::endl;
    return true;
}

bool simpleEthClient::blink()
{
    eOuprot_cmd_BLINK_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_BLINK;
    if (!sendRaw(&cmd, sizeof(cmd))) { perror("sendto"); return false; }
    std::cout << "BLINK sent." << std::endl;
    return true;
}

void simpleEthClient::print_discover_reply(const eOuprot_cmd_DISCOVER_REPLY_t *reply, const char *srcip)
{
    if (!reply) return;
    std::cout << "---- Discover reply from " << srcip << " ----" << std::endl;
    std::cout << "Result: " << (int)reply->reply.res
              << "  ProtVer: " << (int)reply->reply.protversion
              << "  sizeofextra: " << (int)reply->reply.sizeofextra << std::endl;

    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             reply->mac48[5], reply->mac48[4], reply->mac48[3],
             reply->mac48[2], reply->mac48[1], reply->mac48[0]);

    // try to obtain a human readable board name from the boardtype value
    const char *raw_board_name = eoboards_type2string2(static_cast<eObrd_type_t>(reply->boardtype), static_cast<eObool_t>(0));
    std::string board_name;
    if (raw_board_name) {
        board_name = raw_board_name;
        const char prefix[] = "eobrd_";
        if (board_name.rfind(prefix, 0) == 0) { // startswith
            board_name = board_name.substr(strlen(prefix));
        }
    }

    std::cout << "MAC: " << mac << "  Type: " << (int)reply->boardtype;
    if (!board_name.empty())
    {
        std::cout << " ( Board Info: " << board_name << " )";
    }
    std::cout << std::endl;

    std::cout << "Running: " << (int)reply->processes.runningnow
              << "  Def2run: " << (int)reply->processes.def2run
              << "  Startup: " << (int)reply->processes.startup
              << "  NumProcesses: " << (int)reply->processes.numberofthem << std::endl;

    std::cout << "Capabilities mask: 0x" << std::hex << reply->capabilities << std::dec << std::endl;

    int num = std::min<int>(reply->processes.numberofthem, 3);
    for (int i = 0; i < num; ++i)
    {
        const eOuprot_procinfo_t &p = reply->processes.info[i];
        std::cout << " Process[" << i << "] type=" << (int)p.type;
        std::cout << "  ver=";
        std::cout << (int)p.version.major << "." << (int)p.version.minor;
        std::cout << "  rom_addr_kb=" << p.rom_addr_kb << "  rom_size_kb=" << p.rom_size_kb << std::endl;
    }

    // if eoboards mapping did not give a name, fall back to boardinfo32 string if present
    if ((!board_name.empty()) && reply->boardinfo32[0] != EOUPROT_VALUE_OF_UNUSED_BYTE)
    {
        uint8_t len = reply->boardinfo32[0];
        std::string info;
        if (len > 0)
        {
            size_t copylen = std::min<size_t>(len, sizeof(reply->boardinfo32)-1);
            info.assign(reinterpret_cast<const char*>(&reply->boardinfo32[1]), copylen);
        }
        if (!info.empty())
        {
            std::cout << "Board info: " << info << std::endl;
        }
    }

    std::cout << "----------------------------------------" << std::endl;
}


void simpleEthClient::print_legacy_scan_reply(const eOuprot_cmd_LEGACY_SCAN_REPLY_t *scan, const char *srcip)
{
    if (!scan) return;
    std::cout << "---- Legacy scan reply from " << srcip << " ----" << std::endl;
    std::cout << "opc: " << (int)scan->opc
              << "  version: " << (int)scan->version.major << "." << (int)scan->version.minor << std::endl;
    char mac[18] = {0};
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             scan->mac48[5], scan->mac48[4], scan->mac48[3],
             scan->mac48[2], scan->mac48[1], scan->mac48[0]);
    std::cout << "MAC: " << mac << std::endl;

    uint32_t mask = 0;
    memcpy(&mask, scan->ipmask, sizeof(mask));
    mask = ntohl(mask);
    uint8_t b0 = (mask >> 24) & 0xFF;
    uint8_t b1 = (mask >> 16) & 0xFF;
    uint8_t b2 = (mask >> 8) & 0xFF;
    uint8_t b3 = (mask >> 0) & 0xFF;
    std::cout << "IP mask (dotted): " << (int)b0 << "." << (int)b1 << "." << (int)b2 << "." << (int)b3
              << "  (raw 0x" << std::hex << mask << std::dec << ")" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}


std::string simpleEthClient::logname(const std::string &ip)
{
    std::string s = ip;
    for (char &c : s) if (c == '.') c = '_';

    // ensure logs are written to the current working directory (not exe dir)
    char cwd_buf[PATH_MAX];
    const char *cwd = getcwd(cwd_buf, sizeof(cwd_buf));
    std::string prefix = (cwd && cwd[0]) ? std::string(cwd) : std::string(".");

    return prefix + "/" + s + ".log";
}

// Ensure the target at `ip` is in maintenance (eUpdater). Logs progress to `log`.
int simpleEthClient::ensureMaintenance(const char *ip, int max_retries, int retry_delay_sec, std::ostream &log)
{
    if (!open(ip, 5.0)) {
        log << ip << ": open() failed: " << strerror(errno) << "\n";
        return 1;
    }

    // initial discover (single-shot command)
    eOuprot_cmd_DISCOVER_t cmd;
    memset(&cmd, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd));
    cmd.opc = uprot_OPC_LEGACY_SCAN;
    cmd.opc2 = uprot_OPC_DISCOVER;
    cmd.jump2updater = 0;
    if (!sendRaw(&cmd, sizeof(cmd))) {
        log << ip << ": sendRaw(discover) failed\n";
        closeSocket();
        return 1;
    }

    // --- Corrected Receive Logic ---
    // Loop to find the most detailed discover reply available until timeout.
    unsigned char buf[1500];
    bool got = false;
    eOuprot_cmd_DISCOVER_REPLY_t discovered = {0};
    bool has_detailed_reply = false;

    while (!has_detailed_reply) {
        socklen_t sl = sizeof(src_);
        ssize_t r = recvfrom(sock_, buf, sizeof(buf), 0, (struct sockaddr*)&src_, &sl);

        if (r < 0) {
            // If we timed out (EAGAIN/EWOULDBLOCK), just break the loop.
            // If we already got a basic reply, we'll use it. Otherwise, it's a failure.
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            // For other errors, log and fail.
            log << ip << ": recvfrom(discover): " << strerror(errno) << "\n";
            break;
        }

        // Ignore packets from other IPs
        if (src_.sin_addr.s_addr != dest_.sin_addr.s_addr) {
            continue;
        }

        // Process the packet, from most detailed to least detailed type
        if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY2_t)) {
            auto *rep2 = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY2_t*>(buf);
            discovered = rep2->discoveryreply;
            got = true;
            has_detailed_reply = true; // This is the best reply, we can stop listening.
        } else if ((size_t)r >= sizeof(eOuprot_cmd_MOREINFO_REPLY_t)) {
            auto *m = reinterpret_cast<eOuprot_cmd_MOREINFO_REPLY_t*>(buf);
            discovered = m->discover;
            got = true;
            has_detailed_reply = true; // This is also a great reply, stop listening.
        } else if ((size_t)r >= sizeof(eOuprot_cmd_DISCOVER_REPLY_t)) {
            // This is a basic reply. We'll take it, but keep listening for a moment
            // in case a more detailed one is coming right after.
            if (!got) { // Only store it if we don't have a better one yet.
                auto *rep = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY_t*>(buf);
                discovered = *rep;
                got = true;
            }
        }
    }
    // --- End of Corrected Receive Logic ---

    // Attempt to get board's application version (if available)
    int board_major = -1, board_minor = -1;
    if (got) {
        // The process table lists available firmwares. Find the one for 'eApplication'.
        int num = std::min<int>(discovered.processes.numberofthem, 3);
        for (int i = 0; i < num; ++i) {
            const eOuprot_procinfo_t &p = discovered.processes.info[i];
            if (p.type == static_cast<uint8_t>(eApplication)) {
                board_major = static_cast<int>(p.version.major);
                board_minor = static_cast<int>(p.version.minor);
                log << ip << ": discovered application version " << board_major << "." << board_minor << "\n";
                break;
            }
        }
    }

    // derive board name to lookup firmware entry (best-effort)
    std::string board_name;
    if (got) {
        const char *raw_board_name = eoboards_type2string2(static_cast<eObrd_type_t>(discovered.boardtype), static_cast<eObool_t>(0));
        if (raw_board_name) {
            board_name = raw_board_name;
            const char prefix[] = "eobrd_";
            if (board_name.rfind(prefix, 0) == 0) board_name = board_name.substr(strlen(prefix));
        }
    }

    // If we have both board name and discovered version, try to read firmware.info.xml version
    int fw_major = -1, fw_minor = -1;
    std::string hexpath;
    bool fw_found = false;
    if (!board_name.empty()) {
        if (findFirmwareForBoardWithVersion(board_name, hexpath, fw_major, fw_minor)) {
            fw_found = true;
            log << ip << ": firmware entry found: " << hexpath << " version="
                << (fw_major >= 0 ? std::to_string(fw_major) : std::string("N/A")) << "."
                << (fw_minor >= 0 ? std::to_string(fw_minor) : std::string("N/A")) << "\n";
        } else {
            log << ip << ": firmware entry not found for board '" << board_name << "'\n";
        }
    } else {
        log << ip << ": cannot derive board name from discover to compare versions\n";
    }

    // If both board and firmware versions are known, compare and skip if up-to-date
    if (board_major >= 0 && fw_major >= 0) {
        // if (board_major > fw_major || (board_major == fw_major && board_minor >= fw_minor)) {
        if (board_major == fw_major && board_minor == fw_minor) {
            log << ip << ": board version " << board_major << "." << board_minor
                << " >= firmware " << fw_major << "." << fw_minor << " -> up-to-date, skipping programming\n";
            closeSocket();
            return 2; // up-to-date, skip programming
        } else {
            log << ip << ": board version " << board_major << "." << board_minor
                << " < firmware " << fw_major << "." << fw_minor << " -> will update\n";
        }
    } else {
        log << ip << ": version comparison not possible (board/fw version missing), proceeding to ensure maintenance\n";
    }

    if (got && discovered.processes.runningnow == eUpdater) {
        log << ip << ": already in maintenance\n";
        closeSocket();
        return 0; // entered maintenance, needs programming
    }
    log << ip << ": not in maintenance (initial check)\n";

    // attempt jump2updater with retries
    for (int attempt = 0; attempt < max_retries; ++attempt) {
        log << ip << ": attempt " << (attempt+1) << "/" << max_retries << " -> jump2updater\n";
        if (!jump2updater()) {
            log << ip << ": jump2updater send failed\n";
        }
        // wait for device to reboot/enter updater
        std::this_thread::sleep_for(std::chrono::seconds(retry_delay_sec));

        // re-discover (single-shot)
        eOuprot_cmd_DISCOVER_t cmd2;
        memset(&cmd2, EOUPROT_VALUE_OF_UNUSED_BYTE, sizeof(cmd2));
        cmd2.opc = uprot_OPC_LEGACY_SCAN;
        cmd2.opc2 = uprot_OPC_DISCOVER;
        cmd2.jump2updater = 0;
        if (!sendRaw(&cmd2, sizeof(cmd2))) {
            log << ip << ": sendRaw(discover2) failed\n";
            continue;
        }

        bool got2 = false;
        socklen_t sl2 = sizeof(src_);
        ssize_t r2 = recvfrom(sock_, buf, sizeof(buf), 0, (struct sockaddr*)&src_, &sl2);
        if (r2 > 0 && src_.sin_addr.s_addr == dest_.sin_addr.s_addr) {
            if ((size_t)r2 >= sizeof(eOuprot_cmd_DISCOVER_REPLY2_t)) {
                auto *rep2 = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY2_t*>(buf);
                discovered = rep2->discoveryreply;
                got2 = true;
            } else if ((size_t)r2 >= sizeof(eOuprot_cmd_MOREINFO_REPLY_t)) {
                auto *m = reinterpret_cast<eOuprot_cmd_MOREINFO_REPLY_t*>(buf);
                discovered = m->discover;
                got2 = true;
            } else if ((size_t)r2 >= sizeof(eOuprot_cmd_DISCOVER_REPLY_t)) {
                auto *rep = reinterpret_cast<eOuprot_cmd_DISCOVER_REPLY_t*>(buf);
                discovered = *rep;
                got2 = true;
            }
        } else if (r2 < 0 && !(errno == EAGAIN || errno == EWOULDBLOCK)) {
            log << ip << ": recvfrom(discover2): " << strerror(errno) << "\n";
        }

        if (got2 && discovered.processes.runningnow == eUpdater) {
            log << ip << ": entered maintenance on attempt " << (attempt+1) << "\n";
            closeSocket();
            return 0; // entered maintenance
        }
        log << ip << ": still not in maintenance after attempt " << (attempt+1) << "\n";
    }

    log << ip << ": failed to enter maintenance after " << max_retries << " attempts\n";
    closeSocket();
    return 1;
}

// parse IPs from network xml (line oriented, inspired by findFirmwareForBoard)
std::vector<std::string> simpleEthClient::parseIPsFromNetworkFile(const char *xmlpath)
{
    std::vector<std::string> ips;
    std::ifstream f(xmlpath);
    if (!f.is_open()) {
        std::cerr << "parseIPsFromNetworkFile: cannot open " << xmlpath << "\n";
        return ips;
    }

    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    f.close();

    {
        size_t pos = 0;
        while ((pos = content.find("<!--", pos)) != std::string::npos) {
            size_t end = content.find("-->", pos + 4);
            if (end == std::string::npos) {
                // unterminated comment: erase from pos to end
                content.erase(pos);
                break;
            }
            content.erase(pos, (end + 3) - pos);
        }
    }

    std::set<std::string> uniq;

    // Find all <ataddress ...> tags, capture attributes block
    std::regex ataddr_re(R"(<\s*ataddress\b([^>]*)>)", std::regex::icase);
    std::sregex_iterator it(content.begin(), content.end(), ataddr_re), end;

    std::regex ip_attr(R"(\bip\s*=\s*['"](\d{1,3}(?:\.\d{1,3}){3})['"])", std::regex::icase);
    std::regex canbus_attr(R"(\bcanbus\s*=)", std::regex::icase);
    std::regex canadr_attr(R"(\bcanadr\s*=)", std::regex::icase);

    for (; it != end; ++it) {
        std::string attrs = (*it)[1].str();
        std::smatch m;
        if (std::regex_search(attrs, m, ip_attr)) {
            // skip if canbus or canadr attribute present (these are CAN sub-entries)
            if (std::regex_search(attrs, canbus_attr) || std::regex_search(attrs, canadr_attr)) {
                continue;
            }
            uniq.insert(m[1].str());
        }
    }

    ips.assign(uniq.begin(), uniq.end());
    return ips;
}

// helper to spawn the current executable with args and redirect child stdout/stderr to a logfile
pid_t simpleEthClient::spawn_and_log(const std::string &exe_path, const std::vector<std::string> &args, const std::string &logpath, bool append)
{
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        // child
        int flags = O_CREAT | O_WRONLY | (append ? O_APPEND : O_TRUNC);
        int fd = ::open(logpath.c_str(), flags, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > 2) ::close(fd);
        }
        // build argv
        std::vector<char*> argv;
        argv.reserve(args.size() + 2);
        argv.push_back(const_cast<char*>(exe_path.c_str()));
        for (const auto &a : args) argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);
        execv(exe_path.c_str(), argv.data());
        // if exec fails
        _exit(127);
    }
    // parent returns child's pid
    return pid;
}


// Orchestrator: parse network file, prepare boards in parallel, program prepared boards in parallel.
int simpleEthClient::orchestrateParallelProgram()
{
    // Require the shell script to provide the network file path via NETWORK_XML.
    const char *network_xml = getenv("NETWORK_XML");
    if (network_xml == nullptr || network_xml[0] == '\0') {
        std::cerr << "ERROR: NETWORK_XML environment variable not set. Aborting.\n";
        return 1;
    }
    //std::cout << "Using network file: " << network_xml << std::endl;
    auto ips = simpleEthClient::parseIPsFromNetworkFile(network_xml);
    if (ips.empty()) {
        std::cerr << "No IPs found in " << network_xml << std::endl;
        return 1;
    }

    std::cout << "Found " << ips.size() << " unique IP(s) to process\n";

    // find current executable path
    char exe_buf[PATH_MAX];
    ssize_t elen = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf)-1);
    std::string exe_path;
    if (elen > 0) { exe_buf[elen] = '\0'; exe_path = exe_buf; }
    else { std::cerr << "Cannot resolve /proc/self/exe; aborting\n"; return 4; }

    // Preparation phase (spawn processes to capture full output)
    std::vector<pid_t> prep_pids;
    std::vector<std::string> prep_logs;
    for (const auto &ip : ips) {
        std::string logfile = simpleEthClient::logname(ip);
        // spawn prepare helper: program will handle prepare when called with "prepare_ip"
        // truncate logfile so it's fresh for this run
        pid_t pid = simpleEthClient::spawn_and_log(exe_path, { "prepare_ip", ip }, logfile, /*append=*/false);
        if (pid > 0) { prep_pids.push_back(pid); prep_logs.push_back(logfile); }
        else std::cerr << "Failed to spawn prepare process for " << ip << std::endl;
    }
    // wait for prep processes and collect results
    std::vector<std::string> prepared;      // need programming
    std::vector<std::string> not_prepared;  // failed to prepare
    std::vector<std::string> up_to_date;    // skipped (already up-to-date)
    for (size_t i = 0; i < prep_pids.size(); ++i) {
        int status = 0;
        pid_t w = waitpid(prep_pids[i], &status, 0);
        (void)w;
        if (WIFEXITED(status)) {
            int code = WEXITSTATUS(status);
            if (code == 0) {
                prepared.push_back(ips[i]); // needs programming
            } else if (code == 2) {
                up_to_date.push_back(ips[i]); // skip programming
            } else {
                not_prepared.push_back(ips[i]);
            }
        } else {
            not_prepared.push_back(ips[i]);
        }
    }

    // Report preparation summary
    std::cout << "Preparation complete. prepared=" << prepared.size()
              << " up_to_date=" << up_to_date.size()
              << " not_prepared=" << not_prepared.size() << "\n";
    if (!not_prepared.empty()) {
        std::cout << "Not prepared boards (check logs for errors):\n";
        for (auto &ip : not_prepared) std::cout << "  " << ip << "\n";
    }
    if (!up_to_date.empty()) {
        std::cout << "Up-to-date boards (skipped programming):\n";
        for (auto &ip : up_to_date) std::cout << "  " << ip << "\n";
    }

    // Check the outcome of the preparation phase and provide a clear summary.
    if (prepared.empty()) {
        if (!up_to_date.empty() && not_prepared.empty()) {
            // This is the ideal success case where no work was needed.
            std::cout << "\nSuccess: All boards are already up-to-date. No programming was necessary." << std::endl;
            return 0; // Return success.
        } else {
            // This is a failure case where some boards failed and none could be prepared.
            std::cerr << "\nFinished: No boards were prepared for programming. Check logs for details on failed boards." << std::endl;
            return 2; // Return an error code indicating nothing was programmed.
        }
    }

    std::cout << "\nProceeding to program " << prepared.size() << " board(s) that require an update...\n";

    // Programming phase (spawn processes and append to the same per-ip log)
    std::vector<pid_t> prog_pids;
    std::vector<std::string> prog_logs;
    for (const auto &ip : prepared) {
        std::string logfile = simpleEthClient::logname(ip);
        pid_t pid = simpleEthClient::spawn_and_log(exe_path, { "program_ip", ip }, logfile, /*append=*/true);
        if (pid > 0) { prog_pids.push_back(pid); prog_logs.push_back(logfile); }
        else std::cerr << "Failed to spawn program process for " << ip << std::endl;
    }
    std::vector<std::string> prog_ok;
    std::vector<std::string> prog_failed;
    for (size_t i = 0; i < prog_pids.size(); ++i) {
        int status = 0;
        pid_t w = waitpid(prog_pids[i], &status, 0);
        (void)w;
        if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
            prog_ok.push_back(prepared[i]);
        } else {
            prog_failed.push_back(prepared[i]);
        }
    }

    // Final report
    std::cout << "Programming summary: success=" << prog_ok.size() << " failed=" << prog_failed.size() << "\n";
    if (!prog_failed.empty()) {
        std::cout << "Programming failed for:\n";
        for (auto &ip : prog_failed) std::cout << "  " << ip << "\n";
    }
    if (!not_prepared.empty()) {
        std::cout << "Boards not prepared (skipped programming):\n";
        for (auto &ip : not_prepared) std::cout << "  " << ip << "\n";
    }

    // --- Restart phase: restart all successfully programmed boards ---
    if (!prog_ok.empty()) {
        std::cout << "Restarting successfully programmed boards (" << prog_ok.size() << ")...\n";
        std::vector<pid_t> restart_pids;
        std::vector<std::string> restart_logs;
        // spawn restart children (append to same per-ip log)
        for (const auto &ip : prog_ok) {
            std::string logfile = simpleEthClient::logname(ip);
            pid_t pid = simpleEthClient::spawn_and_log(exe_path, { "restart_ip", ip }, logfile, /*append=*/true);
            if (pid > 0) { restart_pids.push_back(pid); restart_logs.push_back(logfile); }
            else std::cerr << "Failed to spawn restart process for " << ip << std::endl;
        }

        std::vector<std::string> restart_ok, restart_failed;
        for (size_t i = 0; i < restart_pids.size(); ++i) {
            int status = 0;
            waitpid(restart_pids[i], &status, 0);
            if (WIFEXITED(status) && WEXITSTATUS(status) == 0) restart_ok.push_back(prog_ok[i]);
            else restart_failed.push_back(prog_ok[i]);
        }

        std::cout << "Restart summary: success=" << restart_ok.size() << " failed=" << restart_failed.size() << "\n";
        if (!restart_failed.empty()) {
            std::cout << "Restart failed for:\n";
            for (auto &ip : restart_failed) std::cout << "  " << ip << "\n";
        }
    }

    return prog_failed.empty() ? 0 : 3;
}


int main(int argc, char *argv[])
{
    // preserve existing helper child modes (exact same checks as before)
    if (argc == 3 && std::string(argv[1]) == "prepare_ip") {
        const char *ip = argv[2];
        simpleEthClient client;
        int result = client.ensureMaintenance(ip, 4, 5, std::cout);
        return result;
    }
    if (argc == 3 && std::string(argv[1]) == "program_ip") {
        const char *ip = argv[2];
        simpleEthClient client;
        if (!client.open(ip, 5.0)) return 2;
        bool ok = client.program();
        return ok ? 0 : 1;
    }
    if (argc == 3 && std::string(argv[1]) == "restart_ip") {
        const char *ip = argv[2];
        simpleEthClient client;
        if (!client.open(ip, 5.0)) {
            std::cerr << ip << ": open failed for restart\n";
            return 2;
        }
        client.def2run_application();
        sleep(1);
        if (!client.restart()) {
            std::cerr << ip << ": restart failed" << std::endl;
            return 1;
        }
        std::cout << ip << ": restart succeeded\n";
        return 0;
    }

    // Support single-argument orchestrator mode (unchanged)
    if (argc == 2) {
        std::string single = argv[1];
        if (single == "parallel_update" || single == "parallel_program") {
            return simpleEthClient::orchestrateParallelProgram();
        }
    }

    // Flexible parsing: accept either "<ip> <cmd>" or "<cmd> <ip>"
    if (argc >= 3) {
        const char *a = argv[1];
        const char *b = argv[2];
        auto is_ip = [](const char *s)->bool {
            if (!s) return false;
            struct in_addr ina;
            return inet_pton(AF_INET, s, &ina) == 1;
        };

        const char *ip = nullptr;
        const char *cmd = nullptr;
        if (is_ip(a)) {
            ip = a; cmd = b;
        } else if (is_ip(b)) {
            ip = b; cmd = a;
        } else {
            std::cerr << "Usage: ./yafu <ip> <command>  OR  ./yafu <command> <ip>\n";
            std::cerr << "Commands: discover, maintenance|jump2updater, application|def2run_application, restart, blink, program\n";
            return 1;
        }

        simpleEthClient client;
        if (!client.open(ip, 5.0)) return 1;

        std::string scmd(cmd);
        if (scmd == "discover") {
            client.discover();
        } else if (scmd == "maintenance" || scmd == "jump2updater") {
            client.jump2updater();
        } else if (scmd == "application" || scmd == "def2run_application") {
            client.def2run_application();
            sleep(1);
            client.restart();
        } else if (scmd == "restart") {
            client.restart();
        } else if (scmd == "blink") {
            client.blink();
        } else if (scmd == "program") {
            if (!client.program()) {
                std::cerr << "Programming failed\n";
                return 1;
            }
        } else {
            std::cerr << "Unknown command: " << scmd << std::endl;
            return 1;
        }
        return 0;
    }

    std::cerr << "Usage: ./yafu <ip> <command>  OR  ./yafu <command> <ip>\n";
    return 1;
}
