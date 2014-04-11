// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef DOWNLOADER_H
#define DOWNLOADER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/CanBusInterface.h>

#include <fstream>
#include "stdint.h"
#include "iCubCanProtocol.h"
#include "iCubCanProto_types.h"

//*****************************************************************/

struct sBoard
{
public:
 int  pid;
 int  type;
 int  version;
 int  release;
 int  build;
 char serial [50];
 int  status;
 bool selected;
 bool eeprom;
 char add_info [50];
};
//*****************************************************************/

#define BOARD_RUNNING       0
#define BOARD_WAITING       1
#define BOARD_WAITING_ACK   2
#define BOARD_DOWNLOADING   3
#define BOARD_OK            4
#define BOARD_ERR           5

#define SPRS_STATE_WAIT           0
#define SPRS_STATE_TYPE           1
#define SPRS_STATE_ADDRESS        2
#define SPRS_STATE_LENGTH         3
#define SPRS_STATE_DATA           4
#define SPRS_STATE_CHECKSUM       5

#define SPRS_TYPE_0 '0'
#define SPRS_TYPE_1 '1'
#define SPRS_TYPE_3 '3'   
#define SPRS_TYPE_4 '4'
#define SPRS_TYPE_7 '7'

#define ID_CMD         ICUBCANPROTO_CLASS_BOOTLOADER
#define ID_MASTER      0x00
#define ID_BROADCAST   0x0F

//#define BOARD_TYPE_DSP    icubCanProto_boardType__dsp
//#define BOARD_TYPE_PIC    icubCanProto_boardType__pic
//#define BOARD_TYPE_2DC    icubCanProto_boardType__2dc
//#define BOARD_TYPE_4DC    icubCanProto_boardType__4dc
//#define BOARD_TYPE_BLL    icubCanProto_boardType__bll
//#define BOARD_TYPE_SKIN   icubCanProto_boardType__skin
//#define BOARD_TYPE_STRAIN icubCanProto_boardType__strain
//#define BOARD_TYPE_MAIS   icubCanProto_boardType__mais
//#define BOARD_TYPE_2FOC   icubCanProto_boardType__2foc
//#define BOARD_TYPE_6SG    icubCanProto_boardType__6sg
#define BOARD_TYPE_JOG      0x0A
#define BOARD_UNKNOWN       icubCanProto_boardType__unknown


#define CMD_BOARD           ICUBCANPROTO_BL_BOARD
#define CMD_ADDRESS         ICUBCANPROTO_BL_ADDRESS
#define CMD_START           ICUBCANPROTO_BL_START
#define CMD_DATA            ICUBCANPROTO_BL_DATA
#define CMD_END             ICUBCANPROTO_BL_END
#define CMD_ERR             ICUBCANPROTO_BL_ERROR
#define CMD_BROADCAST       ICUBCANPROTO_BL_BROADCAST

#define CAN_SET_BOARD_ID    ICUBCANPROTO_POL_MC_CMD__SET_BOARD_ID
#define CAN_GET_BOARD_ID    ICUBCANPROTO_POL_MC_CMD__GET_BOARD_ID
//#define CAN_GET_ADDITIONAL_INFO       ICUBCANPROTO_POL_MC_CMD__GET_ADDITIONAL_INFO
//#define CAN_SET_ADDITIONAL_INFO		ICUBCANPROTO_POL_MC_CMD__SET_ADDITIONAL_INFO

//*****************************************************************/

void drv_sleep (double time);

//*****************************************************************/

class iDriver;

class cDownloader
{
    iDriver* m_candriver;
    yarp::dev::CanBuffer txBuffer;
    yarp::dev::CanBuffer rxBuffer;
    int canbus_id;

private:
int download_motorola_line(char* line, int len, int board_pid);
int download_hexintel_line(char* line, int len, int board_pid, bool eeprom, int board_type);

int build_id(int source, int dest);
int get_src_from_id (int id);
int get_dst_from_id (int id);
int verify_ack(int command, yarp::dev::CanBuffer rx_message, int read_messages);

public:
sBoard*            board_list;
int                board_list_size;
int				   progress;
int				   file_length;
unsigned int       sprsPage;
 std::fstream            filestr;
 int nSelectedBoards; 

bool    connected;
int initdriver(yarp::os::Searchable &config);
int stopdriver();

int initschede			();
int startscheda			(int board_pid,bool board_eeprom, int download_type);
int stopscheda			(int board_pid);
int download_file		(int board_pid, int download_type, bool eeprom);
int open_file			(std::string file);
int change_card_address	(int target_id, int new_id, int board_type);
int change_board_info	(int target_id, char* board_info);
int get_board_info		(int target_id, char* board_info);
int get_serial_no		(int target_id, char* board_info);
int get_canbus_id       ();

int strain_start_sampling    (int target_id);
int strain_stop_sampling     (int target_id);
int strain_calibrate_offset  (int target_id, unsigned int middle_val);
int strain_get_offset		 (int target_id, char channel, unsigned int& offset);
int strain_set_offset		 (int target_id, char channel, unsigned int  offset);
int strain_get_adc			 (int target_id, char channel, unsigned int& adc, int type);
int strain_save_to_eeprom    (int target_id);
int strain_get_matrix_rc	 (int target_id, char r, char c, unsigned int& elem);
int strain_set_matrix_rc	 (int target_id, char r, char c, unsigned int  elem);
int strain_get_matrix_gain	 (int target_id, unsigned int& gain);
int strain_set_matrix_gain	 (int target_id, unsigned int  gain);
int strain_get_calib_bias	 (int target_id, char channel, signed int& bias);
int strain_set_calib_bias	 (int target_id);
int strain_set_calib_bias	 (int target_id, char channel, signed int bias);
int strain_reset_calib_bias	 (int target_id);
int strain_get_curr_bias	 (int target_id, char channel, signed int& bias);
int strain_set_curr_bias	 (int target_id);
int strain_set_curr_bias	 (int target_id, char channel, signed int bias);
int strain_reset_curr_bias	 (int target_id);
int strain_set_full_scale	 (int target_id, unsigned char channel, unsigned int   full_scale);
int strain_get_full_scale	 (int target_id, unsigned char channel, unsigned int&  full_scale);
int strain_get_serial_number (int target_id, char* serial_number);
int strain_set_serial_number (int target_id, const char* serial_number);
int strain_get_eeprom_saved  (int target_id, bool* status);

int sg6_get_amp_gain      (int target_id, char channel, unsigned int& gain1, unsigned int& gain2 );
int sg6_set_amp_gain      (int target_id, char channel, unsigned int  gain1, unsigned int  gain2 );


cDownloader();
};

#endif
