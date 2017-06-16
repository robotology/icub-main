// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo, Marco Accame
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef DOWNLOADER_H
#define DOWNLOADER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/CanBusInterface.h>

#include <fstream>
#include "stdint.h"


//*****************************************************************/

struct sBoard
{
public:
 int  bus;                  // the can bus
 int  pid;                  // the id inside the can bus
 int  type;                 // the board type. use macros ICUBCANPROTO_BOARDTYPE__DSP etc. or .. eObrd_type_t 
 bool applicationisrunning; // if true the board runs the application. if false it runs the bootloader
 int  appl_vers_major;      // the major number of the version of the sw it is running (former ...)
 int  appl_vers_minor;      // the minor number of the version of the sw it is running (former ...)
 int  appl_vers_build;      // the build number of the version of the sw it is running (former ...). not meaningful for bootloader
 int  prot_vers_major;      // the major number of the can protocol of the application. not meaningful for bootloader
 int  prot_vers_minor;      // the mimnor number of the can protocol of the application. not meaningful for bootloader
 char serial [50];
 int  status;
 bool selected;
 bool eeprom;
 char add_info [50];    // but can protocol supports upt to 32 bytes
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

#define ID_MASTER      0x00
#define ID_BROADCAST   0x0F


//*****************************************************************/

void drv_sleep (double time);

//*****************************************************************/



#include "driver.h"

#include "EoBoards.h"
#include "EoCommon.h"

#include <canProtocolLib/iCubCanProto_types.h>

// it forces the use of the new driver2 interface
#define DOWNLOADER_USE_IDRIVER2


#if defined(DOWNLOADER_USE_IDRIVER2)
// if you define it, the canLoader shall be able to retrieve can boards from both CAN1 and CAN2
#define DOWNLOADER_ETH_SUPPORTS_MULTIBUS
#endif

// marco.accame: later on add inclusion of EoBoards.[h, c] and change types eObrd_D_* with eObrd_*.
// this change has the advantage of providing a unique definition of board types, strings associated, etc.
// whihc is is the same alredy used in diagnostics of ETH boards.

#if 0
typedef enum
{
    eobrd_D_cantype_dsp               = ICUBCANPROTO_BOARDTYPE__DSP,      // 0, not used
    eobrd_D_cantype_pic               = ICUBCANPROTO_BOARDTYPE__PIC,      // 2, not used
    eobrd_D_cantype_2dc               = ICUBCANPROTO_BOARDTYPE__2DC,      // 2, not used
    eobrd_D_cantype_mc4               = ICUBCANPROTO_BOARDTYPE__4DC,      // 3
    eobrd_D_cantype_bll               = ICUBCANPROTO_BOARDTYPE__BLL,      // 4, not used
    eobrd_D_cantype_mtb               = ICUBCANPROTO_BOARDTYPE__SKIN,     // 5
    eobrd_D_cantype_strain            = ICUBCANPROTO_BOARDTYPE__STRAIN,   // 6
    eobrd_D_cantype_mais              = ICUBCANPROTO_BOARDTYPE__MAIS,     // 7
    eobrd_D_cantype_foc               = ICUBCANPROTO_BOARDTYPE__2FOC,     // 8
    eobrd_D_cantype_6sg               = ICUBCANPROTO_BOARDTYPE__6SG,      // 9, not used
    eobrd_D_cantype_jog               = ICUBCANPROTO_BOARDTYPE__JOG,      // 10, not used

    eobrd_D_cantype_none              = 254,
    eobrd_D_cantype_unknown           = ICUBCANPROTO_BOARDTYPE__UNKNOWN   // 255
} eObrd_D_cantype_t;

typedef struct
{   // size is: 1+1+0 = 2
    uint8_t                     major;
    uint8_t                     minor;
    uint8_t                     build;
} eObrd_D_firmwareversion_t;


typedef struct
{   // size is: 1+1+0 = 2
    uint8_t                     major;
    uint8_t                     minor;
} eObrd_D_protocolversion_t;


typedef struct
{   // size is: 1+1+2+2+0 = 6
    uint8_t                     type;
    eObrd_D_firmwareversion_t   firmware;
    eObrd_D_protocolversion_t   protocol;
} eObrd_D_info_t;

#endif


class cDownloader
{

private:

#if defined(DOWNLOADER_USE_IDRIVER2)
    iDriver2* m_idriver;
    vector<CanPacket> txBuffer;
    vector<CanPacket> rxBuffer;
#else
    iDriver* m_idriver;
    yarp::dev::CanBuffer txBuffer;
    yarp::dev::CanBuffer rxBuffer;
#endif

    // the id of the can bus is moved inside the struct sBoard because we want to manage boards from different CAN buses.
    int canbus_id;

private:
int download_motorola_line(char* line, int len, int bus, int board_pid);
int download_hexintel_line(char* line, int len, int bus, int board_pid, bool eeprom, int board_type);

int build_id(int source, int dest);
int get_src_from_id (int id);
int get_dst_from_id (int id);

int verify_ack(int command, int read_messages);



public:
sBoard*            board_list;
int                board_list_size;
int				   progress;
int				   file_length;
unsigned int       sprsPage;
 std::fstream            filestr;
 int nSelectedBoards; 

bool    connected;
int initdriver(yarp::os::Searchable &config, bool verbose = true);
int stopdriver();

int initschede			();
int startscheda			(int bus, int board_pid, bool board_eeprom, int download_type);
int stopscheda			(int bus, int board_pid);
int download_file		(int bus, int board_pid, int download_type, bool eeprom);
int open_file			(std::string file);
int change_card_address	(int bus, int target_id, int new_id, int board_type);
int change_board_info	(int bus, int target_id, char* board_info);
int get_board_info		(int bus, int target_id, char* board_info);
int get_serial_no		(int bus, int target_id, char* board_info);
int get_firmware_version(int bus, int target_id, eObrd_cantype_t boardtype, eObrd_info_t *info, bool &noreply);
int get_canbus_id       ();
void set_canbus_id      (int id);

int strain_start_sampling    (int bus, int target_id, string *errorstring = NULL);
int strain_stop_sampling     (int bus, int target_id, string *errorstring = NULL);
int strain_calibrate_offset  (int bus, int target_id, unsigned int middle_val, string *errorstring = NULL);
int strain_get_offset		 (int bus, int target_id, char channel, unsigned int& offset, string *errorstring = NULL);
int strain_set_offset		 (int bus, int target_id, char channel, unsigned int  offset, string *errorstring = NULL);
int strain_get_adc			 (int bus, int target_id, char channel, unsigned int& adc, int type, string *errorstring = NULL);
int strain_save_to_eeprom    (int bus, int target_id, string *errorstring = NULL);
int strain_get_matrix_rc	 (int bus, int target_id, char r, char c, unsigned int& elem, int matrix = 0, string *errorstring = NULL);
int strain_set_matrix_rc	 (int bus, int target_id, char r, char c, unsigned int  elem, int matrix = 0, string *errorstring = NULL);
int strain_get_matrix_gain	 (int bus, int target_id, unsigned int& gain, int matrix = 0, string *errorstring = NULL);
int strain_set_matrix_gain	 (int bus, int target_id, unsigned int  gain, int matrix = 0, string *errorstring = NULL);
int strain_get_calib_bias	 (int bus, int target_id, char channel, signed int& bias, string *errorstring = NULL);
int strain_set_calib_bias	 (int bus, int target_id, string *errorstring = NULL);
int strain_set_calib_bias	 (int bus, int target_id, char channel, signed int bias, string *errorstring = NULL);
int strain_reset_calib_bias	 (int bus, int target_id, string *errorstring = NULL);
int strain_get_curr_bias	 (int bus, int target_id, char channel, signed int& bias, string *errorstring = NULL);
int strain_set_curr_bias	 (int bus, int target_id, string *errorstring = NULL);
int strain_set_curr_bias	 (int bus, int target_id, char channel, signed int bias, string *errorstring = NULL);
int strain_reset_curr_bias	 (int bus, int target_id, string *errorstring = NULL);
int strain_set_full_scale	 (int bus, int target_id, unsigned char channel, unsigned int   full_scale, int matrix = 0, string *errorstring = NULL);
int strain_get_full_scale	 (int bus, int target_id, unsigned char channel, unsigned int&  full_scale, int matrix = 0, string *errorstring = NULL);
int strain_get_serial_number (int bus, int target_id, char* serial_number, string *errorstring = NULL);
int strain_set_serial_number (int bus, int target_id, const char* serial_number, string *errorstring = NULL);
int strain_get_eeprom_saved  (int bus, int target_id, bool* status, string *errorstring = NULL);
int strain_set_matrix        (int bus, int target_id, int matrix = 0, string *errorstring = NULL);
int strain_get_matrix        (int bus, int target_id, int &matrix, string *errorstring = NULL);

int sg6_get_amp_gain      (int bus, int target_id, char channel, unsigned int& gain1, unsigned int& gain2 );
int sg6_set_amp_gain      (int bus, int target_id, char channel, unsigned int  gain1, unsigned int  gain2 );


cDownloader(bool verbose = true);

bool set_verbose(bool verbose);


private:
    void clean_rx(void);

#if defined(DOWNLOADER_USE_IDRIVER2)
    void set_bus(CanPacket &pkt, int bus);
    int get_bus(CanPacket &pkt);
#else
    void set_bus(yarp::dev::CanMessage &msg, int bus);
    int get_bus(yarp::dev::CanMessage &msg);
#endif

    bool _verbose;
};

#endif

// eof

