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
 int  prot_vers_minor;      // the minor number of the can protocol of the application. not meaningful for bootloader
 char serial [50];          // only for strain
 int  strainregsetinuse;    // only for strain
 int  strainregsetatboot;   // only for strain
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

#include <iCubCanProto_types.h>

// it forces the use of the new driver2 interface
#define DOWNLOADER_USE_IDRIVER2


#if defined(DOWNLOADER_USE_IDRIVER2)
// if you define it, the canLoader shall be able to retrieve can boards from both CAN1 and CAN2
#define DOWNLOADER_ETH_SUPPORTS_MULTIBUS
#endif


struct strain2_ampl_regs_t
{
   uint8_t data[6];
   strain2_ampl_regs_t() { data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0; }
   void load(void *mem, size_t s) { memmove(data, mem, sizeof(data)); }
   void * memory() { return data; }
   size_t size() { return 6; }
};

enum strain2_ampl_discretegain_t
{
    ampl_gain48 = 0, ampl_gain36 = 1, ampl_gain24 = 2, ampl_gain20 = 3, ampl_gain16 = 4,
    ampl_gain10 = 5, ampl_gain08 = 6, ampl_gain06 = 7, ampl_gain04 = 8
};


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

    bool strain_is_acquiring_in_calibratedmode;

private:
int download_motorola_line(char* line, int len, int bus, int board_pid);
int download_hexintel_line(char* line, int len, int bus, int board_pid, bool eeprom, int board_type);

int build_id(int source, int dest);
int get_src_from_id (int id);
int get_dst_from_id (int id);

int verify_ack(int command, int read_messages);


enum { ampl_gain_numberOf = 9 };


public:

enum { strain_regset_inuse = 0, strain_regset_one = 1, strain_regset_two = 2, strain_regset_three = 3 };
enum { strain_regsetmode_temporary = 0, strain_regsetmode_permanent = 1 };

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

float strain_amplifier_discretegain2float(strain2_ampl_discretegain_t c);

// the strain2 has multiple (up to 3) regulation sets. with these functions we can get / set the regulation set in use inside the strain2.
// allowed values for regset are only strain_regset_one/two/three. with regsetmode we choose if the value is the one currently used or the one in eeprom.
// attention: funtion strain_set_regulationset() does not want strain_regset_inuse but only strain_regset_one/two/three
int strain_set_regulationset        (int bus, int target_id, int regset = strain_regset_one, int regsetmode = strain_regsetmode_temporary, string *errorstring = NULL);
int strain_get_regulationset        (int bus, int target_id, int &regset, const int regsetmode = strain_regsetmode_temporary, string *errorstring = NULL);


// the calibration of the offset is meaningful only for the calibration set in use
int strain_calibrate_offset  (int bus, int target_id, icubCanProto_boardType_t boardtype, unsigned int middle_val, string *errorstring = NULL);

int strain_calibrate_offset2 (int bus, int target_id, icubCanProto_boardType_t boardtype, const std::vector<strain2_ampl_discretegain_t> &gains, const std::vector<int16_t> &targets, string *errorstring = NULL);

// they are not dependent on the regulation set or we cannot specify one
int strain_get_adc			 (int bus, int target_id, char channel, unsigned int& adc, int type, string *errorstring = NULL);
int strain_get_eeprom_saved  (int bus, int target_id, bool* status, string *errorstring = NULL);
int strain_save_to_eeprom    (int bus, int target_id, string *errorstring = NULL);
// funtions *curr_bias() apply only to strain_regset_inuse because the curr bias is not part of the regulation set. it is a volatile regulation.
int strain_get_curr_bias	 (int bus, int target_id, char channel, signed int& bias, string *errorstring = NULL);
int strain_set_curr_bias	 (int bus, int target_id, string *errorstring = NULL); // only for strain_regset_inuse because the curr bias is not saved in eeprom in the regulation set
int strain_set_curr_bias	 (int bus, int target_id, char channel, signed int bias, string *errorstring = NULL); // only for strain_regset_inuse because the curr bias is not saved in eeprom in the regulation set
int strain_reset_curr_bias	 (int bus, int target_id, string *errorstring = NULL); // only for strain_regset_inuse because the curr bias is not saved in eeprom in the regulation set
// serial number is unique and does not depend on regulation set
int strain_get_serial_number (int bus, int target_id, char* serial_number, string *errorstring = NULL);
int strain_set_serial_number (int bus, int target_id, const char* serial_number, string *errorstring = NULL);


// all of the following can be related to any calibration set: the one in use or one of set 1, 2, 3. default is the one in use (old strain have only one regulation set)
int strain_get_offset		 (int bus, int target_id, char channel, unsigned int& offset, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_set_offset		 (int bus, int target_id, char channel, unsigned int  offset, int regset = strain_regset_inuse, string *errorstring = NULL);

int strain_get_matrix_rc	 (int bus, int target_id, char r, char c, unsigned int& elem, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_set_matrix_rc	 (int bus, int target_id, char r, char c, unsigned int  elem, int regset = strain_regset_inuse, string *errorstring = NULL);

int strain_get_matrix_gain	 (int bus, int target_id, unsigned int& gain, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_set_matrix_gain	 (int bus, int target_id, unsigned int  gain, int regset = strain_regset_inuse, string *errorstring = NULL);

int strain_set_amplifier_regs(int bus, int target_id, unsigned char channel, const strain2_ampl_regs_t &ampregs, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_get_amplifier_regs(int bus, int target_id, unsigned char channel, strain2_ampl_regs_t &ampregs, int regset = strain_regset_inuse, string *errorstring = NULL);

int strain_set_amplifier_discretegain(int bus, int target_id, unsigned char channel, strain2_ampl_discretegain_t ampset, int regset = strain_regset_inuse, string *errorstring = NULL);

int strain_get_amplifier_gain_offset(int bus, int target_id, unsigned char channel, float &gain, uint16_t &offset, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_set_amplifier_gain_offset(int bus, int target_id, unsigned char channel, float gain, uint16_t offset, int regset = strain_regset_inuse, string *errorstring = NULL);

int strain_get_calib_bias	 (int bus, int target_id, char channel, signed int& bias, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_set_calib_bias	 (int bus, int target_id, string *errorstring = NULL);  // used only for strain_regset_inuse
int strain_set_calib_bias	 (int bus, int target_id, char channel, signed int bias, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_reset_calib_bias	 (int bus, int target_id, string *errorstring = NULL);  // used only for strain_regset_inuse

int strain_set_full_scale	 (int bus, int target_id, unsigned char channel, unsigned int   full_scale, int regset = strain_regset_inuse, string *errorstring = NULL);
int strain_get_full_scale	 (int bus, int target_id, unsigned char channel, unsigned int&  full_scale, int regset = strain_regset_inuse, string *errorstring = NULL);






// for use by the future strain calibration data acquisition gui

struct strain_value_t
{
    bool valid;                                             // the acquisition is meaninful
    bool calibrated;                                        // the values are calibrated
    bool saturated;                                         // at least one measure is saturated. see saturationinfo[6] to know more
    unsigned int channel[6];                                // the values
    icubCanProto_strain_saturationInfo_t saturationinfo[6]; // the saturation info
    strain_value_t() : calibrated(false), valid(false), saturated(false) {
        channel[0] = channel[1] = channel[2] = channel[3] = channel[4] = channel[5] = 0;
        saturationinfo[0] = saturationinfo[1] = saturationinfo[2] = saturationinfo[3] = saturationinfo[4] = saturationinfo[5] = saturationNONE;
    }
    void extract(signed short *ss6) const {
        if(NULL == ss6) return;
        for(size_t i=0; i<6; i++) { ss6[i] = static_cast<unsigned short>(channel[i]) - 0x7fff; }
    }
};

typedef enum 
{
    strain_acquisition_mode_streaming   = 0,
    strain_acquisition_mode_polling    = 1
} strain_acquisition_mode_t;

int strain_acquire_start(int bus, int target_id, uint8_t txratemilli = 20, bool calibmode = true, strain_acquisition_mode_t acqmode = strain_acquisition_mode_streaming, string *errorstring = NULL);
int strain_acquire_get(int bus, int target_id, vector<strain_value_t> &values, const unsigned int howmany = 10, void (*updateProgressBar)(void*, float) = NULL, void *arg = NULL, strain_acquisition_mode_t acqmode = strain_acquisition_mode_streaming, const unsigned int maxerrors = 1, string *errorstring = NULL);
int strain_acquire_stop(int bus, int target_id, strain_acquisition_mode_t acqmode = strain_acquisition_mode_streaming, string *errorstring = NULL);

cDownloader(bool verbose = true);

void set_verbose(bool verbose);

void set_external_logger(void *caller = NULL, void (*logger)(void *, const std::string &) = NULL);


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
    
    
    int strain_calibrate_offset2_strain1(int bus, int target_id, int16_t t, string *errorstring);
    int strain_calibrate_offset2_strain1safer(int bus, int target_id, int16_t t, uint8_t nmeasures, bool fullsearch, string *errorstring);
    int strain_calibrate_offset2_strain2(int bus, int target_id, const std::vector<strain2_ampl_discretegain_t> &gains, const std::vector<int16_t> &targets, string *errorstring = NULL);
    
    int readADC(int bus, int target_id, int channel, int nmeasures = 2);

    void (*_externalLoggerFptr)(void *caller, const std::string &output);
    void * _externalLoggerCaller;

    void Log(const std::string &msg);
    
};

#endif

// eof

