#ifndef DOWNLOADER_H
#define DOWNLOADER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/CanBusInterface.h>
#include <ace/streams.h>

#include "driver.h"

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

#define ID_CMD         0x07
#define ID_MASTER      0x00
#define ID_BROADCAST   0x0F

#define BOARD_TYPE_DSP    0x00
#define BOARD_TYPE_PIC    0x01
#define BOARD_TYPE_2DC    0x02
#define BOARD_TYPE_4DC    0x03
#define BOARD_TYPE_BLL    0x04
#define BOARD_TYPE_SKIN   0x05
#define BOARD_TYPE_STRAIN 0x06
#define BOARD_TYPE_MAIS   0x07
#define BOARD_UNKNOWN     0xFF


#define CMD_BOARD 	   0x00
#define CMD_ADDRESS    0x01
#define CMD_START	   0x02
#define CMD_DATA	   0x03
#define CMD_END	       0x04
#define CMD_ERR	       0x05
#define CMD_BROADCAST  0xFF

#define CAN_SET_BOARD_ID	50
#define CAN_GET_BOARD_ID	51
#define CAN_GET_ADDITIONAL_INFO		12
#define CAN_SET_ADDITIONAL_INFO		13

//*****************************************************************/

void drv_sleep (double time);

//*****************************************************************/
class cDownloader
{
    cDriver* m_candriver;
    yarp::dev::CanBuffer txBuffer;
    yarp::dev::CanBuffer rxBuffer;
	
private:
int download_motorola_line(char* line, int len, int board_pid);
int download_hexintel_line(char* line, int len, int board_pid, bool eeprom);

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
fstream            filestr;
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

cDownloader();
};

#endif
