

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "downloader.h"

#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/String.h>
#include <iostream>

using namespace std;
using namespace yarp::os;

cDownloader downloader;

can_parameters_type params;

//This for opened firmware file
char* filename=NULL;
int firmware_board_type=0;
int firmware_version=0;
int firmware_revision=0;

enum
{
  COLUMN_SELECTED,
  COLUMN_ID,
  COLUMN_TYPE,
  COLUMN_VERSION,
  COLUMN_RELEASE,
  COLUMN_STATUS,
  COLUMN_ADD_INFO,
  NUM_COLUMNS
};

int main( int   argc, char *argv[] )
{
    Network::init();
    Port port;
	yarp::String message;
	int ret;
    Bottle input;
	Bottle output;

	//initial values
	params.p_net=0;

	cout << "CANLOADER server started..." << endl << endl;

    port.open("/canloader/server");
    while (true) {
		input.clear();
		output.clear();

        cout << "waiting for input" << endl;

		//receiving commands and preparing for reply
		port.read(input,true);
        if (input!=NULL) 
			{
			cout << "got " << input.toString().c_str() << endl;
		
			//commmand switch
			message = input.get(0).asString().c_str();
			if (message == "INIT_DRIVER")
			{
				//try to connect to the driver
				ret = downloader.initdriver(&params);
				output.addInt(ret);
				port.reply(output);
			}
			else if (message == "CHANGE_ID")
			{
				 int old_board_id=input.get(1).asInt();
				 int new_board_id=input.get(2).asInt();
				 ret = downloader.change_card_address(old_board_id,new_board_id);
				 output.addInt(ret);
				 port.reply(output);
			}
			else if (message == "CHANGE_INFO")
			{
				 int board_id=input.get(1).asInt();
				 yarp::String info =input.get(2).asString().c_str();
				 ret = downloader.change_board_info(board_id, info.c_str());
				 output.addInt(ret);
				 port.reply(output);
			}
			else if (message == "TOGGLE_FIXED")
			{
				int index=input.get(1).asInt();
				if (index >= 0 && index < downloader.board_list_size)
					{
						downloader.board_list[index].selected=bool(fixed);
						ret = 0;
					}
				else
					{
						ret = -1;
					}
				 output.addInt(ret);
				 port.reply(output);
			}
			else if (message == "SELECT_ALL")
			{
				 for (int i = 0; i < downloader.board_list_size; i++)
					{
					 downloader.board_list[i].selected=true;
					}
				 output.addInt(0);
				 port.reply(output);
			}
			else if (message == "DESELECT_ALL")
			{
				 for (int i = 0; i < downloader.board_list_size; i++)
					{
					 downloader.board_list[i].selected=false;
					}
				 output.addInt(0);
				 port.reply(output);
			}
			else if (message == "GET_BOARD_LIST")
			{
				 output.addInt(downloader.board_list_size);
				 for (int i=0; i<downloader.board_list_size; i++)
				 {
					 output.addInt(downloader.board_list[i].pid);
					 output.addInt(downloader.board_list[i].type);
					 output.addInt(downloader.board_list[i].version);
					 output.addInt(downloader.board_list[i].release);
					 output.addInt(downloader.board_list[i].build);
					 output.addInt(downloader.board_list[i].status);
					 output.addInt(downloader.board_list[i].selected);
					 output.addString(downloader.board_list[i].add_info);
				 }
				 port.reply(output);
			}
			else if (message == "STOP_DRIVER")
			{
				 ret = downloader.stopdriver();
				 output.addInt(ret);
				 port.reply(output);
			}
			else if (message == "INIT_SCHEDE")
			{
				 ret = downloader.initschede();
				 output.addInt(ret);
				 port.reply(output);
			}
			else if (message == "START_SCHEDA")
			{
				 int index=input.get(1).asInt();
				 int id=input.get(2).asInt();
				 ret = downloader.startscheda(id);

				 if (ret==0) downloader.board_list[index].status=BOARD_WAITING;

				 output.addInt(ret);
				 port.reply(output);
			}
			else if (message == "DOWNLOAD_FILE")
			{
				 int i=input.get(1).asInt();
				 ret = downloader.download_file(i);
				 output.addInt(ret);
				 output.addDouble(float(downloader.progress)/downloader.file_length);
				 port.reply(output);
			}
			else if (message == "STOP_SCHEDA")
			{
				 int index=input.get(1).asInt();
				 int id=input.get(2).asInt();

				 if (downloader.stopscheda(id)==0)
				 {	
					 downloader.board_list[index].status=BOARD_OK;
					 printf("board %d stopped\n",downloader.board_list[index].pid);
					 output.addInt(0);
				     port.reply(output); 
				 }
				 else
				 {
					 printf ("Unable to stop board %d",downloader.board_list[index].pid);
					 downloader.board_list[index].status=BOARD_ERR;
					 output.addInt(-1);
				     port.reply(output); 
				 }
			}
			else if (message == "NET_SELECT")
			{
				 params.p_net=input.get(1).asInt();
				 cout << "selected net:" << params.p_net << endl;
				 output.addInt(0);
				 port.reply(output);
			}
			else if (message == "FILE_TRANSFER")
			{
				 int i=0;
				 downloader.progress=0;
				 downloader.file_length=input.get(1).asInt();
				 for (i=0; i<downloader.file_length; i++)
				 {
					 yarp::String s;
					 s = input.get(i+2).asString().c_str();
					 downloader.file_data.push_back(s);
				 }
				 output.addInt(0);
				 port.reply(output);
			}				
			else
			{	
				cout << "** ERROR: unknown command message **" << endl;
				output.addInt(-1);
				port.reply(output);
			}
		
        }
    }
    Network::fini();
    return 0;

}


