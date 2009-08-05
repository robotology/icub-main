#include <ace/OS.h>
#include "downloader.h"

#include <yarp/os/Time.h>

//*****************************************************************/
void drv_sleep (unsigned long int time)
{
  /*
 DWORD a=0;
 DWORD b=0;
 DWORD diff=0;
 a = GetTickCount();
 do
 {
  b = GetTickCount();
 }
 while (b-a<time);
  */
  yarp::os::Time::delay(time/1000.0);
}

//*****************************************************************/
//utility function for conversion of hex string to int
int axtoi(char *hexStg) 
{
  int n = 0;         // position in string
  int m = 0;         // position in digit[] to shift
  int count;         // loop index
  int intValue = 0;  // integer value of hex string
  int digit[5];      // hold values to convert
  while (n < 4) {
     if (hexStg[n]=='\0')
        break;
     if (hexStg[n] > 0x29 && hexStg[n] < 0x40 ) //if 0 to 9
        digit[n] = hexStg[n] & 0x0f;            //convert to int
     else if (hexStg[n] >='a' && hexStg[n] <= 'f') //if a to f
        digit[n] = (hexStg[n] & 0x0f) + 9;      //convert to int
     else if (hexStg[n] >='A' && hexStg[n] <= 'F') //if A to F
        digit[n] = (hexStg[n] & 0x0f) + 9;      //convert to int
     else break;
    n++;
  }
  count = n;
  m = n - 1;
  n = 0;
  while(n < count) {
     // digit[n] is value of hex digit at position n
     // (m << 2) is the number of positions to shift
     // OR the bits into return value
     intValue = intValue | (digit[n] << (m << 2));
     m--;   // adjust the position to set
     n++;   // next digit to process
  }
  return (intValue);
}

//*****************************************************************/

int cDownloader::build_id(int source, int dest)
{
  return (ID_CMD << 8) + ( source << 4) + dest;
}

int cDownloader::get_src_from_id (int id)
{
  // 000 1111 0000
  return ((id >> 4) & 0x0F);
}

int cDownloader::get_dst_from_id (int id)
{
  // 000 0000 1111
  return (id & 0x0F);
}

//*****************************************************************/

cDownloader::cDownloader()
{
 board_list = NULL;
 board_list_size = 0;
 connected = false;
 m_candriver=NULL;
}


//*****************************************************************/

int cDownloader::stopdriver()
{
 if (m_candriver !=NULL)
	{
	 delete m_candriver;
     m_candriver=NULL;
	 connected = false;
	}
 return 0;
}

//*****************************************************************/

int cDownloader::initdriver(can_parameters_type* params)
{
 if (m_candriver !=NULL)
 {
	 delete m_candriver;
     m_candriver=NULL;
	 connected = false;
 }

 m_candriver = new cDriver;

 if (m_candriver->init(params)==-1)
	{
	 if (m_candriver) 
	 {
		 delete m_candriver;
		 m_candriver=NULL;
		 connected = false;
	 }
	 return -1;
	}

 connected = true;
 return 0;
}

//*****************************************************************/
int cDownloader::get_board_info	   (int target_id, char* board_info)
{
 int i;
 if (board_info == NULL) return -1;

 memset (board_info,0,32);

 // check if driver is running
 if (m_candriver == NULL)
 {
	 printf ("ERR: Driver not ready\n");
	 return -1;
 }
 
 // Send command
 CMSG tx_message;
 tx_message.id=build_id(ID_MASTER, target_id);
 tx_message.len=1;

 tx_message.data[0]= CAN_GET_ADDITIONAL_INFO;
 int ret = m_candriver->send_message(tx_message);

 // check if send_message was successful
 if (ret==0)
 {
	 printf ("ERR: Unable to send message\n");
	 return -1;
 }
 
 //pause
 drv_sleep(10);

 //riceve la risposta
 CMSG rx_message[MAX_READ_MSG];
 int read_messages = m_candriver->receive_message(rx_message);

 //Timeout: no answers
 if (read_messages==0)
 {
	 printf ("ERR: No answers\n");
	 return -1;
 }

 //One (or more) answers received
 //Counts the number of the boards
 for (i=0; i<read_messages; i++)
 {
	 if (rx_message[i].data[0]==CAN_GET_ADDITIONAL_INFO && rx_message[i].len==6) 
	 {
		 int part = rx_message[i].data[1];
		 for (int j = 0; j< 4; j++)
			 board_info[part*4+j]=rx_message[i].data[j+2];
	 }
 }

 board_info[31]=0;
 return 0;
}


//*****************************************************************/
int cDownloader::change_board_info(int target_id, char* board_info)
{
 // check if driver is running
 if (m_candriver == NULL)
 {
	 printf ("ERR: Driver not ready\n");
	 return -1;
 }
 
 // Send command
 CMSG tx_message;
 int counter =0;
 int ret =0;
 int j = 0;

 for (counter = 0 ; counter < 8; counter++)
 {
	//do {}
	//while (CAN1_getStateTX () == 0) ;
	{ 
		tx_message.data[0]= CAN_SET_ADDITIONAL_INFO;
		tx_message.data[1]= counter;
		tx_message.id=build_id(ID_MASTER, target_id);
		tx_message.len=6;
		for (j=0; j<4; j++)	
			tx_message.data[2+j] = board_info[j+counter*4]; 		
		ret |= m_candriver->send_message(tx_message);
	} 	
 }

 // check if send_message was successful
 if (ret==0)
 {
	 printf ("ERR: Unable to send message\n");
	 return -1;
 }

 //pause
 drv_sleep(10);

 // update the board list
 initschede();

 return 0;
}

//*****************************************************************/
int cDownloader::change_card_address(int target_id, int new_id)
{
 int i = 0;

 // check if driver is running
 if (m_candriver == NULL)
 {
	 printf ("ERR: Driver not ready\n");
	 return -1;
 }
 
 // Send command
 CMSG tx_message;
 tx_message.id=(ID_MASTER << 4) + target_id;
 tx_message.len=2;

 tx_message.data[0]= CAN_SET_BOARD_ID;
 tx_message.data[1]= new_id;
 int ret = m_candriver->send_message(tx_message);

 // check if send_message was successful
 if (ret==0)
 {
	 printf ("ERR: Unable to send message\n");
	 return -1;
 }
 
 // update the board list
 initschede();

 return 0;
}
//*****************************************************************/

int cDownloader::initschede()
{

 int i;

 // check if driver is running
 if (m_candriver == NULL)
 {
	 printf ("ERR: Driver not ready\n");
	 return -1;
 }

  // Send command
 CMSG tx_message;
 tx_message.id=build_id(ID_MASTER,ID_BROADCAST);
 tx_message.len=1;
 tx_message.data[0]= CMD_BROADCAST;
 int ret = m_candriver->send_message(tx_message);

 // check if send_message was successful
 if (ret==0)
 {
	 printf ("ERR: Unable to send message\n");
	 return -1;
 }

 //pause
 drv_sleep(500);

 //riceve la risposta
 CMSG rx_message[MAX_READ_MSG];
 int read_messages = m_candriver->receive_message(rx_message);

 //Timeout: no answers
 if (read_messages==0)
 {
	 printf ("ERR: No answers\n");
	 return -1;
 }

 //One (or more) answers received
 //Counts the number of the boards
 board_list_size = 0;
 for (i=0; i<read_messages; i++)
 {
	 if (rx_message[i].data[0]==CMD_BROADCAST && 
		(rx_message[i].len==5 || rx_message[i].len==4)) board_list_size++;
 }

 if (board_list_size==0)
 { 
	 printf ("No Boards found\n");
	 return -1;
 }

 //Create the list of the boards
 if (board_list !=NULL) delete board_list;
 board_list= new sBoard[board_list_size];

 int j = 0;
 for (i=0; i<read_messages; i++)
 {
	 if (rx_message[i].data[0]==CMD_BROADCAST && rx_message[i].len==5)
	 {
		 board_list[j].pid     = (rx_message[i].id >> 4) & 0x0F;
		 board_list[j].type    = rx_message[i].data[1];
		 board_list[j].version = rx_message[i].data[2];
		 board_list[j].release = rx_message[i].data[3];
		 board_list[j].build   = rx_message[i].data[4];
		 board_list[j].status  = BOARD_RUNNING;
		 board_list[j].selected  = false;
		 memset (board_list[j].add_info,  0, 32);
		 j++;
	 }
	 if (rx_message[i].data[0]==CMD_BROADCAST && rx_message[i].len==4) //old version
	 {
		 board_list[j].pid     = (rx_message[i].id >> 4) & 0x0F;
		 board_list[j].type    = rx_message[i].data[1];
		 board_list[j].version = rx_message[i].data[2];
		 board_list[j].release = rx_message[i].data[3];
		 board_list[j].build   = 0;
		 board_list[j].status  = BOARD_RUNNING;
		 board_list[j].selected  = false;
		 memset (board_list[j].add_info,  0, 32);
		 j++;
	 }
 }

 for (i=0; i<board_list_size; i++)
 {
	 char board_info [32];
	 get_board_info	   (board_list[i].pid, board_info);
	 strcpy (board_list[i].add_info,  board_info);
	 //pause
	 drv_sleep(10);	
 }
 
 printf ("%d Boards found\n", board_list_size);

 return 0;
}

//*****************************************************************/

int cDownloader::startscheda(int board_pid)
{
 // check if driver is running
 if (m_candriver == NULL)
 {
	 printf ("ERR: Driver not ready\n");
	 return -1;
 }

 // Send command
 CMSG tx_message;
 tx_message.id=build_id(ID_MASTER,board_pid);
 tx_message.len=1;
 tx_message.data[0]= CMD_BOARD;
 int ret = m_candriver->send_message(tx_message);

 // check if send_message was successful
 if (ret==0)
 {
	 printf ("ERR: Unable to send message\n");
	 return -1;
 }

 //pause
 drv_sleep(5);

 // riceve la risposta
 CMSG rx_message[MAX_READ_MSG];
 int read_messages = m_candriver->receive_message(rx_message);

 //One (or more) answers received
 for (int i=0; i<read_messages; i++)
 {
	 if (rx_message[i].data[0]==CMD_BOARD &&
		(((rx_message[i].id >> 4) & 0x0F) == board_pid) &&
		(((rx_message[i].id >> 8) & 0x07) == ID_CMD))
	 {
		 //received ACK from board
		 printf ("ACK received from board: %d\n", board_pid);
		 return 0;
	 }
 }
 
 //ERROR
 printf ("ERR: No ACK received from board %d\n", board_pid);	
 return -1;

}

//*****************************************************************/

int cDownloader::stopscheda(int board_pid)
{
 // check if driver is running
 if (m_candriver == NULL)
 {
	 printf ("ERR: Driver not ready\n");
	 return -1;
 }

 // Send command
 CMSG tx_message;
 tx_message.id=build_id(ID_MASTER,board_pid);
 tx_message.len=1;
 tx_message.data[0]= CMD_END;
 int ret = m_candriver->send_message(tx_message);

 // check if send_message was successful
 if (ret==0)
 {
	 printf ("ERR: Unable to send message\n");
	 return -1;
 }

 //pause
 drv_sleep(5);

 // riceve la risposta
 CMSG rx_message[MAX_READ_MSG];
 int read_messages = m_candriver->receive_message(rx_message);

 //One (or more) answers received
 for (int i=0; i<read_messages; i++)
 {
	 if (rx_message[i].data[0]==CMD_END &&
		(((rx_message[i].id >> 4) & 0x0F) == board_pid) &&
		(((rx_message[i].id >> 8) & 0x07) == ID_CMD))
	 {
		 //received ACK from board
		 printf ("ACK received from board: %d\n", board_pid);
		 return 0;
	 }
 }
 
 //ERROR
 printf ("ERR: No ACK received from board %d\n", board_pid);	
 return -1;
}

//*****************************************************************/

int getvalue(char* line, int len)
{
 char hexconv_buffer[5];
 memset (hexconv_buffer, '\0', sizeof(hexconv_buffer) ); 
 strncpy(hexconv_buffer,line,len);
 return axtoi (hexconv_buffer);
}

//*****************************************************************/
int cDownloader::verify_ack(int command, CMSG* rx_message,int read_messages)
{
 int i,k;

 for (i=0; i<board_list_size; i++)
 {
	 if (board_list[i].selected==true)
		 if (board_list[i].status == BOARD_WAITING ||
			 board_list[i].status == BOARD_DOWNLOADING)
			{
			 board_list[i].status = BOARD_WAITING_ACK;
			 
			 for (k=0; k<read_messages; k++)
				{
					if ((rx_message[k].data[0]==command) &&
   						(rx_message[k].len == 2) &&
						(rx_message[k].data[1]==1))
						{
						 if (board_list[i].pid == get_src_from_id(rx_message[k].id))
							 board_list[i].status=BOARD_DOWNLOADING;
						}
				}
			}
 }

 for (i=0; i<board_list_size; i++)
 {
	 if (board_list[i].selected==true && board_list[i].status == BOARD_WAITING_ACK)
	 {
//@@@@		 board_list[i].status=BOARD_ERR;
		 return -1;
	 }
 }
 return 0;
}

//*****************************************************************/
// Return values:
// 0  one line downloaded, continuing the download...
// 1  Current downloading, everything OK
// -1 Fatal error 

int cDownloader::download_motorola_line(char* line, int len, int board_pid)
{
  char  sprsRecordType=0;
  unsigned long int  sprsChecksum=0;
  int  sprsMemoryType=1;
  long unsigned int  sprsAddress;
  int  sprsLength;
  int  i,j,k;
  CMSG tx_message;
  CMSG rx_message[MAX_READ_MSG];
  int ret =0;
  int read_messages=0;

  for (i=2; i<len; i=i+2)
  {
	  int value= getvalue(line+i,2);
	  sprsChecksum+= value;
     // printf ("chk: %d %d\n", value, sprsChecksum);
 		  
  }
  
  if ((sprsChecksum & 0xFF) == 0xFF)
  {
	 //  printf ("Checksum OK\n");
  }
  else
  {
	    printf ("ERR: Failed Checksum\n");
		return -1;
  }

  //state: WAIT
  if (!(line[0] == 'S'))
	  {
		  printf ("start tag character not found\n");	
		  return -1;
	  }
  i=1;
 
  //state: TYPE
  sprsRecordType=char(*(line+i));
  i++;

  //state: LENGTH
  sprsLength=getvalue(line+i,2)-4-1;
  i=i+2;

  switch (sprsRecordType)
		  {
		   case SPRS_TYPE_0:
	
			   return 0;

		   case SPRS_TYPE_3:

			   //state: ADDRESS
               sprsAddress=getvalue(line+i,4);
			   i+=4;

			   if (sprsAddress==0x0020)
				   sprsMemoryType=1;
			   else
				   sprsMemoryType=0;
			   
			   sprsAddress=getvalue(line+i,4);
			   i+=4;

			    //state: SEND
               tx_message.id=build_id(ID_MASTER,board_pid);
               tx_message.len=5;
               tx_message.data[0]= CMD_ADDRESS;
	       tx_message.data[1]= sprsLength;
	       tx_message.data[2]= (unsigned char) ((sprsAddress) & 0x00FF);
	       tx_message.data[3]= (unsigned char) ((sprsAddress>>8) & 0x00FF);
	       tx_message.data[4]= sprsMemoryType;

	       //send here
	       ret = m_candriver->send_message(tx_message);

	       // check if send_message was successful
	       if (ret==0)

				{
				 printf ("ERR: Unable to send message\n");
				 return -1;
				}

				//pause
				drv_sleep(5);

                //prepare packet
				int tmp, rest;
				if ((sprsLength%6) == 0)
				{
					tmp=sprsLength / 6;
					rest=6;
				}
				else
				{
					tmp=sprsLength / 6 + 1;
					rest=sprsLength % 6;
				}

				for (j=1; j<= tmp; j++)
				{
					tx_message.data[0]=CMD_DATA;
					if (j<tmp) tx_message.len=7;
					else tx_message.len=rest+1;
					
					for (k=1; k<=6; k++)
					{
						tx_message.data[k] = getvalue(line+i+((k-1)*2+((j-1)*12)),2);
					}

					//send here
					ret = m_candriver->send_message(tx_message);

					// check if send_message was successful
					if (ret==0)

					{
					 printf ("ERR: Unable to send message\n");
					 return -1;
					}

					//pause
					drv_sleep(5);
				}

				//pause
				drv_sleep(10);

			    //receive one ack for the whole line
				read_messages = m_candriver->receive_message(rx_message);
				verify_ack(CMD_DATA, rx_message, read_messages);
				return 0;
			    break;



		   case SPRS_TYPE_7:

			    //state: SEND
                tx_message.id=build_id(ID_MASTER,board_pid);
                tx_message.len=5;
                tx_message.data[0]= CMD_START;
			    tx_message.data[4]= getvalue(line+i,2); i+=2;
				tx_message.data[3]= getvalue(line+i,2); i+=2;
				tx_message.data[2]= getvalue(line+i,2); i+=2;
				tx_message.data[1]= getvalue(line+i,2); 

			   //send here
				ret = m_candriver->send_message(tx_message);

			    // check if send_message was successful
				if (ret==0)

				{
				 printf ("ERR: Unable to send message\n");
				 return -1;
				}

				//pause
				drv_sleep(10);

				// riceve la risposta
				read_messages = m_candriver->receive_message(rx_message);
				verify_ack(CMD_START, rx_message, read_messages);
				return 0;

			    break;


	 		default:
			    printf ("ERR: wrong format tag character %c (hex:%X)\n", sprsRecordType, sprsRecordType);
			    return -1;

			    break;
		  }

  printf ("ERR: Can't reach here!\n");
  return -1;
}

//*****************************************************************/

int cDownloader::open_file(ACE_CString file)
{
  progress=0;
  filestr.open (file.c_str(), fstream::in);
  if (!filestr.is_open())
  {
	printf ("ERR: Error opening file!\n");
	return -1;
  }

  file_length=0;
  char buffer[256];
  while (!filestr.eof())
  {
	filestr.getline (buffer,256);
    file_length++;
  }
  //printf ("length: %d\n",file_length);

  filestr.close();
  filestr.clear();
  filestr.open (file.c_str(), fstream::in);
  if (!filestr.is_open())
  {
	printf ("ERR: Error opening file!\n");
	return -1;
  }

  return 0;
}

//*****************************************************************/
// Return values:
// 0  Download terminated, everything OK
// 1  Current downloading, everything OK
// -1 Fatal error in sending one command 
int cDownloader::download_file(int board_pid)
{

  if (!filestr.is_open())
  {
	printf ("ERR: File not open!\n");
	return -1;
  }

  char buffer[256];
  int ret = 0;

  if (!filestr.eof())
  {
	  filestr.getline (buffer,256);

	  //avoid to download empty lines
	  if (strlen(buffer)!=0)
		{
		  ret = download_motorola_line(buffer, strlen(buffer), board_pid);		  
		  if (ret != 0)
		  {
			 // fatal error during download, abort 
			 filestr.close();
			 return -1;
		  }
		}

	  progress++;
	  //everything OK
	  return 1;
  }
  else
  {
   filestr.close();
   filestr.clear();
   //download terminated OK
   return 0;
  }
}
