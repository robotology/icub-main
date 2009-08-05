
#include <gtk/gtk.h>
#include <gtk/gtkmain.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>

//#include <ace/containers.h> 
//#include <ace/streams.h>

#include <stdlib.h>
#include <fstream>
#include <iostream>

using namespace std;

using namespace std;
using namespace yarp::os;

#define BOARD_RUNNING       0
#define BOARD_WAITING       1
#define BOARD_WAITING_ACK   2
#define BOARD_DOWNLOADING   3
#define BOARD_OK            4
#define BOARD_ERR           5

#define BOARD_TYPE_DSP 0x00
#define BOARD_TYPE_PIC 0x01
#define BOARD_TYPE_2DC 0x02
#define BOARD_TYPE_4DC 0x03
#define BOARD_TYPE_BLL 0x04

static GtkWidget *window     = NULL;
Port port;   

GtkWidget *vbox_main		 = NULL;
GtkWidget *main_vbox         = NULL;
GtkWidget *top_hbox			 = NULL;
GtkWidget *bottom_hbox       = NULL;
GtkWidget *start_end_button  = NULL;
GtkWidget *combo_net         = NULL;
GtkWidget *progress_bar      = NULL;
GtkWidget *inv1              = NULL; 

GtkWidget *vbox_board_list   = NULL;
GtkWidget *vbox_board_list2  = NULL;
GtkWidget *panel_hbox        = NULL;
GtkWidget *label             = NULL;
GtkWidget *sw                = NULL;

GtkWidget *message_icon		 = NULL; 
GtkWidget *message_hbox		 = NULL; 
GtkWidget *message_right_vbox = NULL; 
GtkWidget *message_label1	 = NULL; 
GtkWidget *message_label2	 = NULL; 

GtkWidget *treeview          = NULL;
GtkWidget *button1           = NULL;
GtkWidget *button2           = NULL;
GtkWidget *download_button   = NULL;

GtkWidget *picker            = NULL;
GtkWidget *message           = NULL;

//can_parameters_type params;
bool downloader_connected = false;

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
  COLUMN_BUILD,
  COLUMN_STATUS,
  COLUMN_ADD_INFO,
  NUM_COLUMNS
};

struct sBoard
{
public:
 int  pid;
 int  type;
 int  version;
 int  release;
 int  build;
 int  status;
 bool selected;
 char add_info [50];
};

sBoard board_list[16];
int    board_list_size;


//*********************************************************************************
void drv_sleep (unsigned long int time)
{
  yarp::os::Time::delay(time/1000.0);
}

//*********************************************************************************
void GetBoardList(void)
{
  //gets the board list from the server
  Bottle wbot, rbot;
  wbot.addString("GET_BOARD_LIST");
  port.write(wbot,rbot);

  board_list_size = rbot.get(0).asInt();
  for (int i=0; i<board_list_size; i++)
	 {
		 board_list[i].pid = rbot.get(1+i*7).asInt();
		 board_list[i].type = rbot.get(2+i*7).asInt();
		 board_list[i].version = rbot.get(3+i*7).asInt();
		 board_list[i].release = rbot.get(4+i*7).asInt();
		 board_list[i].build = rbot.get(5+i*7).asInt();
		 board_list[i].status = rbot.get(6+i*7).asInt();
		 board_list[i].selected = rbot.get(7+i*7).asInt();
		 strcpy(board_list[i].add_info,rbot.get(8+i*7).asString().c_str());
	 }
}

//*********************************************************************************
static GtkTreeModel * refresh_board_list_model (void)
{
 
  gint i = 0;
  GtkListStore *store;
  GtkTreeIter iter;

  // create list store 
  store = gtk_list_store_new ( NUM_COLUMNS,
			      G_TYPE_BOOLEAN,
			      G_TYPE_UINT,
			      G_TYPE_STRING,
			      G_TYPE_STRING,
				  G_TYPE_STRING,
				  G_TYPE_STRING,
				  G_TYPE_STRING,
				  G_TYPE_STRING);

  char board_type			[50]; memset (board_type,0,50);
  char board_status			[50]; memset (board_status,0,50);
  char board_add_info		[50]; memset (board_add_info,0,50);
  char board_version		[10]; memset (board_version,0,10);
  char board_release		[10]; memset (board_release,0,10);
  char board_build		    [10]; memset (board_build,0,10);

  //gets the board list from the server
  GetBoardList();

  // add data to the list store	 
  for (i = 0; i < board_list_size; i++)
    {
	  switch (board_list[i].type)
	  {
	  case BOARD_TYPE_DSP:
		  strcpy(board_type, "RM OLD (DSP)");
		  break;
	  case BOARD_TYPE_PIC:
		  strcpy(board_type, "MPH (PIC)");
		  break;
	  case BOARD_TYPE_2DC:
		  strcpy(board_type, "RM 2DC (DSP)");
		  break;
	  case BOARD_TYPE_4DC:
		  strcpy(board_type, "RM 4DC (DSP)");
		  break;
	  case BOARD_TYPE_BLL:
		  strcpy(board_type, "RM BLL (DSP)");
		  break;
	  default:
		  strcpy(board_type, "UNKNOWN");
		  break;
	  }

	  switch (board_list[i].status)
	  {
	  case BOARD_RUNNING:
		  strcpy(board_status, "RUNNING");
		  break;
	  case BOARD_WAITING:
		  strcpy(board_status, "WAITING");
		  break;
	  case BOARD_WAITING_ACK:
		  strcpy(board_status, "WAITING_ACK");
		  break;
	  case BOARD_DOWNLOADING:
		  strcpy(board_status, "DOWNLOADING");
		  break;
	  case BOARD_OK :
		  strcpy(board_status, "OK");
		  break;
	  case BOARD_ERR:
		  strcpy(board_status, "ERR");
		  break;
	  default:
		  strcpy(board_status, "UNKNOWN");
		  break;
	  }

	  strncpy  (board_add_info, board_list[i].add_info,32);
	  sprintf (board_version,"%d",board_list[i].version);
	  sprintf (board_release,"%X",board_list[i].release,board_list[i].release);
	  sprintf (board_build,"%d",board_list[i].build,board_list[i].build);

      gtk_list_store_append (store, &iter);
      gtk_list_store_set (store, &iter,
		  COLUMN_SELECTED, board_list[i].selected,
			  COLUMN_ID, board_list[i].pid,
			  COLUMN_TYPE, board_type,
			  COLUMN_VERSION, board_version,
			  COLUMN_RELEASE, board_release,
			  COLUMN_BUILD, board_build,
			  COLUMN_STATUS, board_status,
			  COLUMN_ADD_INFO, board_add_info,
			  -1);
			  
    }

  return GTK_TREE_MODEL (store);
}
//*********************************************************************************
static GtkTreeModel * create_net_model (void)
{
  gint i = 0;
  GtkListStore *store;
  GtkTreeIter iter;

  // create list store
  store = gtk_list_store_new (1, G_TYPE_STRING);

  // add data to the list store
      gtk_list_store_append (store, &iter);
      gtk_list_store_set (store, &iter, 0, "Net 0",-1);
      gtk_list_store_append (store, &iter);
      gtk_list_store_set (store, &iter, 0, "Net 1",-1);  
      gtk_list_store_append (store, &iter);
      gtk_list_store_set (store, &iter, 0, "Net 2",-1);  
	  gtk_list_store_append (store, &iter);
      gtk_list_store_set (store, &iter, 0, "Net 3",-1);  

  return GTK_TREE_MODEL (store);
}

//*********************************************************************************
static void combo_net_changed (GtkComboBox *box,	gpointer   user_data)
{
 int net = gtk_combo_box_get_active (box);
 printf ("net selected %d\n", net);

 Bottle wbot, rbot;
 wbot.addString("NET_SELECT");
 wbot.addInt(net);
 port.write(wbot,rbot);
	 if (rbot.get(0)== -1)
	 {
		 printf ("** Error: impossible to select net %d\n **", net);
	 }
}

//*********************************************************************************
// This callback exits from the gtk_main() main loop when the main window is closed
static void finish_download (GtkWidget *wid,	gpointer   user_data)
{	
	// Refresh the list of the selected boards
	Bottle wbot, rbot;
	wbot.addString("INIT_SCHEDE");
	port.write(wbot,rbot);

	if (rbot.get(0)== -1)
	{
		printf ("** Error: INIT_SCHEDE command failed\n **");
	}

    gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
    gtk_widget_draw(treeview, NULL);

	gtk_widget_destroy (wid);
}

//*********************************************************************************
static void select_all (GtkButton *button,	gpointer   user_data)
{
	 Bottle wbot, rbot;
	 wbot.addString("SELECT_ALL");
	 port.write(wbot, rbot);
	 if (rbot.get(0)== -1)
	 {
		printf ("** Error: SELECT_ALL command failed\n **");
	 }

	 gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
	 gtk_widget_draw(treeview, NULL);
	 printf ("all boards selected\n");
}
static void deselect_all (GtkButton *button,	gpointer   user_data)
{
	 Bottle wbot, rbot;
	 wbot.addString("DESELECT_ALL");
	 port.write(wbot, rbot);
	 if (rbot.get(0)== -1)
	 {
		printf ("** Error: DESELECT_ALL command failed\n **");
	 }

	 gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
	 gtk_widget_draw(treeview, NULL);
	 printf ("all boards not selected\n");
}

//*********************************************************************************
void connected_status()
{
 gtk_button_set_label     (GTK_BUTTON(start_end_button), "Disconnect");
 gtk_widget_set_sensitive (button1, true);    
 gtk_widget_set_sensitive (button2, true);  
 gtk_widget_set_sensitive (download_button, true);
 gtk_widget_set_sensitive (treeview, true);
 gtk_widget_set_sensitive (combo_net, false);
}

void not_connected_status()
{
 gtk_button_set_label     (GTK_BUTTON(start_end_button), "Connect");
 gtk_widget_set_sensitive (button1, false);    
 gtk_widget_set_sensitive (button2, false);    
 gtk_widget_set_sensitive (download_button, false);
 gtk_widget_set_sensitive (treeview, false);
 gtk_widget_set_sensitive (combo_net, true);
}
//*********************************************************************************

//Shows a message dialog for displaying infos/errors
GtkWidget * dialog_message_generator(GtkMessageType gtk_message_type, char* const text1, char* const text2, bool connect=true)
{
	GtkWidget *message;

	//message=gtk_message_dialog_new(NULL,GTK_DIALOG_MODAL,gtk_message_type,GTK_BUTTONS_CLOSE, text1);

	
	if		(gtk_message_type!=GTK_MESSAGE_QUESTION)
			{
				message = gtk_dialog_new_with_buttons ("Interactive Dialog",
					GTK_WINDOW (window),
					GTK_DIALOG_MODAL,
					GTK_STOCK_OK,
					GTK_RESPONSE_OK,
					NULL);
			}
	else
			{
				message = gtk_dialog_new_with_buttons ("Interactive Dialog",
					GTK_WINDOW (window),
					GTK_DIALOG_MODAL,
					GTK_STOCK_YES,
					GTK_RESPONSE_YES,
					GTK_STOCK_NO,
					GTK_RESPONSE_NO,
					NULL);
			}
	
/*
	message = gtk_dialog_new ("",
					GTK_WINDOW (window),
					GTK_DIALOG_MODAL,
					NULL);
*/

	gtk_window_set_resizable(GTK_WINDOW(message),false);
	
	message_hbox = gtk_hbox_new (FALSE, 8);
	gtk_container_set_border_width (GTK_CONTAINER (message_hbox), 8);
	gtk_box_pack_start (GTK_BOX (GTK_DIALOG (message)->vbox), message_hbox, FALSE, FALSE, 0);

	if		(gtk_message_type==GTK_MESSAGE_QUESTION)
	{
		gtk_window_set_title    (GTK_WINDOW(message),"Question");
		message_icon = gtk_image_new_from_stock (GTK_STOCK_DIALOG_QUESTION, GTK_ICON_SIZE_DIALOG);
	}
	else if (gtk_message_type==GTK_MESSAGE_ERROR)
	{
		gtk_window_set_title    (GTK_WINDOW(message),"Error");
		message_icon = gtk_image_new_from_stock (GTK_STOCK_DIALOG_ERROR, GTK_ICON_SIZE_DIALOG);
	}
	else if (gtk_message_type==GTK_MESSAGE_INFO)
	{
		gtk_window_set_title    (GTK_WINDOW(message),"Information");
		message_icon = gtk_image_new_from_stock (GTK_STOCK_DIALOG_INFO, GTK_ICON_SIZE_DIALOG);
	}
	else
	{
		gtk_window_set_title    (GTK_WINDOW(message),"Information");
		message_icon = gtk_image_new_from_stock (GTK_STOCK_DIALOG_INFO, GTK_ICON_SIZE_DIALOG);
	}

	gtk_box_pack_start (GTK_BOX (message_hbox), message_icon, FALSE, FALSE, 0);

	message_right_vbox = gtk_vbox_new (FALSE, 8);
	gtk_container_set_border_width (GTK_CONTAINER (message_right_vbox), 8);
	gtk_box_pack_start (GTK_BOX (message_hbox), message_right_vbox, FALSE, FALSE, 0);

	message_label1 = gtk_label_new (text1);
	
	gtk_label_set_justify   (GTK_LABEL(message_label1),  GTK_JUSTIFY_LEFT);
	gtk_box_pack_start (GTK_BOX (message_right_vbox), message_label1, FALSE, FALSE, 0);

	if (text2)
	{
		message_label2 = gtk_label_new (text2);
		
		gtk_label_set_justify   (GTK_LABEL(message_label2),  GTK_JUSTIFY_LEFT);
		gtk_box_pack_start (GTK_BOX (message_right_vbox), message_label2, FALSE, FALSE, 0);
	}

	gtk_widget_show_all (message_hbox);
	
	//GTK_BUTTONS_YES_NO
	return message;

}
//*********************************************************************************
bool dialog_message (GtkMessageType gtk_message_type, char* const text1, char* const text2, bool connect=true)
{
	message=dialog_message_generator(gtk_message_type, text1, text2, connect);

	gtk_widget_show(message);
	if (connect)
		g_signal_connect_swapped (message, "response",G_CALLBACK (gtk_widget_destroy), message);
	
	return 0;
}

//*********************************************************************************
bool dialog_question_additional_field ()
{
 bool ret=false;

 message=dialog_message_generator(GTK_MESSAGE_QUESTION, "Do you really want to change the additional info field of this board?", NULL);
 gint response = gtk_dialog_run (GTK_DIALOG (message));
 if (response == GTK_RESPONSE_YES)
 {
	 ret = true;
 }
 else
 {
	 ret = false;
 }
 gtk_widget_destroy (message);
 return ret;
}

//*********************************************************************************
bool dialog_question_can_address (gint old_id, gint new_id)
{
 bool ret=false;

 char text [200];
 sprintf (text , "Do you really want to change the can address of this board?\r\n\r\nCURRENT CAN ADDRESS %d WILL BE CHANGED IN %d",old_id,new_id) ;

 message=dialog_message_generator(GTK_MESSAGE_QUESTION, text, NULL);
 gtk_window_set_modal(GTK_WINDOW(message),true);

 gint response = gtk_dialog_run (GTK_DIALOG (message));
 if (response == GTK_RESPONSE_YES)
 {
	 ret=true;
 }
 else
 {
	 ret=false;
 }
 gtk_widget_destroy (message);
 return ret;
}
//*********************************************************************************
static void start_end_click (GtkButton *button,	gpointer   user_data)
{   
 int ret = 0;

 Bottle wbot, rbot;

 if (downloader_connected==false)
	{
		 wbot.addString("INIT_DRIVER");
		 port.write(wbot, rbot);

		 if (rbot.get(0)== -1)
		 {
			dialog_message(GTK_MESSAGE_ERROR,"Init driver failed","Hardware busy or not connected?!");
			return;
		 }
		 else
		 {
			 downloader_connected= true;
		 }

		 drv_sleep (2000);
		 wbot.clear();
		 rbot.clear();

		 //get the infos from the board
		 wbot.addString("INIT_SCHEDE");
		 port.write(wbot,rbot);

		 if (rbot.get(0)== -1)
		 {
			dialog_message(GTK_MESSAGE_INFO,"Communication error","No answers received (no boards found).");
			
			wbot.clear();
		    rbot.clear();
			wbot.addString("STOP_DRIVER");
			port.write(wbot, rbot);

			not_connected_status();
			return;
		 }

		gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
		gtk_widget_draw(treeview, NULL);

		//enable select/deselect all buttons
		dialog_message(GTK_MESSAGE_INFO,"Driver Connected","");
		connected_status();     
	 }
	 else
	 {

	 	wbot.clear();
		rbot.clear();
		wbot.addString("STOP_DRIVER");
		port.write(wbot,rbot);

		dialog_message(GTK_MESSAGE_INFO,"Driver Stopped","");
		//disable select/deselect all buttons
		downloader_connected= false;
		not_connected_status();
	 }
}

//*********************************************************************************
static void download_click (GtkButton *button,	gpointer   user_data)
{  
	
 double timer_start =0;
 double timer_end   =0;
 // check if can driver is running
 if (downloader_connected == false)
 {
 	dialog_message(GTK_MESSAGE_ERROR,"Driver not running","Use 'Connect' button to start the driver");
	return;
 }

 //Refresh the board list from the server
 GetBoardList();

 //check if at least one board was selected
 bool at_least_one_board_selected = false;
 int i        = 0;

 for (i=0; i<board_list_size; i++)
 {
	 if (board_list[i].status==BOARD_RUNNING &&
		 board_list[i].selected==true)
		 at_least_one_board_selected = true;
 }

 if (!at_least_one_board_selected)
	{
	 dialog_message(GTK_MESSAGE_ERROR,"No Boards selected!","Select one or more boards to download the firmware");
	 return;
	}

 // Open the selected file 
 if (filename==NULL)
 {
	 dialog_message(GTK_MESSAGE_ERROR,"No files selected!","Select the file You want to download, first");
	 return;
 }
 //transfer selected file over network
 Bottle file_bottle;
 Bottle file_data_only;
 Bottle file_ack;
 fstream filestr;
 filestr.open (filename, fstream::in);
 if (!filestr.is_open())
  {
    dialog_message(GTK_MESSAGE_ERROR,"Error opening the selected file!","");
	printf ("** ERR: Error opening file!\n **");
	return;
  }
 int file_length=0;
 char buffer[256];
 while (!filestr.eof())
 {
	filestr.getline (buffer,256);
	file_data_only.addString(buffer);
    file_length++;
 }
 file_bottle.addString("FILE_TRANSFER");
 file_bottle.addInt(file_length);
 file_bottle.append(file_data_only);
 port.write(file_bottle,file_ack);
 if (file_ack.get(0)== -1)
 {
	 dialog_message(GTK_MESSAGE_ERROR,"Error during file tranfer!","");
	 printf ("ERR: Something wrong during file transfer to server\n");
	 return;
 }
 else
 {
	 printf ("file successfully transferred\n");
 }

 
 // Start the download for the selected boards
 for (i=0; i<board_list_size; i++)
 {
	 if (board_list[i].status==BOARD_RUNNING &&
		 board_list[i].selected==true)
		 {	
			Bottle wbot, rbot;
			wbot.addString("START_SCHEDA");
			wbot.addInt(i);
			wbot.addInt(board_list[i].pid);
			port.write(wbot,rbot);

			if (rbot.get(0) == -1)
			{
				 dialog_message(GTK_MESSAGE_ERROR,"Unable to start the board","Unable to send message 'start' or no answer received");
				 return;
			}
		 }
 }

 // Refresh the list of the selected boards
 gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
 gtk_widget_draw(treeview, NULL);
 gtk_main_iteration_do (false);

 int   ret      = 0;
 int   finished = 0;
 double progress = 0;

 timer_start= yarp::os::Time::now();

 // Start the download for the selected boards
 do
	{
	 Bottle wbot, rbot;
	 wbot.addString("DOWNLOAD_FILE");
	 wbot.addInt(0x0F);
	 port.write(wbot,rbot);

	 ret = rbot.get(0).asInt();
	 progress = rbot.get(1).asDouble();

	 if (ret==1)
		 {
			//Continuing the download
			gtk_progress_bar_set_fraction ((GtkProgressBar*) progress_bar, progress);
 			gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
			gtk_widget_draw(window, NULL);
			gtk_main_iteration_do (false);
		 }
		 if (ret==-1)
		 {
			 //Fatal Error during download, terminate
			 finished = 1;
		 }
		 if (ret==0)
		 {
			 //Download terminated
			 finished = 1;
		 }
	 
	 // Update the progress bar 
	 gtk_progress_bar_set_fraction ((GtkProgressBar*) progress_bar, 0);
	 gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
	 gtk_widget_draw(treeview, NULL);
	 gtk_main_iteration_do (false);
	}
 while (finished!=1);
 
 timer_end= yarp::os::Time::now();

 // End the download for the selected boards
 int errors =0;
 //char tmp [100];

 //Refresh the board list from the server
 GetBoardList();

 for (i=0; i<board_list_size; i++)
 {
	 if (board_list[i].selected==true)
	 {
		if (board_list[i].status==BOARD_DOWNLOADING)
		{
			Bottle wbot, rbot;
			wbot.addString("STOP_SCHEDA");
			wbot.addInt(i);
			wbot.addInt(board_list[i].pid);
			port.write(wbot,rbot);

			ret = rbot.get(0).asInt();

			if (ret ==0)
			{
				printf("board %d stopped\n",board_list[i].pid);
			}
			else
			{
				printf ("Unable to stop board %d",board_list[i].pid);
				errors++;
			}
		}
		else if (board_list[i].status==BOARD_ERR)
		{
			 printf ("Board %d: download failed",board_list[i].pid); 
			 errors++;
		}
	 }
 }

//Display result message
if (errors==0)
	{
	 char time_text [50];
	 double download_time = (timer_end-timer_start) ;
	 sprintf (time_text, "All Board OK\nDownload Time (s): %.2f", download_time);
	 dialog_message(GTK_MESSAGE_INFO,"Download Finished",time_text,false);
	 g_signal_connect_swapped (message, "response",G_CALLBACK (finish_download), message);

	 gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
	 gtk_widget_draw(treeview, NULL);

	}
else
	{
	 dialog_message(GTK_MESSAGE_ERROR,"Error during file transfer","",false);
	 g_signal_connect_swapped (message, "response",G_CALLBACK (finish_download), message);

	 gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
	 gtk_widget_draw(treeview, NULL);
	}
 
}

//*********************************************************************************
static void choose_file (GtkFileChooser *picker,	gpointer   user_data)
{
 gchar* path = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(picker));
 if (path == NULL) return;

 fstream filestr;

 filestr.open ("config.txt", fstream::out);
 if (filestr.is_open())
	{
		filestr.write(path, strlen(path));
		filestr.close();
		filestr.clear();
	}

 // new file has been selected
 printf ("New file has been selected\n");
 g_free (path);

 if (filename!=NULL) g_free (filename);
 filename = gtk_file_chooser_get_filename   (GTK_FILE_CHOOSER(picker));

 // Get an identification of the firmware fot the file that you have selected
 // this is depending on the file name
 printf ("filename: %s \n", filename);
 if		 (strncmp (filename, "4DC", 3) == 0) {firmware_board_type = BOARD_TYPE_4DC; printf ("opened a 4DC firmware file\n"); }
 else if (strncmp (filename, "2BL", 3) == 0) {firmware_board_type = BOARD_TYPE_BLL; printf ("opened a BLL firmware file\n"); }
 else if (strncmp (filename, "2DC", 3) == 0) {firmware_board_type = BOARD_TYPE_2DC; printf ("opened a 2DC firmware file\n"); }
 else									     {firmware_board_type = BOARD_TYPE_DSP; printf ("opened a generic firmware file\n"); }

 char substring [10];

 strncpy (substring, filename+4, 1);
 firmware_version = strtol(substring,  NULL, 10);
 printf  ("opened firmware version: %s %d \n", substring, firmware_version);
 

 strncpy (substring, filename+6, 2);
 firmware_revision = strtol(substring,  NULL, 10);
 printf  ("opened firmware revision: %s %d \n", substring, firmware_revision);
}

//*********************************************************************************
static void
edited_board_id (GtkCellRendererText *cell, gchar *path_str, gchar *new_text, gpointer data)
{
  gint  new_val = atoi (new_text);
  if   (new_val <=0 || new_val> 15) return;
  //---


  //---
  GtkTreeModel *model = gtk_tree_view_get_model (GTK_TREE_VIEW(treeview));
  GtkTreeIter  iter;
  GtkTreePath *path = gtk_tree_path_new_from_string (path_str);

  
  gint*  old_val = new gint;
  // get toggled iter 
  gtk_tree_model_get_iter (model, &iter, path);
  gtk_tree_model_get (model, &iter, COLUMN_ID, old_val,-1);


  if (*old_val == new_val) 
  {  
	  delete (old_val);
	  return;
  }
  
  if (dialog_question_can_address(*old_val,new_val))
  {
	   Bottle wbot, rbot;
	   wbot.addString("CHANGE_ID");
	   wbot.addInt(*old_val);
       wbot.addInt(new_val);
	   port.write(wbot, rbot);

	   if (rbot.get(0)== -1) 
	   {
		   	printf ("ERR: CHANGE_ID command failed\n");
	   }
	   delete (old_val);
  }

  gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
  gtk_widget_draw(treeview, NULL);

  // clean up 
  gtk_tree_path_free (path);
}

//*********************************************************************************
static void
edited_additional_info (GtkCellRendererText *cell, gchar *path_str, gchar *new_text, gpointer data)
{
  GtkTreeModel *model = gtk_tree_view_get_model (GTK_TREE_VIEW(treeview));
  GtkTreeIter  iter;
  GtkTreePath *path = gtk_tree_path_new_from_string (path_str);

  gchar* old_text;
  gint*  board_id = new gint;
  // get toggled iter 
  gtk_tree_model_get_iter (model, &iter, path);
  gtk_tree_model_get (model, &iter, COLUMN_ADD_INFO, &old_text,-1);
  gtk_tree_model_get (model, &iter, COLUMN_ID, board_id,-1);

  if (strcmp (old_text, new_text) == 0) 
  {
	  g_free (old_text);
	  delete (board_id);
	  return;
  }

  char safe_buffer [32];
  memset (safe_buffer,0,32);
  strncpy (safe_buffer,new_text,32);
  if (dialog_question_additional_field())
  {
	  Bottle wbot, rbot;
	  wbot.addString("CHANGE_INFO");
	  wbot.addInt(*board_id);
	  wbot.addString(safe_buffer);
	  port.write(wbot, rbot);

	  if (rbot.get(0)== -1)
		 {
			  printf ("ERR: CHANGE_INFO command failed\n");
		 }

	  g_free (old_text);
	  delete (board_id);
  }

  gtk_tree_view_set_model (GTK_TREE_VIEW (treeview), refresh_board_list_model());
  gtk_widget_draw(treeview, NULL);

  // clean up 
  gtk_tree_path_free (path);
  
}

//*********************************************************************************
static void
fixed_toggled (GtkCellRendererToggle *cell, gchar *path_str, gpointer data)
{
  GtkTreeModel *model = gtk_tree_view_get_model (GTK_TREE_VIEW(treeview));
  GtkTreeIter  iter;
  GtkTreePath *path = gtk_tree_path_new_from_string (path_str);
  gboolean fixed;

  // get toggled iter 
  gtk_tree_model_get_iter (model, &iter, path);
  gtk_tree_model_get (model, &iter, COLUMN_SELECTED, &fixed, -1);

  // do something with the value 
  fixed ^= 1;

  int* index;
  index = gtk_tree_path_get_indices (path);

  if (index != NULL)
  {
	  Bottle wbot, rbot;
	  wbot.addString("TOGGLE_FIXED");
	  wbot.addInt(index[0]);
	  port.write(wbot, rbot);

	  if (rbot.get(0)== -1)
	  {
		 printf ("ERR: Something wrong in the selection\n");
	  }
  }
  else
  {
	  printf ("ERR: Something wrong in getting the index\n");
  }

  // set new value 
  gtk_list_store_set (GTK_LIST_STORE (model), &iter, COLUMN_SELECTED, fixed, -1);

  // clean up 
  gtk_tree_path_free (path);
}
//*********************************************************************************
static void add_columns (GtkTreeView *treeview)
{
  GtkCellRenderer *renderer;
  GtkTreeViewColumn *column;
  GtkTreeModel *model = gtk_tree_view_get_model (treeview);

  // column 1 SELECTED
  renderer = gtk_cell_renderer_toggle_new ();
  g_signal_connect (renderer, "toggled", G_CALLBACK (fixed_toggled), NULL);

  column = gtk_tree_view_column_new_with_attributes ("Selected",
						     renderer,
						     "active", COLUMN_SELECTED,
						     NULL);

  // set this column to a fixed sizing
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 70);
  gtk_tree_view_append_column (treeview, column);

  // column 2 PID
  renderer = gtk_cell_renderer_text_new ();
  GTK_CELL_RENDERER_TEXT(renderer)->editable=true;
  GTK_CELL_RENDERER_TEXT(renderer)->editable_set=true;
  renderer->mode=GTK_CELL_RENDERER_MODE_EDITABLE;
  g_signal_connect (renderer, "edited", G_CALLBACK (edited_board_id), NULL);

  column = gtk_tree_view_column_new_with_attributes ("ID",
						     renderer,
						     "text",
						     COLUMN_ID,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 40);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_ID);
  gtk_tree_view_append_column (treeview, column);

  // column 3 BOARD TYPE
  renderer = gtk_cell_renderer_text_new ();
  column = gtk_tree_view_column_new_with_attributes ("Type",
						     renderer,
						     "text",
						     COLUMN_TYPE,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 80);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_TYPE);
  gtk_tree_view_append_column (treeview, column);

  // column 4 VERSION
  renderer = gtk_cell_renderer_text_new ();
  column = gtk_tree_view_column_new_with_attributes ("Version",
						     renderer,
						     "text",
						     COLUMN_VERSION,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 60);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_VERSION);
  gtk_tree_view_append_column (treeview, column);

  // column 5 REVISION
  renderer = gtk_cell_renderer_text_new ();
  column = gtk_tree_view_column_new_with_attributes ("Release",
						     renderer,
						     "text",
						     COLUMN_RELEASE,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 60);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_RELEASE);
  gtk_tree_view_append_column (treeview, column);

  // column 5b BUILD
  renderer = gtk_cell_renderer_text_new ();
  column = gtk_tree_view_column_new_with_attributes ("Build number",
						     renderer,
						     "text",
						     COLUMN_BUILD,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 70);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_RELEASE);
  gtk_tree_view_append_column (treeview, column);

  // column 6 STATUS
  renderer = gtk_cell_renderer_text_new ();
  column = gtk_tree_view_column_new_with_attributes ("Status",
						     renderer,
						     "text",
						     COLUMN_STATUS,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 100);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_STATUS);
  gtk_tree_view_append_column (treeview, column);

  // column 7 ADDITIONAL INFO
  renderer = gtk_cell_renderer_text_new ();
  GTK_CELL_RENDERER_TEXT(renderer)->editable=true;
  GTK_CELL_RENDERER_TEXT(renderer)->editable_set=true;
  renderer->mode=GTK_CELL_RENDERER_MODE_EDITABLE;
  g_signal_connect (renderer, "edited", G_CALLBACK (edited_additional_info), NULL);

  column = gtk_tree_view_column_new_with_attributes ("Additional Info",
						     renderer,
						     "text",
						     COLUMN_ADD_INFO,
						     NULL);
  gtk_tree_view_column_set_sizing (GTK_TREE_VIEW_COLUMN (column),GTK_TREE_VIEW_COLUMN_FIXED);
  gtk_tree_view_column_set_fixed_width (GTK_TREE_VIEW_COLUMN (column), 200);
  //gtk_tree_view_column_set_sort_column_id (column, COLUMN_ADD_INFO);
  gtk_tree_view_append_column (treeview, column);

}

//*********************************************************************************
// This callback exits from the gtk_main() main loop when the main window is closed
static void destroy_main (GtkWindow *window,	gpointer   user_data)
{
	gtk_widget_destroy (GTK_WIDGET(window));
	gtk_main_quit ();
}

//*********************************************************************************
// Entry point for the GTK application
int myMain( int   argc, char *argv[] )
{	/*  //for debug only
		downloader.board_list= new sBoard[1];
  		downloader.board_list[0].pid     = 10;
		 downloader.board_list[0].type    = BOARD_TYPE_BLL;
		 downloader.board_list[0].version = 11;
		 downloader.board_list[0].release = 1;
		 downloader.board_list[0].status  = BOARD_RUNNING;
         downloader.board_list[0].selected  = false;
		 strcpy(downloader.board_list[0].add_info , "can flasher default");
		  gtk_widget_set_sensitive (treeview, true);
		downloader.board_list_size=1;
*/

	// open yarp connection
	Network::init();

	Contact c = Network::queryName("/canloader/client");
	if (c.isValid())
		cout << c.toString();
	else
		cout << "boh" << endl;

    port.open("/canloader/client");

	bool ret=yarp::os::Network::connect ("/canloader/client", "/canloader/server", "tcp", false);
	if (ret==true)
	{
		cout << "successfully connected to canloader server" << endl;
	}
	else
	{
		cout << "** ERROR: unable to connect to canloader server ***" << endl;
		port.close();
		Network::fini();
		return 0;
	}


	GtkTreeModel *model;

    gtk_init (&argc, &argv);
    
	//create the main window, and sets the callback destroy_main() to quit
	//the application when the main window is closed
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "CAN Flasher V2.2 beta ");
    g_signal_connect (window, "destroy",G_CALLBACK (destroy_main), &window);

    gtk_container_set_border_width (GTK_CONTAINER (window), 8);

	//Creation of main_vbox the container for every other widget
    main_vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (window), main_vbox);

    //creation of the top_hbox
    top_hbox = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (top_hbox), 10);
    gtk_container_add (GTK_CONTAINER (main_vbox), top_hbox);

		//widget in the top frame

	    // button to start connection
	    start_end_button = gtk_button_new_with_mnemonic ("Connect");

		inv1 = gtk_fixed_new ();
	    gtk_container_add (GTK_CONTAINER (top_hbox), inv1);
		gtk_fixed_put   (GTK_FIXED(inv1), start_end_button, 0, 0);
		gtk_widget_set_size_request     (start_end_button, 80, 30);
		g_signal_connect (start_end_button, "clicked", G_CALLBACK (start_end_click), NULL);



        //net id selection
        model = create_net_model ();
        combo_net = gtk_combo_box_new_with_model (model);
        g_object_unref (model);

        GtkCellRenderer* renderer = gtk_cell_renderer_text_new ();
        gtk_cell_layout_pack_start (GTK_CELL_LAYOUT (combo_net), renderer, TRUE);
        gtk_cell_layout_set_attributes (GTK_CELL_LAYOUT (combo_net), renderer,"text", 0, NULL);
        gtk_widget_set_size_request     (combo_net, 100, 30);
 
		gtk_combo_box_set_active (GTK_COMBO_BOX (combo_net), 0);

        g_signal_connect (combo_net, "changed", G_CALLBACK (combo_net_changed), NULL);
	     
		gtk_fixed_put   (GTK_FIXED(inv1), combo_net ,100, 0);


    //Creation of the bottom frame
    bottom_hbox = gtk_hbox_new (FALSE, 8);
	gtk_container_set_border_width (GTK_CONTAINER (bottom_hbox), 10);
    gtk_container_add (GTK_CONTAINER (main_vbox), bottom_hbox);

	//In the bottom frame there is:
	//1) the list of the cards
    sw = gtk_scrolled_window_new (NULL, NULL);
    gtk_scrolled_window_set_shadow_type (GTK_SCROLLED_WINDOW (sw), GTK_SHADOW_ETCHED_IN);
    gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (sw), GTK_POLICY_NEVER, GTK_POLICY_AUTOMATIC);
    gtk_box_pack_start (GTK_BOX (bottom_hbox), sw, TRUE, TRUE, 0);

	// A Label
    // label = gtk_label_new ("Connected Board List");
    // gtk_box_pack_start (GTK_BOX (main_vbox), label, FALSE, FALSE, 0);

		{
         //List of the cards
		 //create tree model
		 model = refresh_board_list_model ();

		 //create tree view
		 treeview = gtk_tree_view_new_with_model (model);
		 gtk_tree_view_set_rules_hint (GTK_TREE_VIEW (treeview), TRUE);
		 gtk_tree_view_set_search_column (GTK_TREE_VIEW (treeview),COLUMN_ID);

		 g_object_unref (model);

		 gtk_container_add (GTK_CONTAINER (sw), treeview);

		 // add columns to the tree view
		 add_columns (GTK_TREE_VIEW (treeview));
		}

	//2) A panel in the right
	panel_hbox = gtk_vbox_new (FALSE, 8);
	gtk_container_set_border_width (GTK_CONTAINER (panel_hbox), 10);
    gtk_container_add (GTK_CONTAINER (bottom_hbox), panel_hbox);

		{
		 //Button 1 in the panel
		 button1 = gtk_button_new_with_mnemonic ("Select All");
		 gtk_container_add (GTK_CONTAINER (panel_hbox ), button1);
		 g_signal_connect (button1, "clicked", G_CALLBACK (select_all),NULL);
		 gtk_widget_set_size_request     (button1, 130, 30);
	
		 //Button 2 in the panel
		 button2 = gtk_button_new_with_mnemonic ("Deselect All");
		 gtk_container_add (GTK_CONTAINER (panel_hbox ), button2);
		 g_signal_connect (button2, "clicked", G_CALLBACK (deselect_all),NULL);
		 gtk_widget_set_size_request     (button2, 130, 30);

		 //Download button in the panel
		 download_button = gtk_button_new_with_mnemonic ("Start Download");
		 gtk_container_add (GTK_CONTAINER (panel_hbox ), download_button);
		 g_signal_connect (download_button, "clicked", G_CALLBACK (download_click),NULL);
		 gtk_widget_set_size_request     (download_button, 130, 30);

		 //file chooser
		 picker = gtk_file_chooser_button_new ("Pick a File", GTK_FILE_CHOOSER_ACTION_OPEN);

		 
		 //find default path
		 fstream filestr;
		 char path[256];
		 filestr.open ("config.txt", fstream::in);
		 if (filestr.is_open())
			{
				filestr.getline (path,255);
				filestr.close();
				filestr.clear();
				gtk_file_chooser_set_filename (GTK_FILE_CHOOSER(picker), path);
			}

		 		 gtk_box_pack_start (GTK_BOX (panel_hbox), picker, TRUE, TRUE, 0);
		 g_signal_connect (picker, "selection-changed", G_CALLBACK (choose_file), NULL);
		 gtk_widget_set_size_request     (picker, 130, 30);
		 
		 //progress bar
		 progress_bar=gtk_progress_bar_new ();
		 gtk_container_add (GTK_CONTAINER (panel_hbox ), progress_bar);
		 gtk_widget_set_size_request     (progress_bar, 130, 25);

		}

    // finish & show
	not_connected_status();
	//	connected_status();
    gtk_window_set_default_size (GTK_WINDOW (window), 480, 250);
	gtk_window_set_resizable (GTK_WINDOW (window), false);

	if (!GTK_WIDGET_VISIBLE (window))
    gtk_widget_show_all (window);
    else
	{
      gtk_widget_destroy (window);
      window = NULL;
    }

    gtk_main ();
	
	//close yarp connection
	port.close();
	Network::fini();
    return 0;
}


#ifdef WIN32
#include <windows.h>
// win32 non-console applications define WinMain as the
// entry point for the linker
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    return myMain (__argc, __argv);

	int i;
	for (i=0;i<100; i++)
	{
		printf("%d",i);
	}
}

#else

int main(int argc, char* argv[])
{
    return myMain(argc, argv);
}
#endif
