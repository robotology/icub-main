#include "downloader.h"
#include "calibrate_window.h"

extern cDownloader downloader;
extern GtkWidget *window;
int	   selected = 0;
unsigned int calibration_value=32767;
int ch[6]={0,1,2,3,4,5};
unsigned int offset[6];
unsigned int adc[6]={0,0,0,0,0,0};
unsigned int maxadc[6]={0,0,0,0,0,0};
unsigned int minadc[6]={65535,65535,65535,65535,65535,65535};
unsigned int matrix[6][6];
unsigned int calib_matrix[6][6];
guint  timer_refresh;   
GtkWidget* curr_measure[6];
GtkWidget* max_measure[6];
GtkWidget* min_measure[6];
GtkWidget* diff_measure[6];
GtkWidget* edit_matrix[6][6];
GtkWidget* slider_gain[6];	
GtkWidget* slider_zero;	
GtkWidget* info_dlg;
GtkWidget* picker_calib;
GtkWidget *save_button;
gboolean timer_func (gpointer data);

bool matrix_changed;
bool something_changed;
		
#define START_TIMER timer_refresh = g_timeout_add (500, timer_func, NULL);
#define STOP_TIMER g_source_remove(timer_refresh);


//*********************************************************************************
void save_click (GtkButton *button,	gpointer   user_data)
{
	STOP_TIMER
	drv_sleep (1000);
	downloader.strain_save_to_eeprom(downloader.board_list[selected].pid);
	drv_sleep (1000);
	something_changed=false;
	START_TIMER
}

//*********************************************************************************
void auto_click (GtkButton *button,	gpointer   user_data)
{
	STOP_TIMER
	downloader.strain_calibrate_offset(downloader.board_list[selected].pid,calibration_value);
	START_TIMER
}
/*
#include <sstream>
#include <string>
#include <string.h>


// Problems using itoa in Linux (not ansi C). Using this code instead.

char *myitoa(int a, char *buff, int d)
{
  std::string tempbuf;
  std::stringstream lStream;
  // lStream.fill('0');
  // lStream.width(3);
  lStream << adc[0];
  tempbuf+=lStream.str();

  strncpy(buff, tempbuf.c_str(), d);
  return buff;
} 
*/

//*********************************************************************************
gboolean timer_func (gpointer data)
{
	downloader.strain_get_offset (downloader.board_list[selected].pid, 0, offset[0]);
	downloader.strain_get_offset (downloader.board_list[selected].pid, 1, offset[1]);
	downloader.strain_get_offset (downloader.board_list[selected].pid, 2, offset[2]);
	downloader.strain_get_offset (downloader.board_list[selected].pid, 3, offset[3]);
	downloader.strain_get_offset (downloader.board_list[selected].pid, 4, offset[4]);
	downloader.strain_get_offset (downloader.board_list[selected].pid, 5, offset[5]);

	downloader.strain_get_adc (downloader.board_list[selected].pid, 0, adc[0]);
	downloader.strain_get_adc (downloader.board_list[selected].pid, 1, adc[1]);
	downloader.strain_get_adc (downloader.board_list[selected].pid, 2, adc[2]);
	downloader.strain_get_adc (downloader.board_list[selected].pid, 3, adc[3]);
	downloader.strain_get_adc (downloader.board_list[selected].pid, 4, adc[4]);
	downloader.strain_get_adc (downloader.board_list[selected].pid, 5, adc[5]);

	int ri,ci=0;
	char tempbuf [50];
	GdkColor color;
	color.red=65535;
	//color.green=0;
	//color.blue=0;

	if (something_changed==true)
	{
		gtk_widget_modify_bg (save_button, GTK_STATE_NORMAL,	   &color);
		gtk_widget_modify_bg (save_button, GTK_STATE_ACTIVE,      &color);
		gtk_widget_modify_bg (save_button, GTK_STATE_PRELIGHT,    &color);
		gtk_widget_modify_bg (save_button, GTK_STATE_SELECTED,    &color);	
		gtk_widget_modify_bg (save_button, GTK_STATE_INSENSITIVE, &color);
	}

	if (matrix_changed==false)
		for (ri=0;ri<6;ri++)
			for (ci=0;ci<6;ci++)
				{
					downloader.strain_get_matrix_rc(downloader.board_list[selected].pid,ri,ci,matrix[ri][ci]);
					sprintf(tempbuf,"%x",matrix[ri][ci]);
					gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci]), tempbuf);
					gtk_widget_modify_base (edit_matrix[ri][ci],GTK_STATE_NORMAL, NULL );
				}
	else
		for (ri=0;ri<6;ri++)
			for (ci=0;ci<6;ci++)
				{
					gtk_widget_modify_base (edit_matrix[ri][ci],GTK_STATE_NORMAL, &color );
				}
	for (int i=0;i<6;i++)
	{
		if (adc[i]>maxadc[i]) maxadc[i]=adc[i];
		if (adc[i]<minadc[i]) minadc[i]=adc[i];
	}

	gtk_range_set_value (GTK_RANGE(slider_gain[0]),(offset[0]));
	gtk_range_set_value (GTK_RANGE(slider_gain[1]),(offset[1]));
	gtk_range_set_value (GTK_RANGE(slider_gain[2]),(offset[2]));
	gtk_range_set_value (GTK_RANGE(slider_gain[3]),(offset[3]));
	gtk_range_set_value (GTK_RANGE(slider_gain[4]),(offset[4]));
	gtk_range_set_value (GTK_RANGE(slider_gain[5]),(offset[5]));

	sprintf(tempbuf,"%d",adc[0]);
	gtk_label_set_text(GTK_LABEL(curr_measure[0]),tempbuf);
	sprintf(tempbuf,"%d",adc[1]);
	gtk_label_set_text(GTK_LABEL(curr_measure[1]),tempbuf);
	sprintf(tempbuf,"%d",adc[2]);
	gtk_label_set_text(GTK_LABEL(curr_measure[2]),tempbuf);
	sprintf(tempbuf,"%d",adc[3]);
	gtk_label_set_text(GTK_LABEL(curr_measure[3]),tempbuf);
	sprintf(tempbuf,"%d",adc[4]);
	gtk_label_set_text(GTK_LABEL(curr_measure[4]),tempbuf);
	sprintf(tempbuf,"%d",adc[5]);
	gtk_label_set_text(GTK_LABEL(curr_measure[5]),tempbuf);

	sprintf(tempbuf,"%d",maxadc[0]);
	gtk_label_set_text(GTK_LABEL(max_measure[0]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[1]);
	gtk_label_set_text(GTK_LABEL(max_measure[1]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[2]);
	gtk_label_set_text(GTK_LABEL(max_measure[2]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[3]);
	gtk_label_set_text(GTK_LABEL(max_measure[3]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[4]);
	gtk_label_set_text(GTK_LABEL(max_measure[4]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[5]);
	gtk_label_set_text(GTK_LABEL(max_measure[5]),tempbuf);

	sprintf(tempbuf,"%d",minadc[0]);
	gtk_label_set_text(GTK_LABEL(min_measure[0]),tempbuf);
	sprintf(tempbuf,"%d",minadc[1]);
	gtk_label_set_text(GTK_LABEL(min_measure[1]),tempbuf);
	sprintf(tempbuf,"%d",minadc[2]);
	gtk_label_set_text(GTK_LABEL(min_measure[2]),tempbuf);
	sprintf(tempbuf,"%d",minadc[3]);
	gtk_label_set_text(GTK_LABEL(min_measure[3]),tempbuf);
	sprintf(tempbuf,"%d",minadc[4]);
	gtk_label_set_text(GTK_LABEL(min_measure[4]),tempbuf);
	sprintf(tempbuf,"%d",minadc[5]);
	gtk_label_set_text(GTK_LABEL(min_measure[5]),tempbuf);

	sprintf(tempbuf,"%d",maxadc[0]-minadc[0]);
	gtk_label_set_text(GTK_LABEL(diff_measure[0]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[1]-minadc[1]);
	gtk_label_set_text(GTK_LABEL(diff_measure[1]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[2]-minadc[2]);
	gtk_label_set_text(GTK_LABEL(diff_measure[2]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[3]-minadc[3]);
	gtk_label_set_text(GTK_LABEL(diff_measure[3]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[4]-minadc[4]);
	gtk_label_set_text(GTK_LABEL(diff_measure[4]),tempbuf);
	sprintf(tempbuf,"%d",maxadc[5]-minadc[5]);
	gtk_label_set_text(GTK_LABEL(diff_measure[5]),tempbuf);

	return true;
}

//*********************************************************************************
void close_window (GtkDialog *window,	gpointer   user_data)
{ 
	STOP_TIMER
    gtk_widget_destroy (GTK_WIDGET(window));
}

//*********************************************************************************
void slider_changed (GtkButton *button,	gpointer ch_p)
{ 
	int chan = *(int*)ch_p;
	offset[chan] = (unsigned int) (gtk_range_get_value (GTK_RANGE(slider_gain[chan])));
	unsigned int curr_offset;
//	downloader.strain_get_offset (downloader.board_list[selected].pid, chan, curr_offset);

//	if (offset[chan]!=curr_offset)
		{
//			something_changed=true;
			downloader.strain_set_offset (downloader.board_list[selected].pid, chan, offset[chan]);
		}
}

//*********************************************************************************
void file_save_click (GtkButton *button,	gpointer ch_p)
{ 
	std::string filename = "C:\\Software\\iCub\\bin\\debug\\ciao.dat";
	fstream filestr;
	filestr.open (filename.c_str(), fstream::out);
	int i=0;
	char buffer[256];
	for (i=0;i<36; i++)
	{
		sprintf (buffer,"%x",matrix[i/6][i%6]);
		filestr<<buffer<<endl;
	}
	printf ("Calibration file saved!\n");
	filestr.close();
}

//*********************************************************************************
void file_load_click (GtkButton *button,	gpointer ch_p)
{ 
	std::string filename = "C:\\Software\\iCub\\bin\\debug\\ciao.dat";

	char* buff;

	buff = gtk_file_chooser_get_filename   (GTK_FILE_CHOOSER(picker_calib));
	if (buff==NULL)
		{
			printf ("ERR: File not found!\n");
			return;
		}

	fstream filestr;
	filestr.open (buff, fstream::in);
    if (!filestr.is_open())
        {
            printf ("ERR: Error opening calibration file!\n");
            return;
        }

	int i=0;
	char buffer[256];
	for (i=0;i<36; i++)
	{
		int ri=i/6;
		int ci=i%6;
		filestr.getline (buffer,256);
		sscanf (buffer,"%x",&calib_matrix[ri][ci]);
		printf("%d %x\n", calib_matrix[ri][ci],calib_matrix[ri][ci]);
		downloader.strain_set_matrix_rc(downloader.board_list[selected].pid,ri,ci,calib_matrix[ri][ci]);
	}
	filestr.close();
	matrix_changed=true;
	something_changed=true;
	printf ("Calibration file loaded!\n");

	int ri=0;
	int ci=0;
	drv_sleep (1000);
	for (ri=0;ri<6;ri++)
			for (ci=0;ci<6;ci++)
				{
					downloader.strain_get_matrix_rc(downloader.board_list[selected].pid,ri,ci,matrix[ri][ci]);
					sprintf(buffer,"%x",matrix[ri][ci]);
					gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci]), buffer);
					gtk_widget_modify_base (edit_matrix[ri][ci],GTK_STATE_NORMAL, NULL );
				}

	int count_ok=0;
	for (i=0;i<36; i++)
	{
		ri=i/6;
		ci=i%6;
		if (calib_matrix[ri][ci]==matrix[ri][ci])
		{
			count_ok++;
		}
	}
	if (count_ok==36)
	{
		printf ("Calibration file applied with no errors\n");
		matrix_changed=false;
	}
	else
	{
		printf ("Found %d errors applying the calibration file!!\n",36-count_ok);
	}
}

//*********************************************************************************
void zero_changed (GtkButton *button,	gpointer ch_p)
{ 
	calibration_value = (unsigned int) (gtk_range_get_value (GTK_RANGE(slider_zero)));
}

//*********************************************************************************
void matrix_change (GtkEntry *entry,	gpointer index)
{ 
	matrix_changed=true;
	something_changed=true;
	//printf("Calibration matrix changed\n");
}

//*********************************************************************************
void matrix_send (GtkEntry *entry,	gpointer index)
{ 
	int ri=0;
	int ci=0;

	for (ri=0; ri<6; ri++)
		for (ci=0; ci<6; ci++)
			{
				const gchar* temp2 = gtk_entry_get_text (GTK_ENTRY (edit_matrix[ri][ci]));
				sscanf (temp2,"%x",&matrix[ri][ci]);
				downloader.strain_set_matrix_rc(downloader.board_list[selected].pid,ri,ci,matrix[ri][ci]);
			}

	printf("Calibration matrix updated\n");
	matrix_changed=false;

	for (ri=0; ri<6; ri++)
		for (ci=0; ci<6; ci++)
			gtk_widget_modify_base (edit_matrix[ri][ci],GTK_STATE_NORMAL, NULL );
}

//*********************************************************************************
void calibrate_click (GtkButton *button,	gpointer   user_data)
{ 
	matrix_changed=false;
	something_changed=false;
	//which strain board is selected)
    int i        = 0;
	int count    = 0;
    for (i=0; i<downloader.board_list_size; i++)
    {
        if (downloader.board_list[i].status==BOARD_RUNNING &&
			downloader.board_list[i].type==BOARD_TYPE_STRAIN &&
            downloader.board_list[i].selected==true)
			{
				selected = i;
				count++;
			}
    }
	//only one board can be calibrated!!
	if (count!=1) return;

	GtkWidget *calib_window;
	GtkWidget *fixed;
	GtkWidget *auto_button;
	GtkWidget *file_load_button;
	GtkWidget *file_save_button;
	GtkWidget* label_gain[6];
	GtkWidget* label_meas[6];	
/*
	calib_window = gtk_dialog_new_with_buttons ("Calibration Dialog",
            GTK_WINDOW (window),
            GTK_DIALOG_MODAL,
            GTK_STOCK_OK,
            GTK_RESPONSE_OK,
            NULL);
*/
	calib_window = gtk_dialog_new ();
	gtk_window_resize (GTK_WINDOW(calib_window),300,480);
	char buff[255];
	sprintf(buff, "Calibration of strain board ID: %d", downloader.board_list[selected].pid);
	gtk_window_set_title    (GTK_WINDOW(calib_window),buff);
	gtk_dialog_set_has_separator (GTK_DIALOG(calib_window),false);
  
	for (i=0;i<6;i++)
	{
		adc[i]=0;
		maxadc[i]=0;
		minadc[i]=65535;
	}

	fixed = gtk_fixed_new();
	picker_calib = gtk_file_chooser_button_new ("Pick a File", GTK_FILE_CHOOSER_ACTION_OPEN);
	auto_button   = gtk_button_new_with_mnemonic ("Automatic \nCalibration"); 
	save_button   = gtk_button_new_with_label ("Save to eeprom"); 
	file_load_button   = gtk_button_new_with_mnemonic ("Load Calibration File"); 
	file_save_button   = gtk_button_new_with_mnemonic ("Save Calibration File"); 
	curr_measure[0] = gtk_label_new_with_mnemonic ("32000");
	curr_measure[1] = gtk_label_new_with_mnemonic ("32000");
	curr_measure[2] = gtk_label_new_with_mnemonic ("32000");
	curr_measure[3] = gtk_label_new_with_mnemonic ("32000");
	curr_measure[4] = gtk_label_new_with_mnemonic ("32000");
	curr_measure[5] = gtk_label_new_with_mnemonic ("32000");
	label_meas[0] = gtk_label_new_with_mnemonic ("Channel 0:");
	label_meas[1] = gtk_label_new_with_mnemonic ("Channel 1:");
	label_meas[2] = gtk_label_new_with_mnemonic ("Channel 2:");
	label_meas[3] = gtk_label_new_with_mnemonic ("Channel 3:");
	label_meas[4] = gtk_label_new_with_mnemonic ("Channel 4:");
	label_meas[5] = gtk_label_new_with_mnemonic ("Channel 5:");
	label_gain[0] = gtk_label_new_with_mnemonic ("Gain 0:");
	label_gain[1] = gtk_label_new_with_mnemonic ("Gain 1:");
	label_gain[2] = gtk_label_new_with_mnemonic ("Gain 2:");
	label_gain[3] = gtk_label_new_with_mnemonic ("Gain 3:");
	label_gain[4] = gtk_label_new_with_mnemonic ("Gain 4:");
	label_gain[5] = gtk_label_new_with_mnemonic ("Gain 5:");

	max_measure[0] = gtk_label_new_with_mnemonic ("32000");
	max_measure[1] = gtk_label_new_with_mnemonic ("32000");
	max_measure[2] = gtk_label_new_with_mnemonic ("32000");
	max_measure[3] = gtk_label_new_with_mnemonic ("32000");
	max_measure[4] = gtk_label_new_with_mnemonic ("32000");
	max_measure[5] = gtk_label_new_with_mnemonic ("32000");
	min_measure[0] = gtk_label_new_with_mnemonic ("32000");
	min_measure[1] = gtk_label_new_with_mnemonic ("32000");
	min_measure[2] = gtk_label_new_with_mnemonic ("32000");
	min_measure[3] = gtk_label_new_with_mnemonic ("32000");
	min_measure[4] = gtk_label_new_with_mnemonic ("32000");
	min_measure[5] = gtk_label_new_with_mnemonic ("32000");
	diff_measure[0] = gtk_label_new_with_mnemonic ("32000");
	diff_measure[1] = gtk_label_new_with_mnemonic ("32000");
	diff_measure[2] = gtk_label_new_with_mnemonic ("32000");
	diff_measure[3] = gtk_label_new_with_mnemonic ("32000");
	diff_measure[4] = gtk_label_new_with_mnemonic ("32000");
	diff_measure[5] = gtk_label_new_with_mnemonic ("32000");

	slider_gain[0] = gtk_hscale_new_with_range   (0,0x3FF,1);
	slider_gain[1] = gtk_hscale_new_with_range   (0,0x3FF,1);
	slider_gain[2] = gtk_hscale_new_with_range   (0,0x3FF,1);
	slider_gain[3] = gtk_hscale_new_with_range   (0,0x3FF,1);
	slider_gain[4] = gtk_hscale_new_with_range   (0,0x3FF,1);
	slider_gain[5] = gtk_hscale_new_with_range   (0,0x3FF,1);
	slider_zero	   = gtk_hscale_new_with_range   (0,65535,1);

	int ri,ci=0;
	for (ri=0;ri<6;ri++)
		for (ci=0;ci<6;ci++)
			edit_matrix[ri][ci] = gtk_entry_new ();

	gtk_container_add  (GTK_CONTAINER(GTK_BOX (GTK_DIALOG (calib_window)->vbox)),fixed);
	
	int r[6]={0+10,60+10,60*2+10,60*3+10,60*4+10,60*5+10};
	int c[7]={0+10,50+10,150+10,230+10,350+10,400+10,450+10};

	gtk_fixed_put(GTK_FIXED(fixed),label_gain[0],c[0],r[0]);
	gtk_fixed_put(GTK_FIXED(fixed),label_gain[1],c[0],r[1]);
	gtk_fixed_put(GTK_FIXED(fixed),label_gain[2],c[0],r[2]);
	gtk_fixed_put(GTK_FIXED(fixed),label_gain[3],c[0],r[3]);
	gtk_fixed_put(GTK_FIXED(fixed),label_gain[4],c[0],r[4]);
	gtk_fixed_put(GTK_FIXED(fixed),label_gain[5],c[0],r[5]);

	gtk_fixed_put(GTK_FIXED(fixed),slider_gain[0],c[1],r[0]-10);
	gtk_fixed_put(GTK_FIXED(fixed),slider_gain[1],c[1],r[1]-10);
	gtk_fixed_put(GTK_FIXED(fixed),slider_gain[2],c[1],r[2]-10);
	gtk_fixed_put(GTK_FIXED(fixed),slider_gain[3],c[1],r[3]-10);
	gtk_fixed_put(GTK_FIXED(fixed),slider_gain[4],c[1],r[4]-10);
	gtk_fixed_put(GTK_FIXED(fixed),slider_gain[5],c[1],r[5]-10);

	for (ri=0;ri<6;ri++)
		for (ci=0;ci<6;ci++)
			{
			 gtk_fixed_put(GTK_FIXED(fixed),edit_matrix[ci][ri],c[5]+ri*42,r[5]+40+ci*32);
			 gtk_widget_set_size_request(edit_matrix[ri][ci],40,20);
			}

	gtk_widget_set_size_request(slider_gain[0],80,30);
	gtk_widget_set_size_request(slider_gain[1],80,30);
	gtk_widget_set_size_request(slider_gain[2],80,30);
	gtk_widget_set_size_request(slider_gain[3],80,30);
	gtk_widget_set_size_request(slider_gain[4],80,30);
	gtk_widget_set_size_request(slider_gain[5],80,30);

	gtk_widget_set_size_request(slider_zero,100,30);

	gtk_fixed_put(GTK_FIXED(fixed),label_meas[0],c[2],r[0]);
	gtk_fixed_put(GTK_FIXED(fixed),label_meas[1],c[2],r[1]);
	gtk_fixed_put(GTK_FIXED(fixed),label_meas[2],c[2],r[2]);
	gtk_fixed_put(GTK_FIXED(fixed),label_meas[3],c[2],r[3]);
	gtk_fixed_put(GTK_FIXED(fixed),label_meas[4],c[2],r[4]);
	gtk_fixed_put(GTK_FIXED(fixed),label_meas[5],c[2],r[5]);

	gtk_fixed_put(GTK_FIXED(fixed),curr_measure[0],c[3],r[0]);
	gtk_fixed_put(GTK_FIXED(fixed),curr_measure[1],c[3],r[1]);
	gtk_fixed_put(GTK_FIXED(fixed),curr_measure[2],c[3],r[2]);
	gtk_fixed_put(GTK_FIXED(fixed),curr_measure[3],c[3],r[3]);
	gtk_fixed_put(GTK_FIXED(fixed),curr_measure[4],c[3],r[4]);
	gtk_fixed_put(GTK_FIXED(fixed),curr_measure[5],c[3],r[5]);

	gtk_fixed_put(GTK_FIXED(fixed),max_measure[0],c[4],r[0]);
	gtk_fixed_put(GTK_FIXED(fixed),max_measure[1],c[4],r[1]);
	gtk_fixed_put(GTK_FIXED(fixed),max_measure[2],c[4],r[2]);
	gtk_fixed_put(GTK_FIXED(fixed),max_measure[3],c[4],r[3]);
	gtk_fixed_put(GTK_FIXED(fixed),max_measure[4],c[4],r[4]);
	gtk_fixed_put(GTK_FIXED(fixed),max_measure[5],c[4],r[5]);

	gtk_fixed_put(GTK_FIXED(fixed),min_measure[0],c[5],r[0]);
	gtk_fixed_put(GTK_FIXED(fixed),min_measure[1],c[5],r[1]);
	gtk_fixed_put(GTK_FIXED(fixed),min_measure[2],c[5],r[2]);
	gtk_fixed_put(GTK_FIXED(fixed),min_measure[3],c[5],r[3]);
	gtk_fixed_put(GTK_FIXED(fixed),min_measure[4],c[5],r[4]);
	gtk_fixed_put(GTK_FIXED(fixed),min_measure[5],c[5],r[5]);

	gtk_fixed_put(GTK_FIXED(fixed),diff_measure[0],c[6],r[0]);
	gtk_fixed_put(GTK_FIXED(fixed),diff_measure[1],c[6],r[1]);
	gtk_fixed_put(GTK_FIXED(fixed),diff_measure[2],c[6],r[2]);
	gtk_fixed_put(GTK_FIXED(fixed),diff_measure[3],c[6],r[3]);
	gtk_fixed_put(GTK_FIXED(fixed),diff_measure[4],c[6],r[4]);
	gtk_fixed_put(GTK_FIXED(fixed),diff_measure[5],c[6],r[5]);

	gtk_widget_set_size_request(auto_button,100,40);
	gtk_widget_set_size_request(save_button,140,40);
	gtk_widget_set_size_request(file_load_button,140,40);
	gtk_widget_set_size_request(file_save_button,140,40);
	gtk_widget_set_size_request(picker_calib,140,40);
	g_signal_connect (file_load_button, "clicked", G_CALLBACK (file_load_click),NULL);
	g_signal_connect (file_save_button, "clicked", G_CALLBACK (file_save_click),NULL);
	g_signal_connect (auto_button, "clicked", G_CALLBACK (auto_click),NULL);
    g_signal_connect (save_button, "clicked", G_CALLBACK (save_click),NULL);
	g_signal_connect (calib_window, "response", G_CALLBACK (close_window),NULL);

	g_signal_connect (slider_gain[0], "value-changed", G_CALLBACK (slider_changed),&ch[0]);
	g_signal_connect (slider_gain[1], "value-changed", G_CALLBACK (slider_changed),&ch[1]);
	g_signal_connect (slider_gain[2], "value-changed", G_CALLBACK (slider_changed),&ch[2]);
	g_signal_connect (slider_gain[3], "value-changed", G_CALLBACK (slider_changed),&ch[3]);
	g_signal_connect (slider_gain[4], "value-changed", G_CALLBACK (slider_changed),&ch[4]);
	g_signal_connect (slider_gain[5], "value-changed", G_CALLBACK (slider_changed),&ch[5]);
	g_signal_connect (slider_zero, "value-changed", G_CALLBACK (zero_changed),NULL);
	
	gtk_fixed_put(GTK_FIXED(fixed),auto_button,c[1]-20,r[5]+40);
	gtk_fixed_put(GTK_FIXED(fixed),slider_zero,c[1]-20,r[5]+100);
	gtk_fixed_put(GTK_FIXED(fixed),save_button,c[2]+10,r[5]+40);
	gtk_fixed_put(GTK_FIXED(fixed),file_load_button,c[2]+10,r[5]+90);
	gtk_fixed_put(GTK_FIXED(fixed),file_save_button,c[2]+10,r[5]+140);
	gtk_fixed_put(GTK_FIXED(fixed),picker_calib,c[2]+10,r[5]+190);

	gtk_range_set_value (GTK_RANGE(slider_zero),calibration_value);

	downloader.strain_start_sampling(downloader.board_list[selected].pid);

	STOP_TIMER
	START_TIMER	
	timer_func (NULL);

	int index[36];
	for (ri=0;ri<6;ri++)
	for (ci=0;ci<6;ci++)
		{
			index[ri*6+ci]=ri*100+ci;
			g_signal_connect(edit_matrix[ri][ci], "changed", G_CALLBACK (matrix_change),&index[ri*6+ci]);
			g_signal_connect(edit_matrix[ri][ci], "activate", G_CALLBACK (matrix_send),&index[ri*6+ci]);
		}

	gtk_widget_show_all (fixed);
	//gtk_window_set_resizable(GTK_WINDOW(calib_window),false);
    gtk_dialog_run (GTK_DIALOG (calib_window));
}
