// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "downloader.h"
#include "calibrate_window.h"
#include <canProtocolLib/iCubCanProtocol.h>
#include <canProtocolLib/iCubCanProto_types.h>
#include <fstream>
#include <yarp/os/Log.h>

using namespace std;

extern cDownloader downloader;
extern GtkWidget *window;
int       selected = 0;
unsigned int calibration_value=32767;
int ch[6]={0,1,2,3,4,5};
unsigned int offset[6];
unsigned int amp_gain1[6];
unsigned int amp_gain2[6];
unsigned int adc[6]={0,0,0,0,0,0};
unsigned int maxadc[6]={0,0,0,0,0,0};
int calib_bias[6]={0,0,0,0,0,0};
int curr_bias[6]={0,0,0,0,0,0};
unsigned int minadc[6]={65535,65535,65535,65535,65535,65535};
unsigned int matrix[6][6][3];
unsigned int calib_matrix[6][6][3];
guint  timer_refresh = 0;
GtkWidget* curr_measure[6];
GtkWidget* max_measure[6];
GtkWidget* min_measure[6];
GtkWidget* diff_measure[6];
GtkWidget* newton_measure[6];
GtkWidget* edit_matrix[6][6][3];
GtkWidget* edit_matrix_gain;
GtkWidget* label_matrix_gain;
GtkWidget* slider_gain[6];
GtkWidget* slider_zero;
GtkWidget* info_dlg;
GtkWidget* picker_calib;
GtkWidget *save_button;
GtkWidget *matrix_reset_button;
GtkWidget *set_calib_bias_button;
GtkWidget *reset_calib_bias_button;
GtkWidget *set_curr_bias_button;
GtkWidget *reset_curr_bias_button;
GtkWidget *curr_bias_label[6];
GtkWidget *calib_bias_label[6];
GtkWidget *full_scale_label[6];
GtkWidget *edit_serial_number;
gboolean timer_func (gpointer data);
GtkWidget* label_use_calib;
GtkWidget* spin_use_calib;

unsigned int calib_const=0;
unsigned int full_scale_const[6]={0,0,0,0,0,0};
char serial_no[8]={'U','N','D','E','F',0,0,0};
bool matrix_changed[3];
bool serial_number_changed;
bool something_changed;
bool eeprom_saved_status;
bool first_time[6]={1,1,1,1,1,1};

#define START_TIMER timer_refresh = g_timeout_add (500, timer_func, NULL);
#define STOP_TIMER {if (timer_refresh>0) g_source_remove(timer_refresh); timer_refresh=0;}
#define HEX_VALC 0x8000

int get_matrix()
{
    int id=0;
    int calibration = (int)gtk_spin_button_get_value (GTK_SPIN_BUTTON (spin_use_calib));
    if (calibration <0 || calibration >= 4) {printf("debug: invalid value of 'edit_use_calib'.\n"); calibration = 0;}
    if (calibration<=0) id = 0; //@@@@@@@@@@@
    else                id = calibration-1;
    return id;
}

void show_matrix(int id)
{
    int ri=0; int ci=0;
    if (id<0)
    {
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][0]), true);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][1]), true);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][2]), true);}
    }
    else if (id==0)
    {
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][0]), true);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][1]), false);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][2]), false);}
    }
    else if (id==1)
    {
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][0]), false);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][1]), true);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][2]), false);}
    }
    else if (id==2)
    {
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][0]), false);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][1]), false);}
        for (ri=0;ri<6;ri++) for (ci=0;ci<6;ci++) {gtk_widget_set_sensitive(GTK_WIDGET (edit_matrix[ri][ci][2]), true);}
    }
}

int get_calibration()
{
    int id=0;
    int calibration = (int)gtk_spin_button_get_value (GTK_SPIN_BUTTON (spin_use_calib));
    if (calibration <0 || calibration >= 4) {printf("debug: invalid value of 'edit_use_calib'.\n"); calibration = 0;}
    return calibration;
}

//*********************************************************************************
void save_click (GtkButton *button,    gpointer   user_data)
{
    STOP_TIMER
    drv_sleep (1000);
    downloader.strain_save_to_eeprom(downloader.board_list[selected].pid);
    drv_sleep (1000);
    something_changed=false;
    START_TIMER
}

//*********************************************************************************
void auto_click (GtkButton *button,    gpointer   user_data)
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
    int ret=0;
    int calibration = get_calibration();
    ret =downloader.strain_get_eeprom_saved(downloader.board_list[selected].pid, &eeprom_saved_status);
    if (ret!=0) printf("debug: message 'strain_get_eeprom_saved' lost.\n");

    /*
    ret =downloader.sg6_get_amp_gain (downloader.board_list[selected].pid, 0, amp_gain1[0], amp_gain2[0]);
    ret|=downloader.sg6_get_amp_gain (downloader.board_list[selected].pid, 1, amp_gain1[1], amp_gain2[1]);
    ret|=downloader.sg6_get_amp_gain (downloader.board_list[selected].pid, 2, amp_gain1[2], amp_gain2[2]);
    ret|=downloader.sg6_get_amp_gain (downloader.board_list[selected].pid, 3, amp_gain1[3], amp_gain2[3]);
    ret|=downloader.sg6_get_amp_gain (downloader.board_list[selected].pid, 4, amp_gain1[4], amp_gain2[4]);
    ret|=downloader.sg6_get_amp_gain (downloader.board_list[selected].pid, 5, amp_gain1[5], amp_gain2[5]);
    if (ret!=0) printf("debug: message 'sg6_get_amp_gain' lost.\n");
    */

    ret =downloader.strain_get_offset (downloader.board_list[selected].pid, 0, offset[0]);
    ret|=downloader.strain_get_offset (downloader.board_list[selected].pid, 1, offset[1]);
    ret|=downloader.strain_get_offset (downloader.board_list[selected].pid, 2, offset[2]);
    ret|=downloader.strain_get_offset (downloader.board_list[selected].pid, 3, offset[3]);
    ret|=downloader.strain_get_offset (downloader.board_list[selected].pid, 4, offset[4]);
    ret|=downloader.strain_get_offset (downloader.board_list[selected].pid, 5, offset[5]);
    if (ret!=0) printf("debug: message 'strain_get_offset' lost.\n");

    ret =downloader.strain_get_adc (downloader.board_list[selected].pid, 0, adc[0], calibration);
    ret|=downloader.strain_get_adc (downloader.board_list[selected].pid, 1, adc[1], calibration);
    ret|=downloader.strain_get_adc (downloader.board_list[selected].pid, 2, adc[2], calibration);
    ret|=downloader.strain_get_adc (downloader.board_list[selected].pid, 3, adc[3], calibration);
    ret|=downloader.strain_get_adc (downloader.board_list[selected].pid, 4, adc[4], calibration);
    ret|=downloader.strain_get_adc (downloader.board_list[selected].pid, 5, adc[5], calibration);
    if (ret!=0) printf("debug: message 'strain_get_adc' lost.\n");

    int ri,ci,id,tid=0;
    char tempbuf [250];
    GdkColor r_color,y_color;
    r_color.red=65535;
    r_color.green=39000;
    r_color.blue=39000;
    y_color.red=65535;
    y_color.green=65535;
    y_color.blue=38000;

    if (eeprom_saved_status==false)
    {
        gtk_widget_modify_bg (save_button, GTK_STATE_NORMAL,      &r_color);
        gtk_widget_modify_bg (save_button, GTK_STATE_ACTIVE,      &r_color);
        gtk_widget_modify_bg (save_button, GTK_STATE_PRELIGHT,    &r_color);
        gtk_widget_modify_bg (save_button, GTK_STATE_SELECTED,    &r_color);
        gtk_widget_modify_bg (save_button, GTK_STATE_INSENSITIVE, &r_color);
        gtk_button_set_label     (GTK_BUTTON(save_button), "Not saved.\nSave to eeprom?");    }
    else
    {
        gtk_button_set_label     (GTK_BUTTON(save_button), "Save to eeprom");
    }
/*    if (something_changed==true)
    {
        gtk_widget_modify_base (save_button, GTK_STATE_NORMAL,      &r_color);
        gtk_widget_modify_bg (save_button,   GTK_STATE_NORMAL,        &r_color);
        gtk_widget_modify_bg (save_button,   GTK_STATE_ACTIVE,      &r_color);
        gtk_widget_modify_bg (save_button,   GTK_STATE_PRELIGHT,    &r_color);
        gtk_widget_modify_bg (save_button,   GTK_STATE_SELECTED,    &r_color);
        gtk_widget_modify_bg (save_button,   GTK_STATE_INSENSITIVE, &r_color);
    }*/

    if (serial_number_changed==false)
        {
            gtk_widget_modify_base (edit_serial_number,GTK_STATE_NORMAL, NULL );
        }
    else
        {
            gtk_widget_modify_base (edit_serial_number,GTK_STATE_NORMAL, &r_color );
        }

    id = get_matrix();
    show_matrix(id);
    if (matrix_changed[id]==false)
        {
            for (ri=0;ri<6;ri++)
                for (ci=0;ci<6;ci++)
                    {
                        downloader.strain_get_matrix_rc(downloader.board_list[selected].pid,ri,ci,id,matrix[ri][ci][id]);
                        sprintf(tempbuf,"%x",matrix[ri][ci][id]);
                        gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci][id]), tempbuf);
                        gtk_widget_modify_base (edit_matrix[ri][ci][id],GTK_STATE_NORMAL, NULL );
                    }
            downloader.strain_get_matrix_gain(downloader.board_list[selected].pid,calib_const);
            sprintf(tempbuf,"%d",calib_const);
            gtk_label_set_text (GTK_LABEL(edit_matrix_gain), tempbuf);

            for (ri=0;ri<6;ri++)
            {
                downloader.strain_get_full_scale(downloader.board_list[selected].pid,ri,full_scale_const[ri]);
                sprintf(tempbuf,"%d",full_scale_const[ri]);
                gtk_label_set_text (GTK_LABEL(full_scale_label[ri]), tempbuf);
            }
        }
    else
        {
            for (ri=0;ri<6;ri++)
                for (ci=0;ci<6;ci++)
 //                   for (tid=0;tid<3;tid++)
                    {
                        gtk_widget_modify_base (edit_matrix[ri][ci][id],GTK_STATE_NORMAL, &r_color );
                    }
        }

    for (int i=0;i<6;i++)
    {
        if (calibration==0)
        {
            if (adc[i]>maxadc[i]) maxadc[i]=adc[i];
            if (adc[i]<minadc[i]) minadc[i]=adc[i];
        }
        else
        {
            maxadc[i]=0;
            minadc[i]=65535;
        }
    }

    for (int i=0;i<6;i++)
    {
        downloader.strain_get_calib_bias(downloader.board_list[selected].pid,i,calib_bias[i]);
        sprintf(tempbuf,"%d",calib_bias[i]);
        gtk_label_set_text (GTK_LABEL(calib_bias_label[i]), tempbuf);

        downloader.strain_get_curr_bias(downloader.board_list[selected].pid,i,curr_bias[i]);
        sprintf(tempbuf,"%d",curr_bias[i]);
        gtk_label_set_text (GTK_LABEL(curr_bias_label[i]), tempbuf);
    }

    gtk_range_set_value (GTK_RANGE(slider_gain[0]),(offset[0]));
    gtk_range_set_value (GTK_RANGE(slider_gain[1]),(offset[1]));
    gtk_range_set_value (GTK_RANGE(slider_gain[2]),(offset[2]));
    gtk_range_set_value (GTK_RANGE(slider_gain[3]),(offset[3]));
    gtk_range_set_value (GTK_RANGE(slider_gain[4]),(offset[4]));
    gtk_range_set_value (GTK_RANGE(slider_gain[5]),(offset[5]));

    sprintf(tempbuf,"%d",adc[0]-HEX_VALC); //@@@@ changed here
    gtk_label_set_text(GTK_LABEL(curr_measure[0]),tempbuf);
    sprintf(tempbuf,"%d",adc[1]-HEX_VALC);
    gtk_label_set_text(GTK_LABEL(curr_measure[1]),tempbuf);
    sprintf(tempbuf,"%d",adc[2]-HEX_VALC);
    gtk_label_set_text(GTK_LABEL(curr_measure[2]),tempbuf);
    sprintf(tempbuf,"%d",adc[3]-HEX_VALC);
    gtk_label_set_text(GTK_LABEL(curr_measure[3]),tempbuf);
    sprintf(tempbuf,"%d",adc[4]-HEX_VALC);
    gtk_label_set_text(GTK_LABEL(curr_measure[4]),tempbuf);
    sprintf(tempbuf,"%d",adc[5]-HEX_VALC);
    gtk_label_set_text(GTK_LABEL(curr_measure[5]),tempbuf);

    if (calibration>0)
    {
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(max_measure[0]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(max_measure[1]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(max_measure[2]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(max_measure[3]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(max_measure[4]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(max_measure[5]),tempbuf);

        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(min_measure[0]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(min_measure[1]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(min_measure[2]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(min_measure[3]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(min_measure[4]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(min_measure[5]),tempbuf);

        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(diff_measure[0]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(diff_measure[1]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(diff_measure[2]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(diff_measure[3]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(diff_measure[4]),tempbuf);
        sprintf(tempbuf,"---");
        gtk_label_set_text(GTK_LABEL(diff_measure[5]),tempbuf);

        /*
        //previous version
        sprintf(tempbuf,"%+.3f N",(int(adc[0])-HEX_VALC)/float(calib_const));
        gtk_label_set_text(GTK_LABEL(newton_measure[0]),tempbuf);
        sprintf(tempbuf,"%+.3f N",(int(adc[1])-HEX_VALC)/float(calib_const));
        gtk_label_set_text(GTK_LABEL(newton_measure[1]),tempbuf);
        sprintf(tempbuf,"%+.3f N",(int(adc[2])-HEX_VALC)/float(calib_const));
        gtk_label_set_text(GTK_LABEL(newton_measure[2]),tempbuf);
        sprintf(tempbuf,"%+.3f N/m",(int(adc[3])-HEX_VALC)/float(calib_const));
        gtk_label_set_text(GTK_LABEL(newton_measure[3]),tempbuf);
        sprintf(tempbuf,"%+.3f N/m",(int(adc[4])-HEX_VALC)/float(calib_const));
        gtk_label_set_text(GTK_LABEL(newton_measure[4]),tempbuf);
        sprintf(tempbuf,"%+.3f N/m",(int(adc[5])-HEX_VALC)/float(calib_const));
        gtk_label_set_text(GTK_LABEL(newton_measure[5]),tempbuf);*/

        bool skip_display_calib=false;
        for (int i=0; i<6; i++)
        {
            if (full_scale_const[i]==0)
            {
                printf("Error getting the full scale %d from the sensor\n",i);
                skip_display_calib=true;
            }
        }

        if (skip_display_calib==false)
        {
            sprintf(tempbuf,"%+.3f N",(int(adc[0])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0]);
            gtk_label_set_text(GTK_LABEL(newton_measure[0]),tempbuf);
            sprintf(tempbuf,"%+.3f N",(int(adc[1])-HEX_VALC)/float(HEX_VALC)*full_scale_const[1]);
            gtk_label_set_text(GTK_LABEL(newton_measure[1]),tempbuf);
            sprintf(tempbuf,"%+.3f N",(int(adc[2])-HEX_VALC)/float(HEX_VALC)*full_scale_const[2]);
            gtk_label_set_text(GTK_LABEL(newton_measure[2]),tempbuf);
            sprintf(tempbuf,"%+.3f N/m",(int(adc[3])-HEX_VALC)/float(HEX_VALC)*full_scale_const[3]);
            gtk_label_set_text(GTK_LABEL(newton_measure[3]),tempbuf);
            sprintf(tempbuf,"%+.3f N/m",(int(adc[4])-HEX_VALC)/float(HEX_VALC)*full_scale_const[4]);
            gtk_label_set_text(GTK_LABEL(newton_measure[4]),tempbuf);
            sprintf(tempbuf,"%+.3f N/m",(int(adc[5])-HEX_VALC)/float(HEX_VALC)*full_scale_const[5]);
            gtk_label_set_text(GTK_LABEL(newton_measure[5]),tempbuf);
        }
        else
        {
            gtk_label_set_text(GTK_LABEL(newton_measure[0]),"ERROR");
            gtk_label_set_text(GTK_LABEL(newton_measure[1]),"ERROR");
            gtk_label_set_text(GTK_LABEL(newton_measure[2]),"ERROR");
            gtk_label_set_text(GTK_LABEL(newton_measure[3]),"ERROR");
            gtk_label_set_text(GTK_LABEL(newton_measure[4]),"ERROR");
            gtk_label_set_text(GTK_LABEL(newton_measure[5]),"ERROR");
        }
    }
    else
    {
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

        sprintf(tempbuf,"--- N");
        gtk_label_set_text(GTK_LABEL(newton_measure[0]),tempbuf);
        sprintf(tempbuf,"--- N");
        gtk_label_set_text(GTK_LABEL(newton_measure[1]),tempbuf);
        sprintf(tempbuf,"--- N");
        gtk_label_set_text(GTK_LABEL(newton_measure[2]),tempbuf);
        sprintf(tempbuf,"--- N/m");
        gtk_label_set_text(GTK_LABEL(newton_measure[3]),tempbuf);
        sprintf(tempbuf,"--- N/m");
        gtk_label_set_text(GTK_LABEL(newton_measure[4]),tempbuf);
        sprintf(tempbuf,"--- N/m");
        gtk_label_set_text(GTK_LABEL(newton_measure[5]),tempbuf);
    }
    return true;
}

//*********************************************************************************
void close_window (GtkDialog *window,    gpointer   user_data)
{
    STOP_TIMER
    gtk_widget_destroy (GTK_WIDGET(window));
}

//*********************************************************************************
void slider_changed (GtkButton *button,    gpointer ch_p)
{
    int chan = *(int*)ch_p;
    //printf("debug: moved slider chan:%d\n",chan);
    offset[chan] = (unsigned int) (gtk_range_get_value (GTK_RANGE(slider_gain[chan])));
//    downloader.strain_get_offset (downloader.board_list[selected].pid, chan, curr_offset);

//    if (offset[chan]!=curr_offset)
/*        {
            something_changed=true;
            downloader.strain_set_offset (downloader.board_list[selected].pid, chan, offset[chan]);
        }*/

    if (first_time[chan]!=true)
        {
            something_changed=true;
            downloader.strain_set_offset (downloader.board_list[selected].pid, chan, offset[chan]);
        }
    first_time[chan]=false;
}

//*********************************************************************************
void file_save_click (GtkButton *button,    gpointer ch_p)
{
    std::string filename = "calibrationData";
    filename += serial_no;
    filename += ".dat";
    fstream filestr;
    filestr.open (filename.c_str(), fstream::out);
    int i=0;
    char buffer[256];

    //file version
    filestr<<"File version:"<<endl;
    filestr<<"3"<<endl;

    //serial number
    filestr<<"Serial number:"<<endl;
    sprintf (buffer,"%s",serial_no);
    filestr<<buffer<<endl;

    //offsets
    filestr<<"Offsets:"<<endl;
    for (i=0;i<6; i++)
    {
        sprintf (buffer,"%d",offset[i]);
        filestr<<buffer<<endl;
    }

    //calibration matrix
    filestr<<"Calibration matrix 0:"<<endl;
    for (i=0;i<36; i++)
    {
        sprintf (buffer,"%x",matrix[i/6][i%6][0]);
        filestr<<buffer<<endl;
    }

    //calibration matrix
    filestr<<"Calibration matrix 1:"<<endl;
    for (i=0;i<36; i++)
    {
        sprintf (buffer,"%x",matrix[i/6][i%6][1]);
        filestr<<buffer<<endl;
    }

    //calibration matrix
    filestr<<"Calibration matrix 2:"<<endl;
    for (i=0;i<36; i++)
    {
        sprintf (buffer,"%x",matrix[i/6][i%6][2]);
        filestr<<buffer<<endl;
    }

    //matrix gain
    filestr<<"Matrix gain:"<<endl;
    sprintf (buffer,"%d",calib_const);
    filestr<<buffer<<endl;

    //tare
    filestr<<"Tare:"<<endl;
    for (i=0;i<6; i++)
    {
        sprintf (buffer,"%d",calib_bias[i]);
        filestr<<buffer<<endl;
    }

    //full scale values
    filestr<<"Full scale values:"<<endl;
    for (i=0;i<6; i++)
    {
        sprintf (buffer,"%d",full_scale_const[i]);
        filestr<<buffer<<endl;
    }

    printf ("Calibration file saved!\n");
    filestr.close();
}

//*********************************************************************************
void reset_matrix_click (GtkButton *button,    gpointer ch_p)
{
    int ri=0;
    int ci=0;
    for (ri=0; ri<6; ri++)
        {
            for (ci=0; ci<6; ci++)
                {
                    if (ri==ci)
                        gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci]),"7fff");
                    else
                        gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci]),"0");
                }
        }
    calib_const=1;
    char tempbuf[255];
    sprintf(tempbuf,"%d",calib_const);
    gtk_entry_set_text (GTK_ENTRY (edit_matrix_gain), tempbuf);
}

//*********************************************************************************
void set_curr_bias_click (GtkButton *button,    gpointer ch_p)
{
    downloader.strain_set_curr_bias(downloader.board_list[selected].pid);
}
//*********************************************************************************
void reset_curr_bias_click (GtkButton *button,    gpointer ch_p)
{
    downloader.strain_reset_curr_bias(downloader.board_list[selected].pid);
}
//*********************************************************************************
void set_calib_bias_click (GtkButton *button,    gpointer ch_p)
{
    something_changed=true;
    downloader.strain_set_calib_bias(downloader.board_list[selected].pid);
}
//*********************************************************************************
void reset_calib_bias_click (GtkButton *button,    gpointer ch_p)
{
    something_changed=true;
    downloader.strain_reset_calib_bias(downloader.board_list[selected].pid);
}

//*********************************************************************************
bool calibration_load_v2 (char* filename, int selected_id);
bool calibration_load_v3 (char* filename, int selected_id);
//*********************************************************************************
bool calibration_load_vX (char* filename, int selected_id)
{
    if (filename==NULL)
        {
            yError("File not found!\n");
            return false;
        }
    if (selected_id <1 || selected_id >= 15)
        {
            yError("Invalid board address!\n");
            return false;
        }

    int file_version=0;
    fstream filestr;
    filestr.open (filename, fstream::in);
    if (!filestr.is_open())
        {
            yError("Error opening calibration file!\n");
            return false;
        }

    int i=0;
    char buffer[256];

    //file version
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sscanf (buffer,"%d",&file_version);
    if (file_version==2)
    {
        filestr.close();
        filestr.clear();
        bool ret = calibration_load_v2(filename,selected_id);
        return ret;
    }
    else
    if (file_version==3)
    {
        filestr.close();
        filestr.clear();
        bool ret = calibration_load_v3(filename,selected_id);
        return ret;
    }
    else
    {
        filestr.close();
        filestr.clear();
        yError("Wrong file version\n");
        return false;
    }
}

//*********************************************************************************
bool calibration_load_v2 (char* filename, int selected_id)
{
    if (filename==NULL)
    {
        yError("File not found!\n");
        return false;
    }
    if (selected_id <1 || selected_id >= 15)
    {
        yError("Invalid board address!\n");
        return false;
    }

    int file_version=0;
    fstream filestr;
    filestr.open (filename, fstream::in);
    if (!filestr.is_open())
    {
        yError("Error opening calibration file!\n");
        return false;
    }

    int i=0;
    char buffer[256];

    //file version
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sscanf (buffer,"%d",&file_version);
    if (file_version!=2)
    {
        yError("Wrong file. Calibration version != 2\n");
        return false;
    }

    //serial number
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sprintf(serial_no,"%s", buffer);
    downloader.strain_set_serial_number(selected_id,serial_no);

    //offsets
    filestr.getline (buffer,256);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&offset[i]);
        downloader.strain_set_offset (downloader.board_list[selected].pid, i, offset[i]);
        drv_sleep(200);
    }

    //calibration matrix
    filestr.getline (buffer,256);
    for (i=0;i<36; i++)
    {
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[ri][ci][0]);
        printf("%d %x\n", calib_matrix[ri][ci][0],calib_matrix[ri][ci][0]);
        downloader.strain_set_matrix_rc(selected_id,ri,ci,0,calib_matrix[ri][ci][0]);
    }

    //matrix gain
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    downloader.strain_set_matrix_gain(selected_id,cc);

    //tare
    filestr.getline (buffer,256);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&calib_bias[i]);
        downloader.strain_set_calib_bias(selected_id,i,calib_bias[i]);
    }

    //full scale values
    filestr.getline (buffer,256);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&full_scale_const[i]);
        downloader.strain_set_full_scale(selected_id,i,full_scale_const[i]);
    }

    filestr.close();
    filestr.clear();

    matrix_changed[0]=true;
    matrix_changed[1]=true;
    matrix_changed[2]=true;
    something_changed=true;
    printf ("Calibration file loaded!\n");

    return true;
}

//*********************************************************************************
bool calibration_load_v3 (char* filename, int selected_id)
{
    if (filename==NULL)
    {
        yError("File not found!\n");
        return false;
    }
    if (selected_id <1 || selected_id >= 15)
    {
        yError("Invalid board address!\n");
        return false;
    }

    int file_version=0;
    fstream filestr;
    filestr.open (filename, fstream::in);
    if (!filestr.is_open())
    {
        yError("Error opening calibration file!\n");
        return false;
    }

    int i=0;
    char buffer[256];

    //file version
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sscanf (buffer,"%d",&file_version);
    if (file_version!=3)
    {
        yError("Wrong file. Calibration version != 3\n");
        return false;
    }

    //serial number
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    sprintf(serial_no,"%s", buffer);
    downloader.strain_set_serial_number(selected_id,serial_no);

    //offsets
    filestr.getline (buffer,256);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&offset[i]);
        downloader.strain_set_offset (downloader.board_list[selected].pid, i, offset[i]);
        drv_sleep(200);
    }

    //calibration matrix A
    filestr.getline (buffer,256);
    for (i=0;i<36; i++)
    {
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[ri][ci][0]);
        printf("%d %x\n", calib_matrix[ri][ci][0],calib_matrix[ri][ci][0]);
        downloader.strain_set_matrix_rc(selected_id,ri,ci,0,calib_matrix[ri][ci][0]);
    }
    //calibration matrix B
    filestr.getline (buffer,256);
    for (i=0;i<36; i++)
    {
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[ri][ci][1]);
        printf("%d %x\n", calib_matrix[ri][ci][1],calib_matrix[ri][ci][1]);
        downloader.strain_set_matrix_rc(selected_id,ri,ci,1,calib_matrix[ri][ci][1]);
    }
    //calibration matrix C
    filestr.getline (buffer,256);
    for (i=0;i<36; i++)
    {
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[ri][ci][2]);
        printf("%d %x\n", calib_matrix[ri][ci][2],calib_matrix[ri][ci][2]);
        downloader.strain_set_matrix_rc(selected_id,ri,ci,2,calib_matrix[ri][ci][2]);
    }

    //matrix gain
    filestr.getline (buffer,256);
    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    downloader.strain_set_matrix_gain(selected_id,cc);

    //tare
    filestr.getline (buffer,256);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&calib_bias[i]);
        downloader.strain_set_calib_bias(selected_id,i,calib_bias[i]);
    }

    //full scale values
    filestr.getline (buffer,256);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf  (buffer,"%d",&full_scale_const[i]);
        downloader.strain_set_full_scale(selected_id,i,full_scale_const[i]);
    }

    filestr.close();
    filestr.clear();

    matrix_changed[0]=true;
    matrix_changed[1]=true;
    matrix_changed[2]=true;
    something_changed=true;
    printf ("Calibration file loaded!\n");

    return true;
}

//*********************************************************************************
void file_load_click (GtkButton *button,    gpointer ch_p)
{
    int selected_id=downloader.board_list[selected].pid;

    char* buff;
    buff = gtk_file_chooser_get_filename   (GTK_FILE_CHOOSER(picker_calib));
    if (buff==NULL)
        {
            yError("File not found!\n");
            return;
        }

    //load data file
    calibration_load_vX (buff, selected_id);

    //update windows graphics
    int i=0;
    int ri=0;
    int ci=0;
    int id=0;
    char buffer[256];

    drv_sleep (500);
    downloader.strain_get_serial_number(selected_id, buffer);
    gtk_entry_set_text (GTK_ENTRY (edit_serial_number), buffer);
    serial_number_changed=false;

    drv_sleep (500);
    for (ri=0;ri<6;ri++)
    for (ci=0;ci<6;ci++)
    for (id=0;id<3;id++)
    {
        downloader.strain_get_matrix_rc(selected_id,ri,ci,id,matrix[ri][ci][id]);
        sprintf(buffer,"%x",matrix[ri][ci][id]);
        gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci][id]), buffer);
        gtk_widget_modify_base (edit_matrix[ri][ci][id],GTK_STATE_NORMAL, NULL );
    }
    drv_sleep (500);
    int count_ok=0;
    for (id=0;id<3; id++)
        for (i=0;i<36; i++)
        {
            ri=i/6;
            ci=i%6;
            if (calib_matrix[ri][ci][id]==matrix[ri][ci][id])
            {
                count_ok++;
            }
            else
            {
                printf ("Found 1 error on element %d,%d, matrix %d !!\n",ri, ci, id);
            }
        }
    if (count_ok>=36*3)
    {
        printf ("Calibration file applied with no errors\n");
        matrix_changed[0]=false;
        matrix_changed[1]=false;
        matrix_changed[2]=false;
    }
    else
    {
        printf ("Found %d errors applying the calibration file!!\n",36-count_ok);
    }
}

//*********************************************************************************
void file_import_click (GtkButton *button,    gpointer ch_p)
{
    std::string filename = "C:\\Software\\iCub\\bin\\debug\\ciao.dat";

    char* buff;

    buff = gtk_file_chooser_get_filename   (GTK_FILE_CHOOSER(picker_calib));
    if (buff==NULL)
        {
            yError("File not found!\n");
            return;
        }

    fstream filestr;
    filestr.open (buff, fstream::in);
    if (!filestr.is_open())
        {
            yError("Error opening calibration file!\n");
            return;
        }

    int i=0;
    char buffer[256];
    int id = get_matrix();
    for (i=0;i<36; i++)
    {
        int ri=i/6;
        int ci=i%6;
        filestr.getline (buffer,256);
        sscanf (buffer,"%x",&calib_matrix[ri][ci][id]);
        printf("%d %x\n", calib_matrix[ri][ci][id],calib_matrix[ri][ci][id]);
        downloader.strain_set_matrix_rc(downloader.board_list[selected].pid,ri,ci,id,calib_matrix[ri][ci][id]);
    }
    filestr.getline (buffer,256);
    int cc=0;
    sscanf (buffer,"%d",&cc);
    downloader.strain_set_matrix_gain(downloader.board_list[selected].pid,cc);
    for (i=0;i<6; i++)
    {
        filestr.getline (buffer,256);
        sscanf (buffer,"%d",&cc);
        downloader.strain_set_full_scale(downloader.board_list[selected].pid,i,cc);
    }
    filestr.close();

    something_changed=true;
    printf ("Calibration file loaded!\n");

    int ri=0;
    int ci=0;

    for (i=0; i<2; i++)
    {
        drv_sleep (1000);
        for (ri=0;ri<6;ri++)
                for (ci=0;ci<6;ci++)
                    {
                        downloader.strain_get_matrix_rc(downloader.board_list[selected].pid,ri,ci,id,matrix[ri][ci][id]);
                        sprintf(buffer,"%x",matrix[ri][ci][id]);
                        gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci][id]), buffer);
                        gtk_widget_modify_base (edit_matrix[ri][ci][id],GTK_STATE_NORMAL, NULL );
                    }
    }
    int count_ok=0;
    for (i=0;i<36; i++)
    {
        ri=i/6;
        ci=i%6;
        if (calib_matrix[ri][ci][id]==matrix[ri][ci][id])
        {
            count_ok++;
        }
        else
        {
            printf ("Found 1 error on element %d,%d !!\n",ri, ci);
        }
    }
    if (count_ok==36)
    {
        printf ("Calibration file %s applied with no errors\n", buff);
        matrix_changed[0]=false;
        matrix_changed[1]=false;
        matrix_changed[2]=false;
    }
    else
    {
        printf ("Found %d errors applying the calibration file!!\n",36-count_ok);
    }
}

//*********************************************************************************
void zero_changed (GtkButton *button,    gpointer ch_p)
{
    calibration_value = (unsigned int) (gtk_range_get_value (GTK_RANGE(slider_zero)));
}

//*********************************************************************************
void matrix_change (GtkEntry *entry,    gpointer index)
{
    int* ip = (int*)(index);
    int  i = *ip;
    int gm;
    if (i>=36*0 && i <=36*1-1) gm =0;
    if (i>=36*1 && i <=36*2-1) gm =1;
    if (i>=36*2 && i <=36*3-1) gm =2;
    matrix_changed[gm]=true;
    something_changed=true;
    printf("%d selected, calibration matrix with id %d changed\n", i, gm);
}

//*********************************************************************************
void serial_number_change (GtkEntry *entry,    gpointer index)
{
    serial_number_changed=true;
    something_changed=true;
    //printf("Calibration matrix changed\n");
}

//*********************************************************************************
void serial_number_send (GtkEntry *entry,    gpointer index)
{
    serial_number_changed=false;
    const gchar* temp2 = gtk_entry_get_text (GTK_ENTRY (edit_serial_number));
    sprintf(serial_no,"%s", temp2);
    downloader.strain_set_serial_number(downloader.board_list[selected].pid,temp2);
}
//*********************************************************************************
void matrix_send (GtkEntry *entry,    gpointer index)
{
    int ri=0;
    int ci=0;
    int tid=0;
    int id=get_matrix();

    for (ri=0; ri<6; ri++)
        for (ci=0; ci<6; ci++)
            {
                const gchar* temp2 = gtk_entry_get_text (GTK_ENTRY (edit_matrix[ri][ci][id]));
                sscanf (temp2,"%x",&matrix[ri][ci][id]);
                downloader.strain_set_matrix_rc(downloader.board_list[selected].pid,ri,ci,id,matrix[ri][ci][id]);
            }

    downloader.strain_set_matrix_gain(downloader.board_list[selected].pid,calib_const);
    printf("Calibration matrix updated\n");

    int gm = get_matrix();
    matrix_changed[gm]=false;

    for (ri=0; ri<6; ri++)
        for (ci=0; ci<6; ci++)
            for (tid=0; tid<3; tid++)
                gtk_widget_modify_base (edit_matrix[ri][ci][tid],GTK_STATE_NORMAL, NULL );
}

//*********************************************************************************
void calibrate_click (GtkButton *button,    gpointer   user_data)
{
    matrix_changed[0]=false;
    matrix_changed[1]=false;
    matrix_changed[2]=false;
    something_changed=false;
    eeprom_saved_status=false;

    //which strain board is selected)
    int i        = 0;
    int count    = 0;

    for (i=0; i<6;i++)
    {
        first_time[i]=true;
        amp_gain1[i]=0;
        amp_gain2[i]=0;
    }

    for (i=0; i<downloader.board_list_size; i++)
    {
        if (downloader.board_list[i].status==BOARD_RUNNING &&
            (downloader.board_list[i].type==icubCanProto_boardType__strain || downloader.board_list[i].type==icubCanProto_boardType__6sg) &&
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
    GtkWidget *file_import_button;
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
    picker_calib            = gtk_file_chooser_button_new ("Pick a File", GTK_FILE_CHOOSER_ACTION_OPEN);
    auto_button             = gtk_button_new_with_mnemonic ("Automatic \nOffset Adj");
    save_button             = gtk_button_new_with_label ("Save to eeprom");
    file_load_button        = gtk_button_new_with_mnemonic ("Load Calibration File");
    file_import_button      = gtk_button_new_with_mnemonic ("Import Calib Matrix");
    file_save_button        = gtk_button_new_with_mnemonic ("Save Calibration File");
    matrix_reset_button     = gtk_button_new_with_mnemonic ("Reset Calibration");
    set_calib_bias_button   = gtk_button_new_with_mnemonic ("Set Calibration Bias"); ;
    reset_calib_bias_button = gtk_button_new_with_mnemonic ("Reset Calibration Bias"); ;
    set_curr_bias_button    = gtk_button_new_with_mnemonic ("Set Current Bias"); ;
    reset_curr_bias_button  = gtk_button_new_with_mnemonic ("Reset Current Bias"); ;

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
    label_gain[0] = gtk_label_new_with_mnemonic ("Offset 0:");
    label_gain[1] = gtk_label_new_with_mnemonic ("Offset 1:");
    label_gain[2] = gtk_label_new_with_mnemonic ("Offset 2:");
    label_gain[3] = gtk_label_new_with_mnemonic ("Offset 3:");
    label_gain[4] = gtk_label_new_with_mnemonic ("Offset 4:");
    label_gain[5] = gtk_label_new_with_mnemonic ("Offset 5:");

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
    newton_measure[0] = gtk_label_new_with_mnemonic ("0");
    newton_measure[1] = gtk_label_new_with_mnemonic ("0");
    newton_measure[2] = gtk_label_new_with_mnemonic ("0");
    newton_measure[3] = gtk_label_new_with_mnemonic ("0");
    newton_measure[4] = gtk_label_new_with_mnemonic ("0");
    newton_measure[5] = gtk_label_new_with_mnemonic ("0");
    calib_bias_label[0] = gtk_label_new_with_mnemonic ("0");
    calib_bias_label[1] = gtk_label_new_with_mnemonic ("0");
    calib_bias_label[2] = gtk_label_new_with_mnemonic ("0");
    calib_bias_label[3] = gtk_label_new_with_mnemonic ("0");
    calib_bias_label[4] = gtk_label_new_with_mnemonic ("0");
    calib_bias_label[5] = gtk_label_new_with_mnemonic ("0");
    curr_bias_label[0] = gtk_label_new_with_mnemonic ("0");
    curr_bias_label[1] = gtk_label_new_with_mnemonic ("0");
    curr_bias_label[2] = gtk_label_new_with_mnemonic ("0");
    curr_bias_label[3] = gtk_label_new_with_mnemonic ("0");
    curr_bias_label[4] = gtk_label_new_with_mnemonic ("0");
    curr_bias_label[5] = gtk_label_new_with_mnemonic ("0");
    full_scale_label[0] = gtk_label_new_with_mnemonic ("0");
    full_scale_label[1] = gtk_label_new_with_mnemonic ("0");
    full_scale_label[2] = gtk_label_new_with_mnemonic ("0");
    full_scale_label[3] = gtk_label_new_with_mnemonic ("0");
    full_scale_label[4] = gtk_label_new_with_mnemonic ("0");
    full_scale_label[5] = gtk_label_new_with_mnemonic ("0");
    label_matrix_gain = gtk_label_new_with_mnemonic ("matrix gain:");
    edit_matrix_gain = gtk_label_new_with_mnemonic ("null");

    label_use_calib = gtk_label_new_with_mnemonic  ("use calib matrix:");
    GtkAdjustment *adjustment ;
    adjustment = (GtkAdjustment*)gtk_adjustment_new (0.0,0.0,3.0,1.0,5.0,0.0); 
    spin_use_calib  = gtk_spin_button_new(adjustment,1.0,0);

    slider_gain[0] = gtk_hscale_new_with_range   (0,0x3FF,1);
    slider_gain[1] = gtk_hscale_new_with_range   (0,0x3FF,1);
    slider_gain[2] = gtk_hscale_new_with_range   (0,0x3FF,1);
    slider_gain[3] = gtk_hscale_new_with_range   (0,0x3FF,1);
    slider_gain[4] = gtk_hscale_new_with_range   (0,0x3FF,1);
    slider_gain[5] = gtk_hscale_new_with_range   (0,0x3FF,1);
    slider_zero       = gtk_hscale_new_with_range   (0,65535,1);

    for (int ri=0;ri<6;ri++)
    for (int ci=0;ci<6;ci++)
    for (int id=0;id<3;id++)
    {
        edit_matrix[ri][ci][id] = gtk_entry_new ();
    }

    edit_serial_number = gtk_entry_new ();

    gtk_container_add  (GTK_CONTAINER(GTK_BOX (GTK_DIALOG (calib_window)->vbox)),fixed);

    int r[8]={0+10,60+10,60*2+10,60*3+10,60*4+10,60*5+10,60*6+10,60*7+10};
    int c[12]={0+10,50+10,150+10,230+10,350+10,400+10,450+10,500+10,550+10,650+10,700+10,750+10};

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

    gtk_fixed_put(GTK_FIXED(fixed),set_calib_bias_button,c[11],r[2]-10);
    gtk_fixed_put(GTK_FIXED(fixed),reset_calib_bias_button,c[11],r[3]-10);
    gtk_fixed_put(GTK_FIXED(fixed),set_curr_bias_button,c[11],r[4]-10);
    gtk_fixed_put(GTK_FIXED(fixed),reset_curr_bias_button,c[11],r[5]-10);

    for (int ri=0;ri<6;ri++)
    for (int ci=0;ci<6;ci++)
    for (int id=0;id<3;id++)
    {
        gtk_fixed_put(GTK_FIXED(fixed),edit_matrix[ci][ri][id],c[5]+ri*42+(id*6*52)-50,r[5]+40+ci*32);
        gtk_widget_set_size_request(edit_matrix[ri][ci][id],40,20);
    }

    gtk_fixed_put(GTK_FIXED(fixed),edit_serial_number,c[1]-20,r[5]+150);
    gtk_widget_set_size_request(edit_serial_number,100,20);

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

    gtk_fixed_put(GTK_FIXED(fixed),newton_measure[0],c[8],r[0]);
    gtk_fixed_put(GTK_FIXED(fixed),newton_measure[1],c[8],r[1]);
    gtk_fixed_put(GTK_FIXED(fixed),newton_measure[2],c[8],r[2]);
    gtk_fixed_put(GTK_FIXED(fixed),newton_measure[3],c[8],r[3]);
    gtk_fixed_put(GTK_FIXED(fixed),newton_measure[4],c[8],r[4]);
    gtk_fixed_put(GTK_FIXED(fixed),newton_measure[5],c[8],r[5]);

    gtk_fixed_put(GTK_FIXED(fixed),calib_bias_label[0],c[9],r[0]);
    gtk_fixed_put(GTK_FIXED(fixed),calib_bias_label[1],c[9],r[1]);
    gtk_fixed_put(GTK_FIXED(fixed),calib_bias_label[2],c[9],r[2]);
    gtk_fixed_put(GTK_FIXED(fixed),calib_bias_label[3],c[9],r[3]);
    gtk_fixed_put(GTK_FIXED(fixed),calib_bias_label[4],c[9],r[4]);
    gtk_fixed_put(GTK_FIXED(fixed),calib_bias_label[5],c[9],r[5]);

    gtk_fixed_put(GTK_FIXED(fixed),curr_bias_label[0],c[10],r[0]);
    gtk_fixed_put(GTK_FIXED(fixed),curr_bias_label[1],c[10],r[1]);
    gtk_fixed_put(GTK_FIXED(fixed),curr_bias_label[2],c[10],r[2]);
    gtk_fixed_put(GTK_FIXED(fixed),curr_bias_label[3],c[10],r[3]);
    gtk_fixed_put(GTK_FIXED(fixed),curr_bias_label[4],c[10],r[4]);
    gtk_fixed_put(GTK_FIXED(fixed),curr_bias_label[5],c[10],r[5]);

    gtk_fixed_put(GTK_FIXED(fixed),full_scale_label[0],c[1]-20,r[5]+180);
    gtk_fixed_put(GTK_FIXED(fixed),full_scale_label[1],c[1]-20,r[5]+195);
    gtk_fixed_put(GTK_FIXED(fixed),full_scale_label[2],c[1]-20,r[5]+210);
    gtk_fixed_put(GTK_FIXED(fixed),full_scale_label[3],c[1]-20,r[5]+225);
    gtk_fixed_put(GTK_FIXED(fixed),full_scale_label[4],c[1]-20,r[5]+240);
    gtk_fixed_put(GTK_FIXED(fixed),full_scale_label[5],c[1]-20,r[5]+255);

    gtk_fixed_put(GTK_FIXED(fixed),label_matrix_gain,c[5],r[5]+40+6*32);
    gtk_fixed_put(GTK_FIXED(fixed),edit_matrix_gain,c[5]+100,r[5]+40+6*32);
    gtk_fixed_put(GTK_FIXED(fixed),matrix_reset_button,c[5]+140,r[5]+40+6*32);

    gtk_widget_set_size_request(auto_button,100,40);
    gtk_widget_set_size_request(save_button,140,40);
    gtk_widget_set_size_request(file_load_button,140,40);
    gtk_widget_set_size_request(file_import_button,140,40);
    gtk_widget_set_size_request(file_save_button,140,40);
    gtk_widget_set_size_request(picker_calib,140,40);
    gtk_widget_set_size_request(matrix_reset_button,140,30);
    gtk_widget_set_size_request(set_calib_bias_button,140,40);
    gtk_widget_set_size_request(reset_calib_bias_button,140,40);
    gtk_widget_set_size_request(set_curr_bias_button,140,40);
    gtk_widget_set_size_request(reset_curr_bias_button,140,40);

    g_signal_connect (file_import_button, "clicked", G_CALLBACK (file_import_click),NULL);
    g_signal_connect (file_load_button, "clicked", G_CALLBACK (file_load_click),NULL);
    g_signal_connect (file_save_button, "clicked", G_CALLBACK (file_save_click),NULL);
    g_signal_connect (matrix_reset_button, "clicked", G_CALLBACK (reset_matrix_click),NULL);
    g_signal_connect (auto_button, "clicked", G_CALLBACK (auto_click),NULL);
    g_signal_connect (save_button, "clicked", G_CALLBACK (save_click),NULL);
    g_signal_connect (calib_window, "response", G_CALLBACK (close_window),NULL);
    g_signal_connect (set_calib_bias_button, "clicked", G_CALLBACK (set_calib_bias_click),NULL);
    g_signal_connect (reset_calib_bias_button, "clicked", G_CALLBACK (reset_calib_bias_click),NULL);
    g_signal_connect (set_curr_bias_button, "clicked", G_CALLBACK (set_curr_bias_click),NULL);
    g_signal_connect (reset_curr_bias_button, "clicked", G_CALLBACK (reset_curr_bias_click),NULL);


    g_signal_connect (slider_gain[0], "value-changed", G_CALLBACK (slider_changed),&ch[0]);
    g_signal_connect (slider_gain[1], "value-changed", G_CALLBACK (slider_changed),&ch[1]);
    g_signal_connect (slider_gain[2], "value-changed", G_CALLBACK (slider_changed),&ch[2]);
    g_signal_connect (slider_gain[3], "value-changed", G_CALLBACK (slider_changed),&ch[3]);
    g_signal_connect (slider_gain[4], "value-changed", G_CALLBACK (slider_changed),&ch[4]);
    g_signal_connect (slider_gain[5], "value-changed", G_CALLBACK (slider_changed),&ch[5]);
    g_signal_connect (slider_zero, "value-changed", G_CALLBACK (zero_changed),NULL);

    gtk_fixed_put(GTK_FIXED(fixed),label_use_calib,c[5],r[5]+60+6*32);
    gtk_fixed_put(GTK_FIXED(fixed),spin_use_calib,c[5],r[5]+60+6*32+30);
    gtk_fixed_put(GTK_FIXED(fixed),auto_button,c[1]-20,r[5]+40);
    gtk_fixed_put(GTK_FIXED(fixed),slider_zero,c[1]-20,r[5]+100);
    gtk_fixed_put(GTK_FIXED(fixed),save_button,c[2]+10,r[5]+40);
    gtk_fixed_put(GTK_FIXED(fixed),file_load_button,c[2]+10,r[5]+90);
    gtk_fixed_put(GTK_FIXED(fixed),file_save_button,c[2]+10,r[5]+140);
    gtk_fixed_put(GTK_FIXED(fixed),file_import_button,c[2]+10,r[5]+190);

    gtk_fixed_put(GTK_FIXED(fixed),picker_calib,c[2]+10,r[5]+240);

    gtk_range_set_value (GTK_RANGE(slider_zero),calibration_value);

    downloader.strain_get_serial_number(downloader.board_list[selected].pid,serial_no);

    char tempbuf [20];
    for (int ri=0;ri<6;ri++)
    for (int ci=0;ci<6;ci++)
    for (int id=0;id<3;id++)
    {
        downloader.strain_get_matrix_rc(downloader.board_list[selected].pid,ri,ci,id,matrix[ri][ci][id]);
        sprintf(tempbuf,"%x",matrix[ri][ci][id]);
        gtk_entry_set_text (GTK_ENTRY (edit_matrix[ri][ci][id]), tempbuf);
        gtk_widget_modify_base (edit_matrix[ri][ci][id],GTK_STATE_NORMAL, NULL );
    }

    gtk_entry_set_text (GTK_ENTRY (edit_serial_number), serial_no);
    downloader.strain_start_sampling(downloader.board_list[selected].pid);

    STOP_TIMER
    START_TIMER
    timer_func (NULL);

    int index[6*6*3];
    for (int ri=0;ri<6;ri++)
    for (int ci=0;ci<6;ci++)
    for (int id=0;id<3;id++)
    {
        index[id*36+ci*6+ri]=id*36+ci*6+ri;
        g_signal_connect(edit_matrix[ri][ci][id], "changed", G_CALLBACK (matrix_change),&index[id*36+ci*6+ri]);
        g_signal_connect(edit_matrix[ri][ci][id], "activate", G_CALLBACK (matrix_send),&index[id*36+ci*6+ri]);
    }

    g_signal_connect(edit_serial_number, "changed", G_CALLBACK (serial_number_change),NULL);
    g_signal_connect(edit_serial_number, "activate", G_CALLBACK (serial_number_send),NULL);

    gtk_widget_show_all (fixed);
    //gtk_window_set_resizable(GTK_WINDOW(calib_window),false);
    gtk_dialog_run (GTK_DIALOG (calib_window));
}
