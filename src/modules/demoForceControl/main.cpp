/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <string>
#include <gtk/gtk.h>

#include "robot_interfaces.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

robot_interfaces *robot;

GtkWidget *radiobutton_mode_pos[5];
GtkWidget *radiobutton_mode_trq[5];
GtkWidget *radiobutton_mode_imp_s[5];
GtkWidget *radiobutton_mode_imp_m[5];
GtkWidget *radiobutton_mode_imp_h[5];
GtkWidget *label_info[5];

void destroy (void)
{
  gtk_main_quit ();
}

#define POS  0
#define TRQ  1
#define IMPS 2
#define IMPM 3
#define IMPH 4

struct radio_data
{
    int mode;
    int id;
};

void put_everything_in_position()
{
    for (int i=0; i<5; i++)
    {
        int jmax=0;
        switch (i)
        { 
            case LEFT_ARM:
            case RIGHT_ARM:
                jmax=5;
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                jmax=6;
            break;
            case TORSO:
                jmax=3;
            break;
        }
        for (int j=0; j<jmax; j++)
        {
            if (robot->icmd[i]) robot->icmd[i]->setPositionMode(j);
            if (robot->iint[i]) robot->iint[i]->setInteractionMode(j,VOCAB_IM_STIFF);
        }
    }
}

void update_radio_buttons()
{
    for (int i=0; i<5; i++)
    {
        int jmax=0;
        switch (i)
        { 
            case LEFT_ARM:
            case RIGHT_ARM:
                jmax=5;
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                jmax=6;
            break;
            case TORSO:
                jmax=3;
            break;
        }
        for (int j=0; j<jmax; j++)
        {
            int c_mode=0;
            yarp::dev::InteractionModeEnum i_mode;
            if (robot->icmd[i]) robot->icmd[i]->getControlMode(j,&c_mode);
            if (robot->iint[i]) robot->iint[i]->getInteractionMode(j,&i_mode);
            switch (c_mode)
            {
                case VOCAB_CM_POSITION:
                    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (radiobutton_mode_pos[i]),true);
                break;

                case VOCAB_CM_TORQUE:
                    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (radiobutton_mode_trq[i]),true);
                break;

                case VOCAB_CM_IMPEDANCE_POS:
                    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (radiobutton_mode_imp_s[i]),true);
                break;

                default:
                case VOCAB_CM_IDLE:
                case VOCAB_CM_VELOCITY:
                case VOCAB_CM_IMPEDANCE_VEL:
                break;
            }
        }
    }
}

void update_labels()
{
    char buff [255];
    for (int i=0; i<5; i++)
    {
        int jmax=0;
        string txt="<small><span font_desc=\"Courier 8\">";
        switch (i)
        { 
            case LEFT_ARM:
            case RIGHT_ARM:
                jmax=5;
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                jmax=6;
            break;
            case TORSO:
                jmax=3;
            break;
        }
        for (int j=0; j<jmax; j++)
        {    
            double stiff=0;
            double damp=0;
            sprintf (buff, "\n \nJ%d:\n", j); txt+= string(buff);
            if (robot->iimp[i]) robot->iimp[i]->getImpedance(j,&stiff,&damp);
            sprintf (buff, "stiff: %3.3f Nm/deg\n", stiff); txt+= string(buff);
            sprintf (buff, "damp:  %3.3f Nm/(deg/s)\n", damp); txt+= string(buff);
        }
        txt+="</span></small>";
        //gtk_label_set_text(GTK_LABEL(label_info[i]),txt.c_str());
        gtk_label_set_markup (GTK_LABEL (label_info[i]), txt.c_str());
    }
}

void radio_click_p( GtkWidget *widget, void* r)
{
    if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget)))
    {
        radio_data* rd = (radio_data*)(r);
        //printf("%d %d\n",rd->id,rd->mode);
        if (!robot->icmd[rd->id]) return;
        switch (rd->id)
        {
            case LEFT_ARM:
            case RIGHT_ARM:
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_STIFF);
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->icmd[rd->id]->setPositionMode(5);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(5, VOCAB_IM_STIFF);
            break;
            case TORSO:
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_STIFF);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_STIFF);
            break;
        }
        update_labels();
    }
}
void radio_click_t( GtkWidget *widget, void* r)
{
    if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget)))
    {
        radio_data* rd = (radio_data*)(r);
        //printf("%d %d\n",rd->id,rd->mode);
        if (!robot->icmd[rd->id]) return;
        switch (rd->id)
        {
            case LEFT_ARM:
            case RIGHT_ARM:
                robot->iimp[rd->id]->setImpedance(0,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(3,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(4,0.0,0.0);
                robot->icmd[rd->id]->setTorqueMode(0);
                robot->icmd[rd->id]->setTorqueMode(1);
                robot->icmd[rd->id]->setTorqueMode(2);
                robot->icmd[rd->id]->setTorqueMode(3);
                robot->icmd[rd->id]->setTorqueMode(4);
           break;
           case LEFT_LEG:
           case RIGHT_LEG:
                robot->iimp[rd->id]->setImpedance(0,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(3,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(4,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(5,0.0,0.0);
                robot->icmd[rd->id]->setTorqueMode(0);
                robot->icmd[rd->id]->setTorqueMode(1);
                robot->icmd[rd->id]->setTorqueMode(2);
                robot->icmd[rd->id]->setTorqueMode(3);
                robot->icmd[rd->id]->setTorqueMode(4);    
                robot->icmd[rd->id]->setTorqueMode(5);
            break;
            case TORSO:
                robot->iimp[rd->id]->setImpedance(0,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.0,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.1,0.0);
                robot->icmd[rd->id]->setTorqueMode(0);
                robot->icmd[rd->id]->setTorqueMode(1);
                //robot->icmd[rd->id]->setTorqueMode(2);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->iint[rd->id]->setInteractionMode(2,VOCAB_IM_COMPLIANT);
            break;
        }
        update_labels();
    }
}
void radio_click_is( GtkWidget *widget, void* r)
{
    if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget)))
    {
        radio_data* rd = (radio_data*)(r);
        //printf("%d %d\n",rd->id,rd->mode);
        if (!robot->icmd[rd->id]) return;
        switch (rd->id)
        {
            case LEFT_ARM:
            case RIGHT_ARM:
                robot->iimp[rd->id]->setImpedance(0,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(3,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(4,0.1,0.0);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                robot->iimp[rd->id]->setImpedance(0,0.3,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.3,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(3,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(4,0.2,0.0);
                robot->iimp[rd->id]->setImpedance(5,0.2,0.0);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->icmd[rd->id]->setPositionMode(5);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(5, VOCAB_IM_COMPLIANT);
            break;
            case TORSO:
                robot->iimp[rd->id]->setImpedance(0,0.1,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.1,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.1,0.0);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
            break;
        }
        update_labels();
    }
}
void radio_click_im( GtkWidget *widget, void* r)
{
    if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget)))
    {
        radio_data* rd = (radio_data*)(r);
        //printf("%d %d\n",rd->id,rd->mode);
        if (!robot->icmd[rd->id]) return;
        switch (rd->id)
        {
            case LEFT_ARM:
            case RIGHT_ARM:
                robot->iimp[rd->id]->setImpedance(0,0.4,0.03);
                robot->iimp[rd->id]->setImpedance(1,0.4,0.03);
                robot->iimp[rd->id]->setImpedance(2,0.4,0.03);
                robot->iimp[rd->id]->setImpedance(3,0.2,0.01);
                robot->iimp[rd->id]->setImpedance(4,0.2,0.00);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                robot->iimp[rd->id]->setImpedance(0,0.6,0.01);
                robot->iimp[rd->id]->setImpedance(1,0.6,0.01);
                robot->iimp[rd->id]->setImpedance(2,0.4,0.01);
                robot->iimp[rd->id]->setImpedance(3,0.4,0.01);
                robot->iimp[rd->id]->setImpedance(4,0.4,0.01);
                robot->iimp[rd->id]->setImpedance(5,0.4,0.01);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->icmd[rd->id]->setPositionMode(5);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(5, VOCAB_IM_COMPLIANT);
            break;
            case TORSO:
                robot->iimp[rd->id]->setImpedance(0,0.3,0.0);
                robot->iimp[rd->id]->setImpedance(1,0.3,0.0);
                robot->iimp[rd->id]->setImpedance(2,0.3,0.0);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
            break;
        }
        update_labels();
    }
}
void radio_click_ih( GtkWidget *widget, void* r)
{
    if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget)))
    {
        radio_data* rd = (radio_data*)(r);
        //printf("%d %d\n",rd->id,rd->mode);
        if (!robot->icmd[rd->id]) return;
        switch (rd->id)
        {
            case LEFT_ARM:
            case RIGHT_ARM:
                robot->iimp[rd->id]->setImpedance(0,0.6,0.06);
                robot->iimp[rd->id]->setImpedance(1,0.6,0.06);
                robot->iimp[rd->id]->setImpedance(2,0.6,0.06);
                robot->iimp[rd->id]->setImpedance(3,0.3,0.02);
                robot->iimp[rd->id]->setImpedance(4,0.2,0.00);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
            break;
            case LEFT_LEG:
            case RIGHT_LEG:
                robot->iimp[rd->id]->setImpedance(0,1.0,0.02);
                robot->iimp[rd->id]->setImpedance(1,1.0,0.02);
                robot->iimp[rd->id]->setImpedance(2,0.7,0.02);
                robot->iimp[rd->id]->setImpedance(3,0.6,0.02);
                robot->iimp[rd->id]->setImpedance(4,0.6,0.02);
                robot->iimp[rd->id]->setImpedance(5,0.6,0.02);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->icmd[rd->id]->setPositionMode(3);
                robot->icmd[rd->id]->setPositionMode(4);
                robot->icmd[rd->id]->setPositionMode(5);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(5, VOCAB_IM_COMPLIANT);
            break;
            case TORSO:
                robot->iimp[rd->id]->setImpedance(0,0.7,0.015);
                robot->iimp[rd->id]->setImpedance(1,0.7,0.015);
                robot->iimp[rd->id]->setImpedance(2,0.7,0.015);
                robot->icmd[rd->id]->setPositionMode(0);
                robot->icmd[rd->id]->setPositionMode(1);
                robot->icmd[rd->id]->setPositionMode(2);
                robot->iint[rd->id]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
                robot->iint[rd->id]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
            break;
        }
        update_labels();
    }
}

int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);
    
    robot = new robot_interfaces();
    std::string robot_name = "icub";

    if (rf.check("robot"))
    {
        robot_name=rf.find("robot").asString();
    }

    robot->init(robot_name);
    
    GtkWidget *window;
    GtkWidget *fixed;
    GtkWidget *part_frame [5];
    GtkWidget *fixed_inside_part_frame [5];

    gtk_init (&argc, &argv);

    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_signal_connect (GTK_OBJECT (window), "destroy",GTK_SIGNAL_FUNC (destroy), NULL);
    gtk_container_border_width (GTK_CONTAINER (window), 10);

    fixed  = gtk_fixed_new ();
    gtk_widget_set_size_request     (fixed, 800, 600);
    gtk_container_add (GTK_CONTAINER (window), fixed);

    part_frame[LEFT_ARM] = gtk_frame_new ("left arm");
    part_frame[RIGHT_ARM] = gtk_frame_new ("right arm");
    part_frame[LEFT_LEG] =  gtk_frame_new ("left leg");
    part_frame[RIGHT_LEG] = gtk_frame_new ("right leg");
    part_frame[TORSO] = gtk_frame_new ("torso (EXPERIMENTAL)");

    gtk_widget_set_size_request    (part_frame[LEFT_ARM],   150, 550    );
    gtk_widget_set_size_request    (part_frame[RIGHT_ARM],  150, 550    );
    gtk_widget_set_size_request    (part_frame[LEFT_LEG],   150, 550    );
    gtk_widget_set_size_request    (part_frame[RIGHT_LEG],  150, 550    );
    gtk_widget_set_size_request    (part_frame[TORSO],      150, 550    );

    gtk_fixed_put    (GTK_FIXED(fixed), part_frame[LEFT_ARM],   30+150*0, 30    );
    gtk_fixed_put    (GTK_FIXED(fixed), part_frame[RIGHT_ARM],  30+150*1, 30    );
    gtk_fixed_put    (GTK_FIXED(fixed), part_frame[LEFT_LEG],   30+150*2, 30    );
    gtk_fixed_put    (GTK_FIXED(fixed), part_frame[RIGHT_LEG],  30+150*3, 30    );
    gtk_fixed_put    (GTK_FIXED(fixed), part_frame[TORSO],      30+150*4, 30    );

    radio_data r[5][5];
    for (int i=0; i<5; i++)
        for (int j=0; j<5; j++)
        {
            r[i][j].id=j;
            r[i][j].mode=i;
        }

    for (int i=0; i<5; i++)
    {
        //create radio buttons
        fixed_inside_part_frame[i]  = gtk_fixed_new ();
        label_info[i] = gtk_label_new("");
        gtk_container_add     (GTK_CONTAINER(part_frame[i]), fixed_inside_part_frame[i]    );
        radiobutton_mode_pos[i]    = gtk_radio_button_new_with_label  (NULL, "position");
        radiobutton_mode_trq[i]    = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radiobutton_mode_pos[i]), "zero torque");
        radiobutton_mode_imp_s[i]  = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radiobutton_mode_pos[i]), "soft spring");
        radiobutton_mode_imp_m[i]  = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radiobutton_mode_pos[i]), "medium spring");
        radiobutton_mode_imp_h[i]  = gtk_radio_button_new_with_label_from_widget  (GTK_RADIO_BUTTON (radiobutton_mode_pos[i]), "hard spring");        
        gtk_fixed_put    (GTK_FIXED(fixed_inside_part_frame[i] ), radiobutton_mode_pos[i],     20, 30     );
        gtk_fixed_put    (GTK_FIXED(fixed_inside_part_frame[i] ), radiobutton_mode_trq[i],     20, 60     );
        gtk_fixed_put    (GTK_FIXED(fixed_inside_part_frame[i] ), radiobutton_mode_imp_s[i],   20, 90     );
        gtk_fixed_put    (GTK_FIXED(fixed_inside_part_frame[i] ), radiobutton_mode_imp_m[i],   20, 120    );
        gtk_fixed_put    (GTK_FIXED(fixed_inside_part_frame[i] ), radiobutton_mode_imp_h[i],   20, 150    );
        gtk_fixed_put    (GTK_FIXED(fixed_inside_part_frame[i] ), label_info[i],               20, 200    );
        //turns off widgets of missing parts
        if (robot->dd[i]==0)
        {
            gtk_widget_set_sensitive(part_frame[i],false);
            gtk_widget_set_sensitive(fixed_inside_part_frame[i], false);
            gtk_widget_set_sensitive(radiobutton_mode_pos[i],    false);
            gtk_widget_set_sensitive(radiobutton_mode_trq[i],    false);
            gtk_widget_set_sensitive(radiobutton_mode_imp_s[i],  false);
            gtk_widget_set_sensitive(radiobutton_mode_imp_m[i],  false);
            gtk_widget_set_sensitive(radiobutton_mode_imp_h[i],  false);
        }
        //connects callbacks
        g_signal_connect (radiobutton_mode_pos[i],    "clicked",G_CALLBACK (radio_click_p),  &r[0][i]);
        g_signal_connect (radiobutton_mode_trq[i],    "clicked",G_CALLBACK (radio_click_t),  &r[1][i]);
        g_signal_connect (radiobutton_mode_imp_s[i],  "clicked",G_CALLBACK (radio_click_is), &r[2][i]);
        g_signal_connect (radiobutton_mode_imp_m[i],  "clicked",G_CALLBACK (radio_click_im), &r[3][i]);
        g_signal_connect (radiobutton_mode_imp_h[i],  "clicked",G_CALLBACK (radio_click_ih), &r[4][i]);
    }

    //update_radio_buttons();
    //gtk_widget_set_sensitive(part_frame[TORSO],false);

    update_labels();
    put_everything_in_position();
        
    gtk_widget_show_all (window);

    gtk_main ();          

    put_everything_in_position();
    return 0;
}


