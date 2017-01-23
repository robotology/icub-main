/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <ace/ACE.h>

#define ETHLOADER_USE_MAINTAINER

#include <gtk/gtk.h>
#include <gtk/gtkmain.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#if !defined(ETHLOADER_USE_MAINTAINER)
#include "EthUpdater.h"
#else
#include "EthMaintainer.h"
#endif


#include <yarp/os/Property.h>
#include <yarp/os/Log.h>
#include <yarp/os/Value.h>

#if !defined(ETHLOADER_USE_MAINTAINER)
EthUpdater gUpdater;
#else
EthMaintainer gMNT;
#endif
GtkWidget *window      = NULL;
GtkWidget *main_hbox   = NULL;

GtkWidget *sw          = NULL;
GtkWidget *treeview    = NULL;

GtkWidget *right_vbox      = NULL; 
GtkWidget *info_scroll     = NULL;
GtkWidget *info_text       = NULL;
GtkWidget *info_cls_button = NULL;

GtkWidget *buttons_box = NULL;

GtkWidget *sel_all_button  = NULL;
GtkWidget *des_all_button  = NULL;
        
GtkWidget *discover_button = NULL;
#if !defined(ETHLOADER_USE_MAINTAINER)
#else
GtkWidget *maintenanceON_button = NULL;
GtkWidget *applicationON_button = NULL;
#endif
GtkWidget *upload_button   = NULL;

GtkWidget *upload_updater_button = NULL;
GtkWidget *upload_loader_button  = NULL;

GtkWidget *boot_upd_button = NULL;
GtkWidget *boot_app_button = NULL;
GtkWidget *jump_upd_button = NULL;
        
GtkWidget *reset_button    = NULL;
GtkWidget *procs_button    = NULL;
GtkWidget *eprom_button    = NULL;         
GtkWidget *blink_button    = NULL;
GtkWidget *einfo_button    = NULL;
GtkWidget *progress_bar    = NULL;

GtkWidget *message         = NULL;
GtkWidget *message_hbox    = NULL;

enum
{
    COLUMN_SELECTED,
    COLUMN_IP_ADDR,
    COLUMN_MAC,
    COLUMN_BOARD_TYPE,
    COLUMN_RUNNING_PROCESS,
    COLUMN_VERSION,
    COLUMN_DATE,
    COLUMN_BUILT,
    COLUMN_INFO,
    NUM_COLUMNS
};

void activate_buttons()
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    int size=gUpdater.getBoardList().size();
    int nsel=gUpdater.getBoardList().numSelected();
#else
    int size=gMNT.boards_get().size();
    int nsel=gMNT.boards_get().numberof(EthMaintainer::ipv4OfAllSelected);
#endif

    bool bHaveSelected=nsel!=0;
    bool bAllSelected=size==nsel;
    
    gtk_widget_set_sensitive(sel_all_button,!bAllSelected);
    gtk_widget_set_sensitive(des_all_button,bHaveSelected);
        
    gtk_widget_set_sensitive(discover_button,true);
#if !defined(ETHLOADER_USE_MAINTAINER)
#else
    gtk_widget_set_sensitive(maintenanceON_button,bHaveSelected);
    gtk_widget_set_sensitive(applicationON_button,bHaveSelected);
#endif
    gtk_widget_set_sensitive(procs_button,bHaveSelected);

//    gtk_widget_set_sensitive(upload_button,true);
    gtk_widget_set_sensitive(upload_button,nsel>=1);
    gtk_widget_set_sensitive(upload_loader_button, nsel==1);
    gtk_widget_set_sensitive(upload_updater_button,nsel==1);

    gtk_widget_set_sensitive(boot_upd_button,bHaveSelected);
    gtk_widget_set_sensitive(boot_app_button,bHaveSelected);
    gtk_widget_set_sensitive(jump_upd_button,nsel==1);
        
    gtk_widget_set_sensitive(reset_button,bHaveSelected);
    gtk_widget_set_sensitive(eprom_button,nsel==1);
    gtk_widget_set_sensitive(blink_button,nsel==1);
    gtk_widget_set_sensitive(einfo_button,nsel==1);
}

static GtkTreeModel* refresh_board_list_model()
{
    GtkListStore *store;
    GtkTreeIter iter;

    // create list store 
    store=gtk_list_store_new(NUM_COLUMNS,
        G_TYPE_BOOLEAN,
        G_TYPE_STRING,
        G_TYPE_STRING,
        G_TYPE_STRING,
        G_TYPE_STRING,
        G_TYPE_STRING,
        G_TYPE_STRING,
        G_TYPE_STRING,
        G_TYPE_STRING
		);

    char board_ipaddr[16];
    char board_mac[32];

    char board_version[24];
    char board_date[24];
    char board_built[24];
    char board_type[24];
    char running_process[24];
    char board_info[32];

#if !defined(ETHLOADER_USE_MAINTAINER)

    // add data to the list store
    for (int i=0; i<gUpdater.getBoardList().size(); ++i)
    {
        ACE_UINT32 ip=gUpdater.getBoardList()[i].mAddress;
        sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
        
        ACE_UINT64 mac=gUpdater.getBoardList()[i].mMac;
        //ACE_UINT32 macL=(ACE_UINT32)(mac&0xFFFFFFFF);
        //ACE_UINT32 macH=(ACE_UINT32)((mac>>32)&0xFFFF);

        //sprintf(board_mac,"%04X%08X",macH,macL);

        snprintf(board_mac, sizeof(board_mac), "%02X-%02X-%02X-%02X-%02X-%02X",
                            (uint8_t)(mac >> 40) & 0xff,
                            (uint8_t)(mac >> 32) & 0xff,
                            (uint8_t)(mac >> 24) & 0xff,
                            (uint8_t)(mac >> 16) & 0xff,
                            (uint8_t)(mac >> 8 ) & 0xff,
                            (uint8_t)(mac      ) & 0xff
                );

        snprintf(board_version, sizeof(board_version), "%d.%d", gUpdater.getBoardList()[i].mVersionMajor, gUpdater.getBoardList()[i].mVersionMinor);
        snprintf(board_type, sizeof(board_type), "%s", gUpdater.getBoardList()[i].mBoardType.c_str());
        snprintf(running_process, sizeof(running_process), "%s", gUpdater.getBoardList()[i].mRunningProcess.c_str());
        snprintf(board_info, sizeof(board_info), "%s", gUpdater.getBoardList()[i].mInfo32.c_str());
        snprintf(board_date, sizeof(board_date), "%s", gUpdater.getBoardList()[i].mReleasedOn.c_str());
        snprintf(board_built, sizeof(board_date), "%s", gUpdater.getBoardList()[i].mBuiltOn.c_str());

        gtk_list_store_append(store,&iter);

        gtk_list_store_set(store,&iter,
            COLUMN_SELECTED,gUpdater.getBoardList()[i].mSelected,
            COLUMN_IP_ADDR,board_ipaddr,
            COLUMN_MAC,board_mac,
            COLUMN_BOARD_TYPE,board_type,
            COLUMN_RUNNING_PROCESS, running_process,
            COLUMN_VERSION, board_version,
            COLUMN_DATE, board_date,
            COLUMN_BUILT, board_built,
            COLUMN_INFO, board_info,
            -1);
    }

#else
    // add data to the list store
    EthBoardList boards = gMNT.boards_get();
    for (int i=0; i<boards.size(); ++i)
    {
        EthBoard board = boards[i];

        snprintf(board_ipaddr, sizeof(board_ipaddr), "%s", board.getIPV4string().c_str());

        ACE_UINT64 mac = board.getInfo().macaddress;



        snprintf(board_mac, sizeof(board_mac), "%02X-%02X-%02X-%02X-%02X-%02X",
                            (uint8_t)(mac >> 40) & 0xff,
                            (uint8_t)(mac >> 32) & 0xff,
                            (uint8_t)(mac >> 24) & 0xff,
                            (uint8_t)(mac >> 16) & 0xff,
                            (uint8_t)(mac >> 8 ) & 0xff,
                            (uint8_t)(mac      ) & 0xff
                );

        snprintf(board_version, sizeof(board_version), "%s", board.getVersionfRunning().c_str());
        snprintf(board_type, sizeof(board_type), "%s", eoboards_type2string2(eoboards_ethtype2type(board.getInfo().boardtype), eobool_true));
        snprintf(running_process, sizeof(running_process), "%s", eouprot_process2string((eOuprot_process_t)board.getInfo().processes.runningnow));
        snprintf(board_info, sizeof(board_info), "%s", board.getInfoOnEEPROM().c_str());
        snprintf(board_date, sizeof(board_date), "%s", board.getDatefRunning().c_str());
        snprintf(board_built, sizeof(board_date), "%s", board.getCompilationDateOfRunning().c_str());

        gtk_list_store_append(store,&iter);

        gtk_list_store_set(store,&iter,
            COLUMN_SELECTED, board.isSelected(),
            COLUMN_IP_ADDR,board_ipaddr,
            COLUMN_MAC,board_mac,
            COLUMN_BOARD_TYPE,board_type,
            COLUMN_RUNNING_PROCESS, running_process,
            COLUMN_VERSION, board_version,
            COLUMN_DATE, board_date,
            COLUMN_BUILT, board_built,
            COLUMN_INFO, board_info,
            -1);
    }

#endif

    return GTK_TREE_MODEL(store);
}

//Shows a message dialog for displaying infos/errors
GtkWidget* dialog_message_generator(GtkMessageType gtk_message_type,const char *text1,const char *text2)
{
    GtkWidget *message;

    if (gtk_message_type==GTK_MESSAGE_QUESTION)
    {
        message=gtk_dialog_new_with_buttons("Interactive Dialog",
            GTK_WINDOW (window),
            GTK_DIALOG_MODAL,
            GTK_STOCK_YES,
            GTK_RESPONSE_YES,
            GTK_STOCK_NO,
            GTK_RESPONSE_NO,
            NULL);
    }
    else
    {
        message=gtk_dialog_new_with_buttons("Interactive Dialog",
            GTK_WINDOW(window),
            GTK_DIALOG_MODAL,
            GTK_STOCK_OK,
            GTK_RESPONSE_OK,
            NULL);
    }

    gtk_window_set_resizable(GTK_WINDOW(message),false);

    GtkWidget *message_hbox=gtk_hbox_new(FALSE,8);

    gtk_container_set_border_width(GTK_CONTAINER(message_hbox),8);
    gtk_box_pack_start(GTK_BOX(GTK_DIALOG(message)->vbox),message_hbox,FALSE,FALSE,0);

    GtkWidget *message_icon=NULL;

    if (gtk_message_type==GTK_MESSAGE_QUESTION)
    {
        gtk_window_set_title(GTK_WINDOW(message),"Question");
        message_icon=gtk_image_new_from_stock(GTK_STOCK_DIALOG_QUESTION,GTK_ICON_SIZE_DIALOG);
    }
    else if (gtk_message_type==GTK_MESSAGE_ERROR)
    {
        gtk_window_set_title(GTK_WINDOW(message),"Error");
        message_icon=gtk_image_new_from_stock(GTK_STOCK_DIALOG_ERROR,GTK_ICON_SIZE_DIALOG);
    }
    else if (gtk_message_type==GTK_MESSAGE_INFO)
    {
        gtk_window_set_title(GTK_WINDOW(message),"Information");
        message_icon=gtk_image_new_from_stock(GTK_STOCK_DIALOG_INFO,GTK_ICON_SIZE_DIALOG);
    }
    else
    {
        gtk_window_set_title(GTK_WINDOW(message),"Information");
        message_icon=gtk_image_new_from_stock(GTK_STOCK_DIALOG_INFO,GTK_ICON_SIZE_DIALOG);
    }

    gtk_box_pack_start(GTK_BOX(message_hbox),message_icon,FALSE,FALSE,0);

    GtkWidget *message_right_vbox=gtk_vbox_new(FALSE,8);
    gtk_container_set_border_width (GTK_CONTAINER (message_right_vbox), 8);
    gtk_box_pack_start (GTK_BOX (message_hbox), message_right_vbox, FALSE, FALSE, 0);

    GtkWidget *message_label1=gtk_label_new(text1);

    gtk_label_set_justify(GTK_LABEL(message_label1),GTK_JUSTIFY_LEFT);
    gtk_box_pack_start(GTK_BOX(message_right_vbox),message_label1,FALSE,FALSE,0);

    if (text2)
    {
        GtkWidget *message_label2=gtk_label_new(text2);

        gtk_label_set_justify(GTK_LABEL(message_label2),GTK_JUSTIFY_LEFT);
        gtk_box_pack_start(GTK_BOX(message_right_vbox),message_label2,FALSE,FALSE,0);
    }

    gtk_widget_show_all(message_hbox);

    return message;
}

bool dialog_message(GtkMessageType gtk_message_type, const char *text1, const char *text2, bool connect=true)
{
    message=dialog_message_generator(gtk_message_type,text1,text2);

    gtk_widget_show(message);

    if (connect) g_signal_connect_swapped(message,"response",G_CALLBACK(gtk_widget_destroy),message);

    return false;
}

//*********************************************************************************

static void fixed_toggled(GtkCellRendererToggle *cell,gchar *path_str,gpointer data)
{
    GtkTreeModel *model=gtk_tree_view_get_model(GTK_TREE_VIEW(treeview));
    GtkTreePath *path=gtk_tree_path_new_from_string(path_str);
     
    GtkTreeIter  iter;
    gtk_tree_model_get_iter(model,&iter,path);
    
    gboolean fixed;
    gtk_tree_model_get(model,&iter,COLUMN_SELECTED,&fixed,-1);

    fixed=!fixed;

    int* index=gtk_tree_path_get_indices(path);

#if !defined(ETHLOADER_USE_MAINTAINER)
    if (index!=NULL && index[0]<gUpdater.getBoardList().size())
    {
        gUpdater.getBoardList()[index[0]].mSelected=fixed;
    }
#else    
    if (index!=NULL && index[0]<gMNT.boards_get().size())
    {
        gMNT.boards_get()[index[0]].setSelected(fixed);
    }
#endif
    else
    {
        yError ("Something wrong in the selection\n");
    }

    gtk_list_store_set(GTK_LIST_STORE(model),&iter,COLUMN_SELECTED,fixed,-1);

    gtk_tree_path_free(path);

    activate_buttons();
}

bool dialog_question_ip_address(char* old_addr,char* new_addr,const char* ip_or_mask)
{
    char text[256];
    sprintf(text,"Do you really want to change the %s of this board to %s?\r\n\r\nThe new %s will be %s",ip_or_mask,old_addr,ip_or_mask,new_addr) ;

    message=dialog_message_generator(GTK_MESSAGE_QUESTION,text,NULL);
    gtk_window_set_modal(GTK_WINDOW(message),true);

    gint response=gtk_dialog_run(GTK_DIALOG(message));
    
    gtk_widget_destroy(message);
    
    return response==GTK_RESPONSE_YES;
}

bool dialog_question_info(string &old_info, string &new_info)
{
    char text[256];
    sprintf(text,"Do you really want to change the info of this board from %s to %s?\r\n\r\n", old_info.c_str(), new_info.c_str()) ;

    message=dialog_message_generator(GTK_MESSAGE_QUESTION,text,NULL);
    gtk_window_set_modal(GTK_WINDOW(message),true);

    gint response=gtk_dialog_run(GTK_DIALOG(message));

    gtk_widget_destroy(message);

    return response==GTK_RESPONSE_YES;
}


bool dialog_eeprom_erase(void)
{
    char text[256];
    sprintf(text,"Do you really want to erase the EEPROM of the selected boards?\r\n Their Info will be lost and after restart their IP addresses will become 10.0.1.99\r\n") ;

    message=dialog_message_generator(GTK_MESSAGE_QUESTION,text,NULL);
    gtk_window_set_modal(GTK_WINDOW(message),true);

    gint response=gtk_dialog_run(GTK_DIALOG(message));

    gtk_widget_destroy(message);

    return response==GTK_RESPONSE_YES;
}


bool dialog_info_erase(void)
{
    char text[256];
    sprintf(text,"Do you really want to erase the EEPROM of the selected boards?\r\n Their Info will be lost.\r\n") ;

    message=dialog_message_generator(GTK_MESSAGE_QUESTION,text,NULL);
    gtk_window_set_modal(GTK_WINDOW(message),true);

    gint response=gtk_dialog_run(GTK_DIALOG(message));

    gtk_widget_destroy(message);

    return response==GTK_RESPONSE_YES;
}


void dialog_error_ip_address(char* new_addr)
{
    char text[256];
    sprintf(text,"New address %s is not permitted!\r\n\r\n",new_addr);

    dialog_message(GTK_MESSAGE_ERROR,"ip address is wrong!",text);
    return;
}

static void edited_ip_addr(GtkCellRendererText *cell,gchar *path_str,gchar *new_addr,gpointer data)
{
    GtkTreeModel *model=gtk_tree_view_get_model(GTK_TREE_VIEW(treeview));
    GtkTreePath *path=gtk_tree_path_new_from_string(path_str);
    GtkTreeIter iter;
    gtk_tree_model_get_iter(model,&iter,path);
    int* index=gtk_tree_path_get_indices(path);

    int ip1, ip2, ip3, ip4;
    sscanf(new_addr,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    if (ip1<0 || ip1>255 || ip2<0 || ip2>255 || ip3<0 || ip3>255 || ip4<0 || ip4>255) return;
    ACE_UINT32 iNewAddress=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;

#if !defined(ETHLOADER_USE_MAINTAINER)
    ACE_UINT32 address=gUpdater.getBoardList()[index[0]].mAddress;
    ACE_UINT32 mask=gUpdater.getBoardList()[index[0]].mMask;
#else
    ACE_UINT32 address = ipv4toace(gMNT.boards_get()[index[0]].getIPV4());
    ACE_UINT32 mask = 0xFFFFFF00;
#endif

    if(iNewAddress == (iNewAddress & mask)) // checks new ip address is not a network address . For example x.y.z.w/24 x.y.z.0
    {
        dialog_error_ip_address(new_addr);
        return;
    }

    if((~mask) == (iNewAddress & (~mask))) // checks new ip address is not a broadcast address . For example x.y.z.w/24 x.y.z.255
    {
        dialog_error_ip_address(new_addr);
        return;
    }


    if (iNewAddress!=address)
    {
        char old_addr[16];
        sprintf(old_addr,"%d.%d.%d.%d",(address>>24)&0xFF,(address>>16)&0xFF,(address>>8)&0xFF,address&0xFF);
        
        if (dialog_question_ip_address(old_addr,new_addr,"IP address"))
        {
#if !defined(ETHLOADER_USE_MAINTAINER)
            gUpdater.cmdChangeAddress(iNewAddress, address);
#else
            gMNT.command_changeaddress(acetoipv4(address), acetoipv4(iNewAddress), true, true, true, true);
#endif
        }

    }

    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview, NULL);

    // clean up 
    gtk_tree_path_free(path);
}

// keep it as a hidden capability. it is dangerous and highly improbable to change the ip mask
//static void edited_ip_mask(GtkCellRendererText *cell,gchar *path_str,gchar *new_mask)
//{
//    GtkTreeModel *model=gtk_tree_view_get_model(GTK_TREE_VIEW(treeview));
//    GtkTreePath *path=gtk_tree_path_new_from_string(path_str);
//    GtkTreeIter iter;
//    gtk_tree_model_get_iter(model,&iter,path);
//    int* index=gtk_tree_path_get_indices(path);

//    int ip1,ip2,ip3,ip4;
//    sscanf(new_mask,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
//    if (ip1<0 || ip1>255 || ip2<0 || ip2>255 || ip3<0 || ip3>255 || ip4<0 || ip4>255) return;
//    ACE_UINT32 iNewMask=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;
//    ACE_UINT32 iOldMask=gUpdater.getBoardList()[index[0]].mMask;
    
//    ACE_UINT32 address=gUpdater.getBoardList()[index[0]].mAddress;

//    if (iNewMask!=iOldMask)
//    {
//        char old_mask[16];
//        sprintf(old_mask,"%d.%d.%d.%d",(iOldMask>>24)&0xFF,(iOldMask>>16)&0xFF,(iOldMask>>8)&0xFF,iOldMask&0xFF);
        
//        if (dialog_question_ip_address(old_mask,new_mask,"IP mask"))
//        {
//            gUpdater.cmdChangeMask(iNewMask, address);
//        }
//    }

//    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
//    gtk_widget_draw(treeview, NULL);

//    // clean up
//    gtk_tree_path_free(path);
//}


static void edited_info(GtkCellRendererText *cell, gchar *path_str, gchar *newinfo, gpointer data)
{
    GtkTreeModel *model=gtk_tree_view_get_model(GTK_TREE_VIEW(treeview));
    GtkTreePath *path=gtk_tree_path_new_from_string(path_str);
    GtkTreeIter iter;
    gtk_tree_model_get_iter(model,&iter,path);
    int* index=gtk_tree_path_get_indices(path);
#if !defined(ETHLOADER_USE_MAINTAINER)
    ACE_UINT32 address=gUpdater.getBoardList()[index[0]].mAddress;

    string oldInfo = gUpdater.getBoardList()[index[0]].mInfo32;
#else
    eOipv4addr_t ipv4 = gMNT.boards_get()[index[0]].getIPV4();

    string oldInfo = gMNT.boards_get()[index[0]].getInfoOnEEPROM();
#endif

    string newInfo = string(newinfo);


    bool strings_are_different = (newInfo != oldInfo);
    // marco.accame.TODO: get the current info and check
    if (strings_are_different)
    {

        if (dialog_question_info(oldInfo, newInfo))
        {
#if !defined(ETHLOADER_USE_MAINTAINER)
            gUpdater.cmdInfo32Set(newInfo, address);
//            gUpdater.cmdGetMoreInfo(true, address);
            vector<string> vv = gUpdater.cmdInfo32Get(address);
            if(vv.size() > 0)
            {
                gUpdater.getBoardList()[index[0]].mInfo32 = vv[0];
            }
#else
            gMNT.command_info32_set(ipv4, newInfo);

            vector<string> vv = gMNT.command_info32_get(ipv4);

            // it already sets it internally to commandInfo32Get()
//            if(vv.size() > 0)
//            {
//                gUpdater.getBoardList()[index[0]].mInfo32 = vv[0];
//            }
#endif
        }

    }

    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview, NULL);

    // clean up
    gtk_tree_path_free(path);
}


//*********************************************************************************


static void add_columns(GtkTreeView *treeview)
{
    GtkCellRenderer *renderer;
    GtkTreeViewColumn *column;
    GtkTreeModel *model=gtk_tree_view_get_model(treeview);

    // column 1 SELECTED
    renderer=gtk_cell_renderer_toggle_new();
    g_signal_connect(renderer,"toggled",G_CALLBACK(fixed_toggled),NULL);
    column=gtk_tree_view_column_new_with_attributes("Selected",renderer,"active",COLUMN_SELECTED,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),70);
    gtk_tree_view_append_column(treeview,column);

    // column 2 IP_ADDR
    renderer=gtk_cell_renderer_text_new();
    GTK_CELL_RENDERER_TEXT(renderer)->editable=true;
    GTK_CELL_RENDERER_TEXT(renderer)->editable_set=true;
    renderer->mode=GTK_CELL_RENDERER_MODE_EDITABLE;
    g_signal_connect(renderer,"edited",G_CALLBACK(edited_ip_addr),NULL);
    column=gtk_tree_view_column_new_with_attributes("IP address",renderer,"text",COLUMN_IP_ADDR,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),90);
    //gtk_tree_view_column_set_sort_column_id(column,COLUMN_IP_ADDR);
    gtk_tree_view_append_column(treeview,column);


    // column 3 MAC
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("MAC",renderer,"text",COLUMN_MAC,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),160);
    gtk_tree_view_append_column(treeview,column);

    // column 4 BOARD TYPE
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes ("Board Type",renderer,"text",COLUMN_BOARD_TYPE,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),120);
    gtk_tree_view_append_column(treeview,column);

    // column 5 RUNNING PROCESS
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes ("Process",renderer,"text",COLUMN_RUNNING_PROCESS,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),140);
    gtk_tree_view_append_column(treeview,column);

    // column 6 VERSION
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("Version",renderer,"text",COLUMN_VERSION,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),65);
    gtk_tree_view_append_column(treeview,column);

    // column 7 DATE
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("Dated",renderer,"text",COLUMN_DATE,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),150);
    gtk_tree_view_append_column(treeview,column);

    // column 7 DATE
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("Built On",renderer,"text",COLUMN_BUILT,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),150);
    gtk_tree_view_append_column(treeview,column);

    // column 9 INFO
    renderer=gtk_cell_renderer_text_new();

    GTK_CELL_RENDERER_TEXT(renderer)->editable=true;
    GTK_CELL_RENDERER_TEXT(renderer)->editable_set=true;
    renderer->mode=GTK_CELL_RENDERER_MODE_EDITABLE;
    g_signal_connect(renderer,"edited",G_CALLBACK(edited_info),NULL);

    column=gtk_tree_view_column_new_with_attributes("Info",renderer,"text",COLUMN_INFO,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),270);
    gtk_tree_view_append_column(treeview,column);


}


static void destroy_main(GtkWindow *window,	gpointer user_data)
{
   gtk_widget_destroy(GTK_WIDGET(window));
   gtk_main_quit();
}

static void updateProgressBar(float fraction)
{
    if (fraction<0.0f) fraction=0.0f; else if (fraction>1.0f) fraction=1.0f;
    gtk_progress_bar_set_fraction((GtkProgressBar*)progress_bar,fraction);
    gtk_widget_draw(window,NULL);
    gtk_main_iteration_do(false);
}

static void discover_cbk(GtkButton *button,gpointer user_data)
{
    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),"",-1);

#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.cmdDiscover();
#else
//    bool clearlist = true;
//    eOipv4addr_t ipv4 = 0;
//    gMNT.clearBoards();
//    gMNT.addBoard(EO_COMMON_IPV4ADDR(10, 0, 1, 1));
//    gMNT.addBoard(EO_COMMON_IPV4ADDR(10, 0, 1, 8),);

    // default values
    bool clearlist = true;
    gMNT.discover(true, 2, 1.0);
#endif

    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);

    activate_buttons();
}

#if !defined(ETHLOADER_USE_MAINTAINER)
#else
static void maintenanceON_cbk(GtkButton *button,gpointer user_data)
{
    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),"",-1);


    gMNT.go2maintenance(EthMaintainer::ipv4OfAllSelected, true, 5, 1.0);


    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);

    activate_buttons();
}

static void applicationON_cbk(GtkButton *button,gpointer user_data)
{
    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),"",-1);

    gMNT.go2application(EthMaintainer::ipv4OfAllSelected, true, 10, true);

    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);

    activate_buttons();
}
#endif

static void upload_cbk(GtkButton *button, gpointer user_data)
{ 
#if !defined(ETHLOADER_USE_MAINTAINER) 
    if (!gUpdater.getBoardList().numSelected())
#else
    if(0 == gMNT.boards_get().numberof(EthMaintainer::ipv4OfAllSelected))
#endif
    {
        dialog_message(GTK_MESSAGE_ERROR,"No board selected!","Select a board to download the firmware");
        return;
    }
   /* else if (gUpdater.getBoardList().numSelected()!=1)
    {
        dialog_message(GTK_MESSAGE_ERROR,"Too many boards selected!","Select a single board to download the firmware");
        return;
    }*/

    GtkWidget *dialog=gtk_file_chooser_dialog_new("Choose File",
                                                  GTK_WINDOW(window),
				                                  GTK_FILE_CHOOSER_ACTION_OPEN,
				                                  GTK_STOCK_CANCEL, 
                                                  GTK_RESPONSE_CANCEL,
				                                  GTK_STOCK_OPEN, 
                                                  GTK_RESPONSE_ACCEPT,
                                                  NULL);

    if (gtk_dialog_run(GTK_DIALOG(dialog))!=GTK_RESPONSE_ACCEPT)
    {
        gtk_widget_destroy(dialog);
        return;
    }

    char *filename=gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
    gtk_widget_destroy(dialog);

    if (!filename)
    {
        dialog_message(GTK_MESSAGE_ERROR,"No files selected!","Select the file You want to download, first");
        return;
    }

	printf("Programming: %s\n",filename);

    FILE *programFile=fopen(filename,"r");
    g_free(filename);

    if (!programFile)
    {
        dialog_message(GTK_MESSAGE_ERROR,"Error opening the selected file!","");
        return;
    }
#if !defined(ETHLOADER_USE_MAINTAINER)
    uint32_t addr = 0; // all selected
    //addr = (10 << 24) | (0 << 16) | (1 << 8) | (2); test only one board w/ address 10.0.1.2
    std::string result=gUpdater.cmdProgram(programFile,*(int*)user_data,updateProgressBar, addr);
    fclose(programFile);
#else

#define TEST_PROG
#if defined(TEST_PROG)


    //fclose(programFile);

    std::string result;

    eOuprot_process_t process = eApplication; //eApplPROGupdater; //eApplication; //eLoader;
    if(*(int*)user_data == uprot_partitionLOADER)
    {
        process = eLoader;

    }
    else if(*(int*)user_data == uprot_partitionUPDATER)
    {
        process = eUpdater;
    }
    else if(*(int*)user_data == uprot_partitionAPPLICATION)
    {
        process = eApplication;
    }


    {

        gMNT.verbose(true);
    eOversion_t ver;
    ver.major = 0;
    ver.minor = 0;
    eOipv4addr_t ipv4 = 0;
    eObrd_ethtype_t type = eobrd_ethtype_none; //eobrd_ethtype_ems4;
    bool ret = gMNT.program(ipv4, type, process, ver, programFile, false, updateProgressBar, false);


    if(ret)
        result = "OK";
    else
        result = "KO";

    fclose(programFile);
#else
    eOipv4addr_t ipv4 = 0; // all selected
    //addr = (10 << 24) | (0 << 16) | (1 << 8) | (2); test only one board w/ address 10.0.1.2
    //std::string result=gUpdater.cmdProgram(programFile,*(int*)user_data,updateProgressBar, addr);
    std::string result;
    bool ok = gMNT.commandProgram(programFile,(eOuprot_partition2prog_t)*(int*)user_data, updateProgressBar, NULL, result, ipv4);
    ok = ok;
    fclose(programFile);
#endif
    }

#endif


    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),result.c_str(),-1);
}

static void sel_all_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.getBoardList().selectAll(true);
#else
    eOipv4addr_t ipv4 = EthBoardList::ipv4all;
    gMNT.boards_get().select(true, ipv4);
#endif

    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);

    activate_buttons();
}

static void des_all_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.getBoardList().selectAll(false);
#else
    eOipv4addr_t ipv4 = EthBoardList::ipv4all;
    gMNT.boards_get().select(false, ipv4);
#endif
   
    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);

    activate_buttons();
}

static void boot_upd_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.cmdSetDEF2RUN(eUpdater);
#else
    gMNT.command_def2run(EthMaintainer::ipv4OfAllSelected, eUpdater, false, false);
#endif
}

static void boot_app_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.cmdSetDEF2RUN(eApplication);
#else
    gMNT.command_def2run(EthMaintainer::ipv4OfAllSelected, eApplication, false, false);
#endif
}

static void jump_upd_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.cmdJumpUpd();
#else
    gMNT.command_jump2updater(EthMaintainer::ipv4OfAllSelected);
#endif
}

static void reset_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
    gUpdater.cmdRestart();
#else
    gMNT.command_restart(EthMaintainer::ipv4OfAllSelected);
#endif
}

static void procs_cbk(GtkButton *button,gpointer user_data)
{
    //std::string procs=gUpdater.cmdGetProcs();
#if !defined(ETHLOADER_USE_MAINTAINER)
    std::string procs=gUpdater.cmdGetMoreInfo();
#else
    std::string procs = gMNT.moreinformation(EthMaintainer::ipv4OfAllSelected, false);
#endif

    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)), procs.c_str(), -1);
}
       
static void blink_cbk(GtkButton *button,gpointer user_data)
{
#if !defined(ETHLOADER_USE_MAINTAINER)
	gUpdater.cmdBlink();
#else
    gMNT.command_blink(EthMaintainer::ipv4OfAllSelected);
#endif
}

static void eraseinfo_cbk(GtkButton *button,gpointer user_data)
{
    if(dialog_info_erase())
    {
#if !defined(ETHLOADER_USE_MAINTAINER)
        gUpdater.cmdInfo32Clear();
#else
        gMNT.command_info32_clr(EthMaintainer::ipv4OfAllSelected);
#endif
    }
}

static void eraseeprom_cbk(GtkButton *button, gpointer user_data)
{
    // by definining this macro we test the set/clr/get of info32
//#define TEST_INFO32

#if defined(TEST_INFO32)

    static int iter = 0;
    const int clear_frequency = 3;

    if(0 == iter)
    {
        gUpdater.cmdInfo32Clear();
    }
    else
    {
        //    const char str32[3][40] = {"ciao uomo come stai?", "ieri ero a parigi", "domani piove"};
        //    static int index = 0;
        //    cmd[2] = strlen(str32[index]);
        //    snprintf((char*)&cmd[3], 32, str32[index]);
        //    index ++;
        //    index %= 3;
        static int ii = 0;
        std::string str32[4] = {"STRINGA CON PIU' DI 32 CARATTERI SICURAMENTE", "ciao come stai?", "ieri oggi domani", "la sciagurata rispose"};
        gUpdater.cmdInfo32Set(str32[ii]);
        printf("\nIn TEST_INFO32: wrnitte str32 = %s\n", str32[ii].c_str());
        ii++;
        ii %= 4;
    }

    std::vector<std::string> str32v = gUpdater.cmdInfo32Get();
    for(int i=0; i<str32v.size(); i++)
    {
        printf("\nIn TEST_INFO32: read str32 = %s\n", str32v[i].c_str());
    }

    iter++;

    iter %= clear_frequency;

#else

    if(dialog_eeprom_erase())
    {
#if !defined(ETHLOADER_USE_MAINTAINER)
        gUpdater.cmdEraseEEPROM();
#else
        gMNT.command_eeprom_erase(EthMaintainer::ipv4OfAllSelected);
#endif
    }

#endif
}

static void info_cls_cbk(GtkButton *button,gpointer user_data)
{
    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),"",-1);
}


#define MY_ADDR "10.0.1.104"
#define MY_PORT 3333

#if !defined(ETHLOADER_USE_MAINTAINER)
#else
const eOipv4addr_t myIPaddress = EO_COMMON_IPV4ADDR(10, 0, 1, 104);
const eOipv4port_t myIPport = 3333;
#endif

// Entry point for the GTK application
int myMain(int argc,char *argv[])
{
    yarp::os::Property config;

    config.fromCommand(argc,argv);
    
    int port=MY_PORT;

    bool bPrintUsage=false;

    if (config.check("port"))
    {
        port=config.find("port").asInt();
    }
    else
    {
        printf("Using default port %d\n",MY_PORT);
        bPrintUsage=true;
    }

    std::string address(MY_ADDR);

    if (config.check("address"))
    {
        address=std::string(config.find("address").asString().c_str());
    }
    else
    {
        printf("Using default port %s\n",MY_ADDR);
        bPrintUsage=true;
    }

    if (bPrintUsage)
    {
        printf("Usage: %s --port n --address xxx.xxx.xxx.xxx\n",argv[0]);
    }

#if !defined(ETHLOADER_USE_MAINTAINER)
    if (!gUpdater.create(port,address))
#else
    if(!gMNT.open())
#endif
    {
        printf("Can't open socket, aborting.");
        
        return -1;
    }

    GtkTreeModel *model;

	gtk_init(&argc,&argv);

    //create the main window, and sets the callback destroy_main() to quit
    //the application when the main window is closed
    window=gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW (window), "ETH Flasher V1.0");
    g_signal_connect(window,"destroy",G_CALLBACK(destroy_main),&window);

    gtk_container_set_border_width(GTK_CONTAINER(window),8);

    //Creation of main_vbox the container for every other widget
    main_hbox=gtk_hbox_new(FALSE,8);
    gtk_container_add(GTK_CONTAINER(window),main_hbox);
    gtk_container_set_border_width(GTK_CONTAINER(main_hbox),10);

    //In the bottom frame there is:
    //1) the list of the cards
    sw=gtk_scrolled_window_new(NULL,NULL);

    gtk_scrolled_window_set_shadow_type(GTK_SCROLLED_WINDOW(sw),GTK_SHADOW_ETCHED_IN);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(sw),GTK_POLICY_NEVER,GTK_POLICY_AUTOMATIC);
    gtk_box_pack_start(GTK_BOX(main_hbox),sw,TRUE,TRUE,0);
    
    //List of the cards
    //create tree model
    model=refresh_board_list_model();

    //create tree view
    treeview=gtk_tree_view_new_with_model(model);
    gtk_tree_view_set_rules_hint(GTK_TREE_VIEW(treeview),TRUE);
    gtk_tree_view_set_search_column(GTK_TREE_VIEW(treeview),COLUMN_IP_ADDR);

    g_object_unref(model);

    gtk_container_add(GTK_CONTAINER(sw),treeview);

    // add columns to the tree view
    add_columns(GTK_TREE_VIEW(treeview));

    //2) A panel in the right
    buttons_box=gtk_vbox_new(FALSE,8);
    gtk_container_set_border_width(GTK_CONTAINER(buttons_box),10);
    gtk_container_add(GTK_CONTAINER(main_hbox),buttons_box);

    ////
        //add_button("Discover",      discover_button, discover_cbk, buttons_box);
        discover_button=gtk_button_new_with_mnemonic("Discover");
        gtk_container_add(GTK_CONTAINER(buttons_box),discover_button);
        g_signal_connect(discover_button,"clicked",G_CALLBACK(discover_cbk),NULL);
        gtk_widget_set_size_request(discover_button,180,30);

        //add_button("Show Procs",    procs_button,    procs_cbk,    buttons_box);
        procs_button=gtk_button_new_with_mnemonic("More Details");
        gtk_container_add(GTK_CONTAINER(buttons_box),procs_button);
        g_signal_connect(procs_button,"clicked",G_CALLBACK(procs_cbk),NULL);
        gtk_widget_set_size_request(procs_button,180,30);

        //add_button("Select All",    sel_all_button,  sel_all_cbk,  buttons_box);
        sel_all_button=gtk_button_new_with_mnemonic("Select All");
        gtk_container_add(GTK_CONTAINER(buttons_box),sel_all_button);
        g_signal_connect(sel_all_button,"clicked",G_CALLBACK(sel_all_cbk),NULL);
        gtk_widget_set_size_request(sel_all_button,180,30);

        //add_button("Deselect All",  des_all_button,  des_all_cbk,  buttons_box);
        des_all_button=gtk_button_new_with_mnemonic("Deselect All");
        gtk_container_add(GTK_CONTAINER(buttons_box),des_all_button);
        g_signal_connect(des_all_button,"clicked",G_CALLBACK(des_all_cbk),NULL);
        gtk_widget_set_size_request(des_all_button,180,30);

#if !defined(ETHLOADER_USE_MAINTAINER)
        int partition_APPLICATION = EthUpdater::partition_APPLICATION;
        int partition_UPDATER = EthUpdater::partition_UPDATER;
        int partition_LOADER = EthUpdater::partition_LOADER;
#else
        int partition_APPLICATION = uprot_partitionAPPLICATION;
        int partition_UPDATER = uprot_partitionUPDATER;
        int partition_LOADER = uprot_partitionLOADER;

        maintenanceON_button=gtk_button_new_with_mnemonic("Maintenance ON");
        gtk_container_add(GTK_CONTAINER(buttons_box), maintenanceON_button);
        g_signal_connect(maintenanceON_button,"clicked",G_CALLBACK(maintenanceON_cbk),NULL);
        gtk_widget_set_size_request(maintenanceON_button,180,30);

        applicationON_button=gtk_button_new_with_mnemonic("Application ON");
        gtk_container_add(GTK_CONTAINER(buttons_box), applicationON_button);
        g_signal_connect(applicationON_button,"clicked",G_CALLBACK(applicationON_cbk),NULL);
        gtk_widget_set_size_request(applicationON_button,180,30);
#endif
        
        //add_button("Start Upload",  upload_button,   upload_cbk,   buttons_box);
        upload_button=gtk_button_new_with_mnemonic("Program eApplication");
        gtk_container_add(GTK_CONTAINER(buttons_box),upload_button);
        g_signal_connect(upload_button,"clicked",G_CALLBACK(upload_cbk),(gpointer*)&partition_APPLICATION);
        gtk_widget_set_size_request(upload_button,180,30);

        //add_button("Upload Updater",  upload_updtater_button,   upload_cbk,   buttons_box);
        upload_updater_button=gtk_button_new_with_mnemonic("Program eUpdater");
        gtk_container_add(GTK_CONTAINER(buttons_box),upload_updater_button);
        g_signal_connect(upload_updater_button,"clicked",G_CALLBACK(upload_cbk),(gpointer*)&partition_UPDATER);
        gtk_widget_set_size_request(upload_updater_button,180,30);

        //add_button("Upload Loader",  upload_loader_button,   upload_cbk,   buttons_box);
        upload_loader_button=gtk_button_new_with_mnemonic("Program eLoader");
        gtk_container_add(GTK_CONTAINER(buttons_box),upload_loader_button);
        g_signal_connect(upload_loader_button,"clicked",G_CALLBACK(upload_cbk),(gpointer*)&partition_LOADER);
        gtk_widget_set_size_request(upload_loader_button,180,30);

        progress_bar=gtk_progress_bar_new();
        gtk_container_add(GTK_CONTAINER(buttons_box),progress_bar);
        gtk_widget_set_size_request(progress_bar,180,10);


        //add_button("Boot from Upd", boot_upd_button, boot_upd_cbk, buttons_box);
        boot_upd_button=gtk_button_new_with_mnemonic("DEF2RUN = eUpdater");
        gtk_container_add(GTK_CONTAINER(buttons_box),boot_upd_button);
        g_signal_connect(boot_upd_button,"clicked",G_CALLBACK(boot_upd_cbk),NULL);
        gtk_widget_set_size_request(boot_upd_button,180,30);

        //add_button("Boot from App", boot_app_button, boot_app_cbk, buttons_box);
        boot_app_button=gtk_button_new_with_mnemonic("DEF2RUN = eApplication");
        gtk_container_add(GTK_CONTAINER(buttons_box),boot_app_button);
        g_signal_connect(boot_app_button,"clicked",G_CALLBACK(boot_app_cbk),NULL);
        gtk_widget_set_size_request(boot_app_button,180,30);

        //add_button("Jump to Upd",   jump_upd_button, jump_upd_cbk, buttons_box);
        jump_upd_button=gtk_button_new_with_mnemonic("Jump to eUpdater");
        gtk_container_add(GTK_CONTAINER(buttons_box),jump_upd_button);
        g_signal_connect(jump_upd_button,"clicked",G_CALLBACK(jump_upd_cbk),NULL);
        gtk_widget_set_size_request(jump_upd_button,180,30);
        
        //add_button("Reset",         reset_button,    reset_cbk,    buttons_box);
        reset_button=gtk_button_new_with_mnemonic("Restart Board");
        gtk_container_add(GTK_CONTAINER(buttons_box),reset_button);
        g_signal_connect(reset_button,"clicked",G_CALLBACK(reset_cbk),NULL);
        gtk_widget_set_size_request(reset_button,180,30);

//        //add_button("Show Procs",    procs_button,    procs_cbk,    buttons_box);
//        procs_button=gtk_button_new_with_mnemonic("More Details");
//        gtk_container_add(GTK_CONTAINER(buttons_box),procs_button);
//        g_signal_connect(procs_button,"clicked",G_CALLBACK(procs_cbk),NULL);
//        gtk_widget_set_size_request(procs_button,180,30);
         
        //add_button("Erase Eprom",    eeprom_button,    procs_cbk,    buttons_box);
        eprom_button=gtk_button_new_with_mnemonic("Erase EEPROM");
        gtk_container_add(GTK_CONTAINER(buttons_box),eprom_button);
        g_signal_connect(eprom_button,"clicked",G_CALLBACK(eraseeprom_cbk),NULL);
        gtk_widget_set_size_request(eprom_button,180,30);

        //add_button("Blink",         blink_button,    blink_cbk,    buttons_box);
        blink_button=gtk_button_new_with_mnemonic("Blink LEDs");
        gtk_container_add(GTK_CONTAINER(buttons_box),blink_button);
        g_signal_connect(blink_button,"clicked",G_CALLBACK(blink_cbk),NULL);
        gtk_widget_set_size_request(blink_button,180,30);

        //add_button("Erase Eprom",    eeprom_button,    procs_cbk,    buttons_box);
        einfo_button=gtk_button_new_with_mnemonic("Erase Info");
        gtk_container_add(GTK_CONTAINER(buttons_box),einfo_button);
        g_signal_connect(einfo_button,"clicked",G_CALLBACK(eraseinfo_cbk),NULL);
        gtk_widget_set_size_request(einfo_button,180,30);

    ////

    //3) A panel in the right
    right_vbox=gtk_vbox_new(FALSE,8);
    gtk_container_set_border_width(GTK_CONTAINER(right_vbox),10);
    gtk_container_add(GTK_CONTAINER(main_hbox),right_vbox);

    info_scroll=gtk_scrolled_window_new(NULL,NULL);
    gtk_scrolled_window_set_shadow_type(GTK_SCROLLED_WINDOW(info_scroll),GTK_SHADOW_ETCHED_IN);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(info_scroll),GTK_POLICY_NEVER,GTK_POLICY_AUTOMATIC);
    gtk_box_pack_start(GTK_BOX(right_vbox),info_scroll,TRUE,TRUE,0);

    info_text=gtk_text_view_new();
    gtk_container_add(GTK_CONTAINER(info_scroll),info_text);
    gtk_widget_set_size_request(info_scroll,200,500);

    info_cls_button=gtk_button_new_with_mnemonic("Clear");
    g_signal_connect(info_cls_button,"clicked",G_CALLBACK(info_cls_cbk),NULL);
    gtk_container_add(GTK_CONTAINER(right_vbox),info_cls_button);
    gtk_widget_set_size_request(info_cls_button,200,26);

    // finish & show

    gtk_window_set_default_size(GTK_WINDOW(window),480,450);
    gtk_window_set_resizable(GTK_WINDOW(window),false);

    if (!GTK_WIDGET_VISIBLE(window))
    {
        gtk_widget_show_all(window);
    }
    else
    {
        gtk_widget_destroy(window);
        window=NULL;
    }

    activate_buttons();

    gtk_main();
    
    return 0;
}

int main(int argc,char* argv[])
{
    return myMain(argc,argv);
}
