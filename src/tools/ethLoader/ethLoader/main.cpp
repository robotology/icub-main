/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <ace/ACE.h>

#include <gtk/gtk.h>
#include <gtk/gtkmain.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include <yarp/os/Property.h>

#include "EthUpdater.h"

EthUpdater gUpdater;

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
GtkWidget *upload_button   = NULL;

GtkWidget *boot_upd_button = NULL;
GtkWidget *boot_app_button = NULL;
GtkWidget *jump_upd_button = NULL;
        
GtkWidget *reset_button    = NULL;
GtkWidget *procs_button    = NULL;
         
GtkWidget *blink_button    = NULL;
GtkWidget *progress_bar    = NULL;

GtkWidget *message         = NULL;
GtkWidget *message_hbox    = NULL;

enum
{
    COLUMN_SELECTED,
    COLUMN_IP_ADDR,
    COLUMN_IP_MASK,
    COLUMN_MAC,
    COLUMN_VERSION,
    COLUMN_RELEASE,
    COLUMN_BUILD,
    NUM_COLUMNS
};

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
        G_TYPE_STRING
		);

    char board_ipaddr[16];
    char board_ipmask[16];
    char board_mac[16];

    char board_version[16];
    char board_release[16];
    char board_build[16];

    // add data to the list store	 
    for (int i=0; i<gUpdater.getBoardList().size(); ++i)
    {
        ACE_UINT32 ip=gUpdater.getBoardList()[i].mAddress;
        sprintf(board_ipaddr,"%d.%d.%d.%d",(ip>>24)&0xFF,(ip>>16)&0xFF,(ip>>8)&0xFF,ip&0xFF);
        
        ACE_UINT32 mk=gUpdater.getBoardList()[i].mMask;
        sprintf(board_ipmask,"%d.%d.%d.%d",(mk>>24)&0xFF,(mk>>16)&0xFF,(mk>>8)&0xFF,mk&0xFF);

        ACE_UINT64 mac=gUpdater.getBoardList()[i].mMac;
        ACE_UINT32 macL=(ACE_UINT32)(mac&0xFFFFFFFF);
        ACE_UINT32 macH=(ACE_UINT32)((mac>>32)&0xFFFF);

        sprintf(board_mac,"%04X%08X",macH,macL);

        sprintf(board_version,"%d",gUpdater.getBoardList()[i].mVersion);
        sprintf(board_release,"%X",gUpdater.getBoardList()[i].mRelease);
        sprintf(board_build,  "%d",gUpdater.getBoardList()[i].mBuild);

        gtk_list_store_append(store,&iter);

        gtk_list_store_set(store,&iter,
            COLUMN_SELECTED,gUpdater.getBoardList()[i].mSelected,
            COLUMN_IP_ADDR,board_ipaddr,
            COLUMN_IP_MASK,board_ipmask,
            COLUMN_MAC,board_mac,
            COLUMN_VERSION,board_version,
            COLUMN_RELEASE,board_release,
            COLUMN_BUILD,"",
            -1);
    }

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

    if (index!=NULL && index[0]<gUpdater.getBoardList().size())
    {
        gUpdater.getBoardList()[index[0]].mSelected=fixed;
    }
    else
    {
        printf("ERR: Something wrong in the selection\n");
    }

    gtk_list_store_set(GTK_LIST_STORE(model),&iter,COLUMN_SELECTED,fixed,-1);

    gtk_tree_path_free(path);
}

bool dialog_question_ip_address(char* old_addr,char* new_addr,bool bMask)
{
    char text[256];
    sprintf(text,"Do you really want to change the IP %s of this board?\r\n\r\nCURRENT CAN ADDRESS %s WILL BE CHANGED IN %s",(bMask?"mask":"address"),old_addr,new_addr) ;

    message=dialog_message_generator(GTK_MESSAGE_QUESTION,text,NULL);
    gtk_window_set_modal(GTK_WINDOW(message),true);

    gint response=gtk_dialog_run(GTK_DIALOG(message));
    
    gtk_widget_destroy(message);
    
    return response==GTK_RESPONSE_YES;
}

static void edited_ip_addr_mask(GtkCellRendererText *cell,gchar *path_str,gchar *new_addr,gpointer data)
{
    bool bMask=(data!=NULL);

    GtkTreeModel *model=gtk_tree_view_get_model(GTK_TREE_VIEW(treeview));
    GtkTreePath *path=gtk_tree_path_new_from_string(path_str);
    GtkTreeIter iter;
    gtk_tree_model_get_iter(model,&iter,path);
    int* index=gtk_tree_path_get_indices(path);

    int ip1,ip2,ip3,ip4;
    sscanf(new_addr,"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    if (ip1<0 || ip1>255 || ip2<0 || ip2>255 || ip3<0 || ip3>255 || ip4<0 || ip4>255) return;
    ACE_UINT32 iNewAddress=(ip1<<24)|(ip2<<16)|(ip3<<8)|ip4;
    ACE_UINT32 iOldAddress=bMask?gUpdater.getBoardList()[index[0]].mMask:gUpdater.getBoardList()[index[0]].mAddress;

    if (iNewAddress!=iOldAddress)
    {
        char old_addr[16];
        sprintf(old_addr,"%d.%d.%d.%d",(iOldAddress>>24)&0xFF,(iOldAddress>>16)&0xFF,(iOldAddress>>8)&0xFF,iOldAddress&0xFF);
        
        if (dialog_question_ip_address(old_addr,new_addr,bMask))
        {
            printf("changing address\n");
            fflush(stdout);
            gUpdater.cmdChangeAddressAndMask(iOldAddress,iNewAddress,bMask);
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
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),60);
    gtk_tree_view_append_column(treeview,column);

    // column 2 IP_ADDR
    renderer=gtk_cell_renderer_text_new();
    GTK_CELL_RENDERER_TEXT(renderer)->editable=true;
    GTK_CELL_RENDERER_TEXT(renderer)->editable_set=true;
    renderer->mode=GTK_CELL_RENDERER_MODE_EDITABLE;
    g_signal_connect(renderer,"edited",G_CALLBACK(edited_ip_addr_mask),(gpointer)false);
    column=gtk_tree_view_column_new_with_attributes("IP addr",renderer,"text",COLUMN_IP_ADDR,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),100);
    //gtk_tree_view_column_set_sort_column_id(column,COLUMN_IP_ADDR);
    gtk_tree_view_append_column(treeview,column);

    //column 3 IP_MASK

    renderer=gtk_cell_renderer_text_new();
    GTK_CELL_RENDERER_TEXT(renderer)->editable=true;
    GTK_CELL_RENDERER_TEXT(renderer)->editable_set=true;
    renderer->mode=GTK_CELL_RENDERER_MODE_EDITABLE;
    g_signal_connect(renderer,"edited",G_CALLBACK(edited_ip_addr_mask),(gpointer)true);
    column=gtk_tree_view_column_new_with_attributes("IP mask",renderer,"text",COLUMN_IP_MASK,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),100);
    gtk_tree_view_append_column(treeview,column);

    // column 4 MAC
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("MAC",renderer,"text",COLUMN_MAC,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),120);
    gtk_tree_view_append_column(treeview,column);

    // column 5 VERSION
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("Version",renderer,"text",COLUMN_VERSION,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),60);
    gtk_tree_view_append_column(treeview,column);

    // column 6 RELEASE
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes("Release",renderer,"text",COLUMN_RELEASE,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),60);
    gtk_tree_view_append_column(treeview,column);

    // column 7 BUILD
    renderer=gtk_cell_renderer_text_new();
    column=gtk_tree_view_column_new_with_attributes ("Build",renderer,"text",COLUMN_BUILD,NULL);
    gtk_tree_view_column_set_sizing(GTK_TREE_VIEW_COLUMN(column),GTK_TREE_VIEW_COLUMN_FIXED);
    gtk_tree_view_column_set_fixed_width(GTK_TREE_VIEW_COLUMN(column),60);
    gtk_tree_view_append_column(treeview,column);
}


static void destroy_main(GtkWindow *window,	gpointer user_data)
{
   gtk_widget_destroy(GTK_WIDGET(window));
   gtk_main_quit();
}

static void updateProgressBar(float fraction)
{
    gtk_progress_bar_set_fraction((GtkProgressBar*)progress_bar,fraction);
    gtk_widget_draw(window,NULL);
    gtk_main_iteration_do(false);
}

static void discover_cbk(GtkButton *button,gpointer user_data)
{
    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),"",-1);

    gUpdater.cmdScan();
    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);
}

static void upload_cbk(GtkButton *button,gpointer user_data)
{  
    if (!gUpdater.getBoardList().numSelected())
    {
        dialog_message(GTK_MESSAGE_ERROR,"No Boards selected!","Select one or more boards to download the firmware");
        return;
    }

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

    FILE *programFile=fopen(filename,"r");
    g_free(filename);

    if (!programFile)
    {
        dialog_message(GTK_MESSAGE_ERROR,"Error opening the selected file!","");
        return;
    }
    
    std::string result=gUpdater.cmdProgram(programFile,updateProgressBar);

    fclose(programFile);

    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),result.c_str(),-1);
}

static void sel_all_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.getBoardList().selectAll(true);

    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);
}

static void des_all_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.getBoardList().selectAll(false);
   
    gtk_tree_view_set_model(GTK_TREE_VIEW(treeview),refresh_board_list_model());
    gtk_widget_draw(treeview,NULL);
}

static void boot_upd_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.cmdBootSelect(1);
}

static void boot_app_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.cmdBootSelect(2);
}

static void jump_upd_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.cmdJumpUpd();
}

static void reset_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.cmdReset();
}

static void procs_cbk(GtkButton *button,gpointer user_data)
{
    std::string procs=gUpdater.cmdGetProcs();

    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),procs.c_str(),-1);
}
         
static void blink_cbk(GtkButton *button,gpointer user_data)
{
    gUpdater.cmdBlink();
}

static void info_cls_cbk(GtkButton *button,gpointer user_data)
{
    gtk_text_buffer_set_text(gtk_text_view_get_buffer(GTK_TEXT_VIEW(info_text)),"",-1);
}

#define MY_ADDR "10.0.0.1"
#define MY_PORT 3333

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

    if (!gUpdater.create(port,address))
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
        //add_button("Select All",    sel_all_button,  sel_all_cbk,  buttons_box);
        sel_all_button=gtk_button_new_with_mnemonic("Select All");
        gtk_container_add(GTK_CONTAINER(buttons_box),sel_all_button);
        g_signal_connect(sel_all_button,"clicked",G_CALLBACK(sel_all_cbk),NULL);
        gtk_widget_set_size_request(sel_all_button,100,30);

        //add_button("Deselect All",  des_all_button,  des_all_cbk,  buttons_box);
        des_all_button=gtk_button_new_with_mnemonic("Deselect All");
        gtk_container_add(GTK_CONTAINER(buttons_box),des_all_button);
        g_signal_connect(des_all_button,"clicked",G_CALLBACK(des_all_cbk),NULL);
        gtk_widget_set_size_request(des_all_button,100,30);
        
        //add_button("Discover",      discover_button, discover_cbk, buttons_box);
        discover_button=gtk_button_new_with_mnemonic("Discover");
        gtk_container_add(GTK_CONTAINER(buttons_box),discover_button);
        g_signal_connect(discover_button,"clicked",G_CALLBACK(discover_cbk),NULL);
        gtk_widget_set_size_request(discover_button,100,30);
        
        //add_button("Start Upload",  upload_button,   upload_cbk,   buttons_box);
        discover_button=gtk_button_new_with_mnemonic("Upload");
        gtk_container_add(GTK_CONTAINER(buttons_box),discover_button);
        g_signal_connect(discover_button,"clicked",G_CALLBACK(upload_cbk),NULL);
        gtk_widget_set_size_request(discover_button,100,30);

        
        progress_bar=gtk_progress_bar_new();
        gtk_container_add(GTK_CONTAINER(buttons_box),progress_bar);
        gtk_widget_set_size_request(progress_bar,100,30);


        //add_button("Boot from Upd", boot_upd_button, boot_upd_cbk, buttons_box);
        boot_upd_button=gtk_button_new_with_mnemonic("Boot from Upd");
        gtk_container_add(GTK_CONTAINER(buttons_box),boot_upd_button);
        g_signal_connect(boot_upd_button,"clicked",G_CALLBACK(boot_upd_cbk),NULL);
        gtk_widget_set_size_request(boot_upd_button,100,30);

        //add_button("Boot from App", boot_app_button, boot_app_cbk, buttons_box);
        boot_app_button=gtk_button_new_with_mnemonic("Boot from App");
        gtk_container_add(GTK_CONTAINER(buttons_box),boot_app_button);
        g_signal_connect(boot_app_button,"clicked",G_CALLBACK(boot_app_cbk),NULL);
        gtk_widget_set_size_request(boot_app_button,100,30);

        //add_button("Jump to Upd",   jump_upd_button, jump_upd_cbk, buttons_box);
        jump_upd_button=gtk_button_new_with_mnemonic("Jump to Upd");
        gtk_container_add(GTK_CONTAINER(buttons_box),jump_upd_button);
        g_signal_connect(jump_upd_button,"clicked",G_CALLBACK(jump_upd_cbk),NULL);
        gtk_widget_set_size_request(jump_upd_button,100,30);
        
        //add_button("Reset",         reset_button,    reset_cbk,    buttons_box);
        reset_button=gtk_button_new_with_mnemonic("Reset");
        gtk_container_add(GTK_CONTAINER(buttons_box),reset_button);
        g_signal_connect(reset_button,"clicked",G_CALLBACK(reset_cbk),NULL);
        gtk_widget_set_size_request(reset_button,100,30);

        //add_button("Show Procs",    procs_button,    procs_cbk,    buttons_box);
        procs_button=gtk_button_new_with_mnemonic("Procs");
        gtk_container_add(GTK_CONTAINER(buttons_box),procs_button);
        g_signal_connect(procs_button,"clicked",G_CALLBACK(procs_cbk),NULL);
        gtk_widget_set_size_request(procs_button,100,30);
         
        //add_button("Blink",         blink_button,    blink_cbk,    buttons_box);
        blink_button=gtk_button_new_with_mnemonic("Blink");
        gtk_container_add(GTK_CONTAINER(buttons_box),blink_button);
        g_signal_connect(blink_button,"clicked",G_CALLBACK(blink_cbk),NULL);
        gtk_widget_set_size_request(blink_button,100,30);

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
    gtk_widget_set_size_request(info_scroll,200,450);

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

    gtk_main();
    
    return 0;
}

int main(int argc,char* argv[])
{
    return myMain(argc,argv);
}
