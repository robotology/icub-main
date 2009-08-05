

void on_menu_window_activation		(GtkWidget * widget, gpointer user_data);

void on_file_open_activate			(GtkMenuItem * menuitem, gpointer user_data);

void on_file_save_activate			(GtkMenuItem * menuitem, gpointer user_data);

void on_file_quit_activate			(GtkMenuItem * menuitem, gpointer user_data);

void update_preview_cb (GtkFileChooser * file_chooser, gpointer data);

void on_image_window_activation (GtkWidget * widget, gpointer user_data);

gboolean delete_event (GtkWidget * widget, GdkEvent * event, gpointer data);

gboolean on_darea_expose(GtkWidget *widget,	GdkEventExpose *event, gpointer user_data);

gboolean on_ext_changed (GtkWidget *widget,gpointer   data);

void on_tools_c2lp_activate(GtkMenuItem * menuitem, 
					   gpointer user_data);

void on_tools_lp2c_activate(GtkMenuItem * menuitem,
					   gpointer user_data);

void on_tools_toolbox_activate(	GtkMenuItem * menuitem, 
								gpointer user_data);

void c2lp_OK_clicked (GtkButton * button, gpointer user_data);

void lp2c_OK_clicked (GtkButton * button, gpointer user_data);

gboolean delete_dialog(	GtkWidget *widget,
						GdkEvent  *event,
						gpointer   data );

gboolean cartesian_expose (	GtkWidget * da,
							GdkEventExpose * event,
							gpointer data);

gboolean remapped_expose (	GtkWidget * da,
							GdkEventExpose * event,
							gpointer data);

gboolean logpolar_expose (	GtkWidget * da,
							GdkEventExpose * event,
							gpointer data);

gboolean logpolar2_expose (	GtkWidget * da,
							GdkEventExpose *event,
							gpointer data);

gboolean mouse_button_press_event (	GtkWidget * widget,
									GdkEventButton * event,
									gpointer data);

gboolean delete_toolbox	(	GtkWidget *widget,
							GdkEvent  *event,
							gpointer   data );

gdouble on_zoomIn_button_clicked(GtkWidget *widget, gpointer user_data);
gdouble on_zoomOut_button_clicked(GtkWidget *widget, gpointer user_data);
void on_mirror_button_clicked(GtkWidget *widget, gpointer user_data);
void on_flip_button_clicked(GtkWidget *widget, gpointer user_data);
