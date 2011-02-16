/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef CALIBRATE_WINDOW_H
#define CALIBRATE_WINDOW_H

#include <gtk/gtk.h>
#include <gtk/gtkmain.h>

void calibrate_click     (GtkButton *button,	gpointer   user_data);
bool calibration_load_v2 (char* filename, int selected_id);

#endif
