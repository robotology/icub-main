/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
#ifndef __FBCANBUSMESSAGE__
#define __FBCANBUSMESSAGE__

struct FCMSG
{
    int id;
    unsigned char data[8];
    int len;
};

#endif
