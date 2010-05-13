#ifndef __FBCANBUSMESSAGE__
#define __FBCANBUSMESSAGE__

struct FCMSG
{
    int id;
    unsigned char data[8];
    int len;
};

#endif
