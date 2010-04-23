#ifndef TCP2YARPCONFIG_H
#define TCP2YARPCONFIG_H

#include <list>

#include <cstring>

#ifndef _DEBUG
//#define _DEBUG
#endif
//#define TRACKING

#define MAX_THREAD 4
#define USE_PACKET_SEQUENCE_NUMBER 1

#define SIZE_RECT 15

#define BUFFER_SIZE 32768//65536//8192//100000

//logpolar
#define TAU 1000
#define THRESHOLD 2
#define TMAX 1000

//#define TRACKING
using namespace std;
typedef struct s_AER_struct
{
    int x;
    int y;
    int pol;
    unsigned int ts;
}AER_struct;

typedef struct s_GROUP
{
    int id;
    list<AER_struct> levts;
    int size;
    int c_x;
    int c_y;
}t_GROUP;

#endif //TCP2YARPCONFIG_H
