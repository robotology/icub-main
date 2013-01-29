#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <sys/time.h>
#include <time.h>


typedef struct {
    struct timeval task_time;
    struct timeval period;
} task_period_t;


typedef struct {
    char                thread_name[8]; 
    unsigned long long  start_time_ns;
    unsigned long long  loop_time_ns;
    unsigned long long  elapsed_time_ns;
    unsigned long long  _prev;
    unsigned long        overruns;
} task_time_stat_t;


typedef void (*ptrFunc)(void *);

typedef struct {
    ptrFunc     init;
    ptrFunc     loop;
    ptrFunc     shutdown;
} rt_hook_t;


#ifdef __cplusplus
extern "C" {
#endif

inline uint64_t get_time_ns(void)
{
    uint64_t time_ns;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    return time_ns;
}


/*
void timeval2msec(struct timeval * tv, long &msec);
void timeval2usec(struct timeval * tv, long long &usec);
void timeval2nsec(struct timeval * tv, long long &nsec);

void msecs2timeval(long msecs, struct timeval * tv);
void usecs2timeval(long long usecs, struct timeval * tv);
void nsecs2timeval(long long nsecs, struct timeval * tv);

int  timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1);
void timeval_print(struct timeval *tv);

void task_period_ms_init(task_period_t * timer, long msecs);
void task_period_us_init(task_period_t * timer, long long usecs);
void calc_timeout(struct timeval * timeout, long msecs);
int  task_wait_period(task_period_t * timer, int dedug = 1);

unsigned long long _get_time_ns(void);
void time_stat(task_time_stat_t * time_stat);
*/

#ifdef __cplusplus
}
#endif


#endif
