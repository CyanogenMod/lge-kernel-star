
/* Example for applcation */

#ifndef __TEST_H__
#define __TEST_H__

#include "raontv.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/poll.h>
//	#include <fcntl.h> 
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include "mtv250_ioctl.h"



#define MAX_NUM_TS_BUF	2048

#if defined(RTV_IF_SPI)
	//#define MAX_READ_TSP_SIZE	(188*16)
//	#define MAX_READ_TSP_SIZE	(188*8)
	#define MAX_READ_TSP_SIZE	(188*24)
#else
	#define MAX_READ_TSP_SIZE	(188*8)
#endif

typedef struct
{
	int len;    
	unsigned char msc_buf[MAX_READ_TSP_SIZE];
} MTV_TS_PKT_INFO; 



#ifdef __linux__ /* Linux application */
struct tsp_queue
{
	pthread_mutex_t mutex;
	int count;
	int front;
	int rear;
	unsigned int buf_addr[MAX_NUM_TS_BUF];
	char name[32];
};

struct mrevent {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    volatile bool triggered;
};
static inline void mrevent_init(struct mrevent *ev) 
{
    pthread_mutex_init(&ev->mutex, 0);
    pthread_cond_init(&ev->cond, 0);
    ev->triggered = false;
}
static inline void mrevent_trigger(struct mrevent *ev) 
{
    pthread_mutex_lock(&ev->mutex);
    ev->triggered = true;
    pthread_cond_signal(&ev->cond);
    pthread_mutex_unlock(&ev->mutex);
}
static inline void mrevent_reset(struct mrevent *ev) 
{
    pthread_mutex_lock(&ev->mutex);
    ev->triggered = false;
    pthread_mutex_unlock(&ev->mutex);
}
static inline void mrevent_wait(struct mrevent *ev) 
{
     pthread_mutex_lock(&ev->mutex);
     while (!ev->triggered)
         pthread_cond_wait(&ev->cond, &ev->mutex);
     pthread_mutex_unlock(&ev->mutex);
}
#endif /* #ifdef __linux __ */


#endif 



