
#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <strings.h>
#include <poll.h>

#include <sys/boardctl.h>
#include <nuttx/timers/timer.h>

#include "task_cc1101.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
cc110x_timemsg   _cc1101_msg_tx; 
cc110x_timemsg * P_cc1101_msg_rx = NULL;
cc110x_timemsg * P_cc1101_msg_tx = &_cc1101_msg_tx;

struct report_req		summon_wave_req;
struct report_res		summon_wave_res;
struct patch_req_head   patch_head;
struct report_status	summon_status;

pthread_mutex_t g_TimerMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_TimerConVar		= PTHREAD_COND_INITIALIZER;
//ping
pthread_mutex_t g_PingMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_PingConVar		= PTHREAD_COND_INITIALIZER;

//#define    REPORTSIZE  1920
#define    REPORTSIZE  2000

uint8_t   PatchIndex[32];
uint8_t   ReportIndex[96];
uint8_t   Reportdata[96][40];
uint8_t   Reportdata_V[REPORTSIZE];
uint8_t   Reportdata_I[REPORTSIZE];


/****************************************************************************
 * hello_main
 ****************************************************************************/
#define TX_BUF 60
#define CONFIG_EXAMPLES_TIMER_DEVNAME 		"/dev/timer2_gps"
#define CONFIG_EXAMPLES_TIMER_CC1101  		"/dev/timer1_cc1101"
#define CONFIG_EXAMPLES_TIMER_INTERVAL 		(1000000)
#define CONFIG_EXAMPLES_TIMERCC1101_INTERVAL (10000)
#define CONFIG_EXAMPLES_TIMER_SIGNO 17
#define CONFIG_EXAMPLES_TIMER_SIGNO_CC1101 18

#define TIMR1_CNT_ADDR 0x40010024
#define TIMR2_CNT_ADDR 0x40000024

#define POLL_TIMEOUT 	12

#define 	CMD_QUERYONLINE 		0
#define 	CMD_GETTIMEROFFSET 	1
#define 	CMD_SETTIMEROFFSET 	2

#define 	MASTER_ADDR 		0X10
#define 	A_ADDR 				0x01
#define 	B_ADDR 				0x02
#define 	C_ADDR 				0x03

#define 	ACK 		1
#define 	NOACK 		0

#define 	FULL 		96
#define 	EMPTY 		0

#define 	ONLINE 		1
#define 	OFFLINE 	0


#define 	MSG_START	0xAA
#define 	MSG_END		0x55

#define 	CMD_READTIME      	 0X01
#define 	CMD_PING	    	 0X02
#define 	CMD_SUMMONWAVE    	 0X03
#define 	CMD_PATCH			 0X04

#define  	GETCC1101BUF_BYTES  0x01

static		uint32_t	systick = 0;
static		uint32_t	patch_systick = 0;
static		uint32_t	patch_pos = 0;

static		uint32_t	msgcmd_type = 0;
static		uint32_t	rcv_timeout = false;

	struct timespec clock1;
	struct timespec clock2;
	struct timespec clock3;
	
//readn
int myreadn(int fd,char* rxbuff,int max_len,int * timeout,int * ready)
{
	ssize_t nbytes = 0;
	int read_loops = 0;
	int bytes_left = 0;
	read_loops = max_len;
	bytes_left = max_len;
	
	/* In any event, read until the pipe is empty */
	  do
	    {
	      nbytes = read(fd, rxbuff, max_len);
	      if (nbytes <= 0)
	        {
	          if (nbytes == 0 || errno == EAGAIN)
	            {
	              if (*ready)
	                {
	                  //printf("myreadn: ERROR no read data\n");
	                }
	            }
	          else if (errno != EINTR)
	            {
	              //printf("myreadn: read failed: %d\n", errno);
	            }
	          nbytes = 0;
			  read_loops--;
	        }
	      else
	        {
	          if (*timeout)
	            {
	              //printf("myreadn: ERROR? Poll timeout, but data read\n");
	              //printf("               (might just be a race condition)\n");
	            }

				read_loops -= nbytes; 
				bytes_left -= nbytes; 
				rxbuff += nbytes; 
				
				//printf("myreadn: Read <%ld bytes> %s\n", (long)nbytes,rxbuff);
	        }


	      *timeout = false;
	      *ready   = false;

	    }
	  while (read_loops > 0);
	  
	return (max_len-bytes_left) ;	  
}

int GetmsgStartaddrAndLen(char *databuff,int maxlen,int **start_addr)
{
	char *ptr = (char*)databuff;
	int  msglen = maxlen;
	int  datalen = 0;
	int  rlen = 0;
	int  nspace = 0;    //invalid bytes
	
	int i = 0;
	
    for(i=0;i < msglen;i++)
    {
		if(MSG_START == *ptr)
		{
			*start_addr = ptr;
			ptr++;
			rlen++;
			msglen--;
			break;
		}
		else
		{
			ptr++;
			nspace++;    //invalid bytes
		}
	}
    //find start_flag fail
	if(0 == rlen)
	{
		while(MSG_START == **start_addr)
		{
			*start_addr +=1;	
		}
		return nspace;
	}

	datalen = *ptr++;
	rlen++;
	while(msglen > 0)
	{
		if(MSG_END == *ptr++)
		{
			rlen++;
			msglen--;
			if(datalen == rlen)
			{
				break;
			}
		}
		else
		{
			rlen++;
			msglen--;
		}
	}
	return rlen;
}

int GetReportdata(char * rxbuff,uint8_t *reportdata,uint8_t *reportindex)
{
	int j = 0;
	struct report_res * P_summon_wave_res = (struct report_res *)rxbuff;
	char * ptr_src = (char *)P_summon_wave_res->data;
	char * ptr_dst = (char *)reportdata;
	char * ptr_index = (char *)reportindex;
	
	ptr_index[P_summon_wave_res->pos-1] = 'Y';
	for(j=0;j<40;j++)
	{
		ptr_dst[(P_summon_wave_res->pos-1)*40 + j] = *ptr_src++;
	}
}


void	ActiveSignal(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	rcv_timeout = true;
	pthread_cond_signal(cond);
	pthread_mutex_unlock(mutex);
}

void waitping(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	while(msgcmd_type != CMD_PING)
	{
		pthread_cond_wait(cond, mutex);   //pthread_cond_wait 会先解除g_AdcMutex锁，再阻塞在条件变量
	}
	msgcmd_type = 0;
	pthread_mutex_unlock(mutex);
}

void waitTimeout(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	while(rcv_timeout != true)
	{
		pthread_cond_wait(cond, mutex);   //pthread_cond_wait 会先解除g_AdcMutex锁，再阻塞在条件变量
	}
	rcv_timeout = false;
	pthread_mutex_unlock(mutex);
}

uint8_t getPatch(uint8_t *patchsum,uint8_t *reportIndex)
{
	uint8_t * ptr_patch = patchsum;
	uint8_t * ptr_report = reportIndex;
	uint8_t lost = 0;
	
	uint8_t i = 0;
	for(i=0;i<96;i++)
	{
		if(ptr_report[i] != 'Y')
		{
			lost++;
			*ptr_patch++ = (i+1);
		}
	}
	return lost;
}

void  calcPatchreport(irqstate_t flags,int fd,uint8_t lost,uint8_t *patchsum,uint8_t *reportIndex,struct patch_req_head* patchhead,uint8_t curball)
{
	int  wBytes = 0;		
	char *p_buf	= NULL; 
	char *p	= NULL; 
	char *phead	= (char*)patchhead; 
	char *patch	= (char*)patchsum; 
	
	//data parsing
	p_buf=p=(char *)malloc((sizeof(struct patch_req_head)+lost + 1));  
	if(p)
	{
		patchhead->start_flag	= MSG_START;
		patchhead->msglen 		= (sizeof(struct patch_req_head)+lost + 1);
		patchhead->type 		= CMD_PATCH;
		switch(curball)
		{
			case A_ADDR:
				 patchhead->dist	= A_ADDR;
				break;
			case B_ADDR:
				 patchhead->dist	= B_ADDR;
				break;
			case C_ADDR:
				 patchhead->dist	= C_ADDR;
				break;			
		}
		patchhead->src 		= MASTER_ADDR;
		patchhead->second 	= patch_systick;
		patchhead->pos 		= patch_pos;
		patchhead->len 		= lost;
		int i;
		//copy head
		for(i=0;i<sizeof(struct patch_req_head);i++)
		{
			*p++ = *phead++;
		}
		//copy patch
		for(i=0;i<lost;i++)
		{
			*p++ = *patch++;
		}
		//copy endflag
		*p = MSG_END;
#if 0		
		char *p_debug	= p_buf; 
		for(i=0;i<(sizeof(struct patch_req_head)+lost + 1);i++)
		{
			
			printf("<%x>\n",*p_debug++);
		}
#endif

		//send patch msg
		flags   = enter_critical_section();
		wBytes = write(fd, (char *)p_buf, (sizeof(struct patch_req_head)+lost + 1));
	    //printf("<%x>wBytes\n",wBytes);
		leave_critical_section(flags);
	}
	else
	{
		printf("malloc fail!\n");
	}
	
	free(p_buf);
	
}
void waitrcvok(uint8_t *sendok,uint8_t *timeout)
{
	//wait data rcv ok
	*sendok = 1;
	while(*timeout != 1)
	{
		usleep(1000*1);
	}
	*sendok  = 0;
	*timeout = 0;
}

int thread_waitnms(pthread_cond_t *cond,pthread_mutex_t *mutex,int nms)
{
	struct timespec ts;
	int status;
	int ret = 0;

	status = pthread_mutex_lock(mutex);
	if (status != 0)
	{
		printf("thread_waiter: ERROR pthread_mutex_lock failed, status=%d\n", status);
	}

	status = clock_gettime(CLOCK_REALTIME, &ts);
	if (status != 0)
	{
		printf("thread_waiter: ERROR clock_gettime failed\n");
	}
	
	//sleep 10ms
	ts.tv_sec += 0;
	ts.tv_nsec += 1000 * 1000 * nms;

	//wait awake
	status = pthread_cond_timedwait(cond, mutex, &ts);
    // pthread_cond_wait会先解除之前的pthread_mutex_lock锁定的mtx，然后阻塞在等待队列里休眠，直到再次被唤醒  
    //（大多数情况下是等待的条件成立而被唤醒，唤醒后，该进程会先锁定先pthread_mutex_lock(&mtx);  
    // 再读取资源 用这个流程是比较清楚的/*unlock-->block-->wait() return-->lock*/  
	if (status != 0)
	{
		if (status == ETIMEDOUT)
		{
			  //printf("5ms timed out\n");
			  ret = 1;
		}
		else
		{
			  printf("thread_waiter: ERROR pthread_cond_timedwait failed, status=%d\n", status);
			  ret = -1;

		}
	}
	else
	{
		printf("sig\n");
	   ret = 2;
	}

	//Release mutex
	status = pthread_mutex_unlock(mutex);
	if (status != 0)
	{
		printf("thread_waiter: ERROR pthread_mutex_unlock failed, status=%d\n", status);
	}
	
	return ret;
}


static void timer1_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
	//*(int*)TIMR1_CNT_ADDR = 0;
	//ActiveSignal(&g_TimerConVar, &g_TimerMutex);
}
static void timer2_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
    systick++;
	*(int*)TIMR2_CNT_ADDR = 0;
}

int  synctime(irqstate_t flags,int fd,int fd_timer,cc110x_timemsg * p_cc1101_msg_rx,char * rxbuff,cc110x_timemsg * p_cc1101_msg_tx)
{
	int 		timercnt_us=0;
	int  		wBytes = 0;	
	static int i = 0;
	i++;

	p_cc1101_msg_rx = (cc110x_timemsg *)rxbuff;
	p_cc1101_msg_tx->start_flag	= MSG_START;
	p_cc1101_msg_tx->msglen		= sizeof(cc110x_timemsg);
	p_cc1101_msg_tx->type			= CMD_READTIME;
	p_cc1101_msg_tx->dist			= p_cc1101_msg_rx->src;
	p_cc1101_msg_tx->src			= p_cc1101_msg_rx->dist;
	p_cc1101_msg_tx->second		= systick;
	//ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt_us));
	//ioctl(fd_timer, TCIOC_SETCOUNTER, (uint32_t)(&timercnt_us));
	
	p_cc1101_msg_tx->us			= p_cc1101_msg_rx->us;
	p_cc1101_msg_tx->endflag		= MSG_END;
	
	flags   = enter_critical_section();
	wBytes = write(fd, (char *)p_cc1101_msg_tx, sizeof(cc110x_timemsg));
    //boardctl(BOARDIOC_TIME2_PPS_DOWN, 0);		
	leave_critical_section(flags);

	printf("<%d>r-%d->%d\n",i,p_cc1101_msg_rx->src,p_cc1101_msg_rx->us);
	printf("<%d>s-%d->%d->%d\n",i,p_cc1101_msg_tx->us,p_cc1101_msg_tx->dist,timercnt_us);
	printf("=====================\n");

	return wBytes;
}

int  summon_wave(irqstate_t flags,int fd,struct report_req * P_summon_wave_req,uint8_t curball)
{
	int  wBytes = 0;	

	P_summon_wave_req->start_flag	= MSG_START;
	P_summon_wave_req->msglen		= sizeof(struct report_req);
	P_summon_wave_req->type		= CMD_SUMMONWAVE;
	switch(curball)
	{
		case A_ADDR:
			 P_summon_wave_req->dist		= A_ADDR;
			break;
		case B_ADDR:
			 P_summon_wave_req->dist		= B_ADDR;
			break;
		case C_ADDR:
			 P_summon_wave_req->dist		= C_ADDR;
			break;			
	}
	P_summon_wave_req->src			= MASTER_ADDR;
	patch_systick = systick;
	patch_pos = 10;
	P_summon_wave_req->second		= patch_systick;
	P_summon_wave_req->pos			= patch_pos;
	P_summon_wave_req->endflag		= MSG_END;
	
	flags   = enter_critical_section();
	wBytes = write(fd, (char *)P_summon_wave_req, sizeof(struct report_req));
	leave_critical_section(flags);

	return wBytes;
}
/****************************************************************************
 * report_cc1101
 * liushuhe
 * 2018.02.23
 ****************************************************************************/
int report_cc1101(int argc, char *argv[])
{
	irqstate_t irqflag;
	uint8_t lost = 0;
	struct timer_notify_s notify;
	struct sigaction act;
	int 	fd_timer1;
	int ret = 0;
	int fd;
	int cnt = 0;

	boardctl(BOARDIOC_TIME2_PPS_INIT, 0);
	
	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
	}


	fd_timer1 = open(CONFIG_EXAMPLES_TIMER_CC1101, O_RDWR);
	if (fd_timer1 < 0)
	{
		printf("ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_TIMER_CC1101, errno);
	}

	ret = ioctl(fd_timer1, TCIOC_SETTIMEOUT, CONFIG_EXAMPLES_TIMERCC1101_INTERVAL);
	if (ret < 0)
	{
		printf("ERROR: Failed to set the timer interval: %d\n", errno);
	}
	
	act.sa_sigaction = timer1_sighandler;
	act.sa_flags     = SA_SIGINFO;

	(void)sigfillset(&act.sa_mask);
	(void)sigdelset(&act.sa_mask, CONFIG_EXAMPLES_TIMER_SIGNO_CC1101);

	ret = sigaction(CONFIG_EXAMPLES_TIMER_SIGNO_CC1101, &act, NULL);
	if (ret != OK)
	{
		printf("ERROR: Fsigaction failed: %d\n", errno);
	}
	
	notify.arg   = NULL;
	notify.pid   = getpid();
	notify.signo = CONFIG_EXAMPLES_TIMER_SIGNO_CC1101;
	
	ret = ioctl(fd_timer1, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
	if (ret < 0)
	{
		printf("ERROR: Failed to set the timer handler: %d\n", errno);
	}

#if 0
	ret = ioctl(fd_timer1, TCIOC_START, 0);
	if (ret < 0)
	{
		printf("ERROR: Failed to start the timer: %d\n", errno);
	}
#endif 

	while(1)
	{
		//have request..................
		//wait signal
		waitping(&g_PingConVar,&g_PingMutex);
		summon_status.summon_status = NOACK;
		//主动招波
		summon_wave(irqflag,fd,&summon_wave_req,summon_status.curball);
		memset(ReportIndex,0,sizeof(ReportIndex));
		memset(Reportdata,0,sizeof(Reportdata));
		memset(Reportdata_V,0,sizeof(Reportdata_V));
		memset(Reportdata_I,0,sizeof(Reportdata_I));
		clock_gettime(CLOCK_REALTIME, &clock1);
		waitTimeout(&g_TimerConVar, &g_TimerMutex);
		waitTimeout(&g_TimerConVar, &g_TimerMutex);
		#if 1
		clock_gettime(CLOCK_REALTIME, &clock2);
		int timeout = (1000000000*(clock2.tv_sec-clock1.tv_sec) + clock2.tv_nsec-clock1.tv_nsec)/1000000;
		printf("%dms\n",timeout);
		if(timeout < (POLL_TIMEOUT +5))
		{
			waitTimeout(&g_TimerConVar, &g_TimerMutex);
			printf("wait\n");
		}
		//printf("1.0tv_nsec=%d\n",1000000000*(clock2.tv_sec-clock1.tv_sec) + clock2.tv_nsec-clock1.tv_nsec);
		//printf("1.1tv_nsec=%d\n",clock1.tv_nsec);
		//printf("1.2tv_nsec=%d\n",clock2.tv_nsec);
		#endif	
		
		//parsing patch
		if(summon_status.summon_status == ACK)
		{
			int try_n = 0;
			int oldlost = 0;
			while(1)
			{
				lost = getPatch((uint8_t *)&PatchIndex,(uint8_t *)&ReportIndex);
				if(lost == 0)
				{
					break;
				}
				if(oldlost == lost)
				{
					if(try_n++ > 5)
					{
						break;
					}
				}
				else
				{
					oldlost = lost;
					try_n = 0;
				}
				if(lost >= 32)
				{
					calcPatchreport(irqflag,fd,32,(uint8_t *)&PatchIndex,(uint8_t *)&ReportIndex,&patch_head,summon_status.curball);			
				}
				else
				{
					calcPatchreport(irqflag,fd,lost,(uint8_t *)&PatchIndex,(uint8_t *)&ReportIndex,&patch_head,summon_status.curball);			
				}
				waitTimeout(&g_TimerConVar, &g_TimerMutex);	
				waitTimeout(&g_TimerConVar, &g_TimerMutex);	
				printf("<%d>N2<%d>\n",summon_status.curball,lost);
			}
		}
		
		//parsing data
		int i= 0;
		int j= 0;
		int total = 0;
		int total2 = 0;
		for(i=0;i<96;i++)
		{		
			if(ReportIndex[i] == 'Y')
			{
				total++;
			}
#if 0
			//printf("<%d>[%c] |",i,ReportIndex[i]);
			if(i%24 == 0)
			{
				//printf("\n");
			}
			if(ReportIndex[i] == 'Y')
			{
				//printf("\n");
				for(j=0;j<40;j++)
				{
					if(j < 20)
					{
						Reportdata_V[i*20 + j] = Reportdata[i][j]; 	
					}
					else
					{
						Reportdata_I[i*20 + j-20] = Reportdata[i][j]; 	
					}
					//printf("[%02x] ",Reportdata[i][j]);
				}
				//printf("\n");
			}
#endif			
		}
		switch(summon_status.curball)
		{
			case A_ADDR:
				 total2 = summon_status.ballA_rcvtotal;
				break;	
			case B_ADDR:
				 total2 = summon_status.ballB_rcvtotal;
				break;	
			case C_ADDR:
				 total2 = summon_status.ballC_rcvtotal;
				break;	
		}
		
		printf("<%d>rcv total1=<%d>,<%d>\n",summon_status.curball,total,total2);
	}
}

/****************************************************************************
 * master_cc1101
 * liushuhe
 * 2018.01.29
 ****************************************************************************/
int master_cc1101(int argc, char *argv[])
{
	int ret =-1;

	irqstate_t flags;
  	struct pollfd fds[1];

	struct timer_notify_s notify;
	struct sigaction act;

	//struct timeval timeout;
	//fd_set 	rfds;
	
    char 	rxbuff[1024*4];
    char 	*P_data = NULL;
	int 	fd;
	int 	fd_timer2;
	int  	iRet = 0;
	int  	poll_ret = 0;
	int  	cc1101buf_datalen = 0;
	int  	rBytes = 0;
	int  	wBytes = 0;

	int 	timeout_f = 0;
	int 	ready_f = 0;
	int 	qqqq = 0;
	
	struct timespec clock;
				
    boardctl(BOARDIOC_TIME2_PPS_INIT, 0);
    boardctl(BOARDIOC_433_PWRON, 0);

	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
	}

	fd_timer2 = open(CONFIG_EXAMPLES_TIMER_DEVNAME, O_RDWR);
	if (fd_timer2 < 0)
	{
		printf("ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_TIMER_DEVNAME, errno);
	}

	iRet = ioctl(fd_timer2, TCIOC_SETTIMEOUT, CONFIG_EXAMPLES_TIMER_INTERVAL);
	if (iRet < 0)
	{
		printf("ERROR: Failed to set the timer interval: %d\n", errno);
	}
	
	act.sa_sigaction = timer2_sighandler;
	act.sa_flags     = SA_SIGINFO;

	(void)sigfillset(&act.sa_mask);
	(void)sigdelset(&act.sa_mask, CONFIG_EXAMPLES_TIMER_SIGNO);

	iRet = sigaction(CONFIG_EXAMPLES_TIMER_SIGNO, &act, NULL);
	if (iRet != OK)
	{
		printf("ERROR: Fsigaction failed: %d\n", errno);
	}
	
	notify.arg   = NULL;
	notify.pid   = getpid();
	notify.signo = CONFIG_EXAMPLES_TIMER_SIGNO;
	
	iRet = ioctl(fd_timer2, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
	if (iRet < 0)
	{
		printf("ERROR: Failed to set the timer handler: %d\n", errno);
	}
  
	iRet = ioctl(fd_timer2, TCIOC_START, 0);
	if (iRet < 0)
	{
		printf("ERROR: Failed to start the timer: %d\n", errno);
	}
	/* Do we already hold the semaphore? */

	while(1)
	{
		memset(fds, 0, sizeof(struct pollfd));
		fds[0].fd       = fd;
		fds[0].events   = POLLIN;
		fds[0].revents  = 0;
		//timeout 15ms
		poll_ret = poll(fds, 1,POLL_TIMEOUT);
		if (poll_ret < 0) 
		{
			//add by liushuhe 2018.01.19
			if(errno != EINTR)
			{
				//printf("select error!!!<%d>\n",errno);
			}
			else
			{
				;
			}
		}
		else if(poll_ret == 0)
		{
			timeout_f = true;	
			ActiveSignal(&g_TimerConVar, &g_TimerMutex);

			//clock_gettime(CLOCK_REALTIME, &clock1);
			//printf("1tv_nsec=%d\n",clock1.tv_nsec);
			//printf("TT\n");
		}
		else if ((fds[0].revents & POLLERR) && (fds[0].revents & POLLHUP))
		{
			printf("ERROR: worker poll read Error %d\n", errno);
		}
		else if (fds[0].revents & POLLIN)
		{
			if (poll_ret != 1)
			{
				//printf("my_read: ERROR poll reported: %d\n", iRet);
			}
			else
			{
				ready_f = true;
			}

			ret = ioctl(fd, GETCC1101BUF_BYTES, (unsigned long)&cc1101buf_datalen);
			{
				memset(rxbuff, 0, sizeof(rxbuff));
			   	rBytes = myreadn(fd,rxbuff,cc1101buf_datalen,&timeout_f,&ready_f);
#if 0
				{
					//int k = 0;
					//for(k=0;k<rBytes;k++)
					//{
					//	printf("<%d>=[%x]\n",k,rxbuff[k]);
					//}
					printf("r<%d>\n",rBytes);
				}
#endif			
	            /****************************************************************/
				int msg_datalen = 0;
				int ptr = 0;
				do
				{
					msg_datalen = GetmsgStartaddrAndLen(&rxbuff[ptr],rBytes,(int **)&P_data);				
					ptr += msg_datalen;
					rBytes -= msg_datalen;
					if(MSG_START == P_data[0])
					{
						switch(P_data[2])
						{
							case CMD_READTIME:
									{
										wBytes = synctime(flags,fd,fd_timer2,P_cc1101_msg_rx,P_data,P_cc1101_msg_tx);
										if(wBytes != sizeof(cc110x_timemsg))
										{
											printf("send synctime fail!\n");
										}
									}
								break;
							case CMD_PING:
									{
										switch(P_data[4])
										{
										#if 1
											//three ball
											case A_ADDR:
												  if((summon_status.ballB_rcvtotal == EMPTY)&&(summon_status.ballC_rcvtotal == EMPTY))
												  {
													  summon_status.curball = A_ADDR;
												  }
												  else if((summon_status.ballB_rcvtotal == FULL)&&(summon_status.ballC_rcvtotal == FULL))
												  {
													  summon_status.curball = A_ADDR;
												  }
												break;
											case B_ADDR:
												  if((summon_status.ballA_rcvtotal == EMPTY)&&(summon_status.ballC_rcvtotal == EMPTY))
												  {
													  summon_status.curball = B_ADDR;
												  }
												  else if((summon_status.ballA_rcvtotal == FULL)&&(summon_status.ballC_rcvtotal == FULL))
												  {
													  summon_status.curball = B_ADDR;
												  }
												break;
											case C_ADDR:
												  if((summon_status.ballA_rcvtotal == EMPTY)&&(summon_status.ballB_rcvtotal == EMPTY))
												  {
													  summon_status.curball = C_ADDR;
												  }
												  else if((summon_status.ballA_rcvtotal == FULL)&&(summon_status.ballB_rcvtotal == FULL))
												  {
													  summon_status.curball = C_ADDR;
												  }
												break;	
										#endif
										#if 0
											//tree ball
											case A_ADDR:
												  if(summon_status.ballB_rcvtotal == EMPTY)
												  {
													  summon_status.curball = A_ADDR;
												  }
												  else if(summon_status.ballB_rcvtotal == FULL)
												  {
													  summon_status.curball = A_ADDR;
													   printf("FULL---------------------------------------B_ADDR\n");
												  }
												break;
											case B_ADDR:
												  if(summon_status.ballA_rcvtotal == EMPTY)
												  {
													  summon_status.curball = B_ADDR;
												  }
												  else if(summon_status.ballA_rcvtotal == FULL)
												  {
													  summon_status.curball = B_ADDR;
													   printf("FULL---------------------------------------A_ADDR\n");
												  }
												break;
										#endif

										
										}
										if(P_data[4] == summon_status.curball)
										{
											pthread_mutex_lock(&g_PingMutex);
											msgcmd_type = CMD_PING;
											pthread_cond_signal(&g_PingConVar);
											pthread_mutex_unlock(&g_PingMutex);
										}
									}
								break;
							case CMD_SUMMONWAVE:

										if(P_data[4] == summon_status.curball)
										{
											switch(summon_status.curball)
											{
												case A_ADDR:
													 summon_status.ballA_rcvtotal++;
													break;	
												case B_ADDR:
													 summon_status.ballB_rcvtotal++;
													break;	
												case C_ADDR:
													 summon_status.ballC_rcvtotal++;
													break;	
											}
											summon_status.summon_status = ACK;
											GetReportdata(P_data,(uint8_t *)&Reportdata,(uint8_t *)&ReportIndex);
										}
									break;
							case CMD_PATCH:
									{
										if(P_data[4] == summon_status.curball)
										{
											switch(summon_status.curball)
											{
												case A_ADDR:
													 summon_status.ballA_rcvtotal++;
													break;	
												case B_ADDR:
													 summon_status.ballB_rcvtotal++;
													break;	
												case C_ADDR:
													 summon_status.ballC_rcvtotal++;
													break;	
											}
											GetReportdata(P_data,(uint8_t *)&Reportdata,(uint8_t *)&ReportIndex);
										}
									}
								break;		
						}
					}
				}
				while(rBytes >0);	
			}
		}
	}    

  return 0;
}






