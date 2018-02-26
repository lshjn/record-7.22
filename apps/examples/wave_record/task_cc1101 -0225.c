
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
struct report_status	summon_status;

pthread_mutex_t g_TimerMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_TimerConVar		= PTHREAD_COND_INITIALIZER;

//#define    REPORTSIZE  1920
#define    REPORTSIZE  2000

uint8_t   ReportIndex[96];
uint8_t   Reportdata[96][40];
uint8_t   Reportdata_V[REPORTSIZE];
uint8_t   Reportdata_I[REPORTSIZE];


/****************************************************************************
 * hello_main
 ****************************************************************************/
#define TX_BUF 60
#define CONFIG_EXAMPLES_TIMER_DEVNAME "/dev/timer2_gps"
#define CONFIG_EXAMPLES_TIMER_INTERVAL (1000000)
#define CONFIG_EXAMPLES_TIMER_SIGNO 17

#define 	CMD_QUERYONLINE 		0
#define 	CMD_GETTIMEROFFSET 	1
#define 	CMD_SETTIMEROFFSET 	2

#define 	MASTER_ADDR 		0X10
#define 	A_ADDR 				0x01
#define 	B_ADDR 				0x02
#define 	C_ADDR 				0x03

#define 	ACK 		1
#define 	NOACK 		0

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
static		uint32_t	msgcmd_type = 0;
//static		uint32_t	summon_ack = false;



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
	char * ptr_src2 = (char *)P_summon_wave_res->data;
	char * ptr_dst = (char *)reportdata;
	
	reportindex[P_summon_wave_res->pos-1] = 'Y';
	
	for(j=0;j<40;j++)
	{
		ptr_dst[(P_summon_wave_res->pos-1)*40 + j] = *ptr_src++;
	}
}


void	ActiveSignal(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	pthread_cond_signal(cond);
	pthread_mutex_unlock(mutex);
}



int thread_wait5ms(pthread_cond_t *cond,pthread_mutex_t *mutex)
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
	ts.tv_nsec += 1000 * 1000 * 5;

	//wait awake
	status = pthread_cond_timedwait(cond, mutex, &ts);
    // pthread_cond_wait会先解除之前的pthread_mutex_lock锁定的mtx，然后阻塞在等待队列里休眠，直到再次被唤醒  
    //（大多数情况下是等待的条件成立而被唤醒，唤醒后，该进程会先锁定先pthread_mutex_lock(&mtx);  
    // 再读取资源 用这个流程是比较清楚的/*unlock-->block-->wait() return-->lock*/  
	if (status != 0)
	{
		if (status == ETIMEDOUT)
		{
			  printf("5ms timed out\n");
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
		printf("pthread_cond_timedwait:rcv ack!\n");
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


static void timer2_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
    systick++;
	*(int*)0x40000024 = 0;
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

int  summon_wave(irqstate_t flags,int fd,struct report_req * P_summon_wave_req,uint8_t req_ballnum)
{
	int  wBytes = 0;	

	P_summon_wave_req->start_flag	= MSG_START;
	P_summon_wave_req->msglen		= sizeof(struct report_req);
	P_summon_wave_req->type		= CMD_SUMMONWAVE;
	switch(req_ballnum)
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
	P_summon_wave_req->second		= systick;
	P_summon_wave_req->pos			= 0;
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
	int ret = 0;
	int fd;

	
	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
	}
	while(1)
	{
		if(msgcmd_type == CMD_PING)
		{
			msgcmd_type = 0;
			//have request..................
			int i=0;
			for(i=0;i<1;i++)
			{
				i=1;
				switch(i)
				{
					case 0:
						 summon_status.req_ballnum	= A_ADDR;
						break;
					case 1:
						 summon_status.req_ballnum	= B_ADDR;
						break;
					case 2:
						 summon_status.req_ballnum	= C_ADDR;
						break;
						
				}
				do
				{
					printf("q\n",summon_status.req_ballnum);
					//主动招波
					memset(ReportIndex,0,sizeof(ReportIndex));
					ret = summon_wave(irqflag,fd,&summon_wave_req,summon_status.req_ballnum);
					if(ret < sizeof(summon_wave_req))
					{
						printf("send summon head fail\n");
					}
					
					ret = thread_wait5ms(&g_TimerConVar, &g_TimerMutex);
				}while(ret == 1); //timeout
				summon_status.req_sendok = 1;
				printf("<%d>req_sendok--------------------- \n",summon_status.req_ballnum);
				//wait data rcv ok
				while(summon_status.res_rcvtimeout != 1)
				{
					usleep(1000);
				}
				printf("<%d>res_rcvtimeout------------------ \n",summon_status.req_ballnum);
				summon_status.req_sendok = 0;
				summon_status.res_rcvtimeout = 0;
#if 1
				sleep(1);
				//data parsing
				int i,j = 0;
				int total = 0;
				for(i=0;i<96;i++)
				{
					if(ReportIndex[i] == 'Y')
					{
						total++;
					}
					printf("<%d>[%c] |",i,ReportIndex[i]);
					if(i%24 == 0)
					{
						printf("\n");
					}
					if(ReportIndex[i] == 'Y')
					{
						printf("\n");
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
						printf("\n");
					}
				}
				printf("rcv total <%d>\n",total);
				memset(ReportIndex,0,sizeof(ReportIndex));
#endif				
			}
		}
		usleep(10);
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
	ret = task_create("report_cc1101", CONFIG_EXAMPLES_CC1101_PRIORITY,CONFIG_EXAMPLES_CC1101_STACKSIZE, report_cc1101,NULL);
	if (ret < 0)
	{
	  int errcode = errno;
	  printf("report_cc1101: ERROR: Failed to start CC1101: %d\n",errcode);
	  return EXIT_FAILURE;
	}

	irqstate_t flags;
  	struct pollfd fds[1];

	struct timer_notify_s notify;
	struct sigaction act;
	//struct timeval timeout;
	//fd_set 	rfds;
	
    char 	rxbuff[255];
    char 	*P_data = NULL;
	int 	fd;
	int 	fd_timer;
	int  	iRet = 0;
	int  	cc1101buf_datalen = 0;
	int  	rBytes = 0;
	int  	wBytes = 0;

	int 	timeout_f = 0;
	int 	ready_f = 0;
	
	struct timespec clock;
				
    boardctl(BOARDIOC_TIME2_PPS_INIT, 0);
    boardctl(BOARDIOC_433_PWRON, 0);

	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
	}

	fd_timer = open(CONFIG_EXAMPLES_TIMER_DEVNAME, O_RDWR);
	if (fd_timer < 0)
	{
		printf("ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_TIMER_DEVNAME, errno);
	}

	iRet = ioctl(fd_timer, TCIOC_SETTIMEOUT, CONFIG_EXAMPLES_TIMER_INTERVAL);
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
	
	iRet = ioctl(fd_timer, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
	if (iRet < 0)
	{
		printf("ERROR: Failed to set the timer handler: %d\n", errno);
	}
  
	iRet = ioctl(fd_timer, TCIOC_START, 0);
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
		//timeout 10ms
		iRet = poll(fds, 1,30);
		if (iRet < 0) 
		{
			//add by liushuhe 2018.01.19
			if(errno != EINTR)
			{
				//printf("select error!!!<%d>\n",errno);
			}
			else
			{
				//int timercnt=0;
				//ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt));
				//printf("time_cnt<%d>\n",timercnt);
			}
		}
		else if(iRet == 0)
		{
			timeout_f = true;	
			if(summon_status.req_sendok == 1)
			{
				summon_status.res_rcvtimeout = 1;
			}
			//clock_gettime(CLOCK_REALTIME, &clock);
			//sleep 10ms
			//printf("tv_nsec<%d>\n",clock.tv_nsec/1000/1000);
		}
		else if ((fds[0].revents & POLLERR) && (fds[0].revents & POLLHUP))
		{
			printf("ERROR: worker poll read Error %d\n", errno);
		}
		else if (fds[0].revents & POLLIN)
		{
			if (iRet != 1)
			{
				//printf("my_read: ERROR poll reported: %d\n", iRet);
			}
			else
			{
				ready_f = true;
			}
			memset(rxbuff, 0, sizeof(rxbuff));
		   
			iRet = ioctl(fd, GETCC1101BUF_BYTES, (unsigned long)&cc1101buf_datalen);
            if(iRet < 0)
            {
				printf("Error:get cc1101 bytes fail!\n");
			}
			rBytes = myreadn(fd,rxbuff,cc1101buf_datalen,&timeout_f,&ready_f);
#if 0
			int k = 0;
			for(k=0;k<rBytes;k++)
			{
				printf("<%d>=[%x]\n",k,rxbuff[k]);
			}

#endif			
			//printf("r<%d>\n",rBytes);
            /****************************************************************/
			int msg_datalen = 0;
			//int loop = 0;
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
									wBytes = synctime(flags,fd,fd_timer,P_cc1101_msg_rx,P_data,P_cc1101_msg_tx);

									if(wBytes != sizeof(cc110x_timemsg))
									{
										printf("send synctime fail!\n");
									}
								}
							break;
						case CMD_PING:
								{
									msgcmd_type = CMD_PING;
								}
							break;
						case CMD_SUMMONWAVE:
								{
									ActiveSignal(&g_TimerConVar, &g_TimerMutex);
									if(P_data[4] == summon_status.req_ballnum)
									{
										GetReportdata(P_data,(uint8_t *)Reportdata,ReportIndex);
									}
								}
							break;		
					}
				}					
			}
			while(rBytes >0);	
			
		    tcflush(fd, TCIFLUSH);
		}
	}    

  return 0;
}






