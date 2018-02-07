
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


pthread_mutex_t g_TimerMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_TimerConVar		= PTHREAD_COND_INITIALIZER;


/****************************************************************************
 * hello_main
 ****************************************************************************/
#define TX_BUF 60
#define CONFIG_EXAMPLES_TIMER_DEVNAME "/dev/timer2_gps"
#define CONFIG_EXAMPLES_TIMER_INTERVAL (1000000)
#define CONFIG_EXAMPLES_TIMER_SIGNO 17

#define		MASTER_ADDR 	32
#define 	SLAVE_ADDR_A 	17
#define 	SLAVE_ADDR_B 	18
#define 	SLAVE_ADDR_C 	0x03

#define 	CMD_QUERYONLINE 		0
#define 	CMD_GETTIMEROFFSET 	1
#define 	CMD_SETTIMEROFFSET 	2

#define 	ACK 		1
#define 	NOACK 		0

#define 	ONLINE 		1
#define 	OFFLINE 	0


#define 	MSG_START	0xAA
#define 	MSG_END		0x55

#define 	CMD_READTIME    0X01

#define  GETCC1101BUF_BYTES  0x01



static		uint32_t	systick = 0;

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
		}
	}
    //find start_flag fail
	if(0 == rlen)
	{
		return 0;
	}

	datalen = *ptr++;
	
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

static void timer2_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
   systick++;
   //printf("timer2_sighandler!\n");
}

int  synctime(int fd,int fd_timer,cc110x_timemsg * P_cc1101_msg_rx,char * rxbuff,cc110x_timemsg * P_cc1101_msg_tx)
{
	irqstate_t 	flags;
	int 		timercnt_us=0;
	int  		wBytes = 0;	
	static int i = 0;
	i++;

	P_cc1101_msg_rx = rxbuff;
	P_cc1101_msg_tx->start_flag	= MSG_START;
	P_cc1101_msg_tx->msglen		= sizeof(cc110x_timemsg);
	P_cc1101_msg_tx->type			= CMD_READTIME;
	P_cc1101_msg_tx->dist			= P_cc1101_msg_rx->src;
	P_cc1101_msg_tx->src			= P_cc1101_msg_rx->dist;
	P_cc1101_msg_tx->second		= systick;
	ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt_us));
	P_cc1101_msg_tx->us			= P_cc1101_msg_rx->us;
	P_cc1101_msg_tx->endflag		= MSG_END;
	
	flags   = enter_critical_section();
	
	wBytes = write(fd, (char *)P_cc1101_msg_tx, sizeof(cc110x_timemsg));
	
	leave_critical_section(flags);

	printf("<%d>r-%d->%d\n",i,P_cc1101_msg_rx->src,P_cc1101_msg_rx->us);
	printf("<%d>s-%d->%d->%d\n",i,P_cc1101_msg_tx->us,P_cc1101_msg_tx->dist,timercnt_us);
	printf("=====================\n");

	return wBytes;
}

/****************************************************************************
 * master_cc1101
 * liushuhe
 * 2018.01.29
 ****************************************************************************/
int master_cc1101(int argc, char *argv[])
{
	irqstate_t flags;

  	struct pollfd fds[1];
	//task_create("slave_cc1101", 100,2048, slave_cc1101,NULL);

	struct timer_notify_s notify;
	struct timeval timeout;
	struct sigaction act;
	fd_set 	rfds;
	
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

		iRet = poll(fds, 1,5*1000);

        //boardctl(BOARDIOC_TIME2_PPS_DOWN, 0);
		int timer3cnt = 0;
		extern uint32_t  cc1101_timer2_us;

		timer3cnt = *(int*)0x40000024;
		timer3cnt = cc1101_timer2_us;
		//printf("----timer3cnt=%d-----\n",timer3cnt);
		
		if (iRet < 0) 
		{
			//add by liushuhe 2018.01.19
			if(errno != EINTR)
			{
				//printf("select error!!!<%d>\n",errno);
			}
			else
			{
				int timercnt=0;
				//ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt));
				//printf("time_cnt<%d>\n",timercnt);
			}
		}
		else if(iRet == 0)
		{
			timeout_f = true;			
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
			//printf("r<%d>\n",rBytes);
            /****************************************************************/
			int msg_datalen = 0;
			int loop = 0;
			int ptr = 0;
			do
			{
				msg_datalen = GetmsgStartaddrAndLen(&rxbuff[ptr],rBytes,&P_data);
				ptr += msg_datalen;
				rBytes -= msg_datalen;
				if(MSG_START == P_data[0])
				{
					switch(P_data[2])
					{
						case CMD_READTIME:
								{
									wBytes = synctime(fd,fd_timer,P_cc1101_msg_rx,P_data,P_cc1101_msg_tx);

									if(wBytes != sizeof(cc110x_timemsg))
									{
										printf("send synctime fail!\n");
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






