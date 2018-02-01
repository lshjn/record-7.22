
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
#define CONFIG_EXAMPLES_TIMER_INTERVAL 1000000
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
	                  printf("myreadn: ERROR no read data\n");
	                }
	            }
	          else if (errno != EINTR)
	            {
	              printf("myreadn: read failed: %d\n", errno);
	            }
	          nbytes = 0;
			  read_loops--;
	        }
	      else
	        {
	          if (*timeout)
	            {
	              printf("myreadn: ERROR? Poll timeout, but data read\n");
	              printf("               (might just be a race condition)\n");
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

/*
int slave_cc1101(int argc, char *argv[])
{
	struct sigaction act;
	struct timer_notify_s notify1;
	
	int 	timercnt=0;
	int 	ret;
	int 	fd;
	int 	fd_timer;
	
    uint8_t 	txbuff[TX_BUF];
	int  	iBytes = 0;
	
    boardctl(BOARDIOC_TIME2_PPS_INIT, 0);
	
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

	ret = ioctl(fd_timer, TCIOC_SETTIMEOUT, CONFIG_EXAMPLES_TIMER_INTERVAL);
	if (ret < 0)
	{
		printf("ERROR: Failed to set the timer interval: %d\n", errno);
	}
	
	act.sa_sigaction = timer2_sighandler;
	act.sa_flags     = SA_SIGINFO;

	(void)sigfillset(&act.sa_mask);
	(void)sigdelset(&act.sa_mask, CONFIG_EXAMPLES_TIMER_SIGNO);

	ret = sigaction(CONFIG_EXAMPLES_TIMER_SIGNO, &act, NULL);
	if (ret != OK)
	{
		printf("ERROR: Fsigaction failed: %d\n", errno);
	}
	
	notify1.arg   = NULL;
	notify1.pid   = getpid();
	notify1.signo = CONFIG_EXAMPLES_TIMER_SIGNO;
	
	ret = ioctl(fd_timer, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify1));
	if (ret < 0)
	{
		printf("ERROR: Failed to set the timer handler: %d\n", errno);
	}
  
	ret = ioctl(fd_timer, TCIOC_START, 0);
	if (ret < 0)
	{
		printf("ERROR: Failed to start the timer: %d\n", errno);
	}



	while(1)
	{
	      //iBytes = write(fd, rxbuff, TX_BUF);
	      
		pthread_mutex_lock(&g_TimerMutex);

		while(cc1101_msg.Ack != true)
		{
			pthread_cond_wait(&g_TimerConVar, &g_TimerMutex);
		}
		switch(cc1101_msg.cmd)
		{
			case CMD_QUERYONLINE:
					{
						int i = 0;
						for(i=0;i<3;i++)
						{
							switch(i)
							{
								case 0:
									txbuff[0] = cc1101_msg.cmd;
									txbuff[1] = SLAVE_ADDR_A;
									txbuff[2] = MASTER_ADDR;						
									break;
								case 1:
									txbuff[0] = cc1101_msg.cmd;
									txbuff[1] = SLAVE_ADDR_B;
									txbuff[2] = MASTER_ADDR;						
									break;
								case 2:
									txbuff[0] = cc1101_msg.cmd;
									txbuff[1] = SLAVE_ADDR_C;
									txbuff[2] = MASTER_ADDR;						
									break;
							}
							
							cc1101_msg.Ack = NOACK;
							
							iBytes = write(fd, txbuff, 3);
							while(cc1101_msg.Ack == NOACK)
							{
								int loop_i = 0;
								if(loop_i++ > 10)
								{
									break;
								}
								usleep(5*1000);
								if(cc1101_msg.Ack == ACK)
								{
									break;
								}
								iBytes = write(fd, txbuff, 3);
							}
						}				
					}
				break;
			case CMD_GETTIMEROFFSET:
					{
						int j = 2;
						//for(j=0;j<3;j++)
						{
							switch(j)
							{
								case 0:
									txbuff[0] = cc1101_msg.cmd;
									txbuff[1] = SLAVE_ADDR_A;
									txbuff[2] = MASTER_ADDR;						
									break;
								case 1:
									txbuff[0] = cc1101_msg.cmd;
									txbuff[1] = SLAVE_ADDR_B;
									txbuff[2] = MASTER_ADDR;						
									break;
								case 2:
									txbuff[0] = cc1101_msg.cmd;
									txbuff[1] = SLAVE_ADDR_C;
									txbuff[2] = MASTER_ADDR;						
									break;
							}
							
							cc1101_msg.Ack = NOACK;
							
							iBytes = write(fd, txbuff, 3);
							while(cc1101_msg.Ack == NOACK)
							{
								int loop_j = 0;
								if(loop_j++ > 10)
								{
									break;
								}
								usleep(5*1000);
								if(cc1101_msg.Ack == ACK)
								{
									break;
								}
								iBytes = write(fd, txbuff, 3);
							}
							//set time
							ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt));
							cc1101_msg.SetTime = timercnt - cc1101_msg.GetTime;
							txbuff[0] = CMD_SETTIMEROFFSET;
							txbuff[3] = 0;
							txbuff[4] = (uint8_t)cc1101_msg.SetTime>>24;
							txbuff[5] = (uint8_t)(cc1101_msg.SetTime>>16&0x00ff);
							txbuff[6] = (uint8_t)(cc1101_msg.SetTime>>8&0x0000ff);						
							txbuff[7] = (uint8_t)cc1101_msg.SetTime;		
							iBytes = write(fd, txbuff, 8);
						}				
					}				
				break;
				
		}
		cc1101_msg.Ack = false;
		pthread_mutex_unlock(&g_TimerMutex);
	}
    
  return 0;
}
*/


int GetmsgStartaddrAndLen(char *databuff,int maxlen,int **start_addr)
{
	char *ptr = (char*)databuff;
	int  msglen = maxlen;
	int  datalen = 0;
	int  rlen = 0;

	int i = 0;
    for(i=0;i < msglen;i++)
    {
		//printf("a<%d>=%x\n",i,*ptr);
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
	//printf("a<%d>=%x\n",i,datalen);
	
	i = 0;
	while(msglen > 0)
	{
		//printf("b<%d>=%x\n",i,*ptr);
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
		i++;
	}
	
	return rlen;
}




static void timer2_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
   systick++;
}



/****************************************************************************
 * master_cc1101
 * liushuhe
 * 2018.01.29
 ****************************************************************************/
int master_cc1101(int argc, char *argv[])
{
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



	while(1)
	{
		FD_ZERO(&rfds);											
		FD_SET(fd, &rfds);
		timeout.tv_sec = 5;
		timeout.tv_usec = 0;			
		iRet = select(fd+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout
		if (iRet < 0) 
		{
			//add by liushuhe 2018.01.19
			//error unless timer2 int
			if(errno != EINTR)
			{
				printf("select error!!!<%d>\n",errno);
			}
			else
			{
				int timercnt=0;
				ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt));
				printf("time_cnt<%d>\n",timercnt);
			}
		}
		else if(iRet == 0)
		{
			timeout_f = true;
		}
		else
		{
			if (iRet != 1)
			{
				printf("my_read: ERROR poll reported: %d\n", iRet);
			}
			else
			{
				ready_f = true;
			}
		
			if(FD_ISSET(fd, &rfds)) 
			{
				//usleep(50*1000L);                                     //sleep 100ms
				memset(rxbuff, 0, sizeof(rxbuff));

				rBytes = myreadn(fd,rxbuff,sizeof(rxbuff),&timeout_f,&ready_f);
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
										P_cc1101_msg_rx = rxbuff;
										P_cc1101_msg_tx->start_flag	= MSG_START;
										P_cc1101_msg_tx->msglen		= sizeof(cc110x_timemsg);
										P_cc1101_msg_tx->type			= CMD_READTIME;
										P_cc1101_msg_tx->dist			= P_cc1101_msg_rx->src;
										P_cc1101_msg_tx->src			= P_cc1101_msg_rx->dist;
										P_cc1101_msg_tx->second		= systick;
										
										int timercnt_us=0;
										ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt_us));
										
										P_cc1101_msg_tx->us			= timercnt_us - P_cc1101_msg_rx->us;
										P_cc1101_msg_tx->endflag		= MSG_END;
										
										wBytes = write(fd, (char *)P_cc1101_msg_tx, sizeof(cc110x_timemsg));
										
										printf("s<%d>\n",P_cc1101_msg_tx->dist);
										/*
										int n=0;
										char* ptrr=(char *)P_cc1101_msg_tx;
										for(n=0;n<sizeof(cc110x_timemsg);n++)
										{
											printf("s-><%d>=%x,rBytes=%d,msg_datalen=%d\n",n,*ptrr++,rBytes,msg_datalen);
										}
										*/
									}
								break;
						}
					}					
				}
				while(rBytes >0);

			    tcflush(fd, TCIFLUSH);
			}
		}
	}    

  return 0;
}









