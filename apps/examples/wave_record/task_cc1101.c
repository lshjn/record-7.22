
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
struct cc110x_msg  cc1101_msg;

pthread_mutex_t g_TimerMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_TimerConVar	= PTHREAD_COND_INITIALIZER;


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
#define 	SLAVE_ADDR_C 	19

#define 	CMD_QUERYONLINE 		0
#define 	CMD_GETTIMEROFFSET 	1
#define 	CMD_SETTIMEROFFSET 	2

#define 	ACK 		1
#define 	NOACK 		0

#define 	ONLINE 		1
#define 	OFFLINE 	0

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
				
				printf("myreadn: Read <%ld bytes> %s\n", (long)nbytes,rxbuff);
	        }


	      *timeout = false;
	      *ready   = false;
	    }
	  while (read_loops > 0);
	  
	return (max_len-bytes_left) ;	  
}

static void timer2_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
  /* Does nothing in this example except for increment a count of signals
   * received.
   *
   * NOTE: The use of signal handler is not recommended if you are concerned
   * about the signal latency.  Instead, a dedicated, high-priority thread
   * that waits on sigwaitinfo() is recommended.  High priority is required
   * if you want a deterministic wake-up time when the signal occurs.
   */
	pthread_mutex_lock(&g_TimerMutex);
	if((cc1101_msg.online_A == ONLINE)&&(cc1101_msg.online_B == ONLINE)&&(cc1101_msg.online_C == ONLINE))
	{
		//cc1101_msg.Ack = true;
		//cc1101_msg.cmd = CMD_GETTIMEROFFSET;
	}
	else
	{
		cc1101_msg.Ack = true;
		cc1101_msg.cmd = CMD_QUERYONLINE;
	}
	pthread_cond_signal(&g_TimerConVar);
	pthread_mutex_unlock(&g_TimerMutex);
}

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
						int j = 0;
						for(j=0;j<3;j++)
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

/****************************************************************************
 * master_cc1101
 * liushuhe
 * 2018.01.29
 ****************************************************************************/
int master_cc1101(int argc, char *argv[])
{
	task_create("slave_cc1101", 100,2048, slave_cc1101,NULL);

	struct timeval timeout;
	fd_set 	rfds;	
	int cnt=0;

	int fd;
	int i=0;
	int data=0;
    char rxbuff[255];
	int  iRet = 0;
	int  iBytes = 0;

	int timeout_f = 0;
	int ready_f = 0;

    boardctl(BOARDIOC_433_PWRON, 0);

	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
	}



	while(1)
	{
		FD_ZERO(&rfds);											
		FD_SET(fd, &rfds);
		timeout.tv_sec = 2;
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
				//int timercnt1=0;
				//ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt1));
				//printf("sleep--timer2--cnt<%d>\n",timercnt1);
			}
		}
		else if(iRet == 0)
		{
			//cnt = 0;
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
				cnt++;
				//usleep(50*1000L);                                     //sleep 100ms
				memset(rxbuff, 0, sizeof(rxbuff));

				iBytes = myreadn(fd,rxbuff,sizeof(rxbuff),&timeout_f,&ready_f);

                /****************************************************************/
				//Parse rcv data
				//cmdtype
				switch(rxbuff[0])
				{
					case CMD_QUERYONLINE:
							{
								//dst addr
								if(rxbuff[1] == MASTER_ADDR)
								{
									//src addr
									switch(rxbuff[2])
									{
										case SLAVE_ADDR_A:
												cc1101_msg.online_A = ONLINE;
												cc1101_msg.Ack 	 = ACK;
												
											break;
										case SLAVE_ADDR_B:
												cc1101_msg.online_B = ONLINE;
												cc1101_msg.Ack 	 = ACK;
											break;
										case SLAVE_ADDR_C:
												cc1101_msg.online_C = ONLINE;
												cc1101_msg.Ack 	 = ACK;
											break;
									}
									if((cc1101_msg.online_A == ONLINE)&&
										(cc1101_msg.online_B == ONLINE)&&
										(cc1101_msg.online_C == ONLINE))
									{
										pthread_mutex_lock(&g_TimerMutex);
										cc1101_msg.Ack = true;
										cc1101_msg.cmd = CMD_GETTIMEROFFSET;
										pthread_cond_signal(&g_TimerConVar);
										pthread_mutex_unlock(&g_TimerMutex);
									}
								}
							}
						break;
					case CMD_GETTIMEROFFSET:
							{
								//dst addr
								if(rxbuff[1] == MASTER_ADDR)
								{
									//src addr
									switch(rxbuff[2])
									{
										case SLAVE_ADDR_A:
												//cc1101_msg.cur_balladdr = SLAVE_ADDR_A;
												cc1101_msg.GetTime = rxbuff[4]<<24|rxbuff[5]<<16|rxbuff[6]<<8|rxbuff[7];
												cc1101_msg.Ack 	 = ACK;
											break;
										case SLAVE_ADDR_B:
												//cc1101_msg.cur_balladdr = SLAVE_ADDR_B;
												cc1101_msg.GetTime = rxbuff[4]<<24|rxbuff[5]<<16|rxbuff[6]<<8|rxbuff[7];
												cc1101_msg.Ack 	 = ACK;
											break;
										case SLAVE_ADDR_C:
												//cc1101_msg.cur_balladdr = SLAVE_ADDR_C;
												cc1101_msg.GetTime = rxbuff[4]<<24|rxbuff[5]<<16|rxbuff[6]<<8|rxbuff[7];
												cc1101_msg.Ack 	 = ACK;
											break;
									}
								}
							}
						break;
					case CMD_SETTIMEROFFSET:
						break;
						
				}


				
                /****************************************************************/
				
				for(i=0;i<iBytes;i++)
				{
					printf("<%d>=%d\n",i,rxbuff[i]);
				}
					
			    tcflush(fd, TCIFLUSH);
			}
		}
	}    

  return 0;
}









