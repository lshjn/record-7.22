/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

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
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/
 #define TX_BUF 60
#  define CONFIG_EXAMPLES_TIMER_DEVNAME "/dev/timer2_gps"
#  define CONFIG_EXAMPLES_TIMER_INTERVAL 1000000
#  define CONFIG_EXAMPLES_TIMER_SIGNO 17

//readn
int myreadn(int fd,char* rxbuff[],int max_len,int * timeout,int * ready)
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

int timer2(int argc, char *argv[])
{
	struct timer_notify_s notify;
	int fd_timer;
	int ret;

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
	
	notify.arg   = NULL;
	notify.pid   = getpid();
	notify.signo = CONFIG_EXAMPLES_TIMER_SIGNO;
	
	ret = ioctl(fd_timer, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
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
				int timercnt1=0;
				
				ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt1));
				printf("sleep--timer2--cnt<%d>\n",timercnt1);
		usleep(1000*1000);                                     //sleep 100ms
	}
    
	
  return 0;

}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	//task_create("timer2", 100,2048, timer2,NULL);


	struct timeval timeout;
	fd_set 	rfds;	

	struct timer_notify_s notify1;
	
	int fd_timer;
	int ret;



	int cnt=0;

    boardctl(BOARDIOC_TIME2_PPS_INIT, 0);
    boardctl(BOARDIOC_433_PWRON, 0);

    printf("Hello, World!!\n");
	
	int fd;
	
	int i=0;
	int data=0;
    char txbuff[TX_BUF]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    char rxbuff[255];
	int  	iRet = 0;
	int  	iBytes = 0;

	int timeout_f = 0;
	int ready_f = 0;



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
	
	
	if(strcmp(argv[1],"t") == 0)
	{
		while(1)
		{
			sleep(5);
			printf("write  data to cc1101\n");
		
			for(i=0;i<TX_BUF;i++)
			{
				txbuff[i]=i;
				//printf("<%d>=%d\n",i,txbuff[i]);				
			}
			
		    iBytes = write(fd ,txbuff,sizeof(txbuff));
			if(iBytes == -1)
			{
				printf("Error:write  Data to cc1101\n");
			}

			
		}
	}
	else if(strcmp(argv[1],"r") == 0)
	{	
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
					int timercnt1=0;
					ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt1));
					printf("sleep--timer2--cnt<%d>\n",timercnt1);
				}
			}
			else if(iRet == 0)
			{
				//cnt = 0;
				timeout_f = true;
			}
			else
			{
				if (ret != 1)
				{
					printf("my_read: ERROR poll reported: %d\n", ret);
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
					
					int timercnt2=0;
					ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt2));

					printf("num%d,timercnt2=%d\n",cnt,timercnt2);

					for(i=0;i<iBytes;i++)
					{
						printf("<%d>=%d\n",i,rxbuff[i]);
					}
						
				    tcflush(fd, TCIFLUSH);
				}
			}
		}    
	}

  return 0;
}
