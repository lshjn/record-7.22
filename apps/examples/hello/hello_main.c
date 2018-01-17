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
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/
 #define TX_BUF 60


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{

	struct timeval timeout;
	fd_set 	rfds;	

	int cnt=0;

    boardctl(BOARDIOC_433_PWRON, 0);

    printf("Hello, World!!\n");
	int fd;
	int i=0;
	int data=0;
    char txbuff[TX_BUF]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    char rxbuff[255];
	int  	iRet = 0;
	int  	iBytes = 0;

	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
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
	/*
		while(1)
		{
		    iBytes = read(fd ,rxbuff,sizeof(rxbuff));
			if(iBytes == -1)
			{
				printf("Error:read  Data from cc1101\n");
			}
            else
           	{
				printf("read  data from cc1101\n");
				for(i=0;i<iBytes;i++)
				{
					printf("<%d>=%d\n",i,rxbuff[i]);
				}
			}
			sleep(3);
		}
		*/
	
		while(1)
		{
			printf("select-----\n");
			FD_ZERO(&rfds);											
			FD_SET(fd, &rfds);									
			timeout.tv_sec = 10;
			timeout.tv_usec = 0;
			iRet = select(fd+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout
			cnt++;
			if (iRet < 0) 
			{
				printf("select error!!!\n");
			}
			else if(iRet == 0)
			{
				printf("cc1101 rcv timeout!!!\n");
			}
			else
			{
				if(FD_ISSET(fd, &rfds)) 
				{
					//usleep(300*1000L);                                     //sleep 100ms
					memset(rxbuff, 0, sizeof(rxbuff));
				    iBytes = read(fd ,rxbuff,sizeof(rxbuff));
					if(iBytes == -1)
					{
						printf("Error:read  Data from cc1101\n");
					}
		            else
		           	{
						//printf("read  data from cc1101\n");
						printf("num%d\n",cnt);
			
						
						for(i=0;i<iBytes;i++)
						{
							printf("<%d>=%d\n",i,rxbuff[i]);
						}
						
						//printf("rcv bytes=<%d>......................................................\n",iBytes);
                    /******************************************************************
					//ACK
						usleep(300*1000L);                                     //sleep 100ms
						printf("write ack bytes=<35> to cc1101......................................................\n");
						for(i=0;i<35;i++)
						{
							txbuff[i]=10+i;
							//printf("<%d>=%d\n",i,txbuff[i]);	
						}
						
					    iBytes = write(fd ,txbuff,35);
						if(iBytes == -1)
						{
							printf("Error:write  Data to cc1101\n");
						}
					
					*******************************************************************/

						
					}
				    tcflush(fd, TCIFLUSH);
					/*************************************************************************************/
					//printf("rcv gps <%d> bytes msg:%s\n",iBytes,cArray);
					/*************************************************************************************/
				}
			}
	}
	    
	}

	
  return 0;
}
