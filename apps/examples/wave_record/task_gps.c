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
#include <errno.h>
#include <debug.h>
#include "task_gps.h"



// 28,9
//in:ubx+name+RTCM out:NMEA 115200
char bUBX_NMEA_115200[28] = {0xB5 ,0x62 ,0x06 ,0x00 ,0x14 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,0xD0,
						 	0x08 ,0x00 ,0x00 ,0x00 ,0xC2 ,0x01 ,0x00 ,0x07 ,0x00 ,0x02 ,0x00,
							0x00 ,0x00 ,0x00 ,0x00 ,0xBF ,0x78};
// 68,BD+GPS
char bGNSS_BDGPS[68]={0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,
					0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,0x01,0x01,
					0x03,0x00,0x01,0x00,0x01,0x01,0x02,0x00,0x00,0x00,
					0x00,0x00,0x00,0x01,0x03,0x08,0x10,0x00,0x01,0x00,
					0x01,0x01,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
					0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01,0x06,0x08,
					0x0E,0x00,0x00,0x00,0x01,0x01,0x19,0xF1};          
// 28,NAME 4.0
char bNMEA40[28]={0xB5,0x62,0x06,0x17,0x14,0x00,0x00,0x40,0x00,0x01,  
				0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x00,  
				0x00,0x00,0x00,0x00,0x00,0x00,0x74,0x3F};            

// 16,RMC
char bNMEA_RMC[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x01,0x01,0x00,0x00,0x00,0x05,0x48};

// 16,GGA OFF
char bNMEA_GGA_OFF[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x23};            
// 16,GGA ON
char bNMEA_GGA_ON[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x2C};  

// 16,GSA OFF
char bNMEA_GSA_OFF[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31};            
// 16,GSA ON
char bNMEA_GSA_ON[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x01,0x01,0x00,0x00,0x00,0x03,0x3A};            
	
// 16,GSV OFF
char bNMEA_GSV_OFF[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38};            
// 16,GSV ON
char bNMEA_GSV_ON[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x01,0x01,0x00,0x00,0x00,0x04,0x41};            

// 16,GLL OFF
char bNMEA_GLL_OFF[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A};            
// 16,GSV ON
char bNMEA_GLL_ON[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x01,0x01,0x00,0x00,0x00,0x02,0x33 };            
 

static	bool	g_gps_started;
static	int	    fd_gps;

/****************************************************************************
 * Uart_init
 * liushuhe
 * 2017.11.07
 ****************************************************************************/
int Uart_init(int baudrate)
{	
	struct termios newtio;

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD; 
	newtio.c_cflag |= CS8;	      						// 8 bit
	newtio.c_cflag &= ~CSTOPB;    						// 1 stop bit
	newtio.c_cflag &= ~PARENB;    						// none parity
	cfsetispeed(&newtio, baudrate);

	printf("Uart:%s -> %d, N, 8, 1 \n",CONFIG_EXAMPLES_GPS_DEVPATH,baudrate);
	
	tcflush(fd_gps, TCIFLUSH);
   	if ((tcsetattr(fd_gps, TCSANOW, &newtio)) != 0)
	{  
          perror("set device param error\n");
	      exit(1);		
	}
	
	printf("\nOpen %s ok\n",CONFIG_EXAMPLES_GPS_DEVPATH);		
	
	return 0;
} 	

/****************************************************************************
 * Ublox_init
 * liushuhe
 * 2017.11.07
 ****************************************************************************/
void Ublox_init(int fd)
{
	int iBytes;
	//BD+GPS
	iBytes = write(fd, bGNSS_BDGPS, sizeof(bGNSS_BDGPS));
	if(iBytes == -1)
	{
		printf("Error:bGNSS_BDGPS\n");
	}
	sleep(1);
	//NAME 4.0
	iBytes = write(fd, bNMEA40, sizeof(bNMEA40));
	if(iBytes == -1)
	{
		printf("Error:bNMEA40\n");
	}
	sleep(1);
	//RMC
	iBytes = write(fd, bNMEA_RMC, sizeof(bNMEA_RMC));
	if(iBytes == -1)
	{
		printf("Error:bNMEA_RMC\n");
	}
	sleep(1);
	//GGA OFF
	iBytes = write(fd, bNMEA_GGA_OFF, sizeof(bNMEA_GGA_OFF));
	if(iBytes == -1)
	{
		printf("Error:bNMEA_GGA_OFF\n");
	}
	sleep(1);
	//GSA OFF
	iBytes = write(fd, bNMEA_GSA_OFF, sizeof(bNMEA_GSA_OFF));
	if(iBytes == -1)
	{
		printf("Error:bNMEA_GSA_OFF\n");
	}
	sleep(1);
	//GSV OFF
	iBytes = write(fd, bNMEA_GSV_OFF, sizeof(bNMEA_GSV_OFF));
	if(iBytes == -1)
	{
		printf("Error:bNMEA_GSV_OFF\n");
	}
	sleep(1);
	//GLL OFF
	iBytes = write(fd, bNMEA_GLL_OFF, sizeof(bNMEA_GLL_OFF));
	if(iBytes == -1)
	{
		printf("Error:bNMEA_GLL_OFF\n");
	}
}
/****************************************************************************
 * Ublox_init
 * liushuhe
 * 2017.11.07
 ****************************************************************************/
void Ublox_baud_set(int fd)
{
	int iBytes;
	//in:ubx+name+RTCM out:NMEA 115200
	iBytes = write(fd, bUBX_NMEA_115200, sizeof(bUBX_NMEA_115200));
	if(iBytes == -1)
	{
		printf("Error:bGNSS_BDGPS\n");
	}
	sleep(1);
}

/****************************************************************************
 * master_gps
 * liushuhe
 * 2017.11.07
 ****************************************************************************/
int master_gps(int argc, char *argv[])
{
	struct timeval timeout;
	fd_set 	rfds;	
	
	char 	cArray[200];
	int  	iRet = 0;
	int  	iBytes = 0;
	
	g_gps_started = true;


	fd_gps = open(CONFIG_EXAMPLES_GPS_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd_gps < 0)
	{
		int errcode = errno;
		printf("gps: ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_GPS_DEVPATH, errcode);
		goto errout;
	}

    /*
	//gps power
	boardctl(BOARDIOC_GPS_PWRON, 0);
	usleep(100*1000);
	printf("gps power on!!!\n");
    */
	sleep(1);
	
	Uart_init(B9600);
	Ublox_init(fd_gps);
	Ublox_baud_set(fd_gps);
	Uart_init(B115200);
	Ublox_init(fd_gps);
	
	while(1)
	{
		FD_ZERO(&rfds);											
		FD_SET(fd_gps, &rfds);									
		timeout.tv_sec = 60;
		timeout.tv_usec = 0;
		iRet = select(fd_gps+1, &rfds, NULL, NULL, &timeout);  	//recv-timeout

		if (iRet < 0) 
		{
			printf("select error!!!\n");
		}
		else if(iRet == 0)
		{
			//printf("gprs_dev rcv timeout!!!\n");
		}
		else
		{
			if(FD_ISSET(fd_gps, &rfds)) 
			{
				usleep(300*1000L);                                     //sleep 100ms
				memset(cArray, 0, sizeof(cArray));
				iBytes = read(fd_gps, cArray, sizeof(cArray));
				if(iBytes == -1)
				{
					printf("Error:read  Data to gprs\n");
				}
			    tcflush(fd_gps, TCIFLUSH);
				/*************************************************************************************/
				//printf("rcv gps <%d> bytes msg:%s\n",iBytes,cArray);
				/*************************************************************************************/
			}
		}
	}
	    
errout:
  g_gps_started = false;

  printf("gps: Terminating\n");
  return EXIT_FAILURE;
}


