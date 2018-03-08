#include <sys/wait.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
#include "netutils/netlib.h"
#include <debug.h>
#include <errno.h>
#include "task_cc1101.h"

#define ETH0_DEVNAME "eth0"
#define CONFIG_ETH0_MACADDR   0x00e0deadbeef
#define CONFIG_ETH0_IPADDR    0xc0a803f1      //192.168.3.241
#define CONFIG_ETH0_GATEWAY   0xc0a803f0		//192.168.3.240
#define CONFIG_ETH0_NETMASK   0xffffff00     //255.255.255.0

#define CONFIG_SVER_IPADDR    0xc0a803f0      //server IP :192.168.1.240
#define CONFIG_SVER_PORT      4001      		//server port :5000


#define BUFSIZE   50

in_addr_t server_ipv4 = HTONL(CONFIG_SVER_IPADDR);


void  udp_clisend(int sock)
{
	int			ret;
	char		sendbuf[1024] = {0};

	struct		sockaddr_in servaddr;
	memset(&servaddr,0,sizeof(servaddr));
	servaddr.sin_family			= AF_INET;
	servaddr.sin_port			= htons(5000);
	servaddr.sin_addr.s_addr	= inet_addr("192.168.3.240");
	
	int ch = 0x30;
	int i = 0;
	for (i = 0; i < BUFSIZE; i++ )
	{
		sendbuf[i] = ch+i;
	}

		
    sendto(sock,sendbuf,strlen(sendbuf),0,(struct sockaddr*)&servaddr,sizeof(servaddr));

	memset(sendbuf,0,sizeof(sendbuf));
	close(sock);
}


void udp_client()
{
	
    int  sock;
	
    if((sock = socket(PF_INET,SOCK_DGRAM,0)) < 0)
   	{
		printf("socket error \n");
	}

	udp_clisend(sock);
	
    return 0;  
}


void tcp_client(void)
{
  int fd = -1;
  struct sockaddr_in addr;
  
  char * out = NULL;
  
  printf("tcp test\n");
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    {
      printf("scoket error\n");
      return -1;
    }
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(4001);
  addr.sin_addr.s_addr = inet_addr("192.168.3.240");
  if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      printf("connect error\n");
      return -1;
    }

  out = (char *)&Reportdata_II[0][0];
  
  write(fd, out, sizeof(Reportdata_II));
  printf("connect ok\n");
  
  sleep(3);
  close(fd);
  
  return;
}

static void eth0_set_macaddr(void)
{
	uint8_t mac[IFHWADDRLEN];
	mac[0] = (CONFIG_ETH0_MACADDR >> (8 * 5)) & 0xff;
	mac[1] = (CONFIG_ETH0_MACADDR >> (8 * 4)) & 0xff;
	mac[2] = (CONFIG_ETH0_MACADDR >> (8 * 3)) & 0xff;
	mac[3] = (CONFIG_ETH0_MACADDR >> (8 * 2)) & 0xff;
	mac[4] = (CONFIG_ETH0_MACADDR >> (8 * 1)) & 0xff;
	mac[5] = (CONFIG_ETH0_MACADDR >> (8 * 0)) & 0xff;
	netlib_setmacaddr(ETH0_DEVNAME, mac);
}

static void eth0_set_ipaddrs(void)
{
	struct in_addr addr;
	addr.s_addr = HTONL(CONFIG_ETH0_IPADDR);
	netlib_set_ipv4addr(ETH0_DEVNAME, &addr);

	addr.s_addr = HTONL(CONFIG_ETH0_GATEWAY);
	netlib_set_dripv4addr(ETH0_DEVNAME, &addr);

	addr.s_addr = HTONL(CONFIG_ETH0_NETMASK);
	netlib_set_ipv4netmask(ETH0_DEVNAME, &addr);
}

static void eth0_bringup(void)
{
	netlib_ifup(ETH0_DEVNAME);
}



static void eth0_configure(void)
{
	eth0_set_macaddr();
	eth0_set_ipaddrs();
	eth0_bringup();
}


int report_tcp(int argc, FAR char *argv[])
{
	sleep(2);
	//eth0_configure();
	sleep(2);
     printf("eth0 up....\n");
	while(1)
	{
     	printf("wait report signal....\n");

		pthread_mutex_lock(&g_TcpMutex);
		pthread_cond_wait(&g_TcpConVar, &g_TcpMutex);   //pthread_cond_wait 会先解除g_AdcMutex锁，再阻塞在条件变量
		pthread_mutex_unlock(&g_TcpMutex);
	
		tcp_client();
	}

  return EXIT_SUCCESS;
}

