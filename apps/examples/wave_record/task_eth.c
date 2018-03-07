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
	struct sockaddr_in server;
	char *outbuf;
	static int sockfd = -1;
	
	socklen_t addrlen;
	int nbytessent;
	int nbytesrecvd;
	int totalbytesrecvd;

	if(sockfd < 0)
	{
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			close(sockfd);
			printf("client socket failure %d\n", errno);
			return;
		}

		memset(&server,0,sizeof(server));
		server.sin_family             = AF_INET;
		server.sin_port               = htons(CONFIG_SVER_PORT);
		server.sin_addr.s_addr        = inet_addr("192.168.3.240");
		addrlen                       = sizeof(struct sockaddr_in);

		printf("Connecting to IPv4 Address: %x\n", (unsigned long)server_ipv4);

		if(connect( sockfd, (struct sockaddr*)&server, addrlen) < 0)
		{
			close(sockfd);
			printf("client: connect failure: %d\n", errno);
			return;
		}

		printf("client: Connected OK\n");
	}

  outbuf = (char*)&Reportdata_V[0][0];

  //for (;;)
    {
		//send V
		nbytessent = send(sockfd, outbuf, sizeof(Reportdata_V), 0);
		if (nbytessent < 0)
		{
			close(sockfd);
			sockfd = -1;
			printf("client:V send failed: %d\n", errno);
		}
		else if (nbytessent != sizeof(Reportdata_V))
		{
			printf("client:V send length=%d: total=%d of \n",nbytessent, sizeof(Reportdata_V));
		}
		printf("V:Sent %d bytes\n", nbytessent);
		
		sleep(1);
#if 0		
		//send I
		if(nbytessent == sizeof(Reportdata_V))
		{
			outbuf = (char*)&Reportdata_I[0][0];
			nbytessent = send(sockfd, outbuf, sizeof(Reportdata_I), 0);
			if (nbytessent < 0)
			{
				printf("client:I send failed: %d\n", errno);
			}
			else if (nbytessent != sizeof(Reportdata_I))
			{
				printf("client:I send length=%d: total=%d of \n",nbytessent, sizeof(Reportdata_I));
			}
			printf("I:Sent %d bytes\n", nbytessent);
		}
#endif		
    }
  printf("client: Terminating\n");
  
  //close(sockfd);
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
	eth0_configure();
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

