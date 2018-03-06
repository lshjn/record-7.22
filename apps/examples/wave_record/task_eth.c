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

#define ETH0_DEVNAME "eth0"
#define CONFIG_ETH0_MACADDR   0x00e0deadbeef
#define CONFIG_ETH0_IPADDR    0xc0a803f1      //192.168.3.241
#define CONFIG_ETH0_GATEWAY   0xc0a803f0		//192.168.3.240
#define CONFIG_ETH0_NETMASK   0xffffff00     //255.255.255.0

#define CONFIG_SVER_IPADDR    0xc0a803f0      //server IP :192.168.1.240
#define CONFIG_SVER_PORT      5000      		//server port :5000


#define BUFSIZE   20

in_addr_t server_ipv4 = HTONL(CONFIG_SVER_IPADDR);


void tcp_client(void)
{
  struct sockaddr_in server;
  char *outbuf;

  int sockfd;
  socklen_t addrlen;
  int nbytessent;
  int nbytesrecvd;
  int totalbytesrecvd;
  int ch;
  int i;

  /* Allocate buffers */

  outbuf = (char*)malloc(BUFSIZE);

  /* Create a new TCP socket */

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      printf("client socket failure %d\n", errno);
    }

  /* Set up the server address */
  memset(&server,0,sizeof(server));
  
  server.sin_family             = AF_INET;
  server.sin_port               = htons(CONFIG_SVER_PORT);
  server.sin_addr.s_addr        = inet_addr("192.168.3.240");
  addrlen                       = sizeof(struct sockaddr_in);

  printf("Connecting to IPv4 Address: %x\n", (unsigned long)server_ipv4);

  if(connect( sockfd, (struct sockaddr*)&server, addrlen) < 0)
  {
	  printf("client: connect failure: %d\n", errno);
  }

  printf("client: Connected\n");

  ch = 0x30;
  for (i = 0; i < BUFSIZE; i++ )
    {
      outbuf[i] = ch+i;
    }

  //for (;;)
    {
		nbytessent = send(sockfd, outbuf, BUFSIZE, 0);
		if (nbytessent < 0)
		{
			printf("client: send failed: %d\n", errno);
		}
		else if (nbytessent != BUFSIZE)
		{
			printf("client: Bad send length=%d: %d of \n",nbytessent, BUFSIZE);
		}

		printf("Sent %d bytes\n", nbytessent);
		usleep(100*1000);
    }
  printf("client: Terminating\n");
  
  close(sockfd);
  free(outbuf);
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
	//sleep(5);
	eth0_configure();
	printf("111ETH:eth0 up.....\n");

	while(1)
	{
		printf("222ETH:eth0 up.....\n");
		sleep(10);
		printf("333ETH:eth0 up.....\n");
		tcp_client();
	}

  return EXIT_SUCCESS;
}

