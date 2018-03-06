
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
//#define HTONS(a)       htons(a)
//#define HTONL(a)       htonl(a)

#define DEVNAME "eth0"

#define BUFSIZE   20
in_addr_t server_ipv4 = HTONL(0xc0a80140);

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

  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      printf("client socket failure %d\n", errno);
    }

  /* Set up the server address */

  server.sin_family             = AF_INET;
  server.sin_port               = HTONS(5000);
  server.sin_addr.s_addr        = (in_addr_t)server_ipv4;
  addrlen                       = sizeof(struct sockaddr_in);

  printf("Connecting to IPv4 Address: %08lx\n", (unsigned long)server_ipv4);

  if (connect( sockfd, (struct sockaddr*)&server, addrlen) < 0)
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
    }
  printf("client: Terminating\n");
  
  close(sockfd);
  free(outbuf);
  return;
}


void nettest_initialize(void)
{
  struct in_addr addr;
  
  uint8_t mac[6];


  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  netlib_setmacaddr(DEVNAME, mac);

  /* Set up our host address */
  addr.s_addr = HTONL(0xc0a80145);
  netlib_set_ipv4addr(DEVNAME, &addr);

  /* Set up the default router address */
  addr.s_addr = HTONL(0xc0a80101);
  netlib_set_dripv4addr(DEVNAME, &addr);

  /* Setup the subnet mask */
  addr.s_addr = HTONL(0xffffff00);
  netlib_set_ipv4netmask(DEVNAME, &addr);

  netlib_ifup(DEVNAME);
}


int report_tcp(int argc, FAR char *argv[])
{

  nettest_initialize();

  tcp_client();

  return EXIT_SUCCESS;
}

