
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
#include <time.h>
#include <sys/time.h>

#include <sys/boardctl.h>
#include <nuttx/timers/timer.h>

#include "task_cc1101.h"
#include "task_eth.h"
/****************************************************************************
 * Private Data
 ****************************************************************************/
cc110x_timemsg   _cc1101_msg_tx; 
cc110x_timemsg * P_cc1101_msg_rx = NULL;
cc110x_timemsg * P_cc1101_msg_tx = &_cc1101_msg_tx;

struct report_req		summon_wave_req;
struct report_res		summon_wave_res;
struct patch_req_head   patch_head;
struct report_status	summon_status;
struct work_status	    work_sts;

pthread_mutex_t g_TimerMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_TimerConVar		= PTHREAD_COND_INITIALIZER;
//ping
pthread_mutex_t g_PingMutex		= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_PingConVar		= PTHREAD_COND_INITIALIZER;

pthread_mutex_t g_SummonMutex		= PTHREAD_MUTEX_INITIALIZER;

#define    REPORTSIZE  1920
//#define    REPORTSIZE  2000

uint8_t   PatchIndex[32];
uint8_t   ReportIndex[96];
uint8_t   Reportdata[96][40];
uint8_t   Reportdata_V[3][REPORTSIZE];
uint8_t   Reportdata_I[3][REPORTSIZE];


/****************************************************************************
 * hello_main
 ****************************************************************************/
#define TX_BUF 60
#define CONFIG_EXAMPLES_TIMER_DEVNAME 		"/dev/timer2_gps"
#define CONFIG_EXAMPLES_TIMER_CC1101  		"/dev/timer1_cc1101"
#define CONFIG_EXAMPLES_TIMER_INTERVAL 		(1000000)
#define CONFIG_EXAMPLES_TIMERCC1101_INTERVAL (10000)
#define CONFIG_EXAMPLES_TIMER_SIGNO 17
#define CONFIG_EXAMPLES_TIMER_SIGNO_CC1101 18

#define TIMR1_CNT_ADDR 0x40010024
#define TIMR2_CNT_ADDR 0x40000024


#define 	CMD_QUERYONLINE 		0
#define 	CMD_GETTIMEROFFSET 	1
#define 	CMD_SETTIMEROFFSET 	2

#define 	MASTER_ADDR 		0X10
#define 	A_ADDR 				0x01
#define 	B_ADDR 				0x02
#define 	C_ADDR 				0x03

#define 	ACK 		1
#define 	NOACK 		0

#define 	FULL 		96
#define 	EMPTY 		0

#define 	ONLINE 		1
#define 	OFFLINE 	0


#define 	MSG_START	0xAA
#define 	MSG_END		0x55

#define 	CMD_IDLE      	 	 	 0X00
#define 	CMD_READTIME      	 	 0X01
#define 	CMD_PING	    	 	 0X02
#define 	CMD_SUMMONWAVE    	 	 0X03
#define 	CMD_SUMMONWAVE_OK    	 0X13
#define 	CMD_PATCH			 	 0X04
#define 	CMD_PATCH_OK			 0X14

#define  	GETCC1101BUF_BYTES  0x01

static		uint32_t	systick = 0;
static		uint32_t	patch_systick = 0;
static		uint32_t	patch_pos = 0;

static		uint32_t	msgcmd_type = 0;
static		uint32_t	rcv_timeout = false;

#define		TIMEOUT_VALUE		25
static		uint32_t	POLL_TIMEOUT = TIMEOUT_VALUE;

struct timespec clock1;
struct timespec clock2;
struct timespec clock3;


unsigned int totalA = 0;
unsigned int successA = 0;
unsigned int failA = 0;

unsigned int totalB = 0;
unsigned int successB = 0;
unsigned int failB = 0;

unsigned int totalC = 0;
unsigned int successC = 0;
unsigned int failC = 0;

unsigned int total = 0;
unsigned int success = 0;
unsigned int fail = 0;


	
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

int GetmsgStartaddrAndLen(char *databuff,int maxlen,char **start_addr)
{
	char *ptr = (char*)databuff;
	int  msglen = maxlen;
	int  datalen = 0;
	int  rlen = 0;
	int  nspace = 0;    //invalid bytes
	
	int i = 0;
	
    for(i=0;i < msglen;i++)
    {
		if(MSG_START == *ptr)
		{
			*start_addr = (char *)ptr;
			ptr++;
			rlen++;
			msglen--;
			break;
		}
		else
		{
			ptr++;
			nspace++;    //invalid bytes
		}
	}
    //find start_flag fail
	if(0 == rlen)
	{
	    **start_addr = NULL;	
		return nspace;
	}

	datalen = *ptr++;
	rlen++;
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

int GetReportdata(char * rxbuff,uint8_t *reportdata,uint8_t *reportindex)
{
	int j = 0;
	struct report_res * P_summon_wave_res = (struct report_res *)rxbuff;
	char * ptr_src = (char *)P_summon_wave_res->data;
	char * ptr_dst = (char *)reportdata;
	char * ptr_index = (char *)reportindex;
	
	ptr_index[P_summon_wave_res->pos-1] = 'Y';
	for(j=0;j<40;j++)
	{
		ptr_dst[(P_summon_wave_res->pos-1)*40 + j] = *ptr_src++;
	}
}


void	ActiveSignal(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	//rcv_timeout = true;
	pthread_cond_signal(cond);
	pthread_mutex_unlock(mutex);
}

void waitping(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	while(msgcmd_type != CMD_PING)
	{
		pthread_cond_wait(cond, mutex);   //pthread_cond_wait 会先解除g_AdcMutex锁，再阻塞在条件变量
	}
	msgcmd_type = 0;
	pthread_mutex_unlock(mutex);
}

void waitTimeout(pthread_cond_t *cond,pthread_mutex_t *mutex)
{
	pthread_mutex_lock(mutex);
	while(rcv_timeout != true)
	{
		pthread_cond_wait(cond, mutex);   //pthread_cond_wait 会先解除g_AdcMutex锁，再阻塞在条件变量
	}
	rcv_timeout = false;
	pthread_mutex_unlock(mutex);
}

uint8_t getPatch(uint8_t *patchsum,uint8_t *reportIndex)
{
	uint8_t * ptr_patch = patchsum;
	uint8_t * ptr_report = reportIndex;
	uint8_t lost = 0;
	
	uint8_t i = 0;
	for(i=0;i<FULL;i++)
	{
		if(ptr_report[i] != 'Y')
		{
			lost++;
			*ptr_patch++ = (i+1);
		}
	}
	return lost;
}

void  calcPatchreport(irqstate_t flags,int fd,uint8_t lost,uint8_t *patchsum,uint8_t *reportIndex,struct patch_req_head* patchhead,uint8_t curball)
{
	int  wBytes = 0;		
	char *p_buf	= NULL; 
	char *p	= NULL; 
	char *phead	= (char*)patchhead; 
	char *patch	= (char*)patchsum; 

	if(!lost)
	{
		return;
	}
	
	//data parsing
	p_buf=p=(char *)malloc((sizeof(struct patch_req_head)+lost + 1));  
	if(p)
	{
		patchhead->start_flag	= MSG_START;
		patchhead->msglen 		= (sizeof(struct patch_req_head)+lost + 1);
		patchhead->type 		= CMD_PATCH;
		switch(curball)
		{
			case A_ADDR:
				 patchhead->dist	= A_ADDR;
				break;
			case B_ADDR:
				 patchhead->dist	= B_ADDR;
				break;
			case C_ADDR:
				 patchhead->dist	= C_ADDR;
				break;			
		}
		patchhead->src 		= MASTER_ADDR;
		patchhead->second 	= patch_systick;
		patchhead->pos 		= patch_pos;
		patchhead->len 		= lost;
		int i;
		//copy head
		for(i=0;i<sizeof(struct patch_req_head);i++)
		{
			*p++ = *phead++;
		}
		//copy patch
		for(i=0;i<lost;i++)
		{
			*p++ = *patch++;
		}
		//copy endflag
		*p = MSG_END;
#if 0		
		char *p_debug	= p_buf; 
		for(i=0;i<(sizeof(struct patch_req_head)+lost + 1);i++)
		{
			
			printf("<%x>\n",*p_debug++);
		}
#endif

		//send patch msg
		flags   = enter_critical_section();
		wBytes = write(fd, (char *)p_buf, (sizeof(struct patch_req_head)+lost + 1));
	    //printf("<%x>wBytes\n",wBytes);
		leave_critical_section(flags);
	}
	else
	{
		printf("malloc fail!\n");
	}
	
	free(p_buf);
	
}
void waitrcvok(uint8_t *sendok,uint8_t *timeout)
{
	//wait data rcv ok
	*sendok = 1;
	while(*timeout != 1)
	{
		usleep(1000*1);
	}
	*sendok  = 0;
	*timeout = 0;
}

int thread_waitnms(pthread_cond_t *cond,pthread_mutex_t *mutex,int nms)
{
	struct timespec ts;
	int status;
	int ret = 0;
	int time_ms = nms;

	status = pthread_mutex_lock(mutex);
	if (status != 0)
	{
		printf("thread_waiter: ERROR pthread_mutex_lock failed, status=%d\n", status);
	}

	status = clock_gettime(CLOCK_REALTIME, &ts);
	if (status != 0)
	{
		printf("thread_waiter: ERROR clock_gettime failed\n");
	}
	
	//sleep nms
	ts.tv_sec += time_ms/1000;
	ts.tv_nsec += 1000 * 1000 * (time_ms%1000);

	//wait awake
	status = pthread_cond_timedwait(cond, mutex, &ts);
    // pthread_cond_wait会先解除之前的pthread_mutex_lock锁定的mtx，然后阻塞在等待队列里休眠，直到再次被唤醒  
    //（大多数情况下是等待的条件成立而被唤醒，唤醒后，该进程会先锁定先pthread_mutex_lock(&mtx);  
    // 再读取资源 用这个流程是比较清楚的/*unlock-->block-->wait() return-->lock*/  
	if (status != 0)
	{
		if (status == ETIMEDOUT)
		{
			  printf("<%d>ms timed out\n",nms);
			  ret = 1;
		}
		else
		{
			  printf("thread_waiter: ERROR pthread_cond_timedwait failed, status=%d\n", status);
			  ret = -1;

		}
	}
	else
	{
	   printf("sig\n");
	   ret = 2;
	}

	//Release mutex
	status = pthread_mutex_unlock(mutex);
	if (status != 0)
	{
		printf("thread_waiter: ERROR pthread_mutex_unlock failed, status=%d\n", status);
	}
	
	return ret;
}


static void timer1_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
	//*(int*)TIMR1_CNT_ADDR = 0;
	//ActiveSignal(&g_TimerConVar, &g_TimerMutex);
}
static void timer2_sighandler(int signo, FAR siginfo_t *siginfo,FAR void *context)
{
    systick++;
	*(int*)TIMR2_CNT_ADDR = 0;
}

int  synctime(irqstate_t flags,int fd,int fd_timer,cc110x_timemsg * p_cc1101_msg_rx,char * rxbuff,cc110x_timemsg * p_cc1101_msg_tx)
{
	int 		timercnt_us=0;
	int  		wBytes = 0;	
	static int i = 0;
	i++;

	p_cc1101_msg_rx = (cc110x_timemsg *)rxbuff;
	p_cc1101_msg_tx->start_flag	= MSG_START;
	p_cc1101_msg_tx->msglen		= sizeof(cc110x_timemsg);
	p_cc1101_msg_tx->type			= CMD_READTIME;
	p_cc1101_msg_tx->dist			= p_cc1101_msg_rx->src;
	p_cc1101_msg_tx->src			= p_cc1101_msg_rx->dist;
	p_cc1101_msg_tx->second		= systick;
	//ioctl(fd_timer, TCIOC_GETCOUNTER, (uint32_t)(&timercnt_us));
	//ioctl(fd_timer, TCIOC_SETCOUNTER, (uint32_t)(&timercnt_us));
	
	p_cc1101_msg_tx->us			= p_cc1101_msg_rx->us;
	p_cc1101_msg_tx->endflag		= MSG_END;
	
	flags   = enter_critical_section();
	wBytes = write(fd, (char *)p_cc1101_msg_tx, sizeof(cc110x_timemsg));
    //boardctl(BOARDIOC_TIME2_PPS_DOWN, 0);		
	leave_critical_section(flags);

	printf("<%d>r-%d->%d\n",i,p_cc1101_msg_rx->src,p_cc1101_msg_rx->us);
	printf("<%d>s-%d->%d->%d\n",i,p_cc1101_msg_tx->us,p_cc1101_msg_tx->dist,timercnt_us);
	printf("=====================\n");

	return wBytes;
}

int  summon_wave(irqstate_t flags,int fd,struct report_req * P_summon_wave_req,uint8_t curball)
{
	int  wBytes = 0;	

	P_summon_wave_req->start_flag	= MSG_START;
	P_summon_wave_req->msglen		= sizeof(struct report_req);
	P_summon_wave_req->type		= CMD_SUMMONWAVE;
	switch(curball)
	{
		case A_ADDR:
			 P_summon_wave_req->dist		= A_ADDR;
			break;
		case B_ADDR:
			 P_summon_wave_req->dist		= B_ADDR;
			break;
		case C_ADDR:
			 P_summon_wave_req->dist		= C_ADDR;
			break;			
	}
	P_summon_wave_req->src			= MASTER_ADDR;
	P_summon_wave_req->second		= patch_systick;
	P_summon_wave_req->pos			= patch_pos;
	P_summon_wave_req->endflag		= MSG_END;
	
	flags   = enter_critical_section();
	wBytes = write(fd, (char *)P_summon_wave_req, sizeof(struct report_req));
	leave_critical_section(flags);

	printf("<%d>s:%d,us:%d\n",curball,P_summon_wave_req->second,P_summon_wave_req->pos);
	return wBytes;
}

/****************************************************************************
 * getSystimeFlag
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int   getSystimeFlag(void)
{
	struct	tm 	*tmp;
	time_t	timelocal;
	
	time(&timelocal);	
	tmp = localtime(&timelocal);  //获取本地时间

	if(tmp->tm_sec%10 == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/****************************************************************************
 * initSummonState
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int   initSummonState(void)
{

		clock1.tv_sec = 0;
		clock2.tv_sec = 0;
		clock1.tv_nsec = 0;
		clock2.tv_nsec = 0;

	
	summon_status.ballA_rcvState = NOACK;
	summon_status.ballB_rcvState = NOACK;
	summon_status.ballC_rcvState = NOACK;
	summon_status.ballA_rcvtotal = EMPTY;
	summon_status.ballB_rcvtotal = EMPTY;
	summon_status.ballC_rcvtotal = EMPTY;

	memset(Reportdata_V,0,sizeof(Reportdata_V));
	memset(Reportdata_I,0,sizeof(Reportdata_I));

	
	patch_systick =systick;
	patch_pos = 10;

	summon_status.enAsk = true;
}

void GetballData(int curball,uint8_t *reportindex,uint8_t *reportdata,uint8_t (*data_V)[REPORTSIZE],uint8_t (*data_I)[REPORTSIZE])
{
	//data parsing	
	int i,j = 0;
	uint8_t * p_data = reportdata;
	
	for(i=0;i<96;i++)
	{					
		if(reportindex[i] == 'Y')
		{						
			for(j=0;j<40;j++)	
			{							
				if(j < 20)	
				{			
					data_V[curball-1][i*20 + j] = *(p_data + i*40 + j); 
				}							
				else							
				{						
					data_I[curball-1][i*20 + j-20] = *(p_data + i*40 + j); 
				}						
			}						
		}				
	}
}


int  SummonwaveParsing(struct work_status * workstatus,uint8_t *patchindex,uint8_t *reportindex,int curball)
{
	int res_lost = 0;
	//summonwave parsing
	if(workstatus->work_mode == CMD_SUMMONWAVE)
	{
		res_lost = getPatch(patchindex,reportindex);
		if(res_lost)
		{
			workstatus->work_mode = CMD_PATCH;
			printf("<%d>Rn=%d\n",curball,res_lost);
		}
		else
		{
			workstatus->work_mode = CMD_SUMMONWAVE_OK;
		}
	}
	return res_lost;
}

void PatchParsing(irqstate_t flags,int fd,struct work_status * workstatus,struct patch_req_head * patchhead,uint8_t *patchindex,uint8_t *reportindex,int curball)
{
	static int 	trypatch_n = 0;
	static int 	old_lost = 0;
	
	if(workstatus->work_mode == CMD_PATCH)
	{
		int patch_lost = 0;
		patch_lost = getPatch(patchindex,reportindex);
		if(old_lost == patch_lost)
		{
			trypatch_n++;
			#if 0
			if(patch_lost >= 32)
			{
				POLL_TIMEOUT +=5*32; 
			}
			else
			{
				POLL_TIMEOUT +=5*patch_lost; 
			}
			#endif
			if(trypatch_n > 20)
			{
				workstatus->work_mode = CMD_PATCH_OK;
				trypatch_n = 0;
				old_lost = 0;
				POLL_TIMEOUT = TIMEOUT_VALUE;
			}
		}
		else
		{
			old_lost = patch_lost;
			trypatch_n = 0;
			POLL_TIMEOUT = TIMEOUT_VALUE;
		}

		if(patch_lost >= 32)
		{
			calcPatchreport(flags,fd,32,patchindex,reportindex,patchhead,curball);			
		}
		else
		{
			calcPatchreport(flags,fd,patch_lost,patchindex,reportindex,patchhead,curball);			
		}
		
		if(patch_lost)
		{
			printf("<%d>Pn=%d\n",curball,patch_lost);
			//printf("p<%d>clock1:%ds,%dms,%x\n",curball,clock1.tv_sec,clock1.tv_nsec/1000000,&clock1);
		}
		else
		{
			workstatus->work_mode = CMD_PATCH_OK;
			old_lost = 0;
		}
	}
}
void RcvdataParsing(struct work_status * workstatus,struct report_status *summon,uint8_t *patchindex,uint8_t *reportindex,uint8_t *reportdata,pthread_cond_t *cond,pthread_mutex_t *mutex,uint8_t (*data_V)[REPORTSIZE],uint8_t (*data_I)[REPORTSIZE])
{
	int total_lost = 0;
	if((workstatus->work_mode == CMD_SUMMONWAVE_OK)||(workstatus->work_mode == CMD_PATCH_OK))
	{
		//printf("a0<%d>clock1:%ds,%dms\n",summon->curball,clock1.tv_sec,clock1.tv_nsec/1000000);
		clock_gettime(CLOCK_REALTIME, &clock2);

#if 1
		//printf("a1<%d>clock1:%ds,%dms\n",summon->curball,clock1.tv_sec,clock1.tv_nsec/1000000);
		unsigned int time_diff = 0;
		time_diff = (1000000000*(clock2.tv_sec - clock1.tv_sec) + (clock2.tv_nsec - clock1.tv_nsec));
		time_diff = time_diff/1000000;
#endif
		total_lost = getPatch(patchindex,reportindex);
		workstatus->work_mode = CMD_READTIME;
		switch(summon->curball)
		{
			case A_ADDR:
				 summon->ballA_rcvtotal = FULL-total_lost;
				 summon->ballA_rcvState = ACK;
				 if(summon->ballA_rcvtotal == FULL)
				 {
					 successA++;
				 }
				 else
				 {
					  failA++;
				 }
				break;
			case B_ADDR:
				 summon->ballB_rcvtotal = FULL-total_lost;
				 summon->ballB_rcvState = ACK;
				 if(summon->ballB_rcvtotal == FULL)
				 {
					 successB++;
				 }
				 else
				 {
					  failB++;
				 }
				break;
			case C_ADDR:
				 summon->ballC_rcvtotal = FULL-total_lost;
				 summon->ballC_rcvState = ACK;
				 if(summon->ballC_rcvtotal == FULL)
				 {
					 successC++;
				 }
				 else
				 {
					  failC++;
				 }
				break;
		}
	 	GetballData(summon->curball,reportindex,reportdata,data_V,data_I);
#if 1
		//printf("b<%d>clock2:%ds,%dms\n",summon->curball,clock2.tv_sec,clock2.tv_nsec/1000000);
		printf("<%d>:%dms\n",summon->curball,time_diff);
		//printf("<%d>clock2:%ds,%dms\n",summon->curball,clock2.tv_sec,clock2.tv_nsec/1000000);

		clock1.tv_sec = 0;
		clock2.tv_sec = 0;
		clock1.tv_nsec = 0;
		clock2.tv_nsec = 0;		
#endif
		printf("<%d>rcv<%d>\n",summon->curball,FULL-total_lost);
		if((summon->ballA_rcvState == ACK)&&
		    (summon->ballB_rcvState == ACK)&&
			(summon->ballC_rcvState == ACK))
		{
			  ActiveSignal(cond, mutex);
		}
	}

}
/****************************************************************************
 * report_cc1101
 * liushuhe
 * 2018.02.23
 ****************************************************************************/
int report_cc1101(int argc, char *argv[])
{

	while(1)
	{
		waitping(&g_PingConVar,&g_PingMutex);
		
		while(!getSystimeFlag())
		{
			usleep(500*1000);
		}
		
		initSummonState();
		
		total++;
		
		thread_waitnms(&g_TimerConVar, &g_TimerMutex,5*1000);
		
		summon_status.enAsk = false;
		
		if((summon_status.ballA_rcvtotal == FULL)&&
		    (summon_status.ballB_rcvtotal == FULL)&&
			(summon_status.ballC_rcvtotal == FULL))
		{
			success++;
			
			printf("report SUCCESS:ballA<%d>,ballB<%d>ballC<%d>\n",
										summon_status.ballA_rcvtotal,
									    summon_status.ballB_rcvtotal,
									    summon_status.ballC_rcvtotal);
		}
		else
		{
			fail++;
			
			printf("report FAIL:ballA<%d>,ballB<%d>ballC<%d>\n",
										summon_status.ballA_rcvtotal,
									    summon_status.ballB_rcvtotal,
									    summon_status.ballC_rcvtotal);
		}
		
		printf("SUMMON ASK:<%d>  SUCCESS:<%d>  FAIL:<%d>\n",total,success,fail);
		printf("A ASK:<%d>  SUCCESS:<%d>  FAIL:<%d>\n",totalA,successA,failA);
		printf("B ASK:<%d>  SUCCESS:<%d>  FAIL:<%d>\n",totalB,successB,failB);
		printf("C ASK:<%d>  SUCCESS:<%d>  FAIL:<%d>\n",totalC,successC,failC);
#if 0 
		int i = 0;
		int j = 0;
		for(i=0;i<3;i++)
		{
			printf("<%d>-ball\n",i);
			printf("V<%d>:\n",i);
			for(j=0;j<REPORTSIZE;j++)
			{
				if(j%50 == 0)
				{
					printf("\n");
				}
				printf("<%d> ",Reportdata_V[i][j]);
			}
			printf("\n");
			printf("I<%d>:\n",i);
			for(j=0;j<REPORTSIZE;j++)
			{
				if(j%50 == 0)
				{
					printf("\n");
				}
				printf("<%d> ",Reportdata_I[i][j]);
			}
			printf("\n");
		}
#endif		
	}
}


/****************************************************************************
 * master_cc1101
 * liushuhe
 * 2018.01.29
 ****************************************************************************/
int master_cc1101(int argc, char *argv[])
{
	int ret =-1;
	
	irqstate_t flags;
  	struct pollfd fds[1];

	struct timer_notify_s notify;
	struct sigaction act;

	//struct timeval timeout;
	//fd_set 	rfds;
	
    char 	rxbuff[1024];
    char 	*P_data = NULL;
	int 	fd;
	int 	fd_timer2;
	int  	iRet = 0;
	int  	poll_ret = 0;
	int  	cc1101buf_datalen = 0;
	int  	rBytes = 0;
	int  	wBytes = 0;

	int 	timeout_f = 0;
	int 	ready_f = 0;

	struct timespec clock;
				
    boardctl(BOARDIOC_TIME2_PPS_INIT, 0);
    boardctl(BOARDIOC_433_PWRON, 0);

	fd = open("/dev/cc1101", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if (fd < 0)
	{
		printf("open cc1101 error:\n");
	}

	fd_timer2 = open(CONFIG_EXAMPLES_TIMER_DEVNAME, O_RDWR);
	if (fd_timer2 < 0)
	{
		printf("ERROR: Failed to open %s: %d\n",CONFIG_EXAMPLES_TIMER_DEVNAME, errno);
	}

	iRet = ioctl(fd_timer2, TCIOC_SETTIMEOUT, CONFIG_EXAMPLES_TIMER_INTERVAL);
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
	
	iRet = ioctl(fd_timer2, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
	if (iRet < 0)
	{
		printf("ERROR: Failed to set the timer handler: %d\n", errno);
	}
  
	iRet = ioctl(fd_timer2, TCIOC_START, 0);
	if (iRet < 0)
	{
		printf("ERROR: Failed to start the timer: %d\n", errno);
	}
	/* Do we already hold the semaphore? */
	
	work_sts.work_mode = CMD_READTIME;
	
	while(1)
	{
		memset(fds, 0, sizeof(struct pollfd));
		fds[0].fd       = fd;
		fds[0].events   = POLLIN;
		fds[0].revents  = 0;
		//timeout 15ms
		poll_ret = poll(fds, 1,POLL_TIMEOUT);
		if (poll_ret < 0) 
		{
			//add by liushuhe 2018.01.19
			if(errno != EINTR)
			{
				//printf("select error!!!<%d>\n",errno);
			}
			else
			{
				;
			}
		}
		else if(poll_ret == 0)
		{
			timeout_f = true;
			//lock 
			pthread_mutex_lock(&g_SummonMutex);
#if 1
			#if 0
			//get timeout
			static int time_old = 0;
			int time_diff = 0;
			clock_gettime(CLOCK_REALTIME, &clock1);
			time_diff = clock1.tv_nsec/1000000 - time_old;
			time_old =  clock1.tv_nsec/1000000;
			printf("%dms\n",time_diff);
			#endif
			//summonwave parsing
			int lost = 0;
			lost = SummonwaveParsing(&work_sts,(uint8_t *)&PatchIndex,(uint8_t *)&ReportIndex,summon_status.curball);	
			if(lost == FULL)
			{
				continue;
			}
			//patch parsing
			PatchParsing(flags,fd,&work_sts,&patch_head,(uint8_t *)&PatchIndex,(uint8_t *)&ReportIndex,summon_status.curball);
			//rcv total
			RcvdataParsing	(&work_sts,&summon_status,(uint8_t *)&PatchIndex,(uint8_t *)&ReportIndex,(uint8_t *)&Reportdata,&g_TimerConVar,&g_TimerMutex,Reportdata_V,Reportdata_I);
#endif
			pthread_mutex_unlock(&g_SummonMutex);
		}
		else if ((fds[0].revents & POLLERR) && (fds[0].revents & POLLHUP))
		{
			printf("ERROR: worker poll read Error %d\n", errno);
		}
		else if (fds[0].revents & POLLIN)
		{
			if (poll_ret != 1)
			{
				//printf("my_read: ERROR poll reported: %d\n", iRet);
			}
			else
			{
				ready_f = true;
			}

			while((ret = ioctl(fd, GETCC1101BUF_BYTES, (unsigned long)&cc1101buf_datalen)))
			{
				if(cc1101buf_datalen > sizeof(rxbuff))
				{
					cc1101buf_datalen = sizeof(rxbuff);
				}
				memset(rxbuff, 0, sizeof(rxbuff));
			   	rBytes = myreadn(fd,rxbuff,cc1101buf_datalen,&timeout_f,&ready_f);
	            /****************************************************************/
				int msg_datalen = 0;
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
								if(work_sts.work_mode == CMD_READTIME)
								{
									wBytes = synctime(flags,fd,fd_timer2,P_cc1101_msg_rx,P_data,P_cc1101_msg_tx);
									if(wBytes != sizeof(cc110x_timemsg))
									{
										printf("send synctime fail!\n");
									}
								}
								break;
							case CMD_PING:
								msgcmd_type = CMD_PING;
								ActiveSignal(&g_PingConVar,&g_PingMutex);
								if((summon_status.enAsk == true)&&(work_sts.work_mode == CMD_READTIME))
								{
									if(!pthread_mutex_trylock(&g_SummonMutex))
									{
										switch(P_data[4])
										{
											//three ball
											case A_ADDR:
												if(summon_status.ballA_rcvState != ACK)
												{
													clock_gettime(CLOCK_REALTIME, &clock1);
													printf("<%d>clock1:%ds,%dms\n",P_data[4],clock1.tv_sec,clock1.tv_nsec/1000000);

													totalA++;
												  	summon_status.curball = A_ADDR;
													summon_wave(flags,fd,&summon_wave_req,summon_status.curball);
													work_sts.work_mode = CMD_SUMMONWAVE;
													memset(ReportIndex,0,sizeof(ReportIndex));
													memset(Reportdata,0,sizeof(Reportdata));
													printf("<%d>summon_wave\n",summon_status.curball);
												}
												break;
											case B_ADDR:
												if(summon_status.ballB_rcvState != ACK)
												{
													clock_gettime(CLOCK_REALTIME, &clock1);
													printf("<%d>clock1:%ds,%dms\n",P_data[4],clock1.tv_sec,clock1.tv_nsec/1000000);
													
													totalB++;
												    summon_status.curball = B_ADDR;
													summon_wave(flags,fd,&summon_wave_req,summon_status.curball);
													work_sts.work_mode = CMD_SUMMONWAVE;
													memset(ReportIndex,0,sizeof(ReportIndex));
													memset(Reportdata,0,sizeof(Reportdata));
													printf("<%d>summon_wave\n",summon_status.curball);
												}
												break;
											case C_ADDR:
												if(summon_status.ballC_rcvState != ACK)
												{
													clock_gettime(CLOCK_REALTIME, &clock1);
													printf("<%d>clock1:%ds,%dms\n",P_data[4],clock1.tv_sec,clock1.tv_nsec/1000000);

													totalC++;
												    summon_status.curball = C_ADDR;
													summon_wave(flags,fd,&summon_wave_req,summon_status.curball);
													work_sts.work_mode = CMD_SUMMONWAVE;
													memset(ReportIndex,0,sizeof(ReportIndex));
													memset(Reportdata,0,sizeof(Reportdata));
													printf("<%d>summon_wave\n",summon_status.curball);
												}
												break;	
										}
										pthread_mutex_unlock(&g_SummonMutex);
									}
								}
								break;
							case CMD_SUMMONWAVE:
								if(P_data[4] == summon_status.curball)
								{
									GetReportdata(P_data,(uint8_t *)&Reportdata,(uint8_t *)&ReportIndex);
								}
								break;
							case CMD_PATCH:
								if(P_data[4] == summon_status.curball)
								{
									GetReportdata(P_data,(uint8_t *)&Reportdata,(uint8_t *)&ReportIndex);
								}
							break;		
						}
					}
				}
				while(rBytes >0);	
			}
		}
	}    

  return 0;
}






