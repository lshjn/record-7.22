#ifndef _TASK_CC1101_H
#define _TASK_CC1101_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define ALIGN __attribute__((packed))
#define FRAME_REPORT_SIZE  10

extern pthread_mutex_t g_TimerMutex;
extern pthread_cond_t  g_TimerConVar;
//ping
extern pthread_mutex_t g_PingMutex;
extern pthread_cond_t  g_PingConVar;

//tcp
extern pthread_mutex_t g_TcpMutex;
extern pthread_cond_t  g_TcpConVar;

#define    REPORTSIZE  1920

extern uint8_t   PatchIndex[32];
extern uint8_t   ReportIndex[96];
extern uint8_t   Reportdata[96][40];
extern uint8_t   Reportdata_V[3][REPORTSIZE];
extern uint8_t   Reportdata_I[3][REPORTSIZE];


extern uint16_t   Reportdata_VV[3][80*12];
extern uint16_t   Reportdata_II[3][80*12];


struct cc110x_msg1{
	uint8_t cmd;
	uint8_t online_A;
	uint8_t online_B;
	uint8_t online_C;


	//u8_t CheckSum;
	uint8_t Ack;
	uint32_t  TimerCnt;
	uint32_t  SysTitk;
	
   	uint8_t cur_balladdr;										//当前处理的采集球的地址编号
 
	uint8_t src;
	uint8_t dist;
	
	uint32_t  GetTime;
	uint32_t  SetTime;
	
};

//sync
typedef struct ALIGN timemsg{
	uint8_t  start_flag;
	uint8_t  msglen;
	uint8_t  type;
	uint8_t  dist;
	uint8_t  src;
	//private
	uint32_t second; 		//定时器中断累计值
	uint32_t us;            //定时器的cnt
	uint8_t  endflag;        
}cc110x_timemsg;

#if 0
struct report_data{
	uint8_t   ReportIndex[96];
	uint8_t   Reportdata[96][40];
	uint8_t   Reportdata_V[REPORTSIZE];
	uint8_t   Reportdata_I[REPORTSIZE];
};
#endif

struct work_status{
	uint8_t  work_mode;
};


struct report_status{
	uint8_t  enAsk;
	
	uint8_t  curball;
	uint8_t  ballA_rcvtotal;
	uint8_t  ballB_rcvtotal;
	uint8_t  ballC_rcvtotal;
	
	uint8_t  ballA_rcvState;
	uint8_t  ballB_rcvState;
	uint8_t  ballC_rcvState;
};

//report_req
struct ALIGN report_req{
	uint8_t  start_flag;
	uint8_t  msglen;
	uint8_t  type;
	uint8_t  dist;
	uint8_t  src;
	//private
	uint32_t second; 		//定时器中断累计值,第几秒数据
	uint32_t pos;           //0-3999
	uint8_t  endflag;        
};

//report_res
struct ALIGN report_res{
	uint8_t  start_flag;
	uint8_t  msglen;
	uint8_t  type;
	uint8_t  dist;
	uint8_t  src;
	//private
	uint8_t  sum; 		    //总包数
	uint8_t  pos;           //第几包         
	uint16_t data[FRAME_REPORT_SIZE << 1];         
	uint8_t  endflag;        
};


//patch_req
struct ALIGN patch_req_head{
	uint8_t  start_flag;
	uint8_t  msglen;
	uint8_t  type;
	uint8_t  dist;
	uint8_t  src;
	//private
	uint32_t  second; 		    ////定时器中断累计值,第几秒数据
	uint32_t  pos;           	//0-3999       
	uint8_t  len;           	//patch 的个数    
	//patch[max = 32]
	//uint8_t  endflag;        
};



int master_cc1101(int argc, char *argv[]);
int report_cc1101(int argc, char *argv[]);



#ifdef __cplusplus
}
#endif

#endif 
