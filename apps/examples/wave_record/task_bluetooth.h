#ifndef _TASK_BLUETOOTH_H
#define _TASK_BLUETOOTH_H

#ifdef __cplusplus
 extern "C" {
#endif 

#define		BT_WAIT					0
#define		BT_OPEN_LOCK			1

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct msgbt
{
	char type;				//1:open lock
	char msg_state;			//0:noack	1:ack
	char msgbuf[50];
	char msglen;
};

extern struct msgbt bluetooth_msg;
/****************************************************************************
 * Private function
 ****************************************************************************/
int master_bluetooth(int argc, char *argv[]);




#ifdef __cplusplus
}
#endif

#endif 
