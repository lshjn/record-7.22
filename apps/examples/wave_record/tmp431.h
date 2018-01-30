#ifndef _TASK_TMP431_H
#define _TASK_TMP431_H

#ifdef __cplusplus
 extern "C" {
#endif 
#include <nuttx/i2c/i2c_master.h>

extern int I2C_M_WRIRE; /* write data, master to slave*/



int master_tmp431(int argc, char *argv[]);
int i2cdev_transfer(int fd, FAR struct i2c_msg_s *msgv, int msgc);
int tmp431_init(int fd);
int get_tempvalue  (int fd);
int read_temp_low  (int fd);
int read_temp_high  (int fd);
int en_local_chanel  (int fd);
int set_tempe_range  (int fd);
int set_rate (int fd);




#ifdef __cplusplus
}
#endif

#endif 
