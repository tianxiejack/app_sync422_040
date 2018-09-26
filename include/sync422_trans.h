

#ifndef _SYNC422_TRANS_H_
#define _SYNC422_TRANS_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/types.h>

int sync422_spi_create(int uart, int mode);// 0 0
int sync422_spi_destory(int uart);
int sync422_spi_speed(int uart, int ispeed);
int sync422_ontime_video(int dtype, unsigned char *buf, int len);
int sync422_ontime_ctrl(int iCmd, int iPrm);

///////////
int sync422_demo_start(void);
int sync422_demo_stop(void);
void testSnd(int ichl, int mode);


#endif

