

#ifndef _SPIDEV_TRANS_H_
#define _SPIDEV_TRANS_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <osa.h>

#define PLAT_TX1		0
#define PLAT_TX2		1

#if PLAT_TX1
#define Tranverse32(X) (X)
#elif PLAT_TX2
#define Tranverse32(X) ((((Uint32)(X)&0xff000000)>>24)|(((Uint32)(X)&0x00ff0000)>>8)|(((Uint32)(X)&0x0000ff00)<<8)|(((Uint32)(X)&0x000000ff)<<24))
//Tranverse32 -----  switch 32bit data from Little-endian  to Big-endian
#endif

int spi_init(const char *device, uint8_t bits, uint32_t speed);
int spi_close(int fd);
int spi_transfer(int fd, struct spi_ioc_transfer *tr);
int spi_DataTransform(Uint8 *dst, Uint8 *src, int srcNum);

#endif

