/*
 * Copyright 2012 Andrew Smith
 * Copyright 2012 Luke Dashjr
 * Copyright 2013 Axel Walsleben
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"
 
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "logging.h"
#include "miner.h"
#include "fpgautils.h"
#include "util.h"
#include <termios.h> // POSIX terminal control definitionss
#include <linux/serial.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss

#include <linux/spi/spidev.h>

#define BITSTREAM_FILENAME "artix.bit"

struct device_drv artix_drv;

static const char *device = "/dev/spidev0.0";

#define TX_BUF (4096)

#define SPI_BUS_SPEED 5000000
#define SPI_BUS_BITS 8
#define SPI_BUS_DELAY 0

#define MAXnumDevices 10

#define BLOCK_SIZE 65536
#define CHUNK_SIZE 1024
#define TICK_COUNT 2048

typedef unsigned char byte;

static unsigned char ones[CHUNK_SIZE], zeros[CHUNK_SIZE];

void longToByteArray(unsigned long l, byte *b){
    b[0]=(byte)(l&0xff);
    b[1]=(byte)((l>>8)&0xff);
    b[2]=(byte)((l>>16)&0xff);
    b[3]=(byte)((l>>24)&0xff);
  }

void shortToByteArray(const unsigned short l, byte *b){
    b[0]=(byte)(l&0xff);
    b[1]=(byte)((l>>8)&0xff);
  }
unsigned long byteArrayToLong(const byte *b){
    return ((unsigned long)b[3]<<24)+((unsigned long)b[2]<<16)+
      ((unsigned long)b[1]<<8)+(unsigned long)b[0];
  }
  
void flush_tms(struct cgpu_info *artix, int force)
{
	uint32_t y;
	applog(LOG_DEBUG, "flush_tms");
  memset(artix->jtag_buf_tx,  0x88, CHUNK_SIZE);
  artix->jtag_buf_tx[0] = 0x10; // Add JTAG CMD Header
  artix->jtag_len = 1;
}

void set_tms(struct cgpu_info *artix, bool val)
{
  artix->tms = val;
  artix->jtag_buf_tx[artix->jtag_len] = 0x20;
  if(artix->tms) artix->jtag_buf_tx[artix->jtag_len] |= 0x44; // Set TMS Value
	if(artix->tdo) artix->jtag_buf_tx[artix->jtag_len] |= 0x11; // Set TDO Value
  artix->jtag_len++;
}

void set_lasttms(struct cgpu_info *artix)
{
	artix->jtag_buf_tx[artix->jtag_len-1] |= 0x44; // Set TMS Value
}


void tapTestLogicReset(struct cgpu_info *artix)
{
  int i;
  memset(ones , 0xff, CHUNK_SIZE);
  memset(zeros, 0x00, CHUNK_SIZE);
  flush_tms(artix, true);
  for(i=0; i<5; i++)
      set_tms(artix, true);

}

void shiftMyTDI(struct cgpu_info *artix, const unsigned char *tdi, int length, bool last)
{
  uint32_t i, ret;
	static uint8_t bits = 8;
	static uint32_t speed = 5000000;
	static uint16_t delay = 0;
  
  if(length==0) return;
	
	for (i=0; i<length; i++)
	{
		if (tdi) {
			artix->tdo = (tdi[i/8] >> i%8) & 1;
		}
	  if ((i == length-1) && last) artix->tms = true;
	  artix->jtag_buf_tx[artix->jtag_len] = 0x20;
	  if(artix->tms) artix->jtag_buf_tx[artix->jtag_len] |= 0x44; // Set TMS Value
		if(artix->tdo) artix->jtag_buf_tx[artix->jtag_len] |= 0x11; // Set TDO Value
	  artix->jtag_len++;			
	}
}

void shiftTDITDO(struct cgpu_info *artix, const unsigned char *tdi, unsigned char *tdo,
			 int length, bool last)
{
  uint32_t i, ret;
	static uint8_t bits = 8;
	static uint32_t speed = 5000000;
	static uint16_t delay = 0;
  
  if(length==0) return;
	
	for (i=0; i<length; i++)
	{
		if (tdi) {
			artix->tdo = (tdi[i/8] >> i%8) & 1;
		}
	  
	  artix->jtag_buf_tx[artix->jtag_len] = 0x20;
	  if(artix->tms) artix->jtag_buf_tx[artix->jtag_len] |= 0x44; // Set TMS Value
		if(artix->tdo) artix->jtag_buf_tx[artix->jtag_len] |= 0x11; // Set TDO Value
	  artix->jtag_len++;			
	}
	if (last) {
//		applog(LOG_ERR, "last is set");
		artix->jtag_buf_tx[artix->jtag_len] = 0x88; // Add Dummy Cmd to get last result
		artix->jtag_len++;

/*	
	for ( i=0; i<artix->jtag_len; i=i+16)
	{
		applog(LOG_ERR, ">> %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", artix->jtag_buf_tx[i], artix->jtag_buf_tx[i+1],  artix->jtag_buf_tx[i+2], artix->jtag_buf_tx[i+3], artix->jtag_buf_tx[i+4], artix->jtag_buf_tx[i+5], artix->jtag_buf_tx[i+6], artix->jtag_buf_tx[i+7], artix->jtag_buf_tx[i+8], artix->jtag_buf_tx[i+9], artix->jtag_buf_tx[i+10], artix->jtag_buf_tx[i+11], artix->jtag_buf_tx[i+12], artix->jtag_buf_tx[i+13], artix->jtag_buf_tx[i+14], artix->jtag_buf_tx[i+15]);
	}
*/	
		struct spi_ioc_transfer tr = {
		        .tx_buf = (unsigned long)artix->jtag_buf_tx,
		        .rx_buf = (unsigned long)artix->jtag_buf_rx,
		        .len = artix->jtag_len,
		        .delay_usecs = SPI_BUS_DELAY,
		        .speed_hz = SPI_BUS_SPEED,
		        .bits_per_word = SPI_BUS_BITS,
		};	
		tr.len = artix->jtag_len;
		ret = ioctl(artix->device_fd, SPI_IOC_MESSAGE(1), &tr);
/*
	for ( i=0; i<ret; i=i+16)
	{
		applog(LOG_ERR, "<<%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", artix->jtag_buf_rx[i], artix->jtag_buf_rx[i+1],  artix->jtag_buf_rx[i+2], artix->jtag_buf_rx[i+3], artix->jtag_buf_rx[i+4], artix->jtag_buf_rx[i+5], artix->jtag_buf_rx[i+6], artix->jtag_buf_rx[i+7], artix->jtag_buf_rx[i+8], artix->jtag_buf_rx[i+9], artix->jtag_buf_rx[i+10], artix->jtag_buf_rx[i+11], artix->jtag_buf_rx[i+12], artix->jtag_buf_rx[i+13], artix->jtag_buf_rx[i+14], artix->jtag_buf_rx[i+15]);
	}
*/
		  if (tdo) {
	  	for (i=0; i<length/8; i++)
	  	{
	  		tdo[i] = 0x00;
	  	}
	  	for (i=0; i<length; i++)
	  	{
	  		
	  		tdo[i/8] |= ((artix->jtag_buf_rx[ret-length+i+1] & 1) << i%8);
	  	}			
	  }
	
	  memset(artix->jtag_buf_tx,  0x88, CHUNK_SIZE);
	  memset(artix->jtag_buf_rx,  0x00, CHUNK_SIZE);
	  
	  artix->jtag_buf_tx[0] = 0x10; // Add JTAG CMD Header
	  artix->jtag_len = 1;
	}
}

void shiftTDI(struct cgpu_info *artix, const unsigned char *tdi, int length, bool last)
{
  shiftTDITDO(artix, tdi, NULL, length,last);
}

void shiftTDO(struct cgpu_info *artix, unsigned char *tdo, int length, bool last)
{
    shiftTDITDO(artix, NULL, tdo, length,last);
}

int getChain(struct cgpu_info *artix, bool detect)
{
  int i;
  if(artix->numDevices  == -1 || detect)
    {
      tapTestLogicReset(artix);
			
			set_tms(artix, false); // Jetzt zum Shift-DR
			set_tms(artix, true);
			set_tms(artix, false);
			set_tms(artix, false);

      byte idx[4];
      byte zero[4];
      artix->numDevices=0;
      for(i=0; i<4; i++)zero[i]=0;
      do{
					shiftTDITDO(artix, zero,idx,32,true);
					unsigned long id=byteArrayToLong(idx);
//					applog(LOG_ERR, "Getchain %d, %08x", artix->numDevices, id);
					if(id==0x13631093){
					  artix->numDevices++;
					}
					else{
					  if (id == 0xffffffff && artix->numDevices >0)
					    {
					      applog(LOG_ERR, "Probably a broken Atmel device in your chain!");
					      applog(LOG_ERR, "No succeeding device can be identified");
					    }
					  break;
					}
      }while(artix->numDevices<MAXnumDevices);
    }
  applog(LOG_ERR, "getChain found %d devices",artix->numDevices);
  return artix->numDevices;
}

int SelectDevice(struct cgpu_info *artix, int dev) {
  if(dev>=artix->numDevices)artix->deviceIndex=-1;
  	else artix->deviceIndex=dev;
	applog(LOG_DEBUG,"selectDevice %d", artix->deviceIndex);
  return artix->deviceIndex;	
}

void poke(struct cgpu_info *artix, int addr, long value) {
  applog(LOG_DEBUG, "Poke %d : %08x", addr, value);
	uint32_t i, dev;
  byte mydata[512];
  byte checksum;
  
  mydata[0] = 0x02;
	for(i=0; i<5; i++)  set_tms(artix, true); // Reset TAP Controller
	set_tms(artix, false); // Jetzt zum Shift-IR
	set_tms(artix, true);
	set_tms(artix, true);
	set_tms(artix, false);
	set_tms(artix, false);

  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    shiftMyTDI(artix, ones, 6, false);  // Send pre BYPASS bits.
	shiftMyTDI(artix, mydata, 6, false);	// Alle mit Load UserReg1 laden
  for(dev=0; dev<artix->deviceIndex; dev++)
    shiftMyTDI(artix, ones, 6, false);  // Send post BYPASS bits.
  set_lasttms(artix);

	set_tms(artix, true); // Jetzt zum Shift-DR
	set_tms(artix, true);
	set_tms(artix, false);
	set_tms(artix, false);

  mydata[0]=(byte)(value&0xff);
	mydata[1]=(byte)((value>>8)&0xff);
	mydata[2]=(byte)((value>>16)&0xff);
	mydata[3]=(byte)((value>>24)&0xff);
  mydata[4] = 0x00;
  checksum = mydata[0] ^ mydata[1] ^ mydata[2] ^ mydata[3];
  checksum = (checksum >> 4)  ^ (checksum & 0xF);
  checksum = (checksum >> 2)  ^ (checksum & 0x3);
  checksum = (checksum >> 1)  ^ (checksum & 0x1);
  mydata[4] = ((( (addr >> 3) & 1) ^ ((addr >> 2) & 1) ^ ((addr >> 1) & 1) ^ (addr & 1) ^ (checksum & 1)) << 5 ) | 0x10 | (addr & 0x0F);
	
  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    shiftMyTDI(artix, ones, 1, false);  // Send pre BYPASS bits.
	shiftMyTDI(artix, mydata, 38, false);	// Alle mit Load UserReg1 laden
  for(dev=0; dev<artix->deviceIndex; dev++)
    shiftMyTDI(artix, ones, 1, false);  // Send post BYPASS bits.
  set_lasttms(artix);

	set_tms(artix, true); // zum Update-DR
	set_tms(artix, true); // zum Select-DR
	set_tms(artix, false); // Zum Capture-DR
	set_tms(artix, false);

  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    shiftMyTDI(artix, ones, 1, false);  // Send pre BYPASS bits.
	shiftTDITDO(artix, NULL, mydata, 33, true);	
}

long peek(struct cgpu_info *artix, int addr) {
 	applog(LOG_DEBUG, "Peek %d", addr);
	uint32_t i, dev;
  byte mydata[512];

  mydata[0] = 0x02;
	for(i=0; i<5; i++)  set_tms(artix, true); // Reset TAP Controller
	set_tms(artix, false); // Jetzt zum Shift-IR
	set_tms(artix, true);
	set_tms(artix, true);
	set_tms(artix, false);
	set_tms(artix, false);

  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    shiftMyTDI(artix, ones, 6, false);  // Send pre BYPASS bits.
	shiftMyTDI(artix, mydata, 6, false);	// Alle mit Load UserReg1 laden
  for(dev=0; dev<artix->deviceIndex; dev++)
    shiftMyTDI(artix, ones, 6, false);  // Send post BYPASS bits.
  set_lasttms(artix);

	set_tms(artix, true); // Jetzt zum Shift-DR
	set_tms(artix, true);
	set_tms(artix, false);
	set_tms(artix, false);

  mydata[0] = 0X00;
  mydata[1] = 0x00;
  mydata[2] = 0x00;
  mydata[3] = 0x00;
  mydata[4] = (addr & 0x0f) | ((1 ^ ((addr >> 3) & 1) ^ ((addr >> 2) & 1) ^ ((addr >> 1) & 1) ^ (addr & 1) ) << 5 );
	
  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    shiftMyTDI(artix, ones, 1, false);  // Send pre BYPASS bits.
	shiftMyTDI(artix, mydata, 38, false);	// Alle mit Load UserReg1 laden
  for(dev=0; dev<artix->deviceIndex; dev++)
    shiftMyTDI(artix, ones, 1, false);  // Send post BYPASS bits.
  set_lasttms(artix);

	set_tms(artix, true); // zum Update-DR
	set_tms(artix, true); // zum Select-DR
	set_tms(artix, false); // Zum Capture-DR
	set_tms(artix, false);

  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    shiftMyTDI(artix, ones, 1, false);  // Send pre BYPASS bits.
  
	shiftTDITDO(artix, NULL, mydata, 30, true);	
	
	return byteArrayToLong(mydata);    ;
}


long getUsercode(struct cgpu_info *artix) {
//	applog(LOG_ERR, "getUSercode");
	uint32_t i;
        byte mydata[512];

        mydata[0] = 0x08;
	for(i=0; i<5; i++)  set_tms(artix, true); // Reset TAP Controller
	set_tms(artix, false); // Jetzt zum Shift-IR
	set_tms(artix, true);
	set_tms(artix, true);
	set_tms(artix, false);
	set_tms(artix, false);
	for (i=0; i<artix->numDevices-1; i++)
	{
		shiftMyTDI(artix, mydata, 6, false);	// Alle mit Load UserCode laden
	}
	shiftMyTDI(artix, mydata, 6, true);	// Alle mit Load UserCode geladen und beim letzten bit TMS=1
	set_tms(artix, true); // Jetzt zum Shift-DR
	set_tms(artix, true);
	set_tms(artix, false);
	set_tms(artix, false);
	mydata[0] = 0;
	shiftTDITDO(artix, NULL, mydata, (32*artix->numDevices)+1, true);
	
	for (i=0; i<artix->numDevices; i++)
	{
		applog(LOG_ERR, "UserIDs %d: %08x",i ,byteArrayToLong(&mydata[i*4]) );
	}
  set_tms(artix, true);
  set_tms(artix, false);
  
  return byteArrayToLong(mydata);
}

uint32_t sendJtag(struct cgpu_info *artix, const unsigned char *tx, unsigned char *rx, int length)
{
  uint32_t i, ret;
  if(length==0) return;

	struct spi_ioc_transfer tr = {
		        .tx_buf = (unsigned long)tx,
		        .rx_buf = (unsigned long)rx,
		        .len = length,
		        .delay_usecs = SPI_BUS_DELAY,
		        .speed_hz = SPI_BUS_SPEED,
		        .bits_per_word = SPI_BUS_BITS,
	};	
	tr.len = length;
	ret = ioctl(artix->device_fd, SPI_IOC_MESSAGE(1), &tr);
			
	return ret;		
}

void resetJtag(struct cgpu_info *artix)
{
	unsigned char tx_buf[] = {0x10, 0x64, 0x64, 0x64, 0x64, 0x64, 0x88};
	unsigned char rx_buf[32];
	sendJtag (artix, tx_buf, rx_buf, 7);
}

uint32_t readRegJtag(struct cgpu_info *artix, uint32_t chip, uint32_t reg)
{
	unsigned char tx_buf[] = {0x10, 
														0x20, 0x64, 0x64, 0x20, 0x20,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x75,
														0x64, 0x64, 0x20, 0x20, // Shift-DR
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x75,
														0x64, 0x64, 0x20, 0x20, 
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
														0x31, 0x31, 0x31, 0x31, 0x75,
														0x64 ,0x20, 0x88
														};
	unsigned char rx_buf[256];
	
	uint32_t i, ret;
	
	tx_buf[6+((7-chip)*6)  ] = 0x20;
	tx_buf[6+((7-chip)*6)+1] = 0x31;
	tx_buf[6+((7-chip)*6)+2] = 0x20;
	tx_buf[6+((7-chip)*6)+3] = 0x20;
	tx_buf[6+((7-chip)*6)+4] = 0x20;
	tx_buf[6+((7-chip)*6)+5] = 0x20;
  tx_buf[53] |=0x44;

  for (i=0; i<38; i++)
  	tx_buf[(58+7-chip)+i] = 0x20;	
  
	switch (reg) {
		case 0:  break;
		case 1:  tx_buf[(58+7-chip)+32] = 0x31;
						 break;
		case 2:  tx_buf[(58+7-chip)+33] = 0x31;
						 break;
		case 3:  tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+33] = 0x31;
						 break;
		case 4:  tx_buf[(58+7-chip)+34] = 0x31;
						 break;
		case 5:  tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+34] = 0x31;
						 break;
		case 6:  tx_buf[(58+7-chip)+33] = 0x31;
						 tx_buf[(58+7-chip)+34] = 0x31;
						 break;
		case 7:  tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+33] = 0x31;
						 tx_buf[(58+7-chip)+34] = 0x31;
						 break;
		case 8:  tx_buf[(58+7-chip)+35] = 0x31;
						 break;
		case 9:  tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		case 10: tx_buf[(58+7-chip)+33] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		case 11: tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+33] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		case 12: tx_buf[(58+7-chip)+34] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		case 13: tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+34] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		case 14: tx_buf[(58+7-chip)+33] = 0x31;
						 tx_buf[(58+7-chip)+34] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		case 15: tx_buf[(58+7-chip)+32] = 0x31;
						 tx_buf[(58+7-chip)+33] = 0x31;
						 tx_buf[(58+7-chip)+34] = 0x31;
						 tx_buf[(58+7-chip)+35] = 0x31;			
						 break;
		default: break;
	}  
  tx_buf[(58+7-chip)+36] = 0x20;
  if ((1 ^ (tx_buf[(58+7-chip)+32] & 1) ^ (tx_buf[(58+7-chip)+33] & 1) ^ (tx_buf[(58+7-chip)+34] & 1) ^ (tx_buf[(58+7-chip)+35] & 1)) == 1)
  	tx_buf[(58+7-chip)+37] = 0x31;
  else
  	tx_buf[(58+7-chip)+37] = 0x20;
  tx_buf[102] |=0x44;
  
	ret = sendJtag (artix, tx_buf, rx_buf, sizeof(tx_buf));
/*	
	for ( i=0; i<sizeof(tx_buf); i=i+16)
	{
		applog(LOG_ERR, "-> %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", tx_buf[i], tx_buf[i+1],  tx_buf[i+2], tx_buf[i+3], tx_buf[i+4], tx_buf[i+5], tx_buf[i+6], tx_buf[i+7], tx_buf[i+8], tx_buf[i+9], tx_buf[i+10], tx_buf[i+11], tx_buf[i+12], tx_buf[i+13], tx_buf[i+14], tx_buf[i+15]);
	}
*/

/*
	for ( i=0; i<ret; i=i+16)
	{
		applog(LOG_ERR, "-> %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", rx_buf[i], rx_buf[i+1],  rx_buf[i+2], rx_buf[i+3], rx_buf[i+4], rx_buf[i+5], rx_buf[i+6], rx_buf[i+7], rx_buf[i+8], rx_buf[i+9], rx_buf[i+10], rx_buf[i+11], rx_buf[i+12], rx_buf[i+13], rx_buf[i+14], rx_buf[i+15]);
	}
*/
	for (i=0; i<4; i++)
	{
		rx_buf[i] = 0x00;
	}

	for (i=0; i<32; i++)
	{
		rx_buf[i/8] |= ((rx_buf[i+116-chip] & 1) << i%8);
	}			
	return byteArrayToLong(rx_buf);
}

static pthread_mutex_t artix_mutex;

static void artix_detect()
{
	int fd;
	int ret = 0;
	uint8_t mode = 0;
	uint32_t i, y;
	const char *fwfile = BITSTREAM_FILENAME;
	struct cgpu_info *artix;
	uint8_t bits = SPI_BUS_BITS;
	uint32_t speed = SPI_BUS_SPEED;
	
	fd = open(device, O_RDWR);
	if (fd <0) return;
	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
	        applog(LOG_ERR,"can't set spi mode");
	 				return;       
	}
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) {
	        applog(LOG_ERR,"can't get spi mode");
	 				return;       
	}
	
	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
	        applog(LOG_ERR,"can't set bits per word");
	 				return;       
	}
	
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) {
	        applog(LOG_ERR,"can't get bits per word");
	 				return;       
	}
	
	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
	        applog(LOG_ERR,"can't set max speed hz");
	 				return;       
	}
	
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
	        applog(LOG_ERR,"can't get max speed hz");
	 				return;       
	}

        uint8_t tx[1028] = {0, };
        uint8_t rx[1028] = {0, };
        uint32_t txlen;
        uint32_t fileLen;
        unsigned char *buffer;
        uint32_t bufferPointer;
        int f;
        uint32_t blocksize;
        txlen = 11;
        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = txlen,
                .delay_usecs = SPI_BUS_DELAY,
                .speed_hz = SPI_BUS_SPEED,
                .bits_per_word = SPI_BUS_BITS,
        };
        tx[0] = 0x00;
        tx[1] = 0x01;
        tx[2] = 0x02;
        tx[3] = 0x07; // Bank Select
        tx[4] = 0xff;
        tx[5] = 0x03;
        tx[6] = 0x09; // CLR_PROGB
        tx[7] = 0x08; // Set PROGB
        tx[8] = 0x05; // Read Select Bank 8&9
        tx[9] = 0x04; // Read Select Bank 0-7
        tx[10] = 0x00;

        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1) {
                applog(LOG_ERR,"can't send spi message");
                return;
				}
				FILE *fp = open_bitstream("artix", fwfile);
        if (fp == NULL) {
                applog(LOG_ERR,"Couldn't open the File.\n");
                return;
        }

        fseek(fp, 0, SEEK_END);
        fileLen=ftell(fp);
        fseek(fp, 0, SEEK_SET);
        buffer=(char *)malloc(fileLen+1);
        if (!buffer)    applog(LOG_ERR,"Memoryerror while loading Firmware\n");
        f = fread(buffer, fileLen, 1, fp);
        fclose(fp);

        bufferPointer = 0;
        bufferPointer += (buffer[bufferPointer] * 256) + buffer[bufferPointer+1] + 2;
        bufferPointer += (buffer[bufferPointer] * 256) + buffer[bufferPointer+1] + 2;
        bufferPointer += (buffer[bufferPointer] * 256) + buffer[bufferPointer+1] + 2;
        bufferPointer += (buffer[bufferPointer+1] * 256) + buffer[bufferPointer+2] + 2 + 1;
        bufferPointer += (buffer[bufferPointer+1] * 256) + buffer[bufferPointer+2] + 2 + 1;
        bufferPointer += (buffer[bufferPointer+1] * 256) + buffer[bufferPointer+2] + 2 + 1;
        if (buffer[bufferPointer] != 0x65) applog(LOG_ERR,"Couldn't find binary start\n");
        bufferPointer += 5;
				applog(LOG_ERR, "Loading Artixboards with Bitstream");
        while (bufferPointer != fileLen) {
                if (bufferPointer+256 > fileLen) {
                        blocksize = fileLen-bufferPointer;
                } else {
                        blocksize = 256;
                }
                tx[0] = 0x0A;
                for (ret = 0; ret < blocksize; ret++ ) {
                        tx[1+ret] = buffer[bufferPointer+ret];
                }
                tr.len = blocksize + 1;
                ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
                if (ret < 1){
                	applog(LOG_ERR,"can't send spi message");
                	return;
								}
                bufferPointer += blocksize;
        }
        free(buffer);
	
  mutex_init(&artix_mutex);	
	mutex_lock(&artix_mutex);

	for (i=0; i<10; i++)
	{
		tx[0] = 0x07; // Bank Select	
		y = 1<<i;
		tx[1] = y & 255;
		tx[2] = (y>>8) & 3;
		tx[3] = 0x01; // Read Status
		tx[4] = 0xFF;
		tr.len = 5;
		ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

		applog(LOG_ERR, "Slot %d Status: %02x", i, rx[4]); 
		if ((rx[4] & 2) == 2) {
			artix = calloc(1, sizeof(*artix));
			artix->drv = &artix_drv;
//			mutex_init(artix_mutex);
			artix->device_fd = fd;
			artix->deven = DEV_ENABLED;
			artix->threads = 8;
		  artix->deviceIndex = -1;                           
		  artix->tms = false;
		  artix->tdo = false;                 
		  artix->slot = i;
		  artix->numDevices = getChain(artix, true);        
			add_cgpu(artix);	
 			getUsercode(artix);
			// TestCode
//			SelectDevice(artix, 7);
//			applog (LOG_ERR, "a %08x", peek(artix, 0x0f));
			uint32_t dev;
			resetJtag(artix);
/*			
			for (y=0; y<16; y++)
			{
				for (dev=0; dev<8; dev++)
				{
					applog (LOG_ERR, "Chip: %d REG:%02x = %08x", dev, y, readRegJtag(artix, dev, y));
				}
			}
*/
		}
	}
	mutex_unlock(&artix_mutex);

}

static int64_t artix_scanhash(struct thr_info *thr, struct work *work, int64_t __maybe_unused max_nonce)
{
	struct cgpu_info *artix = thr->cgpu;
	uint32_t nonce, lastnonce, errorcounter, state;
  uint8_t tx[16] = {0, };
  uint8_t rx[16] = {0, };
	static bool first_run = true;
	if (first_run) {
		usleep(50000*thr->id);
		first_run = false;
		//applog(LOG_ERR,"ART%d:%d First Run", artix->device_id, thr->id % 8);	
	}
	
  struct spi_ioc_transfer tr = {
          .tx_buf = (unsigned long)tx,
          .rx_buf = (unsigned long)rx,
          .len = 0,
          .delay_usecs = SPI_BUS_DELAY,
          .speed_hz = SPI_BUS_SPEED,
          .bits_per_word = SPI_BUS_BITS,
  };              
	uint32_t i;
	int ret;
	errorcounter = 0;
	tx[0] = 0x07; // Bank Select	
	tx[1] = (1<<artix->slot) & 255;
	tx[2] = ((1<<artix->slot)>>8) & 3;
	tx[3] = 0x01; // Read Status
	tx[4] = 0xFF;
	tr.len = 5;
	mutex_lock(&artix_mutex);
	ret = ioctl(artix->device_fd, SPI_IOC_MESSAGE(1), &tr);
//  applog(LOG_ERR,"ART%d:%d Loading Work", artix->device_id, thr->id % 8);
	SelectDevice(artix, thr->id % 8);
  poke(artix, 0x01, byteArrayToLong(&work->midstate[0]));
  poke(artix, 0x02, byteArrayToLong(&work->midstate[4]));
  poke(artix, 0x03, byteArrayToLong(&work->midstate[8]));
  poke(artix, 0x04, byteArrayToLong(&work->midstate[12]));
  poke(artix, 0x05, byteArrayToLong(&work->midstate[16]));
  poke(artix, 0x06, byteArrayToLong(&work->midstate[20]));
  poke(artix, 0x07, byteArrayToLong(&work->midstate[24]));
  poke(artix, 0x08, byteArrayToLong(&work->midstate[28]));

  poke(artix, 0x09, byteArrayToLong(&work->data[64]));
  poke(artix, 0x0A, byteArrayToLong(&work->data[68]));
  poke(artix, 0x0B, byteArrayToLong(&work->data[72]));
	
	mutex_unlock(&artix_mutex);
	usleep(10000000);
	lastnonce = 0;
	while(1) {
		mutex_lock(&artix_mutex);
		ret = ioctl(artix->device_fd, SPI_IOC_MESSAGE(1), &tr);
		resetJtag(artix);
//		SelectDevice(artix, thr->id % 8);
		nonce = readRegJtag(artix, thr->id % 8, 0x0f);

		if (((nonce & 1) == 0) && (((nonce >> 16) & 7) == (thr->id % 8))){ // finished work
  //		  applog(LOG_ERR,"ART%d:%d = %08x", artix->device_id, thr->id % 8, nonce);

				nonce = readRegJtag(artix, thr->id % 8, 0x0e);
				while (nonce != 0xFFFFFFFF) {
//					applog(LOG_ERR,"ART%d:%d = %08x", artix->device_id, thr->id % 8, nonce);

					if ((nonce != 0x00000000) && (nonce != 0x7FFFFFFF)){
						if (!thr->work_restart) {
							if (submit_nonce(thr, work, nonce)) {	
								artix->fpga_status[thr->id % 8] = 1;
							} else {
								artix->fpga_status[thr->id % 8] = 0;
							}
						}
					} else {
						errorcounter ++;
						artix->fpga_status[thr->id % 8] = 0;
					}
					nonce = readRegJtag(artix, thr->id % 8, 0x0e);
					if (errorcounter >= 128) {
						artix->fpga_status[thr->id % 8] = 0;
						nonce =0xFFFFFFFF;
						applog(LOG_ERR,"ART%d:%d Zero Nonce", artix->device_id, thr->id % 8);
						
					}
				}
				mutex_unlock(&artix_mutex);
				break;
		} else {
			mutex_unlock(&artix_mutex);
//			applog(LOG_ERR, "ART%d: Lastnonce: %08x - %08x", artix->device_id, lastnonce, nonce);
			usleep(1000);	
			lastnonce = nonce;
		}
	}
  
//	applog(LOG_ERR, "ART%d: Block finished", artix->device_id);
  return 0xFFFFFFFF;
}

static bool artix_thread_prepare(struct thr_info *thr)
{
	return true;
}

static void artix_statline_before(char *buf, struct cgpu_info *artix)
{
	char info[64];

	sprintf(info, " %d-%d-%d-%d-%d-%d-%d-%d  | ",
			artix->fpga_status[0], 
			artix->fpga_status[1], 
			artix->fpga_status[2], 
			artix->fpga_status[3], 
			artix->fpga_status[4], 
			artix->fpga_status[5], 
			artix->fpga_status[6], 
			artix->fpga_status[7] );

	strcat(buf, info);
}

struct device_drv artix_drv = {
	.dname = "artix",
	.name = "ART",
	.drv_detect = artix_detect,
	.thread_prepare = artix_thread_prepare,
	.get_statline_before = artix_statline_before,
	.scanhash = artix_scanhash,

};
