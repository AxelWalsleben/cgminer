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
  
void tx_tms(struct cgpu_info *artix, unsigned char *pat, int length, int force)
{
	uint32_t ret;
	applog(LOG_ERR,"tx_tms");
	struct spi_ioc_transfer tr = {
	        .tx_buf = (unsigned long)artix->jtag_buf_tx,
	        .rx_buf = (unsigned long)artix->jtag_buf_rx,
	        .len = artix->jtag_len,
	        .delay_usecs = SPI_BUS_DELAY,
	        .speed_hz = SPI_BUS_SPEED,
	        .bits_per_word = SPI_BUS_BITS,
	};	
	ret = ioctl(artix->device_fd, SPI_IOC_MESSAGE(1), &tr);
  memset(artix->jtag_buf_tx,  0x88, CHUNK_SIZE);
  artix->jtag_buf_tx[0] = 0x10; // Add JTAG CMD Header
  artix->jtag_len = 1;
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
  flush_tms(artix, true);
  for(i=0; i<5; i++)
      set_tms(artix, true);
  artix->current_state=TEST_LOGIC_RESET;
 
}

void setTapState(struct cgpu_info *artix, enum tapState_t state, int pre)
{
  bool tms;
  while(artix->current_state!=state){
    switch(artix->current_state){

    case TEST_LOGIC_RESET:
      switch(state){
      case TEST_LOGIC_RESET:
					tms=true;
					break;
      default:
					tms=false;
					artix->current_state=RUN_TEST_IDLE;
      };
      break;

    case RUN_TEST_IDLE:
      switch(state){
      case RUN_TEST_IDLE:
	tms=false;
	break;
      default:
	tms=true;
	artix->current_state=SELECT_DR_SCAN;
      };
      break;

    case SELECT_DR_SCAN:
      switch(state){
      case CAPTURE_DR:
      case SHIFT_DR:
      case EXIT1_DR:
      case PAUSE_DR:
      case EXIT2_DR:
      case UPDATE_DR:
	tms=false;
	artix->current_state=CAPTURE_DR;
	break;
      default:
	tms=true;
	artix->current_state=SELECT_IR_SCAN;
      };
      break;

    case CAPTURE_DR:
      switch(state){
      case SHIFT_DR:
				tms=false;
				artix->current_state=SHIFT_DR;
				break;
      default:
				tms=true;
				artix->current_state=EXIT1_DR;
      };
      break;

    case SHIFT_DR:
      switch(state){
      case SHIFT_DR:
				tms=false;
				break;
      default:
				tms=true;
				artix->current_state=EXIT1_DR;
      };
      break;

    case EXIT1_DR:
      switch(state){
      case PAUSE_DR:
      case EXIT2_DR:
      case SHIFT_DR:
      case EXIT1_DR:
				tms=false;
				artix->current_state=PAUSE_DR;
				break;
      default:
				tms=true;
				artix->current_state=UPDATE_DR;
      };
      break;

    case PAUSE_DR:
      switch(state){
      case PAUSE_DR:
	tms=false;
	break;
      default:
	tms=true;
	artix->current_state=EXIT2_DR;
      };
      break;

    case EXIT2_DR:
      switch(state){
      case SHIFT_DR:
      case EXIT1_DR:
      case PAUSE_DR:
	tms=false;
	artix->current_state=SHIFT_DR;
	break;
      default:
	tms=true;
	artix->current_state=UPDATE_DR;
      };
      break;

    case UPDATE_DR:
      switch(state){
      case RUN_TEST_IDLE:
	tms=false;
	artix->current_state=RUN_TEST_IDLE;
	break;
      default:
	tms=true;
	artix->current_state=SELECT_DR_SCAN;
      };
      break;

    case SELECT_IR_SCAN:
      switch(state){
      case CAPTURE_IR:
      case SHIFT_IR:
      case EXIT1_IR:
      case PAUSE_IR:
      case EXIT2_IR:
      case UPDATE_IR:
	tms=false;
	artix->current_state=CAPTURE_IR;
	break;
      default:
	tms=true;
	artix->current_state=TEST_LOGIC_RESET;
      };
      break;

    case CAPTURE_IR:
      switch(state){
      case SHIFT_IR:
	tms=false;
	artix->current_state=SHIFT_IR;
	break;
      default:
	tms=true;
	artix->current_state=EXIT1_IR;
      };
      break;

    case SHIFT_IR:
      switch(state){
      case SHIFT_IR:
	tms=false;
	break;
      default:
	tms=true;
	artix->current_state=EXIT1_IR;
      };
      break;

    case EXIT1_IR:
      switch(state){
      case PAUSE_IR:
      case EXIT2_IR:
      case SHIFT_IR:
      case EXIT1_IR:
	tms=false;
	artix->current_state=PAUSE_IR;
	break;
      default:
	tms=true;
	artix->current_state=UPDATE_IR;
      };
      break;

    case PAUSE_IR:
      switch(state){
      case PAUSE_IR:
	tms=false;
	break;
      default:
	tms=true;
	artix->current_state=EXIT2_IR;
      };
      break;

    case EXIT2_IR:
      switch(state){
      case SHIFT_IR:
      case EXIT1_IR:
      case PAUSE_IR:
	tms=false;
	artix->current_state=SHIFT_IR;
	break;
      default:
	tms=true;
	artix->current_state=UPDATE_IR;
      };
      break;

    case UPDATE_IR:
      switch(state){
      case RUN_TEST_IDLE:
	tms=false;
	artix->current_state=RUN_TEST_IDLE;
	break;
      default:
	tms=true;
	artix->current_state=SELECT_DR_SCAN;
      };
      break;

    default:
      tapTestLogicReset(artix);
      tms=true;
    };
    set_tms(artix, tms);
  }
  int i;
  for(i=0; i<pre; i++)
    set_tms(artix, false);
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
		applog(LOG_ERR, "last is set");
		artix->jtag_buf_tx[artix->jtag_len] = 0x88; // Add Dummy Cmd to get last result
		artix->jtag_len++;
	
	for ( i=0; i<artix->jtag_len; i=i+16)
	{
		applog(LOG_ERR, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", artix->jtag_buf_tx[i], artix->jtag_buf_tx[i+1],  artix->jtag_buf_tx[i+2], artix->jtag_buf_tx[i+3], artix->jtag_buf_tx[i+4], artix->jtag_buf_tx[i+5], artix->jtag_buf_tx[i+6], artix->jtag_buf_tx[i+7], artix->jtag_buf_tx[i+8], artix->jtag_buf_tx[i+9], artix->jtag_buf_tx[i+10], artix->jtag_buf_tx[i+11], artix->jtag_buf_tx[i+12], artix->jtag_buf_tx[i+13], artix->jtag_buf_tx[i+14], artix->jtag_buf_tx[i+15]);
	}
	
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

	for ( i=0; i<ret; i=i+16)
	{
		applog(LOG_ERR, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", artix->jtag_buf_rx[i], artix->jtag_buf_rx[i+1],  artix->jtag_buf_rx[i+2], artix->jtag_buf_rx[i+3], artix->jtag_buf_rx[i+4], artix->jtag_buf_rx[i+5], artix->jtag_buf_rx[i+6], artix->jtag_buf_rx[i+7], artix->jtag_buf_rx[i+8], artix->jtag_buf_rx[i+9], artix->jtag_buf_rx[i+10], artix->jtag_buf_rx[i+11], artix->jtag_buf_rx[i+12], artix->jtag_buf_rx[i+13], artix->jtag_buf_rx[i+14], artix->jtag_buf_rx[i+15]);
	}
	
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

// TDI gets a load of zeros, we just record TDO.
void shiftTDO(struct cgpu_info *artix, unsigned char *tdo, int length, bool last)
{
    shiftTDITDO(artix, NULL, tdo, length,last);
}

  
  
/* Detect chain length on first start, return chain length else*/
int getChain(struct cgpu_info *artix, bool detect)
{
  int i;
  if(artix->numDevices  == -1 || detect)
    {
      tapTestLogicReset(artix);
      setTapState(artix, SHIFT_DR, 0);
      byte idx[4];
      byte zero[4];
      artix->numDevices=0;
      for(i=0; i<4; i++)zero[i]=0;
      do{
					shiftTDITDO(artix, zero,idx,32,true);
					unsigned long id=byteArrayToLong(idx);
					applog(LOG_ERR, "Getchain %d, %08x", artix->numDevices, id);
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
      setTapState(artix, TEST_LOGIC_RESET, 0);
    }
  applog(LOG_ERR, "getChain found %d devices",artix->numDevices);
  return artix->numDevices;
}

int SelectDevice(struct cgpu_info *artix, int dev) {
  if(dev>=artix->numDevices)artix->deviceIndex=-1;
  	else artix->deviceIndex=dev;
	applog(LOG_ERR,"selectDevice %d", artix->deviceIndex);
  return artix->deviceIndex;	
}

void nextTapState(struct cgpu_info *artix, bool tms)
{
  if(artix->current_state==SHIFT_DR){
    if(tms)artix->current_state=EXIT1_DR; // If TMS was set then goto next state
  }
  else if(artix->current_state==SHIFT_IR){
    if(tms)artix->current_state=EXIT1_IR; // If TMS was set then goto next state
  }
  else 
    {
      applog(LOG_ERR,"Unexpected state %d",artix->current_state);
      tapTestLogicReset(artix); // We were in an unexpected state
    }
}

// TDI gets a load of zeros or ones, and we ignore TDO
void shift(struct cgpu_info *artix, bool tdi, int length, bool last)
{
    int len = length;
    unsigned char *block = (tdi)?ones:zeros;
    applog(LOG_ERR, "shift");
//    flush_tms(artix, false);
    while (len > CHUNK_SIZE)
    {
			shiftTDITDO(artix, block, NULL, CHUNK_SIZE, false);
			len -= (CHUNK_SIZE);
    }
    shiftTDITDO(artix, block, NULL, len, last);
}

void shiftDR(struct cgpu_info *artix, const byte *tdi, byte *tdo, int length,
		   int align, bool exit)
{
  applog(LOG_ERR, "shiftDR");
  if(artix->deviceIndex<0)return;
  int post=artix->deviceIndex;

  if(!artix->shiftDRincomplete){
    int pre=artix->numDevices-artix->deviceIndex-1;
    if(align){
      pre=-post;
      while(pre<=0)pre+=align;
    }
    /* We can combine the pre bits to reach the target device with
     the TMS bits to reach the SHIFT-DR state, as the pre bit can be '0'*/
    setTapState(artix, SHIFT_DR,pre);
  }

  if(tdi!=0&&tdo!=0) shiftTDITDO(artix, tdi,tdo,length,false);
  else if(tdi!=0&&tdo==0) shiftTDI(artix, tdi,length,false);
  else if(tdi==0&&tdo!=0) shiftTDO(artix, tdo,length,false);
  else  shift(artix, false,length,false);

	shiftTDITDO(artix, NULL,tdo,length,true);
  nextTapState(artix, false); // If TMS is set the the state of the tap changes
  
  if(exit){
     shift(artix, false,post, true);
    if (!(post==0&&exit))
      nextTapState(artix, true);
    setTapState(artix, artix->postDRState, 0);
    artix->shiftDRincomplete=false;
  }
  else artix->shiftDRincomplete=true;
}


void shiftIR(struct cgpu_info *artix, const byte *tdi, byte *tdo)
{
  applog(LOG_ERR, "shiftIR");
  if(artix->deviceIndex<0)return;
  setTapState(artix, SHIFT_IR, 0);

  int pre=0;
  int dev=0;
  for(dev=artix->deviceIndex+1; dev<artix->numDevices; dev++)
    pre+= 6; // Calculate number of pre BYPASS bits.
  int post=0;
  for(dev=0; dev<artix->deviceIndex; dev++)
    post+= 6; // Calculate number of post BYPASS bits.
  shift(artix, true,pre,false);
  if(tdo!=0) shiftTDITDO(artix, tdi,tdo, 6, false);
    else if(tdo==0) shiftTDI(artix, tdi, 6, false);
  shift(artix, true,post, false);
  nextTapState(artix, true);
  setTapState(artix, artix->postIRState, 0);
}

void poke(struct cgpu_info *artix, int addr, long value) {
        byte mydata[8];
        byte checksum;
//        applog(LOG_DEBUG, "Poke %d : %08x)", addr, value);
        mydata[0] = 0x02;
        shiftIR(artix, mydata, 0);
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
        shiftDR(artix, mydata, NULL, 38, 0, true);
}

long peek(struct cgpu_info *artix, int addr) {
        byte mydata[8];
//	applog(LOG_DEBUG, "Peek %d", addr);
        mydata[0] = 0x02;
        shiftIR(artix, mydata, 0);
        mydata[0] = 0X00;
        mydata[1] = 0x00;
        mydata[2] = 0x00;
        mydata[3] = 0x00;
        mydata[4] = (addr & 0x0f) | ((1 ^ ((addr >> 3) & 1) ^ ((addr >> 2) & 1) ^ ((addr >> 1) & 1) ^ (addr & 1) ) << 5 );
      
        shiftDR(artix, mydata, 0, 38, 0, true);
        shiftDR(artix, 0, mydata, 32, 0, true);
        return byteArrayToLong(mydata);
}

void myPoke(struct cgpu_info *artix, int addr, long value) {
        byte mydata[512];
        byte checksum;
				uint32_t i, dev;

        applog(LOG_DEBUG, "Poke %d : %08x)", addr, value);
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
	shiftTDITDO(artix, mydata, NULL, 38, false);	
	set_lasttms(artix);
	set_tms(artix, true); // zum Update-DR
	set_tms(artix, false); // zum Run-Test-Idle

	
	shiftTDITDO(artix, NULL, mydata, 8, true);	

}


long myPeek(struct cgpu_info *artix, int addr) {
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
	shiftTDITDO(artix, NULL, mydata, 33, true);	
	
	return byteArrayToLong(mydata);    ;
}

long getUsercode(struct cgpu_info *artix) {
	applog(LOG_ERR, "getUSercode");
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
        return byteArrayToLong(mydata);
}

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
			mutex_init(&artix->device_mutex);
			artix->device_fd = fd;
			artix->deven = DEV_ENABLED;
			artix->threads = 1;
		  artix->current_state = UNKNOWN_STATE;        
		  artix->postDRState = RUN_TEST_IDLE;    
		  artix->postIRState = RUN_TEST_IDLE;    
		  artix->deviceIndex = -1;                           
		  artix->shiftDRincomplete = false;                 
		  artix->tms = false;
		  artix->tdo = false;                 
		  artix->numDevices = getChain(artix, true);        
			add_cgpu(artix);	
			SelectDevice(artix, 0);
			applog (LOG_ERR, "ID: %d has Code %08x ", 0, getUsercode(artix));
			for (y=0; y<8; y++)
			{
				SelectDevice(artix, y);
				myPoke(artix, 0x02, 0xFFFFFFFF);
				applog (LOG_ERR, "myPeek: %d has Value %08x", y, myPeek(artix, 0x02));
			}
		}
	}
}

static bool artix_thread_prepare(struct thr_info *thr)
{
	struct cgpu_info *artix = thr->cgpu;
	struct timeval now;
        
	cgtime(&now);
	get_datestamp(artix->init, &now);
  return true;
}

static int64_t artix_scanhash(struct thr_info *thr)
{
	struct cgpu_info *artix = thr->cgpu;
	struct work **works;

  return 0x01;
}

static void artix_fpga_shutdown(struct thr_info *thr)
{
	struct cgpu_info *artix = thr->cgpu;

//	free(thr->cgpu_data);
}

static void artix_reinit(struct cgpu_info *artix)
{

}

struct device_drv artix_drv = {
	.dname = "artix",
	.name = "ART",
	.drv_detect = artix_detect,
	.thread_prepare = artix_thread_prepare,
	.hash_work = hash_queued_work,
	.scanwork = artix_scanhash,
	.reinit_device = artix_reinit,
	.thread_shutdown = artix_fpga_shutdown,

};
