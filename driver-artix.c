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

#define MAXnumDevices 1000

#define BLOCK_SIZE 65536
#define CHUNK_SIZE 128
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
  
void mpsse_send(struct cgpu_info *artix) {

}

void mpsse_add_cmd(struct cgpu_info *artix, unsigned char const *const buf, int const len) {
 /* The TX FIFO has 128 Byte. It can easily be overrun
    So send only chunks of the TX Buffersize and hope
    that the OS USB scheduler gives the MPSSE machine 
    enough time empty the buffer
 */
 if (artix->bptr + len +1 >= TX_BUF)
//   mpsse_send(artix);
  memcpy(artix->usbuf + artix->bptr, buf, len);
  artix->bptr += len;
}

void tx_tms(struct cgpu_info *artix, unsigned char *pat, int length, int force)
{

}

void flush_tms(struct cgpu_info *artix, int force)
{
  if (artix->tms_len)
    tx_tms(artix, artix->tms_buf, artix->tms_len, force);
  memset(artix->tms_buf,   0,CHUNK_SIZE);
  artix->tms_len = 0;
}

void set_tms(struct cgpu_info *artix, bool val)
{
  if( artix->tms_len + 1 > CHUNK_SIZE*8)
    flush_tms(artix, false);
  if(val)
    artix->tms_buf[artix->tms_len/8] |= (1 <<(artix->tms_len &0x7));
  artix->tms_len++;
}

void tapTestLogicReset(struct cgpu_info *artix)
{
  int i;
  for(i=0; i<5; i++)
      set_tms(artix, true);
  artix->current_state=TEST_LOGIC_RESET;
  flush_tms(artix, true);
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

unsigned int readusb(struct cgpu_info *artix, unsigned char * rbuf, unsigned long len)
{

}


void txrx_block(struct cgpu_info *artix, const unsigned char *tdi, unsigned char *tdo,
			int length, bool last)
{

}


void shiftTDITDO(struct cgpu_info *artix, const unsigned char *tdi, unsigned char *tdo,
			 int length, bool last)
{
  if(length==0) return;
  flush_tms(artix, false);
  txrx_block(artix, tdi, tdo, length,last);
  return;
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
					shiftTDITDO(artix, zero,idx,32,false);
					unsigned long id=byteArrayToLong(idx);
					if(id!=0 && id !=0xffffffff){
					  artix->numDevices++;
//					  chainParam_t dev;
//					  dev.idcode=id;
//					  devices.insert(devices.begin(),dev);
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
	applog(LOG_DEBUG,"selectDevices %d", artix->deviceIndex);
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
    flush_tms(artix, false);
    while (len > CHUNK_SIZE*8)
    {
	txrx_block(artix, block, NULL, CHUNK_SIZE*8, false);
	len -= (CHUNK_SIZE*8);
    }
    shiftTDITDO(artix, block, NULL, len, last);
}

void shiftDR(struct cgpu_info *artix, const byte *tdi, byte *tdo, int length,
		   int align, bool exit)
{
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

  if(tdi!=0&&tdo!=0) shiftTDITDO(artix, tdi,tdo,length,post==0&&exit);
  else if(tdi!=0&&tdo==0) shiftTDI(artix, tdi,length,post==0&&exit);
  else if(tdi==0&&tdo!=0) shiftTDO(artix, tdo,length,post==0&&exit);
  else  shift(artix, false,length,post==0&&exit);

  nextTapState(artix, post==0&&exit); // If TMS is set the the state of the tap changes
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
  if(tdo!=0) shiftTDITDO(artix, tdi,tdo, 6,post==0);
    else if(tdo==0) shiftTDI(artix, tdi, 6,post==0);
  shift(artix, true,post, true);
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

long getUsercode(struct cgpu_info *artix) {
        byte mydata[8];
        mydata[0] = 0x08;
        shiftIR(artix, mydata, 0);
        shiftDR(artix, 0, mydata, 32, 0, true);
        return byteArrayToLong(mydata);
}

static void artix_detect()
{
	int fd;
	int ret = 0;
	uint8_t mode = 0;
	uint8_t bits = 8;
	uint32_t speed = 5000000;
	uint16_t delay = 0;
	uint32_t i, y;
	const char *fwfile = BITSTREAM_FILENAME;
	struct cgpu_info *artix;
	
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
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
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
		  artix->numDevices  = -1;  
		  artix->chainlength = 8;                         
			add_cgpu(artix);	
		}
	}	
}

static bool artix_thread_prepare(struct thr_info *thr)
{
	struct cgpu_info *artix = thr->cgpu;
        byte mydata [8];
        int i, y, len;
	struct timeval now;
        unsigned char buf[0x100];
        
	artix->works = calloc(2 * artix->chainlength * sizeof(struct work *), 4); // AVALON_ARRAY_SIZE=4
        artix->work_array = 0;
	cgtime(&now);
	get_datestamp(artix->init, &now);
        for (i=0; i<artix->chainlength; i++) {
            SelectDevice(artix, i);
            artix->nonces[i] = peek(artix, 0x0c);
//            poke(artix, 0x0d, 175);
            artix->clocks[i] = peek(artix, 0x0d);
            artix->workingon[i] = false;
            artix->has_restart[i] = true;
            artix->is_disabled[i] = true;
            artix->numnonces[i] = 0;
            artix->errorcount[i] = 0;
            for (y=0; y<250; y++) artix->errorrate[i][y] = 0;
            applog (LOG_ERR, "ID: %d has Code %08x Clockrate: %08x", i, getUsercode(artix), peek(artix, 0x0d));
            
            
        }

  return true;
}

static int64_t artix_scanhash(struct thr_info *thr)
{
	struct cgpu_info *artix = thr->cgpu;
	struct work **works;
        int i, tries;
	uint32_t nonce , lastnonce;
	int64_t hash_count = 0;
        static int first_try = 0;
        int y= 0;
 	works = artix->works;
        int start_count = 0;
	for (i=0; i<artix->chainlength; i++) {

		usleep(20000);
            SelectDevice(artix, i);
            if (thr->work_restart) {
                for (tries=0; tries<(artix->chainlength*2); tries++) {
                    works[tries]->devflag = true;
                    artix->has_restart[tries] = true;
                }
                break;
            } else {
                if (artix->is_disabled[i]) {

                    if (artix->workingon[i] == false) {
                        start_count = 0;
                    } else {
                        start_count = artix->chainlength;
                    }

                    nonce = 0x00;
                    tries = 0;
                    lastnonce = 0xFFFFFFFF;
                    while (nonce != 0xFFFFFFFF) {
                        nonce = peek(artix, 0x0e);
// 			applog(LOG_ERR, "nonce %d-%d %08x",i,i,nonce);
                        if ((nonce != 0x00) && ((nonce & 0xFFFFFFF0) != 0xFFFFFFF0) && (first_try != 0)){
                            if (!artix->has_restart[i]) {
                                if (y == 0xFFFFFFFF) {
                                    artix->errorcount[i]++;
                                } else {
                                    artix->numnonces[i]++;
                                }
                                if (!submit_nonce(thr, works[start_count+i], nonce)) {
                                    if (lastnonce == nonce) {
                                        artix->is_disabled[i] = false;
                                        tries = 100;
                                        applog(LOG_ERR, "Disabling Device: %d", i);
                                    }
                                } else {
                                    tries = 0;
                                    
                                }
                                lastnonce = nonce;
                            }
                        }
                        tries++;
                        if(tries > 10) break;
                    }

                    nonce = peek(artix, 0x0c);
		    if (i==7) applog(LOG_ERR,"nonce %d: %08x",i, nonce);
                    if (nonce > artix->nonces[i]) hash_count += (nonce - artix->nonces[i]);
                    if (nonce < artix->nonces[i]) hash_count += (nonce + (0xFFFFFFFF - artix->nonces[i]));
                    artix->nonces[i] = nonce;
//		    applog(LOG_ERR, "State %d:%08x", i, peek(artix, 0x0F));
                    if ((peek(artix, 0x0F) & 0x01) == 0) {

                        artix->has_restart[i] = false;
                        applog(LOG_DEBUG, "Loading new Work in %d", i);
                        // Needs new work !
                        works[start_count+i]->devflag = true;
                        works[start_count+i]->blk.nonce = 0xffffffff;
                        artix->workingon[i] = !artix->workingon[i];
                        if (artix->workingon[i] == false) {
                            start_count = 0;
                        } else {
                            start_count = artix->chainlength;
                        }
                        works[start_count+i]->devflag = false;
			nonce = peek(artix, 0x0c);
			applog(LOG_ERR, "Startnonce: %d: %08x", i, nonce);
                        poke(artix, 0x01, byteArrayToLong(&works[start_count+i]->midstate[0]));
                        poke(artix, 0x02, byteArrayToLong(&works[start_count+i]->midstate[4]));
                        poke(artix, 0x03, byteArrayToLong(&works[start_count+i]->midstate[8]));
                        poke(artix, 0x04, byteArrayToLong(&works[start_count+i]->midstate[12]));
                        poke(artix, 0x05, byteArrayToLong(&works[start_count+i]->midstate[16]));
                        poke(artix, 0x06, byteArrayToLong(&works[start_count+i]->midstate[20]));
                        poke(artix, 0x07, byteArrayToLong(&works[start_count+i]->midstate[24]));
                        poke(artix, 0x08, byteArrayToLong(&works[start_count+i]->midstate[28]));

                        poke(artix, 0x09, byteArrayToLong(&works[start_count+i]->data[64]));
                        poke(artix, 0x0A, byteArrayToLong(&works[start_count+i]->data[68]));
                        poke(artix, 0x0B, byteArrayToLong(&works[start_count+i]->data[72]));
                        works[start_count+i]->subid = i;
                    }
                }
            }
            	}
        first_try = 1;

  return hash_count;
}

static void artix_fpga_shutdown(struct thr_info *thr)
{
	struct cgpu_info *artix = thr->cgpu;

//	free(thr->cgpu_data);
}

static void artix_reinit(struct cgpu_info *artix)
{

}

/* We use a replacement algorithm to only remove references to work done from
 * the buffer when we need the extra space for new work. */
static bool artix_fill(struct cgpu_info *artix)
{
        int i;
        struct work *work;
        work = get_queued(artix);
        if (artix->queued < (artix->chainlength * 2)) {
            artix->works[artix->queued] = work;
            artix->queued++;
            return false;
        }
            
        for (i=0; i< (artix->chainlength * 2); i++) {
            if (artix->works[i]->devflag) {
                work_completed(artix, artix->works[i]);
                artix->works[i] = work;
                return false;
            }
        }
        return true;
}

struct device_drv artix_drv = {
	.dname = "artix",
	.name = "ART",
	.drv_detect = artix_detect,
	.thread_prepare = artix_thread_prepare,
	.hash_work = hash_queued_work,
	.queue_full = artix_fill,
	.scanwork = artix_scanhash,
	.reinit_device = artix_reinit,
	.thread_shutdown = artix_fpga_shutdown,

};
