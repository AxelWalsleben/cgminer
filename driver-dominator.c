/*
 * Copyright 2012 Andrew Smith
 * Copyright 2012 Luke Dashjr
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

#include "ftdi.h"

#define BITSTREAM_FILENAME "dominator.bit"

#define CLK_ADJUST_STEP 5

#define dominator_CUTOFF_TEMP 60.0
#define dominator_OVERHEAT_TEMP 50.0
#define dominator_OVERHEAT_CLOCK -10

#define dominator_HW_ERROR_PERCENT 0.75

#define dominator_MAX_CLOCK 220
#define dominator_DEF_CLOCK 200
#define dominator_MIN_CLOCK 160

#define dominator_CLOCK_DOWN -2
#define dominator_CLOCK_SET 0
#define dominator_CLOCK_UP 2

// Maximum how many good shares in a row means clock up
// 96 is ~34m22s at 200MH/s
#define dominator_TRY_UP 96
// Initially how many good shares in a row means clock up
// This is doubled each down clock until it reaches dominator_TRY_UP
// 6 is ~2m9s at 200MH/s
#define dominator_EARLY_UP 6

struct device_drv dominator_drv;

#define dominator_vendor 0x0403
#define dominator_product 0x6010
#define TX_BUF (4096)

#define MAXnumDevices 1000

#define BLOCK_SIZE 65536
#define CHUNK_SIZE 128
#define TICK_COUNT 2048

typedef unsigned char byte;

static unsigned char ones[CHUNK_SIZE], zeros[CHUNK_SIZE];

/* 
static enum tapState_t dominator->current_state = UNKNOWN;
static enum tapState_t dominator->postDRState = RUN_TEST_IDLE;
static enum tapState_t dominator->postIRState = RUN_TEST_IDLE;
static int dominator->deviceIndex = -1;
static bool dominator->shiftDRincomplete = false;
static int dominator->numDevices  = -1;
*/

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
  
void mpsse_send(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle) {
  	if(dominator->bptr == 0)  return;
    int written = ftdi_write_data(ftdi_handle, dominator->usbuf, dominator->bptr);
    if(written != (int) dominator->bptr) 
      	{
          applog(LOG_ERR,"mpsse_send: Short write %d vs %d, Err: %s", 
                  written, dominator->bptr, ftdi_get_error_string(ftdi_handle));
      	}
  dominator->bptr = 0;
}

void mpsse_add_cmd(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, unsigned char const *const buf, int const len) {
 /* The TX FIFO has 128 Byte. It can easily be overrun
    So send only chunks of the TX Buffersize and hope
    that the OS USB scheduler gives the MPSSE machine 
    enough time empty the buffer
 */
 if (dominator->bptr + len +1 >= TX_BUF)
   mpsse_send(dominator, ftdi_handle);
  memcpy(dominator->usbuf + dominator->bptr, buf, len);
  dominator->bptr += len;
}

void tx_tms(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, unsigned char *pat, int length, int force)
{
    unsigned char buf[3] = {MPSSE_WRITE_TMS|MPSSE_LSB|MPSSE_BITMODE|
			    MPSSE_WRITE_NEG, 0, pat[0]};
    int len = length, i, j=0;
    if (!len)
      return;
    while (len>0)
      {
	/* Attention: Bug in FT2232L(D?, H not!). 
	   With 7 bits TMS shift, static TDO 
	   value gets set to TMS on last TCK edge*/ 
	buf[1] = (len >6)? 5: (len-1);
	buf[2] = 0x80;
	for (i=0; i < (buf[1]+1); i++)
	  {
	    buf[2] |= (((pat[j>>3] & (1<< (j &0x7)))?1:0)<<i);
	    j++;
	  }
	len -=(buf[1]+1);
	mpsse_add_cmd (dominator, ftdi_handle, buf, 3);
      }
    if(force)
      mpsse_send(dominator, ftdi_handle);
}

void flush_tms(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, int force)
{
  if (dominator->tms_len)
    tx_tms(dominator, ftdi_handle, dominator->tms_buf, dominator->tms_len, force);
  memset(dominator->tms_buf,   0,CHUNK_SIZE);
  dominator->tms_len = 0;
}

void set_tms(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, bool val)
{
  if( dominator->tms_len + 1 > CHUNK_SIZE*8)
    flush_tms(dominator, ftdi_handle, false);
  if(val)
    dominator->tms_buf[dominator->tms_len/8] |= (1 <<(dominator->tms_len &0x7));
  dominator->tms_len++;
}

void tapTestLogicReset(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle)
{
  int i;
  for(i=0; i<5; i++)
      set_tms(dominator, ftdi_handle, true);
  dominator->current_state=TEST_LOGIC_RESET;
  flush_tms(dominator, ftdi_handle, true);
}

void setTapState(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, enum tapState_t state, int pre)
{
  bool tms;
  while(dominator->current_state!=state){
    switch(dominator->current_state){

    case TEST_LOGIC_RESET:
      switch(state){
      case TEST_LOGIC_RESET:
	tms=true;
	break;
      default:
	tms=false;
	dominator->current_state=RUN_TEST_IDLE;
      };
      break;

    case RUN_TEST_IDLE:
      switch(state){
      case RUN_TEST_IDLE:
	tms=false;
	break;
      default:
	tms=true;
	dominator->current_state=SELECT_DR_SCAN;
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
	dominator->current_state=CAPTURE_DR;
	break;
      default:
	tms=true;
	dominator->current_state=SELECT_IR_SCAN;
      };
      break;

    case CAPTURE_DR:
      switch(state){
      case SHIFT_DR:
	tms=false;
	dominator->current_state=SHIFT_DR;
	break;
      default:
	tms=true;
	dominator->current_state=EXIT1_DR;
      };
      break;

    case SHIFT_DR:
      switch(state){
      case SHIFT_DR:
	tms=false;
	break;
      default:
	tms=true;
	dominator->current_state=EXIT1_DR;
      };
      break;

    case EXIT1_DR:
      switch(state){
      case PAUSE_DR:
      case EXIT2_DR:
      case SHIFT_DR:
      case EXIT1_DR:
	tms=false;
	dominator->current_state=PAUSE_DR;
	break;
      default:
	tms=true;
	dominator->current_state=UPDATE_DR;
      };
      break;

    case PAUSE_DR:
      switch(state){
      case PAUSE_DR:
	tms=false;
	break;
      default:
	tms=true;
	dominator->current_state=EXIT2_DR;
      };
      break;

    case EXIT2_DR:
      switch(state){
      case SHIFT_DR:
      case EXIT1_DR:
      case PAUSE_DR:
	tms=false;
	dominator->current_state=SHIFT_DR;
	break;
      default:
	tms=true;
	dominator->current_state=UPDATE_DR;
      };
      break;

    case UPDATE_DR:
      switch(state){
      case RUN_TEST_IDLE:
	tms=false;
	dominator->current_state=RUN_TEST_IDLE;
	break;
      default:
	tms=true;
	dominator->current_state=SELECT_DR_SCAN;
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
	dominator->current_state=CAPTURE_IR;
	break;
      default:
	tms=true;
	dominator->current_state=TEST_LOGIC_RESET;
      };
      break;

    case CAPTURE_IR:
      switch(state){
      case SHIFT_IR:
	tms=false;
	dominator->current_state=SHIFT_IR;
	break;
      default:
	tms=true;
	dominator->current_state=EXIT1_IR;
      };
      break;

    case SHIFT_IR:
      switch(state){
      case SHIFT_IR:
	tms=false;
	break;
      default:
	tms=true;
	dominator->current_state=EXIT1_IR;
      };
      break;

    case EXIT1_IR:
      switch(state){
      case PAUSE_IR:
      case EXIT2_IR:
      case SHIFT_IR:
      case EXIT1_IR:
	tms=false;
	dominator->current_state=PAUSE_IR;
	break;
      default:
	tms=true;
	dominator->current_state=UPDATE_IR;
      };
      break;

    case PAUSE_IR:
      switch(state){
      case PAUSE_IR:
	tms=false;
	break;
      default:
	tms=true;
	dominator->current_state=EXIT2_IR;
      };
      break;

    case EXIT2_IR:
      switch(state){
      case SHIFT_IR:
      case EXIT1_IR:
      case PAUSE_IR:
	tms=false;
	dominator->current_state=SHIFT_IR;
	break;
      default:
	tms=true;
	dominator->current_state=UPDATE_IR;
      };
      break;

    case UPDATE_IR:
      switch(state){
      case RUN_TEST_IDLE:
	tms=false;
	dominator->current_state=RUN_TEST_IDLE;
	break;
      default:
	tms=true;
	dominator->current_state=SELECT_DR_SCAN;
      };
      break;

    default:
      tapTestLogicReset(dominator, ftdi_handle);
      tms=true;
    };
    set_tms(dominator, ftdi_handle, tms);
  }
  int i;
  for(i=0; i<pre; i++)
    set_tms(dominator, ftdi_handle, false);
}

unsigned int readusb(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, unsigned char * rbuf, unsigned long len)
{
    unsigned char buf[1] = { SEND_IMMEDIATE};
    mpsse_add_cmd(dominator, ftdi_handle, buf,1);
    mpsse_send(dominator, ftdi_handle);
    unsigned int read = 0;

        int length = (int) len;
        int timeout=0, last_errno, last_read;
        last_read = ftdi_read_data(ftdi_handle, rbuf, length );
        if (last_read > 0)
            read += last_read;
        while (((int)read <length) && ( timeout <1000)) 
        {
            last_errno = 0;
            last_read = ftdi_read_data(ftdi_handle, rbuf+read, length -read);
            if (last_read > 0)
                read += last_read;
            else
                last_errno = errno;
            timeout++;
        }
        if (timeout >= 1000)
        {
            applog(LOG_DEBUG,"readusb waiting too long for %ld bytes, only %d read",
                    len, last_read);
            if (last_errno)
            {
                applog(LOG_DEBUG,"error %s", strerror(last_errno));
            }
        }
        if (last_read <0)
        {
            applog(LOG_DEBUG,"Error %d str: %s", -last_read, strerror(-last_read));
        }
 return read;
}


void txrx_block(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, const unsigned char *tdi, unsigned char *tdo,
			int length, bool last)
{
  unsigned char rbuf[TX_BUF];
  unsigned const char *tmpsbuf = tdi;
  unsigned char *tmprbuf = tdo;
  /* If we need to shift state, treat the last bit separate*/
  unsigned int rem = (last)? length - 1: length; 
  unsigned char buf[TX_BUF];
  unsigned int buflen = TX_BUF - 3 ; /* we need the preamble*/
  unsigned int rembits;
  
  /*out on -ve edge, in on +ve edge */
  if (rem/8 > buflen)
    {
      while (rem/8 > buflen) 
	{
	  /* full chunks*/
	  buf[0] = ((tdo)?(MPSSE_DO_READ |MPSSE_READ_NEG):0)
	    |((tdi)?MPSSE_DO_WRITE:0)|MPSSE_LSB|MPSSE_WRITE_NEG;
	  buf[1] = (buflen-1) & 0xff;        /* low lenbth byte */
	  buf[2] = ((buflen-1) >> 8) & 0xff; /* high lenbth byte */
	  mpsse_add_cmd (dominator, ftdi_handle, buf, 3);
	  if(tdi) 
	    {
	      mpsse_add_cmd (dominator, ftdi_handle, tmpsbuf, buflen);
	      tmpsbuf+=buflen;
	    }
	  rem -= buflen * 8;
	  if (tdo) 
	    {
	      if  (readusb(dominator, ftdi_handle, tmprbuf,buflen) != buflen) 
		{
		  applog(LOG_DEBUG,"IO_JTAG_MPSSE::shiftTDITDO:"
			  "Failed to read block 0x%x bytes", buflen );
		}
	      tmprbuf+=buflen;
	    }
	}
    }
  rembits = rem % 8;
  rem  = rem - rembits;
  if (rem %8 != 0 ) 
    applog(LOG_DEBUG,"IO_JTAG_MPSSE::shiftTDITDO: Programmer error");
  buflen = rem/8;
   if(rem) 
    {
      buf[0] = ((tdo)?(MPSSE_DO_READ|MPSSE_READ_NEG):0)
	|((tdi)?MPSSE_DO_WRITE:0)|MPSSE_LSB|MPSSE_WRITE_NEG;
      buf[1] =  (buflen - 1)       & 0xff; /* low length byte */
      buf[2] = ((buflen - 1) >> 8) & 0xff; /* high length byte */
      mpsse_add_cmd (dominator, ftdi_handle, buf, 3);
      if(tdi) 
	    {
	      mpsse_add_cmd (dominator, ftdi_handle, tmpsbuf, buflen );
	      tmpsbuf  += buflen;
	    }
    }
  
  if (buflen >=(TX_BUF - 4))
    {
      /* No space for the last data. Send and evenually read 
         As we handle whole bytes, we can use the receiv buffer direct*/
      if(tdo)
	{
	  readusb(dominator, ftdi_handle, tmprbuf, buflen);
	  tmprbuf+=buflen;
	}
      buflen = 0;
    }
  if( rembits) 
    {
      /* Clock Data Bits Out on -ve Clock Edge LSB First (no Read)
	 (use if TCK/SK starts at 0) */
      buf[0] = ((tdo)?(MPSSE_DO_READ|MPSSE_READ_NEG):0)
	|((tdi)?MPSSE_DO_WRITE:0)|MPSSE_LSB|MPSSE_BITMODE|MPSSE_WRITE_NEG;
      buf[1] = rembits-1; /* length: only one byte left*/
      mpsse_add_cmd (dominator, ftdi_handle, buf, 2);
      if(tdi)
				mpsse_add_cmd (dominator, ftdi_handle, tmpsbuf,1) ;
      buflen ++;
    }
  if(last) 
    {
      bool lastbit = false;
      if(tdi) 
	lastbit = (*tmpsbuf & (1<< rembits));
      /* TMS/CS with LSB first on -ve TCK/SK edge, read on +ve edge 
	 - use if TCK/SK is set to 0*/
      buf[0] = MPSSE_WRITE_TMS|((tdo)?(MPSSE_DO_READ|MPSSE_READ_NEG):0)|
	MPSSE_LSB|MPSSE_BITMODE|MPSSE_WRITE_NEG;
      buf[1] = 0;     /* only one bit */
      buf[2] = (lastbit) ? 0x81 : 1 ;     /* TMS set */
      mpsse_add_cmd (dominator, ftdi_handle, buf, 3);
      buflen ++;
    }
  if(tdo) 
    {
      if (!last) 
	{
	  readusb(dominator, ftdi_handle, tmprbuf, buflen);
	  if (rembits) /* last bits for incomplete byte must get shifted down*/
	    tmprbuf[buflen-1] = tmprbuf[buflen-1]>>(8-rembits);
	}
      else 
	{
	  /* we need to handle the last bit. It's much faster to
		 read into an extra buffer than to issue two USB reads */
	  readusb(dominator, ftdi_handle, rbuf, buflen); 
	  if(!rembits) 
	    rbuf[buflen-1] = (rbuf[buflen - 1]& 0x80)?1:0;
	  else 
	    {
	      /* TDO Bits are shifted downwards, so align them 
		 We only shift TMS once, so the relevant bit is bit 7 (0x80) */
	      rbuf[buflen-2] = rbuf[buflen-2]>>(8-rembits) |
		((rbuf[buflen - 1]&0x80) >> (7 - rembits));
	      buflen--;
	    }
	  memcpy(tmprbuf,rbuf,buflen);
	}
    }
}


void shiftTDITDO(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, const unsigned char *tdi, unsigned char *tdo,
			 int length, bool last)
{
  if(length==0) return;
  flush_tms(dominator, ftdi_handle, false);
  txrx_block(dominator, ftdi_handle, tdi, tdo, length,last);
  return;
}

void shiftTDI(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, const unsigned char *tdi, int length, bool last)
{
  shiftTDITDO(dominator, ftdi_handle, tdi, NULL, length,last);
}

// TDI gets a load of zeros, we just record TDO.
void shiftTDO(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, unsigned char *tdo, int length, bool last)
{
    shiftTDITDO(dominator, ftdi_handle, NULL, tdo, length,last);
}

  
  
/* Detect chain length on first start, return chain length else*/
int getChain(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, bool detect)
{
  int i;
  if(dominator->numDevices  == -1 || detect)
    {
      tapTestLogicReset(dominator, ftdi_handle);
      setTapState(dominator, ftdi_handle, SHIFT_DR, 0);
      byte idx[4];
      byte zero[4];
      dominator->numDevices=0;
      for(i=0; i<4; i++)zero[i]=0;
      do{
					shiftTDITDO(dominator, ftdi_handle, zero,idx,32,false);
					unsigned long id=byteArrayToLong(idx);
					if(id!=0 && id !=0xffffffff){
					  dominator->numDevices++;
//					  chainParam_t dev;
//					  dev.idcode=id;
//					  devices.insert(devices.begin(),dev);
					}
					else{
					  if (id == 0xffffffff && dominator->numDevices >0)
					    {
					      applog(LOG_ERR, "Probably a broken Atmel device in your chain!");
					      applog(LOG_ERR, "No succeeding device can be identified");
					    }
					  break;
					}
      }while(dominator->numDevices<MAXnumDevices);
      setTapState(dominator, ftdi_handle, TEST_LOGIC_RESET, 0);
    }
  applog(LOG_ERR, "getChain found %d devices",dominator->numDevices);
  return dominator->numDevices;
}

int dom_ftdi_init(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, const char *serial, unsigned int freq) {
  unsigned char   buf1[5];
  unsigned char   buf[9] = { SET_BITS_LOW, 0x00, 0x0b,
                             TCK_DIVISOR,  0x03, 0x00 ,
                             SET_BITS_HIGH,0x00, 0x00};
  char *description = NULL;
  char descstring[256];
  unsigned int vendor = dominator_vendor, product = dominator_product;
  unsigned int channel = 0;
  unsigned int dbus_data =0, dbus_en = 0xb, cbus_data= 0, cbus_en = 0;
  unsigned int divisor;
  int res;
  bool device_has_fast_clock; 
  memset( ones,0xff,CHUNK_SIZE);
  memset(zeros,   0,CHUNK_SIZE);
  memset(dominator->tms_buf,   0,CHUNK_SIZE);
  dominator->tms_len = 0;

  
      /* set for now. If we have a fast device, correct later */
  if ((freq == 0 )|| (freq >= 6000000)) /* freq = 0 means max rate, 3 MHz for now*/
      divisor = 0;
  else
      divisor = 6000000/freq - ((6000000&freq)?0:1);
  if (divisor > 0xffff)
      divisor = 0xffff;

  buf[4] = divisor & 0xff;
  buf[5] = (divisor >> 8) & 0xff;	

      // allocate and initialize FTDI structure
//      ftdi_handle = ftdi_new();
      // Set interface
      if (channel > 2)
      {
          applog(LOG_ERR, "Invalid MPSSE channel: %d", channel);
          res = 2;
          goto ftdi_fail;
      }
      res =ftdi_set_interface(ftdi_handle, 0x01); // Interface A
      if(res <0)
      {
          applog(LOG_ERR, "ftdi_set_interface: %s",
                  ftdi_get_error_string(ftdi_handle));
          goto ftdi_fail;
      }
      
      // Open device
      res = ftdi_usb_open_desc(ftdi_handle, vendor, product, 
                               description, serial);
      if (res == 0)
      {
          res = ftdi_set_bitmode(ftdi_handle, 0x00, BITMODE_RESET);
          if(res < 0)
          {
              applog(LOG_ERR, "ftdi_set_bitmode: %s",
                      ftdi_get_error_string(ftdi_handle));
              goto ftdi_fail;
          }
          res = ftdi_usb_purge_buffers(ftdi_handle);
          if(res < 0)
          {
              applog(LOG_ERR, "ftdi_usb_purge_buffers: %s",
                      ftdi_get_error_string(ftdi_handle));
              goto ftdi_fail;
         }
          //Set the lacentcy time to a low value
          res = ftdi_set_latency_timer(ftdi_handle, 1);
          if( res <0)
          {
              applog(LOG_ERR, "ftdi_set_latency_timer: %s",
                      ftdi_get_error_string(ftdi_handle));
              goto ftdi_fail;
          }
          
          // Set mode to MPSSE
          res = ftdi_set_bitmode(ftdi_handle, 0xfb, BITMODE_MPSSE);
          if(res< 0)
          {
              applog(LOG_ERR, "ftdi_set_bitmode: %s",
                      ftdi_get_error_string(ftdi_handle));
              goto ftdi_fail;
          }
          /* FIXME: Without this read, consecutive runs on the 
             FT2232H may hang */
          ftdi_read_data(ftdi_handle, buf1,5);

          /* Check if we have a fast clock cabable device*/
          switch(ftdi_handle->type)
          {
          case TYPE_2232H:
          case TYPE_4232H:
              device_has_fast_clock = true;
              break;
          default:
              device_has_fast_clock = false;
          }

      }
      else /* Unconditionally try ftd2xx on error*/
      {
          applog(LOG_ERR, "Could not open FTDI device (using libftdi): %s",
                  ftdi_get_error_string(ftdi_handle));
          ftdi_free(ftdi_handle);
          ftdi_handle = 0;
      }

  if (!ftdi_handle)
  {
      applog(LOG_ERR, "Unable to access FTDI device with either libftdi");
      res = 1;
      goto fail;
  };
	applog(LOG_DEBUG, "Opened FTDI");
  // Prepare for JTAG operation
  buf[1] |= dbus_data;
  buf[2] |= dbus_en;
  buf[7] = cbus_data;
  buf[8] = cbus_en;
  
  mpsse_add_cmd(dominator, ftdi_handle, buf, 9);
  mpsse_send(dominator, ftdi_handle);
  /* On H devices, use the non-divided clock*/
  if (device_has_fast_clock && ((freq == 0) ||(freq > 458)))
  {
      if ((freq == 0) ||(freq >= 30000000)) /* freq = 0 means max rate, 30 MHz for now*/
          divisor = 0;
      else
          divisor = 30000000/freq -((30000000%freq)?0:1);
      if (divisor > 0xffff)
          divisor = 0xffff;
#ifndef DIS_DIV_5
#define DIS_DIV_5 0x8a
#endif
      buf[0] = DIS_DIV_5;
      buf[1] = TCK_DIVISOR;
      buf[2] =  divisor & 0xff;
      buf[3] = (divisor >> 8) & 0xff;
      mpsse_add_cmd(dominator, ftdi_handle, buf, 4);
      mpsse_send(dominator, ftdi_handle);
  }
  return 0;

ftdi_fail:
fail:
  return res;
     
}
int SelectDevice(struct cgpu_info *dominator, int dev) {
  if(dev>=dominator->numDevices)dominator->deviceIndex=-1;
  	else dominator->deviceIndex=dev;
	applog(LOG_DEBUG,"selectDevices %d", dominator->deviceIndex);
  return dominator->deviceIndex;	
}

void nextTapState(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, bool tms)
{
  if(dominator->current_state==SHIFT_DR){
    if(tms)dominator->current_state=EXIT1_DR; // If TMS was set then goto next state
  }
  else if(dominator->current_state==SHIFT_IR){
    if(tms)dominator->current_state=EXIT1_IR; // If TMS was set then goto next state
  }
  else 
    {
      applog(LOG_ERR,"Unexpected state %d",dominator->current_state);
      tapTestLogicReset(dominator, ftdi_handle); // We were in an unexpected state
    }
}

// TDI gets a load of zeros or ones, and we ignore TDO
void shift(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, bool tdi, int length, bool last)
{
    int len = length;
    unsigned char *block = (tdi)?ones:zeros;
    flush_tms(dominator, ftdi_handle, false);
    while (len > CHUNK_SIZE*8)
    {
	txrx_block(dominator, ftdi_handle, block, NULL, CHUNK_SIZE*8, false);
	len -= (CHUNK_SIZE*8);
    }
    shiftTDITDO(dominator, ftdi_handle, block, NULL, len, last);
}

void shiftDR(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, const byte *tdi, byte *tdo, int length,
		   int align, bool exit)
{
  if(dominator->deviceIndex<0)return;
  int post=dominator->deviceIndex;

  if(!dominator->shiftDRincomplete){
    int pre=dominator->numDevices-dominator->deviceIndex-1;
    if(align){
      pre=-post;
      while(pre<=0)pre+=align;
    }
    /* We can combine the pre bits to reach the target device with
     the TMS bits to reach the SHIFT-DR state, as the pre bit can be '0'*/
    setTapState(dominator, ftdi_handle, SHIFT_DR,pre);
  }

  if(tdi!=0&&tdo!=0) shiftTDITDO(dominator, ftdi_handle, tdi,tdo,length,post==0&&exit);
  else if(tdi!=0&&tdo==0) shiftTDI(dominator, ftdi_handle, tdi,length,post==0&&exit);
  else if(tdi==0&&tdo!=0) shiftTDO(dominator, ftdi_handle, tdo,length,post==0&&exit);
  else  shift(dominator, ftdi_handle, false,length,post==0&&exit);

  nextTapState(dominator, ftdi_handle, post==0&&exit); // If TMS is set the the state of the tap changes
  if(exit){
     shift(dominator, ftdi_handle, false,post, true);
    if (!(post==0&&exit))
      nextTapState(dominator, ftdi_handle, true);
    setTapState(dominator, ftdi_handle, dominator->postDRState, 0);
    dominator->shiftDRincomplete=false;
  }
  else dominator->shiftDRincomplete=true;
}


void shiftIR(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, const byte *tdi, byte *tdo)
{
  if(dominator->deviceIndex<0)return;
  setTapState(dominator, ftdi_handle, SHIFT_IR, 0);

  int pre=0;
  int dev=0;
  for(dev=dominator->deviceIndex+1; dev<dominator->numDevices; dev++)
    pre+= 6; // Calculate number of pre BYPASS bits.
  int post=0;
  for(dev=0; dev<dominator->deviceIndex; dev++)
    post+= 6; // Calculate number of post BYPASS bits.
  shift(dominator, ftdi_handle, true,pre,false);
  if(tdo!=0) shiftTDITDO(dominator, ftdi_handle, tdi,tdo, 6,post==0);
    else if(tdo==0) shiftTDI(dominator, ftdi_handle, tdi, 6,post==0);
  shift(dominator, ftdi_handle, true,post, true);
  nextTapState(dominator, ftdi_handle, true);
  setTapState(dominator, ftdi_handle, dominator->postIRState, 0);
}

void poke(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, int addr, long value) {
        byte mydata[8];
        byte checksum;
//        applog(LOG_DEBUG, "Poke %d : %08x)", addr, value);
        mydata[0] = 0x02;
        shiftIR(dominator, ftdi_handle, mydata, 0);
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
        shiftDR(dominator, ftdi_handle, mydata, NULL, 38, 0, true);
}

long peek(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle, int addr) {
        byte mydata[8];
//	applog(LOG_DEBUG, "Peek %d", addr);
        mydata[0] = 0x02;
        shiftIR(dominator, ftdi_handle, mydata, 0);
        mydata[0] = 0X00;
        mydata[1] = 0x00;
        mydata[2] = 0x00;
        mydata[3] = 0x00;
        mydata[4] = (addr & 0x0f) | ((1 ^ ((addr >> 3) & 1) ^ ((addr >> 2) & 1) ^ ((addr >> 1) & 1) ^ (addr & 1) ) << 5 );
      
        shiftDR(dominator, ftdi_handle, mydata, 0, 38, 0, true);
        shiftDR(dominator, ftdi_handle, 0, mydata, 32, 0, true);
        return byteArrayToLong(mydata);
}

long getUsercode(struct cgpu_info *dominator, struct ftdi_context *ftdi_handle) {
        byte mydata[8];
        mydata[0] = 0x08;
        shiftIR(dominator, ftdi_handle, mydata, 0);
        shiftDR(dominator, ftdi_handle, 0, mydata, 32, 0, true);
        return byteArrayToLong(mydata);
}

int setup_serial_port(const char*devpath)
{
	struct termios newtio;
	int fd;
	fd = open(devpath, O_RDWR | O_CLOEXEC | O_NOCTTY);

	if (fd < 0) {
		printf("Error opening serial port");
		return(-1);
	}

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/* man termios get more info on below settings */
	newtio.c_cflag = B3000000 | CS8 | CLOCAL | CREAD;

	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	// block for up till 0 characters
	newtio.c_cc[VMIN] = 0;

	// 0.5 seconds read timeout
	newtio.c_cc[VTIME] = 5;

	/* now clean the modem line and activate the settings for the port */
	tcflush(fd, TCIFLUSH);
	tcflush(fd, TCOFLUSH);
	tcsetattr(fd,TCSANOW,&newtio);
	
	return fd;
}

ssize_t myserial_read2(struct cgpu_info *dominator, int fd, char *buf, size_t bufsiz)
{
	
	fd_set set;
  	struct timeval timeout;
  	int rv, breakit, bufpos, i;
	unsigned char timeoutbuffer[256];

	FD_ZERO(&set); 
  	FD_SET(fd, &set); 

  	timeout.tv_sec = 0;
  	timeout.tv_usec = 300000;
	rv = 0;
	breakit = 0;
	bufpos = 0;

  	while ((bufpos < bufsiz) && (breakit==0) ) {
		rv = select(fd + 1, &set, NULL, NULL, &timeout);
  		if (rv == -1)
			breakit = 1;
  		else if(rv == 0) {
			breakit = 1;
			if (dominator == NULL) applog(LOG_ERR, "Timeout !!!");
				else applog(LOG_ERR, "%s %u: Timeout (%s)", dominator->drv->name, dominator->device_id, dominator->device_path);
		}
		else {
			rv = read (fd, buf+bufpos, rv);
			bufpos += rv;
		}
	}
	if (rv > 0 )
 		return bufpos;
	else return rv;
	
}

ssize_t myserial_read(int fd, char *buf, size_t bufsiz)
{
	return myserial_read2(NULL, fd, buf, bufsiz);
}

static bool dominator_fpga_upload_bitstream(int fd, const char *devpath)
{
	fd_set fds;
	unsigned char buf[0x100];
	unsigned char *ubuf = (unsigned char *)buf;
	unsigned long len, bufferPointer;
	unsigned char *p;
	const char *fwfile = BITSTREAM_FILENAME;
	int f;
	unsigned char *buffer;
	unsigned long fileLen, blocksize;

				buf[0] = 0x04; // Clear FPGAs
		    f = write(fd, buf, 1);
		  	f = myserial_read(fd, buf, 3);

	applog(LOG_DEBUG,"Fpga States after Clear: %0.2x %0.2x %0.2x",buf[0],buf[1],buf[2]);
	applog(LOG_ERR,"Loading %s", devpath);

	FILE *fp = open_bitstream("dominator", fwfile);
	if (!fp)
		applog(LOG_ERR, "Error opening dominator firmware file %s", fwfile);
	fseek(fp, 0, SEEK_END);
	fileLen=ftell(fp);
	fseek(fp, 0, SEEK_SET);
	buffer=(char *)malloc(fileLen+1);
	if (!buffer)
		applog(LOG_ERR, "Memoryerror while loading Firmware %s", fwfile);
	
	f = fread(buffer, fileLen, 1, fp);
	
	fclose(fp);
		
	bufferPointer = 0;
	bufferPointer += (buffer[bufferPointer] * 256) + buffer[bufferPointer+1] + 2;
	bufferPointer += (buffer[bufferPointer] * 256) + buffer[bufferPointer+1] + 2;
	bufferPointer += (buffer[bufferPointer] * 256) + buffer[bufferPointer+1] + 2;
	bufferPointer += (buffer[bufferPointer+1] * 256) + buffer[bufferPointer+2] + 2 + 1;
	bufferPointer += (buffer[bufferPointer+1] * 256) + buffer[bufferPointer+2] + 2 + 1;
	bufferPointer += (buffer[bufferPointer+1] * 256) + buffer[bufferPointer+2] + 2 + 1;
	if (buffer[bufferPointer] != 0x65)
		applog(LOG_ERR, "Error couldnot find Binary start in %s", fwfile);
	
	buf[0] =  0x44; // CMD Load FPGA
	
	bufferPointer += 5;
	if (write(fd, buf, 1) != 1)
		applog(LOG_ERR, "Cmd send failed: 0x%d", buf[0]);
	if ((myserial_read(fd, buf, 1) != 1))
		applog(LOG_ERR, "FPGA Load Init failed (No Answer)");
	if (buf[0] != 0x80)
		applog(LOG_ERR, "FPGA Load Init failed (Wrong Answer)");
	
	while (bufferPointer != fileLen) {
		if (bufferPointer+240 > fileLen) {
			blocksize = fileLen-bufferPointer;
		} else {
			blocksize = 240;			
		}
		buf[0] = 0x45;
		buf[1] = blocksize;
		if (write(fd, buf, 2) != 2)
			applog(LOG_ERR, "FPGA Load Data failed (CMD)");
		if (write(fd, buffer+bufferPointer, blocksize) != blocksize)
			applog(LOG_ERR, "FPGA Load Data failed (Data)");
		if ((myserial_read(fd, buf, 1) != 1 ))
			applog(LOG_ERR, "FPGA Load Data failed (No Answer)");
		if (buf[0] != 0x80)
			applog(LOG_ERR, "FPGA Load Data failed (Wrong Answer)");
	  bufferPointer += blocksize;
	}
	free(buffer);
	buf[0] = 0x46; // CMD FPGA finalize
	if (write(fd, buf, 1) != 1)
		applog(LOG_ERR, "Cmd send failed: 0x%d", buf[0]);
	if (myserial_read(fd, buf, 3) != 3)
		applog(LOG_ERR, "FPGA Load Finalize failed");
	if ((buf[0] != 0xFF) || (buf[1] != 0xFF) || (buf[2] != 0x80)) {
		applog(LOG_ERR,"Fpga States after Loading: %0.2x %0.2x %0.2x",buf[0],buf[1],buf[2]);
		
		return false;
	} 

//	applog(LOG_WARNING, "%s %u: Done programming %s", dominator->drv->name, dominator->device_id, dominator->device_path);
	return true;
}

static bool dominatorJTAG_detect_one(char *serial) {
	int i;
	applog(LOG_DEBUG, "dominatorJTAG_detect_one");

	struct cgpu_info *dominator;
	dominator = calloc(1, sizeof(*dominator));
	dominator->drv = &dominator_drv;
	mutex_init(&dominator->device_mutex);
	dominator->device_path = serial;
	dominator->device_fd = -1;
	dominator->deven = DEV_ENABLED;
	dominator->name = serial;
	dominator->ftdi_handle = ftdi_new(); 
	dominator->threads = 0;
  dominator->current_state = UNKNOWN_STATE;        
  dominator->postDRState = RUN_TEST_IDLE;    
  dominator->postIRState = RUN_TEST_IDLE;    
  dominator->deviceIndex = -1;                           
  dominator->shiftDRincomplete = false;                 
  dominator->numDevices  = -1;                           


	dom_ftdi_init (dominator, dominator->ftdi_handle, serial, 1000000);
	if (getChain(dominator, dominator->ftdi_handle, true) >=0) {
		dominator->threads = 1; 
		dominator->chainlength = getChain(dominator, dominator->ftdi_handle, false);
                dominator->device_path = calloc(256, 1);
                dominator->device_path[0] = 0;
                strcat(dominator->device_path ,"/dev/serial/by-id/usb-FTDI_Dominator_");
               
                strcat (dominator->device_path , serial);
                strcat (dominator->device_path, "-if01-port0");
//                applog(LOG_ERR, "%s", dominator->device_path);
		return add_cgpu(dominator);		
	} else {
		return 0;
	}
}
 
static void dominator_detect()
{
  struct ftdi_context myFTDI;
  struct ftdi_device_list *myDevs, *curDev;
  char manufactor[128], description[128];
  char serialnumber[128];
  int i, y;
  int countdevices = 0;
  ftdi_init(&myFTDI);
  i = ftdi_usb_find_all(&myFTDI, &myDevs, 0x0403, 0x6010);

  i = 0;
  for (curDev = myDevs; curDev != NULL; i++) {
      y = ftdi_usb_get_strings(&myFTDI, curDev->dev, manufactor, 128, description, 128, serialnumber, 128);
     	if (dominatorJTAG_detect_one(serialnumber)) countdevices++;
//     	if (dominatorJTAG_detect_one("00029")) countdevices++;

      curDev = curDev->next;
//      if (countdevices==1) break;
  }
  ftdi_list_free(&myDevs);
  applog(LOG_ERR, "Anzahl: %d", countdevices);
	
}

static bool dominator_thread_prepare(struct thr_info *thr)
{
	struct cgpu_info *dominator = thr->cgpu;
        byte mydata [8];
        int i, y, len;
	struct timeval now;
        unsigned char buf[0x100];
        
/*
        int fd = setup_serial_port(dominator->device_path);
        if (unlikely(-1 == fd)) {
            applog(LOG_ERR, "Could not open the Serial Device (%s) for loading FPGA", dominator->device_path);
            return false;
        }
        dominator->device_fd = fd;
        tcflush(fd, TCIOFLUSH);
        memset(buf, 0x100, 0xFF);
        if (select_write(fd, buf, 255) < 1) applog(LOG_ERR, "NOP sending failed (%s)", dominator->device_path);
        len = select_read(fd, buf, sizeof(buf)-1);
        if (buf[len-1] != 0x80) applog(LOG_ERR, "Wrong NOP Reply (%s)", dominator->device_path);
        tcflush(fd, TCIOFLUSH);
        usleep(500);
        i = 0;
        while (i<1) {
            buf[0] = 0x03; // Get FPGA Loaded State
            if (write(fd, buf, 1) != 1) applog(LOG_ERR, "Error writing loading State (%s)", dominator->device_path);
            if (myserial_read(fd, buf,3) != 3) applog(LOG_ERR, "Error reading loading State (%s)", dominator->device_path);
            if (buf[2] != 0x80) applog(LOG_ERR, "Loading FPGA State has wrong answer (%s)", dominator->device_path);
            if ((buf[0] != 0xFF) || (buf[1] != 0xFF)) {
                if (dominator_fpga_upload_bitstream(fd, dominator->device_path)) {
                    i = 100;
                } else {
                    i++;
                }
            } else {
                i=100;
            }
        }
        tcflush(fd, TCIOFLUSH);
*/
//  applog(LOG_DEBUG, "ChainLength: %d", dominator->chainlength);
//	free(dominator->works);
	dominator->works = calloc(2 * dominator->chainlength * sizeof(struct work *), 4); // AVALON_ARRAY_SIZE=4
        dominator->work_array = 0;
	cgtime(&now);
	get_datestamp(dominator->init, &now);
        for (i=0; i<dominator->chainlength; i++) {
            SelectDevice(dominator, i);
            for (y=0; y<16; y++)
            {
            	applog(LOG_ERR, "ID: %d reg: %02x=%08x", i, y,peek(dominator, dominator->ftdi_handle, y));
            }
            dominator->nonces[i] = peek(dominator, dominator->ftdi_handle, 0x0c);
//            poke(dominator, dominator->ftdi_handle, 0x0d, 175);
            dominator->clocks[i] = peek(dominator, dominator->ftdi_handle, 0x0d);
            dominator->workingon[i] = false;
            dominator->has_restart[i] = true;
            dominator->is_disabled[i] = true;
            dominator->mydiff[i] = 0;
            dominator->numnonces[i] = 0;
            dominator->errorcount[i] = 0;
            for (y=0; y<250; y++) dominator->errorrate[i][y] = 0;
            applog (LOG_ERR, "ID: %d has Code %08x Clockrate: %08x", i, getUsercode(dominator, dominator->ftdi_handle), peek(dominator, dominator->ftdi_handle, 0x0d));
            
            
        }

  return true;
}

static int64_t dominator_scanhash(struct thr_info *thr)
{
	struct cgpu_info *dominator = thr->cgpu;
	struct work **works;
        int i, tries;
	uint32_t nonce , lastnonce;
	int64_t hash_count = 0;
        static int first_try = 0;
        int y= 0;
 	works = dominator->works;
        int start_count = 0;
	for (i=0; i<dominator->chainlength; i++) {

		usleep(20000);
            SelectDevice(dominator, i);
            if (thr->work_restart) {
                for (tries=0; tries<(dominator->chainlength*2); tries++) {
                    works[tries]->devflag = true;
                    dominator->has_restart[tries] = true;
                }
                break;
            } else {
                if (dominator->is_disabled[i]) {

                    if (dominator->workingon[i] == false) {
                        start_count = 0;
                    } else {
                        start_count = dominator->chainlength;
                    }

                    nonce = 0x00;
                    tries = 0;
                    lastnonce = 0xFFFFFFFF;
                    while (nonce != 0xFFFFFFFF) {
                        nonce = peek(dominator, dominator->ftdi_handle, 0x0e);
// 			applog(LOG_ERR, "nonce %d-%d %08x",i,i,nonce);
                        if ((nonce != 0x00) && ((nonce & 0xFFFFFFF0) != 0xFFFFFFF0) && (first_try != 0)){
                            if (!dominator->has_restart[i]) {
                                y = check_diff_nonce(thr, works[start_count+i], nonce);
                                if (y == 0xFFFFFFFF) {
                                    dominator->errorcount[i]++;
                                } else {
                                    dominator->mydiff[y]++;
                                    dominator->numnonces[i]++;
                                }
                                works[start_count+i]->mydiff = y;
                                if (!submit_nonce(thr, works[start_count+i], nonce)) {
                                    if (lastnonce == nonce) {
                                        dominator->is_disabled[i] = false;
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

                    nonce = peek(dominator, dominator->ftdi_handle, 0x0c);
		    if (i==7) applog(LOG_ERR,"nonce %d: %08x",i, nonce);
                    if (nonce > dominator->nonces[i]) hash_count += (nonce - dominator->nonces[i]);
                    if (nonce < dominator->nonces[i]) hash_count += (nonce + (0xFFFFFFFF - dominator->nonces[i]));
                    dominator->nonces[i] = nonce;
//		    applog(LOG_ERR, "State %d:%08x", i, peek(dominator, dominator->ftdi_handle, 0x0F));
                    if ((peek(dominator, dominator->ftdi_handle, 0x0F) & 0x01) == 0) {
/*
                        // Adjust Clock
                        if ((dominator->numnonces[i] + dominator->errorcount[i]) >30) {
                            dominator->errorrate[i][dominator->clocks[i]] = (dominator->errorcount[i] / dominator->numnonces[i]) * 100;
                            if (dominator->errorrate[i][dominator->clocks[i]] > 4.0) {
				if (dominator->clocks[i] >170) {
                                dominator->clocks[i] -= CLK_ADJUST_STEP;
                                poke(dominator, dominator->ftdi_handle, 0x0d, dominator->clocks[i]);
                                applog(LOG_ERR, "Reducing Clock(%d) to %d Mhz Errorrate was %f", i, dominator->clocks[i], dominator->errorrate[i][dominator->clocks[i]+CLK_ADJUST_STEP ]);
				}
                            } else {
                                if (dominator->errorrate[i][dominator->clocks[i] + CLK_ADJUST_STEP] < 6.0) {
                                    dominator->clocks[i] += CLK_ADJUST_STEP;
                                    poke(dominator, dominator->ftdi_handle, 0x0d, dominator->clocks[i]);
                                    applog(LOG_ERR, "Increasing Clock(%d) to %d Mhz Errorrate was %f", i, dominator->clocks[i], dominator->errorrate[i][dominator->clocks[i]-CLK_ADJUST_STEP ]);
                                    
                                }
                                
                                
                            }
                            dominator->numnonces[i] = 0;
                            dominator->errorcount[i] = 0;
                        }
*/
                        dominator->has_restart[i] = false;
                        applog(LOG_DEBUG, "Loading new Work in %d", i);
                        // Needs new work !
                        works[start_count+i]->devflag = true;
                        works[start_count+i]->blk.nonce = 0xffffffff;
                        dominator->workingon[i] = !dominator->workingon[i];
                        if (dominator->workingon[i] == false) {
                            start_count = 0;
                        } else {
                            start_count = dominator->chainlength;
                        }
                        works[start_count+i]->devflag = false;
			nonce = peek(dominator, dominator->ftdi_handle, 0x0c);
			applog(LOG_ERR, "Startnonce: %d: %08x", i, nonce);
                        poke(dominator, dominator->ftdi_handle, 0x01, byteArrayToLong(&works[start_count+i]->midstate[0]));
                        poke(dominator, dominator->ftdi_handle, 0x02, byteArrayToLong(&works[start_count+i]->midstate[4]));
                        poke(dominator, dominator->ftdi_handle, 0x03, byteArrayToLong(&works[start_count+i]->midstate[8]));
                        poke(dominator, dominator->ftdi_handle, 0x04, byteArrayToLong(&works[start_count+i]->midstate[12]));
                        poke(dominator, dominator->ftdi_handle, 0x05, byteArrayToLong(&works[start_count+i]->midstate[16]));
                        poke(dominator, dominator->ftdi_handle, 0x06, byteArrayToLong(&works[start_count+i]->midstate[20]));
                        poke(dominator, dominator->ftdi_handle, 0x07, byteArrayToLong(&works[start_count+i]->midstate[24]));
                        poke(dominator, dominator->ftdi_handle, 0x08, byteArrayToLong(&works[start_count+i]->midstate[28]));

                        poke(dominator, dominator->ftdi_handle, 0x09, byteArrayToLong(&works[start_count+i]->data[64]));
                        poke(dominator, dominator->ftdi_handle, 0x0A, byteArrayToLong(&works[start_count+i]->data[68]));
                        poke(dominator, dominator->ftdi_handle, 0x0B, byteArrayToLong(&works[start_count+i]->data[72]));
                        works[start_count+i]->subid = i;
                    }
                }
            }
            	}
        first_try = 1;

  return hash_count;
}

static void dominator_fpga_shutdown(struct thr_info *thr)
{
	struct cgpu_info *dominator = thr->cgpu;

//	free(thr->cgpu_data);
}

static void dominator_reinit(struct cgpu_info *dominator)
{

}

/* We use a replacement algorithm to only remove references to work done from
 * the buffer when we need the extra space for new work. */
static bool dominator_fill(struct cgpu_info *dominator)
{
        int i;
        struct work *work;
        work = get_queued(dominator);
        if (dominator->queued < (dominator->chainlength * 2)) {
            dominator->works[dominator->queued] = work;
            dominator->queued++;
            return false;
        }
            
        for (i=0; i< (dominator->chainlength * 2); i++) {
            if (dominator->works[i]->devflag) {
                work_completed(dominator, dominator->works[i]);
                dominator->works[i] = work;
                return false;
            }
        }
        return true;
}

static struct api_data *dominator_api_stats(struct cgpu_info *dominator)
{
	struct api_data *root = NULL;

	root = api_add_int(root, "clock0", &(dominator->clocks[0]), false);
	root = api_add_int(root, "clock1", &(dominator->clocks[1]), false);
	root = api_add_int(root, "clock2", &(dominator->clocks[2]), false);
	root = api_add_int(root, "clock3", &(dominator->clocks[3]), false);
	root = api_add_int(root, "clock4", &(dominator->clocks[4]), false);
	root = api_add_int(root, "clock5", &(dominator->clocks[5]), false);
	root = api_add_int(root, "clock6", &(dominator->clocks[6]), false);
	root = api_add_int(root, "clock7", &(dominator->clocks[7]), false);
	root = api_add_int(root, "clock8", &(dominator->clocks[8]), false);
	root = api_add_int(root, "clock9", &(dominator->clocks[9]), false);
	root = api_add_int(root, "clock10", &(dominator->clocks[10]), false);
	root = api_add_int(root, "clock11", &(dominator->clocks[11]), false);
	root = api_add_int(root, "clock12", &(dominator->clocks[12]), false);
	root = api_add_int(root, "clock13", &(dominator->clocks[13]), false);
	root = api_add_int(root, "clock14", &(dominator->clocks[14]), false);
	root = api_add_int(root, "clock15", &(dominator->clocks[15]), false);

	return root;
}

struct device_drv dominator_drv = {
	.dname = "Dominator",
	.name = "DOM",
	.drv_detect = dominator_detect,
	.thread_prepare = dominator_thread_prepare,
	.hash_work = hash_queued_work,
	.queue_full = dominator_fill,
	.scanwork = dominator_scanhash,
	.reinit_device = dominator_reinit,
        .get_api_stats = dominator_api_stats,
	.thread_shutdown = dominator_fpga_shutdown,

};
