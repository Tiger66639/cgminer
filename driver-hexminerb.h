/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
/*
 * Copyright 2013 Avalon project Copyright 2013 Con Kolivas <kernel@kolivas.org>
 * This program is free software;
 * you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version. See
 * COPYING for more details. Thank you guys!
 */
#ifndef HEXB_H
#define HEXB_H
#ifdef USE_HEXMINERB
//#define DBG_HEXB 
#include "util.h"
/* hexminerb_task/work_reply status Definitions: */
#define HEXB_STAT_IDLE					0       /* Idle or data already Sent to the buffer */
#define HEXB_STAT_NEW_WORK				6       /* Request for write in the buffer */
#define HEXB_STAT_WAITING				2       /* Wait For Buffer Empty Position */
#define HEXB_STAT_CLR_BUFF				3       /* Force Buffer Empty */
#define HEXB_STAT_STOP_REQ				4       /* Stop Request */
#define HEXB_STAT_NEW_WORK_CLEAR_OLD		5       /* Clear Buffers and after that fill the first buffer */
#define HEXB_STAT_UNUSED					7

/* libhexb_eatHashData/BUF_reply status Definitions: */
#define HEXB_BUF_DATA 0
#define HEXB_BUF_ERR  1
#define HEXB_BUF_SKIP 2


#define HEXMINERB_ARRAY_PIC_SIZE		64
#define HEXMINERB_ARRAY_SIZE                  HEXMINERB_ARRAY_PIC_SIZE * 4
#define HEXMINERB_ARRAY_SIZE_REAL	HEXMINERB_ARRAY_SIZE - 2
//#define HEXMINERB_ARRAY_SIZE 253
#define HEXB_NONCE_CASH_SIZE				16
#define HEXMINERB_PUSH_THRESH		8       /* At least 2 queued works available to be written to PIC */
#define HEXMINERB_ARRAY_MAX_POP		1
#define HEXB_USB_R_SIZE					64
#define HEXB_USB_WR_SIZE					64
#define HEXB_HASH_BUF_SIZE				2048
#define HEXB_USB_R_BAD_ID					32
#define HEXB_USB_WR_TIME_OUT				500
#define HEXMINERB_BULK_READ_TIMEOUT 1000
#define HEXB_MINER_THREADS			1
#define HEXB_DEFAULT_MINER_NUM		0x01
#define HEXB_DEFAULT_ASIC_NUM		0x10
#define HEXB_MIN_FREQUENCY			0       //Bits / 10
#define HEXB_MAX_FREQUENCY			610     //Bits / 10
#define HEXB_DEFAULT_FREQUENCY		540     //Bits / 10 - That is Max which works 40 GHs for 16 chips
#define HEXB_DEFAULT_CORE_VOLTAGE	840     /* in millivolts */
#define HEXB_MIN_COREMV				700     /* in millivolts */
#define HEXB_MAX_COREMV	1101    /* in millivolts */
struct chip_resultsb
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEXB_NONCE_CASH_SIZE];
};
struct workb_result
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t lastnonce;           //1x32
  uint8_t lastnonceid;          //1x32
  uint8_t status;
  uint16_t lastvoltage;         //1x32
  uint8_t lastchippos;          //1x32
  uint8_t prevnonceid;          //16 bit words aligned with lastchippos
  uint8_t csum;

} __attribute__ ((packed, aligned (4)));

struct hexminerb_info
{
  int miner_count;
  int asic_count;
  int core_voltage;
  int frequency;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  int pic_voltage_readings;
  int hash_read_pos;
  int hash_write_pos;
  int dupe[HEXB_DEFAULT_ASIC_NUM];
  int matching_work[HEXB_DEFAULT_ASIC_NUM];
  unsigned char *readbuf;
  struct workb_result *wr;
  struct chip_resultsb *array_nonce_cache;
  uint8_t wr_status;
  pthread_t write_thr;
  pthread_mutex_t lock;
  cgsem_t wsem;
  struct thr_info *thr;
  struct work **hexworks;
  #ifdef DBG_HEXB
  pthread_t dbg_thr;
  int read_pos;
  int write_pos;
  uint64_t totworks;
  uint64_t sub_nonces;
  uint64_t hwerr;
  int roled;
  #endif
};

struct hexminerb_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t merkle[3];
  uint32_t a1;                  //midstate3[0]
  uint32_t a0;                  //midstate3[1]
  uint32_t e2;                  //midstate3[2]
  uint32_t e1;                  //midstate3[3]
  uint32_t e0;                  //midstate3[4]
  uint8_t midstate[32];
  uint32_t a2;                  //midstate3[5]
  uint32_t startnonce;          //midstate3[6]
  uint8_t id;
  uint8_t status;
  uint16_t hashclock;
  uint16_t chipcount;
  uint16_t refvoltage;
  uint16_t reftemperature;      //midstate3[7]
  uint16_t reffanrpm;           //midstate3[7]
  uint8_t csum;
  uint8_t pad[2];
} __attribute__ ((packed, aligned (4)));
#define HEXB_WORKANSWER_ADR	0x3000
#define HEXB_WORKQUEUE_ADR	0x4008
#define HEXB_PTCON_ADR		0x0C00
#define HEXB_START_STOP_ADR	0x646E
#define HEXMINERB_TASK_SIZE	(sizeof(struct hexminerb_task) - 2)
#define HEXB_MAX_WORK_SIZE		(sizeof(struct workb_result))
#define HEXB_BASE_WORK_SIZE		6       /* Min uint8_t startbyte + uint8_t datalength + uint8_t command + uint16_t
                                                 * address;
                                                 * + uint8_t csum */
extern int opt_hexminerb_core_voltage;
extern char *libhexb_set_config_voltage (char *arg);
extern struct hexminerb_info **hexminerb_info;
#endif /* USE_HEXMINERB */
#endif /* HEXB_H */
