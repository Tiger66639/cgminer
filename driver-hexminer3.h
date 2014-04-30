/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
/*
 * Copyright 2013 Avalon project Copyright 2013 Con Kolivas <kernel@kolivas.org>
 * This program is free software;
 * you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version. See
 * COPYING for more details. Thank you guys!
 */
#ifndef HEX3_H
#define HEX3_H
#ifdef USE_HEXMINER3
#include "util.h"
/* hexminer3_task/work_reply status Definitions: */
//#define DBG_HEX3
#define PWR_HEX3

#define HEX3_STAT_IDLE					0       /* Idle or data already Sent to the buffer */
#define HEX3_STAT_NEW_WORK				1       /* Request for write in the buffer */
#define HEX3_STAT_WAITING				2       /* Wait For Buffer Empty Position */
#define HEX3_STAT_CLR_BUFF				3       /* Force Buffer Empty */
#define HEX3_STAT_STOP_REQ				4       /* Stop Request */
#define HEX3_STAT_NEW_WORK_CLEAR_OLD		5       /* Clear Buffers and after that fill the first buffer */
#define HEX3_STAT_UNUSED					6
/* libhex3_eatHashData/BUF_reply status Definitions: */
#define HEX3_BUF_DATA 0
#define HEX3_BUF_ERR  1
#define HEX3_BUF_SKIP 2

#define HEXMINER3_ARRAY_PIC_SIZE		64
#define HEXMINER3_ARRAY_SIZE                  HEXMINER3_ARRAY_PIC_SIZE * 4
#define HEXMINER3_ARRAY_SIZE_REAL	HEXMINER3_ARRAY_SIZE - 2

#define HEX3_NONCE_CASH_SIZE				16

#define HEX3_USB_R_SIZE					64
#define HEX3_USB_WR_SIZE					64
#define HEX3_HASH_BUF_SIZE				2048*2

#define HEXMINER3_BULK_READ_TIMEOUT 1000
#define HEX3_USB_WR_TIME_OUT				500

#define HEX3_MINER_THREADS			1
#define HEX3_DEFAULT_MINER_NUM		0x01
#define HEX3_DEFAULT_ASIC_NUM		0x10
#define HEX3_MIN_FREQUENCY			5000
#define HEX3_MAX_FREQUENCY			10000
#define HEX3_DEFAULT_FREQUENCY		7000
#define HEX3_DEFAULT_CORE_VOLTAGE	850     /* in millivolts */
#define HEX3_MIN_COREMV				600     /* in millivolts */
#define HEX3_MAX_COREMV	1100    /* in millivolts */
struct chip_results3
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEX3_NONCE_CASH_SIZE];
};

struct work3_result
{
  uint8_t startbyte;
  uint8_t datalength; //16
  uint8_t command;
  uint16_t address;     //16
  uint32_t lastnonce;           //32
  uint8_t lastnonceid;          //16
  uint8_t status;                 //16
  uint16_t lastvoltage;         
  uint8_t lastchippos;      //3008    
  uint8_t buf_empty_space;      //3009
  uint16_t dum;               ///A,B
  uint32_t word1;
  uint32_t word2;
  uint8_t csum;             //16
  uint16_t dum1;
 
} __attribute__ ((packed, aligned (4)));

struct hexminer3_info
{
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  int wsem_timing;
  int chip_mask;
  int miner_count;
  int asic_count;
  int core_voltage;
  int frequency;
  int usb_r_errors;
  int usb_w_errors;
  int usb_reset_count;
  int pic_voltage_readings;
  int hash_read_pos;
  int hash_write_pos;
  int dupe[HEX3_DEFAULT_ASIC_NUM];
  int matching_work[HEX3_DEFAULT_ASIC_NUM];
  uint8_t wr_lastnonceid;
  uint8_t buf_empty_space;
  unsigned char *readbuf;
  struct work3_result *wr;
  struct chip_results3 *array_nonce_cache;
  struct thr_info *thr;
  struct work **hexworks;
  time_t last_chip_valid_work[HEX3_DEFAULT_ASIC_NUM];
  uint32_t nonces_range[HEX3_DEFAULT_ASIC_NUM];
  time_t power_checked;
  pthread_t write_thr;
  pthread_mutex_t lock;
  pthread_mutex_t power;
  cgsem_t wsem;
#ifdef DBG_HEX3
  pthread_t dbg_thr;
  int roled;
  int read_pos;
  int buf_empty_was_64;
  int buf_empty_was_above_60;
  int buf_empty_was_below_5;
  int buf_empty_was_zero;
  uint32_t word1;
  uint32_t word2;
  uint32_t send_j;
  uint32_t r_j;
#endif
};

struct hexminer3_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint32_t merkle[3];
  uint32_t a1;
  uint32_t a0;
  uint32_t e2;
  uint32_t e1;
  uint32_t e0;
  uint8_t midstate[32];
  uint32_t a2;
  uint8_t id;
  uint8_t status;
  uint8_t csum;
} __attribute__ ((packed, aligned (4)));


struct hexminer3_config_task
{
  uint8_t startbyte;
  uint8_t datalength; //16
  uint8_t command;
  uint16_t address;
  uint16_t hashclock;           
  uint16_t refvoltage;
  uint32_t startnonce;
  uint16_t chipcount; //6x16
  uint8_t chip_mask;  //16
  uint8_t pad;
  uint8_t csum;                 //16
  uint8_t pad1[2];
} __attribute__ ((packed, aligned (4)));

 
#define HEX3_WORKANSWER_ADR	0x3000
#define HEXMINER3_TASK_SIZE	(sizeof(struct hexminer3_task))
#define HEX3_MAX_WORK_SIZE		(sizeof(struct work3_result) - 2)
#define HEX3_BASE_WORK_SIZE		6

extern int opt_hexminer3_core_voltage;
extern int opt_hexminer3_chip_mask;

extern char *libhex3_set_config_voltage (char *arg);
extern char *libhex3_set_config_chip_mask (char *arg);

extern struct hexminer3_info **hexminer3_info;
#endif /* USE_HEXMINER3 */
#endif /* HEX3_H */
