/*$T indentinput.h GC 1.140 10/16/13 10:20:01 */
/*
 * Copyright 2013 Avalon project Copyright 2013 Con Kolivas <kernel@kolivas.org>
 * This program is free software;
 * you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version. See
 * COPYING for more details. Thank you guys!
 */
#ifndef HEX8_H
#define HEX8_H
#ifdef USE_HEXMINER8
#include "util.h"
/* hexminer8_task/work_reply status Definitions: */
//#define DBG_HEX8

#define HEX8_STAT_IDLE					0       /* Idle or data already Sent to the buffer */
#define HEX8_STAT_NEW_WORK				1       /* Request for write in the buffer */
#define HEX8_STAT_WAITING				2       /* Wait For Buffer Empty Position */
#define HEX8_STAT_CLR_BUFF				3       /* Force Buffer Empty */
#define HEX8_STAT_STOP_REQ				4       /* Stop Request */
#define HEX8_STAT_NEW_WORK_CLEAR_OLD		5       /* Clear Buffers and after that fill the first buffer */
#define HEX8_STAT_UNUSED					6
/* libhex8_eatHashData/BUF_reply status Definitions: */
#define HEX8_BUF_DATA 0
#define HEX8_BUF_ERR  1
#define HEX8_BUF_SKIP 2

#define HEXMINER8_ARRAY_PIC_SIZE		64
#define HEXMINER8_ARRAY_SIZE                  HEXMINER8_ARRAY_PIC_SIZE * 4
#define HEXMINER8_ARRAY_SIZE_REAL	HEXMINER8_ARRAY_SIZE - 2

#define HEX8_NONCE_CASH_SIZE				1

#define HEX8_USB_R_SIZE					64
#define HEX8_USB_WR_SIZE					64
#define HEX8_HASH_BUF_SIZE				2048*4

#define HEXMINER8_BULK_READ_TIMEOUT 1000
#define HEX8_USB_WR_TIME_OUT				500

#define HEX8_MINER_THREADS			1
#define HEX8_DEFAULT_MINER_NUM		0x01
#define HEX8_DEFAULT_ASIC_NUM		0x08
#define HEX8_MIN_FREQUENCY			0
#define HEX8_MAX_FREQUENCY			511
#define HEX8_DEFAULT_FREQUENCY		200
#define HEX8_DEFAULT_CORE_VOLTAGE	800     /* in millivolts */
#define HEX8_MIN_COREMV				300     /* in millivolts */
#define HEX8_MAX_COREMV	2101    /* in millivolts */
struct chip_results8
{
  uint8_t nonce_cache_write_pos;
  uint32_t nonces[HEX8_NONCE_CASH_SIZE];
};

struct work8_result
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
  uint8_t buf_empty_space;      //16 bit words aligned with lastchippos
  uint8_t good_engines;
  uint8_t dum;                  //7
  uint8_t csum;
  uint8_t pad[2];
} __attribute__ ((packed, aligned (4)));

struct hexminer8_info
{
  bool shut_read;
  bool shut_write;
  bool shut_reset;
  bool diff1;
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
  int dupe[HEX8_DEFAULT_ASIC_NUM];
  int matching_work[HEX8_DEFAULT_ASIC_NUM];
  int engines[HEX8_DEFAULT_ASIC_NUM];
  uint8_t wr_lastnonceid;
  uint8_t buf_empty_space;
  unsigned char *readbuf;
  struct work8_result *wr;
  struct chip_results8 *array_nonce_cache;
  struct thr_info *thr;
  struct work **hexworks;
  time_t last_chip_valid_work[HEX8_DEFAULT_ASIC_NUM];
  time_t power_checked;
  pthread_t write_thr;
  pthread_mutex_t lock;
  pthread_mutex_t power;
  cgsem_t wsem;

#ifdef DBG_HEX8
  pthread_t dbg_thr;
  int roled;
  int totworks;
  int read_pos;
  int buf_empty_was_64;
  int buf_empty_was_above_60;
  int buf_empty_was_below_5;
  int buf_empty_was_zero;
#endif

};

struct hexminer8_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint8_t midstate[32];         //8x32
  uint32_t merkle[3];           //3x32
  uint32_t difficulty;          //1x32
  uint16_t id;
  uint16_t status;              //1x32
  uint8_t csum;                 //1x32
  uint8_t pad[2];               //1x32
} __attribute__ ((packed, aligned (4)));

struct hexminer8_config_task
{
  uint8_t startbyte;
  uint8_t datalength;
  uint8_t command;
  uint16_t address;
  uint16_t hashclock;           //1x32
  uint16_t refvoltage;
  uint32_t difficulty;          //1x32
  uint8_t chip_mask;
  uint8_t wr_interwal;
  uint8_t csum;                 //1x32
} __attribute__ ((packed, aligned (4)));

#define HEX8_WORKANSWER_ADR	0x3000
#define HEXMINER8_TASK_SIZE	(sizeof(struct hexminer8_task)-2)
#define HEX8_MAX_WORK_SIZE		(sizeof(struct work8_result)-2)
#define HEX8_BASE_WORK_SIZE		6

extern int opt_hexminer8_core_voltage;
extern int opt_hexminer8_chip_mask;
extern int opt_hexminer8_set_config_diff_to_one;

extern char *libhex8_set_config_voltage (char *arg);
extern char *libhex8_set_config_chip_mask (char *arg);
extern char *libhex8_set_config_diff_to_one (char *arg);

extern struct hexminer8_info **hexminer8_info;
#endif /* USE_HEXMINER8 */
#endif /* HEX8_H */
