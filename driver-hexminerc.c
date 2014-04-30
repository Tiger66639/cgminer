/*$T indentinput.c GC 1.140 10/16/13 10:19:47 */
/*
 * Copyright 2013 Con Kolivas <kernel@kolivas.org> Copyright 2012-2013 Xiangfu
 * <xiangfu@openmobilefree.com> Copyright 2012 Luke Dashjr Copyright 2012 Andrew
 * Smith This program is free software;
 * you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation;
 * either version 3 of the License, or (at your option) any later version. See
 * COPYING for more details. Thank you guys!
 */
#include "config.h"
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <ctype.h>
#include <dirent.h>
#include <unistd.h>
#ifndef WIN32
#include <sys/select.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef O_CLOEXEC
#define O_CLOEXEC	0
#endif
#else
#include "compat.h"
#include <windows.h>
#include <io.h>
#endif
#include "elist.h"
#include "miner.h"
#include "usbutils.h"
#include "driver-hexminerc.h"
#include "util.h"

extern unsigned int work_block;
extern struct work *copy_work_noffset_fast_no_id(struct work *base_work, int noffset);
static int option_offset = -1;
struct device_drv hexminerc_drv;
int opt_hexminerc_core_voltage = HEXC_DEFAULT_CORE_VOLTAGE;
#include "libhexc.c"
extern bool no_work;
static void
hexminerc_flush_work (struct cgpu_info *hexminerc)
{
  struct hexminerc_info *info = hexminerc->device_data;
  
  cgsem_post (&info->qsem);

}

static int
hexminerc_send_task (struct hexminerc_task *ht, struct cgpu_info *hexminerc)
{
  int ret = 0;
  size_t nr_len = HEXMINERC_TASK_SIZE;
  struct hexminerc_info *info;
  info = hexminerc->device_data;

  libhexc_csum (&ht->startbyte, &ht->csum, &ht->csum);


  ret = libhexc_sendHashData (hexminerc, &ht->startbyte, nr_len);

  if (ret != nr_len)
    {
      libhexc_reset (hexminerc);
      info->usb_w_errors++;
      return -1;
    }

  return ret;
}

static inline void
hexminerc_create_task (bool reset_work, struct hexminerc_task *ht,
                       struct work *work)
{
  if (reset_work)
    {
      ht->status = HEXC_STAT_NEW_WORK_CLEAR_OLD;
    }
  else
    {
      ht->status = HEXC_STAT_NEW_WORK;
    }
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
  libhexc_calc_hexminer (work, ht);
}

static inline void
hexminerc_init_task (struct hexminerc_task *ht, struct hexminerc_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINERC_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (HEXC_WORKQUEUE_ADR);
  libhexc_generateclk (info->frequency, HEXC_DEFAULT_XCLKIN_CLOCK,
                       (uint32_t *) & ht->clockcfg[0]);
  libhexc_setvoltage (info->core_voltage, &ht->refvoltage);
  ht->chipcount = htole16 (info->asic_count);
  ht->hashclock = htole16 ((uint16_t) info->frequency);
  ht->startnonce = 0x00000000;
}
static bool
need_reset(struct cgpu_info *hexminerc)
{
	
  struct hexminerc_info *info = hexminerc->device_data;

	time_t now = time (NULL);
	bool ret = false;
	int i = 0;	
	int secs = 90;
	
	while (i < HEXC_DEFAULT_ASIC_NUM)
        {
        	
        	if(info->matching_work[i] && ((info->last_chip_valid_work[i] + secs) < now)) {
        		//applog(LOG_ERR, "HEXC %i Chip[%i] last valid work %i secs ago", hexminerc->device_id, i + 1, (int)(now-info->last_chip_valid_work[i]));
        		ret = true;
        		break;
        	}
        	i++;
        }	
        
   if(ret || no_work) {
   	ret = !no_work;
     i = 0; 	
     while (i < HEXC_DEFAULT_ASIC_NUM)
      	info->last_chip_valid_work[i++] = now;
   }
    
  return ret;      	
}
static void *
hexminerc_send_tasks (void *userdata)
{
  struct cgpu_info *hexminerc = (struct cgpu_info *) userdata;
  struct hexminerc_info *info = hexminerc->device_data;
  struct hexminerc_task *ht;
  struct thr_info *thr = info->thr;
  struct work *work = NULL;
  struct work *tmpwork = NULL;
  char threadname[24];
  int write_pos = 0;
  int jobs_to_send = 2;
  bool reset_work = true;
  int send_jobs, roll, ret;
  unsigned int work_block_local;
  bool power;
  snprintf (threadname, 24, "hexc_send/%d", hexminerc->device_id);
  RenameThread (threadname);
 // libhexc_reset (hexminerc);

  ht = calloc (sizeof (struct hexminerc_task), 1);
  hexminerc_init_task (ht, info);


  while (!libhexc_usb_dead (hexminerc))
    {
      send_jobs = 0;
      
        	if(time (NULL) - info->power_checked > 30) {
    	
    			info->power_checked = time (NULL);
    			mutex_lock(&info->power);
    			power = need_reset(hexminerc);
    			mutex_unlock(&info->power);
    			if(power) {
    				libhexc_set_word (hexminerc, HEXC_WORKQUEUE_ADR + 80, 0x0004);
            reset_work = true;
            cgsleep_ms (200);
    			}
    	}
      
      while ((work_block_local != work_block) || (send_jobs < jobs_to_send && (info->wr_status == HEXC_STAT_IDLE
                                || info->wr_status == HEXC_STAT_NEW_WORK))
             || reset_work)
        {
        again:
          if (!work)
            {
              roll = 0;
              work = get_work (thr, thr->id);
							work->ping = 1;
              if(work_block_local != work_block) {
        	 			reset_work = true;
        	 			work_block_local = work_block;
        			}
            }
            
        if (stale_work (work, false))
        {
          free_work (work);
          work = NULL;
          goto again;
        }   
          
          if (write_pos >= HEXMINERC_ARRAY_SIZE_REAL || reset_work)
            write_pos = 0;
        
              work->subid = write_pos;
              tmpwork = copy_work_noffset_fast_no_id (work, roll++);
              hexminerc_create_task (reset_work, ht,
                                 tmpwork);
              
              mutex_lock (&info->lock);
          		free_work (info->hexworks[write_pos]);
              info->hexworks[write_pos] = tmpwork;
              mutex_unlock (&info->lock);
              
              if (work->drv_rolllimit)
                {
                  work->drv_rolllimit--;
                }
              else
                {
                  free_work (work);
                  work = NULL;
                }
          
          ret = hexminerc_send_task (ht, hexminerc);

          write_pos++;
          send_jobs++;

          if (ret == HEXMINERC_TASK_SIZE && reset_work)
            {       
              reset_work = false; 
              send_jobs-=2;
            }
        }
 		if(!reset_work)
      cgsem_mswait (&info->qsem, info->usb_timing);
    }
  if (work)
    free_work (work);
  free (ht);
  pthread_exit (NULL);
}




static struct cgpu_info *
hexminerc_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int miner_count, asic_count, frequency;
  int this_option_offset = ++option_offset;
  struct hexminerc_info *info;
  struct cgpu_info *hexminerc;
  bool configured;
  int i = 0;
  hexminerc = usb_alloc_cgpu (&hexminerc_drv, HEXC_MINER_THREADS);
  if (!usb_init (hexminerc, dev, found))
    {
      usb_uninit (hexminerc);
      return NULL;
    }
  hexminerc->device_data = calloc (sizeof (struct hexminerc_info), 1);
  if (unlikely (!(hexminerc->device_data)))
    {
      hexminerc->device_data = NULL;
      usb_uninit (hexminerc);
      return NULL;
    }
  configured =
    libhexc_get_options (this_option_offset, &asic_count, &frequency);
  if (opt_hexminerc_core_voltage < HEXC_MIN_COREMV
      || opt_hexminerc_core_voltage > HEXC_MAX_COREMV)
    {
      applog
        (LOG_ERR,
         "Invalid hexminerc-voltage %d must be %dmV - %dmV",
         opt_hexminerc_core_voltage, HEXC_MIN_COREMV, HEXC_MAX_COREMV);
      free (hexminerc->device_data);
      hexminerc->device_data = NULL;
      usb_uninit (hexminerc);
      return NULL;
    }
  info = hexminerc->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERC_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminerc->device_data);
      hexminerc->device_data = NULL;
      usb_uninit (hexminerc);
      return NULL;
    }
  info->wr = (struct workc_result *) malloc (sizeof (struct workc_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_resultsc));
 
  info->readbuf = calloc (HEXC_HASH_BUF_SIZE, sizeof (unsigned char));
  
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  
  info->wr_status = HEXC_STAT_IDLE;
  info->miner_count = HEXC_DEFAULT_MINER_NUM;
  info->asic_count = HEXC_DEFAULT_ASIC_NUM;
  info->frequency = HEXC_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEXC_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminerc_core_voltage;
  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }
  info->usb_timing =
    (int64_t) (0x80000000ll / 1000 / info->asic_count / info->frequency *
               HEXMINERC_WORK_FACTOR * 2);
  if (!add_cgpu (hexminerc))
    {
      free (info->hexworks);
      free (hexminerc->device_data);
      hexminerc->device_data = NULL;
      hexminerc = usb_free_cgpu (hexminerc);
      usb_uninit (hexminerc);
      return NULL;
    }
   
    while (i < HEXMINERC_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
    }
    
  i = 0; 
  info->power_checked = time (NULL);
  while (i < HEXC_DEFAULT_ASIC_NUM)
    {
      info->matching_work[i] = 0;
      info->last_chip_valid_work[i++] = time (NULL);
    }
    
  libhexc_generatenrange_new ((unsigned char *) &info->nonces_range,
                              info->asic_count);
  return hexminerc;
}

static void
hexminerc_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminerc_drv, hexminerc_detect_one);
}

static void
do_hexminerc_close (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  int i = 0;
  cgsleep_ms (300);
  pthread_join (info->write_thr, NULL);
  
  pthread_mutex_destroy (&info->lock);
  pthread_mutex_destroy (&info->power);
  cgsem_destroy (&info->qsem);

  while (i < HEXMINERC_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->readbuf);
  free (info->array_nonce_cache);
  free (info->wr);
  //Hotplug Story
  //free (hexminerc->device_data);
  //hexminerc->device_data = NULL;
  //thr->cgpu = usb_free_cgpu(hexminerc);
}

static void
hexminerc_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;

  cgsem_post (&info->qsem);
  
  do_hexminerc_close (thr);
  
  usb_nodev(hexminerc);
}



static bool
hexminerc_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  info->thr = thr;
  mutex_init (&info->lock);
  mutex_init (&info->power);
  cgsem_init (&info->qsem);

  if (pthread_create
      (&info->write_thr, NULL, hexminerc_send_tasks, (void *) hexminerc))
    quit (1, "Failed to create hexminerc write_thr");

  return true;
}


static int64_t
hexminerc_scanhash (struct thr_info *thr)
{

  struct cgpu_info *hexminerc = thr->cgpu;
  struct hexminerc_info *info = hexminerc->device_data;
  
  uint32_t nonce;
  int notdupe, found, i, lastchippos;
 
  int ret_r = 0;
  
  int64_t hash_count = 0;
   if (libhexc_usb_dead (hexminerc))
    return -1;
    
    if (info->hash_write_pos + HEXC_USB_R_SIZE >= HEXC_HASH_BUF_SIZE)
        {
          info->hash_write_pos = info->hash_write_pos - info->hash_read_pos;
          memcpy (info->readbuf, info->readbuf + info->hash_read_pos, info->hash_write_pos);
          info->hash_read_pos = 0;
   }
   
   if (info->hash_write_pos - info->hash_read_pos > 7)
        {
        again:
          ret_r =
            libhexc_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                                 &info->hash_write_pos);
          if (ret_r > HEXC_BUF_DATA)
            goto out;

          info->wr_status = info->wr->status;
          if (info->wr->datalength == 1)
            goto done;

          if (info->wr->lastnonceid > HEXMINERC_ARRAY_SIZE_REAL)
            info->wr->lastnonceid = 0;

          nonce = htole32 (info->wr->lastnonce);
          i = 0;
          while (i < info->asic_count)
            {
              if (nonce < info->nonces_range[++i])
                {
                  lastchippos = --i;
                  break;
                }
            }

            if (i == info->asic_count)
            lastchippos = info->asic_count - 1;

          notdupe =
            libhexc_cachenonce (&info->array_nonce_cache[lastchippos], nonce);
          if (lastchippos > 0)
            notdupe &= libhexc_cachenonce (&info->array_nonce_cache[0], nonce);
            
          if (notdupe)
            {
              found = hexminerc_predecode_nonce (hexminerc, thr, nonce,
                                                 info->wr->lastnonceid);
              if (found > 0)
                {
                	
                	mutex_lock(&info->power);
                  info->matching_work[lastchippos]++;
                  info->last_chip_valid_work[(uint8_t) lastchippos] = time (NULL);
                  mutex_unlock(&info->power);
                  if (hash_count == 0)
                    libhexc_getvoltage (htole16 (info->wr->lastvoltage),
                                        &info->pic_voltage_readings);
                  hash_count += found;
                  
                }
              else
                {
                  inc_hw_errors (thr);
                }
            }
          else
            {
              info->dupe[lastchippos]++;
            }  
            
            
             out:
          if (ret_r == HEXC_BUF_ERR)
            {
              info->usb_r_errors++;
            }
        done:
          if (info->hash_write_pos - info->hash_read_pos >= HEXC_MAX_WORK_SIZE)
            goto again;
        }

      ret_r =
        libhexc_readHashData (hexminerc, info->readbuf, &info->hash_write_pos,
                              HEXMINERC_BULK_READ_TIMEOUT, true);
                              

 
   
  
      hash_count = (int64_t) (0xffffffffull * hash_count);
      
 
    
  if (libhexc_usb_dead (hexminerc))
    return -1;
    
    return hash_count;
}


static void
get_hexminerc_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminerc)
{
	if (!hexminerc->device_data)
		return;
  struct hexminerc_info *info = hexminerc->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

static struct api_data *
hexminerc_api_stats (struct cgpu_info *cgpu)
{
  struct api_data *root = NULL;
  struct hexminerc_info *info = cgpu->device_data;
  if (!info)
		return NULL;
  uint64_t dh64, dr64;
  double dev_runtime;
  struct timeval now;
  int i;
  char displayed_hashes[16], displayed_rolling[16];
  double hwp =
    (cgpu->hw_errors +
     cgpu->diff1) ? (double) (cgpu->hw_errors) / (double) (cgpu->hw_errors +
                                                           cgpu->diff1) : 0;
  if (cgpu->dev_start_tv.tv_sec == 0)
    dev_runtime = total_secs;
  else
    {
      cgtime (&now);
      dev_runtime = tdiff (&now, &(cgpu->dev_start_tv));
    }
  if (dev_runtime < 1.0)
    dev_runtime = 1.0;
  dh64 = (double) cgpu->total_mhashes / dev_runtime * 1000000ull;
  dr64 = (double) cgpu->rolling * 1000000ull;
  suffix_string (dh64, displayed_hashes, sizeof (displayed_hashes), 4);
  suffix_string (dr64, displayed_rolling, sizeof (displayed_rolling), 4);
  root = api_add_string (root, "MHS 5s", displayed_rolling, true);
  root = api_add_string (root, "MHS av", displayed_hashes, true);
  root = api_add_int (root, "Hardware Errors", &(cgpu->hw_errors), true);
  root = api_add_percent (root, "Hardware Errors%", &hwp, true);
  root = api_add_int (root, "USB Read Errors", &(info->usb_r_errors), true);
  root = api_add_int (root, "USB Write Errors", &(info->usb_w_errors), true);
  root =
    api_add_int (root, "USB Reset Count", &(info->usb_reset_count), true);
  root =
    api_add_time (root, "Last Share Time", &(cgpu->last_share_pool_time),
                  true);
  root = api_add_int (root, "Chip Count", &(info->asic_count), true);
  root = api_add_int (root, "Frequency", &(info->frequency), true);
  root = api_add_int (root, "Core Voltage", &(info->core_voltage), true);
  root =
    api_add_int (root, "PIC Voltage Readings", &(info->pic_voltage_readings),
                 true);
  for (i = 0; i < info->asic_count; i++)
    {
      /*~ */
      char mcw[24];
      /*~ */
      sprintf (mcw, "Chip%d Nonces", i + 1);
      root = api_add_int (root, mcw, &(info->matching_work[i]), true);
      sprintf (mcw, "Chip%d Dupes", i + 1);
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
    }
  return root;
}



struct device_drv hexminerc_drv = {
  .drv_id = DRIVER_hexminerc,
  .dname = "hexminerc",
  .name = "HEXc",
  .drv_detect = hexminerc_detect,
  .thread_init = hexminerc_thread_init,
  .hash_work = hash_queued_work,
  .scanwork = hexminerc_scanhash,
  .flush_work = hexminerc_flush_work,
  .get_api_stats = hexminerc_api_stats,
  .get_statline_before = get_hexminerc_statline_before,
  .thread_shutdown = hexminerc_shutdown,
};
