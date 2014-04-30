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
#include "driver-hexminerb.h"
#include "util.h"

extern unsigned int work_block;
extern struct work *copy_work_noffset_fast_no_id(struct work *base_work, int noffset);
static int option_offset = -1;
struct device_drv hexminerb_drv;
int opt_hexminerb_core_voltage = HEXB_DEFAULT_CORE_VOLTAGE;
#include "libhexb.c"
/*
    We use a replacement algorithm to only remove references to work done from the buffer when we need the extra space
    for new work. Thanks to Avalon code with some mods
 */


static void
hexminerb_flush_work (struct cgpu_info *hexminerb)
{
  struct hexminerb_info *info = hexminerb->device_data;


  cgsem_post (&info->wsem);
#ifdef DBG_HEXB
  applog (LOG_ERR, "HEXb%i hexminerb_flush_work", hexminerb->device_id);
#endif

}

static int
hexminerb_send_task (struct hexminerb_task *ht, struct cgpu_info *hexminerb)
{
  int ret = 0;
  size_t nr_len = HEXMINERB_TASK_SIZE;
  struct hexminerb_info *info;
  info = hexminerb->device_data;

  libhexb_csum (&ht->startbyte, &ht->csum, &ht->csum);


  ret = libhexb_sendHashData (hexminerb, &ht->startbyte, nr_len);

  if (ret != nr_len)
    {
      libhexb_reset (hexminerb);
      info->usb_w_errors++;
      return -1;
    }

  return ret;
}


static inline void
hexminerb_create_task (bool reset_work, struct hexminerb_task *ht,
                       struct work *work)
{
  if (reset_work)
    {
      ht->status = HEXB_STAT_NEW_WORK_CLEAR_OLD;
    }
  else
    {
      ht->status = HEXB_STAT_NEW_WORK;
    }
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
  BITFURY_MS3compute (work, ht);
}

static inline void
hexminerb_init_task (struct hexminerb_task *ht, struct hexminerb_info *info)
{

  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINERB_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (HEXB_WORKQUEUE_ADR);
  libhexb_setvoltage (info->core_voltage, &ht->refvoltage);
  ht->chipcount = htole16 (info->asic_count);
  ht->hashclock = htole16 ((uint16_t) info->frequency);
}
static void *
hexminerb_send_tasks (void *userdata)
{
  struct cgpu_info *hexminerb = (struct cgpu_info *) userdata;
  struct hexminerb_info *info = hexminerb->device_data;
  struct hexminerb_task *ht;
  struct thr_info *thr = info->thr;
  struct work *work = NULL;
  struct work *tmpwork = NULL;
  unsigned int work_block_local;
  char threadname[24];
  int write_pos = 0;
  int jobs_to_send = 1;
  bool reset_work = true;
  int send_jobs, roll, ret;
  
  snprintf (threadname, 24, "hexb_send/%d", hexminerb->device_id);
  RenameThread (threadname);
  //libhexb_reset (hexminerb);
  ht = calloc (sizeof (struct hexminerb_task), 1);
  
  hexminerb_init_task (ht, info);

  while (!libhexb_usb_dead (hexminerb))
    {
      send_jobs = 0;
                
      while ((work_block_local != work_block) || (send_jobs < jobs_to_send && (info->wr_status == HEXB_STAT_IDLE
                                || info->wr_status == HEXB_STAT_NEW_WORK))
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
          
          if (write_pos >= HEXMINERB_ARRAY_SIZE_REAL || reset_work)
            write_pos = 0;
          
              work->subid = write_pos;
              tmpwork = copy_work_noffset_fast_no_id (work, roll++);
              hexminerb_create_task (reset_work, ht,
                                 tmpwork);
              
              mutex_lock (&info->lock);
          		free_work (info->hexworks[write_pos]);
              info->hexworks[write_pos] = tmpwork;
              mutex_unlock (&info->lock);
              
              if (work->drv_rolllimit)
                {
                  work->drv_rolllimit--;
#ifdef DBG_HEXB
                  info->roled++;
#endif
                }
              else
                {
                  free_work (work);
                  work = NULL;
                }
              ret = hexminerb_send_task (ht, hexminerb);
#ifdef DBG_HEXB
          info->write_pos = write_pos;
#endif
          write_pos++;
          send_jobs++;

          if (ret == HEXMINERB_TASK_SIZE && reset_work)
            {
              reset_work = false; 
              send_jobs-=2;
            }
        }
//330
     if(!reset_work)
      cgsem_mswait (&info->wsem, 44);
    }
  if (work)
    free_work (work);
  free (ht);
  pthread_exit (NULL);
}

static struct cgpu_info *
hexminerb_detect_one (libusb_device * dev, struct usb_find_devices *found)
{

  int miner_count, asic_count, frequency;
  int this_option_offset = ++option_offset;
  struct hexminerb_info *info;
  struct cgpu_info *hexminerb;
  bool configured;
  int i = 0;

  hexminerb = usb_alloc_cgpu (&hexminerb_drv, HEXB_MINER_THREADS);
  if (!usb_init (hexminerb, dev, found))
    {
      usb_uninit (hexminerb);
      return NULL;
    }
  hexminerb->device_data = calloc (sizeof (struct hexminerb_info), 1);

  if (unlikely (!(hexminerb->device_data)))
    {
      hexminerb->device_data = NULL;
      usb_uninit (hexminerb);
      return NULL;
    }
  configured =
    libhexb_get_options (this_option_offset, &asic_count, &frequency);
  if (opt_hexminerb_core_voltage < HEXB_MIN_COREMV
      || opt_hexminerb_core_voltage > HEXB_MAX_COREMV)
    {

      applog
        (LOG_ERR,
         "Invalid hexminerb-voltage %d must be %dmV - %dmV",
         opt_hexminerb_core_voltage, HEXB_MIN_COREMV, HEXB_MAX_COREMV);
      free (hexminerb->device_data);
      hexminerb->device_data = NULL;
      usb_uninit (hexminerb);
      return NULL;
    }
  info = hexminerb->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERB_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminerb->device_data);
      hexminerb->device_data = NULL;
      usb_uninit (hexminerb);
      return NULL;
    }
  
  info->wr = (struct workb_result *) malloc (sizeof (struct workb_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_resultsb));
  info->readbuf = calloc (HEXB_HASH_BUF_SIZE, sizeof (unsigned char));
  
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->wr_status = HEXB_STAT_IDLE;
  info->miner_count = HEXB_DEFAULT_MINER_NUM;
  info->asic_count = HEXB_DEFAULT_ASIC_NUM;
  info->frequency = HEXB_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEXB_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminerb_core_voltage;
  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }
  if (!add_cgpu (hexminerb))
    {
      free (info->hexworks);
      free (hexminerb->device_data);
      hexminerb->device_data = NULL;
      hexminerb = usb_free_cgpu (hexminerb);
      usb_uninit (hexminerb);
      return NULL;
    }
  while (i < HEXMINERB_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
    }

  return hexminerb;
}

static void
hexminerb_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminerb_drv, hexminerb_detect_one);
}

static void
do_hexminerb_close (struct thr_info *thr)
{
  struct cgpu_info *hexminerb = thr->cgpu;
  struct hexminerb_info *info = hexminerb->device_data;
  int i = 0;
  cgsleep_ms (500);
  
  pthread_join (info->write_thr, NULL);
  pthread_mutex_destroy (&info->lock);
#ifdef DBG_HEXB
  pthread_join (info->dbg_thr, NULL);
#endif
  cgsem_destroy (&info->wsem);
  
  while (i < HEXMINERB_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->readbuf);
  free (info->array_nonce_cache);
  free (info->wr);
  //usb_uninit(hexminerb);
  //Hotplug fucks on full mem free :) 
  //free (hexminerb->device_data);
  //hexminerb->device_data = NULL;
  //thr->cgpu = usb_free_cgpu(hexminerb);

}

static void
hexminerb_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminerb = thr->cgpu;
  struct hexminerb_info *info = hexminerb->device_data;

  cgsem_post (&info->wsem);
  
  do_hexminerb_close (thr);
  
  usb_nodev(hexminerb);
}


#ifdef DBG_HEXB
static void *
hexminerb_get_stats (void *userdata)
{
  struct cgpu_info *hexminerb = (struct cgpu_info *) userdata;
  struct hexminerb_info *info = hexminerb->device_data;
  char threadname[24];
  snprintf (threadname, 24, "hexb_dbg/%d", hexminerb->device_id);
  RenameThread (threadname);

  while (!libhexb_usb_dead (hexminerb))
    {

      cgsleep_ms (30 * 1000);


    }
  pthread_exit (NULL);
}
#endif

static bool
hexminerb_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminerb = thr->cgpu;
  struct hexminerb_info *info = hexminerb->device_data;
  info->thr = thr;
  
 mutex_init (&info->lock);
 cgsem_init (&info->wsem);

  if (pthread_create
      (&info->write_thr, NULL, hexminerb_send_tasks, (void *) hexminerb))
    quit (1, "Failed to create hexminerb write_thr");

  return true;
}


static int64_t
hexminerb_scanhash (struct thr_info *thr)
{

  struct cgpu_info *hexminerb = thr->cgpu;
  struct hexminerb_info *info = hexminerb->device_data;
  
  uint32_t nonce;
  int found;
 
  int ret_r = 0;
  
  int64_t hash_count = 0;
  
  if (libhexb_usb_dead (hexminerb))
    return -1;

  if (info->hash_write_pos + HEXB_USB_R_SIZE >= HEXB_HASH_BUF_SIZE)
        {
          info->hash_write_pos = info->hash_write_pos - info->hash_read_pos;
          memcpy (info->readbuf, info->readbuf + info->hash_read_pos, info->hash_write_pos);
          info->hash_read_pos = 0;
   }
   
   
   if (info->hash_write_pos - info->hash_read_pos > 7)
        {
        again:
          ret_r =
            libhexb_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                                 &info->hash_write_pos);
          if (ret_r > HEXB_BUF_DATA)
            goto out;

          info->wr_status = info->wr->status;
          if (info->wr->datalength == 1)
            goto done;

          if (info->wr->lastnonceid > HEXMINERB_ARRAY_SIZE_REAL)
            info->wr->lastnonceid = 0;

          if (info->wr->prevnonceid > HEXMINERB_ARRAY_SIZE_REAL)
            info->wr->prevnonceid = 0;

          if (info->wr->lastchippos > 15)
            info->wr->lastchippos = 15;

          if (libhexb_cachenonce
              (&info->array_nonce_cache[info->wr->lastchippos], info->wr->lastnonce))
            {
              nonce = decnonce (htole32 (info->wr->lastnonce));

              found = hexminerb_predecode_nonce (hexminerb, thr, nonce,
                                                 info->wr->lastnonceid);
#ifdef DBG_HEXB
              if (found > 0)
                info->read_pos = info->wr->lastnonceid;

#endif
              if (found == 0)
                {
                  found = hexminerb_predecode_nonce (hexminerb, thr, nonce,
                                                     info->wr->prevnonceid);
#ifdef DBG_HEXB
                  if (found > 0)
                    info->read_pos = info->wr->prevnonceid;

#endif
                }

              if (found > 0)
                {
                  if (hash_count == 0)
                    libhexb_getvoltage (htole16 (info->wr->lastvoltage),
                                        &info->pic_voltage_readings);
                  
                  hash_count += found;
                  
                  info->matching_work[info->wr->lastchippos]++;
                }
              else
                {
                  //Due to implementation there is no way for now to count them. 
                  //The number is inaccurate and too big!

                  //inc_hw_errors (thr);
                }
            }
          else
            {
              info->dupe[info->wr->lastchippos]++;
            }
        out:
          if (ret_r == HEXB_BUF_ERR)
            {
              info->usb_r_errors++;
            }
        done:
          //More nonces 
          if (info->hash_write_pos - info->hash_read_pos >= HEXB_MAX_WORK_SIZE)
            goto again;
        }


      ret_r =
        libhexb_readHashData (hexminerb, info->readbuf, &info->hash_write_pos,
                              HEXMINERB_BULK_READ_TIMEOUT, true);
   
  
      hash_count = (int64_t) (0xffffffffull * hash_count);
      
  
  
  
  if (libhexb_usb_dead (hexminerb))
    return -1;

  return hash_count;

}

static void
get_hexminerb_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminerb)
{
  if (!hexminerb->device_data)
		return;
  struct hexminerb_info *info = hexminerb->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

static struct api_data *
hexminerb_api_stats (struct cgpu_info *cgpu)
{

  struct api_data *root = NULL;
  struct hexminerb_info *info = cgpu->device_data;
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
      char mcw[24];
      sprintf (mcw, "Chip%d Nonces", i + 1);
      root = api_add_int (root, mcw, &(info->matching_work[i]), true);
      sprintf (mcw, "Chip%d Dupes", i + 1);
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
    }
  return root;
}



struct device_drv hexminerb_drv = {
  .drv_id = DRIVER_hexminerb,
  .dname = "hexminerb",
  .name = "HEXb",
  .thread_init = hexminerb_thread_init,
  .drv_detect = hexminerb_detect,
  .hash_work = hash_queued_work,
  .scanwork = hexminerb_scanhash,
  .flush_work = hexminerb_flush_work,
  .get_api_stats = hexminerb_api_stats,
  .get_statline_before = get_hexminerb_statline_before,
  .thread_shutdown = hexminerb_shutdown,
};
