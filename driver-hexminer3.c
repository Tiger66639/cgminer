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
#include "driver-hexminer3.h"

#include "util.h"
extern unsigned int work_block;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
static int option_offset = -1;
struct device_drv hexminer3_drv;
extern bool no_work;
int opt_hexminer3_chip_mask = 0xFF;
//int opt_hexminer3_set_config_diff_to_one = 1;
int opt_hexminer3_core_voltage = HEX3_DEFAULT_CORE_VOLTAGE;

#include "libhex3.c"

/*
    We use a replacement algorithm to only remove references to work done from the buffer when we need the extra space
    for new work. Thanks to Avalon code with some mods
 */

static void
hexminer3_flush_work (struct cgpu_info *hexminer3)
{
  struct hexminer3_info *info = hexminer3->device_data;

  cgsem_post (&info->wsem);
#ifdef DBG_HEX3
  applog (LOG_ERR, "HEX3 %i  hexminer3_flush_work !", hexminer3->device_id);
#endif

}

static int
hexminer3_send_task (struct hexminer3_task *ht, struct cgpu_info *hexminer3)
{
  int ret = 0;
  size_t nr_len = HEXMINER3_TASK_SIZE;
  struct hexminer3_info *info;
  info = hexminer3->device_data;

  libhex3_csum (&ht->startbyte, &ht->csum, &ht->csum);

  ret = libhex3_sendHashData (hexminer3, &ht->startbyte, nr_len);

  if (ret != nr_len)
    {
      libhex3_reset (hexminer3);
      info->usb_w_errors++;
      return -1;
    }

  return ret;
}

static inline void
hexminer3_create_task (bool reset_work, struct hexminer3_task *ht,
                       struct work *work)
{
  if (reset_work)
    {
      ht->status = HEX3_STAT_NEW_WORK_CLEAR_OLD;
    }
  else
    {
      ht->status = HEX3_STAT_NEW_WORK;
    }
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = (uint8_t) work->subid;
  libhex3_calc_hexminer (work, ht);

}

static inline void
hexminer3_init_task_c (struct hexminer3_config_task *htc,
                       struct hexminer3_info *info)
{
  htc->startbyte = 0x53;
  htc->datalength =
    (uint8_t) ((sizeof (struct hexminer3_config_task) - 6 - 2) / 2);
  htc->command = 0x57;
  htc->address = htole16 (0x30C0);
  htc->hashclock = htole16 ((uint16_t) info->frequency);

  libhex3_setvoltage (info->core_voltage, &htc->refvoltage);
  htc->chip_mask = (uint8_t) info->chip_mask;
  htc->chipcount = htole16 (info->asic_count);
  libhex3_csum (&htc->startbyte, &htc->csum, &htc->csum);
}

static inline void
hexminer3_init_task (struct hexminer3_task *ht, struct hexminer3_info *info)
{
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINER3_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (0x3080);

}

#ifdef PWR_HEX3
static bool
need_reset (struct cgpu_info *hexminer3)
{

  struct hexminer3_info *info = hexminer3->device_data;

  time_t now = time (NULL);
  bool ret = false;
  int i = 0;
  int secs = 20;


  while (i < HEX3_DEFAULT_ASIC_NUM)
    {

      if ((info->last_chip_valid_work[i] + secs < now))
        {
#ifdef DBG_HEX3
          applog (LOG_ERR, "HEX3 %i Chip[%i] last valid work %i secs ago",
                  hexminer3->device_id, i + 1,
                  (int) (now - info->last_chip_valid_work[i]));
#endif
          ret = true;
          break;
        }
      i++;
    }

  if (ret || no_work)
    {
      ret = !no_work;
      i = 0;
      while (i < HEX3_DEFAULT_ASIC_NUM)
        info->last_chip_valid_work[i++] = now;
    }

  return ret;
}
#endif

static void *
hexminer3_send_tasks (void *userdata)
{
  struct cgpu_info *hexminer3 = (struct cgpu_info *) userdata;
  struct hexminer3_info *info = hexminer3->device_data;
  struct hexminer3_task *ht;
  struct thr_info *thr = info->thr;
  struct hexminer3_config_task *htc;
  struct work *work = NULL;
  struct work *tmpwork = NULL;
  char threadname[24];
  int write_pos = 0;
  int jobs_to_send = 2;

  int send_jobs, roll, ret;
  struct timeval tm;
  unsigned int work_block_local;

  bool reset_work = true;
  bool power = false;
  snprintf (threadname, 24, "hex3_send/%d", hexminer3->device_id);
  RenameThread (threadname);
  htc = calloc (sizeof (struct hexminer3_config_task), 1);

  hexminer3_init_task_c (htc, info);
  //Wait for asic reset
  cgsleep_ms (100);

  ret =
    libhex3_sendHashData (hexminer3, &htc->startbyte,
                          sizeof (struct hexminer3_config_task) - 2);

  if (ret != sizeof (struct hexminer3_config_task) - 2)
    applog (LOG_ERR, "HEX3 %i Send config failed", hexminer3->device_id);

  ht = calloc (sizeof (struct hexminer3_task), 1);
  hexminer3_init_task (ht, info);

  while (!libhex3_usb_dead (hexminer3))
    {
#ifdef PWR_HEX3
      if (time (NULL) - info->power_checked > 30)
        {

          info->power_checked = time (NULL);
          mutex_lock (&info->power);
          power = need_reset (hexminer3);
          mutex_unlock (&info->power);
          if (power)
            {
#ifdef DBG_HEX3
              applog (LOG_ERR, "hex3 %i RES", hexminer3->device_id);
#endif
#ifndef DBG_HEX3
              libhex3_set_word (hexminer3, 0x3080 + HEXMINER3_TASK_SIZE - 8,
                                0x0004);
#endif
              reset_work = true;
              cgsleep_ms (200);
            }
        }
#endif
      send_jobs = 0;


      while ((work_block_local != work_block)
             || (info->buf_empty_space > 30 && send_jobs < jobs_to_send)
             || reset_work)
        {
        again:
          if (!work)
            {
              roll = 0;
              work = get_work (thr, thr->id);
              work->ping = 1;
              if (work_block_local != work_block)
                {
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

          if (write_pos >= HEXMINER3_ARRAY_SIZE_REAL || reset_work)
            write_pos = 0;

          work->subid = write_pos;
          tmpwork = copy_work_noffset_fast_no_id (work, roll++);
#ifdef DBG_HEX3
          info->send_j++;
#endif
          hexminer3_create_task (reset_work, ht, tmpwork);
          mutex_lock (&info->lock);
          free_work (info->hexworks[write_pos]);
          info->hexworks[write_pos] = tmpwork;
          mutex_unlock (&info->lock);

          if (work->drv_rolllimit)
            {
              work->drv_rolllimit--;
#ifdef DBG_HEX3
              info->roled++;
#endif
            }
          else
            {
              free_work (work);
              work = NULL;
            }

#ifdef DBG_HEX3
          if (reset_work)
            applog (LOG_ERR, "HEX3 %i  Reset Work Task!",
                    hexminer3->device_id);
#endif
          ret = hexminer3_send_task (ht, hexminer3);
          write_pos++;
          send_jobs++;
          if (ret == HEXMINER3_TASK_SIZE && reset_work)
            {
              reset_work = false;
              send_jobs -= 4;
            }
        }
      if (!reset_work)
        cgsem_mswait (&info->wsem, info->wsem_timing);
    }
  if (work)
    free_work (work);
  free (ht);
  free (htc);
  pthread_exit (NULL);
}

static struct cgpu_info *
hexminer3_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int miner_count, asic_count, frequency;
  int this_option_offset = ++option_offset;
  struct hexminer3_info *info;
  struct cgpu_info *hexminer3;

  bool configured;
  int i = 0;

  hexminer3 = usb_alloc_cgpu (&hexminer3_drv, HEX3_MINER_THREADS);
  if (!usb_init (hexminer3, dev, found))
    {
      usb_uninit (hexminer3);
      return NULL;
    }
  hexminer3->device_data = calloc (sizeof (struct hexminer3_info), 1);

  if (unlikely (!(hexminer3->device_data)))
    {
      hexminer3->device_data = NULL;
      usb_uninit (hexminer3);
      return NULL;
    }
  configured =
    libhex3_get_options (this_option_offset, &asic_count, &frequency);
  if (opt_hexminer3_core_voltage < HEX3_MIN_COREMV
      || opt_hexminer3_core_voltage > HEX3_MAX_COREMV)
    {

      applog
        (LOG_ERR,
         "Invalid hexminer3-voltage %d must be %dmV - %dmV",
         opt_hexminer3_core_voltage, HEX3_MIN_COREMV, HEX3_MAX_COREMV);
      free (hexminer3->device_data);
      hexminer3->device_data = NULL;
      usb_uninit (hexminer3);
      return NULL;
    }
  info = hexminer3->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINER3_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminer3->device_data);
      hexminer3->device_data = NULL;
      usb_uninit (hexminer3);
      return NULL;
    }

  info->wr = (struct work3_result *) malloc (sizeof (struct work3_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_results3));
  info->readbuf = calloc (HEX3_HASH_BUF_SIZE, sizeof (unsigned char));


  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->miner_count = HEX3_DEFAULT_MINER_NUM;
  info->asic_count = HEX3_DEFAULT_ASIC_NUM;
  info->frequency = HEX3_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEX3_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminer3_core_voltage;
  info->chip_mask = opt_hexminer3_chip_mask;
  info->buf_empty_space = 63;

  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }

  info->wsem_timing =
    (int) (0x80000000ll / (info->asic_count * info->frequency) /
           1000 * 2 * 0.93);

  if (!add_cgpu (hexminer3))
    {
      free (info->hexworks);
      free (hexminer3->device_data);
      hexminer3->device_data = NULL;
      hexminer3 = usb_free_cgpu (hexminer3);
      usb_uninit (hexminer3);
      return NULL;
    }

  while (i < HEXMINER3_ARRAY_SIZE)
    info->hexworks[i++] = calloc (1, sizeof (struct work));

  i = 0;
  info->power_checked = time (NULL);
  while (i < HEX3_DEFAULT_ASIC_NUM)
    {
      info->last_chip_valid_work[i++] = time (NULL);
    }
  libhex3_generatenrange_new ((unsigned char *) &info->nonces_range,
                              info->asic_count);
  return hexminer3;
}

static void
hexminer3_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminer3_drv, hexminer3_detect_one);
}

static void
do_hexminer3_close (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;
  int i = 0;
  cgsleep_ms (300);

  pthread_join (info->write_thr, NULL);
#ifdef DBG_HEX3
  pthread_join (info->dbg_thr, NULL);
#endif

  pthread_mutex_destroy (&info->lock);
#ifdef PWR_HEX3
  pthread_mutex_destroy (&info->power);
#endif
  cgsem_destroy (&info->wsem);


  while (i < HEXMINER3_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->readbuf);
  free (info->array_nonce_cache);
  free (info->wr);
  //usb_uninit(hexminer3);
  //Hotplug fucks on full mem free :) 
  //free (hexminer3->device_data);
  //hexminer3->device_data = NULL;
  //thr->cgpu = usb_free_cgpu(hexminer3);

}

static void
hexminer3_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;

  cgsem_post (&info->wsem);

  do_hexminer3_close (thr);

  usb_nodev (hexminer3);
}

#ifdef DBG_HEX3
static void *
hexminer3_get_stats (void *userdata)
{
  struct cgpu_info *hexminer3 = (struct cgpu_info *) userdata;
  struct hexminer3_info *info = hexminer3->device_data;
  char threadname[24];
  snprintf (threadname, 24, "hex3_dbg/%d", hexminer3->device_id);
  RenameThread (threadname);
  while (!libhex3_usb_dead (hexminer3))
    {

      cgsleep_ms (30 * 1000);

      applog (LOG_ERR,
              "HEX3 %i was_64 %i, was_above_60 %i was_zero %i, was_below_5 %i",
              hexminer3->device_id, info->buf_empty_was_64,
              info->buf_empty_was_above_60, info->buf_empty_was_zero,
              info->buf_empty_was_below_5);

      applog (LOG_ERR,
              "HEX3 %i PIC received jobs %i, PIC sent nonces %i",
              hexminer3->device_id, info->word1, info->word2);

      applog (LOG_ERR,
              "HEX3 %i CGM sent jobs %i, CGM received nonces %i",
              hexminer3->device_id, info->send_j, info->r_j);


      if (info->buf_empty_was_above_60 > 0)
        {
          mutex_lock (&info->lock);
          info->buf_empty_was_64 = 0;
          info->buf_empty_was_above_60 = 0;
          mutex_unlock (&info->lock);
        }

      if (info->buf_empty_was_below_5 > 0)
        {
          mutex_lock (&info->lock);
          info->buf_empty_was_below_5 = 0;
          info->buf_empty_was_zero = 0;
          mutex_unlock (&info->lock);
        }
    }
  pthread_exit (NULL);
}
#endif

static bool
hexminer3_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;

  info->thr = thr;

  mutex_init (&info->lock);
#ifdef PWR_HEX3
  mutex_init (&info->power);
#endif
  cgsem_init (&info->wsem);

  if (pthread_create
      (&info->write_thr, NULL, hexminer3_send_tasks, (void *) hexminer3))
    quit (1, "Failed to create hexminer3 write_thr");

#ifdef DBG_HEX3
  if (pthread_create
      (&info->dbg_thr, NULL, hexminer3_get_stats, (void *) hexminer3))
    quit (1, "Failed to create hexminer3 dbg_thr");
#endif


  return true;
}

static int64_t
hexminer3_scanhash (struct thr_info *thr)
{
  struct cgpu_info *hexminer3 = thr->cgpu;
  struct hexminer3_info *info = hexminer3->device_data;

  uint32_t nonce;
  double found;
  double hash_count = 0;
  int i = 0, lastchippos;
  int ret_r = 0;
  int64_t rethash_count = 0;

  if (libhex3_usb_dead (hexminer3))
    return -1;

  if (info->hash_write_pos + HEX3_USB_R_SIZE >= HEX3_HASH_BUF_SIZE)
    {
      info->hash_write_pos = info->hash_write_pos - info->hash_read_pos;
      memcpy (info->readbuf, info->readbuf + info->hash_read_pos,
              info->hash_write_pos);
      info->hash_read_pos = 0;
    }
  if (info->hash_write_pos - info->hash_read_pos > 7)
    {
    again:
      ret_r =
        libhex3_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                             &info->hash_write_pos);
      if (ret_r > HEX3_BUF_DATA)
        goto out;
#ifdef DBG_HEX3
      if (info->wr->address == 0x300C)
        {
          info->word1 = htole32 (info->wr->word1);
          info->word2 = htole32 (info->wr->word2);
          goto out;
        }
#endif
      info->buf_empty_space = info->wr->buf_empty_space;

#ifdef DBG_HEX3
      if (info->wr->buf_empty_space > 60)
        {
          mutex_lock (&info->lock);
          if (info->wr->buf_empty_space == 64)
            info->buf_empty_was_64++;
          info->buf_empty_was_above_60++;
          mutex_unlock (&info->lock);
        }
      if (info->wr->buf_empty_space < 5)
        {
          mutex_lock (&info->lock);
          info->buf_empty_was_below_5++;
          if (info->wr->buf_empty_space == 0)
            info->buf_empty_was_zero++;
          mutex_unlock (&info->lock);
        }
#endif


      if (info->wr->datalength == 1)
        goto done;
#ifdef DBG_HEX3
      info->r_j++;
#endif
      if (info->wr->lastnonceid > HEXMINER3_ARRAY_SIZE_REAL)
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

      if (libhex3_cachenonce
          (&info->array_nonce_cache[lastchippos], info->wr->lastnonce))
        {

          found = hexminer3_predecode_nonce (hexminer3, thr, nonce,
                                             info->wr->lastnonceid);

          if (found > 0)
            {
#ifdef PWR_HEX3
              mutex_lock (&info->power);

              info->last_chip_valid_work[(uint8_t) lastchippos] = time (NULL);
              mutex_unlock (&info->power);
#endif
              if (hash_count == 0)
                libhex3_getvoltage (htole16 (info->wr->lastvoltage),
                                    &info->pic_voltage_readings);

              hash_count += found;
              info->matching_work[(uint8_t) lastchippos]++;
            }
          else
            {
              inc_hw_errors (thr);
            }
        }
      else
        {
          info->dupe[(uint8_t) lastchippos]++;
        }
    out:
      if (ret_r == HEX3_BUF_ERR)
        {
          info->usb_r_errors++;
        }
    done:
      if (info->hash_write_pos - info->hash_read_pos >= HEX3_MAX_WORK_SIZE)
        goto again;
    }

  ret_r =
    libhex3_readHashData (hexminer3, info->readbuf, &info->hash_write_pos,
                          HEXMINER3_BULK_READ_TIMEOUT, true);

  rethash_count = (0xffffffffull * (int64_t) hash_count);

  if (libhex3_usb_dead (hexminer3))
    return -1;

  return rethash_count;
}

#ifdef DBG_HEX3
static void
zero_hexminer3_stats (struct cgpu_info *hexminer3)
{
  if (!hexminer3->device_data)
    return;
  struct hexminer3_info *info = hexminer3->device_data;
  int i;
  for (i = 0; i < info->asic_count; i++)
    {
      info->matching_work[i] = 0;
      info->dupe[i] = 0;

    }

}
#endif
static void
get_hexminer3_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminer3)
{
  if (!hexminer3->device_data)
    return;
  struct hexminer3_info *info = hexminer3->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

static struct api_data *
hexminer3_api_stats (struct cgpu_info *cgpu)
{

  struct api_data *root = NULL;
  struct hexminer3_info *info = cgpu->device_data;
  if (!info)
    return NULL;
  uint64_t dh64, dr64;

#ifdef DBG_HEX3
  uint32_t du32, no32;
#endif
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
#ifdef DBG_HEX3
  du32 = 0;
  no32 = 0;
#endif
  for (i = 0; i < info->asic_count; i++)
    {
      char mcw[24];
      sprintf (mcw, "Chip%d Nonces", i + 1);
      root = api_add_int (root, mcw, &(info->matching_work[i]), true);
      sprintf (mcw, "Chip%d Dupes", i + 1);
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
#ifdef DBG_HEX3
      no32 += info->matching_work[i];
      du32 += info->dupe[i];
#endif
    }
#ifdef DBG_HEX3
  double dpr = (double) (du32) / (double) (no32);
  root = api_add_percent (root, "Dupes%", &dpr, true);
#endif
  return root;
}

struct device_drv hexminer3_drv = {
  .drv_id = DRIVER_hexminer3,
  .dname = "hexminer3",
  .name = "HEX3",
  .drv_detect = hexminer3_detect,
  .thread_init = hexminer3_thread_init,
  .hash_work = hash_queued_work,
  .scanwork = hexminer3_scanhash,
  .flush_work = hexminer3_flush_work,
  .get_api_stats = hexminer3_api_stats,
  .get_statline_before = get_hexminer3_statline_before,
#ifdef DBG_HEX3
  .zero_stats = zero_hexminer3_stats,
#endif
  .thread_shutdown = hexminer3_shutdown,
};
