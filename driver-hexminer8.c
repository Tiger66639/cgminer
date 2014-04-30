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
#include "driver-hexminer8.h"

#include "util.h"
extern unsigned int work_block;
extern struct work *copy_work_noffset_fast_no_id (struct work *base_work,
                                                  int noffset);
static int option_offset = -1;
struct device_drv hexminer8_drv;
extern bool no_work;
int opt_hexminer8_chip_mask = 0xFF;
int opt_hexminer8_set_config_diff_to_one = 1;
int opt_hexminer8_core_voltage = HEX8_DEFAULT_CORE_VOLTAGE;

#include "libhex8.c"

/*
    We use a replacement algorithm to only remove references to work done from the buffer when we need the extra space
    for new work. Thanks to Avalon code with some mods
 */

static void
hexminer8_flush_work (struct cgpu_info *hexminer8)
{
  struct hexminer8_info *info = hexminer8->device_data;

  cgsem_post (&info->wsem);
#ifdef DBG_HEX8
  applog (LOG_ERR, "HEX8 %i  hexminer8_flush_work !", hexminer8->device_id);
#endif

}

static int
hexminer8_send_task (struct hexminer8_task *ht, struct cgpu_info *hexminer8)
{
  int ret = 0;
  size_t nr_len = HEXMINER8_TASK_SIZE;
  struct hexminer8_info *info;
  info = hexminer8->device_data;

  libhex8_csum (&ht->startbyte, &ht->csum, &ht->csum);

  ret = libhex8_sendHashData (hexminer8, &ht->startbyte, nr_len);

  if (ret != nr_len)
    {
      libhex8_reset (hexminer8);
      info->usb_w_errors++;
      return -1;
    }

  return ret;
}

static inline void
hexminer8_create_task (bool reset_work, struct hexminer8_task *ht,
                       struct work *work, bool diff1,
                       uint32_t * asic_difficulty, double *cached_diff)
{
  if (reset_work)
    {
      ht->status = htole16 ((uint16_t) HEX8_STAT_NEW_WORK_CLEAR_OLD);
    }
  else
    {
      ht->status = htole16 ((uint16_t) HEX8_STAT_NEW_WORK);
    }
  memcpy (ht->midstate, work->midstate, 32);
  memcpy (ht->merkle, work->data + 64, 12);
  ht->id = htole16 ((uint16_t) work->subid);
  //Try to save some CPU cycles not fancy primary/backup scenarios... 
  if (!diff1)
    {
      if (work->ping)
        {
          ht->difficulty = htole32 (0xFFFF001D);
          return;
        }
      if (*cached_diff != work->work_difficulty)
        {
          *cached_diff = work->work_difficulty;
#if defined(__BIG_ENDIAN__) || defined(MIPSEB)
          *asic_difficulty = libhex8_get_target (work->work_difficulty);
#else
          *asic_difficulty =
            be32toh (libhex8_get_target (work->work_difficulty));
#endif
        }
      ht->difficulty = *asic_difficulty;
    }

}

static inline void
hexminer8_init_task_c (struct hexminer8_config_task *htc,
                       struct hexminer8_info *info)
{
  htc->startbyte = 0x53;
  htc->datalength =
    (uint8_t) ((sizeof (struct hexminer8_config_task) - 6) / 2);
  htc->command = 0x57;
  htc->address = htole16 (0x30C0);
  htc->hashclock = htole16 ((uint16_t) info->frequency);
  libhex8_setvoltage (info->core_voltage, &htc->refvoltage);
  htc->difficulty = htole32 (0xFFFF001D);
  htc->chip_mask = (uint8_t) info->chip_mask;

  libhex8_csum (&htc->startbyte, &htc->csum, &htc->csum);
}

static inline void
hexminer8_init_task (struct hexminer8_task *ht, struct hexminer8_info *info)
{
  
  ht->startbyte = 0x53;
  ht->datalength = (uint8_t) ((HEXMINER8_TASK_SIZE - 6) / 2);
  ht->command = 0x57;
  ht->address = htole16 (0x3080);
  ht->difficulty = htole32 (0xFFFF001D);
}

static bool
need_reset (struct cgpu_info *hexminer8)
{

  struct hexminer8_info *info = hexminer8->device_data;

  time_t now = time (NULL);
  bool ret = false;
  int i = 0;
  int secs = 20;

  if (!info->diff1)
    secs = 60;
  while (i < HEX8_DEFAULT_ASIC_NUM)
    {
      //Up to diff 1024 900 secs double
      if (info->engines[i]
          && (info->last_chip_valid_work[i] +
              (int) (secs * 32 / info->engines[i]) < now))
        {
          //applog(LOG_ERR, "HEX8 %i Chip[%i] last valid work %i secs ago", hexminer8->device_id, i + 1, (int)(now-info->last_chip_valid_work[i]));
          ret = true;
          break;
        }
      i++;
    }

  if (ret || no_work)
    {
      ret = !no_work;
      i = 0;
      while (i < HEX8_DEFAULT_ASIC_NUM)
        info->last_chip_valid_work[i++] = now;
    }

  return ret;
}

static void *
hexminer8_send_tasks (void *userdata)
{
  struct cgpu_info *hexminer8 = (struct cgpu_info *) userdata;
  struct hexminer8_info *info = hexminer8->device_data;
  struct hexminer8_task *ht;
  struct thr_info *thr = info->thr;
  struct hexminer8_config_task *htc;
  struct work *work = NULL;
  struct work *tmpwork = NULL;
  char threadname[24];
  int write_pos = 0;
  int jobs_to_send = 15;
  int ping_period =
    (int) (1000 / info->wsem_timing * 60 / info->asic_count / 14);
  int ping_counter = 0;
  int random_job = 0;
  int send_jobs, roll, ret;
  struct timeval tm;
  unsigned int work_block_local;
  double cached_diff = -1;
  uint32_t asic_difficulty;
  bool reset_work = true;
  bool power = false;
  snprintf (threadname, 24, "hex8_send/%d", hexminer8->device_id);
  RenameThread (threadname);
  htc = calloc (sizeof (struct hexminer8_config_task), 1);

  hexminer8_init_task_c (htc, info);

  ret =
    libhex8_sendHashData (hexminer8, &htc->startbyte,
                          sizeof (struct hexminer8_config_task));

  if (ret != sizeof (struct hexminer8_config_task))
    applog (LOG_ERR, "HEX8 %i Send config failed", hexminer8->device_id);
  ht = calloc (sizeof (struct hexminer8_task), 1);
  
  hexminer8_init_task (ht, info);

  while (!libhex8_usb_dead (hexminer8))
    {
      if (time (NULL) - info->power_checked > 30)
        {

          info->power_checked = time (NULL);
          mutex_lock (&info->power);
          power = need_reset (hexminer8);
          mutex_unlock (&info->power);
          if (power)
            {
              libhex8_set_word (hexminer8, 0x3080 + HEXMINER8_TASK_SIZE - 8,
                                0x0004);
              reset_work = true;
              cgsleep_ms (200);
            }
        }
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
              work->ping = info->diff1;
              if (work_block_local != work_block)
                {
                  reset_work = true;
                  work_block_local = work_block;
                }
#ifdef DBG_HEX8
              info->totworks++;
#endif
            }

          if (stale_work (work, false))
            {
              free_work (work);
              work = NULL;
              goto again;
            }

          if (write_pos >= HEXMINER8_ARRAY_SIZE_REAL || reset_work)
            write_pos = 0;

          work->subid = write_pos;
          tmpwork = copy_work_noffset_fast_no_id (work, roll++);
          if (ping_counter == random_job)
            tmpwork->ping = 1;
          hexminer8_create_task (reset_work, ht,
                                 tmpwork,
                                 info->diff1, &asic_difficulty, &cached_diff);
          mutex_lock (&info->lock);
          free_work (info->hexworks[write_pos]);
          info->hexworks[write_pos] = tmpwork;
          mutex_unlock (&info->lock);

          if (!info->diff1)
            {
              ping_counter++;
              if (ping_counter == ping_period)
                {
                  ping_counter = 0;
                  gettimeofday (&tm, NULL);
                  srandom (tm.tv_sec + tm.tv_usec * 1000000ul);
                  random_job = rand () % ping_period;
                }
            }
          if (work->drv_rolllimit)
            {
              work->drv_rolllimit--;
#ifdef DBG_HEX8
              info->roled++;
#endif
            }
          else
            {
              free_work (work);
              work = NULL;
            }

#ifdef DBG_HEX8
          if (reset_work)
            applog (LOG_ERR, "HEX8 %i  Reset Work Task!",
                    hexminer8->device_id);
#endif


          ret = hexminer8_send_task (ht, hexminer8);
          write_pos++;
          send_jobs++;
          if (ret == HEXMINER8_TASK_SIZE && reset_work)
            {
              reset_work = false;
              send_jobs-=4;
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
hexminer8_detect_one (libusb_device * dev, struct usb_find_devices *found)
{
  int miner_count, asic_count, frequency;
  int this_option_offset = ++option_offset;
  struct hexminer8_info *info;
  struct cgpu_info *hexminer8;

  bool configured;
  int i = 0;

  hexminer8 = usb_alloc_cgpu (&hexminer8_drv, HEX8_MINER_THREADS);
  if (!usb_init (hexminer8, dev, found))
    {
      usb_uninit (hexminer8);
      return NULL;
    }
  hexminer8->device_data = calloc (sizeof (struct hexminer8_info), 1);

  if (unlikely (!(hexminer8->device_data)))
    {
      hexminer8->device_data = NULL;
      usb_uninit (hexminer8);
      return NULL;
    }
  configured =
    libhex8_get_options (this_option_offset, &asic_count, &frequency);
  if (opt_hexminer8_core_voltage < HEX8_MIN_COREMV
      || opt_hexminer8_core_voltage > HEX8_MAX_COREMV)
    {

      applog
        (LOG_ERR,
         "Invalid hexminer8-voltage %d must be %dmV - %dmV",
         opt_hexminer8_core_voltage, HEX8_MIN_COREMV, HEX8_MAX_COREMV);
      free (hexminer8->device_data);
      hexminer8->device_data = NULL;
      usb_uninit (hexminer8);
      return NULL;
    }
  info = hexminer8->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINER8_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexminer8->device_data);
      hexminer8->device_data = NULL;
      usb_uninit (hexminer8);
      return NULL;
    }

  info->wr = (struct work8_result *) malloc (sizeof (struct work8_result));
  info->array_nonce_cache = calloc (16, sizeof (struct chip_results8));
  info->readbuf = calloc (HEX8_HASH_BUF_SIZE, sizeof (unsigned char));
  
  info->hash_read_pos = 0;
  info->hash_write_pos = 0;
  info->shut_read = false;
  info->shut_write = false;
  info->shut_reset = false;
  info->miner_count = HEX8_DEFAULT_MINER_NUM;
  info->asic_count = HEX8_DEFAULT_ASIC_NUM;
  info->frequency = HEX8_DEFAULT_FREQUENCY;
  info->pic_voltage_readings = HEX8_DEFAULT_CORE_VOLTAGE;
  info->core_voltage = opt_hexminer8_core_voltage;
  info->chip_mask = opt_hexminer8_chip_mask;
  info->diff1 = (bool) opt_hexminer8_set_config_diff_to_one;
  info->buf_empty_space = 63;
  //info->dead_chip = false;

  if (configured)
    {
      info->asic_count = asic_count;
      info->frequency = frequency;
    }

  info->wsem_timing =
    (int) (0x80000000ll / (info->asic_count * info->frequency * 4 * 32) /
           1000 * 2);

  if (!add_cgpu (hexminer8))
    {
      free (info->hexworks);
      free (hexminer8->device_data);
      hexminer8->device_data = NULL;
      hexminer8 = usb_free_cgpu (hexminer8);
      usb_uninit (hexminer8);
      return NULL;
    }

  while (i < HEXMINER8_ARRAY_SIZE)
    info->hexworks[i++] = calloc (1, sizeof (struct work));

  i = 0;
  info->power_checked = time (NULL);
  while (i < HEX8_DEFAULT_ASIC_NUM)
    {
      info->engines[i] = 0;
      info->last_chip_valid_work[i++] = time (NULL);
    }

  return hexminer8;
}

static void
hexminer8_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexminer8_drv, hexminer8_detect_one);
}

static void
do_hexminer8_close (struct thr_info *thr)
{
  struct cgpu_info *hexminer8 = thr->cgpu;
  struct hexminer8_info *info = hexminer8->device_data;
  int i = 0;
  cgsleep_ms (300);

  pthread_join (info->write_thr, NULL);
#ifdef DBG_HEX8
  pthread_join (info->dbg_thr, NULL);
#endif

  pthread_mutex_destroy (&info->lock);
  pthread_mutex_destroy (&info->power);
  cgsem_destroy (&info->wsem);


  while (i < HEXMINER8_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->readbuf);
  free (info->array_nonce_cache);
  free (info->wr);
  //usb_uninit(hexminer8);
  //Hotplug fucks on full mem free :) 
  //free (hexminer8->device_data);
  //hexminer8->device_data = NULL;
  //thr->cgpu = usb_free_cgpu(hexminer8);

}

static void
hexminer8_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexminer8 = thr->cgpu;
  struct hexminer8_info *info = hexminer8->device_data;

  cgsem_post (&info->wsem);

  do_hexminer8_close (thr);

  usb_nodev (hexminer8);
}

#ifdef DBG_HEX8
static void *
hexminer8_get_stats (void *userdata)
{
  struct cgpu_info *hexminer8 = (struct cgpu_info *) userdata;
  struct hexminer8_info *info = hexminer8->device_data;
  char threadname[24];
  snprintf (threadname, 24, "hex8_dbg/%d", hexminer8->device_id);
  RenameThread (threadname);
  while (!libhex8_usb_dead (hexminer8))
    {

      cgsleep_ms (30 * 1000);

      applog (LOG_ERR,
              "HEX8 %i was_64 %i, was_above_60 %i was_zero %i, was_below_5 %i",
              hexminer8->device_id, info->buf_empty_was_64,
              info->buf_empty_was_above_60, info->buf_empty_was_zero,
              info->buf_empty_was_below_5);

      applog (LOG_ERR,
              "HEX8 %i roled %i, getworks %i", hexminer8->device_id,
              info->roled, info->totworks);


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
hexminer8_thread_init (struct thr_info *thr)
{
  struct cgpu_info *hexminer8 = thr->cgpu;
  struct hexminer8_info *info = hexminer8->device_data;

  info->thr = thr;

  mutex_init (&info->lock);
  mutex_init (&info->power);
  cgsem_init (&info->wsem);

  if (pthread_create
      (&info->write_thr, NULL, hexminer8_send_tasks, (void *) hexminer8))
    quit (1, "Failed to create hexminer8 write_thr");

#ifdef DBG_HEX8
  if (pthread_create
      (&info->dbg_thr, NULL, hexminer8_get_stats, (void *) hexminer8))
    quit (1, "Failed to create hexminer8 dbg_thr");
#endif


  return true;
}

static int64_t
hexminer8_scanhash (struct thr_info *thr)
{
  struct cgpu_info *hexminer8 = thr->cgpu;
  struct hexminer8_info *info = hexminer8->device_data;

  uint32_t nonce;
  double found;
  double hash_count = 0;
  int i = 0;
  int ret_r = 0;
  int64_t rethash_count = 0;

  if (libhex8_usb_dead (hexminer8))
    return -1;

  if (info->hash_write_pos + HEX8_USB_R_SIZE >= HEX8_HASH_BUF_SIZE)
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
        libhex8_eatHashData (info->wr, info->readbuf, &info->hash_read_pos,
                             &info->hash_write_pos);
      if (ret_r > HEX8_BUF_DATA)
        goto out;

      info->buf_empty_space = info->wr->buf_empty_space;

#ifdef DBG_HEX8
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

      if (info->wr->lastnonceid > HEXMINER8_ARRAY_SIZE_REAL)
        info->wr->lastnonceid = 0;

      if (info->wr->lastchippos >= HEX8_DEFAULT_ASIC_NUM)
        info->wr->lastchippos = 7;

      if (libhex8_cachenonce
          (&info->array_nonce_cache[info->wr->lastchippos],
           info->wr->lastnonce))
        {
          nonce = htole32 (info->wr->lastnonce);

          found = hexminer8_predecode_nonce (hexminer8, thr, nonce,
                                             info->wr->lastnonceid,
                                             info->diff1);

          if (found > 0)
            {
              mutex_lock (&info->power);
              info->engines[(uint8_t) info->wr->lastchippos] =
                info->wr->good_engines;
              info->last_chip_valid_work[(uint8_t) info->wr->lastchippos] =
                time (NULL);
              mutex_unlock (&info->power);
              if (hash_count == 0)
                libhex8_getvoltage (htole16 (info->wr->lastvoltage),
                                    &info->pic_voltage_readings);

              hash_count += found;
              info->matching_work[info->wr->lastchippos]++;
            }
          else
            {
              inc_hw_errors_hex8 (thr, (int) found);
            }
        }
      else
        {
          info->dupe[info->wr->lastchippos]++;
        }
    out:
      if (ret_r == HEX8_BUF_ERR)
        {
          info->usb_r_errors++;
        }
    done:
      if (info->hash_write_pos - info->hash_read_pos >= HEX8_MAX_WORK_SIZE)
        goto again;
    }

  ret_r =
    libhex8_readHashData (hexminer8, info->readbuf, &info->hash_write_pos,
                          HEXMINER8_BULK_READ_TIMEOUT, true);

  rethash_count = (0xffffffffull * (int64_t) hash_count);

  if (libhex8_usb_dead (hexminer8))
    return -1;

  return rethash_count;
}

static void
get_hexminer8_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexminer8)
{
  if (!hexminer8->device_data)
    return;
  struct hexminer8_info *info = hexminer8->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency,
               info->core_voltage, info->pic_voltage_readings);
}

static struct api_data *
hexminer8_api_stats (struct cgpu_info *cgpu)
{

  struct api_data *root = NULL;
  struct hexminer8_info *info = cgpu->device_data;
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
      sprintf (mcw, "Chip%d Engines", i + 1);
      root = api_add_int (root, mcw, &(info->engines[i]), true);
      sprintf (mcw, "Chip%d Dupes", i + 1);
      root = api_add_int (root, mcw, &(info->dupe[i]), true);
    }

  return root;
}

struct device_drv hexminer8_drv = {
  .drv_id = DRIVER_hexminer8,
  .dname = "hexminer8",
  .name = "HEX8",
  .drv_detect = hexminer8_detect,
  .thread_init = hexminer8_thread_init,
  .hash_work = hash_queued_work,
  .scanwork = hexminer8_scanhash,
  .flush_work = hexminer8_flush_work,
  .get_api_stats = hexminer8_api_stats,
  .get_statline_before = get_hexminer8_statline_before,
  .thread_shutdown = hexminer8_shutdown,
};
