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
#include "driver-hexmineru.h"
#include "util.h"
//static int option_offset = -1;
extern struct work *copy_work_noffset_fast_no_id(struct work *base_work, int noffset);
struct device_drv hexmineru_drv;
#include "libhexu.c"
#include "lib_mcp2210_hexu.c"
extern unsigned int work_block;


/*
    We use a replacement algorithm to only remove references to work done from the buffer when we need the extra space
    for new work. Thanks to Avalon code with some mods
 */


static inline void
hexmineru_create_task (uint32_t * vec, struct hexmineru_task *ht,
                       struct work *work)
{
  libhexu_work_to_bitfury_payload (ht, work);
  libhexu_bitfury_payload_to_atrvec (vec, ht);
}


static struct cgpu_info *
hexmineru_detect_one (libusb_device * dev, struct usb_find_devices *found)
{

  struct hexmineru_info *info;
  struct cgpu_info *hexmineru;
  unsigned char buf[1024];
  unsigned char trash[1024];
  uint64_t freq;
  const uint8_t *osc6 = (unsigned char *) &freq;
  int i = 0;

  hexmineru = usb_alloc_cgpu (&hexmineru_drv, HEXU_MINER_THREADS);
  if (!usb_init (hexmineru, dev, found))
    {
      usb_uninit (hexmineru);
      return NULL;
    }
  hexmineru->device_data = calloc (sizeof (struct hexmineru_info), 1);

  if (unlikely (!(hexmineru->device_data)))
    {
      hexmineru->device_data = NULL;
      usb_uninit (hexmineru);
      return NULL;
    }

  info = hexmineru->device_data;
  info->hexworks = calloc (sizeof (struct work *), HEXMINERU_ARRAY_SIZE);
  if (unlikely (!(info->hexworks)))
    {
      free (hexmineru->device_data);
      hexmineru->device_data = NULL;
      usb_uninit (hexmineru);
      return NULL;
    }
  info->spipos = 0;
  info->job_switch = true;
  info->shut_read = false;
  info->shut_write = false;
  info->work = NULL;
  info->c_job_id = 0;
  info->l_job_id = 0;
  info->array_nonce_cache = calloc (1, sizeof (struct chip_resultsu));
  //bzero (info->array_nonce_cache, 1 * sizeof (struct chip_resultsu));
  info->buf_switch = 0xffffffff;
  info->spipos = 0;
  info->read_pos = 0;
  info->frequency = (uint8_t) HEXU_DEFAULT_FREQUENCY;
  if (opt_hexmineru_options != NULL
      && atoi (opt_hexmineru_options) > HEXU_MIN_FREQUENCY
      && atoi (opt_hexmineru_options) < HEXU_MAX_FREQUENCY)
    {
      info->frequency = (uint8_t) atoi (opt_hexmineru_options);
    }
  if (!add_cgpu (hexmineru))
    goto out;

 // libhexu_reset (hexmineru);
  if (!libhexu_mcp2210_get_configs (hexmineru))
    goto out;
  if (!hex_nanofury_checkport (hexmineru))
    goto out;

  freq = htole64 ((1ULL << info->frequency) - 1ULL);

  libhexu_spi_emit_break (&info->spipos, (unsigned char *) buf);
  libhexu_spi_emit_data (&info->spipos, (unsigned char *) buf, 0x6000, osc6, 8);      // Program internal on-die slow oscillator frequency 
  libhexu_spi_send_conf (&info->spipos, (unsigned char *) buf);
  libhexu_spi_send_init (&info->spipos, (unsigned char *) buf);
  libhexu_nanofury_spi_reset (hexmineru);
  if (!libhexu_nanofury_spi_txrx
      (hexmineru, &info->spipos, (unsigned char *) buf, (unsigned char *) trash,
       true))
    goto out;
    i = 0;
      while (i < HEXMINERU_ARRAY_SIZE)
    {
      info->hexworks[i] = calloc (1, sizeof (struct work));
      info->hexworks[i]->pool = NULL;
      i++;
    }  
    info->spipos = 0;
  
  libhexu_spi_emit_break (&info->spipos, (unsigned char *) info->wr_spi);
  
  return hexmineru;

out:
  free (info->hexworks);
  free (hexmineru->device_data);
  hexmineru->device_data = NULL;
  hexmineru = usb_free_cgpu (hexmineru);
  usb_uninit (hexmineru);
  return NULL;
}

static void
hexmineru_detect (bool __maybe_unused hotplug)
{
  usb_detect (&hexmineru_drv, hexmineru_detect_one);
}

static void
do_hexmineru_close (struct thr_info *thr)
{
  struct cgpu_info *hexmineru = thr->cgpu;
  struct hexmineru_info *info = hexmineru->device_data;
  int i = 0;

	cgsleep_ms (300);
  cgsem_destroy (&info->qsem);
  
  while (i < HEXMINERU_ARRAY_SIZE)
    {
      free_work (info->hexworks[i]);
      i++;
    }
  free (info->hexworks);
  free (info->array_nonce_cache);
 // free (info->wr);
  //usb_uninit(hexmineru);
  //Hotplug fucks on full mem free :) 
  //free (hexmineru->device_data);
  //hexmineru->device_data = NULL;
  //thr->cgpu = usb_free_cgpu(hexmineru);

}

static void
hexmineru_shutdown (struct thr_info *thr)
{
  struct cgpu_info *hexmineru = thr->cgpu;
  struct hexmineru_info *info = hexmineru->device_data;

 cgsem_post (&info->qsem);

  
  do_hexmineru_close (thr);
  
  usb_nodev(hexmineru);
}

static bool
hexmineru_thread_init (struct thr_info *thr)
{
	  struct cgpu_info *hexmineru = thr->cgpu;
  struct hexmineru_info *info = hexmineru->device_data;
 cgsem_init (&info->qsem);
  return true;
}

static int64_t
hexmineru_scanhash (struct thr_info *thr)
{
  struct cgpu_info *hexmineru = thr->cgpu;
  struct hexmineru_info *info = hexmineru->device_data;
  bool reset_work = false;
  int64_t hash_count = 0;
  if (libhexu_usb_dead (hexmineru))
    {
      return -1;
    }
    
  
  struct hexmineru_task ht;
  uint32_t nonce;
 

  int i, found, c_found;

again:
    if (!info->work)
            {
              info->roll = 0;
              info->work = get_work (thr, thr->id);
              info->work->ping = 1;
							if(work_block != info->work_block) {
  							reset_work = true;
  							info->work_block = work_block;
  						}
            }
      if (stale_work (info->work, false))
        {
          free_work (info->work);
          info->work = NULL;
          goto again;
        }       
            
      if (info->job_switch || reset_work)
        {
          info->spipos = 1;
          info->l_job_id = info->c_job_id;
          info->c_job_id = info->read_pos;
          free_work (info->hexworks[info->read_pos]);
          info->hexworks[info->read_pos] = copy_work_noffset_fast_no_id (info->work, info->roll++);
          hexmineru_create_task (&info->atrvecs[info->read_pos][0], &ht, info->hexworks[info->read_pos]);
          memcpy (&info->atrvec[0], &info->atrvecs[info->read_pos][0], 76);
          info->read_pos++;
          if (info->read_pos >= HEXMINERU_ARRAY_SIZE_REAL)
           info->read_pos = 0;
          libhexu_spi_emit_data (&info->spipos, (unsigned char *) info->wr_spi, 0x3000,
                                 &info->atrvec[0], 76);
                                 
             if (info->work->drv_rolllimit)
                {
                  info->work->drv_rolllimit--;
                }
              else
                {
                  free_work (info->work);
                  info->work = NULL;
                }
        }
      if (libhexu_nanofury_spi_txrx
          (hexmineru, &info->spipos, (unsigned char *) info->wr_spi,
           (unsigned char *) info->read_spi, false))
        {
          if (info->read_spi[17] == info->buf_switch)
            {
              info->job_switch = false;
            }
          else
            {
              info->job_switch = true;
              info->buf_switch = info->read_spi[17];
            }

          //Skip first and last - only crap there 

          found = 0;
          
          for (i = 1; i < 17; i++)
            {
              if (libhexu_cachenonce (&info->array_nonce_cache[0], info->read_spi[i]))
                {
          
#if defined(__BIG_ENDIAN__) || defined(MIPSEB)
                  nonce = libhexu_decnonce (htole32 (info->read_spi[i]));
#else
                  nonce = libhexu_decnonce (info->read_spi[i]);
#endif
                  c_found = 0;
                  c_found +=
                    hexmineru_predecode_nonce (hexmineru, thr, nonce,
                                               info->c_job_id);
                  if (c_found == 0 && info->c_job_id != info->l_job_id)
                    c_found +=
                      hexmineru_predecode_nonce (hexmineru, thr, nonce,
                                                 info->l_job_id);

                  found += c_found;
                }
              else
                {
                  
                  info->dupe[0]++;
                  
                }
            }

          if (found > 0)
            {
              
              hash_count += found;
              
            }
          else
            {
              //Due to implementation there is no way for now to count them. 
              //The number is inaccurate and too big!

              // if (n_cache) inc_hw_errors (thr);
            }

        }
      else
        {
          //applog (LOG_ERR, "WTF??");
          //libhexu_reset (hexmineru);
          //libhexu_nanofury_spi_reset (hexmineru);
          //info->dev_reset_count++;
          info->shut_read = true;
          
          

        }
      info->spipos = 80;
       hash_count = (int64_t) (0xffffffffull * hash_count);
        if (libhexu_usb_dead (hexmineru))
      return -1;
    
   cgsem_mswait (&info->qsem, 40 );
  return hash_count;
}

static void
get_hexmineru_statline_before (char *buf, size_t bufsiz,
                               struct cgpu_info *hexmineru)
{
	 if (!hexmineru->device_data)
		return;
  struct hexmineru_info *info = hexmineru->device_data;
  tailsprintf (buf, bufsiz, "%3d %4d/%4dmV", info->frequency, 0, 0);
}

static struct api_data *
hexmineru_api_stats (struct cgpu_info *cgpu)
{

  struct api_data *root = NULL;
  struct hexmineru_info *info = cgpu->device_data;
  if (!info)
		return NULL;
  uint64_t dh64, dr64;
  double dev_runtime;
  struct timeval now;
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
  root = api_add_int (root, "Reset Count", &(info->dev_reset_count), true);
  root =
    api_add_time (root, "Last Share Time", &(cgpu->last_share_pool_time),
                  true);
  root = api_add_uint8 (root, "Frequency", &(info->frequency), true);
  char mcw[24];
  sprintf (mcw, "Chip1 Dupes");
  root = api_add_int (root, mcw, &(info->dupe[0]), true);

  return root;
}


static void
hexmineru_flush_work (struct cgpu_info *hexmineru)
{
  struct hexmineru_info *info = hexmineru->device_data;



  cgsem_post (&info->qsem);

}
struct device_drv hexmineru_drv = {
  .drv_id = DRIVER_hexmineru,
  .dname = "hexmineru",
  .name = "HEXu",
  .drv_detect = hexmineru_detect,
  .thread_init = hexmineru_thread_init,
  .hash_work = hash_queued_work,
  .scanwork = hexmineru_scanhash,
  .flush_work = hexmineru_flush_work,
  .get_api_stats = hexmineru_api_stats,
  .get_statline_before = get_hexmineru_statline_before,
  .thread_shutdown = hexmineru_shutdown,
};
