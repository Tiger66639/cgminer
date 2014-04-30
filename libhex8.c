//Thank you Zefir !!!!
static uint32_t
libhex8_get_target (double diff)
{
  unsigned nBits;
  int shift = 29;
  double ftarg = (double) 0x0000ffff / diff;
  while (ftarg < (double) 0x00008000)
    {
      shift--;
      ftarg *= 256.0;
    }
  while (ftarg >= (double) 0x00800000)
    {
      shift++;
      ftarg /= 256.0;
    }
  nBits = (int) ftarg + (shift << 24);
  
  return nBits;
}

//Once More - Thank you Zefir :)

static bool
libhex8_cachenonce (struct chip_results8 *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEX8_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEX8_NONCE_CASH_SIZE)
    return false;
  //Rotate
  if (nonce_cache->nonce_cache_write_pos == HEX8_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

char *
libhex8_set_config_voltage (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminer8-voltage";
  if (val1 < HEX8_MIN_COREMV || val1 > HEX8_MAX_COREMV)
    return "Invalid value passed to hexminer8-voltage";
  opt_hexminer8_core_voltage = val1;
  return NULL;
}

char *
libhex8_set_config_chip_mask (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminer8-chip-mask";
  opt_hexminer8_chip_mask = val1;
  return NULL;
}

char *
libhex8_set_config_diff_to_one (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminer8-set-diff-to-one";
  opt_hexminer8_set_config_diff_to_one = val1;
  return NULL;
}

static void
libhex8_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhex8_get_options (int this_option_offset, int *asic_count, int *frequency)
{
  char buf[BUFSIZ + 1];
  char *ptr, *comma, *colon, *colon2, *colon3, *colon4;
  bool timeout_default;
  size_t max;
  int i, tmp;
  if (opt_hexminer8_options == NULL)
    buf[0] = '\0';
  else
    {
      ptr = opt_hexminer8_options;
      for (i = 0; i < this_option_offset; i++)
        {
          comma = strchr (ptr, ',');
          if (comma == NULL)
            break;
          ptr = comma + 1;
        }
      comma = strchr (ptr, ',');
      if (comma == NULL)
        max = strlen (ptr);
      else
        max = comma - ptr;
      if (max > BUFSIZ)
        max = BUFSIZ;
      strncpy (buf, ptr, max);
      buf[max] = '\0';
    }
  if (!(*buf))
    return false;
  colon = strchr (buf, ':');
  if (colon)
    *(colon++) = '\0';
  tmp = atoi (buf);
  if (tmp > 0 && tmp <= HEX8_DEFAULT_ASIC_NUM)
    *asic_count = tmp;
  else
    {
      quit (1,
            "Invalid hexminer8-options for " "asic_count (%s) must be 1 ~ %d",
            buf, HEX8_DEFAULT_ASIC_NUM);
    }
  if (colon && *colon)
    {
      tmp = atoi (colon);
      if (tmp < HEX8_MIN_FREQUENCY || tmp > HEX8_MAX_FREQUENCY)
        {
          quit
            (1,
             "Invalid hexminer8-options for frequency (%s) must be %d <= frequency <= %d",
             colon, HEX8_MIN_FREQUENCY, HEX8_MAX_FREQUENCY);
        }
      *frequency = tmp;
    }
  return true;
}

static bool
libhex8_usb_dead (struct cgpu_info *hexminer8)
{
  struct cg_usb_device *usbdev;
  struct hexminer8_info *info = hexminer8->device_data;

  usbdev = hexminer8->usbdev;
  bool ret = (usbdev == NULL
              || usbdev->handle == NULL
              || hexminer8->shutdown
              || info->shut_read || info->shut_write || info->shut_reset
              || hexminer8->usbinfo.nodev || hexminer8->deven != DEV_ENABLED);
  if (ret)
    hexminer8->shutdown = true;

  return ret;
}


static int
libhex8_sendHashData (struct cgpu_info *hexminer8, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminer8_info *info = hexminer8->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminer8->usbdev;
  if (libhex8_usb_dead (hexminer8))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err = libusb_bulk_transfer
        (usbdev->handle,
         0x02,
         sendbuf + written,
         MIN (HEX8_USB_WR_SIZE, buf_len - written), &wrote,
         HEX8_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;

  return written;
}

static void
libhex8_reset (struct cgpu_info *hexminer8)
{

  struct hexminer8_info *info = hexminer8->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminer8->usbdev;
  if (libhex8_usb_dead (hexminer8))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;

  info->usb_reset_count++;
}

static int libhex8_readHashData
  (struct cgpu_info *hexminer8,
   unsigned char *hash, int *hash_write_pos, int timeout, bool read_once)
{
  struct hexminer8_info *info = hexminer8->device_data;
  struct cg_usb_device *usbdev;
  int read = 0, total = 0;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminer8->usbdev;
  if (libhex8_usb_dead (hexminer8))
    goto out;
  while (*hash_write_pos + HEX8_USB_R_SIZE < HEX8_HASH_BUF_SIZE
         && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                              HEX8_USB_R_SIZE, &read, timeout);
      if (err == LIBUSB_SUCCESS)
        {
          *hash_write_pos += read;
          total += read;
        }
      if (read_once)
        break;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    {
      info->shut_read = true;
      cgsem_post (&info->wsem);
    }

  return err;
}

static double
hexminer8_predecode_nonce (struct cgpu_info *hexminer8, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id, bool diff1)
{
  struct hexminer8_info *info = hexminer8->device_data;
  struct work *work_sub;
  mutex_lock (&info->lock);
  if (info->hexworks[work_id]->pool == NULL)
    {
      mutex_unlock (&info->lock);
      return 0;
    }

  work_sub = copy_work_noffset_fast_no_id (info->hexworks[work_id], 0);
  mutex_unlock (&info->lock);

  double diff = (diff1 || work_sub->ping ? 1 : work_sub->work_difficulty);

  if (test_nonce (work_sub, nonce))
    {
      //work_sub->device_diff = diff1;
      submit_tested_work_no_clone (thr, work_sub, diff1 || work_sub->ping);
      return diff;
    }
  free_work (work_sub);
  return -diff;
}

static void
libhex8_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 3300 / (float) ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhex8_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhex8_eatHashData (struct work8_result *wr, unsigned char *hash,
                     int *hash_read_pos, int *hash_write_pos)
{
  uint8_t psum;
  int wrpos;
  unsigned char *csum_pos;
  bool ok;
eat:
  while (*hash_read_pos < *hash_write_pos && hash[*hash_read_pos] != 0x53)
    {
#ifdef DBG_HEX8
      //  applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif

      *hash_read_pos += 1;
    }
  if (*hash_write_pos - *hash_read_pos < 8)
    return HEX8_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEX8_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  /* Address is outside be strict to avoid mem corruption - not fancy but it works */

  ok = (wr->command == 0x52) &&
    ((wr->address == HEX8_WORKANSWER_ADR && wr->datalength == 0x06)
     || (wr->address == 0x3008 && wr->datalength == 1));
  if (!ok)
    {
#ifdef DBG_HEX8
      //applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif
      *hash_read_pos += 1;
      goto eat;
    }
  if (*hash_write_pos - *hash_read_pos <
      HEX8_BASE_WORK_SIZE + wr->datalength * 2)
    return HEX8_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEX8_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  //Crap?
  if (csum_pos - hash < HEX8_HASH_BUF_SIZE)
    {
//That was writing somewhere and corrupting memory because of faulty usb reads....
      libhex8_csum (hash + *hash_read_pos, csum_pos, &psum);
      if (psum != *csum_pos)
        {
#ifdef DBG_HEX8
          //applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif
          *hash_read_pos += 1;
          return HEX8_BUF_ERR;
        }
    }
  else
    {
#ifdef DBG_HEX8
      //applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif
      *hash_read_pos += 1;
      return HEX8_BUF_ERR;
    }
  wrpos = (wr->address - HEX8_WORKANSWER_ADR) + HEX8_BASE_WORK_SIZE - 1;
  memcpy
    ((char *) &wr->startbyte + wrpos,
     &hash[*hash_read_pos + HEX8_BASE_WORK_SIZE - 1], wr->datalength * 2);
  *hash_read_pos += HEX8_BASE_WORK_SIZE + wr->datalength * 2;

  return HEX8_BUF_DATA;
}


static void
libhex8_set_word (struct cgpu_info *hexminer8, uint16_t address,
                  uint16_t word)
{
  unsigned char status[10];
  uint16_t wr_adr = htole16 (address);
  uint16_t ledata = htole16 (word);
  status[0] = 0x53;
  status[1] = 0x01;
  status[2] = 0x57;
  memcpy (status + 3, &wr_adr, 2);
  memcpy (status + 5, &ledata, 2);
  libhex8_csum (status, status + 7, status + 7);
  libhex8_sendHashData (hexminer8, status, 8);
}
