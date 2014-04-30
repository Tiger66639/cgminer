/*$T indentinput.c GC 1.140 10/16/13 10:20:34 */
#define rotate(x, y)	((x << y) | (x >> (sizeof(x) * 8 - y)))
#define rotr(x, y)		((x >> y) | (x << (sizeof(x) * 8 - y)))
#define R(a, b, c, d, e, f, g, h, w, k) \
		h = h + \
		(rotate(e, 26) ^ rotate(e, 21) ^ rotate(e, 7)) + \
		(g ^ (e & (f ^ g))) + \
		k + \
		w; \
	d = d + h; \
	h = h + (rotate(a, 30) ^ rotate(a, 19) ^ rotate(a, 10)) + ((a & b) | (c & (a | b)))
extern const uint32_t SHA256_K[64];

static bool
libhex3_cachenonce (struct chip_results3 *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEX3_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEX3_NONCE_CASH_SIZE)
    return false;
  //Rotate
  if (nonce_cache->nonce_cache_write_pos == HEX3_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

char *
libhex3_set_config_voltage (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminer3-voltage";
  if (val1 < HEX3_MIN_COREMV || val1 > HEX3_MAX_COREMV)
    return "Invalid value passed to hexminer3-voltage";
  opt_hexminer3_core_voltage = val1;
  return NULL;
}

/*
    Thanks to BkkCoins & devileraser!
 */

static void
libhex3_calc_hexminer (struct work *work, struct hexminer3_task *ht)
{
  uint32_t a0a1a2e0e1e2[6];
  uint32_t A, B, C, D, E, F, G, H, T;
  uint32_t state[8];
  uint32_t data[3];
  memcpy (&state, work->midstate, 32);
  memcpy (&data, work->data + 64, 12);
#if defined(__BIG_ENDIAN__) || defined(MIPSEB)
  int i;
  for (i = 0; i < 8; i++)
    state[i] = htole32 (state[i]);
  for (i = 0; i < 3; i++)
    data[i] = htole32 (data[i]);
#endif
  A = state[0];
  B = state[1];
  C = state[2];
  D = state[3];
  E = state[4];
  F = state[5];
  G = state[6];
  H = state[7];
  R (A, B, C, D, E, F, G, H, data[0], SHA256_K[0]);
  a0a1a2e0e1e2[0] = htole32 (H);
  a0a1a2e0e1e2[3] = htole32 (D);
  R (H, A, B, C, D, E, F, G, data[1], SHA256_K[1]);
  a0a1a2e0e1e2[1] = htole32 (G);
  a0a1a2e0e1e2[4] = htole32 (C);
  R (G, H, A, B, C, D, E, F, data[2], SHA256_K[2]);
  a0a1a2e0e1e2[2] = htole32 (F);
  a0a1a2e0e1e2[5] = htole32 (B);
  memcpy (&ht->a0, &a0a1a2e0e1e2[0], 4);
  memcpy (&ht->a1, &a0a1a2e0e1e2[1], 4);
  memcpy (&ht->a2, &a0a1a2e0e1e2[2], 4);
  memcpy (&ht->e0, &a0a1a2e0e1e2[3], 4);
  memcpy (&ht->e1, &a0a1a2e0e1e2[4], 4);
  memcpy (&ht->e2, &a0a1a2e0e1e2[5], 4);
}


char *
libhex3_set_config_chip_mask (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminer3-chip-mask";
  opt_hexminer3_chip_mask = val1;
  return NULL;
}

static void
libhex3_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhex3_get_options (int this_option_offset, int *asic_count, int *frequency)
{
  char buf[BUFSIZ + 1];
  char *ptr, *comma, *colon, *colon2, *colon3, *colon4;
  bool timeout_default;
  size_t max;
  int i, tmp;
  if (opt_hexminer3_options == NULL)
    buf[0] = '\0';
  else
    {
      ptr = opt_hexminer3_options;
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
  if (tmp > 0 && tmp <= HEX3_DEFAULT_ASIC_NUM)
    *asic_count = tmp;
  else
    {
      quit (1,
            "Invalid hexminer3-options for " "asic_count (%s) must be 1 ~ %d",
            buf, HEX3_DEFAULT_ASIC_NUM);
    }
  if (colon && *colon)
    {
      tmp = atoi (colon);
      if (tmp < HEX3_MIN_FREQUENCY || tmp > HEX3_MAX_FREQUENCY)
        {
          quit
            (1,
             "Invalid hexminer3-options for frequency (%s) must be %d <= frequency <= %d",
             colon, HEX3_MIN_FREQUENCY, HEX3_MAX_FREQUENCY);
        }
      *frequency = tmp;
    }
  return true;
}

static bool
libhex3_usb_dead (struct cgpu_info *hexminer3)
{
  struct cg_usb_device *usbdev;
  struct hexminer3_info *info = hexminer3->device_data;

  usbdev = hexminer3->usbdev;
  bool ret = (usbdev == NULL
              || usbdev->handle == NULL
              || hexminer3->shutdown
              || info->shut_read || info->shut_write || info->shut_reset
              || hexminer3->usbinfo.nodev || hexminer3->deven != DEV_ENABLED);
  if (ret)
    hexminer3->shutdown = true;

  return ret;
}

static int
libhex3_sendHashData (struct cgpu_info *hexminer3, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminer3_info *info = hexminer3->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminer3->usbdev;
  if (libhex3_usb_dead (hexminer3))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err = libusb_bulk_transfer
        (usbdev->handle,
         0x02,
         sendbuf + written,
         MIN (HEX3_USB_WR_SIZE, buf_len - written), &wrote,
         HEX3_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_write = true;

  return written;
}

static void
libhex3_reset (struct cgpu_info *hexminer3)
{

  struct hexminer3_info *info = hexminer3->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminer3->usbdev;
  if (libhex3_usb_dead (hexminer3))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
    info->shut_reset = true;

  info->usb_reset_count++;
}

static int libhex3_readHashData
  (struct cgpu_info *hexminer3,
   unsigned char *hash, int *hash_write_pos, int timeout, bool read_once)
{
  struct hexminer3_info *info = hexminer3->device_data;
  struct cg_usb_device *usbdev;
  int read = 0, total = 0;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminer3->usbdev;
  if (libhex3_usb_dead (hexminer3))
    goto out;
  while (*hash_write_pos + HEX3_USB_R_SIZE < HEX3_HASH_BUF_SIZE
         && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                              HEX3_USB_R_SIZE, &read, timeout);
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
hexminer3_predecode_nonce (struct cgpu_info *hexminer3, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id)
{
  struct hexminer3_info *info = hexminer3->device_data;
  struct work *work_sub;
  mutex_lock (&info->lock);
  if (info->hexworks[work_id]->pool == NULL)
    {
      mutex_unlock (&info->lock);
      return 0;
    }

  work_sub = copy_work_noffset_fast_no_id (info->hexworks[work_id], 0);
  mutex_unlock (&info->lock);



  if (test_nonce (work_sub, nonce))
    {
      //work_sub->device_diff = diff1;
      submit_tested_work_no_clone (thr, work_sub, 1);
      return 1;
    }
  free_work (work_sub);
  return 0;
}

static void
libhex3_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 3300 / (float) ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhex3_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static void
libhex3_generatenrange_new (unsigned char *buf, int asic_num)
{
  uint32_t nonceAdd;
  int noncePos;
  int64_t nonceCalc = 0x100000000ll;
  nonceCalc /= asic_num;
  nonceAdd = (uint32_t) nonceCalc;
  uint32_t chip_noce;
  for (noncePos = 0; noncePos < asic_num; noncePos++)
    {
      /*
       * chip_noce = htole32(noncePos * nonceAdd);
       */
      chip_noce = noncePos * nonceAdd;
      memcpy (buf + noncePos * 4, &chip_noce, 4);
    }
}

static int
libhex3_eatHashData (struct work3_result *wr, unsigned char *hash,
                     int *hash_read_pos, int *hash_write_pos)
{
  uint8_t psum;
  int wrpos;
  unsigned char *csum_pos;
  bool ok;
eat:
  while (*hash_read_pos < *hash_write_pos && hash[*hash_read_pos] != 0x53)
    {
#ifdef DBG_HEX3
      //  applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif

      *hash_read_pos += 1;
    }
  if (*hash_write_pos - *hash_read_pos < 8)
    return HEX3_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEX3_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  /* Address is outside be strict to avoid mem corruption - not fancy but it works */

  //applog(LOG_ERR,"wr->address= %x, wr->command = %x, wr->datalength=%x",wr->address,wr->command,wr->datalength);
#ifdef DBG_HEX3
  ok = (wr->command == 0x52) &&
    ((wr->address == HEX3_WORKANSWER_ADR && wr->datalength == 0x05)
     || (wr->address == 0x3008 && wr->datalength == 1)
     || (wr->address == 0x300C && wr->datalength == 4));
#else
  ok = (wr->command == 0x52) &&
    ((wr->address == HEX3_WORKANSWER_ADR && wr->datalength == 0x05)
     || (wr->address == 0x3008 && wr->datalength == 1));
#endif
  if (!ok)
    {
#ifdef DBG_HEX3
      //applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif
      *hash_read_pos += 1;
      goto eat;
    }
  if (*hash_write_pos - *hash_read_pos <
      HEX3_BASE_WORK_SIZE + wr->datalength * 2)
    return HEX3_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEX3_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  //Crap?
  if (csum_pos - hash < HEX3_HASH_BUF_SIZE)
    {
//That was writing somewhere and corrupting memory because of faulty usb reads....
      libhex3_csum (hash + *hash_read_pos, csum_pos, &psum);
      if (psum != *csum_pos)
        {
#ifdef DBG_HEX3
          //applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif
          *hash_read_pos += 1;
          return HEX3_BUF_ERR;
        }
    }
  else
    {
#ifdef DBG_HEX3
      //applog (LOG_ERR, "%x", hash[*hash_read_pos]);
#endif
      *hash_read_pos += 1;
      return HEX3_BUF_ERR;
    }
  wrpos = (wr->address - HEX3_WORKANSWER_ADR) + HEX3_BASE_WORK_SIZE - 1;
  memcpy
    ((char *) &wr->startbyte + wrpos,
     &hash[*hash_read_pos + HEX3_BASE_WORK_SIZE - 1], wr->datalength * 2);
  *hash_read_pos += HEX3_BASE_WORK_SIZE + wr->datalength * 2;


  return HEX3_BUF_DATA;
}


static void
libhex3_set_word (struct cgpu_info *hexminer3, uint16_t address,
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
  libhex3_csum (status, status + 7, status + 7);
  libhex3_sendHashData (hexminer3, status, 8);
}
