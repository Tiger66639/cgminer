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
const uint32_t SHA256_K[3] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf };

static bool
libhexa_cachenonce (struct chip_resultsa *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEXA_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEXA_NONCE_CASH_SIZE)
    return false;
  //Rotate
  if (nonce_cache->nonce_cache_write_pos == HEXA_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

static void
libhexa_generatenrange_new (unsigned char *buf, int asic_num)
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

char *
libhexa_set_config_voltage (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminera-voltage";
  if (val1 < HEXA_MIN_COREMV || val1 > HEXA_MAX_COREMV)
    return "Invalid value passed to hexminera-voltage";
  opt_hexminera_core_voltage = val1;
  return NULL;
}

/*
    Thanks to BkkCoins & devileraser!
 */

static void
libhexa_calc_hexminer (struct work *work, struct hexminera_task *ht)
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

/*
    From Hexminer core developer Thanks!
 */

static void
libhexa_generateclk (uint16_t HashClock, uint16_t XCLKIN, uint32_t * res)
{
  uint32_t configL = 0;
  uint32_t configH = 0;
  int RValue = XCLKIN;
  int NValue = (HashClock * 2 * RValue / XCLKIN);
  configL =
    ((uint32_t) RValue << 29) | ((uint32_t) NValue << 18) |
    HEXA_CLOCK_LOW_CFG;
  configH = ((uint32_t) RValue >> 3) | HEXA_CLOCK_HIGH_CFG;
  res[0] = htole32 (configL);
  res[1] = htole32 (configH);
}

static void
libhexa_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{

  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhexa_get_options (int this_option_offset, int *asic_count, int *frequency)
{
  char buf[BUFSIZ + 1];
  char *ptr, *comma, *colon, *colon2, *colon3, *colon4;
  bool timeout_default;
  size_t max;
  int i, tmp;
  if (opt_hexminera_options == NULL)
    buf[0] = '\0';
  else
    {
      ptr = opt_hexminera_options;
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
  if (tmp > 0 && tmp <= HEXA_DEFAULT_ASIC_NUM)
    *asic_count = tmp;
  else
    {
      quit (1,
            "Invalid hexminera-options for " "asic_count (%s) must be 1 ~ %d",
            buf, HEXA_DEFAULT_ASIC_NUM);
    }
  if (colon && *colon)
    {
      tmp = atoi (colon);
      if (tmp < HEXA_MIN_FREQUENCY || tmp > HEXA_MAX_FREQUENCY)
        {
          quit
            (1,
             "Invalid hexminera-options for frequency (%s) must be %d <= frequency <= %d",
             colon, HEXA_MIN_FREQUENCY, HEXA_MAX_FREQUENCY);
        }
      *frequency = tmp;
    }
  return true;
}

static bool
libhexa_usb_dead (struct cgpu_info *hexminera)
{
  struct cg_usb_device *usbdev;
  struct hexminera_info *info = hexminera->device_data;
  
  usbdev = hexminera->usbdev;
  bool ret = (usbdev == NULL
          || usbdev->handle == NULL
          || hexminera->shutdown
          || info->shut_read  || info->shut_write || info->shut_reset || info->shut_read
          || hexminera->usbinfo.nodev || hexminera->deven != DEV_ENABLED);
  if(ret)        
  	hexminera->shutdown = true;
  	
  return ret;
}


unsigned long long
libhexa_et ()
{
struct timeval tv;

gettimeofday(&tv, NULL);

return (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
    
} 
  
static int
libhexa_sendHashData (struct cgpu_info *hexminera, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminera_info *info = hexminera->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;
  usbdev = hexminera->usbdev;
  if (libhexa_usb_dead (hexminera))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err = libusb_bulk_transfer
        (usbdev->handle,
         0x02,
         sendbuf + written,
         MIN (HEXA_USB_WR_SIZE, buf_len - written), &wrote,
         HEXA_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
  	info->shut_write = true; 
	    
  return written;
}

static void
libhexa_reset (struct cgpu_info *hexminera)
{

  struct hexminera_info *info = hexminera->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;
  
  usbdev = hexminera->usbdev;
  if (libhexa_usb_dead (hexminera))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
 
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
	   info->shut_reset = true; 
     
  info->usb_reset_count++;

}

static int libhexa_readHashData
  (struct cgpu_info *hexminera,
   unsigned char *hash, int *hash_write_pos, int timeout, bool read_once)
{
  struct hexminera_info *info = hexminera->device_data;
  struct cg_usb_device *usbdev;
  int read = 0, total = 0;
  int err = LIBUSB_SUCCESS;
  
  usbdev = hexminera->usbdev;
  if (libhexa_usb_dead (hexminera))
    goto out;
  while (*hash_write_pos + HEXA_USB_R_SIZE < HEXA_HASH_BUF_SIZE
         && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                              HEXA_USB_R_SIZE, &read, timeout);
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
    }
  
  return err;
}

static int
hexminera_predecode_nonce (struct cgpu_info *hexminera, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id)
{
  struct hexminera_info *info = hexminera->device_data;
  struct work *work_sub;

  
  if (info->hexworks[work_id]->pool == NULL)
      return 0;
  work_sub = copy_work_noffset_fast_no_id (info->hexworks[work_id], 0);
  
  if (test_nonce (work_sub, nonce))
    {
      submit_tested_work_no_clone (thr, work_sub, true);
      return 1;
    }
  else
    {
      free_work (work_sub);
    }

  return 0;
}

/*
    From Hexminer core developer Thanks!
 */

static void
libhexa_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 1000 * (float) 3.3 /
             ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

/*
    From Hexminer core developer Thanks!
 */

static void
libhexa_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhexa_eatHashData (struct worka_result *wr, unsigned char *hash,
                     int *hash_read_pos, int *hash_write_pos)
{
  uint8_t psum;
  int wrpos;
  unsigned char *csum_pos;
  bool ok;
eat:
  while (*hash_read_pos < *hash_write_pos && hash[*hash_read_pos] != 0x53)
    {
      *hash_read_pos += 1;
    }
  if (*hash_write_pos - *hash_read_pos < HEXA_BASE_WORK_SIZE)
    return HEXA_BUF_SKIP;

  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEXA_BASE_WORK_SIZE - 1);
  wr->address = htole16 (wr->address);
  /* Address is outside be strict to avoid mem corruption - not fancy but it works */
  //applog (LOG_ERR , "libhexa_eatHashData wr->command %0x, wr->address %0x,wr->datalength %0x, ", wr->command,wr->address,wr->datalength  ); 
  ok = (wr->command == 0x52) &&
    ((wr->address == HEXA_WORKANSWER_ADR && wr->datalength == 0x06)
     || (wr->address == HEXA_WORKANSWER_ADR && wr->datalength == 0x0C)
     || (wr->address == HEXA_WORKANSWER_STAT_ADR && wr->datalength == 1));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (*hash_write_pos - *hash_read_pos <
      HEXA_BASE_WORK_SIZE + wr->datalength * 2)
    return HEXA_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEXA_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  //Crap?
  if (csum_pos - hash < HEXA_HASH_BUF_SIZE)
    {
      //That was writing somewhere and corrupting memory because of faulty usb reads....
      libhexa_csum (hash + *hash_read_pos, csum_pos, &psum);
      if (psum != *csum_pos)
        {
          *hash_read_pos += 1;
          return HEXA_BUF_ERR;
        }
    }
  else
    {

      *hash_read_pos += 1;
      return HEXA_BUF_ERR;
    }
  wrpos = (wr->address - HEXA_WORKANSWER_ADR) + HEXA_BASE_WORK_SIZE - 1;

  memcpy
    ((char *) &wr->startbyte + wrpos,
     &hash[*hash_read_pos + HEXA_BASE_WORK_SIZE - 1],
     MIN (wr->datalength * 2, HEXA_MAX_WORK_SIZE - HEXA_BASE_WORK_SIZE - 1));
  *hash_read_pos += HEXA_BASE_WORK_SIZE + wr->datalength * 2;

  return HEXA_BUF_DATA;

}

static void
BitUpdateInRAMAndSend (struct cgpu_info *hexminera, uint16_t address,
                       uint16_t bitPos, bool value)
{
  unsigned char buf[10];
  uint16_t wr_adr = htole16 (address);
  buf[0] = 0x53;
  buf[1] = 0x01;
  buf[2] = 0x42;
  memcpy (buf + 3, &wr_adr, 2);
  if (value == true)
    {
      buf[6] = 0x80;
    }
  else
    {
      buf[6] = 0x00;
    }
  buf[5] = bitPos;
  libhexa_csum (buf, buf + 7, buf + 7);
  libhexa_sendHashData (hexminera, buf, 8);
}

static void
libhexa_get_words (struct cgpu_info *hexminera, uint16_t address,
                   uint8_t words)
{
  unsigned char status[10];
  uint16_t wr_adr = htole16 (address);
  status[0] = 0x53;
  status[1] = 0x01;
  status[2] = 0x52;
  memcpy (status + 3, &wr_adr, 2);
  status[5] = words;
  status[6] = 0x00;
  libhexa_csum (status, status + 7, status + 7);
  libhexa_sendHashData (hexminera, status, 8);
}

static void
libhexa_set_word (struct cgpu_info *hexminera, uint16_t address,
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
  libhexa_csum (status, status + 7, status + 7);
  libhexa_sendHashData (hexminera, status, 8);
}
