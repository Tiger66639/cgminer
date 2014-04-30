/*$T indentinput.c GC 1.140 10/16/13 10:20:34 */
const uint32_t SHA_M[64] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf
};

#define Ch(x, y, z) ((x & (y ^ z)) ^ z)
#define Maj(x, y, z) ((x & (y | z)) | (y & z))
#define ROTR(x, n) ((x >> n) | (x << (32 - n)))
#define S0(x) (ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define S1(x) (ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25))
//0x2800000, 0x2C00000, 0x400000 - CPU wasteed No valid results for the moment
#define BT_OFFSETS_B 6
//static const uint32_t bf_offsetsu[] = {-0x800000, 0, 0xffc00000, 0xff800000, 0x02800000, 0x02C00000, 0x00400000};
const uint32_t bf_offsetsb[] =
  { -0x800000, 0, -0x400000, 0x2800000, 0x2C00000, 0x400000 };

static void
BITFURY_MS3compute (struct work *work, struct hexminerb_task *ht)
{
  uint32_t state[8];
  uint32_t data[3];
  memcpy (&state, work->midstate, 32);
  memcpy (&data, work->data + 64, 12);
  uint32_t a, b, c, d, e, f, g, h, ne, na;
  int i;
#if defined(__BIG_ENDIAN__) || defined(MIPSEB)
  for (i = 0; i < 8; i++)
    state[i] = htole32 (state[i]);
  for (i = 0; i < 3; i++)
    data[i] = htole32 (data[i]);
#endif
  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];
  f = state[5];
  g = state[6];
  h = state[7];
  for (i = 0; i < 3; i++)
    {
      ne = data[i] + SHA_M[i] + h + Ch (e, f, g) + S1 (e) + d;
      na = data[i] + SHA_M[i] + h + Ch (e, f, g) + S1 (e) + S0 (a) +
        Maj (a, b, c);
      d = c;
      c = b;
      b = a;
      a = na;
      h = g;
      g = f;
      f = e;
      e = ne;
    }
#if defined(__BIG_ENDIAN__) || defined(MIPSEB)
  a = htole32 (a);
  b = htole32 (b);
  c = htole32 (c);
  d = htole32 (d);
  e = htole32 (e);
  f = htole32 (f);
  g = htole32 (g);
  h = htole32 (h);
#endif
  memcpy (&ht->a1, &h, 4);
  memcpy (&ht->a0, &g, 4);
  memcpy (&ht->e2, &f, 4);
  memcpy (&ht->e1, &e, 4);
  memcpy (&ht->e0, &d, 4);
  memcpy (&ht->a2, &c, 4);
  memcpy (&ht->startnonce, &b, 4);
  memcpy (&ht->reftemperature, &a, 4);
}

static bool
libhexb_cachenonce (struct chip_resultsb *nonce_cache, uint32_t nonce)
{
  int i = 0;
  while (i < HEXB_NONCE_CASH_SIZE && nonce_cache->nonces[i] != nonce)
    i++;
  if (i < HEXB_NONCE_CASH_SIZE)
    return false;
  //Rotate
  if (nonce_cache->nonce_cache_write_pos == HEXB_NONCE_CASH_SIZE)
    nonce_cache->nonce_cache_write_pos = 0;
  nonce_cache->nonces[nonce_cache->nonce_cache_write_pos++] = nonce;
  return true;
}

char *
libhexb_set_config_voltage (char *arg)
{
  int val1, ret;
  ret = sscanf (arg, "%d", &val1);
  if (ret < 1)
    return "No values passed to hexminerb-voltage";
  if (val1 < HEXB_MIN_COREMV || val1 > HEXB_MAX_COREMV)
    return "Invalid value passed to hexminerb-voltage";
  opt_hexminerb_core_voltage = val1;
  return NULL;
}

static void
libhexb_csum (unsigned char *startptr, unsigned char *endptr,
              unsigned char *resptr)
{
  unsigned char *b = startptr;
  uint8_t sum = 0;
  while (b < endptr)
    sum += *b++;
  memcpy (resptr, &sum, 1);
}

static bool
libhexb_get_options (int this_option_offset, int *asic_count, int *frequency)
{
  char buf[BUFSIZ + 1];
  char *ptr, *comma, *colon, *colon2, *colon3, *colon4;
  bool timeout_default;
  size_t max;
  int i, tmp;

  if (opt_hexminerb_options == NULL)
    buf[0] = '\0';
  else
    {
      ptr = opt_hexminerb_options;
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
  if (tmp > 0 && tmp <= HEXB_DEFAULT_ASIC_NUM)
    *asic_count = tmp;
  else
    {
      quit (1,
            "Invalid hexminerb-options for " "asic_count (%s) must be 1 ~ %d",
            buf, HEXB_DEFAULT_ASIC_NUM);
    }
  if (colon && *colon)
    {
      tmp = atoi (colon);
      if (tmp < HEXB_MIN_FREQUENCY || tmp > HEXB_MAX_FREQUENCY)
        {
          quit
            (1,
             "Invalid hexminerb-options for frequency (%s) must be %d <= frequency <= %d",
             colon, HEXB_MIN_FREQUENCY, HEXB_MAX_FREQUENCY);
        }
      *frequency = tmp;
    }
  return true;
}

static bool
libhexb_usb_dead (struct cgpu_info *hexminerb)
{
  struct cg_usb_device *usbdev;
  struct hexminerb_info *info = hexminerb->device_data;
  
  usbdev = hexminerb->usbdev;
  bool ret = (usbdev == NULL
          || usbdev->handle == NULL
          || hexminerb->shutdown
          || info->shut_read  || info->shut_write || info->shut_reset
          || hexminerb->usbinfo.nodev || hexminerb->deven != DEV_ENABLED);
  if(ret)        
  	hexminerb->shutdown = true;
  	
  return ret;
}


static int
libhexb_sendHashData (struct cgpu_info *hexminerb, unsigned char *sendbuf,
                      size_t buf_len)
{
  struct hexminerb_info *info = hexminerb->device_data;
  struct cg_usb_device *usbdev;
  int wrote = 0, written = 0;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminerb->usbdev;
  if (libhexb_usb_dead (hexminerb))
    goto out;
  while (written < buf_len && err == LIBUSB_SUCCESS)
    {
      err = libusb_bulk_transfer
        (usbdev->handle,
         0x02,
         sendbuf + written,
         MIN (HEXB_USB_WR_SIZE, buf_len - written), &wrote,
         HEXB_USB_WR_TIME_OUT);
      if (err == LIBUSB_SUCCESS)
        written += wrote;
    }
out:
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
  	info->shut_write = true; 
	    
  return written;
}

static void
libhexb_reset (struct cgpu_info *hexminerb)
{

  struct hexminerb_info *info = hexminerb->device_data;
  struct cg_usb_device *usbdev;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminerb->usbdev;
  if (libhexb_usb_dead (hexminerb))
    goto out;
  err = libusb_reset_device (usbdev->handle);
out:
 
  if (err == LIBUSB_ERROR_NO_DEVICE || err == LIBUSB_ERROR_NOT_FOUND)
	   info->shut_reset = true; 
     
  info->usb_reset_count++;
}

static int libhexb_readHashData
  (struct cgpu_info *hexminerb,
   unsigned char *hash, int *hash_write_pos, int timeout, bool read_once)
{
  struct hexminerb_info *info = hexminerb->device_data;
  struct cg_usb_device *usbdev;
  int read = 0, total = 0;
  int err = LIBUSB_SUCCESS;

  usbdev = hexminerb->usbdev;
  if (libhexb_usb_dead (hexminerb))
    goto out;
  while (*hash_write_pos + HEXB_USB_R_SIZE < HEXB_HASH_BUF_SIZE
         && err == LIBUSB_SUCCESS)
    {
      err =
        libusb_bulk_transfer (usbdev->handle, 0x82, hash + *hash_write_pos,
                              HEXB_USB_R_SIZE, &read, timeout);
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

static uint32_t
decnonce (uint32_t in)
{
  uint32_t out;
  /* First part load */
  out = (in & 0xFF) << 24;
  in >>= 8;
  /* Byte reversal */
  in = (((in & 0xaaaaaaaa) >> 1) | ((in & 0x55555555) << 1));
  in = (((in & 0xcccccccc) >> 2) | ((in & 0x33333333) << 2));
  in = (((in & 0xf0f0f0f0) >> 4) | ((in & 0x0f0f0f0f) << 4));
  out |= (in >> 2) & 0x3FFFFF;
  /* Extraction */
  if (in & 1)
    out |= (1 << 23);
  if (in & 2)
    out |= (1 << 22);
  out -= 0x800004;
  return out;
}

  /*
  double pool_diff;
  char *mjob_id;
  cg_rlock(&work->pool->data_lock);
	pool_diff = work->pool->sdiff;
	mjob_id = strdup(work->pool->swork.job_id);
	
	cg_runlock(&work->pool->data_lock);
  //work->gbt
    	  if(work->sdiff != pool_diff) {
    	  	//free(work->job_id);
          //work->job_id = mjob_id; 	
          	applog(LOG_ERR, "Retarget work->work_difficulty %f,work->sdiff %f, pool_diff %f, work->job_id %s, mjob_id %s",
          	work->work_difficulty, work->sdiff,pool_diff, work->job_id,mjob_id);
           //work->sdiff = pool_diff;
           //set_target(work->target, work->sdiff);
           //work->work_difficulty = work->sdiff;
           //calc_diff(work, work->sdiff); 
        }
  */

static int
bitfury_checkresults (struct thr_info *thr, struct work *work, uint32_t nonce)
{
	#ifdef DBG_HEXB
	struct cgpu_info *hexminerb = thr->cgpu;
  struct hexminerb_info *info = hexminerb->device_data;
  #endif
  int i;
  
  for (i = 0; i < BT_OFFSETS_B; i++)
    {
    	
      if (test_nonce (work, nonce + bf_offsetsb[i]))
        {
       
          submit_tested_work_no_clone (thr, work, true);
          
          return 1;
        }
    }
  free_work (work);
          
  return 0;
}

static int
hexminerb_predecode_nonce (struct cgpu_info *hexminerb, struct thr_info *thr,
                           uint32_t nonce, uint8_t work_id)
{
  struct hexminerb_info *info = hexminerb->device_data;
  struct work *work_sub;
  mutex_lock (&info->lock);
  if(info->hexworks[work_id]->pool==NULL) {
  	mutex_unlock (&info->lock);
  	return 0;
	}

  work_sub = copy_work_noffset_fast_no_id (info->hexworks[work_id], 0);
  mutex_unlock (&info->lock);


  return bitfury_checkresults (thr, work_sub, nonce);

}

static void
libhexb_getvoltage (uint16_t wr_bukvoltage, int *info_pic_voltage_readings)
{
  float voltagehuman;
  voltagehuman =
    (float) ((float) wr_bukvoltage * (float) 1000 * (float) 3.3 /
             ((1 << 12) - 1));
  *info_pic_voltage_readings = (int) voltagehuman;
}

static void
libhexb_setvoltage (int info_voltage, uint16_t * refvoltage)
{
  uint16_t voltageadc;
  voltageadc =
    (uint16_t) ((float) info_voltage / (float) 1000 / (float) 3.3 *
                ((1 << 12) - 1));
  *refvoltage = htole16 (voltageadc);
}

static int
libhexb_eatHashData (struct workb_result *wr, unsigned char *hash,
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
  if (*hash_write_pos - *hash_read_pos < HEXB_BASE_WORK_SIZE + 2)
    return HEXB_BUF_SKIP;
  memcpy ((char *) &wr->startbyte, &hash[*hash_read_pos],
          HEXB_BASE_WORK_SIZE - 1);

  wr->address = htole16 (wr->address);

  /* Address is outside be strict to avoid mem corruption - not fancy but it works */

  ok = (wr->command == 0x52) &&
    ((wr->address == HEXB_WORKANSWER_ADR && wr->datalength == 0x05)
     || (wr->address == HEXB_WORKANSWER_ADR + 4 && wr->datalength == 1));
  if (!ok)
    {
      *hash_read_pos += 1;
      goto eat;
    }
  if (*hash_write_pos - *hash_read_pos <
      HEXB_BASE_WORK_SIZE + wr->datalength * 2)
    return HEXB_BUF_SKIP;
  csum_pos =
    hash + *hash_read_pos + HEXB_BASE_WORK_SIZE + wr->datalength * 2 - 1;
  //Crap?
  if (csum_pos - hash < HEXB_HASH_BUF_SIZE)
    {
//That was writing somewhere and corrupting memory because of faulty usb reads....
      libhexb_csum (hash + *hash_read_pos, csum_pos, &psum);
      if (psum != *csum_pos)
        {

          *hash_read_pos += 1;
          return HEXB_BUF_ERR;
        }
    }
  else
    {
#ifdef DBG_HEXB_BUF
      applog (LOG_ERR,
              "Lost Nonce/Status due to incomplete usb read?? wr->datalength = %i csum_pos=%i, *hash_write_pos%i, left=%i",
              wr->datalength, csum_pos, *hash_write_pos,
              HEXB_HASH_BUF_SIZE - *hash_write_pos);
#endif
      *hash_read_pos += 1;
      return HEXB_BUF_ERR;
    }

  wrpos = (wr->address - HEXB_WORKANSWER_ADR) + HEXB_BASE_WORK_SIZE - 1;
  memcpy
    ((char *) &wr->startbyte + wrpos,
     &hash[*hash_read_pos + HEXB_BASE_WORK_SIZE - 1], wr->datalength * 2);
  *hash_read_pos += HEXB_BASE_WORK_SIZE + wr->datalength * 2;
  return HEXB_BUF_DATA;

}
