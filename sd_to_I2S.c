// WAV header parse → fill: rate, bits=16, ch=1 or 2, data_off, data_len

// I2S: std TX channel
i2s_chan_handle_t tx;
i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
i2s_new_channel(&chan_cfg, &tx, NULL);

i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),  // sample rate
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16,
        ch == 2 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
    .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                 .bclk = BCLK_PIN,
                 .ws = LRCK_PIN,
                 .dout = DOUT_PIN,
                 .din = I2S_GPIO_UNUSED,
                 .invert_flags = {0}}};
i2s_channel_init_std(tx, &std_cfg);
i2s_channel_enable(tx);

// SD: unbuffered FILE
FILE *fp = fopen(path, "rb");
setvbuf(fp, NULL, _IONBF, 0);
fseek(fp, data_off, SEEK_SET);

// Ping-pong DMA buffers
const size_t CHUNK = 32 * 1024;  // or 64*1024
uint8_t *bufA = heap_caps_aligned_alloc(4, CHUNK, MALLOC_CAP_DMA);
uint8_t *bufB = heap_caps_aligned_alloc(4, CHUNK, MALLOC_CAP_DMA);

uint8_t *cur = bufA, *nxt = bufB;
size_t remain = data_len;

while (remain > 0) {
  size_t to_read = remain < CHUNK ? remain : CHUNK;
  size_t br = fread(cur, 1, to_read, fp);
  if (br == 0) break;

  size_t written = 0;
  while (written < br) {
    size_t w;
    i2s_channel_write(tx, cur + written, br - written, &w, portMAX_DELAY);
    written += w;
  }

  remain -= br;
  uint8_t *tmp = cur;
  cur = nxt;
  nxt = tmp;
}

i2s_channel_disable(tx);
fclose(fp);