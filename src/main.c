// main/app_main.c
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SD_BENCH";

/* === Pins (change if needed) === */
#define SD_CS 15
#define SD_MISO 12
#define SD_MOSI 13
#define SD_SCK 14

/* === Paths & tunables === */
#define MOUNT_POINT "/sdcard"
#define TEST_FILE_VFS_SMALL MOUNT_POINT "/DROPLE~5.WAV"
#define TEST_FILE_VFS_LARGE MOUNT_POINT "/BACKGR~3.WAV"

#define VFS_READ_CHUNK_KB 32
static const int STDIO_TRY_KB[] = {128, 64, 32};

#define RAW_XFER_SECTORS 256  // 128 KB per transfer
#define RAW_TOTAL_MB 8
#define RAW_START_LBA 4096

#ifndef CHECK
#define CHECK(call, msg)                                  \
  do {                                                    \
    esp_err_t __e = (call);                               \
    if (__e != ESP_OK) {                                  \
      ESP_LOGE(TAG, "%s: %s", msg, esp_err_to_name(__e)); \
      return __e;                                         \
    }                                                     \
  } while (0)
#endif

static sdmmc_card_t *s_card = NULL;
static int s_host_slot = SPI2_HOST;  // HSPI

// Forward declarations for helpers used before their definitions
static void unmount_and_free_bus(void);

static double kbps(size_t bytes, int64_t us) {
  if (us <= 0) return 0.0;
  return (bytes / 1024.0) / (us / 1000000.0);
}

static void list_files(const char *dirpath) {
  ESP_LOGI(TAG, "Listing %s:", MOUNT_POINT);
  DIR *dir = opendir(dirpath);
  if (!dir) {
    ESP_LOGW(TAG, "Could not open %s", dirpath);
    return;
  }
  struct dirent *e;
  while ((e = readdir(dir)) != NULL)
    ESP_LOGI(TAG, "FILE: %s/%s", dirpath, e->d_name);
  closedir(dir);
}

/* ---- fread benchmark with dynamic INTERNAL stdio buffer ---- */
static void bench_vfs_fread(const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) {
    ESP_LOGE(TAG, "fopen(%s) failed", path);
    return;
  }

  // Try to allocate an INTERNAL stdio buffer: 128K -> 64K -> 32K -> 16K
  uint8_t *stdio_buf = NULL;
  int chosen_kb = 0;
  for (size_t i = 0; i < sizeof(STDIO_TRY_KB) / sizeof(STDIO_TRY_KB[0]); ++i) {
    size_t bytes = STDIO_TRY_KB[i] * 1024;
    stdio_buf = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL);
    if (stdio_buf) {
      setvbuf(fp, (char *)stdio_buf, _IOFBF, bytes);
      chosen_kb = STDIO_TRY_KB[i];
      break;
    }
  }
  if (stdio_buf)
    ESP_LOGI(TAG, "VFS stdio buffer: %d KB (INTERNAL)", chosen_kb);
  else
    ESP_LOGW(TAG, "No INTERNAL stdio buffer; using default (slow)");

  // One-shot path disabled (unstable across runs); using fixed chunking

  // Choose a read chunk size with fallbacks (32K -> 16K)
  static const int READ_TRY_KB[] = {32, 16};
  size_t chunk_bytes = 0;
  uint8_t *buf = NULL;
  for (size_t i = 0; i < sizeof(READ_TRY_KB) / sizeof(READ_TRY_KB[0]); ++i) {
    size_t try_bytes = READ_TRY_KB[i] * 1024;
    // Must be DMA-capable and 4-byte aligned for SDSPI
    buf = (uint8_t *)heap_caps_aligned_alloc(4, try_bytes, MALLOC_CAP_DMA);
    if (buf) {
      chunk_bytes = try_bytes;
      break;
    }
  }
  if (!buf) {
    ESP_LOGE(TAG, "No DMA read buffer");
    if (stdio_buf) heap_caps_free(stdio_buf);
    fclose(fp);
    return;
  }
  ESP_LOGI(TAG, "VFS fread chunk: %u KB", (unsigned)(chunk_bytes / 1024));

  ESP_LOGI(TAG, "VFS fread: %s in %u KB chunks...", path,
           (unsigned)(chunk_bytes / 1024));

  int64_t t0 = esp_timer_get_time();
  size_t total = 0;
  for (;;) {
    size_t br = fread(buf, 1, chunk_bytes, fp);
    total += br;
    if (br < chunk_bytes) break;
  }
  int64_t dt = esp_timer_get_time() - t0;

  ESP_LOGI(TAG, "VFS fread %s %u bytes in %lld us: %.1f KB/s (%.2f MB/s)", path,
           (unsigned)total, (long long)dt, kbps(total, dt),
           kbps(total, dt) / 1024.0);

  heap_caps_free(buf);
  fclose(fp);
  if (stdio_buf) heap_caps_free(stdio_buf);
}

// dump_card_state removed for simplicity

/* ---- SD init/mount (SPI @ 20 MHz, stronger drive) ---- */
static esp_err_t init_and_mount_sdcard_at(int max_freq_khz) {
  ESP_LOGI(TAG, "Initializing SD (SPI mode) @ %d kHz...", max_freq_khz);
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = s_host_slot;  // HSPI
  host.max_freq_khz = max_freq_khz;

  // Ensure CS idles high prior to bus init
  gpio_config_t cs_cfg = {
      .pin_bit_mask = 1ULL << SD_CS,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&cs_cfg);
  gpio_set_level(SD_CS, 1);

  spi_bus_config_t bus_cfg = {.mosi_io_num = SD_MOSI,
                              .miso_io_num = SD_MISO,
                              .sclk_io_num = SD_SCK,
                              .quadwp_io_num = -1,
                              .quadhd_io_num = -1,
                              .max_transfer_sz = 128 * 1024};
  CHECK(spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO),
        "spi_bus_initialize");
  // Allow shifters/power to settle before card init
  vTaskDelay(pdMS_TO_TICKS(20));

  sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_cfg.gpio_cs = SD_CS;
  slot_cfg.host_id = host.slot;

  esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 0,
      .disk_status_check_enable = false};

  esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg,
                                          &mount_cfg, &s_card);
  if (err != ESP_OK) {
    // Retry once on timeout by resetting the bus
    if (err == ESP_ERR_TIMEOUT) {
      unmount_and_free_bus();
      vTaskDelay(pdMS_TO_TICKS(50));
      // Re-init bus and retry mount
      if (spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mount_cfg,
                                      &s_card);
      }
    }
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mount/init failed: %s", esp_err_to_name(err));
      return err;
    }
  }
  sdmmc_card_print_info(stdout, s_card);
  ESP_LOGI(TAG, "card->max_freq_khz=%d (host asked %d)", s_card->max_freq_khz,
           max_freq_khz);
  return ESP_OK;
}

static void unmount_and_free_bus(void) {
  if (s_card) {
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
    s_card = NULL;
  }
  spi_bus_free(s_host_slot);
}

/* ---- Run it in a modest stack to free INTERNAL RAM ---- */
static void bench_task(void *arg) {
  // First mount: bench immediately
  if (init_and_mount_sdcard_at(26000) != ESP_OK) {
    vTaskDelete(NULL);
    return;
  }
  bench_vfs_fread(TEST_FILE_VFS_LARGE);
  unmount_and_free_bus();

  // Second mount: ensure fast path
  vTaskDelay(pdMS_TO_TICKS(50));
  if (init_and_mount_sdcard_at(26000) != ESP_OK) {
    vTaskDelete(NULL);
    return;
  }
  bench_vfs_fread(TEST_FILE_VFS_LARGE);
  unmount_and_free_bus();
  ESP_LOGI(TAG, "Bench complete.");
  vTaskDelete(NULL);
}

void print_memory_info(void) {
  ESP_LOGI(TAG, "8-bit total/free/largest: %u / %u / %u",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  ESP_LOGI(TAG, "DMA total/free/largest:   %u / %u / %u",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

  ESP_LOGI(TAG, "INTERNAL total/free:      %u / %u",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

#ifdef CONFIG_SPIRAM
  ESP_LOGI(TAG, "SPIRAM total/free/largest:%u / %u / %u",
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
#endif
}

void app_main(void) {
  // esp_log_level_set("*", ESP_LOG_INFO);           // allow INFO for all
  // tags esp_log_level_set(TAG, ESP_LOG_INFO);
  // esp_log_level_set("sdspi_transaction", ESP_LOG_DEBUG);
  // esp_log_level_set("sdspi_host",        ESP_LOG_DEBUG);
  // esp_log_level_set("sdmmc_cmd",         ESP_LOG_DEBUG);
  // esp_log_level_set("vfs_fat_sdmmc",     ESP_LOG_DEBUG);

  //   Smaller stack (10 KB) so large INTERNAL buffers are more likely to
  //   succeed
  const uint32_t stack_words = (10 * 1024) / sizeof(StackType_t);
  xTaskCreatePinnedToCore(bench_task, "bench_task", stack_words, NULL,
                          tskIDLE_PRIORITY + 1, NULL, tskNO_AFFINITY);
}
