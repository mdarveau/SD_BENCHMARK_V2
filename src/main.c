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

#define VFS_STDIO_BUF_PREFERRED_KB 32  // Preferred stdio buffer size
#define VFS_DMA_BUF_PREFERRED_KB 8     // Preferred DMA buffer size

static void bench_vfs_fread(const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) {
    ESP_LOGE(TAG, "fopen(%s) failed", path);
    return;
  }

  uint8_t *stdio_buf = NULL;
  ESP_LOGI(TAG, "VFS stdio buffer: %d KB (INTERNAL)",
           VFS_STDIO_BUF_PREFERRED_KB);
  size_t bytes = VFS_STDIO_BUF_PREFERRED_KB * 1024;
  stdio_buf = (uint8_t *)heap_caps_aligned_alloc(
      32, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  if (stdio_buf) {
    setvbuf(fp, (char *)stdio_buf, _IOFBF, bytes);
  } else {
    ESP_LOGE(TAG, "No INTERNAL stdio buffer;");
    fclose(fp);
    return;
  }

  size_t chunk_bytes = VFS_DMA_BUF_PREFERRED_KB * 1024;
  uint8_t *buf = NULL;
  buf = (uint8_t *)heap_caps_aligned_alloc(32, chunk_bytes, MALLOC_CAP_DMA);
  if (!buf) {
    ESP_LOGE(TAG, "No DMA read buffer");
    if (stdio_buf) heap_caps_free(stdio_buf);
    fclose(fp);
    return;
  }
  ESP_LOGI(TAG, "VFS fread chunk: %u KB", (unsigned)(chunk_bytes / 1024));

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

#define SAFE_DMA_BUF_PREFERRED_KB 32  // Preferred DMA buffer size

static void bench_safe_fread(const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) {
    ESP_LOGE(TAG, "fopen(%s) failed", path);
    return;
  }

  // Get file size for progress tracking
  fseek(fp, 0, SEEK_END);
  long file_size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  ESP_LOGI(TAG, "File size: %ld bytes (%.1f KB)", file_size,
           file_size / 1024.0);

  size_t chunk_bytes = SAFE_DMA_BUF_PREFERRED_KB * 1024;
  uint8_t *buf =
      (uint8_t *)heap_caps_aligned_alloc(32, chunk_bytes, MALLOC_CAP_DMA);

  if (!buf) {
    ESP_LOGE(TAG, "No DMA read buffer available");
    fclose(fp);
    return;
  }

  int64_t t0 = esp_timer_get_time();
  size_t total = 0;
  int read_count = 0;

  for (;;) {
    size_t br = fread(buf, 1, chunk_bytes, fp);
    total += br;
    read_count++;

    // Check for EOF or short read (end of file)
    if (br < chunk_bytes) {
      // EOF or short read - file is complete
      break;
    }

    // Additional safety check based on file size
    if (file_size > 0 && total >= file_size) {
      ESP_LOGI(TAG, "Read complete file (%zu bytes)", total);
      break;
    }
  }

  int64_t dt = esp_timer_get_time() - t0;

  ESP_LOGI(TAG, "Safe fread %s %u bytes in %lld us: %.1f KB/s (%.2f MB/s)",
           path, (unsigned)total, (long long)dt, kbps(total, dt),
           kbps(total, dt) / 1024.0);

  heap_caps_free(buf);
  fclose(fp);
}

static esp_err_t init_spi(sdmmc_host_t host, spi_bus_config_t bus_cfg) {
  ESP_LOGI(TAG, "Initializing SD (SPI mode)...");

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

  CHECK(spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO),
        "spi_bus_initialize");
  // Allow shifters/power to settle before card init
  vTaskDelay(pdMS_TO_TICKS(20));
  return ESP_OK;
}

static esp_err_t mount_sdcard_at(sdmmc_host_t host, spi_bus_config_t bus_cfg) {
  ESP_LOGI(TAG, "Mounting SD card at %s...", MOUNT_POINT);
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
  ESP_LOGI(TAG, "card->max_freq_khz=%d", s_card->max_freq_khz);
  return ESP_OK;
}

static esp_err_t init_and_mount_sdcard_at(sdmmc_host_t host,
                                          spi_bus_config_t bus_cfg) {
  if (init_spi(host, bus_cfg) != ESP_OK) {
    return ESP_ERR_INVALID_STATE;
  }
  if (mount_sdcard_at(host, bus_cfg) != ESP_OK) {
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

static void unmount_and_free_bus(void) {
  if (s_card) {
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
    s_card = NULL;
  }
  spi_bus_free(s_host_slot);
}

#define READ_ON_FIRST_MOUNT 0

void app_main(void) {
  // Delay start to avoid current spikes (maybe?)
  vTaskDelay(pdMS_TO_TICKS(200));

  ESP_LOGI(TAG, "Starting SD benchmark!");

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  // host.max_freq_khz = 26000;

  spi_bus_config_t bus_cfg = {.mosi_io_num = SD_MOSI,
                              .miso_io_num = SD_MISO,
                              .sclk_io_num = SD_SCK,
                              .quadwp_io_num = -1,
                              .quadhd_io_num = -1,
                              .max_transfer_sz = 128 * 1024};

  // First mount: for some reason, the performance is quite bad on the first
  // spi init
  ESP_LOGI(TAG, "First mount...");
  if (init_spi(host, bus_cfg) != ESP_OK) {
    return;
  }
  if (READ_ON_FIRST_MOUNT) {
    if (mount_sdcard_at(host, bus_cfg) != ESP_OK) {
      return;
    }
    bench_vfs_fread(TEST_FILE_VFS_LARGE);
    bench_safe_fread(TEST_FILE_VFS_LARGE);
  }
  unmount_and_free_bus();

  // vTaskDelay(pdMS_TO_TICKS(50));

  // Second mount: ensure fast path
  ESP_LOGI(TAG, "Second mount...");
  if (init_and_mount_sdcard_at(host, bus_cfg) != ESP_OK) {
    return;
  }
  bench_vfs_fread(TEST_FILE_VFS_LARGE);
  bench_safe_fread(TEST_FILE_VFS_LARGE);
  unmount_and_free_bus();
  ESP_LOGI(TAG, "Bench complete.");
}
