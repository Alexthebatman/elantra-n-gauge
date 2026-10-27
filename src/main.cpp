#include <Arduino.h>
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "hyundai_n_lvgl.h"
#include "hyundai_n_watermark.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <Wire.h>
#include <esp_system.h>

// ============================================================================
// Elantra N Gauge - FULL UI / FAST CORE
//
// Goals:
//   * Preserve the full round gauge, N artwork, overview, and individual pages.
//   * Make the hot path deterministic: no Arduino String, no blocking OBD waits,
//     no predictive value animation, no periodic redraw loop.
//   * Render only when fresh data actually changes something visible.
//   * Use fixed-size LVGL labels and one display buffer to reduce the chance of
//     the occasional half-drawn / jumbled number seen in the original gauge.
//
// Vehicle data is read-only standard OBD-II Mode 01.
// ============================================================================

#define BUTTON_BOOT 0

// Physical side buttons on this Waveshare board:
//   PWR  -> TCA9554 EXIO4 (high while pressed)
//   BOOT -> ESP32-S3 GPIO0 (low while pressed)
// The installed gauge orientation used by this project treats PWR as the
// easy-to-reach LEFT button and BOOT as the RIGHT button.
static constexpr uint8_t TCA9554_ADDR = 0x20;
static constexpr uint8_t TCA9554_INPUT_REG = 0x00;
static constexpr uint8_t TCA9554_PWR_MASK = (1u << 4);
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 25u;
static constexpr uint32_t BUTTON_REARM_RELEASE_MS = 120u;
static constexpr uint32_t BUTTON_ACTION_COOLDOWN_MS = 120u;
static constexpr uint32_t LEFT_NAV_BOOT_LOCKOUT_MS = 12000u;
static constexpr uint32_t BUTTON_RUNAWAY_WINDOW_MS = 1800u;
static constexpr uint8_t BUTTON_RUNAWAY_MAX_ACTIONS = 6u;
static constexpr uint32_t BUTTON_RUNAWAY_UNLOCK_MS = 1000u;
static constexpr uint32_t CONFIG_HOLD_MS = 2500u;
static constexpr char GAUGE_FW_VERSION[] = "2026.10.26-r5-stable-buttons-ui";

// The ELM327 BLE service. Standard Veepeak/vLinker/most BLE OBD dongles expose
// data over the 0xFFF0 service (write 0xFFF2 / notify 0xFFF1). We connect to
// whatever advertises this service instead of hard-coding one dongle's MAC, so
// the same firmware works on any buyer's adapter.
#define OBD_SERVICE_UUID   ((uint16_t)0xFFF0)
#define OBD_WRITE_UUID     ((uint16_t)0xFFF2)
#define OBD_NOTIFY_UUID    ((uint16_t)0xFFF1)

// Fallback name matching, used only if nothing advertises 0xFFF0 in its
// advertisement packet. Any post-connect device is still verified by actually
// finding the 0xFFF0 service, so a loose name match here can never connect us
// to the wrong kind of device.
static const char* const OBD_NAME_HINTS[] = { "VEEPEAK", "OBD", "ELM", "VLINK", "VLINKER" };

// Leave enabled for the first test build. There is no per-sample Serial spam,
// so this has effectively no driving-time performance cost.
#define GAUGE_DEBUG 1

#if GAUGE_DEBUG
  #define DBG_BEGIN()       Serial.begin(115200)
  #define DBG_PRINT(x)      Serial.print(x)
  #define DBG_PRINTLN(x)    Serial.println(x)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN()       do {} while (0)
  #define DBG_PRINT(x)      do {} while (0)
  #define DBG_PRINTLN(x)    do {} while (0)
  #define DBG_PRINTF(...)   do {} while (0)
#endif

// ----------------------------------------------------------------------------
// Display hardware
// ----------------------------------------------------------------------------
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1,
  LCD_SDIO2, LCD_SDIO3);

Arduino_CO5300 *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *drawBuffer = nullptr;
static uint16_t *rotateBuffer = nullptr;

// ----------------------------------------------------------------------------
// LVGL objects
// ----------------------------------------------------------------------------
// Single gauge screen
static lv_obj_t *scr_single;
static lv_obj_t *arc_rpm;
static lv_obj_t *label_main_value;
static lv_obj_t *label_main_unit;
static lv_obj_t *label_status;
static lv_obj_t *label_extra;
static lv_obj_t *label_rpm_bottom;
static lv_obj_t *img_n_badge;
static lv_obj_t *iat_divider;

// Overview screen
static lv_obj_t *scr_overview;
static lv_obj_t *img_n_watermark;
static lv_obj_t *ov_rows[6][2];  // BOOST, IAT, IAT DELTA, COOLANT, IGN, BATT
static lv_obj_t *ov_rpm_label;
static lv_obj_t *ov_arc_rpm;

// Boot screen
static lv_obj_t *scr_boot;
static lv_obj_t *img_boot_logo;

// Wi-Fi setup screen
static lv_obj_t *scr_config;
static lv_obj_t *label_config_password;
static bool transitionedToGauge = false;

// Persistent text buffers. lv_label_set_text_static() points directly at these,
// avoiding LVGL text reallocations while the gauge is running.
static char singleRpmText[16] = "-- RPM";
static char overviewRpmText[16] = "-- RPM";
static char mainValueText[24] = "--";
static char statusText[32] = "";
static char extraText[32] = "";
static char overviewValueText[6][16] = {"--", "--", "--", "--", "--", "--"};

// ----------------------------------------------------------------------------
// Gauge pages
// ----------------------------------------------------------------------------
enum GaugeType : uint8_t {
  GAUGE_OVERVIEW = 0,
  GAUGE_BOOST,
  GAUGE_IAT,
  GAUGE_COOLANT,
  GAUGE_SPARK,
  GAUGE_BATTERY,
  GAUGE_COUNT
};

static GaugeType currentGauge = GAUGE_OVERVIEW;

// ----------------------------------------------------------------------------
// Persistent user configuration
// ----------------------------------------------------------------------------
// LEFT / PWR cycles pages. RIGHT / BOOT short press returns immediately to
// Home/Overview. Hold RIGHT / BOOT for ~2.5 seconds to enter parked setup mode.
// While setup is active, a fresh short RIGHT press reboots back to gauge mode.
// BLE/OBD is shut down in setup, the ESP32 starts this local Wi-Fi network, and
// configuration / phone firmware upload is served at http://192.168.4.1.
static constexpr char CONFIG_AP_SSID[] = "ElantraN-Gauge-Setup";
static constexpr uint8_t DEFAULT_BRIGHTNESS = 240;
static constexpr uint8_t VALID_PAGE_MASK = (1u << GAUGE_COUNT) - 1u;
static constexpr uint8_t DEFAULT_PAGE_MASK = VALID_PAGE_MASK;

// Setup Wi-Fi is WPA2-protected with a per-device password derived from the
// ESP32-S3 eFuse MAC. The password is shown on the gauge whenever setup mode
// is active, so there is no fixed fleet-wide password to leak.
static char configApPassword[20] = "NGAUGE-000000";
static uint32_t configCsrfToken = 0;

// Browser/app OTA state. Update.h writes the new image into the inactive OTA
// slot and only selects it for boot after Update.end(false) verifies the complete image.
static bool otaInProgress = false;
static bool otaUploadComplete = false;
static bool otaUploadAuthorized = false;
static size_t otaUploadBytes = 0;
static size_t otaExpectedBytes = 0;
static char otaErrorText[96] = "";

struct GaugeConfig {
  uint8_t pageMask = DEFAULT_PAGE_MASK;
  uint8_t brightness = DEFAULT_BRIGHTNESS;
};

static GaugeConfig gaugeConfig;
static Preferences preferences;
static WebServer configServer(80);
static bool configModeActive = false;

static const char* const GAUGE_PAGE_NAMES[GAUGE_COUNT] = {
  "Overview", "Boost", "IAT + Ambient", "Coolant", "Ignition Timing", "Battery"
};

static inline bool isPageEnabled(GaugeType page) {
  return (gaugeConfig.pageMask & (1u << (uint8_t)page)) != 0;
}

// Convert the old seven-page layout
//   Overview, Boost, IAT, IAT-AMB, Coolant, Spark, Battery
// into the new six-page layout where IAT and IAT-AMB are one page.
static uint8_t migrateOldPageMask(uint8_t oldMask) {
  uint8_t out = 0;
  if (oldMask & (1u << 0)) out |= (1u << GAUGE_OVERVIEW);
  if (oldMask & (1u << 1)) out |= (1u << GAUGE_BOOST);
  if (oldMask & ((1u << 2) | (1u << 3))) out |= (1u << GAUGE_IAT);
  if (oldMask & (1u << 4)) out |= (1u << GAUGE_COOLANT);
  if (oldMask & (1u << 5)) out |= (1u << GAUGE_SPARK);
  if (oldMask & (1u << 6)) out |= (1u << GAUGE_BATTERY);
  return out & VALID_PAGE_MASK;
}

static void buildSetupCredentials() {
  const uint64_t mac = ESP.getEfuseMac();
  const uint32_t suffix = (uint32_t)(mac & 0xFFFFFFu);
  snprintf(configApPassword, sizeof(configApPassword), "NGAUGE-%06lX",
           (unsigned long)suffix);
}

static void rotateConfigCsrfToken() {
  configCsrfToken = esp_random();
  if (configCsrfToken == 0) configCsrfToken = 1;
}

static void csrfTokenText(char *out, size_t outLen) {
  snprintf(out, outLen, "%08lX", (unsigned long)configCsrfToken);
}

static bool configTokenMatches() {
  if (!configServer.hasArg("token")) return false;
  char expected[9];
  csrfTokenText(expected, sizeof(expected));
  return configServer.arg("token").equalsIgnoreCase(expected);
}

static void setOtaError(const char *message) {
  if (!message || !message[0]) message = "Unknown OTA error";
  snprintf(otaErrorText, sizeof(otaErrorText), "%s", message);
}

static void setDefaultConfig() {
  gaugeConfig.pageMask = DEFAULT_PAGE_MASK;
  gaugeConfig.brightness = DEFAULT_BRIGHTNESS;
}

static void loadConfig() {
  setDefaultConfig();
  if (!preferences.begin("ngauge", true)) return;

  // "pages3" belongs to this merged-IAT/single-profile firmware. On the first
  // boot after the older Normal/Sport firmware, preserve every page that was
  // enabled in either profile, then collapse IAT + IAT-AMB into one IAT page.
  const uint8_t storedNew = preferences.getUChar("pages3", 0xFF);
  const uint8_t legacySingle = preferences.getUChar("pages", 0xFF);
  const uint8_t legacyNormal = preferences.getUChar("normal", 0xFF);
  const uint8_t legacySport = preferences.getUChar("sport", 0xFF);
  const uint8_t storedBrightness = preferences.getUChar("bright", DEFAULT_BRIGHTNESS);
  preferences.end();

  uint8_t pageMask = DEFAULT_PAGE_MASK;
  if (storedNew != 0xFF) {
    pageMask = storedNew & VALID_PAGE_MASK;
  } else {
    uint8_t oldMask = 0;
    bool haveOld = false;
    if (legacySingle != 0xFF) {
      oldMask |= legacySingle;
      haveOld = true;
    }
    if (legacyNormal != 0xFF) {
      oldMask |= legacyNormal;
      haveOld = true;
    }
    if (legacySport != 0xFF) {
      oldMask |= legacySport;
      haveOld = true;
    }
    if (haveOld) pageMask = migrateOldPageMask(oldMask);
  }

  gaugeConfig.pageMask = (pageMask ? pageMask : DEFAULT_PAGE_MASK) | (1u << GAUGE_OVERVIEW);
  gaugeConfig.brightness = constrain(storedBrightness, (uint8_t)40, (uint8_t)255);
}

static bool saveConfig(uint8_t pageMask, uint8_t brightness) {
  pageMask &= VALID_PAGE_MASK;
  pageMask |= (1u << GAUGE_OVERVIEW);  // Home is always available for RIGHT-short.

  if (!preferences.begin("ngauge", false)) return false;
  preferences.putUChar("pages3", pageMask);
  preferences.putUChar("bright", constrain(brightness, (uint8_t)40, (uint8_t)255));
  preferences.end();

  gaugeConfig.pageMask = pageMask;
  gaugeConfig.brightness = constrain(brightness, (uint8_t)40, (uint8_t)255);
  return true;
}

static void selectFirstEnabledPage() {
  if (isPageEnabled(currentGauge)) return;
  for (uint8_t i = 0; i < GAUGE_COUNT; ++i) {
    const GaugeType candidate = (GaugeType)i;
    if (isPageEnabled(candidate)) {
      currentGauge = candidate;
      return;
    }
  }
  gaugeConfig.pageMask = DEFAULT_PAGE_MASK;
  currentGauge = GAUGE_OVERVIEW;
}

// ----------------------------------------------------------------------------
// Sensor cache - fixed point only in the hot path
// ----------------------------------------------------------------------------
struct SensorCache {
  uint16_t rpm = 0;
  int16_t iatF10 = 0;       // degrees F * 10
  int16_t ambientF10 = 0;   // degrees F * 10
  uint8_t mapKpa = 0;       // absolute kPa
  uint8_t baroKpa = 0;      // absolute kPa
  int16_t coolantF10 = 0;   // degrees F * 10
  int16_t sparkHalfDeg = 0; // degrees * 2 (1 = 0.5 degree)
  uint16_t batteryMv = 0;

  bool rpmValid = false;
  bool iatValid = false;
  bool ambientValid = false;
  bool mapValid = false;
  bool baroValid = false;
  bool coolantValid = false;
  bool sparkValid = false;
  bool batteryValid = false;
};

static SensorCache sensors;

// Highest boost seen since the current OBD session began (psi * 10). A turbo
// gauge's peak is the number you actually want after a pull; the instantaneous
// value has already fallen back to vacuum by the time you glance down. Reset on
// every (re)connect so each drive starts fresh.
static int16_t boostPeakPsi10 = 0;

// Dirty bits: data acquisition only marks these. UI consumes them later.
enum UiDirty : uint16_t {
  DIRTY_RPM      = 1u << 0,
  DIRTY_IAT      = 1u << 1,
  DIRTY_AMBIENT  = 1u << 2,
  DIRTY_MAP      = 1u << 3,
  DIRTY_BARO     = 1u << 4,
  DIRTY_COOLANT  = 1u << 5,
  DIRTY_SPARK    = 1u << 6,
  DIRTY_BATTERY  = 1u << 7,
  DIRTY_PAGE     = 1u << 8,
  DIRTY_ALL      = 0x01FFu
};

static uint16_t uiDirty = DIRTY_ALL;

// ----------------------------------------------------------------------------
// Tiny formatting / math helpers
// ----------------------------------------------------------------------------
static inline int16_t rawTempToF10(uint8_t raw) {
  // C = raw - 40.  F*10 = C*18 + 320.
  return (int16_t)(((int16_t)raw - 40) * 18 + 320);
}

static inline int16_t roundedF(int16_t f10) {
  return (f10 >= 0) ? (int16_t)((f10 + 5) / 10)
                    : (int16_t)((f10 - 5) / 10);
}

static inline int16_t boostPsi10() {
  if (!sensors.mapValid || !sensors.baroValid) return 0;
  const int16_t deltaKpa = (int16_t)sensors.mapKpa - (int16_t)sensors.baroKpa;
  // 1 kPa = 0.1450377 psi => psi*10 ~= kPa * 1.450377.
  const int32_t scaled = (int32_t)deltaKpa * 145;
  const int16_t psi10 = (int16_t)((scaled >= 0) ? ((scaled + 50) / 100)
                                                : ((scaled - 50) / 100));
  // Track the session peak wherever boost is computed (boost page + overview).
  if (psi10 > boostPeakPsi10) boostPeakPsi10 = psi10;
  return psi10;
}

static inline uint8_t redOf(uint32_t c)   { return (uint8_t)((c >> 16) & 0xFF); }
static inline uint8_t greenOf(uint32_t c) { return (uint8_t)((c >> 8) & 0xFF); }
static inline uint8_t blueOf(uint32_t c)  { return (uint8_t)(c & 0xFF); }

static uint32_t lerpColorInt(uint32_t c1, uint32_t c2, int32_t num, int32_t den) {
  if (num <= 0) return c1;
  if (num >= den) return c2;

  const int32_t r = redOf(c1)   + ((int32_t)redOf(c2)   - redOf(c1))   * num / den;
  const int32_t g = greenOf(c1) + ((int32_t)greenOf(c2) - greenOf(c1)) * num / den;
  const int32_t b = blueOf(c1)  + ((int32_t)blueOf(c2)  - blueOf(c1))  * num / den;
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

static uint32_t colorForRPM(uint16_t rpm) {
  const uint32_t WHITE = 0xFFFFFF, ORANGE = 0xFF7800, RED = 0xFF2020;
  if (rpm <= 4500) return WHITE;
  if (rpm >= 6500) return RED;
  if (rpm < 5500) return lerpColorInt(WHITE, ORANGE, rpm - 4500, 1000);
  return lerpColorInt(ORANGE, RED, rpm - 5500, 1000);
}

static uint32_t colorForIAT(int16_t f10) {
  const uint32_t WHITE = 0xFFFFFF, YELLOW = 0xFFD000, ORANGE = 0xFF7800, RED = 0xFF2020;
  if (f10 <= 1100) return WHITE;
  if (f10 >= 1700) return RED;
  if (f10 < 1300) return lerpColorInt(WHITE, YELLOW, f10 - 1100, 200);
  if (f10 < 1500) return lerpColorInt(YELLOW, ORANGE, f10 - 1300, 200);
  return lerpColorInt(ORANGE, RED, f10 - 1500, 200);
}

static uint32_t colorForIATDelta(int16_t f10) {
  // Street-performance heat-soak scale:
  //   0-10F excellent, 11-20F good, 21-30F warm,
  //   31-40F hot, 41F+ heavily heat soaked.
  // Keep ~20F visually calm; make ~30F clearly warm and 40F unmistakably hot.
  const uint32_t GREEN = 0x40FF40, WHITE = 0xFFFFFF;
  const uint32_t YELLOW = 0xFFD000, ORANGE = 0xFF7800, RED = 0xFF2020;
  if (f10 <= 100) return GREEN;
  if (f10 < 200) return lerpColorInt(GREEN, WHITE, f10 - 100, 100);
  if (f10 < 300) return lerpColorInt(WHITE, YELLOW, f10 - 200, 100);
  if (f10 < 350) return lerpColorInt(YELLOW, ORANGE, f10 - 300, 50);
  if (f10 < 400) return lerpColorInt(ORANGE, RED, f10 - 350, 50);
  return RED;
}

static uint32_t colorForCoolant(int16_t f10) {
  const uint32_t WHITE = 0xFFFFFF, YELLOW = 0xFFD000, ORANGE = 0xFF7800, RED = 0xFF2020;
  if (f10 <= 2200) return WHITE;
  if (f10 >= 2400) return RED;
  if (f10 < 2300) return lerpColorInt(WHITE, YELLOW, f10 - 2200, 100);
  return lerpColorInt(YELLOW, ORANGE, f10 - 2300, 100);
}

static uint32_t colorForBattery(uint16_t mv) {
  const uint32_t RED = 0xFF2020, YELLOW = 0xFFD000, WHITE = 0xFFFFFF, GREEN = 0x40FF40;
  if (mv < 11500) return RED;
  if (mv < 12400) return lerpColorInt(RED, YELLOW, mv - 11500, 900);
  if (mv < 13000) return lerpColorInt(YELLOW, WHITE, mv - 12400, 600);
  if (mv < 13800) return WHITE;
  if (mv < 14500) return lerpColorInt(WHITE, GREEN, mv - 13800, 700);
  if (mv < 15000) return GREEN;
  if (mv >= 16000) return RED;
  return lerpColorInt(GREEN, RED, mv - 15000, 1000);
}



static const char* statusForCoolant(int16_t f10) {
  if (f10 < 1600) return "WARMING";
  if (f10 <= 2200) return "NORMAL";
  if (f10 <= 2300) return "WARM";
  if (f10 <= 2400) return "HOT";
  return "VERY HOT";
}

static const char* statusForBattery(uint16_t mv) {
  if (mv < 11500) return "CRITICAL";
  if (mv < 12400) return "LOW";
  if (mv < 13000) return "RESTING";
  if (mv < 13800) return "OK";
  if (mv < 14500) return "CHARGING";
  if (mv < 15000) return "STRONG";
  return "OVERVOLT";
}

static const char* statusForSpark(int16_t halfDeg) {
  // PID 010E is commanded ignition timing relative to TDC. Positive means
  // advance, negative means after-TDC/retard. This is NOT a knock-retard PID.
  if (halfDeg > 0) return "ADVANCE";
  if (halfDeg < 0) return "RETARD";
  return "TDC";
}

// ----------------------------------------------------------------------------
// Display driver
// ----------------------------------------------------------------------------
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  const uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
  const uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

  // The panel is physically mounted 270 degrees from LVGL's logical layout.
  // LVGL 8.4's built-in software rotation breaks partial refreshes into a small
  // temporary rotation buffer. On this CO5300/QSPI combination that produced
  // occasional torn/jumbled dynamic labels. Rotate each invalidated rectangle
  // ourselves into one persistent PSRAM scratch buffer instead. There are no
  // temporary allocations, no split rotation flushes, and the panel receives
  // one contiguous rectangle for each LVGL invalidation.
  //
  // Logical -> panel mapping for the previous LV_DISP_ROT_270 orientation:
  //   panelX = LCD_WIDTH - 1 - logicalY
  //   panelY = logicalX
  // A logical w x h rectangle therefore becomes h x w on the panel.
  const int16_t outX = (int16_t)(LCD_WIDTH - 1 - area->y2);
  const int16_t outY = area->x1;

  for (uint32_t y = 0; y < h; ++y) {
    const lv_color_t *src = color_p + (y * w);
    const uint32_t dstX = h - 1u - y;
    for (uint32_t x = 0; x < w; ++x) {
      rotateBuffer[(x * h) + dstX] = src[x].full;
    }
  }

  // Arduino_GFX's ESP32 QSPI writePixels() path is synchronous (polling SPI),
  // so the scratch buffer is no longer in use when this call returns.
  gfx->draw16bitRGBBitmap(outX, outY, rotateBuffer, (int16_t)h, (int16_t)w);
  lv_disp_flush_ready(disp);
}

static void configureFixedLabel(lv_obj_t *label, lv_coord_t width, lv_text_align_t align) {
  lv_obj_set_width(label, width);
  lv_obj_set_style_text_align(label, align, 0);
}

static void setDynamicText(lv_obj_t *label, char *persistentBuffer) {
  // Invalidate before changing the backing memory. All dynamic labels have a
  // fixed width, so old and new glyphs redraw inside a stable rectangle.
  lv_obj_invalidate(label);
  lv_label_set_text_static(label, persistentBuffer);
}

static void buildBootScreen() {
  scr_boot = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(scr_boot, lv_color_black(), 0);
  lv_obj_clear_flag(scr_boot, LV_OBJ_FLAG_SCROLLABLE);

  img_boot_logo = lv_img_create(scr_boot);
  lv_img_set_src(img_boot_logo, &hyundai_n_lvgl);
  lv_img_set_zoom(img_boot_logo, 256);
  lv_img_set_antialias(img_boot_logo, true);
  lv_obj_center(img_boot_logo);
}

static void buildConfigScreen() {
  scr_config = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(scr_config, lv_color_black(), 0);
  lv_obj_clear_flag(scr_config, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(scr_config);
  lv_label_set_text_static(title, "SETUP / UPDATE");
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_align(title, LV_ALIGN_CENTER, 0, -108);

  lv_obj_t *wifi = lv_label_create(scr_config);
  configureFixedLabel(wifi, 400, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(wifi, "Wi-Fi: ElantraN-Gauge-Setup");
  lv_obj_set_style_text_color(wifi, lv_color_white(), 0);
  lv_obj_set_style_text_font(wifi, &lv_font_montserrat_18, 0);
  lv_obj_align(wifi, LV_ALIGN_CENTER, 0, -55);

  label_config_password = lv_label_create(scr_config);
  configureFixedLabel(label_config_password, 400, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_fmt(label_config_password, "Password: %s", configApPassword);
  lv_obj_set_style_text_color(label_config_password, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_config_password, &lv_font_montserrat_18, 0);
  lv_obj_align(label_config_password, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *ip = lv_label_create(scr_config);
  configureFixedLabel(ip, 390, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(ip, "Open 192.168.4.1");
  lv_obj_set_style_text_color(ip, lv_color_white(), 0);
  lv_obj_set_style_text_font(ip, &lv_font_montserrat_22, 0);
  lv_obj_align(ip, LV_ALIGN_CENTER, 0, 25);

  lv_obj_t *hint = lv_label_create(scr_config);
  configureFixedLabel(hint, 390, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(hint, "Pages + brightness + firmware update\nRIGHT returns to gauge");
  lv_obj_set_style_text_color(hint, lv_color_white(), 0);
  lv_obj_set_style_text_font(hint, &lv_font_montserrat_18, 0);
  lv_obj_align(hint, LV_ALIGN_CENTER, 0, 83);
}

static void buildSingleGaugeScreen() {
  scr_single = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(scr_single, lv_color_black(), 0);
  lv_obj_clear_flag(scr_single, LV_OBJ_FLAG_SCROLLABLE);

  arc_rpm = lv_arc_create(scr_single);
  lv_obj_set_size(arc_rpm, 460, 460);
  lv_obj_center(arc_rpm);
  lv_arc_set_range(arc_rpm, 500, 7000);
  lv_arc_set_value(arc_rpm, 500);
  lv_arc_set_bg_angles(arc_rpm, 130, 50);
  lv_arc_set_rotation(arc_rpm, 0);
  lv_obj_remove_style(arc_rpm, nullptr, LV_PART_KNOB);
  lv_obj_clear_flag(arc_rpm, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(arc_rpm, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc_rpm, 14, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc_rpm, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc_rpm, 14, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(arc_rpm, true, LV_PART_INDICATOR);

  img_n_badge = lv_img_create(scr_single);
  lv_img_set_src(img_n_badge, &hyundai_n_lvgl);
  lv_img_set_zoom(img_n_badge, 205);
  lv_img_set_antialias(img_n_badge, true);
  lv_obj_align(img_n_badge, LV_ALIGN_TOP_MID, 0, 23);

  label_main_unit = lv_label_create(scr_single);
  configureFixedLabel(label_main_unit, 330, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(label_main_unit, "IAT");
  lv_obj_set_style_text_color(label_main_unit, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_main_unit, &lv_font_montserrat_22, 0);
  lv_obj_align(label_main_unit, LV_ALIGN_CENTER, 0, -72);

  label_main_value = lv_label_create(scr_single);
  configureFixedLabel(label_main_value, 360, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(label_main_value, mainValueText);
  lv_obj_set_style_text_color(label_main_value, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_main_value, &lv_font_montserrat_48, 0);
  lv_label_set_long_mode(label_main_value, LV_LABEL_LONG_CLIP);
  lv_obj_align(label_main_value, LV_ALIGN_CENTER, 0, -20);

  iat_divider = lv_obj_create(scr_single);
  lv_obj_set_size(iat_divider, 235, 2);
  lv_obj_set_style_bg_color(iat_divider, lv_color_hex(0x484848), 0);
  lv_obj_set_style_bg_opa(iat_divider, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(iat_divider, 0, 0);
  lv_obj_set_style_radius(iat_divider, 0, 0);
  lv_obj_clear_flag(iat_divider, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(iat_divider, LV_ALIGN_CENTER, 0, 25);
  lv_obj_add_flag(iat_divider, LV_OBJ_FLAG_HIDDEN);

  label_status = lv_label_create(scr_single);
  configureFixedLabel(label_status, 350, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(label_status, "");
  lv_obj_set_style_text_color(label_status, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_status, &lv_font_montserrat_28, 0);
  lv_obj_align(label_status, LV_ALIGN_CENTER, 0, 52);

  label_extra = lv_label_create(scr_single);
  configureFixedLabel(label_extra, 350, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(label_extra, "");
  lv_obj_set_style_text_color(label_extra, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_extra, &lv_font_montserrat_24, 0);
  lv_obj_align(label_extra, LV_ALIGN_CENTER, 0, 84);

  label_rpm_bottom = lv_label_create(scr_single);
  configureFixedLabel(label_rpm_bottom, 250, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(label_rpm_bottom, singleRpmText);
  lv_obj_set_style_text_color(label_rpm_bottom, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_rpm_bottom, &lv_font_montserrat_28, 0);
  lv_obj_align(label_rpm_bottom, LV_ALIGN_BOTTOM_MID, 0, -45);
}

static void buildOverviewScreen() {
  scr_overview = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(scr_overview, lv_color_black(), 0);
  lv_obj_clear_flag(scr_overview, LV_OBJ_FLAG_SCROLLABLE);

  ov_arc_rpm = lv_arc_create(scr_overview);
  lv_obj_set_size(ov_arc_rpm, 460, 460);
  lv_obj_center(ov_arc_rpm);
  lv_arc_set_range(ov_arc_rpm, 500, 7000);
  lv_arc_set_value(ov_arc_rpm, 500);
  lv_arc_set_bg_angles(ov_arc_rpm, 130, 50);
  lv_arc_set_rotation(ov_arc_rpm, 0);
  lv_obj_remove_style(ov_arc_rpm, nullptr, LV_PART_KNOB);
  lv_obj_clear_flag(ov_arc_rpm, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_color(ov_arc_rpm, lv_color_hex(0x202020), LV_PART_MAIN);
  lv_obj_set_style_arc_width(ov_arc_rpm, 14, LV_PART_MAIN);
  lv_obj_set_style_arc_color(ov_arc_rpm, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(ov_arc_rpm, 14, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(ov_arc_rpm, true, LV_PART_INDICATOR);

  img_n_watermark = lv_img_create(scr_overview);
  lv_img_set_src(img_n_watermark, &hyundai_n_lvgl);
  lv_img_set_zoom(img_n_watermark, 190);
  lv_img_set_antialias(img_n_watermark, true);
  lv_obj_align(img_n_watermark, LV_ALIGN_TOP_MID, 0, 18);

  static const char* const labels[6] = {"BOOST", "IAT", "IAT DELTA", "COOLANT", "IGN", "BATT"};
  static const int colX[2] = {-92, 92};
  static const int rowY[3] = {-73, 2, 77};

  for (uint8_t i = 0; i < 6; ++i) {
    const uint8_t col = i & 1u;
    const uint8_t row = i >> 1;
    ov_rows[i][0] = lv_label_create(scr_overview);
    configureFixedLabel(ov_rows[i][0], 165, LV_TEXT_ALIGN_CENTER);
    lv_label_set_text_static(ov_rows[i][0], labels[i]);
    lv_obj_set_style_text_color(ov_rows[i][0], lv_color_hex(0xB8B8B8), 0);
    lv_obj_set_style_text_font(ov_rows[i][0], &lv_font_montserrat_18, 0);
    lv_obj_align(ov_rows[i][0], LV_ALIGN_CENTER, colX[col], rowY[row] - 18);

    ov_rows[i][1] = lv_label_create(scr_overview);
    configureFixedLabel(ov_rows[i][1], 175, LV_TEXT_ALIGN_CENTER);
    lv_label_set_text_static(ov_rows[i][1], overviewValueText[i]);
    lv_obj_set_style_text_color(ov_rows[i][1], lv_color_white(), 0);
    lv_obj_set_style_text_font(ov_rows[i][1], &lv_font_montserrat_32, 0);
    lv_obj_align(ov_rows[i][1], LV_ALIGN_CENTER, colX[col], rowY[row] + 13);
  }

  ov_rpm_label = lv_label_create(scr_overview);
  configureFixedLabel(ov_rpm_label, 250, LV_TEXT_ALIGN_CENTER);
  lv_label_set_text_static(ov_rpm_label, overviewRpmText);
  lv_obj_set_style_text_color(ov_rpm_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(ov_rpm_label, &lv_font_montserrat_28, 0);
  lv_obj_align(ov_rpm_label, LV_ALIGN_BOTTOM_MID, 0, -45);
}

// ----------------------------------------------------------------------------
// BLE / ELM receive path - fixed buffer, cross-core protected
// ----------------------------------------------------------------------------
static NimBLEClient* obdClient = nullptr;
static NimBLERemoteCharacteristic* obdWrite = nullptr;
static NimBLERemoteCharacteristic* obdNotify = nullptr;
static bool obdConnected = false;
static uint32_t lastConnectAttempt = 0;
static const uint32_t RECONNECT_INTERVAL_MS = 5000;

static constexpr size_t RX_CAPACITY = 96;
static char rxBuffer[RX_CAPACITY];
static uint8_t rxLength = 0;
static bool rxReady = false;
static bool rxOverflow = false;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

static void resetRx() {
  portENTER_CRITICAL(&rxMux);
  rxLength = 0;
  rxReady = false;
  rxOverflow = false;
  rxBuffer[0] = '\0';
  portEXIT_CRITICAL(&rxMux);
}

static void obdNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  (void)pChar;
  (void)isNotify;

  portENTER_CRITICAL(&rxMux);

  // Once a prompt has completed this transaction, ignore any trailing bytes
  // until the main loop consumes it and intentionally starts the next request.
  if (!rxReady) {
    for (size_t i = 0; i < length; i++) {
      const char c = (char)data[i];

      if (c == '>') {
        rxBuffer[rxLength] = '\0';
        rxReady = true;
        break;
      }

      // CR/LF are unnecessary because ATS0/ATL0 are enabled. Keep printable
      // text so failures like "NO DATA" can still be recognized/rejected.
      if (c >= 32 && c < 127) {
        if (rxLength < RX_CAPACITY - 1) {
          rxBuffer[rxLength++] = c;
          rxBuffer[rxLength] = '\0';
        } else {
          rxOverflow = true;
        }
      }
    }
  }

  portEXIT_CRITICAL(&rxMux);
}

static bool takeResponse(char *out, size_t outCapacity, bool &overflowed) {
  bool haveResponse = false;

  portENTER_CRITICAL(&rxMux);
  if (rxReady) {
    const size_t copyLen = (rxLength < outCapacity - 1) ? rxLength : (outCapacity - 1);
    memcpy(out, rxBuffer, copyLen);
    out[copyLen] = '\0';
    overflowed = rxOverflow || (rxLength >= outCapacity);

    rxLength = 0;
    rxReady = false;
    rxOverflow = false;
    rxBuffer[0] = '\0';
    haveResponse = true;
  }
  portEXIT_CRITICAL(&rxMux);

  return haveResponse;
}

static bool obdSendBlocking(const char *cmd, size_t len, uint32_t timeoutMs) {
  if (!obdWrite) return false;

  resetRx();
  obdWrite->writeValue((uint8_t*)cmd, len, false);

  const uint32_t start = millis();
  char response[RX_CAPACITY];
  bool overflowed = false;

  while ((uint32_t)(millis() - start) < timeoutMs) {
    if (takeResponse(response, sizeof(response), overflowed)) return !overflowed;
    delay(1);
  }
  return false;
}

#define SEND_INIT(cmd, timeout) obdSendBlocking(cmd, sizeof(cmd) - 1, timeout)

// ----------------------------------------------------------------------------
// Strict Mode 01 parser
// ----------------------------------------------------------------------------
static inline int8_t hexNibble(char c) {
  if (c >= '0' && c <= '9') return (int8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
  return -1;
}

static bool parseMode01Value(const char *response, uint8_t pid, uint8_t dataBytes, uint16_t &value) {
  if (!response || dataBytes == 0 || dataBytes > 2) return false;

  const size_t len = strlen(response);
  const size_t required = 4u + (size_t)dataBytes * 2u; // 41 + PID + data
  if (len < required) return false;

  const uint8_t pidHi = (uint8_t)(pid >> 4);
  const uint8_t pidLo = (uint8_t)(pid & 0x0F);

  for (size_t i = 0; i + required <= len; i++) {
    if (response[i] != '4' || response[i + 1] != '1') continue;
    if (hexNibble(response[i + 2]) != pidHi || hexNibble(response[i + 3]) != pidLo) continue;

    uint16_t parsed = 0;
    bool valid = true;
    size_t p = i + 4;

    for (uint8_t b = 0; b < dataBytes; b++) {
      const int8_t hi = hexNibble(response[p++]);
      const int8_t lo = hexNibble(response[p++]);
      if (hi < 0 || lo < 0) {
        valid = false;
        break;
      }
      parsed = (uint16_t)((parsed << 8) | (uint8_t)((hi << 4) | lo));
    }

    if (valid) {
      value = parsed;
      return true;
    }
  }

  return false;
}

// ----------------------------------------------------------------------------
// Asynchronous OBD request scheduler
// ----------------------------------------------------------------------------
enum ObdRequest : uint8_t {
  REQ_RPM = 0,
  REQ_MAP,
  REQ_IAT,
  REQ_SPARK,
  REQ_COOLANT,
  REQ_BATTERY,
  REQ_AMBIENT,
  REQ_BARO,
  REQ_COUNT,
  REQ_NONE = 0xFF
};

static bool requestNeeded(ObdRequest req) {
  // Home permanently displays all six metrics. Detail-page toggles only affect
  // LEFT-button navigation, not the Home data feed.
  switch (req) {
    case REQ_RPM:
    case REQ_MAP:
    case REQ_BARO:
    case REQ_IAT:
    case REQ_AMBIENT:
    case REQ_COOLANT:
    case REQ_SPARK:
    case REQ_BATTERY:
      return true;
    default:
      return false;
  }
}

static const char CMD_RPM[]     = "010C\r";
static const char CMD_MAP[]     = "010B\r";
static const char CMD_IAT[]     = "010F\r";
static const char CMD_SPARK[]   = "010E\r";
static const char CMD_COOLANT[] = "0105\r";
static const char CMD_BATTERY[] = "0142\r";
static const char CMD_AMBIENT[] = "0146\r";
static const char CMD_BARO[]    = "0133\r";

static bool requestInFlight = false;
static ObdRequest inFlightRequest = REQ_NONE;
static ObdRequest lastCompletedRequest = REQ_NONE;
static uint32_t requestStartedMs = 0;
static uint32_t lastRequestMs[REQ_COUNT] = {0};
static const uint32_t REQUEST_TIMEOUT_MS = 700;

static bool due(ObdRequest req, uint32_t now, uint32_t intervalMs) {
  return (uint32_t)(now - lastRequestMs[req]) >= intervalMs;
}

static void makeDue(ObdRequest req, uint32_t now) {
  lastRequestMs[req] = now - 60000u;
}

static ObdRequest chooseVisibleSecondary(uint32_t now) {
  switch (currentGauge) {
    case GAUGE_OVERVIEW:
      if (!sensors.baroValid) return REQ_BARO;
      if (!sensors.mapValid) return REQ_MAP;
      if (!sensors.iatValid) return REQ_IAT;
      if (!sensors.ambientValid) return REQ_AMBIENT;
      if (!sensors.coolantValid) return REQ_COOLANT;
      if (!sensors.sparkValid) return REQ_SPARK;
      if (!sensors.batteryValid) return REQ_BATTERY;
      if (due(REQ_MAP, now, 120)) return REQ_MAP;
      if (due(REQ_IAT, now, 500)) return REQ_IAT;
      if (due(REQ_SPARK, now, 350)) return REQ_SPARK;
      if (due(REQ_COOLANT, now, 1000)) return REQ_COOLANT;
      if (due(REQ_BATTERY, now, 1500)) return REQ_BATTERY;
      if (due(REQ_AMBIENT, now, 4000)) return REQ_AMBIENT;
      if (due(REQ_BARO, now, 10000)) return REQ_BARO;
      return REQ_NONE;

    case GAUGE_BOOST:
      if (!sensors.baroValid || due(REQ_BARO, now, 10000)) return REQ_BARO;
      return REQ_MAP;

    case GAUGE_IAT:
      if (!sensors.iatValid || due(REQ_IAT, now, 250)) return REQ_IAT;
      if (!sensors.ambientValid || due(REQ_AMBIENT, now, 1500)) return REQ_AMBIENT;
      return REQ_NONE;

    case GAUGE_COOLANT:
      if (!sensors.coolantValid || due(REQ_COOLANT, now, 500)) return REQ_COOLANT;
      return REQ_NONE;

    case GAUGE_SPARK:
      if (!sensors.sparkValid || due(REQ_SPARK, now, 100)) return REQ_SPARK;
      return REQ_NONE;

    case GAUGE_BATTERY:
      if (!sensors.batteryValid || due(REQ_BATTERY, now, 500)) return REQ_BATTERY;
      return REQ_NONE;

    default:
      return REQ_NONE;
  }
}

static ObdRequest chooseBackgroundSecondary(uint32_t now) {
  // Home permanently needs every displayed metric, so all Home-supporting
  // PIDs remain available in the background even when a detail page is hidden.
  if (requestNeeded(REQ_BARO) && (!sensors.baroValid || due(REQ_BARO, now, 10000))) return REQ_BARO;
  if (requestNeeded(REQ_IAT) && (!sensors.iatValid || due(REQ_IAT, now, 2000))) return REQ_IAT;
  if (requestNeeded(REQ_COOLANT) && (!sensors.coolantValid || due(REQ_COOLANT, now, 3000))) return REQ_COOLANT;
  if (requestNeeded(REQ_BATTERY) && (!sensors.batteryValid || due(REQ_BATTERY, now, 3000))) return REQ_BATTERY;
  if (requestNeeded(REQ_AMBIENT) && (!sensors.ambientValid || due(REQ_AMBIENT, now, 5000))) return REQ_AMBIENT;
  if (requestNeeded(REQ_SPARK) && (!sensors.sparkValid || due(REQ_SPARK, now, 2000))) return REQ_SPARK;

  if (requestNeeded(REQ_MAP) &&
      currentGauge != GAUGE_BOOST && currentGauge != GAUGE_OVERVIEW &&
      (!sensors.mapValid || due(REQ_MAP, now, 1500))) return REQ_MAP;

  return REQ_NONE;
}

static ObdRequest chooseNextRequest(uint32_t now) {
  if (!sensors.rpmValid) return REQ_RPM;

  // Never send two non-RPM data requests back-to-back. This guarantees RPM at
  // least half of all transactions and often much more on slow-changing pages.
  if (lastCompletedRequest != REQ_RPM) return REQ_RPM;

  const ObdRequest visible = chooseVisibleSecondary(now);
  if (visible != REQ_NONE) return visible;

  const ObdRequest background = chooseBackgroundSecondary(now);
  if (background != REQ_NONE) return background;

  return REQ_RPM;
}

static bool startRequest(ObdRequest req) {
  if (!obdConnected || !obdWrite || requestInFlight || req == REQ_NONE) return false;

  const char *cmd = nullptr;
  size_t len = 0;

  switch (req) {
    case REQ_RPM:     cmd = CMD_RPM;     len = sizeof(CMD_RPM) - 1; break;
    case REQ_MAP:     cmd = CMD_MAP;     len = sizeof(CMD_MAP) - 1; break;
    case REQ_IAT:     cmd = CMD_IAT;     len = sizeof(CMD_IAT) - 1; break;
    case REQ_SPARK:   cmd = CMD_SPARK;   len = sizeof(CMD_SPARK) - 1; break;
    case REQ_COOLANT: cmd = CMD_COOLANT; len = sizeof(CMD_COOLANT) - 1; break;
    case REQ_BATTERY: cmd = CMD_BATTERY; len = sizeof(CMD_BATTERY) - 1; break;
    case REQ_AMBIENT: cmd = CMD_AMBIENT; len = sizeof(CMD_AMBIENT) - 1; break;
    case REQ_BARO:    cmd = CMD_BARO;    len = sizeof(CMD_BARO) - 1; break;
    default: return false;
  }

  resetRx();
  inFlightRequest = req;
  requestStartedMs = millis();
  requestInFlight = true;
  obdWrite->writeValue((uint8_t*)cmd, len, false);
  return true;
}

static void updateRpm(uint16_t rpm) {
  if (rpm > 8000) return;
  const bool validityChanged = !sensors.rpmValid;
  if (validityChanged || sensors.rpm != rpm) {
    sensors.rpm = rpm;
    sensors.rpmValid = true;
    uiDirty |= DIRTY_RPM;
  }
}

static void updateMap(uint8_t kpa) {
  if (kpa == 0) return;
  const bool validityChanged = !sensors.mapValid;
  if (validityChanged || sensors.mapKpa != kpa) {
    sensors.mapKpa = kpa;
    sensors.mapValid = true;
    uiDirty |= DIRTY_MAP;
  }
}

static void updateBaro(uint8_t kpa) {
  if (kpa < 30 || kpa > 110) return;
  const bool validityChanged = !sensors.baroValid;
  if (validityChanged || sensors.baroKpa != kpa) {
    sensors.baroKpa = kpa;
    sensors.baroValid = true;
    uiDirty |= DIRTY_BARO;
  }
}

static void updateIat(uint8_t raw) {
  const int16_t f10 = rawTempToF10(raw);
  if (f10 < -400 || f10 > 2500) return;
  const bool validityChanged = !sensors.iatValid;
  if (validityChanged || sensors.iatF10 != f10) {
    sensors.iatF10 = f10;
    sensors.iatValid = true;
    uiDirty |= DIRTY_IAT;
  }
}

static void updateAmbient(uint8_t raw) {
  const int16_t f10 = rawTempToF10(raw);
  if (f10 < -400 || f10 > 1400) return;
  const bool validityChanged = !sensors.ambientValid;
  if (validityChanged || sensors.ambientF10 != f10) {
    sensors.ambientF10 = f10;
    sensors.ambientValid = true;
    uiDirty |= DIRTY_AMBIENT;
  }
}

static void updateCoolant(uint8_t raw) {
  const int16_t f10 = rawTempToF10(raw);
  if (f10 < -400 || f10 > 3000) return;
  const bool validityChanged = !sensors.coolantValid;
  if (validityChanged || sensors.coolantF10 != f10) {
    sensors.coolantF10 = f10;
    sensors.coolantValid = true;
    uiDirty |= DIRTY_COOLANT;
  }
}

static void updateSpark(uint8_t raw) {
  // Standard PID 010E: degrees = A/2 - 64.
  const int16_t halfDeg = (int16_t)raw - 128;
  const bool validityChanged = !sensors.sparkValid;
  if (validityChanged || sensors.sparkHalfDeg != halfDeg) {
    sensors.sparkHalfDeg = halfDeg;
    sensors.sparkValid = true;
    uiDirty |= DIRTY_SPARK;
  }
}

static void updateBattery(uint16_t mv) {
  if (mv < 8000 || mv > 18000) return;
  const bool validityChanged = !sensors.batteryValid;
  if (validityChanged || sensors.batteryMv != mv) {
    sensors.batteryMv = mv;
    sensors.batteryValid = true;
    uiDirty |= DIRTY_BATTERY;
  }
}

static void markDisconnected();

static void processResponse(ObdRequest req, const char *response, bool overflowed) {
  if (overflowed) return;

  uint16_t value = 0;

  switch (req) {
    case REQ_RPM:
      if (parseMode01Value(response, 0x0C, 2, value)) updateRpm((uint16_t)(value / 4));
      break;

    case REQ_MAP:
      if (parseMode01Value(response, 0x0B, 1, value)) updateMap((uint8_t)value);
      break;

    case REQ_IAT:
      if (parseMode01Value(response, 0x0F, 1, value)) updateIat((uint8_t)value);
      break;

    case REQ_SPARK:
      if (parseMode01Value(response, 0x0E, 1, value)) updateSpark((uint8_t)value);
      break;

    case REQ_COOLANT:
      if (parseMode01Value(response, 0x05, 1, value)) updateCoolant((uint8_t)value);
      break;

    case REQ_BATTERY:
      if (parseMode01Value(response, 0x42, 2, value)) updateBattery(value);
      break;

    case REQ_AMBIENT:
      if (parseMode01Value(response, 0x46, 1, value)) updateAmbient((uint8_t)value);
      break;

    case REQ_BARO:
      if (parseMode01Value(response, 0x33, 1, value)) updateBaro((uint8_t)value);
      break;

    default:
      break;
  }
}

static void serviceOBD(uint32_t now) {
  if (!obdConnected || !obdWrite) return;

  if (requestInFlight) {
    char response[RX_CAPACITY];
    bool overflowed = false;

    if (takeResponse(response, sizeof(response), overflowed)) {
      const ObdRequest completed = inFlightRequest;
      requestInFlight = false;
      inFlightRequest = REQ_NONE;
      lastCompletedRequest = completed;
      lastRequestMs[completed] = now;

      processResponse(completed, response, overflowed);

      // Start the next transaction immediately. The display can render while
      // this request is in flight, overlapping panel work with BLE/ECU latency.
      startRequest(chooseNextRequest(now));
      return;
    }

    if ((uint32_t)(now - requestStartedMs) >= REQUEST_TIMEOUT_MS) {
      // 700 ms is already much longer than the ELM timeout. If the prompt is
      // still missing, do not risk letting a late reply cross into the next
      // PID. Reconnect/reinitialize the adapter and keep the last good values.
      DBG_PRINTLN("[OBD] Request timeout; reconnecting");
      markDisconnected();
    }
    return;
  }

  startRequest(chooseNextRequest(now));
}

static bool tryConnectOBD() {
  DBG_PRINTLN("[OBD] Scanning...");

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  pScan->setInterval(45);
  pScan->setWindow(15);

  NimBLEScanResults results = pScan->start(3, false);
  NimBLEAdvertisedDevice target;
  bool found = false;
  bool foundByService = false;

  const NimBLEUUID obdService(OBD_SERVICE_UUID);

  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice d = results.getDevice(i);

    // Strongest signal: the device advertises the ELM327 0xFFF0 service. Take
    // the first such device immediately.
    if (d.isAdvertisingService(obdService)) {
      target = d;
      found = true;
      foundByService = true;
      break;
    }

    // Weaker fallback: the advertised name looks like an OBD dongle. Remember
    // the first candidate but keep scanning in case a 0xFFF0 device shows up.
    if (!found && d.getName().length()) {
      String name(d.getName().c_str());
      name.toUpperCase();
      for (const char* hint : OBD_NAME_HINTS) {
        if (name.indexOf(hint) >= 0) {
          target = d;
          found = true;
          break;
        }
      }
    }
  }
  pScan->clearResults();

  if (!found) {
    DBG_PRINTLN("[OBD] No BLE OBD adapter found");
    return false;
  }
  DBG_PRINTF("[OBD] Candidate %s (%s)\n",
             target.getAddress().toString().c_str(),
             foundByService ? "by 0xFFF0 service" : "by name");

  DBG_PRINTLN("[OBD] Connecting...");
  if (!obdClient) obdClient = NimBLEDevice::createClient();
  if (!obdClient->connect(&target)) {
    DBG_PRINTLN("[OBD] Connect failed");
    return false;
  }

  NimBLERemoteService* svc = obdClient->getService(NimBLEUUID(OBD_SERVICE_UUID));
  if (!svc) {
    DBG_PRINTLN("[OBD] Missing FFF0 service");
    return false;
  }

  obdWrite = svc->getCharacteristic(NimBLEUUID(OBD_WRITE_UUID));
  obdNotify = svc->getCharacteristic(NimBLEUUID(OBD_NOTIFY_UUID));
  if (!obdWrite || !obdNotify) {
    DBG_PRINTLN("[OBD] Missing FFF1/FFF2 characteristic");
    return false;
  }

  obdNotify->subscribe(true, obdNotifyCallback);
  delay(250);

  static const char ATD[]      = "ATD\r";
  static const char ATE0[]     = "ATE0\r";
  static const char ATL0[]     = "ATL0\r";
  static const char ATS0[]     = "ATS0\r";
  static const char ATH0[]     = "ATH0\r";
  static const char ATSP6[]    = "ATSP6\r";
  static const char ATST32[]   = "ATST32\r";
  static const char ATCAF1[]   = "ATCAF1\r";
  static const char ATSH7DF[]  = "ATSH7DF\r";
  static const char WAKE[]     = "0100\r";

  DBG_PRINTLN("[OBD] Initializing ELM...");
  SEND_INIT(ATD, 600);
  SEND_INIT(ATE0, 300);
  SEND_INIT(ATL0, 300);
  SEND_INIT(ATS0, 300);
  SEND_INIT(ATH0, 300);
  SEND_INIT(ATSP6, 500);
  SEND_INIT(ATST32, 300);
  SEND_INIT(ATCAF1, 300);
  SEND_INIT(ATSH7DF, 300); // Set once; never resend in the polling loop.
  SEND_INIT(WAKE, 2000);

  obdConnected = true;
  requestInFlight = false;
  inFlightRequest = REQ_NONE;
  lastCompletedRequest = REQ_NONE;
  resetRx();

  // Boost is MAP - BARO. Barometric pressure (PID 0133) is not guaranteed on
  // every ECU, and if it never validates the boost page would spin on BARO and
  // sit on "--" forever. Seed a sea-level default so boost always reports; a
  // real 0133 reply overwrites this within a few seconds and corrects for
  // altitude. (~101 kPa vs a true ~95-100 kPa is well under 1 psi of error.)
  if (isPageEnabled(GAUGE_BOOST) && !sensors.baroValid) {
    sensors.baroKpa = 101;
    sensors.baroValid = true;
    uiDirty |= DIRTY_BARO;
  }

  boostPeakPsi10 = 0;  // Fresh peak-hold for this drive.

  const uint32_t now = millis();
  for (uint8_t i = 0; i < REQ_COUNT; i++) makeDue((ObdRequest)i, now);

  DBG_PRINTLN("[OBD] Ready");
  return true;
}

static void markDisconnected() {
  if (!obdConnected) return;

  DBG_PRINTLN("[OBD] Disconnected");
  obdConnected = false;
  if (obdClient && obdClient->isConnected()) obdClient->disconnect();
  requestInFlight = false;
  inFlightRequest = REQ_NONE;
  obdWrite = nullptr;
  obdNotify = nullptr;
  resetRx();

  transitionedToGauge = false;
  lv_scr_load(scr_boot);
}

// ----------------------------------------------------------------------------
// Parked Wi-Fi configuration mode
// ----------------------------------------------------------------------------
static String buildConfigHtml() {
  String html;
  html.reserve(10500);

  char token[9];
  csrfTokenText(token, sizeof(token));

  html += F(
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Elantra N Gauge</title><style>"
    "body{font-family:system-ui,-apple-system,sans-serif;background:#0b0b0d;color:#fff;margin:0;padding:20px}"
    ".wrap{max-width:620px;margin:auto}.card{background:#17171b;border:1px solid #303038;border-radius:18px;padding:20px}"
    "h1{font-size:24px;margin:0 0 6px}h2{font-size:19px;margin-top:26px}.muted{color:#e8e8ed;line-height:1.45}"
    ".row{display:flex;align-items:center;justify-content:space-between;padding:12px 0;border-bottom:1px solid #29292f}"
    ".row:last-child{border-bottom:0}label{font-size:18px}.switch{width:24px;height:24px}input[type=range]{width:100%}"
    ".bright{display:flex;gap:12px;align-items:center}.bright output{min-width:42px;text-align:right}"
    ".file{width:100%;box-sizing:border-box;padding:12px;border:1px solid #404047;border-radius:10px;background:#101014;color:#fff}"
    "button{width:100%;margin-top:20px;padding:15px;border:0;border-radius:12px;background:#fff;color:#000;"
    "font-size:17px;font-weight:700}.ota{background:#202027;margin-top:26px;padding:16px;border-radius:14px}"
    ".note{font-size:14px;color:#e8e8ed;margin-top:14px}.warn{color:#ffd37a}.ok{color:#7dff95}"
    "progress{width:100%;height:18px;margin-top:12px}</style></head><body>"
    "<div class='wrap'><div class='card'><h1>Elantra N Gauge</h1>"
  );

  html += F("<p class='muted'>Firmware <b>");
  html += GAUGE_FW_VERSION;
  html += F("</b><br><b>LEFT / PWR:</b> next page &nbsp;•&nbsp; "
            "<b>RIGHT / BOOT:</b> Home &nbsp;•&nbsp; "
            "<b>Hold RIGHT:</b> setup / phone update.</p>"
            "<p class='muted'>Drive-mode switching is intentionally not performed by the gauge; the app owns that behavior. "
            "Home is always available. Disabled detail pages are skipped when LEFT cycles.</p>"
            "<form method='post' action='/save'><h2>Pages</h2>");

  html += F("<input type='hidden' name='token' value='");
  html += token;
  html += F("'>");

  html += F("<div class='row'><label>Home / Overview</label><span class='ok'>Always on</span></div>");
  for (uint8_t i = 1; i < GAUGE_COUNT; ++i) {
    html += F("<div class='row'><label for='p");
    html += i;
    html += F("'>");
    html += GAUGE_PAGE_NAMES[i];
    html += F("</label><input class='switch' type='checkbox' id='p");
    html += i;
    html += F("' name='p");
    html += i;
    html += F("' value='1'");
    if (gaugeConfig.pageMask & (1u << i)) html += F(" checked");
    html += F("></div>");
  }

  html += F(
    "<h2>Brightness</h2><div class='bright'><input id='brightness' type='range' name='brightness' min='40' "
    "max='255' step='1' value='"
  );
  html += gaugeConfig.brightness;
  html += F("' oninput='b.value=this.value'><output id='b'>");
  html += gaugeConfig.brightness;
  html += F(
    "</output></div><button type='submit'>SAVE &amp; REBOOT</button>"
    "<p class='note'>Home is always enabled. Detail-page choices and brightness are stored in NVS and survive firmware updates.</p></form>"
    "<div class='ota'><h2>Firmware update from phone</h2>"
    "<p class='muted'>Choose the PlatformIO <b>firmware.bin</b> file. Do not upload bootloader.bin, partitions.bin, or a merged-flash image. "
    "The new image is written to the inactive OTA slot; this running firmware is not replaced until the upload verifies successfully.</p>"
    "<input class='file' id='fw' type='file' accept='.bin,application/octet-stream'>"
    "<button id='updateButton' type='button' onclick='startUpdate()'>INSTALL FIRMWARE</button>"
    "<progress id='prog' max='100' value='0'></progress><p id='otaStatus' class='note'>Keep the gauge powered until it reboots.</p>"
    "</div><p class='note'>Setup Wi-Fi exists only in parked setup mode; normal gauge operation reboots with Wi-Fi off and BLE/OBD back on.</p>"
    "</div></div><script>"
    "function startUpdate(){const f=document.getElementById('fw').files[0],s=document.getElementById('otaStatus'),p=document.getElementById('prog'),b=document.getElementById('updateButton');"
    "if(!f){s.textContent='Choose firmware.bin first.';return;}"
    "if(!f.name.toLowerCase().endsWith('.bin')){s.textContent='That is not a .bin firmware file.';return;}"
    "if(!confirm('Install '+f.name+'? Keep the gauge powered until it reboots.'))return;"
    "const d=new FormData();d.append('firmware',f,f.name);const x=new XMLHttpRequest();"
    "x.upload.onprogress=e=>{if(e.lengthComputable){p.value=Math.round(e.loaded*100/e.total);s.textContent='Uploading '+p.value+'%';}};"
    "x.onload=()=>{s.textContent=x.responseText||((x.status===200)?'Update complete. Rebooting...':'Update failed.');if(x.status!==200)b.disabled=false;};"
    "x.onerror=()=>{s.textContent='Connection lost. If the gauge did not reboot, reconnect and try again.';b.disabled=false;};"
    "b.disabled=true;s.textContent='Starting update...';"
    "x.open('POST','/update?size='+encodeURIComponent(f.size)+'&token="
  );
  html += token;
  html += F("',true);x.send(d);}</script></body></html>");

  return html;
}

static uint8_t maskFromConfigArgs() {
  uint8_t mask = (1u << GAUGE_OVERVIEW);
  for (uint8_t i = 1; i < GAUGE_COUNT; ++i) {
    char argName[4];
    snprintf(argName, sizeof(argName), "p%u", (unsigned)i);
    if (configServer.hasArg(argName)) mask |= (1u << i);
  }
  return mask;
}

static void handleConfigSave() {
  if (otaInProgress) {
    configServer.send(409, "text/plain", "Firmware update is in progress. Wait for it to finish.");
    return;
  }

  if (!configTokenMatches()) {
    configServer.send(403, "text/plain", "Invalid or expired setup token. Reopen 192.168.4.1.");
    return;
  }

  const uint8_t pageMask = maskFromConfigArgs();
  if (pageMask == 0) {
    configServer.send(400, "text/html",
      "<html><body style='font-family:sans-serif;background:#111;color:white;padding:30px'>"
      "<h2>Keep at least one page enabled.</h2>"
      "<a style='color:white' href='/'>Go back</a></body></html>");
    return;
  }

  int brightness = configServer.hasArg("brightness") ? configServer.arg("brightness").toInt()
                                                     : gaugeConfig.brightness;
  brightness = constrain(brightness, 40, 255);

  if (!saveConfig(pageMask, (uint8_t)brightness)) {
    configServer.send(500, "text/plain", "Could not save settings.");
    return;
  }

  configServer.send(200, "text/html",
    "<html><body style='font-family:sans-serif;background:#111;color:white;padding:30px'>"
    "<h2>Saved.</h2><p>Rebooting into gauge mode...</p></body></html>");
  delay(600);
  ESP.restart();
}

static void handleFirmwareUploadChunk() {
  HTTPUpload &upload = configServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaInProgress = false;
    otaUploadComplete = false;
    otaUploadAuthorized = configTokenMatches();
    otaUploadBytes = 0;
    otaExpectedBytes = 0;
    otaErrorText[0] = '\0';

    if (!otaUploadAuthorized) {
      setOtaError("Invalid or expired setup token");
      return;
    }

    if (!upload.filename.endsWith(".bin") && !upload.filename.endsWith(".BIN")) {
      setOtaError("Choose the PlatformIO firmware.bin file");
      return;
    }

    if (!configServer.hasArg("size")) {
      setOtaError("Missing firmware size");
      return;
    }
    otaExpectedBytes = (size_t)configServer.arg("size").toInt();
    const size_t maxOtaBytes = (size_t)ESP.getFreeSketchSpace();
    if (otaExpectedBytes < 65536u || otaExpectedBytes > maxOtaBytes) {
      setOtaError("Firmware size is invalid or too large for the OTA slot");
      return;
    }

    DBG_PRINTF("[OTA] Starting upload: %s (%lu bytes)\n", upload.filename.c_str(),
               (unsigned long)otaExpectedBytes);
    if (!Update.begin(otaExpectedBytes, U_FLASH)) {
      setOtaError(Update.errorString());
      DBG_PRINTF("[OTA] begin failed: %s\n", otaErrorText);
      return;
    }
    otaInProgress = true;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaUploadAuthorized || !otaInProgress || otaErrorText[0]) return;
    const size_t written = Update.write(upload.buf, upload.currentSize);
    if (written != upload.currentSize) {
      setOtaError(Update.errorString());
      Update.abort();
      otaInProgress = false;
      DBG_PRINTF("[OTA] write failed: %s\n", otaErrorText);
      return;
    }
    otaUploadBytes += written;
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!otaUploadAuthorized || !otaInProgress || otaErrorText[0]) return;
    if (upload.totalSize != otaExpectedBytes || otaUploadBytes != otaExpectedBytes) {
      setOtaError("Upload length did not match the selected firmware file");
      Update.abort();
    } else if (Update.end(false)) {
      otaUploadComplete = true;
      otaUploadBytes = upload.totalSize;
      DBG_PRINTF("[OTA] Verified %lu bytes; new image selected for next boot\n",
                 (unsigned long)otaUploadBytes);
    } else {
      setOtaError(Update.errorString());
      DBG_PRINTF("[OTA] finalize failed: %s\n", otaErrorText);
    }
    otaInProgress = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (otaInProgress) Update.abort();
    otaInProgress = false;
    otaUploadComplete = false;
    setOtaError("Upload aborted before completion");
    DBG_PRINTLN("[OTA] Upload aborted");
  }

  delay(0);
}

static void handleFirmwareUploadFinished() {
  configServer.sendHeader("Connection", "close");

  if (!otaUploadAuthorized) {
    configServer.send(403, "text/plain", otaErrorText[0] ? otaErrorText : "Unauthorized update request");
    return;
  }

  if (!otaUploadComplete) {
    if (!otaErrorText[0]) setOtaError("No complete firmware image was received");
    configServer.send(500, "text/plain", otaErrorText);
    return;
  }

  char response[160];
  snprintf(response, sizeof(response),
           "Firmware verified (%lu bytes). Rebooting into the new version...",
           (unsigned long)otaUploadBytes);
  configServer.send(200, "text/plain", response);
  delay(900);
  ESP.restart();
}

static void handleApiInfo() {
  String json;
  json.reserve(220);
  json += F("{\"firmware\":\"");
  json += GAUGE_FW_VERSION;
  json += F("\",\"ota\":true,\"max_ota_bytes\":");
  json += (unsigned long)ESP.getFreeSketchSpace();
  json += F(",\"page_mask\":");
  json += gaugeConfig.pageMask;
  json += F(",\"brightness\":");
  json += gaugeConfig.brightness;
  json += F("}");
  configServer.send(200, "application/json", json);
}

static void startConfigServer() {
  configServer.on("/", HTTP_GET, []() {
    configServer.send(200, "text/html", buildConfigHtml());
  });
  configServer.on("/api/info", HTTP_GET, handleApiInfo);
  configServer.on("/save", HTTP_POST, handleConfigSave);
  configServer.on("/update", HTTP_POST, handleFirmwareUploadFinished, handleFirmwareUploadChunk);
  configServer.onNotFound([]() {
    configServer.sendHeader("Location", "/", true);
    configServer.send(302, "text/plain", "");
  });
  configServer.begin();
}

static void enterConfigMode() {
  if (configModeActive) return;
  configModeActive = true;
  rotateConfigCsrfToken();
  DBG_PRINTLN("[CONFIG] Entering Wi-Fi setup mode");

  // Wi-Fi and BLE share the ESP32-S3 radio. Configuration is intentionally a
  // parked, exclusive mode: stop OBD/BLE completely and reboot after saving.
  if (obdConnected) {
    markDisconnected();
  } else if (obdClient && obdClient->isConnected()) {
    obdClient->disconnect();
  }
  obdConnected = false;
  requestInFlight = false;
  inFlightRequest = REQ_NONE;
  obdWrite = nullptr;
  obdNotify = nullptr;
  resetRx();

  NimBLEDevice::deinit(true);

  WiFi.mode(WIFI_AP);
  delay(50);
  const bool apStarted = WiFi.softAP(CONFIG_AP_SSID, configApPassword);
  startConfigServer();

  if (label_config_password) {
    lv_label_set_text_fmt(label_config_password, "Password: %s", configApPassword);
  }
  lv_scr_load(scr_config);
  lv_obj_invalidate(scr_config);
  lv_timer_handler();

  DBG_PRINTF("[CONFIG] AP %s, IP %s\n",
             apStarted ? "started" : "FAILED",
             WiFi.softAPIP().toString().c_str());
}

// ----------------------------------------------------------------------------
// UI rendering - event driven only
// ----------------------------------------------------------------------------
static void updateRpmObjects(lv_obj_t *arc, lv_obj_t *label, char *textBuffer, size_t textCapacity) {
  uint16_t rpm = sensors.rpmValid ? sensors.rpm : 0;
  uint16_t arcRpm = rpm;
  if (arcRpm < 500) arcRpm = 500;
  if (arcRpm > 7000) arcRpm = 7000;

  if (sensors.rpmValid) snprintf(textBuffer, textCapacity, "%u RPM", (unsigned)rpm);
  else snprintf(textBuffer, textCapacity, "-- RPM");

  setDynamicText(label, textBuffer);
  lv_arc_set_value(arc, arcRpm);

  const uint32_t c = colorForRPM(rpm);
  lv_obj_set_style_arc_color(arc, lv_color_hex(c), LV_PART_INDICATOR);
  lv_obj_set_style_text_color(label, lv_color_hex(c), 0);
}

static void setMainInvalid() {
  lv_label_set_text_static(label_main_value, "--");
  lv_label_set_text_static(label_status, "NO DATA");
  lv_obj_set_style_text_color(label_main_value, lv_color_white(), 0);
  lv_obj_set_style_text_color(label_status, lv_color_white(), 0);
}

static void configureSinglePageStatic() {
  lv_obj_add_flag(iat_divider, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text_static(label_status, "");
  lv_label_set_text_static(label_extra, "");
  lv_obj_set_style_text_font(label_status, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_font(label_extra, &lv_font_montserrat_24, 0);
  switch (currentGauge) {
    case GAUGE_BOOST: lv_label_set_text_static(label_main_unit, "BOOST"); break;
    case GAUGE_IAT: lv_label_set_text_static(label_main_unit, "IAT"); lv_obj_clear_flag(iat_divider, LV_OBJ_FLAG_HIDDEN); break;
    case GAUGE_COOLANT: lv_label_set_text_static(label_main_unit, "COOLANT"); break;
    case GAUGE_SPARK: lv_label_set_text_static(label_main_unit, "IGN"); break;
    case GAUGE_BATTERY: lv_label_set_text_static(label_main_unit, "BATT"); break;
    default: break;
  }
}

static void formatIgnitionFixed(char *out, size_t outLen, int16_t halfDeg) {
  const bool negative = halfDeg < 0;
  const uint16_t mag = (uint16_t)(negative ? -halfDeg : halfDeg);
  // Always render sign + two integer digits so the visible number does not
  // walk left/right as timing crosses 10 degrees or zero. Examples:
  //   +04.5°  +14.5°  -04.0°
  snprintf(out, outLen, "%c%02u.%u\xC2\xB0",
           negative ? '-' : '+',
           (unsigned)(mag / 2),
           (unsigned)((mag & 1u) ? 5 : 0));
}

static void renderSingleValue() {
  switch (currentGauge) {
    case GAUGE_BOOST: {
      if (!sensors.mapValid || !sensors.baroValid) { setMainInvalid(); return; }
      const int16_t psi10 = boostPsi10();
      snprintf(mainValueText, sizeof(mainValueText), "%d.%u PSI", (int)(psi10 / 10), (unsigned)abs(psi10 % 10));
      setDynamicText(label_main_value, mainValueText);
      lv_obj_set_style_text_color(label_main_value, lv_color_white(), 0);
      const int16_t pk = boostPeakPsi10;
      snprintf(extraText, sizeof(extraText), "PEAK %d.%u PSI", (int)(pk / 10), (unsigned)abs(pk % 10));
      setDynamicText(label_extra, extraText);
      break;
    }
    case GAUGE_IAT: {
      if (sensors.iatValid) snprintf(mainValueText, sizeof(mainValueText), "%d\xC2\xB0""F", roundedF(sensors.iatF10));
      else snprintf(mainValueText, sizeof(mainValueText), "--");
      setDynamicText(label_main_value, mainValueText);
      lv_obj_set_style_text_color(label_main_value, lv_color_white(), 0);
      if (sensors.iatValid && sensors.ambientValid) {
        const int16_t delta10 = sensors.iatF10 - sensors.ambientF10;
        const int16_t delta = roundedF(delta10);
        if (delta >= 0) snprintf(statusText, sizeof(statusText), "DELTA +%d\xC2\xB0", delta);
        else snprintf(statusText, sizeof(statusText), "DELTA %d\xC2\xB0", delta);
        setDynamicText(label_status, statusText);
        lv_obj_set_style_text_color(label_status, lv_color_hex(colorForIATDelta(delta10)), 0);
      } else {
        snprintf(statusText, sizeof(statusText), "DELTA --"); setDynamicText(label_status, statusText);
        lv_obj_set_style_text_color(label_status, lv_color_white(), 0);
      }
      if (sensors.ambientValid) snprintf(extraText, sizeof(extraText), "AMB %d\xC2\xB0", roundedF(sensors.ambientF10));
      else snprintf(extraText, sizeof(extraText), "AMB --");
      setDynamicText(label_extra, extraText);
      break;
    }
    case GAUGE_COOLANT: {
      if (!sensors.coolantValid) { setMainInvalid(); return; }
      snprintf(mainValueText, sizeof(mainValueText), "%d\xC2\xB0""F", roundedF(sensors.coolantF10));
      setDynamicText(label_main_value, mainValueText);
      lv_obj_set_style_text_color(label_main_value, lv_color_hex(colorForCoolant(sensors.coolantF10)), 0);
      break;
    }
    case GAUGE_SPARK: {
      if (!sensors.sparkValid) { setMainInvalid(); return; }
      formatIgnitionFixed(mainValueText, sizeof(mainValueText), sensors.sparkHalfDeg);
      setDynamicText(label_main_value, mainValueText);
      // Reassert the fixed geometry on every update. This keeps the value
      // optically anchored even when the text changes rapidly.
      configureFixedLabel(label_main_value, 360, LV_TEXT_ALIGN_CENTER);
      lv_obj_align(label_main_value, LV_ALIGN_CENTER, 0, -20);
      lv_obj_set_style_text_color(label_main_value, lv_color_white(), 0);
      break;
    }
    case GAUGE_BATTERY: {
      if (!sensors.batteryValid) { setMainInvalid(); return; }
      const uint16_t mv=sensors.batteryMv;
      snprintf(mainValueText, sizeof(mainValueText), "%u.%u V", (unsigned)(mv/1000), (unsigned)((mv%1000)/100));
      setDynamicText(label_main_value, mainValueText);
      lv_obj_set_style_text_color(label_main_value, lv_color_hex(colorForBattery(mv)), 0);
      break;
    }
    default: break;
  }
}

static void renderOverviewValue(uint8_t row) {
  switch (row) {
    case 0: {
      if (sensors.mapValid && sensors.baroValid) { const int16_t p=boostPsi10(); snprintf(overviewValueText[0],sizeof(overviewValueText[0]),"%d.%u",(int)(p/10),(unsigned)abs(p%10)); }
      else snprintf(overviewValueText[0],sizeof(overviewValueText[0]),"--");
      lv_obj_set_style_text_color(ov_rows[0][1],lv_color_white(),0); setDynamicText(ov_rows[0][1],overviewValueText[0]); break;
    }
    case 1: {
      if (sensors.iatValid) snprintf(overviewValueText[1],sizeof(overviewValueText[1]),"%d\xC2\xB0",roundedF(sensors.iatF10)); else snprintf(overviewValueText[1],sizeof(overviewValueText[1]),"--");
      lv_obj_set_style_text_color(ov_rows[1][1],lv_color_white(),0); setDynamicText(ov_rows[1][1],overviewValueText[1]); break;
    }
    case 2: {
      if (sensors.iatValid && sensors.ambientValid) { const int16_t d10=sensors.iatF10-sensors.ambientF10; const int16_t d=roundedF(d10); if(d>=0)snprintf(overviewValueText[2],sizeof(overviewValueText[2]),"+%d\xC2\xB0",d);else snprintf(overviewValueText[2],sizeof(overviewValueText[2]),"%d\xC2\xB0",d); lv_obj_set_style_text_color(ov_rows[2][1],lv_color_hex(colorForIATDelta(d10)),0); }
      else { snprintf(overviewValueText[2],sizeof(overviewValueText[2]),"--"); lv_obj_set_style_text_color(ov_rows[2][1],lv_color_white(),0); }
      setDynamicText(ov_rows[2][1],overviewValueText[2]); break;
    }
    case 3: {
      if (sensors.coolantValid) snprintf(overviewValueText[3],sizeof(overviewValueText[3]),"%d\xC2\xB0",roundedF(sensors.coolantF10)); else snprintf(overviewValueText[3],sizeof(overviewValueText[3]),"--");
      lv_obj_set_style_text_color(ov_rows[3][1],sensors.coolantValid?lv_color_hex(colorForCoolant(sensors.coolantF10)):lv_color_white(),0); setDynamicText(ov_rows[3][1],overviewValueText[3]); break;
    }
    case 4: {
      if (sensors.sparkValid) formatIgnitionFixed(overviewValueText[4], sizeof(overviewValueText[4]), sensors.sparkHalfDeg);
      else snprintf(overviewValueText[4],sizeof(overviewValueText[4]),"--");
      lv_obj_set_style_text_color(ov_rows[4][1],lv_color_white(),0); setDynamicText(ov_rows[4][1],overviewValueText[4]); break;
    }
    case 5: {
      if (sensors.batteryValid) { const uint16_t mv=sensors.batteryMv; snprintf(overviewValueText[5],sizeof(overviewValueText[5]),"%u.%uV",(unsigned)(mv/1000),(unsigned)((mv%1000)/100)); lv_obj_set_style_text_color(ov_rows[5][1],lv_color_hex(colorForBattery(mv)),0); }
      else { snprintf(overviewValueText[5],sizeof(overviewValueText[5]),"--"); lv_obj_set_style_text_color(ov_rows[5][1],lv_color_white(),0); }
      setDynamicText(ov_rows[5][1],overviewValueText[5]); break;
    }
  }
}

static void renderVisibleUi() {
  if (!transitionedToGauge || uiDirty == 0) return;
  const uint16_t dirty = uiDirty;
  uiDirty = 0;

  if (currentGauge == GAUGE_OVERVIEW) {
    if (dirty & (DIRTY_RPM | DIRTY_PAGE)) updateRpmObjects(ov_arc_rpm, ov_rpm_label, overviewRpmText, sizeof(overviewRpmText));
    if (dirty & (DIRTY_MAP | DIRTY_BARO | DIRTY_PAGE)) renderOverviewValue(0);
    if (dirty & (DIRTY_IAT | DIRTY_PAGE)) renderOverviewValue(1);
    if (dirty & (DIRTY_IAT | DIRTY_AMBIENT | DIRTY_PAGE)) renderOverviewValue(2);
    if (dirty & (DIRTY_COOLANT | DIRTY_PAGE)) renderOverviewValue(3);
    if (dirty & (DIRTY_SPARK | DIRTY_PAGE)) renderOverviewValue(4);
    if (dirty & (DIRTY_BATTERY | DIRTY_PAGE)) renderOverviewValue(5);
    return;
  }

  if (dirty & (DIRTY_RPM | DIRTY_PAGE)) updateRpmObjects(arc_rpm, label_rpm_bottom, singleRpmText, sizeof(singleRpmText));
  bool valueDirty = (dirty & DIRTY_PAGE) != 0;
  switch (currentGauge) {
    case GAUGE_BOOST: valueDirty |= (dirty & (DIRTY_MAP | DIRTY_BARO)) != 0; break;
    case GAUGE_IAT: valueDirty |= (dirty & (DIRTY_IAT | DIRTY_AMBIENT)) != 0; break;
    case GAUGE_COOLANT: valueDirty |= (dirty & DIRTY_COOLANT) != 0; break;
    case GAUGE_SPARK: valueDirty |= (dirty & DIRTY_SPARK) != 0; break;
    case GAUGE_BATTERY: valueDirty |= (dirty & DIRTY_BATTERY) != 0; break;
    default: break;
  }
  if (valueDirty) renderSingleValue();
}

static void forceCurrentPageRequestDue(uint32_t now) {
  switch (currentGauge) {
    case GAUGE_OVERVIEW:
      makeDue(REQ_MAP, now); makeDue(REQ_BARO, now); makeDue(REQ_IAT, now); makeDue(REQ_AMBIENT, now);
      makeDue(REQ_COOLANT, now); makeDue(REQ_SPARK, now); makeDue(REQ_BATTERY, now);
      break;
    case GAUGE_BOOST: makeDue(REQ_MAP, now); makeDue(REQ_BARO, now); break;
    case GAUGE_IAT: makeDue(REQ_IAT, now); makeDue(REQ_AMBIENT, now); break;
    case GAUGE_COOLANT: makeDue(REQ_COOLANT, now); break;
    case GAUGE_SPARK: makeDue(REQ_SPARK, now); break;
    case GAUGE_BATTERY: makeDue(REQ_BATTERY, now); break;
    default: break;
  }
}

static void showCurrentScreen() {
  lv_obj_t *target = nullptr;

  if (currentGauge == GAUGE_OVERVIEW) {
    target = scr_overview;
  } else {
    configureSinglePageStatic();
    target = scr_single;
  }

  // Detail pages all reuse scr_single. Do not reload the exact same LVGL screen
  // for every LEFT press; just update its fixed labels and redraw it. This
  // removes a noticeable page-change stall and avoids overlapping transition
  // invalidations that could leave a partial old frame on the panel.
  if (lv_scr_act() != target) {
    // On the first boot->gauge transition, clear the physical panel once before
    // LVGL paints the rotated UI. Black is rotation-independent and prevents
    // any retained startup fragments from being visible underneath the first frame.
    if (lv_scr_act() == scr_boot) gfx->fillScreen(RGB565_BLACK);
    lv_scr_load(target);
  }

  lv_obj_invalidate(target);
  uiDirty = DIRTY_ALL;
  forceCurrentPageRequestDue(millis());
  renderVisibleUi();

  // Page changes are user actions, so flush them now instead of waiting for the
  // next normal loop pass. This makes LEFT feel immediate while preserving the
  // event-driven renderer for ordinary sensor updates.
  lv_timer_handler();
}

static void cycleGauge() {
  const uint8_t start = (uint8_t)currentGauge;
  for (uint8_t step = 1; step <= GAUGE_COUNT; ++step) {
    const GaugeType candidate = (GaugeType)((start + step) % GAUGE_COUNT);
    if (isPageEnabled(candidate)) {
      currentGauge = candidate;
      DBG_PRINTF("[UI] Gauge %u\n", (unsigned)currentGauge);
      showCurrentScreen();
      return;
    }
  }
}

static void goHome() {
  if (currentGauge == GAUGE_OVERVIEW) return;
  currentGauge = GAUGE_OVERVIEW;
  DBG_PRINTLN("[UI] Home");
  showCurrentScreen();
}


// Both side buttons are polled. BOOT is a direct GPIO; PWR is the conditioned
// SYS_OUT signal on TCA9554 EXIO4. The expander read is throttled to 50 Hz so
// button handling has negligible impact on the OBD/display hot path.
static bool tcaReadReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TCA9554_ADDR, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

static bool tcaWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static void initPwrButtonExpander() {
  // TCA9554 config register is 0x03; a 1 configures the pin as input. Preserve
  // every other expander pin because other board peripherals also share it.
  uint8_t cfg = 0xFF;
  if (tcaReadReg(0x03, cfg)) tcaWriteReg(0x03, (uint8_t)(cfg | TCA9554_PWR_MASK));
}

static bool readPwrButton(bool &pressed) {
  uint8_t input = 0;
  if (!tcaReadReg(TCA9554_INPUT_REG, input)) return false;
  pressed = (input & TCA9554_PWR_MASK) != 0;
  return true;
}



enum ButtonEvent : uint8_t { BUTTON_EVENT_NONE=0, BUTTON_EVENT_PRESSED=1, BUTTON_EVENT_RELEASED=2 };
struct DebouncedButton {
  bool initialized=false, rawDown=false, stableDown=false, armed=false, pressWasArmed=false, runawayLock=false;
  uint32_t rawChangedMs=0, downSinceMs=0, releaseSinceMs=0, lastActionMs=0, burstWindowStartMs=0;
  uint8_t burstActions=0;
};

static uint8_t updateDebouncedButton(DebouncedButton &b, bool rawDown, uint32_t now) {
  if(!b.initialized){ b.initialized=true; b.rawDown=rawDown; b.stableDown=rawDown; b.rawChangedMs=now; b.releaseSinceMs=rawDown?0:now; return BUTTON_EVENT_NONE; }
  if(rawDown!=b.rawDown){ b.rawDown=rawDown; b.rawChangedMs=now; }
  if(!b.stableDown && !b.rawDown && !b.armed && (uint32_t)(now-b.releaseSinceMs)>=BUTTON_REARM_RELEASE_MS) b.armed=true;
  if(b.runawayLock && !b.stableDown && !b.rawDown && (uint32_t)(now-b.releaseSinceMs)>=BUTTON_RUNAWAY_UNLOCK_MS){ b.runawayLock=false; b.burstActions=0; b.burstWindowStartMs=now; b.armed=true; DBG_PRINTLN("[BUTTON] LEFT runaway lock cleared"); }
  if(b.rawDown==b.stableDown || (uint32_t)(now-b.rawChangedMs)<BUTTON_DEBOUNCE_MS) return BUTTON_EVENT_NONE;
  b.stableDown=b.rawDown;
  if(b.stableDown){ b.pressWasArmed=b.armed && !b.runawayLock; b.armed=false; b.downSinceMs=now; return b.pressWasArmed?BUTTON_EVENT_PRESSED:BUTTON_EVENT_NONE; }
  b.releaseSinceMs=now;
  return b.pressWasArmed?BUTTON_EVENT_RELEASED:BUTTON_EVENT_NONE;
}

static bool buttonActionAllowed(DebouncedButton &b, uint32_t now, bool protectRunaway) {
  if(b.lastActionMs != 0u && (uint32_t)(now-b.lastActionMs)<BUTTON_ACTION_COOLDOWN_MS) return false;
  if(protectRunaway){
    if(b.burstActions==0 || (uint32_t)(now-b.burstWindowStartMs)>BUTTON_RUNAWAY_WINDOW_MS){ b.burstWindowStartMs=now; b.burstActions=1; }
    else if(++b.burstActions>=BUTTON_RUNAWAY_MAX_ACTIONS){ b.runawayLock=true; b.armed=false; DBG_PRINTLN("[BUTTON] LEFT runaway protection engaged"); return false; }
  }
  b.lastActionMs=now;
  return true;
}

static void serviceButtons(uint32_t now) {
  static DebouncedButton rightButton, leftButton;
  static bool rightLongTriggered=false;
  static uint32_t lastPwrPoll=0;
  static bool leftWasBootLocked=true;

  // RIGHT / BOOT is a direct GPIO and remains available during startup so a
  // long hold can always enter Wi-Fi/OTA setup. A short RIGHT only goes Home
  // after the real gauge UI has actually been shown.
  const uint8_t rightEvent=updateDebouncedButton(rightButton,digitalRead(BUTTON_BOOT)==LOW,now);
  if(rightEvent&BUTTON_EVENT_PRESSED) rightLongTriggered=false;
  if(rightButton.stableDown && rightButton.pressWasArmed && !rightLongTriggered && !configModeActive &&
     (uint32_t)(now-rightButton.downSinceMs)>=CONFIG_HOLD_MS){
    rightLongTriggered=true;
    enterConfigMode();
  }
  if(rightEvent&BUTTON_EVENT_RELEASED){
    const uint32_t held=(uint32_t)(now-rightButton.downSinceMs);
    if(!rightLongTriggered && held>=BUTTON_DEBOUNCE_MS && buttonActionAllowed(rightButton,now,false)){
      if(configModeActive){
        if(otaInProgress) DBG_PRINTLN("[CONFIG] RIGHT ignored during OTA");
        else { delay(40); ESP.restart(); }
      } else if (transitionedToGauge) {
        goHome();
      }
    }
    rightLongTriggered=false;
  }

  if((uint32_t)(now-lastPwrPoll)<20u) return;
  lastPwrPoll=now;

  bool pwr=false;
  if(!readPwrButton(pwr)) return;
  const uint8_t leftEvent=updateDebouncedButton(leftButton,pwr,now);

  // SYS_OUT can move while the PMIC/expander is settling at power-up. Never let
  // those electrical transitions navigate the UI. LEFT stays hard-locked until
  // both the gauge is genuinely on-screen and the board has had 12 seconds to
  // settle. While locked we still sample it, but deliberately disarm actions.
  const bool leftBootLocked = !transitionedToGauge || configModeActive || now < LEFT_NAV_BOOT_LOCKOUT_MS;
  if(leftBootLocked){
    leftButton.armed=false;
    leftButton.pressWasArmed=false;
    leftWasBootLocked=true;
    return;
  }

  if(leftWasBootLocked){
    // Require a clean released state after the lockout before accepting the
    // first real press. This prevents a press/noise edge that began during boot
    // from leaking through as soon as the timer expires.
    leftButton.armed=false;
    leftButton.pressWasArmed=false;
    leftButton.releaseSinceMs = pwr ? 0u : now;
    leftWasBootLocked=false;
    return;
  }

  // LEFT has no long-press meaning, so cycle on the debounced PRESS instead of
  // waiting for release. That removes the sluggish feel while still producing
  // exactly one page action per physical press.
  if(leftEvent&BUTTON_EVENT_PRESSED){
    if(buttonActionAllowed(leftButton,now,true)) cycleGauge();
  }
}

// ----------------------------------------------------------------------------
// Arduino setup / loop
// ----------------------------------------------------------------------------
void setup() {
  DBG_BEGIN();
  DBG_PRINTF("\nElantra N Gauge - %s\n", GAUGE_FW_VERSION);

  pinMode(BUTTON_BOOT, INPUT_PULLUP);
  buildSetupCredentials();
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  initPwrButtonExpander();
  loadConfig();
  selectFirstEnabledPage();

  if (!gfx->begin()) {
    DBG_PRINTLN("[DISPLAY] gfx begin failed");
  }
  gfx->setBrightness(gaugeConfig.brightness);
  gfx->fillScreen(RGB565_BLACK);

  lv_init();

  // One full-screen RGB565 LVGL buffer plus one full-screen rotation scratch
  // buffer in PSRAM. The extra scratch buffer replaces LVGL 8.4's chunked
  // software-rotation path, which is the leading suspect in the old/new
  // half-drawn dynamic-number corruption. Both are allocated once at boot.
  const uint32_t pixelCount = (uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT;
  drawBuffer = (lv_color_t*)heap_caps_malloc(pixelCount * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  rotateBuffer = (uint16_t*)heap_caps_malloc(pixelCount * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  if (!drawBuffer || !rotateBuffer) {
    DBG_PRINTLN("[DISPLAY] draw/rotation buffer allocation failed");
    while (true) delay(1000);
  }
  lv_disp_draw_buf_init(&draw_buf, drawBuffer, nullptr, pixelCount);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;

  // Keep LVGL in its logical orientation and rotate in my_disp_flush(). This
  // avoids LVGL's LV_DISP_ROT_MAX_BUF chunking entirely. The display is square,
  // so the logical resolution remains 466x466 exactly as before.
  disp_drv.sw_rotate = 0;
  disp_drv.rotated = LV_DISP_ROT_NONE;
  lv_disp_drv_register(&disp_drv);

  buildBootScreen();
  buildConfigScreen();
  buildSingleGaugeScreen();
  buildOverviewScreen();
  lv_scr_load(scr_boot);

  NimBLEDevice::init("");
  DBG_PRINTLN("[BLE] NimBLE ready");

  // Make the first reconnect attempt immediately from loop().
  lastConnectAttempt = millis() - RECONNECT_INTERVAL_MS;
}

void loop() {
  const uint32_t now = millis();

  // LVGL timebase first. No 16 ms animation task exists anymore; LVGL only has
  // work when we invalidate something because new data or a page change occurred.
  static uint32_t lastTick = now;
  const uint32_t elapsed = (uint32_t)(now - lastTick);
  if (elapsed) {
    lv_tick_inc(elapsed);
    lastTick = now;
  }

  serviceButtons(now);

  if (configModeActive) {
    configServer.handleClient();
    lv_timer_handler();
    delay(1);
    return;
  }

  if (obdConnected && obdClient && !obdClient->isConnected()) {
    markDisconnected();
  }

  if (!obdConnected) {
    if ((uint32_t)(now - lastConnectAttempt) >= RECONNECT_INTERVAL_MS) {
      lastConnectAttempt = now;
      if (tryConnectOBD()) {
        transitionedToGauge = true;
        showCurrentScreen();
        startRequest(REQ_RPM);
      }
    }
  } else {
    serviceOBD(now);
  }

  renderVisibleUi();
  lv_timer_handler();

  // No delay(): when the Veepeak response is ready, the next request should be
  // dispatched as soon as possible. FreeRTOS/NimBLE still get CPU time through
  // their own tasks and through blocking panel/BLE driver calls.
}
