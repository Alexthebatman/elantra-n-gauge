/**
 * @file lv_conf.h
 * LVGL v8.4 configuration for the Elantra N gauge
 * (Waveshare ESP32-S3-Touch-AMOLED-1.75, CO5300 466x466)
 *
 * NOTE: You do NOT need the full 1000-line template. LVGL's
 * lv_conf_internal.h provides a default for every setting that isn't
 * defined here, so this file only overrides what this project needs.
 *
 * Place at: include/lv_conf.h
 * Requires -DLV_CONF_INCLUDE_SIMPLE and -I include in platformio.ini
 */

#if 1 /* Set to "1" to enable this config file */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/* RGB565. The flush callback hands the buffer straight to
   Arduino_GFX draw16bitRGBBitmap(), which expects native-endian RGB565,
   so LV_COLOR_16_SWAP must stay 0. If the display shows correct shapes
   in wrong colors, this is the setting to flip. */
#define LV_COLOR_DEPTH     16
#define LV_COLOR_16_SWAP   0

/* Screens are drawn on black; no chroma keying used. */
#define LV_COLOR_SCREEN_TRANSP  0
#define LV_COLOR_MIX_ROUND_OFS  (LV_COLOR_DEPTH == 32 ? 0 : 128)
#define LV_COLOR_CHROMA_KEY     lv_color_hex(0x00ff00)

/*=========================
   MEMORY SETTINGS
 *=========================*/

/* Use LVGL's own allocator. 64 KB is comfortable for this UI
   (a handful of screens, arcs, labels and two images).
   The big 466x466/4 frame buffers are allocated separately in
   setup() from PSRAM with heap_caps_malloc(), not from here. */
#define LV_MEM_CUSTOM      0
#define LV_MEM_SIZE        (64U * 1024U)
#define LV_MEM_ADR         0
#define LV_MEM_BUF_MAX_NUM 16
#define LV_MEMCPY_MEMSET_STD 0

/*====================
   HAL SETTINGS
 *====================*/

/* ~60 fps refresh and input period. */
#define LV_DISP_DEF_REFR_PERIOD  16
#define LV_INDEV_DEF_READ_PERIOD 30

/* The sketch calls lv_tick_inc() itself in loop(), so custom tick is OFF. */
#define LV_TICK_CUSTOM     0

/* Assume ~130 DPI for a 1.75" 466x466 round panel. */
#define LV_DPI_DEF         130

/*=======================
   FEATURE CONFIGURATION
 *=======================*/

#define LV_DRAW_COMPLEX             1
#define LV_SHADOW_CACHE_SIZE        0
#define LV_CIRCLE_CACHE_SIZE        4

#define LV_IMG_CACHE_DEF_SIZE       0
#define LV_GRADIENT_MAX_STOPS       2
#define LV_GRAD_CACHE_DEF_SIZE      0
#define LV_DITHER_GRADIENT          0

#define LV_DISP_ROT_MAX_BUF         (10*1024)

/*-----------
 * GPU - none on this board, software rendering only
 *-----------*/
#define LV_USE_GPU_STM32_DMA2D      0
#define LV_USE_GPU_NXP_PXP          0
#define LV_USE_GPU_NXP_VG_LITE      0
#define LV_USE_GPU_SDL              0

/*-------------
 * Logging
 *-----------*/
#define LV_USE_LOG      0

/*-------------
 * Asserts
 *-----------*/
#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0
#define LV_ASSERT_HANDLER_INCLUDE   <stdint.h>
#define LV_ASSERT_HANDLER  while(1);

/*-------------
 * Others
 *-----------*/
#define LV_USE_PERF_MONITOR  0
#define LV_USE_MEM_MONITOR   0
#define LV_USE_REFR_DEBUG    0

#define LV_SPRINTF_CUSTOM    0
#define LV_SPRINTF_USE_FLOAT 0

#define LV_USE_USER_DATA     1
#define LV_ENABLE_GC         0

/*==================
 *   FONT USAGE
 *===================*/

/* Every font the gauge code references must be enabled here.
   A missing one produces:
     undefined reference to `lv_font_montserrat_XX'
   which names the exact font you forgot. */
#define LV_FONT_MONTSERRAT_8   0
#define LV_FONT_MONTSERRAT_10  0
#define LV_FONT_MONTSERRAT_12  0
#define LV_FONT_MONTSERRAT_14  1   /* default font + disclaimer label */
#define LV_FONT_MONTSERRAT_16  0
#define LV_FONT_MONTSERRAT_18  1   /* ambient annotation on IC Delta */
#define LV_FONT_MONTSERRAT_20  1   /* overview row labels, status text */
#define LV_FONT_MONTSERRAT_22  1   /* main unit label */
#define LV_FONT_MONTSERRAT_24  1   /* overview values, bottom RPM */
#define LV_FONT_MONTSERRAT_26  0
#define LV_FONT_MONTSERRAT_28  1   /* single-gauge bottom RPM */
#define LV_FONT_MONTSERRAT_30  0
#define LV_FONT_MONTSERRAT_32  1   /* Home overview values */
#define LV_FONT_MONTSERRAT_34  0
#define LV_FONT_MONTSERRAT_36  0
#define LV_FONT_MONTSERRAT_38  0
#define LV_FONT_MONTSERRAT_40  0
#define LV_FONT_MONTSERRAT_42  0
#define LV_FONT_MONTSERRAT_44  0
#define LV_FONT_MONTSERRAT_46  0
#define LV_FONT_MONTSERRAT_48  1   /* big main value */

#define LV_FONT_MONTSERRAT_12_SUBPX      0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_16_CJK            0

#define LV_FONT_UNSCII_8   0
#define LV_FONT_UNSCII_16  0

#define LV_FONT_CUSTOM_DECLARE

#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_FONT_FMT_TXT_LARGE   0
#define LV_USE_FONT_COMPRESSED  0
#define LV_USE_FONT_SUBPX       0
#define LV_USE_FONT_PLACEHOLDER 1

/*=================
 *  TEXT SETTINGS
 *=================*/

/* Degree symbol (\u00B0) is used in the temperature labels. */
#define LV_TXT_ENC LV_TXT_ENC_UTF8
#define LV_TXT_BREAK_CHARS " ,.;:-_"
#define LV_TXT_LINE_BREAK_LONG_LEN 0
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN  3
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3
#define LV_TXT_COLOR_CMD "#"
#define LV_USE_BIDI       0
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*==================
 *  WIDGET USAGE
 *================*/

/* Only what the gauge actually draws. Everything else off to save flash. */
#define LV_USE_ARC        1
#define LV_USE_IMG        1
#define LV_USE_LABEL      1
#define LV_USE_LINE       1

#define LV_LABEL_TEXT_SELECTION 0
#define LV_LABEL_LONG_TXT_HINT  1

#define LV_USE_ANIMIMG    0
#define LV_USE_BAR        0
#define LV_USE_BTN        0
#define LV_USE_BTNMATRIX  0
#define LV_USE_CANVAS     0
#define LV_USE_CHECKBOX   0
#define LV_USE_DROPDOWN   0
#define LV_USE_ROLLER     0
#define LV_USE_SLIDER     0
#define LV_USE_SWITCH     0
#define LV_USE_TEXTAREA   0
#define LV_USE_TABLE      0

/*==================
 * EXTRA COMPONENTS
 *==================*/

#define LV_USE_CALENDAR   0
#define LV_USE_CHART      0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN     0
#define LV_USE_KEYBOARD   0
#define LV_USE_LED        0
#define LV_USE_LIST       0
#define LV_USE_MENU       0
#define LV_USE_METER      0
#define LV_USE_MSGBOX     0
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    0
#define LV_USE_TABVIEW    0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0
#define LV_USE_SPAN       0

/*-----------
 * Themes
 *----------*/

/* Default theme is needed for basic styling to work. */
#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1
#define LV_THEME_DEFAULT_GROW 0
#define LV_THEME_DEFAULT_TRANSITION_TIME 0
#define LV_USE_THEME_BASIC   0
#define LV_USE_THEME_MONO    0

/*-----------
 * Layouts
 *----------*/
#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/*---------------------
 * Third party libs
 *--------------------*/
#define LV_USE_FS_STDIO   0
#define LV_USE_FS_POSIX   0
#define LV_USE_FS_WIN32   0
#define LV_USE_FS_FATFS   0
#define LV_USE_PNG        0
#define LV_USE_BMP        0
#define LV_USE_SJPG       0
#define LV_USE_GIF        0
#define LV_USE_QRCODE     0
#define LV_USE_FREETYPE   0
#define LV_USE_TINY_TTF   0
#define LV_USE_RLOTTIE    0
#define LV_USE_FFMPEG     0

/*-----------
 * Others
 *----------*/
#define LV_USE_SNAPSHOT   0
#define LV_USE_MONKEY     0
#define LV_USE_GRIDNAV    0
#define LV_USE_FRAGMENT   0
#define LV_USE_IMGFONT    0
#define LV_USE_MSG        0
#define LV_USE_IME_PINYIN 0

/*==================
 * EXAMPLES / DEMOS
 *==================*/
#define LV_BUILD_EXAMPLES 0

#define LV_USE_DEMO_WIDGETS        0
#define LV_USE_DEMO_KEYPAD_AND_ENCODER 0
#define LV_USE_DEMO_BENCHMARK      0
#define LV_USE_DEMO_STRESS         0
#define LV_USE_DEMO_MUSIC          0

#endif /* LV_CONF_H */

#endif /* End of "Content enable" */
