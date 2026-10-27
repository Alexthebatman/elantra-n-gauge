#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ============================================================================
// Waveshare ESP32-S3-Touch-AMOLED-1.75 (and -B enclosure variant)
// GPIO assignments taken from Waveshare's official HARDWARE_REFERENCE.md:
//   https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.75
//     /blob/main/HARDWARE_REFERENCE.md
//
// Display : CO5300 AMOLED, 466 x 466, QSPI
// Touch   : CST9217 on the shared I2C bus (not used by the gauge firmware)
// ============================================================================

// ---- Display geometry ----
#define LCD_WIDTH    466
#define LCD_HEIGHT   466

// ---- CO5300 QSPI display bus ----
#define LCD_SDIO0     4    // LCD_DATA0 - QSPI SIO0
#define LCD_SDIO1     5    // LCD_DATA1 - QSPI SIO1
#define LCD_SDIO2     6    // LCD_DATA2 - QSPI SIO2
#define LCD_SDIO3     7    // LCD_DATA3 - QSPI SIO3
#define LCD_CS       12    // CO5300 chip select
#define LCD_SCLK     38    // LCD_PCLK - QSPI clock
#define LCD_RESET    39    // CO5300 reset (separate from touch reset)
#define LCD_TE       13    // Tearing-effect signal (unused by this firmware)

// ---- Shared onboard I2C bus ----
#define IIC_SCL      14
#define IIC_SDA      15

// ---- Touch controller (unused by the gauge firmware, kept for reference) ----
#define TP_INT       11
#define TP_RST       40
#define TP_I2C_ADDR  0x5A  // CST9217, 7-bit address

// ---- Buttons ----
// BOOT is externally pulled up and reads LOW while pressed.
// The gauge firmware uses BOOT as the RIGHT button; hold it for setup/OTA.
#define BUTTON_BOOT_PIN  0

// ---- Notes ----
// * The AMOLED has no GPIO backlight. Brightness is a CO5300 panel command,
//   which is why the firmware calls gfx->setBrightness() instead of PWM.
// * GPIO 1/2/3/41 are the microSD lines, 8/9/10/42/45/46 are audio,
//   19/20 are native USB. Don't repurpose them.
// * Free GPIOs broken out on the 8-pin header: 16, 17, 18 (plus UART0 43/44).

#endif // PIN_CONFIG_H
