/* Renesas R7FA4M1AB (Arduino Uno R4), laid out for the stock Arduino bootloader.
 *
 * The bootloader owns the first 16 KB of flash and loads the sketch at 0x4000, so
 * that is where this image starts. The values match `FLASH_IMAGE_START`,
 * `FLASH_LENGTH` and `RAM_LENGTH` in ArduinoCore-renesas, variants/MINIMA:
 *
 *     FLASH_START       = 0x00000000
 *     FLASH_LENGTH      = 0x40000      (256 KB total)
 *     FLASH_IMAGE_START = 0x4000       (16 KB bootloader)
 *     RAM_START         = 0x20000000
 *     RAM_LENGTH        = 0x8000       (32 KB)
 *
 * The option-setting memory (OFS0, OFS1 and the security-MPU settings at
 * 0x400..0x440) belongs to the bootloader in this layout, not to us; Arduino's own
 * linker script gives the sketch an OPTION_SETTING region of length 0 for the same
 * reason. Nothing here needs to avoid it.
 *
 * Because the image no longer starts at 0, the vector table is not where the core
 * looks for it out of reset. `uno_r4_hal::relocate_vector_table` points VTOR at it,
 * and `uno_r4_hal::take_peripherals` calls that for you.
 *
 * To flash over SWD with no bootloader instead, use the bare-metal layout:
 *
 *     FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
 *
 * and add `_stext = ORIGIN(FLASH) + 0x440;` below, to keep code out of the
 * option-setting memory that the boot sequence reads during reset. `cortex-m-rt`
 * reserves 1 KB for the vector table, so without that nudge .text would begin at
 * exactly 0x400 and the hardware would read your first instructions as option
 * settings.
 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00004000, LENGTH = 240K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}
