#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * All host/APP/Bootloader communication uses the nominal baud rate 1,000,000.
 *
 * Cold entry: the chip runs at 48 MHz.
 * Timer1 divisor 12 gives exact 1,000,000 baud and IAP_TPS must be 48.
 *
 * Hot entry (SEEKFREE APP writes DfuFlag then IAP_CONTR=0x28): software reset
 * preserves the APP's 124 MHz clock. The STC integer Timer1 formula used by
 * the SEEKFREE UART driver selects divisor 31 for a requested 1,000,000 baud,
 * producing exactly 1,000,000 baud (0% error). The PC opens the port at
 * 1,000,000; this baud rate is exact on both cold and hot entry because
 * FOSC_COLD/4 and FOSC_HOT/4 are integer multiples of 1,000,000.
 * IAP_TPS must remain 124 for Flash programming at the retained clock.
 *
 * Force-pin entry (P3.3 low at power-on) forces DFU mode.
 * Safe on cold boot: FastBoot runs with EA=0, so the camera VSYNC
 * (INT1 / P3.3) never fires. The pin is sampled once before the APP
 * ever executes.
 * Hot entry is triggered by the 12-byte "SBLR" frame on UART1
 * from the running APP.
 */
#define FOSC_COLD               48000000UL
#define FOSC_HOT                124000000UL
#define UART_BAUD               1000000UL
#define UART_T1_DIV_COLD        (FOSC_COLD / UART_BAUD / 4UL)
#define UART_T1_DIV_HOT         (FOSC_HOT / UART_BAUD / 4UL)
#define UART_T1_RELOAD_COLD     (65536UL - UART_T1_DIV_COLD)
#define UART_T1_RELOAD_HOT      (65536UL - UART_T1_DIV_HOT)

#define IAP_TPS_COLD            48U
#define IAP_TPS_HOT             124U

//#define DEBUG

#define LDR_SIZE                (4 * 1024UL)
#define CHIP_SIZE               (246 * 1024UL)
#define AP_SIZE                 (CHIP_SIZE - LDR_SIZE)
#define IAP_BASE                (0x01000000UL - AP_SIZE)

/*
 * With EEPROM=8 KiB and user-system=4 KiB, the normal APP starts at
 * physical FC:4800. IAP protocol addresses remain relative to IAP_BASE
 * (FC:3800), therefore APP data begins at logical offset 0x1000.
 */
#define APP_LOGICAL_BEGIN       0x00001000UL
#define APP_SIZE                (AP_SIZE - APP_LOGICAL_BEGIN)
#define FLASH_PAGE_SIZE         0x00000200UL

/* Dedicated 512-byte upgrade metadata page: physical FC:4600-FC:47FF. */
#define UPDATE_META_LOGICAL     0x00000E00UL
#define UPDATE_META_PHYSICAL    (IAP_BASE + UPDATE_META_LOGICAL)

/* v2.6: turbo incremental page CRC, fast program/erase and 1 Mbaud UART. */
#define LDR_VERSION             0x0206U

#endif
