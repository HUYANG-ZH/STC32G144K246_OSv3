/*********************************************************************************************************************
* STC32G144K Opensource Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可声明为译文版本
* 许可声明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/


#ifndef __ZF_DRIVER_EEPROM_H
#define __ZF_DRIVER_EEPROM_H

#include "zf_common_typedef.h"

/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║   IAP ADDRESS GUARD — DO NOT BYPASS                         ║
 * ║   Bootloader region [0xFC3800 – 0xFC47FF] is PROTECTED.    ║
 * ║   Writing / erasing this range WILL BRICK the device.       ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * Whitelist: only physical Flash [0xFC2800, 0xFFFFFF] is valid.
 * The bootloader + metadata page [0xFC3800, 0xFC47FF] is excluded
 * even within that range.
 */
#define IAP_FLASH_START         0xFC2800UL
#define IAP_FLASH_END           0xFFFFFFUL
#define IAP_BOOTLOADER_START    0xFC3800UL
#define IAP_BOOTLOADER_END      0xFC47FFUL

/* Returns 1 if addr is safe to write/erase, 0 otherwise. */
#define iap_is_addr_safe(addr)  \
    (((addr) >= IAP_FLASH_START && (addr) <= IAP_FLASH_END)  \
     && !((addr) >= IAP_BOOTLOADER_START && (addr) <= IAP_BOOTLOADER_END))

void iap_init(void);
void iap_idle(void);
void iap_set_tps(void);
uint8 iap_get_cmd_state(void);

/* Read — always safe, no guard needed. */
uint8 iap_read_byte(uint32 addr);
void iap_read_buff(uint32 addr, uint8 *buf, uint16 len);

/*
 * ═══  WARNING: Write / Erase — IAP guard enforced  ═══
 *
 * iap_write_byte, iap_write_buff, iap_erase_page and
 * extern_iap_write_buff all call iap_is_addr_safe(addr)
 * internally and silently return if the address falls in the
 * bootloader region [0xFC3800 – 0xFC47FF] or outside
 * physical Flash [0xFC2800 – 0xFFFFFF].
 *
 * Callers SHOULD still pre-validate with iap_is_addr_safe()
 * and treat a rejection as a fatal error.
 *
 *  DO NOT add new call sites that write/erase Flash without
 *  first verifying the address against iap_is_addr_safe().
 */
void iap_write_byte(uint32 addr, uint8 byte);
void iap_write_buff(uint32 addr, uint8 *buf, uint16 len);
void iap_erase_page(uint32 addr);
void extern_iap_write_buff(uint16 addr, uint8 *buf, uint16 len);


#endif


