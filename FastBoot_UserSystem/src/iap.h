#ifndef __IAP_H__
#define __IAP_H__

void iap_init(void);
BOOL iap_check_addr(DWORD addr);
BYTE iap_read_byte(DWORD physical_addr);
BOOL iap_program_block(DWORD addr, BYTE near *src_ptr, BYTE size);
BOOL iap_erase_page(DWORD addr);
BOOL iap_crc32(DWORD addr, DWORD length, DWORD *result);
BOOL iap_meta_erase(void);
BOOL iap_meta_write(BYTE offset, BYTE value);
BYTE iap_meta_read(BYTE offset);

#endif
