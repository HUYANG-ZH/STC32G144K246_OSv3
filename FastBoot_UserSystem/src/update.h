#ifndef __UPDATE_H__
#define __UPDATE_H__

#define UPDATE_STATE_EMPTY       0xffU
#define UPDATE_STATE_IN_PROGRESS 0x7fU
#define UPDATE_STATE_VERIFIED    0x3fU
#define UPDATE_STATE_COMMITTED   0x1fU

BOOL update_recovery_pending(void);
BOOL update_write_allowed(void);
BYTE update_get_status(DWORD *image_size, DWORD *image_crc);
BOOL update_begin(DWORD image_size, DWORD image_crc);
BOOL update_verify(DWORD *actual_crc);
BOOL update_commit(void);

#endif
