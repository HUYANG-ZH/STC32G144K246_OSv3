#ifndef __DFU_H__
#define __DFU_H__

#define DFU_FORCEPIN            P33
#define DFU_TAG                 0x12abcd34UL

#define DFU_CMD_CONNECT         0xa0
#define DFU_CMD_READ            0xa1
#define DFU_CMD_PROGRAM         0xa2
#define DFU_CMD_ERASE           0xa3
#define DFU_CMD_REBOOT          0xa4
#define DFU_CMD_ERASE_PAGE      0xa5
#define DFU_CMD_CRC32           0xa6
#define DFU_CMD_STATUS          0xa7
#define DFU_CMD_UPDATE_BEGIN    0xa8
#define DFU_CMD_UPDATE_VERIFY   0xa9
#define DFU_CMD_UPDATE_COMMIT   0xaa
#define DFU_CMD_PAGE_CRC_TABLE  0xab

#define STATUS_OK               0x00
#define STATUS_ERRORCMD         0x01
#define STATUS_OUTOFRANGE       0x02
#define STATUS_PROGRAMERR       0x03
#define STATUS_ERASEERR         0x04
#define STATUS_CRCERR           0x05
#define STATUS_STATEERR         0x06
#define STATUS_METAERR          0x07
#define STATUS_ERRORWRAP        0xff

void dfu_check(void);
void dfu_events(void);

extern long xdata DfuFlag;
extern bit bDfuHotEntry;
extern char *USER_STCISPCMD;

#endif
