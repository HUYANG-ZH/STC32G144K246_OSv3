#include "stc.h"
#include "iap.h"
#include "update.h"

#define META_MAGIC0          ((BYTE)'F')
#define META_MAGIC1          ((BYTE)'B')
#define META_MAGIC2          ((BYTE)'S')
#define META_MAGIC3          ((BYTE)'1')
#define META_FORMAT          1U

#define META_OFF_MAGIC       0U
#define META_OFF_FORMAT      4U
#define META_OFF_STATE       5U
#define META_OFF_SIZE        8U
#define META_OFF_CRC         12U

static DWORD read_dword(BYTE offset)
{
    DWORD value;

    value = (DWORD)iap_meta_read(offset) << 24;
    value |= (DWORD)iap_meta_read((BYTE)(offset + 1U)) << 16;
    value |= (DWORD)iap_meta_read((BYTE)(offset + 2U)) << 8;
    value |= (DWORD)iap_meta_read((BYTE)(offset + 3U));
    return value;
}

static BOOL write_dword(BYTE offset, DWORD value)
{
    if (!iap_meta_write(offset, BYTE3(value))) return 0;
    if (!iap_meta_write((BYTE)(offset + 1U), BYTE2(value))) return 0;
    if (!iap_meta_write((BYTE)(offset + 2U), BYTE1(value))) return 0;
    if (!iap_meta_write((BYTE)(offset + 3U), BYTE0(value))) return 0;
    return 1;
}

static BYTE metadata_state(void)
{
    BYTE state;

    if (iap_meta_read((BYTE)(META_OFF_MAGIC + 0U)) != META_MAGIC0) return UPDATE_STATE_EMPTY;
    if (iap_meta_read((BYTE)(META_OFF_MAGIC + 1U)) != META_MAGIC1) return UPDATE_STATE_EMPTY;
    if (iap_meta_read((BYTE)(META_OFF_MAGIC + 2U)) != META_MAGIC2) return UPDATE_STATE_EMPTY;
    if (iap_meta_read((BYTE)(META_OFF_MAGIC + 3U)) != META_MAGIC3) return UPDATE_STATE_EMPTY;
    if (iap_meta_read((BYTE)META_OFF_FORMAT) != META_FORMAT) return UPDATE_STATE_EMPTY;
    if (read_dword((BYTE)META_OFF_SIZE) != APP_SIZE) return UPDATE_STATE_EMPTY;

    state = iap_meta_read((BYTE)META_OFF_STATE);
    if ((state != UPDATE_STATE_IN_PROGRESS) &&
        (state != UPDATE_STATE_VERIFIED) &&
        (state != UPDATE_STATE_COMMITTED))
        return UPDATE_STATE_EMPTY;

    return state;
}

BYTE update_get_status(DWORD *image_size, DWORD *image_crc)
{
    BYTE state;

    state = metadata_state();
    if (state == UPDATE_STATE_EMPTY)
    {
        *image_size = 0UL;
        *image_crc = 0UL;
        return state;
    }

    *image_size = APP_SIZE;
    *image_crc = read_dword((BYTE)META_OFF_CRC);
    return state;
}

BOOL update_recovery_pending(void)
{
    BYTE state;

    state = metadata_state();
    return ((state == UPDATE_STATE_IN_PROGRESS) ||
            (state == UPDATE_STATE_VERIFIED));
}

BOOL update_write_allowed(void)
{
    return (metadata_state() == UPDATE_STATE_IN_PROGRESS);
}

BOOL update_begin(DWORD image_size, DWORD image_crc)
{
    if (image_size != APP_SIZE)
        return 0;
    if (!iap_meta_erase())
        return 0;

    /* Write all fields first and the magic last. A torn BEGIN is invalid. */
    if (!iap_meta_write((BYTE)META_OFF_FORMAT, (BYTE)META_FORMAT)) return 0;
    if (!iap_meta_write((BYTE)META_OFF_STATE, (BYTE)UPDATE_STATE_IN_PROGRESS)) return 0;
    if (!write_dword((BYTE)META_OFF_SIZE, image_size)) return 0;
    if (!write_dword((BYTE)META_OFF_CRC, image_crc)) return 0;
    if (!iap_meta_write((BYTE)(META_OFF_MAGIC + 0U), META_MAGIC0)) return 0;
    if (!iap_meta_write((BYTE)(META_OFF_MAGIC + 1U), META_MAGIC1)) return 0;
    if (!iap_meta_write((BYTE)(META_OFF_MAGIC + 2U), META_MAGIC2)) return 0;
    if (!iap_meta_write((BYTE)(META_OFF_MAGIC + 3U), META_MAGIC3)) return 0;

    return (metadata_state() == UPDATE_STATE_IN_PROGRESS);
}

BOOL update_verify(DWORD *actual_crc)
{
    DWORD expected_crc;
    BYTE state;

    state = metadata_state();
    if ((state != UPDATE_STATE_IN_PROGRESS) &&
        (state != UPDATE_STATE_VERIFIED))
        return 0;

    expected_crc = read_dword((BYTE)META_OFF_CRC);
    if (!iap_crc32(APP_LOGICAL_BEGIN, APP_SIZE, actual_crc))
        return 0;
    if (*actual_crc != expected_crc)
        return 0;

    if (state == UPDATE_STATE_IN_PROGRESS)
    {
        if (!iap_meta_write((BYTE)META_OFF_STATE, (BYTE)UPDATE_STATE_VERIFIED))
            return 0;
    }
    return 1;
}

BOOL update_commit(void)
{
    BYTE state;

    state = metadata_state();
    if (state == UPDATE_STATE_COMMITTED)
        return 1;
    if (state != UPDATE_STATE_VERIFIED)
        return 0;

    return iap_meta_write((BYTE)META_OFF_STATE, (BYTE)UPDATE_STATE_COMMITTED);
}
