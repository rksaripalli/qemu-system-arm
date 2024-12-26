/*
 * 3511 IPMC on i2c
 *
 *  LCR Embedded systems
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qom/object.h"
#include "system/rtc.h"
#include "trace.h"

/* Size of NVRAM including both the user-accessible area and the
 * secondary register area.
 */
#define NVRAM_SIZE 64

/* Flags definitions */
#define SECONDS_CH 0x80
#define HOURS_12   0x40
#define HOURS_PM   0x20
#define CTRL_OSF   0x20

#define TYPE_IPMC3511 "ipmc3511"
OBJECT_DECLARE_SIMPLE_TYPE(IPMC3511State, IPMC3511)

struct IPMC3511State {
    I2CSlave parent_obj;

    int64_t offset;
    uint8_t wday_offset;
    uint8_t nvram[NVRAM_SIZE];
    int32_t ptr;
    bool addr_byte;
};

static const VMStateDescription vmstate_ipmc3511 = {
    .name = "ipmc3511",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPMC3511State),
        VMSTATE_INT64(offset, IPMC3511State),
        VMSTATE_UINT8_V(wday_offset, IPMC3511State, 2),
        VMSTATE_UINT8_ARRAY(nvram, IPMC3511State, NVRAM_SIZE),
        VMSTATE_INT32(ptr, IPMC3511State),
        VMSTATE_BOOL(addr_byte, IPMC3511State),
        VMSTATE_END_OF_LIST()
    }
};

static void capture_current_time(IPMC3511State *s)
{
    /* Capture the current time into the secondary registers
     * which will be actually read by the data transfer operation.
     */
    struct tm now;
    qemu_get_timedate(&now, s->offset);
    s->nvram[0] = to_bcd(now.tm_sec);
    s->nvram[1] = to_bcd(now.tm_min);
    if (s->nvram[2] & HOURS_12) {
        int tmp = now.tm_hour;
        if (tmp % 12 == 0) {
            tmp += 12;
        }
        if (tmp <= 12) {
            s->nvram[2] = HOURS_12 | to_bcd(tmp);
        } else {
            s->nvram[2] = HOURS_12 | HOURS_PM | to_bcd(tmp - 12);
        }
    } else {
        s->nvram[2] = to_bcd(now.tm_hour);
    }
    s->nvram[3] = (now.tm_wday + s->wday_offset) % 7 + 1;
    s->nvram[4] = to_bcd(now.tm_mday);
    s->nvram[5] = to_bcd(now.tm_mon + 1);
    s->nvram[6] = to_bcd(now.tm_year - 100);
}

static void inc_regptr(IPMC3511State *s)
{
    /* The register pointer wraps around after 0x3F; wraparound
     * causes the current time/date to be retransferred into
     * the secondary registers.
     */
    s->ptr = (s->ptr + 1) & (NVRAM_SIZE - 1);
    if (!s->ptr) {
        capture_current_time(s);
    }
}

static int ipmc3511_event(I2CSlave *i2c, enum i2c_event event)
{
    IPMC3511State *s = IPMC3511(i2c);

    switch (event) {
    case I2C_START_RECV:
        /* In h/w, capture happens on any START condition, not just a
         * START_RECV, but there is no need to actually capture on
         * START_SEND, because the guest can't get at that data
         * without going through a START_RECV which would overwrite it.
         */
        capture_current_time(s);
        break;
    case I2C_START_SEND:
        s->addr_byte = true;
        break;
    default:
        break;
    }

    return 0;
}

static uint8_t ipmc3511_recv(I2CSlave *i2c)
{
    IPMC3511State *s = IPMC3511(i2c);
    uint8_t res;

    res  = s->nvram[s->ptr];

    //trace_ipmc3511_recv(s->ptr, res);

    inc_regptr(s);
    return res;
}

static int ipmc3511_send(I2CSlave *i2c, uint8_t data)
{
    IPMC3511State *s = IPMC3511(i2c);

    //trace_ipmc3511_send(s->ptr, data);

    if (s->addr_byte) {
        s->ptr = data & (NVRAM_SIZE - 1);
        s->addr_byte = false;
        return 0;
    }
    if (s->ptr < 7) {
        /* Time register. */
        struct tm now;
        qemu_get_timedate(&now, s->offset);
        switch(s->ptr) {
        case 0:
            /* TODO: Implement CH (stop) bit.  */
            now.tm_sec = from_bcd(data & 0x7f);
            break;
        case 1:
            now.tm_min = from_bcd(data & 0x7f);
            break;
        case 2:
            if (data & HOURS_12) {
                int tmp = from_bcd(data & (HOURS_PM - 1));
                if (data & HOURS_PM) {
                    tmp += 12;
                }
                if (tmp % 12 == 0) {
                    tmp -= 12;
                }
                now.tm_hour = tmp;
            } else {
                now.tm_hour = from_bcd(data & (HOURS_12 - 1));
            }
            break;
        case 3:
            {
                /* The day field is supposed to contain a value in
                   the range 1-7. Otherwise behavior is undefined.
                 */
                int user_wday = (data & 7) - 1;
                s->wday_offset = (user_wday - now.tm_wday + 7) % 7;
            }
            break;
        case 4:
            now.tm_mday = from_bcd(data & 0x3f);
            break;
        case 5:
            now.tm_mon = from_bcd(data & 0x1f) - 1;
            break;
        case 6:
            now.tm_year = from_bcd(data) + 100;
            break;
        }
        s->offset = qemu_timedate_diff(&now);
    } else if (s->ptr == 7) {
        /* Control register. */

        /* Ensure bits 2, 3 and 6 will read back as zero. */
        data &= 0xB3;

        /* Attempting to write the OSF flag to logic 1 leaves the
           value unchanged. */
        data = (data & ~CTRL_OSF) | (data & s->nvram[s->ptr] & CTRL_OSF);

        s->nvram[s->ptr] = data;
    } else {
        s->nvram[s->ptr] = data;
    }
    inc_regptr(s);
    return 0;
}

static void ipmc3511_reset(DeviceState *dev)
{
    IPMC3511State *s = IPMC3511(dev);

    /* The clock is running and synchronized with the host */
    s->offset = 0;
    s->wday_offset = 0;
    memset(s->nvram, 0, NVRAM_SIZE);
    s->ptr = 0;
    s->addr_byte = false;
}

static void ipmc3511_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = ipmc3511_event;
    k->recv = ipmc3511_recv;
    k->send = ipmc3511_send;
    device_class_set_legacy_reset(dc, ipmc3511_reset);
    dc->vmsd = &vmstate_ipmc3511;
}

static const TypeInfo ipmc3511_types[] = {
    {
        .name          = TYPE_IPMC3511,
        .parent        = TYPE_I2C_SLAVE,
        .instance_size = sizeof(IPMC3511State),
        .class_init    = ipmc3511_class_init,
    },
};

DEFINE_TYPES(ipmc3511_types)
