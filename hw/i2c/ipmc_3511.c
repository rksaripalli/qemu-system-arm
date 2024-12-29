/*
 * 3511 IPMC on i2c
 *
 *  LCR Embedded systems
 *  This code handles IPMI over IPMB protocol
 *  and implements the commands specified in the
 *  3511 specification
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "system/rtc.h"
#include "trace.h"

// IPMI message format constants
#define IPMI_HEADER_SIZE     6  // Bytes before data
#define IPMI_CHECKSUM_SIZE   1  // Size of each checksum
#define MAX_IPMI_MSG_SIZE    256 

// IPMI Network Functions
#define NETFN_APP            0x06
#define NETFN_STORAGE        0x0A
#define NETFN_SENSOR_EVENT   0x04
#define NETFN_OEM           0x2E

// IPMI Commands
#define CMD_GET_DEVICE_ID    0x01
#define CMD_GET_SELF_TEST    0x04
#define CMD_GET_ACPI_POWER   0x07

// Completion Codes
#define CC_OK               0x00
#define CC_INVALID_CMD     0xC1
#define CC_LENGTH_ERROR    0xC7
#define CC_PARAM_ERROR     0xC9

/* Size of NVRAM including both the user-accessible area and the
 * secondary register area.
 */
#define NVRAM_SIZE 64

/* Flags definitions */
#define SECONDS_CH 0x80
#define HOURS_12   0x40
#define HOURS_PM   0x20
#define CTRL_OSF   0x20

static FILE *g_debugFile = NULL;

#define TYPE_IPMC3511 "ipmc3511"
OBJECT_DECLARE_SIMPLE_TYPE(IPMC3511State, IPMC3511)

struct IPMC3511State {
    I2CSlave parent_obj;

    // Message handling state
    uint8_t rx_buffer[MAX_IPMI_MSG_SIZE];  // Buffer for receiving message
    uint8_t tx_buffer[MAX_IPMI_MSG_SIZE];  // Buffer for transmitting response
    uint16_t rx_count;    // Number of bytes received
    uint16_t tx_count;    // Number of bytes to transmit
    uint16_t tx_pos;      // Current position in tx buffer
    bool receiving;       // Currently receiving a message
    bool transmitting;    // Currently transmitting a response

    // Device state
    uint8_t fru_state;
    uint8_t ipmb_state;
    uint8_t sensors[8];

    // VITA 46.11 state
    struct {
        uint8_t version_major;
        uint8_t version_minor;
        uint8_t capabilities;
    } vita;
};

// Calculate IPMI checksum
static uint8_t calculate_checksum(uint8_t *data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return -sum;
}

// Process received IPMI message and prepare response
static void process_ipmi_message(IPMC3511State *s)
{
    fprintf(g_debugFile, "process_ipmi_message\n");
    fflush(g_debugFile);
    if (s->rx_count < IPMI_HEADER_SIZE + IPMI_CHECKSUM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "IPMC: Message too short\n");
        return;
    }

    // Verify first checksum (bytes 0-1)
    uint8_t checksum1 = calculate_checksum(s->rx_buffer, 2);
    if (checksum1 != s->rx_buffer[2]) {
        qemu_log_mask(LOG_GUEST_ERROR, "IPMC: Header checksum error\n");
        return;
    }

    // Extract message fields
    uint8_t netfn = (s->rx_buffer[1] >> 2) & 0x3F;
    uint8_t rqLun = s->rx_buffer[1] & 0x03;
    uint8_t rqSA = s->rx_buffer[3];
    uint8_t rqSeq = s->rx_buffer[4];
    uint8_t cmd = s->rx_buffer[5];

    // Prepare response header
    s->tx_buffer[0] = rqSA;                      // Requester becomes responder
    s->tx_buffer[1] = ((netfn + 1) << 2) | rqLun; // Response NetFn
    s->tx_buffer[2] = 0;                         // Checksum 1 (calculated later)
    s->tx_buffer[3] = s->rx_buffer[0];          // Original responder's address
    s->tx_buffer[4] = rqSeq;                    // Original sequence number
    s->tx_buffer[5] = cmd;                      // Original command
    s->tx_buffer[6] = 0x00;                     // Completion code

    s->tx_count = 7;  // Start with header + completion code

    // Handle commands
    switch (netfn) {
        case NETFN_APP:  // App NetFn
            switch (cmd) {
                case CMD_GET_DEVICE_ID:
                    s->tx_buffer[7] = 0x00;      // Device ID
                    s->tx_buffer[8] = 0x02;      // Device revision
                    s->tx_buffer[9] = 0x82;      // Firmware revision 1
                    s->tx_buffer[10] = 0x3C;     // Firmware revision 2
                    s->tx_buffer[11] = 0x02;     // IPMI version
                    s->tx_buffer[12] = 0xBF;     // Additional device support
                    s->tx_buffer[13] = 0x28;     // Manufacturer ID LSB (Abaco)
                    s->tx_buffer[14] = 0x08;     // Manufacturer ID
                    s->tx_buffer[15] = 0x00;     // Manufacturer ID MSB
                    s->tx_count = 16;
                    break;

                case CMD_GET_SELF_TEST:
                    s->tx_buffer[7] = 0x55;      // Self test pass
                    s->tx_buffer[8] = 0x00;      // No errors
                    s->tx_count = 9;
                    break;
            }
            break;
    }

    // Calculate checksums
    s->tx_buffer[2] = calculate_checksum(s->tx_buffer, 2);
    s->tx_buffer[s->tx_count] = calculate_checksum(&s->tx_buffer[3], s->tx_count - 3);
    s->tx_count++;

    // Prepare for transmitting response
    s->tx_pos = 0;
    s->transmitting = true;

    qemu_log_mask(LOG_UNIMP, "IPMC: Processed message NetFn=0x%02x Cmd=0x%02x\n", netfn, cmd);
}

static int ipmc3511_event(I2CSlave *i2c, enum i2c_event event)
{
    IPMC3511State *s = IPMC3511(i2c);
    fprintf(g_debugFile, "ipmc3511_event\n");
    fflush(g_debugFile);
    switch (event) {
        case I2C_START_SEND:
            // Master is starting to send data
            s->rx_count = 0;
            s->receiving = true;
            s->transmitting = false;
            break;

        case I2C_START_RECV:
            // Master is starting to receive data
            s->tx_pos = 0;
            s->receiving = false;
            if (!s->transmitting) {
                // If we're not already transmitting a response,
                // process the received message
                process_ipmi_message(s);
            }
            break;

        case I2C_FINISH:
            // End of transaction
            if (s->receiving && s->rx_count > 0) {
                // Process message if we were receiving
                process_ipmi_message(s);
            }
            s->receiving = false;
            s->transmitting = false;
            break;

        case I2C_NACK:
            break;

        case I2C_START_SEND_ASYNC:
            break;
    }
    fprintf(g_debugFile, "ipmc3511_event\n");
    fflush(g_debugFile);
    return 0; 
}

static uint8_t ipmc3511_recv(I2CSlave *i2c)
{
    IPMC3511State *s = IPMC3511(i2c);

    fprintf(g_debugFile, "ipmc3511_recv\n");
    fflush(g_debugFile);

    if (s->transmitting && s->tx_pos < s->tx_count) {
        return s->tx_buffer[s->tx_pos++];
    }
    return 0xff;
}

static int ipmc3511_send(I2CSlave *i2c, uint8_t data)
{
    IPMC3511State *s = IPMC3511(i2c);

    fprintf(g_debugFile, "ipmc3511_send\n");
    fflush(g_debugFile);

    if (s->receiving && s->rx_count < MAX_IPMI_MSG_SIZE) {
        s->rx_buffer[s->rx_count++] = data;
    }
    return 0;
}


// Initialize device state
static void ipmc3511_init(IPMC3511State *s)
{
    s->fru_state = 0x01;        // M1 - Deactivated
    s->ipmb_state = 0x01;       // Enabled

    // Initialize sensors
    s->sensors[0] = 0xF0;  // VSO_FRU_STATE
    s->sensors[1] = 0xF1;  // VSO_IPMB_LINK
    s->sensors[2] = 0xF2;  // VSO_FRU_HEALTH
    s->sensors[3] = 0x02;  // VSO_VOLTAGE
    s->sensors[4] = 0xF3;  // VSO_TEMPERATURE
    s->sensors[5] = 0xF4;  // VSO_PAYLOAD_TEST
    s->sensors[6] = 0xF5;  // VSO_PAYLOAD_STATUS
    s->sensors[7] = 0xF6;  // FRU_MODE_SENSOR

    s->vita.version_major = 0x01;
    s->vita.version_minor = 0x00;
    s->vita.capabilities = 0x03;  // Tier 2 capabilities
}

static void ipmc3511_reset(DeviceState *dev)
{
    IPMC3511State *s = IPMC3511(dev);
    fprintf(g_debugFile, "ipmc3511_reset\n");
    fflush(g_debugFile);

    ipmc3511_init(s);
    fprintf(g_debugFile, "<===ipmc3511_reset\n");
    fflush(g_debugFile);
}

static void ipmc3511_realize(DeviceState *dev, Error **errp)
{
	IPMC3511State *s = IPMC3511(dev);
    fprintf(g_debugFile, "ipmc3511_realize\n");
    fflush(g_debugFile);
    ipmc3511_init(s);
    fprintf(g_debugFile, "ipmc3511_realize\n");
    fflush(g_debugFile);
}

static const VMStateDescription vmstate_ipmc3511 = {
    .name = "ipmc3511",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPMC3511State),
        VMSTATE_UINT8(fru_state, IPMC3511State),
        VMSTATE_UINT8(ipmb_state, IPMC3511State),
        VMSTATE_UINT8_ARRAY(sensors, IPMC3511State, 8),
        VMSTATE_END_OF_LIST()
    }
}; 

static void ipmc3511_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = ipmc3511_realize;
    k->event = ipmc3511_event;
    k->recv = ipmc3511_recv;
    k->send = ipmc3511_send;
    device_class_set_legacy_reset(dc, ipmc3511_reset);
    dc->vmsd = &vmstate_ipmc3511;

    /*
     * Open a file to log.
     * This is the debug logging
     */
    g_debugFile = fopen("ipmc3511.log","w");
    fprintf(g_debugFile, "ipmc3511_class_init\n");
    fflush(g_debugFile);
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
