/*
 * 3511 IPMC on i2c
 *
 *  LCR Embedded systems
 *  This code handles IPMI over IPMB protocol
 *  and implements the commands specified in the
 *  3511 specification
 *  This is an emulation layer
 * 
 *  [Version 1.0] implements get device id and
 *  get vso capabilities for now
 */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/bcd.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "system/rtc.h"
#include "trace.h"
#include "qemu/timer.h"

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

//various states of the IPMB state machine
//I2C_START_SEND is sent as event when master sends
// START condition followed by device address with
// write bit 0
// This is when the device starts getting data
typedef enum {
    IPMB_STATE_IDLE,
    IPMB_STATE_RECEIVING,
    IPMB_STATE_SENDING
} IPMBState;

struct IPMC3511State {
    I2CSlave parent_obj;

    // Message handling state
    uint8_t rx_buffer[MAX_IPMI_MSG_SIZE];  // Buffer for receiving message
    uint8_t tx_buffer[MAX_IPMI_MSG_SIZE];  // Buffer for transmitting response
    uint16_t rx_len;    // Number of bytes received
    uint16_t tx_len;    // Number of bytes to transmit
    uint16_t tx_pos;      // Current position in tx buffer
    bool tx_overflow;  // Flag to track overflow
    bool rx_overflow;  // Flag to track overflow

    // Device state
    uint8_t fru_state;
    IPMBState ipmb_state;
    uint8_t sensors[8];

    // VITA 46.11 state
    struct {
        uint8_t version_major;
        uint8_t version_minor;
        uint8_t capabilities;
    } vita;
};

static void reset_buffers(IPMC3511State *dev) {
    fprintf(g_debugFile, "Resetting buffers\n");
    fflush(g_debugFile);

    // Reset receive
    dev->rx_len = 0;
    dev->rx_overflow = false;
    memset(dev->rx_buffer, 0, MAX_IPMI_MSG_SIZE);

    // Reset transmit
    dev->tx_len = 0;
    dev->tx_pos = 0;
    dev->tx_overflow = false;
    memset(dev->tx_buffer, 0, MAX_IPMI_MSG_SIZE);
}

// Command Handlers
static void handle_get_device_id(IPMC3511State *dev, 
                               uint8_t *request, uint8_t req_len,
                               uint8_t *response, uint8_t *resp_len) 
{
    fprintf(g_debugFile, "handle_get_device_id");
    fflush(g_debugFile);

    response[0] = 0x00;     // Completion Code
    response[1] = 0x20;     // Device ID
    response[2] = 0x80;     // Device Revision
    response[3] = 0x02;     // Firmware Rev 1
    response[4] = 0x00;     // Firmware Rev 2
    response[5] = 0x02;     // IPMI Version 2.0
    response[6] = 0x1F;     // Device Support
    response[7] = 0x00;     // Manufacturer ID [7:0]
    response[8] = 0x00;     // Manufacturer ID [15:8]
    response[9] = 0x00;     // Manufacturer ID [23:16]
    response[10] = 0x00;    // Product ID [7:0]
    response[11] = 0x00;    // Product ID [15:8]

    *resp_len = 12;
}

static void handle_get_vso_capabilities(IPMC3511State *dev,
                                      uint8_t *request, uint8_t req_len,
                                      uint8_t *response, uint8_t *resp_len)
{
    fprintf(g_debugFile, "handle_get_vso_capabilities");
    fflush(g_debugFile);

    response[0] = 0x00;     // Completion Code
    response[1] = 0x03;     // VITA Group Extension
    response[2] = 0x01;     // VSO Specification Rev
    response[3] = 0xFF;     // Reserved
    response[4] = 0x00;     // Reserved

    *resp_len = 5;
}

// IPMI Command Handler function type
typedef void (*IPMICommandHandler)(IPMC3511State *dev, 
                                 uint8_t *request, uint8_t req_len,
                                 uint8_t *response, uint8_t *resp_len);

typedef struct {
    uint8_t netfn;
    uint8_t cmd;
    uint8_t group_ext;  // 0xFF means no group extension
    IPMICommandHandler handler;
} IPMICommandEntry;


// Command Table
// Handler function for each type of ipmi command
// sent over ipmb
static const IPMICommandEntry command_table[] = {
    {0x06, 0x01, 0xFF, handle_get_device_id},        // Get Device ID
    {0x2C, 0x00, 0x03, handle_get_vso_capabilities}, // Get VSO Capabilities
    {0x00, 0x00, 0x00, NULL}                         // End marker
};

static uint8_t calculate_checksum(uint8_t *data, int length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return -sum;
}

// process a full ipmi request
static void process_ipmi_request(IPMC3511State *dev) {
    uint8_t netfn = dev->rx_buffer[0] >> 2;
    uint8_t cmd = dev->rx_buffer[4];
    uint8_t *request_data = &dev->rx_buffer[6];
    uint8_t request_len = dev->rx_len - 7;  // Exclude header and checksum
    uint8_t group_ext = 0xFF;

    fprintf(g_debugFile, "PIPMIREQ: netfn:%d request_len:%d\n", netfn, request_len);
    fflush(g_debugFile);

    // If it's a group extension command, get the group ID
    if (netfn == 0x2C && request_len > 0) {
        group_ext = request_data[0];
        request_data++;
        request_len--;
    }

    // Find matching command handler
    const IPMICommandEntry *entry = command_table;
    while (entry->handler != NULL) {
        fprintf(g_debugFile, "walk: entry:%d:%d:%d netfn:%d cmd:%d group_ext:%d\n",
                entry->netfn, entry->cmd, entry->group_ext, netfn, cmd, group_ext);
        if (entry->netfn == netfn && entry->cmd == cmd &&
            (entry->group_ext == 0xFF || entry->group_ext == group_ext)) {
            
            uint8_t response_data[256];
            uint8_t response_len = 0;

            // Call command handler
            entry->handler(dev, request_data, request_len,
                         response_data, &response_len);

            // Build IPMB response packet
            dev->tx_buffer[0] = dev->rx_buffer[2];    // rqSA
            dev->tx_buffer[1] = ((netfn + 1) << 2);   // rqNetFn + 1
            dev->tx_buffer[2] = 0;                    // Checksum 1
            dev->tx_buffer[3] = 0x20;                 // rsSA (BMC)
            dev->tx_buffer[4] = dev->rx_buffer[4];    // Sequence
            dev->tx_buffer[5] = cmd;                  // Command

            // Add response data
            memcpy(&dev->tx_buffer[6], response_data, response_len);
            dev->tx_len = response_len + 6;

            // Calculate checksums
            dev->tx_buffer[2] = calculate_checksum(dev->tx_buffer, 2);
            dev->tx_buffer[dev->tx_len] = 
                calculate_checksum(&dev->tx_buffer[3], dev->tx_len - 3);
            dev->tx_len++;

            dev->tx_pos = 0;
            dev->ipmb_state = IPMB_STATE_SENDING;
            break;
        }
        entry++;
    }
}

static int ipmc3511_event(I2CSlave *i2c, enum i2c_event event)
{
    IPMC3511State *dev = IPMC3511(i2c);

    fprintf(g_debugFile, "ipmc3511_event. event:%d ipmb_state:%d\n", event, dev->ipmb_state);
    fflush(g_debugFile);

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_SEND_ASYNC:
        fprintf(g_debugFile, "I2C_START_SEND\n");
        fflush(g_debugFile);
        reset_buffers(dev);  // Reset buffers on new transaction
        dev->ipmb_state = IPMB_STATE_RECEIVING;
        break;
    
    case I2C_START_RECV:
        fprintf(g_debugFile, "I2C_START_RECV\n");
        fflush(g_debugFile);
        if (dev->ipmb_state == IPMB_STATE_RECEIVING) {
            if (!dev->rx_overflow) {
                process_ipmi_request(dev);
            }
            dev->ipmb_state = IPMB_STATE_SENDING;
        }
        break;
    
    case I2C_FINISH:
        fprintf(g_debugFile, "I2C_FINISH\n");
        fflush(g_debugFile);
        if (dev->tx_overflow || dev->rx_overflow) {
            reset_buffers(dev);
        }
        if (dev->ipmb_state != IPMB_STATE_SENDING) {
            dev->ipmb_state = IPMB_STATE_IDLE;
        }
        break;
    case I2C_NACK:
        fprintf(g_debugFile, "I2C_NACK\n");
        fflush(g_debugFile);
        // Handle NACK - typically means receiver isn't ready
        // Could reset state or implement retry logic
        dev->ipmb_state = IPMB_STATE_IDLE;
        break;
    }
    return 0;
}

static uint8_t ipmc3511_recv(I2CSlave *i2c)
{
    IPMC3511State *dev = IPMC3511(i2c);
    uint8_t data = 0xFF;

    if (dev->ipmb_state == IPMB_STATE_SENDING) {
        if (dev->tx_pos < dev->tx_len && dev->tx_len <= MAX_IPMI_MSG_SIZE) {
            data = dev->tx_buffer[dev->tx_pos++];
        } else if (dev->tx_len > MAX_IPMI_MSG_SIZE) {
            dev->tx_overflow = true;
            fprintf(g_debugFile, "TX buffer overflow!\n");
            fflush(g_debugFile);
        }
    }

    fprintf(g_debugFile, "ipmc3511_tx. data:0x%x tx_pos:%d tx_len:%d\n", 
            data, dev->tx_pos, dev->tx_len);
    fflush(g_debugFile);

    return data;
}

static int ipmc3511_send(I2CSlave *i2c, uint8_t data)
{
    IPMC3511State *dev = IPMC3511(i2c);

    fprintf(g_debugFile, "ipmc3511_rx. data:0x%x rx_len:%d\n", data, dev->rx_len);
    fflush(g_debugFile);

    if (dev->ipmb_state == IPMB_STATE_RECEIVING) {
        if (dev->rx_len < MAX_IPMI_MSG_SIZE) {
            dev->rx_buffer[dev->rx_len++] = data;
        } else {
            dev->rx_overflow = true;
            fprintf(g_debugFile, "RX buffer overflow!\n");
            fflush(g_debugFile);
        }
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
    fprintf(g_debugFile, "==>ipmc3511_realize\n");
    fflush(g_debugFile);
    ipmc3511_init(s);
    fprintf(g_debugFile, "<===ipmc3511_realize\n");
    fflush(g_debugFile);
}

static const VMStateDescription vmstate_ipmc3511 = {
    .name = "ipmc3511",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPMC3511State),
        VMSTATE_UINT8(fru_state, IPMC3511State),
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
