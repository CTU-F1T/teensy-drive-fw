#ifndef _TEENSY_DRIVE_PROTOCOL_H
#define _TEENSY_DRIVE_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

// packet
//   min size = 2 + 1 + 2 = 5 bytes
//   max size = 2 + 4 + 2 = 8 bytes
//
//   structure
//     uint8_t type
//     uint8_t size
//     uint8_t payload[1 - 4]
//     uint16_t checksum // big endian (MSB first)
//

struct message_bool {
	bool data;
};

struct packet_message_bool {
	uint8_t type;
	uint8_t size;
	struct message_bool payload;
	uint16_t checksum; // big endian (MSB first)
} __attribute__((__packed__));

struct message_drive_values {
	int16_t pwm_drive;
	int16_t pwm_angle;
};

struct packet_message_drive_values {
	uint8_t type;
	uint8_t size;
	struct message_drive_values payload;
	uint16_t checksum; // big endian (MSB first)
} __attribute__((__packed__));

struct message_pwm_high {
	uint16_t period_thr;
	uint16_t period_str;
};

struct packet_message_pwm_high {
	uint8_t type;
	uint8_t size;
	struct message_pwm_high payload;
	uint16_t checksum; // big endian (MSB first)
} __attribute__((__packed__));

struct message_version {
	char data[50];
};

struct packet_message_version {
	uint8_t type;
	uint8_t size;
	struct message_version payload;
	uint16_t checksum; // big endian (MSB first)
} __attribute__((__packed__));

union packet {
	struct packet_message_bool estop;
	struct packet_message_drive_values drive_pwm;
	struct packet_message_pwm_high pwm_high;
	struct packet_message_version version;
};

enum packet_type {
	MESSAGE_ESTOP = 1,
	MESSAGE_DRIVE_PWM = 2,
	MESSAGE_PWM_HIGH = 3,
	MESSAGE_VERSION = 4,
};

static const int packet_type_to_payload_size_table[] = {
	[0] = -1,
	[MESSAGE_ESTOP] = sizeof(struct message_bool),
	[MESSAGE_DRIVE_PWM] = sizeof(struct message_drive_values),
	[MESSAGE_PWM_HIGH] = sizeof(struct message_pwm_high),
	[MESSAGE_VERSION] = sizeof(struct message_version),
};

#define packet_max_size sizeof(union packet)

#ifdef static_assert
static_assert(
	sizeof(struct message_bool) == 1,
	"sizeof struct message_bool is not 1 byte"
);
static_assert(
	sizeof(struct packet_message_bool) == 5,
	"sizeof struct packet_message_bool is not 1 byte"
);
static_assert(
	sizeof(struct message_drive_values) == 4,
	"sizeof struct message_drive_values is not 4 bytes"
);
static_assert(
	sizeof(struct packet_message_drive_values) == 8,
	"sizeof struct packet_message_drive_values is not 8 bytes"
);
static_assert(
	sizeof(struct message_pwm_high) == 4,
	"sizeof struct message_pwm_high is not 4 bytes"
);
static_assert(
	sizeof(struct packet_message_pwm_high) == 8,
	"sizeof struct packet_message_pwm_high is not 8 bytes"
);
static_assert(
	sizeof(union packet) == 8,
	"sizeof union packet struct is not 8 bytes"
);
#endif // static_assert

void try_receive_packet();

typedef void (*packet_handler)(const union packet *packet);

void set_packet_handler(enum packet_type type, packet_handler handler);

bool send_packet(union packet *packet);

#ifdef __cplusplus
}
#endif

#endif // _TEENSY_DRIVE_PROTOCOL_H
