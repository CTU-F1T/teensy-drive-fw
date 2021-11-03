#ifndef _TEENSY_DRIVE_PROTOCOL_H
#define _TEENSY_DRIVE_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

struct message_data_bool {
	bool data;
};

struct message_data_drive_values {
	int16_t pwm_drive;
	int16_t pwm_angle;
};

struct message_data_pwm_high {
	uint16_t period_thr;
	uint16_t period_str;
};

enum message_type {
	MESSAGE_TYPE_ESTOP = 1,
	MESSAGE_TYPE_DRIVE_PWM = 2,
	MESSAGE_TYPE_PWM_HIGH = 3,
};

const static int message_data_size[] = {
	[0] = -1,
	[MESSAGE_TYPE_ESTOP] = sizeof(struct message_data_bool),
	[MESSAGE_TYPE_DRIVE_PWM] = sizeof(struct message_data_drive_values),
	[MESSAGE_TYPE_PWM_HIGH] = sizeof(struct message_data_pwm_high),
};

struct message {
	uint8_t type;
	union message_data {
		struct message_data_bool estop;
		struct message_data_drive_values drive_pwm;
		struct message_data_pwm_high pwm_high;
	} data;
};

#define message_max_size sizeof(struct message)

#ifdef static_assert
static_assert(sizeof(struct message_data_bool) == 1, "sizeof struct message_data_bool is not 1 byte");
static_assert(sizeof(struct message_data_drive_values) == 4, "sizeof struct message_data_drive_values is not 4 bytes");
static_assert(sizeof(struct message_data_pwm_high) == 4, "sizeof struct message_data_pwm_high is not 4 bytes");
static_assert(sizeof(union message_data) == 4, "sizeof union message_data struct is not 4 bytes");
static_assert(sizeof(struct message) == 6, "sizeof struct message is not 6 bytes");
#endif // static_assert

void try_receive_message();

typedef void (*message_handler)(void *);

void set_message_handler(enum message_type type, message_handler handler);

#ifdef __cplusplus
}
#endif

#endif // _TEENSY_DRIVE_PROTOCOL_H
