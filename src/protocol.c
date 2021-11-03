#include "protocol.h"
#include <usb_serial.h>

static message_handler message_handlers[] = {
	[0] = NULL,
	[MESSAGE_TYPE_ESTOP] = NULL,
	[MESSAGE_TYPE_DRIVE_PWM] = NULL,
	[MESSAGE_TYPE_PWM_HIGH] = NULL,
};

static struct message msg;
static const int buffer_max_size = sizeof(union message_data);
static uint8_t *buffer = (uint8_t *) &msg.data;
static int buffer_size = 0;
static int data_size = 0;

static int message_type_to_data_size(uint8_t type) {

	if (type >= sizeof(message_data_size)) {
		return -1;
	}

	return message_data_size[type];

}

/**
 * Tries to receive at most one message via USB Serial
 *
 * Does NOT block.
 * When a message is received, a handler for the corresponding message type
 * (if any is set using set_message_handler())  is invoked.
 */
void try_receive_message() {

	// to prevent this while loop to run indefinitely when new data are coming faster
	// than we are reading, let's limit the maximum number of read bytes
	// use 2 * message_max_size to ensure we always process one whole message (if there is one)
	for (int num_bytes_received = 0; num_bytes_received < 2 * message_max_size; ++num_bytes_received) {

		// usb_serial_getchar is non-blocking
		int value = usb_serial_getchar();

		// no data in buffer, nothing to read
		if (value == -1) {
			break;
		}

		uint8_t byte = (uint8_t) value;

		if (msg.type == 0) {
			data_size = message_type_to_data_size(byte);
			// assume that only non-zero sizes are allowed
			if (data_size > 0 && data_size <= buffer_max_size) {
				// save message type
				msg.type = byte;
				// reset buffer index
				buffer_size = 0;
			}
			continue;
		}

		// save byte to buffer and then increase buffer index
		buffer[buffer_size++] = byte;

		// message is complete
		if (buffer_size == data_size) {
			message_handler handler = message_handlers[msg.type];
			if (handler != NULL) {
				handler(&msg.data);
			}
			// reset buffer
			msg.type = 0;
			// do not receive more than one message
			break;
		}

	}

}

/**
 * Sets the handler that is invoked every time a message of the given type
 * is received in try_receive_message()
 *
 * @param type the message type
 * @param handler the handler function with one argument which is a pointer to the received message,
 *                the message must not be accessed upon returning from the handler
 */
void set_message_handler(enum message_type type, message_handler handler) {
	message_handlers[type] = handler;
}
