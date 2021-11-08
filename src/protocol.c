#include "protocol.h"
#include <usb_serial.h>
#include "crc.h"

static packet_handler packet_handlers[] = {
	[0] = NULL,
	[MESSAGE_ESTOP] = NULL,
	[MESSAGE_DRIVE_PWM] = NULL,
	[MESSAGE_PWM_HIGH] = NULL,
};

static int packet_type_to_payload_size(uint16_t type) {

	if (type >= sizeof(packet_type_to_payload_size_table)) {
		return -1;
	}

	return packet_type_to_payload_size_table[type];

}

/**
 * Tries to receive at most one packet via USB Serial
 *
 * Does NOT block.
 * When a packet is received, a handler for the corresponding packet type
 * (if any is set using set_packet_handler()) is invoked.
 */
void try_receive_packet() {

	static const int payload_max_size = packet_max_size - 4;
	static union packet packet;
	static uint16_t *packet_type = ((uint16_t *) &packet);
	static uint8_t *buffer = (uint8_t *) &packet;
	static int buffer_size = 0;
	static int payload_size = 0;
	static int packet_size = 0;

	// to prevent this while loop to run indefinitely when new data are coming faster
	// than we are reading, let's limit the maximum number of read bytes
	// use 2 * packet_max_size to ensure we always process one whole packet (if there is one)
	for (int num_bytes_received = 0; num_bytes_received < 2 * packet_max_size; ++num_bytes_received) {

		// usb_serial_getchar is non-blocking
		int value = usb_serial_getchar();

		// no data in buffer, nothing to read
		if (value == -1) {
			break;
		}

		uint8_t byte = (uint8_t) value;

		// save byte to buffer and then increase buffer index
		buffer[buffer_size++] = byte;

		// once we have packet type we validate it and use it to determine the packet payload size
		if (buffer_size == 2) {
			payload_size = packet_type_to_payload_size(*((uint16_t *) &packet));
			packet_size = 2 + payload_size + 2;
			if (payload_size < 1 || payload_size > payload_max_size) {
				// reset buffer index on invalid packet type (drops packet type)
				buffer_size = 0;
			}
			continue;
		}

		// packet is complete
		if (buffer_size == packet_size) {

			// validate checksum
			uint16_t checksum = crc16(buffer, buffer_size);

			// if we calculate CRC over the whole packet (including the checksum) it must equal to 0
			if (checksum != 0) {
				// broken packet
				// reset buffer
				buffer_size = 0;
				continue;
			}

			packet_handler handler = packet_handlers[*packet_type];
			if (handler != NULL) {
				handler(&packet);
			}

			// reset buffer
			buffer_size = 0;
			// do not receive more than one packet
			break;

		}

	}

}

/**
 * Sets the handler that is invoked every time a packet of the given type
 * is received in try_receive_packet()
 *
 * @param type the packet type
 * @param handler the handler function with one argument which is a pointer to the received packet,
 *                the packet must not be accessed upon returning from the handler
 */
void set_packet_handler(enum packet_type type, packet_handler handler) {
	packet_handlers[type] = handler;
}

bool send_packet(const union packet *packet) {

	uint16_t type = *((uint16_t *) packet); // first two bytes are packet type
	uint32_t payload_size = packet_type_to_payload_size(type);

	if (payload_size == -1) {
		return -1;
	}

	// two bytes checksum field is at the end of packet
	uint16_t *checksum = (uint16_t *) (((uint8_t *) packet) + 2 + payload_size);

	*checksum = crc16((uint8_t *) packet, payload_size + 2);

	// send the whole packet (including two byte type and two bytes checksum)
	int result = usb_serial_write(packet, payload_size + 4);

	return result == payload_size;

}
