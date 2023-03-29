#include <Arduino.h>
#include "version.h"
#include "config.h"
#include "protocol.h"
#include "debug.h"

// Input capture FTM register values
#define FTM_SC_VALUE (FTM_SC_TOIE | FTM_SC_CLKS(1) | FTM_SC_PS(0))
#define FTM_CSC_RAISING (FTM_CSC_CHIE | FTM_CSC_ELSA)
#define FTM_CSC_FALLING (FTM_CSC_CHIE | FTM_CSC_ELSB)

// Input capture helper variables
static volatile uint32_t duty_cycle_channel_0 = 0;
static volatile uint32_t duty_cycle_channel_1 = 0;
static volatile uint32_t channel_0_done = 0;
static volatile uint32_t channel_1_done = 0;

// App state
static volatile boolean flagStop = false;
static volatile boolean flagManualOverride = true;
static volatile boolean vescMode = false;
elapsedMillis manual_blink_elapsed;
elapsedMillis last_drive_msg_reception_elapsed;
elapsedMillis last_ftm1_irq_elapsed;

// messages to send
struct packet_message_bool msg_estop = {
	.type = MESSAGE_ESTOP,
	.size = sizeof(packet_message_bool),
};
struct packet_message_pwm_high msg_pwm_high = {
	.type = MESSAGE_PWM_HIGH,
	.size = sizeof(packet_message_pwm_high),
};

// Measured values (v2):
//   TRIM STEERING - CALM STATE
//       left: 8247, right: 9622
//       => str_center_value = avg() = 8934; using 9361
//       => str boundaries: using corrected center +- 300
//
//   TRIM THROTTLE - CALM STATE
//       backward: 9618, forward: 8240
//       => thr_center_value = avg() = 8929; not driving for +- 250
//       => thr boundaries: using center +- 200
//
//   TRIM STEERING left
//       RATE MIN
//           left: 7799, right: 8950
//       RATE MAX
//           left: 6000, right: 11674
//       => str_lowerlimit = 6000
//
//   TRIM STEERING right
//       RATE MIN
//           left: 8901, right: 10600
//       RATE MAX
//           left: 6014, right: 11764
//       => str_upperlimit = 11764
//
//   TRIM THROTTLE backward
//       backward: 11312, forward: 6567
//           thr_upperlimit = 11312
//
//   TRIM THROTTLE forward
//       backward: 11312, forward: 6567
//           thr_lowerlimit = 6567
//

// The frequency of the input PWM signal is 91 Hz (measured by an oscilloscope).
// Also used as the frequency of the output (generated) PWM signal.
#define PWM_FREQUENCY 91

// Measured important PWM duty cycle values:
// (now defined in 'config.h')
//
//   #define pwm_str_lowerlimit    6000  //  10% duty cycle - V1 7000
//   #define pwm_str_center_value  9361  //  15% duty cycle - V1 8611
//   #define pwm_str_upperlimit   11764  //  20% duty cycle - V1 10684
//
//   #define pwm_thr_lowerlimit    6567  //  10% duty cycle - V1 6458
//   #define pwm_thr_center_value  8929  //  15% duty cycle - V1 8850
//   #define pwm_thr_upperlimit   11312  //  20% duty cycle - V1 11241
//

// pins definition
// Teensy 3.2 pin 6 == PTD4
#define PIN_STEERING_OUTPUT 6
// Teensy 3.2 pin 3 == PTA12
// not used directly (see setupFTM1)
#define PIN_STEERING_INPUT 3
// Teensy 3.2 pin 5 == PTD7
#define PIN_THROTTLE_OUTPUT 5
// Teensy 3.2 pin 4 == PTA13
// not used directly (see setupFTM1)
#define PIN_THROTTLE_INPUT 4
// Teensy 3.2 onboard orange LED pin 13 == PTC5
#define PIN_LED 13
// Teensy 3.2 pin 14 for VESC kill switch
#define PIN_KILL 14

void handleDrivePwmPacket(struct packet_message_drive_values *packet) {

	debug(Serial1.printf(
		"hDPP: drive=%d angle=%d\n",
		packet->payload.pwm_drive, packet->payload.pwm_angle
	));

	int16_t pwm_drive = packet->payload.pwm_drive;
	int16_t pwm_angle = packet->payload.pwm_angle;

	if (flagStop || flagManualOverride) {
		// skip message reception completely
		// do not update last_drive_msg_reception_elapsed
		return;
	}

	// we are in the VESC mode BUT pwm_drive value is out of the center (calm) range
	// interpret this as a request to disable the VESC mode
	if (vescMode && (pwm_thr_center_upper < pwm_drive || pwm_drive < pwm_thr_center_lower)) {
		NVIC_DISABLE_IRQ(IRQ_FTM1);
		analogWrite(PIN_THROTTLE_OUTPUT, pwm_thr_center_value);
		NVIC_ENABLE_IRQ(IRQ_FTM1);
		// klapajar:
		//   Experimentally it was found out that at least 80ms of "calm state" (but it might be possible
		//   with other valid values) is required for VESC to properly handle mode switching.
		delay(80);
		vescMode = false;
	}

	NVIC_DISABLE_IRQ(IRQ_FTM1);

	// only control throttle if we are not in the VESC mode
	if (!vescMode) {
		// TODO: replace with clap(value, lower_bound_incl, upper_bound_incl)
		if (pwm_drive < pwm_thr_lowerlimit) {
			analogWrite(PIN_THROTTLE_OUTPUT, pwm_thr_lowerlimit); // Safety lower limit
		} else if (pwm_drive > pwm_thr_upperlimit) {
			analogWrite(PIN_THROTTLE_OUTPUT, pwm_thr_upperlimit); // Safety upper limit
		} else {
			analogWrite(PIN_THROTTLE_OUTPUT, pwm_drive); // Incoming data
		}
	}

	// always control steering
	// TODO: replace with clap(value, lower_bound_incl, upper_bound_incl)
	if (pwm_angle < pwm_str_lowerlimit) {
		analogWrite(PIN_STEERING_OUTPUT, pwm_str_lowerlimit); // Safety lower limit
	} else if (pwm_angle > pwm_str_upperlimit) {
		analogWrite(PIN_STEERING_OUTPUT, pwm_str_upperlimit); // Safety upper limit
	} else {
		analogWrite(PIN_STEERING_OUTPUT, pwm_angle); // Incoming data
	}

	NVIC_ENABLE_IRQ(IRQ_FTM1);

	// update reception timestamp
	last_drive_msg_reception_elapsed = 0;

}

void handleEmergencyStopPacket(struct packet_message_bool *packet) {

	debug(Serial1.printf("handleEmergencyStopPacket: %d\n", packet->payload.data));

	flagStop = packet->payload.data;

	NVIC_DISABLE_IRQ(IRQ_FTM1);
	if (flagStop && !flagManualOverride) {
		digitalWrite(PIN_LED, HIGH); // turn on the LED (signals manual mode, blinking done as part of the ftm1_isr)
		digitalWrite(PIN_KILL, LOW);
		analogWrite(PIN_STEERING_OUTPUT, pwm_str_center_value);
		analogWrite(PIN_THROTTLE_OUTPUT, pwm_thr_center_value);
	}
	NVIC_ENABLE_IRQ(IRQ_FTM1);

	if (!flagStop) {
		flagManualOverride = false;
		vescMode = true;
		digitalWrite(PIN_LED, LOW); // turn off the LED (signals autonomous mode)
		digitalWrite(PIN_KILL, HIGH);
	}

}

void ftm1_isr() {

	static uint16_t capture_ovf_bits = 0;
	static uint32_t prev_cap_c0 = 0;
	static uint32_t prev_cap_c1 = 0;
	static bool next_edge_falling_c0 = false;
	static bool next_edge_falling_c1 = false;
	static int LED_state = LOW;

	bool overflowed = false;
	uint32_t cap_val, status_reg_copy = FTM1_STATUS; // so that it has constant value during the whole ISR
	uint32_t pwm_high_c0, pwm_high_c1;

	if (FTM1_SC & FTM_SC_TOF) {
		FTM1_SC = FTM_SC_VALUE;
		capture_ovf_bits++;
		overflowed = true;
	}

	// event on channel 0
	if (status_reg_copy & 0b01) {
		cap_val = FTM1_C0V;
		if (cap_val <= 0x8000 || !overflowed) {
			cap_val |= capture_ovf_bits << 16;
		} else {
			cap_val |= (capture_ovf_bits - 1) << 16;
		}
		next_edge_falling_c0 = !next_edge_falling_c0;
		FTM1_C0SC = next_edge_falling_c0 ? FTM_CSC_FALLING : FTM_CSC_RAISING;
		if (next_edge_falling_c0) {
			prev_cap_c0 = cap_val;
		} else {
			pwm_high_c0 = cap_val - prev_cap_c0;
			// duty_cycle_c0 = (PWM_FREQUENCY * pwm_high_c0 * 65535) / F_BUS;
			// the expr above replaced with the expr below to fix uint32_t overflow
			duty_cycle_channel_0 = (pwm_high_c0 * 12424) / 100000;

			// Steering
			if (!flagManualOverride &&
				(
					duty_cycle_channel_0 > pwm_str_center_upper || duty_cycle_channel_0 < pwm_str_center_lower
				)) { // V1 - > 9000; < 8300
				// TODO: this is replacement
				msg_estop.payload.data = true;
				send_packet(reinterpret_cast<union packet *>(&msg_estop));

				flagManualOverride = true;
			}

			if (flagManualOverride) {
				analogWrite(PIN_STEERING_OUTPUT, (int) duty_cycle_channel_0);
			}

			channel_0_done = 1;
		}
	}

	// event on channel 1
	if (status_reg_copy & 0b10) {
		cap_val = FTM1_C1V;
		if (cap_val <= 0x8000 || !overflowed) {
			cap_val |= capture_ovf_bits << 16;
		} else {
			cap_val |= (capture_ovf_bits - 1) << 16;
		}
		next_edge_falling_c1 = !next_edge_falling_c1;
		FTM1_C1SC = next_edge_falling_c1 ? FTM_CSC_FALLING : FTM_CSC_RAISING;
		if (next_edge_falling_c1) {
			prev_cap_c1 = cap_val;
		} else {
			pwm_high_c1 = cap_val - prev_cap_c1;
			// duty_cycle_c1 = (PWM_FREQUENCY * pwm_high_c1 * 65535) / F_BUS;
			// the expr above replaced with the expr below to fix uint32_t overflow
			duty_cycle_channel_1 = (pwm_high_c1 * 12424) / 100000;

			// Throttle
			if (!flagManualOverride &&
				(
					duty_cycle_channel_1 > pwm_thr_center_upper || duty_cycle_channel_1 < pwm_thr_center_lower
				)) {
				// TODO: this is replacement
				msg_estop.payload.data = true;
				send_packet(reinterpret_cast<union packet *>(&msg_estop));

				flagManualOverride = true;
			}

			if (flagManualOverride) {
				analogWrite(PIN_THROTTLE_OUTPUT, (int) duty_cycle_channel_1);
			}

			channel_1_done = 1;
		}
	}

	if (flagManualOverride && manual_blink_elapsed > 250) {
		if (LED_state == LOW) {
			LED_state = HIGH;
		} else {
			LED_state = LOW;
		}
		digitalWrite(PIN_LED, LED_state);
		manual_blink_elapsed = 0;
	}

	last_ftm1_irq_elapsed = 0;

}

/**
 * Setups FTM1 (FlexTimer Module 1)
 * so it can be used to read PWM signals from radio receiver for
 * ... steering ... input on pin 3
 * ... throttle ... input on pin 4
 */
void setupFTM1() {

	// Useful info:
	// [K20P64M72SF1RM]
	//   K20 Sub-Family Reference Manual (MK20DX64VLH7, MK20DX128VLH7, MK20DX256VLH7)
	//   Document Number: K20P64M72SF1RM
	//   note that Teensy 3.2 has Freescale/NXP MK20DX256VLH7 (Cortex-M4)
	//                            (Freescale was acquired by NXP in 2015)

	NVIC_DISABLE_IRQ(IRQ_FTM1);

	// Select alternative function 3 for pins 3 and 4 to be FTM1_CH0 and FTM1_CH1 respectively:
	//   Teensy pin 3 is PTA12 (configured via PORTA_PCR12)
	//   Teensy pin 4 is PTA13 (configured via PORTA_PCR13)
	//   see Chapter 11: Port control and interrupts (PORT)
	//       and also Chapter 10: Signal Multiplexing and Signal Descriptions
	CORE_PIN3_CONFIG = PORT_PCR_MUX(3); // alternative function 3 of pin 3 is FTM1_CH0
	CORE_PIN4_CONFIG = PORT_PCR_MUX(3); // alternative function 3 of pin 4 is FTM1_CH1

	// Configure FlexTimer Module 1 so that it can be used
	// to read input PWM signals of a known frequency (91 Hz, PWM_FREQUENCY)
	//   see Chapter 36: FlexTimer Module (FTM) in [K20P64M72SF1RM]

	// TODO: document the meaning of these values
	FTM1_SC = 0;
	FTM1_CNT = 0;
	FTM1_MOD = 0xFFFF;
	FTM1_SC = FTM_SC_VALUE;
	FTM1_MODE = FTM_MODE_WPDIS;

	// TODO: document the meaning of these values
	FTM1_FILTER = FTM_FILTER_CH0FVAL(2) | FTM_FILTER_CH1FVAL(2);
	FTM1_C0SC = FTM_CSC_RAISING;
	FTM1_C1SC = FTM_CSC_RAISING;

	// Change priority of FTM1's generated interrupt:
	//   Freescale/NXP MK20DX256VLH7 supports only 16 priority levels
	//   (even though Cortex-M4 can support up to 256 priority levels,
	//    but vendors/implementers can reduce their number)
	//   see Section 3.2.2.1 Interrupt priority levels in [K20P64M72SF1RM]
	//   see Section 4.2.7 in Cortex-M4 Generic User Guide (ARM DUI 0553B (ID012616))
	//   NOTE: teensy3/mk20dx128.c startup code in ResetHandler sets all interrupts to medium priority level (128)
	//   TODO: There was bug in the older main.cpp code:
	//           Priority 1 is not supported and setting such value results in priority 0.
	//           (Reason: priority is 8 bits but only the 4 high-order bits are implemented on MK20DX256VLH7,
	//            the 4 low-order bits are ignored.)
	NVIC_SET_PRIORITY(IRQ_FTM1, 0); // 0 = highest priority

	NVIC_ENABLE_IRQ(IRQ_FTM1);

}

void setup() {

	if (DEBUG) {
		// send debug messages over hardware UART
		// see https://www.pjrc.com/teensy/td_uart.html
		Serial1.setRX(0); // for Teensy 3.2 supported values are 0 or 21
		Serial1.setTX(1); // for Teensy 3.2 supported values are 1 or 5
		Serial1.begin(19200, SERIAL_8N1);
	}

	// this is config of PWM signal output pins
	analogWriteFrequency(PIN_STEERING_OUTPUT, PWM_FREQUENCY);
	analogWriteFrequency(PIN_THROTTLE_OUTPUT, PWM_FREQUENCY);
	analogWriteResolution(16);
	analogWrite(PIN_STEERING_OUTPUT, pwm_str_center_value);
	analogWrite(PIN_THROTTLE_OUTPUT, pwm_thr_center_value);

	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW); // turn off the LED

	pinMode(PIN_KILL, OUTPUT);
	digitalWrite(PIN_KILL, LOW);

	pinMode(2, INPUT); // TODO: What purpose has pin 2? Maybe it is connected to the PCB?

	setupFTM1();

	// nh.getHardware()->setBaud(115200);
	//
	// nh.initNode();
	//
	// nh.subscribe(sub_drive);
	// nh.subscribe(sub_stop);
	//
	// nh.advertise(pwm_high);
	// nh.advertise(estop_pub);
	//
	// // Following lines are effective only during first connecting. But why?
	// // Wait until actually connected
	// while (!nh.connected()) {
	// 	nh.spinOnce();
	// }
	//
	// nh.logwarn((String("Starting Teensy -- FW build ") + VERSION).c_str());

}

void usb_debug_loop() {

#define TEXT_64B "%04d part %02d 000000000000000000000000000000000000000000000000000"

	static int count = 0;
	static elapsedMillis debug_elapsed;

	if (debug_elapsed > 5000) {

		debug_elapsed = 0;

		count++;

		Serial1.printf(
			"\n%04d usb=%d baud=%d, dtr=%d, rts=%d\n",
			count, usb_configuration, Serial.baud(), Serial.dtr(), Serial.rts()
		);

		// if (count < 10) {

		char buffer[65];
		int r[15];
		elapsedMicros t;

		for (int i = 0; i < 15; ++i) {
			snprintf(buffer, 65, TEXT_64B, count, i + 1);
			r[i] = usb_serial_write(buffer, 64);
			// usb_serial_flush_output();
			// r[i] = Serial.print(buffer);
			// r[i] = Serial.printf(TEXT_64B, count, i + 1);
		}

		Serial1.printf(
			"time=%d\n"
			// "%d\n\n",
			"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
			(int) t,
			// r[0]
			r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14]
		);

		t = 0;

		// }

	}

};

#define IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100)

void print_bytes(const unsigned char *ptr, int size) {

	Serial1.printf("  bytes in %s: ", IS_BIG_ENDIAN ? "MSB" : "LSB");
	for (int i = 0; i < size; i++) {
		Serial1.printf(" 0x%02x", *(ptr + i));
	}
	Serial1.printf("\n");

}

void fake_messages_loop() {

	static elapsedMillis fake_estop_elapsed;
	static elapsedMillis fake_last_pwm_high_elapsed;
	static uint16_t value = 0;

	// if (fake_estop_elapsed > 2000) {
	// 	msg_estop->data = true;
	// 	// debug(elapsedMicros t1);
	// 	send_message(&raw_msg_estop);
	// 	print_bytes(reinterpret_cast<const unsigned char *>(&raw_msg_estop), sizeof(struct message));
	// 	// debug(Serial1.printf("send estop took %d us\n", (int) t1));
	// 	fake_estop_elapsed = 0;
	// }

	if (fake_last_pwm_high_elapsed > 2000) {

		debug(Serial1.printf("value=%hu \n", value));
		msg_pwm_high.payload.period_thr = value;
		msg_pwm_high.payload.period_str = UINT16_MAX - value;
		// debug(elapsedMicros t2);
		send_packet(reinterpret_cast<union packet *>(&msg_pwm_high));
		// debug(Serial1.printf("send pwm high took %d us\n", (int) t2));
		print_bytes(reinterpret_cast<const unsigned char *>(&msg_pwm_high), sizeof(msg_pwm_high));
		value++;

		msg_estop.payload.data = true;
		send_packet(reinterpret_cast<union packet *>(&msg_estop));
		print_bytes(reinterpret_cast<const unsigned char *>(&msg_estop), sizeof(msg_estop));

		fake_last_pwm_high_elapsed = 0;

	}

}

void loop() {

	// static elapsedMillis last_pwm_high_elapsed;

	if (channel_0_done && channel_1_done) {
		// TODO: this is replacement
		msg_pwm_high.payload.period_thr = duty_cycle_channel_1;
		msg_pwm_high.payload.period_str = duty_cycle_channel_0;
		send_packet(reinterpret_cast<union packet *>(&msg_pwm_high));
		channel_0_done = 0;
		channel_1_done = 0;
		// debug(Serial1.printf("pwm_high after %d ms\n", (int) last_pwm_high_elapsed));
		// last_pwm_high_elapsed = 0;
	}

	// TODO: this is replacement of nh.spinOnce();
	try_receive_packet();

	// drive command timeout
	NVIC_DISABLE_IRQ(IRQ_FTM1);
	if (vescMode && !flagManualOverride && !flagStop) {
		// analogWrite(PIN_STEERING_OUTPUT, pwm_str_center_value);
		analogWrite(PIN_THROTTLE_OUTPUT, 0);
	} else if (
		// handles drive messages stream lost in the autonomous mode
		(!flagManualOverride && !flagStop && last_drive_msg_reception_elapsed > 300)
		// handles RF PWM signal loss in the manual override mode
		|| (flagManualOverride && last_ftm1_irq_elapsed > 100)
		) {
		// set steering to straight and stop the car
		digitalWrite(PIN_KILL, LOW);
		analogWrite(PIN_STEERING_OUTPUT, pwm_str_center_value);
		analogWrite(PIN_THROTTLE_OUTPUT, pwm_thr_center_value);
	}
	NVIC_ENABLE_IRQ(IRQ_FTM1);

}

int main() {

	setup();

	set_packet_handler(MESSAGE_ESTOP, reinterpret_cast<packet_handler>(handleEmergencyStopPacket));
	set_packet_handler(MESSAGE_DRIVE_PWM, reinterpret_cast<packet_handler>(handleDrivePwmPacket));

	digitalWrite(PIN_KILL, HIGH);

	while (true) {
		// fake_messages_loop();
		loop();
		// yield(); // no need to call it as we are not interested in the events it produces
	}

	return 0;

}
