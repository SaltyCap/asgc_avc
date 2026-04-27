#ifndef UART_H
#define UART_H

#include <stdint.h>

/* Maestro USB Servo Controller — Compact Protocol channel config */
#define SERVO_CHANNEL_CH0   0       /* scoop arm 1 */
#define SERVO_CHANNEL_CH1   1       /* scoop arm 2 (mirrored) */

/* Servo pulse range (µs) */
#define SERVO_PULSE_MIN     500u
#define SERVO_PULSE_CENTER  1500u
#define SERVO_PULSE_MAX     2500u

/* UART device (Maestro virtual COM port) */
#define UART_DEVICE         "/dev/ttyAMA0"
#define UART_BAUD           B9600

int  uart_init(void);
void uart_cleanup(void);
int  uart_send_servo_bytes(uint8_t low, uint8_t high);

#endif /* UART_H */
