#include "../include/uart.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int uart_fd = -1;

int uart_init(void) {
    if (uart_fd >= 0) {
        return 0; /* already open */
    }

    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0) {
        fprintf(stderr, "[UART] Failed to open %s: %s\n", UART_DEVICE, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(uart_fd, &tty) != 0) {
        fprintf(stderr, "[UART] tcgetattr error: %s\n", strerror(errno));
        close(uart_fd);
        uart_fd = -1;
        return -1;
    }

    cfsetospeed(&tty, UART_BAUD);
    cfsetispeed(&tty, UART_BAUD);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[UART] tcsetattr error: %s\n", strerror(errno));
        close(uart_fd);
        uart_fd = -1;
        return -1;
    }

    printf("[UART] Initialized %s\n", UART_DEVICE);
    fflush(stdout);
    return 0;
}

void uart_cleanup(void) {
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
        printf("[UART] Closed\n");
        fflush(stdout);
    }
}

int uart_send_servo_bytes(uint8_t low, uint8_t high) {
    if (uart_fd < 0) {
        fprintf(stderr, "[UART] Not initialized - call uart_init() first\n");
        return -1;
    }

    /* Compact Protocol - Set Target (4 bytes per channel):
     *   0x84, channel, target_low_7bits, target_high_7bits
     *
     * CH0 receives the command directly.
     * CH1 receives the mirrored pulse (opposite direction) so the two arm
     * servos move symmetrically, matching scoop_control.py behavior. */

    /* CH0 packet */
    uint8_t pkt0[4] = { 0x84, SERVO_CHANNEL_CH0, low & 0x7F, high & 0x7F };

    /* Compute mirrored target for CH1:
     * mirrored_us = 2 * PULSE_CENTER - ch0_us
     * Reconstruct ch0_us from the 7-bit bytes: target_qus = (high<<7)|low, us = qus/4 */
    unsigned int target_qus = ((unsigned int)(high & 0x7F) << 7) | (unsigned int)(low & 0x7F);
    unsigned int ch0_us = target_qus / 4u;
    unsigned int mir_us = (unsigned int)(2 * SERVO_PULSE_CENTER) - ch0_us;
    if (mir_us < (unsigned int)SERVO_PULSE_MIN) mir_us = SERVO_PULSE_MIN;
    if (mir_us > (unsigned int)SERVO_PULSE_MAX) mir_us = SERVO_PULSE_MAX;
    unsigned int mir_qus = mir_us * 4u;
    uint8_t mir_low  = (uint8_t)( mir_qus        & 0x7Fu);
    uint8_t mir_high = (uint8_t)((mir_qus >> 7u) & 0x7Fu);

    uint8_t pkt1[4] = { 0x84, SERVO_CHANNEL_CH1, mir_low, mir_high };

    /* Send CH0 */
    if (write(uart_fd, pkt0, sizeof(pkt0)) != (ssize_t)sizeof(pkt0)) {
        fprintf(stderr, "[UART] CH0 write error: %s\n", strerror(errno));
        return -1;
    }
    /* Send CH1 */
    if (write(uart_fd, pkt1, sizeof(pkt1)) != (ssize_t)sizeof(pkt1)) {
        fprintf(stderr, "[UART] CH1 write error: %s\n", strerror(errno));
        return -1;
    }

    tcdrain(uart_fd);

    printf("[UART] CH0=%u us  CH1=%u us (mirrored)\n", ch0_us, mir_us);
    fflush(stdout);
    return 0;
}
