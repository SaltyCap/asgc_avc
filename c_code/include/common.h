#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <time.h>

// Constants
#define COUNTS_PER_REV 4096
#define STOP_THRESHOLD 30

// Time utilities
double get_time_sec(void);
void sleep_us(uint32_t microseconds);
void sleep_ms(uint32_t ms);

#endif
