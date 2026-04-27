CC = gcc
CFLAGS = -O3 -Wall -Wextra -Iinclude
LDFLAGS = -lm -lpthread

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = .

SRCS = $(SRC_DIR)/main.c $(SRC_DIR)/runtime_state.c $(SRC_DIR)/control_loop.c $(SRC_DIR)/odometry.c $(SRC_DIR)/command_processor.c $(SRC_DIR)/motor.c $(SRC_DIR)/i2c.c $(SRC_DIR)/common.c $(SRC_DIR)/imu.c $(SRC_DIR)/sensors.c $(SRC_DIR)/logging.c $(SRC_DIR)/inference.c $(SRC_DIR)/uart.c
OBJS = $(patsubst src/%.c,obj/%.o,$(SRCS))
TARGET = asgc_motor_control

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

obj/%.o: src/%.c
	@mkdir -p obj
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf obj $(TARGET)

.PHONY: all clean
