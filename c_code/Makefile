CC = gcc
CFLAGS = -O3 -Wall -Wextra -Iinclude
LDFLAGS = -lm -lpthread

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = .

SRCS = $(SRC_DIR)/main.c $(SRC_DIR)/motor.c $(SRC_DIR)/i2c.c $(SRC_DIR)/common.c $(SRC_DIR)/imu.c $(SRC_DIR)/kalman.c $(SRC_DIR)/sensors.c
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