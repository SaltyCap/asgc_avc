CC = gcc
CFLAGS = -O3 -Wall -Wextra -Iinclude
LDFLAGS = -lm -lpthread

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = .

SRCS = $(SRC_DIR)/main.c $(SRC_DIR)/motor.c $(SRC_DIR)/i2c.c $(SRC_DIR)/pid.c $(SRC_DIR)/common.c
OBJS = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRCS))
TARGET = $(BIN_DIR)/mc2_coordinated

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(TARGET)

.PHONY: all clean