# Makefile

CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LIBS    =

TARGET  = main

# Automatically find all .c files under src/ (including subdirectories)
SRCS := $(shell find src -name '*.c')

# Convert src/.../file.c → build/.../file.o
OBJS := $(patsubst src/%.c,build/%.o,$(SRCS))

# Default target
all: build/$(TARGET)

# Create build directory if needed
build:
	@mkdir -p build

.PHONY: all clean run

# Link the final executable
build/$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LIBS)

# Compile .c files → .o files (supports subdirectories)
build/%.o: src/%.c | build
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf build flight_log.csv

# Build and run
run: build/$(TARGET)
	@./build/$(TARGET)

