# Makefile

CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LIBS    = -lm

TARGET  = main

# All project sources live in src/
SRCS := $(shell find src -name '*.c')
OBJS := $(patsubst src/%.c,build/%.o,$(SRCS))

# Module objects = everything except the app's main.o
# (so tests can link against modules without colliding with main())
MODULE_OBJS := $(filter-out build/main.o,$(OBJS))

# Tests live in tests/, one executable per file
TEST_SRCS    := $(wildcard tests/test_*.c)
TEST_TARGETS := $(patsubst tests/%.c,build/%,$(TEST_SRCS))

.PHONY: all clean run test build

# Default target
all: build/$(TARGET)

build:
	@mkdir -p build

# Link the final executable
build/$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LIBS)

# Compile project .c → .o (supports subdirectories under src/)
build/%.o: src/%.c | build
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Compile and link each test against all modules.
# Tests include module headers as "../src/quad.h" etc.
build/test_%: tests/test_%.c $(MODULE_OBJS) | build
	$(CC) $(CFLAGS) -Isrc $< $(MODULE_OBJS) -o $@ $(LIBS)

test: $(TEST_TARGETS)
	@echo "── Running tests ──"
	@for t in $(TEST_TARGETS); do echo ""; echo "  $$t"; ./$$t || exit 1; done
	@echo ""
	@echo "── All tests passed ──"

run: build/$(TARGET)
	@./build/$(TARGET)

viz: build/$(TARGET)
	@./build/$(TARGET)
	@echo "── Launching 3D viz ──"
	python3 viz3d.py

clean:
	rm -rf build flight_log.csv

