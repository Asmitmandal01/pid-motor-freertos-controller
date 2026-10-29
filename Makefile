CC      = gcc
CFLAGS  = -std=c11 -O2 -Wall -Wextra -Iinclude
LDFLAGS = -lm

SRC     = src/freertos_sim.c src/motor_plant.c src/pid_controller.c src/main.c
BIN     = build/pid_rtos_sim
TESTBIN = build/test_pid_rtos

.PHONY: all sim test plots clean

all: sim test plots

build:
	mkdir -p build results

sim: build
	$(CC) $(CFLAGS) $(SRC) -o $(BIN) $(LDFLAGS)
	./$(BIN)

test: build
	$(CC) $(CFLAGS) -DUNIT_TEST_BUILD $(SRC) tests/test_pid_rtos.c -o $(TESTBIN) $(LDFLAGS)
	./$(TESTBIN)

plots: sim
	python3 scripts/plot_results.py

clean:
	rm -rf build
