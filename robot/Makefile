CC = g++
CFLAGS = -Wall -Wextra -O2
LIB = -lbluetooth -pthread
DEPS = Robot.hpp BTHandler.hpp UARTHandler.hpp enums.hpp structs.hpp

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main: main.o Robot.o BTHandler.o UARTHandler.o
	$(CC) -o main main.o Robot.o BTHandler.o UARTHandler.o $(LIB)