CC = gcc

CFLAGS = -Wall -g

OBJ += main.o serial.o sensors.o trouble_code_reader.o topwork.o
BIN = ScanTool.exe

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) -o $(BIN) $(OBJ) $(LIBS)

release:
	make RELEASE=1

all: $(BIN)

clean:
	rm -f $(OBJ)

veryclean: clean
	rm -f $(BIN)

main.o: main.c globals.h serial.h
	$(CC) $(CFLAGS) -c main.c

serial.o: serial.c globals.h serial.h
	$(CC) $(CFLAGS) -c serial.c

sensors.o: sensors.c globals.h serial.h sensors.h
	$(CC) $(CFLAGS) -c sensors.c

trouble_code_reader.o: trouble_code_reader.c globals.h serial.h trouble_code_reader.h
	$(CC) $(CFLAGS) -c trouble_code_reader.c

