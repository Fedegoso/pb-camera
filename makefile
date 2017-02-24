CC=gcc
CXX=g++
LINK=-pthread
OPTS=-g -Wall
OPENCV_LIBS=$(shell pkg-config --cflags --libs opencv)

all: cam_daemon cam_reader.o

cam_daemon: Trakking_final_bbb.cpp
	$(CXX) Trakking_final_bbb.cpp $(LINKS) $(OPENCV_LIBS) $(OPTS) -o cam_daemon

cam_reader.o: cam_reader.c cam_reader.h
	$(CC) cam_reader.c -c $(OPTS)

test: main_mmap.c cam_reader.o
	$(CC) main_mmap.c -c
	$(CC) main_mmap.o cam_reader.o $(LINKS) $(OPTS) -o test

clean:
	-rm *.o 2> /dev/null
	-rm finalcpu finalgpu mmap
