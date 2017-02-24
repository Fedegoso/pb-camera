all:
	g++ Trakking_final_bbb.cpp `pkg-config --cflags --libs opencv` -o finalcpu -pthread -g
	g++ cam_reader.c -c
	g++ main_mmap.c -c
	g++ main_mmap.o cam_reader.o -pthread -g -o mmap

clean:
	-rm *.o 2> /dev/null
	-rm finalcpu finalgpu mmap
