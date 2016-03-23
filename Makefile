all:future_net
future_net:route.o io.o future_net.o
	g++ -g -o future_net route.o io.o future_net.o
	cp future_net ./Debug/
future_net.o:future_net.cpp route.h lib/lib_io.h lib/lib_time.h
	g++ -g -c future_net.cpp 
io.o:io.cpp
	g++ -g -c io.cpp
route.o:route.cpp route.h lib/lib_record.h
	g++ -g -c route.cpp 
clean:
	rm *.o future_net
