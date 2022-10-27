CC = g++
FLAGS = -Wall -g -std=c++14

INC = -Iinclude

LIB = -L/usr/local/lib

connect : build/connect.o build/SerialInterface.o
	$(CC) $(FLAGS) -o connect build/SerialInterface.o build/connect.o ${LIB}

build/connect.o : src/connect.cpp include/SerialInterface.hpp src/SerialInterface.cpp
	$(CC) $(FLAGS) -o build/connect.o -c src/connect.cpp ${INC}

build/SerialInterface.o : src/SerialInterface.cpp include/SerialInterface.hpp include/Exception.hpp
	$(CC) $(FLAGS) -o build/SerialInterface.o -c src/SerialInterface.cpp ${INC}

clean:
	rm build/*.o connect