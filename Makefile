CC=g++
CFLAGS=-c -Wall -std=c++11
OPENCV=`pkg-config opencv --cflags --libs`
LDFLAGS=$(OPENCV) -lrt -lpthread
SOURCES=json11.cpp camera.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=camera

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
