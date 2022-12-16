CXX = gcc

targets = blob_two_line

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
serial_dev.o: serial_dev.c
	$(CXX) -c serial_dev.c	

blob_two_line.o: blob_two_line.cpp
	g++ `pkg-config --cflags --libs opencv4` -c blob_two_line.cpp

blob_two_line: blob_two_line.o serial_dev.o
	g++ -o $@ $(CXXFLAGS) $(INCLUDES) $^ `pkg-config --cflags --libs opencv4`

clean:
	rm -rf *.o $(targets)
  
