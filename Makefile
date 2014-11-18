

# Do not forget to link or copy the library path of openni2 the usr/local/lib path as it is the dafult location to seach for the linked library

################################################
#Set the compiler
CXX= g++
# Enter Compiler flags here
CXXFLAGS=-c -g -Wall
# Compilation (add flags as needed)
CV_FLAGS=`pkg-config opencv --cflags` -I /usr/include/ni -I ~/OpenNI/OpenNI2-master/Include -I ~/OpenNI/OpenNI2-master/Samples/Common
# Linker flags (add flags as needed)
CV_LIBS=`pkg-config opencv --libs` -lglut -lOpenNI -lOpenNI2 -lGL -L ~/OpenNI/OpenNI2-master/Bin/x64-Release/ -L /home/ise/OpenNI/OpenNI2-master/Bin/x64-Release/OpenNI2/Drivers -L /home/ise/OpenNI/working-directory/test -L /home/ise/OpenNI/libfreenect/build/lib
#############################
# List all .cpp files in SOURCES, separated by spaces
SOURCES=main.cpp Ctracker.cpp Kalman.cpp HungarianAlg.cpp Detector.cpp Timer.cpp
OBJECTS=$(SOURCES:.cpp=.o)
# List all .h files in DEPS
DEPS=Ctracker.h Kalman.h HungarianAlg.h Detector.h Ctracker.h depthColor.h OniSampleUtilities.h Timer.h
EXECUTABLE=multiKinectTrack
######### DO NOT EDIT BELOW ####################

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CXX) $(OBJECTS) -o $@ $(CV_LIBS)

# Set the dependency for 'file'.o to 'file'.cpp as well as any files in $DEPS [file.h]
%.o: %.cpp $(DEPS)
	$(CC) $(CXXFLAGS) $(CV_FLAGS) $< -o $@

clean:
	rm -rf *.o $(EXECUTABLE)

