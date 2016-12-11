CXX := g++
CXX_FLAGS := -Wall -g -std=c++14
L_PATH := -L/usr/local/lib
LIBS := -lopencv_highgui -lopencv_core -lopencv_imgcodecs -lopencv_imgproc
INC := -Iinclude -Iuser/local/include -Iusr/local/include/opencv
EXEC := final

O_FILES := tri.o
RM_FILES := $(EXEC) *.o *.gch

$(EXEC): $(EXEC).cpp $(O_FILES)
	$(CXX) -o $@ $^ $(CXX_FLAGS) $(INC) $(L_PATH) $(LIBS)

%.o: %.cpp %.hpp
	$(CXX) -c $^ $(CXX_FLAGS) $(INC) $(L_PATH) $(LIBS)

clean:
	rm -rf $(RM_FILES)
