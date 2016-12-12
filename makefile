CC = g++
SRCDIR = src
BUILDDIR = build
TARGET = gui

SRCEXT := cpp
SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT)) #$(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
CPPFLAGS = -Wall -g -std=c++11
LIB := -lopencv_highgui -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lm
INC := -I include -I /usr/local/include -I /usr/local/include/opencv

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@echo " Linking..."
	@echo " $(CC) $^ -o $(TARGET) $(LIB)"; $(CC) $^ -o $(TARGET) $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CPPFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CPPFLAGS) $(INC) -c -o $@ $<

clean:
	@echo " Cleaning...";
	@echo " $(RM) -r $(BUILDDIR) $(TARGET)"; $(RM) -r $(BUILDDIR) $(TARGET)
