CC = gcc
CXX = g++
OUTPUT_DIR ?= .
    
CFLAGS  ?= -lwiringPi -lpthread -Wdiscarded-qualifiers -Woverflow
CFLAGS += -I.

CXXFLAGS = -I. -lwiringPi -lpthread -fpermissive

TARGET = logger

IDIRS   +=  
LDIRS   +=  

DEPS    +=  \
		Icm20948Defs.h \
		icm20948.h \
		spi_if.h
		
CSRCS   +=  \
		icm20948.c \
		spi_if.c
    
OBJS := $(notdir $(CSRCS))
OBJS := $(addsuffix .o,$(OBJS))

all: $(TARGET) $(OBJS)

define CRule
$(OUTPUT_DIR)/$(notdir $(1).o): $(1) $(DEPS)
	$(CC) -c $(CFLAGS) $(IDIRS) $(1) -o $(OUTPUT_DIR)/$(notdir $(1)).o
endef
$(foreach _,$(CSRCS), $(eval $(call CRule,$_)))

%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

logger: logger.o $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS)

.PHONY: clean

clean:
	rm -f $(OBJS) $(TARGET)