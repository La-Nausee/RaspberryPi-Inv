CC = gcc
CPP = g++
OUTPUT_DIR ?= .
#static library use 'ar' command   
AR = ar
    
CFLAGS  ?= $(FLAGS) -pipe -std=c99 -fdata-sections -ffunction-sections -Wall -Wextra -Wno-unused-parameter -fsigned-char \
	-fno-strict-aliasing -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -Wchar-subscripts \
	-Wcomment -Wformat=2 -Wimplicit-int -Wmain -Wparentheses -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused -Wuninitialized -Wunknown-pragmas \
	-Wundef -Wbad-function-cast -Wwrite-strings -Wsign-compare -Waggregate-return -Wmissing-declarations -Wformat -Wmissing-format-attribute \
	-Wno-deprecated-declarations -Wpacked -Wunreachable-code --param max-inline-insns-single=500	 
CFLAGS += -I.

CPPFLAGS = -I. -lwiringPi -lpthread -fpermissive -Werror=implicit-function-declaration

#TARGET = libeMD-icm20948.a  
TARGET = icm_test  

IDIRS   +=  \
	Invn/Devices/Drivers/Icm20948 \
	Invn/Devices \
	Invn/DynamicProtocol \
	Invn/EmbUtils \
	Invn
LDIRS   +=  

DEPS    +=  \
	Invn/IDDVersion.h \
	Invn/InvBool.h \
	Invn/InvError.h \
	Invn/InvExport.h \
	Invn/Devices/Drivers/Icm20948/Icm20948.h \
	Invn/Devices/Drivers/Icm20948/Icm20948Augmented.h \
	Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.h \
	Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.h \
	Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.h \
	Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h \
	Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.h \
	Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h \
	Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.h \
	Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h \
	Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.h \
	Invn/Devices/Drivers/Icm20948/Icm20948Setup.h \
	Invn/Devices/Drivers/Icm20948/Icm20948Transport.h \
	Invn/Devices/Drivers/Icm20948/Icm20948Serif.h \
	Invn/Devices/Drivers/Icm20948/Icm20948Defs.h \
	Invn/Devices/Device.h \
	Invn/Devices/DeviceIcm20948.h \
	Invn/Devices/HostSerif.h \
	Invn/Devices/SensorConfig.h \
	Invn/Devices/SensorTypes.h \
	Invn/Devices/SerifHal.h \
	Invn/Devices/VSensorId.h \
	Invn/DynamicProtocol/DynProtocol.h \
	Invn/DynamicProtocol/DynProtocolTransport.h \
	Invn/DynamicProtocol/DynProtocolTransportUart.h \
	Invn/EmbUtils/CBinaryReader.h \
	Invn/EmbUtils/CBinaryWriter.h \
	Invn/EmbUtils/DataConverter.h \
	Invn/EmbUtils/ErrorHelper.h \
	Invn/EmbUtils/InvAssert.h \
	Invn/EmbUtils/InvBasicMath.h \
	Invn/EmbUtils/InvBits.h \
	Invn/EmbUtils/InvCksum.h \
	Invn/EmbUtils/InvFormat.h \
	Invn/EmbUtils/InvList.h \
	Invn/EmbUtils/InvPrintf.h \
	Invn/EmbUtils/InvProtocol.h \
	Invn/EmbUtils/InvScheduler.h \
	Invn/EmbUtils/Message.h \
	Invn/EmbUtils/RingBuffer.h \
	Invn/EmbUtils/InvQueue.h \
	Invn/EmbUtils/InvString.h \
	Invn/EmbUtils/Logger.h \
	Invn/EmbUtils/RingByteBuffer.h \
	Invn/EmbUtils/UartTxEmulator.h \
	Invn/Images/icm20948_img.dmp3a.h \
	Invn/VSensor/VSensor.h \
	Invn/VSensor/VSensorData.h \
	Invn/VSensor/VSensorConfig.h \
	Invn/VSensor/VSensorEvent.h  \
	Invn/VSensor/VSensorId.h \
	Invn/VSensor/VSensorListener.h \
	Invn/VSensor/VSensorSmartListener.h \
	Invn/VSensor/VSensorType.h \
	Invn/VSensor/VSensorConfig.h \
	Invn/VSensor/VSensorVersion.h
CSRCS   +=  \
	Invn/Devices/Drivers/Icm20948/Icm20948Augmented.c \
	Invn/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.c \
	Invn/Devices/Drivers/Icm20948/Icm20948AuxTransport.c \
	Invn/Devices/Drivers/Icm20948/Icm20948DataBaseControl.c \
	Invn/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.c \
	Invn/Devices/Drivers/Icm20948/Icm20948DataConverter.c \
	Invn/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.c \
	Invn/Devices/Drivers/Icm20948/Icm20948LoadFirmware.c \
	Invn/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.c \
	Invn/Devices/Drivers/Icm20948/Icm20948SelfTest.c \
	Invn/Devices/Drivers/Icm20948/Icm20948Setup.c \
	Invn/Devices/Drivers/Icm20948/Icm20948Transport.c \
	Invn/Devices/DeviceIcm20948.c \
	Invn/Devices/HostSerif.c \
	Invn/Devices/Sensor.c \
	Invn/Devices/VSensorId.c \
	Invn/DynamicProtocol/DynProtocol.c \
	Invn/DynamicProtocol/DynProtocolTransportUart.c \
	Invn/EmbUtils/DataConverter.c \
	Invn/EmbUtils/ErrorHelper.c \
	Invn/EmbUtils/InvProtocol.c \
	Invn/EmbUtils/InvQueue.c \
	Invn/EmbUtils/InvScheduler.c \
	Invn/EmbUtils/Message.c \
	Invn/EmbUtils/RingByteBuffer.c \
	Invn/EmbUtils/InvBasicMath.c \
	Invn/EmbUtils/InvCksum.c \
	Invn/EmbUtils/InvFormat.c \
	Invn/EmbUtils/InvList.c \
	Invn/EmbUtils/InvPrintf.c \
	Invn/EmbUtils/Logger.c \
	Invn/EmbUtils/UartTxEmulator.c \
	Invn/VSensor/VSensor.c \
	icm_sleep.c

    
OBJS := $(notdir $(CSRCS))
OBJS := $(addsuffix .o,$(OBJS))
OBJS := $(addprefix $(OUTPUT_DIR)/objs/,$(OBJS))

all: $(TARGET) $(OBJS)
#%.o: %.c
#	$(CC) $(CFLAGS) -c $< -o $@
define CRule
$(OUTPUT_DIR)/objs/$(notdir $(1).o): $(1) $(DEPS)
	$(CC) -c $(CFLAGS) $(IDIRS) $(1) -o $(OUTPUT_DIR)/objs/$(notdir $(1)).o
endef
$(foreach _,$(CSRCS), $(eval $(call CRule,$_)))

%.o: %.cpp
	$(CPP) -c -o $@ $< $(CPPFLAGS)

#$(TARGET):$(OBJS)
#	${AR} rv ${TARGET} $? 
icm_test: icm_test.o spi_if.o $(OBJS)
	$(CPP) -o $@ $^ $(CPPFLAGS)

.PHONY: clean

clean:
	rm -f $(OBJS) $(TARGET) *.o icm_test 
