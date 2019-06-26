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
	eMD/Devices/Drivers/Icm20948 \
	eMD/Devices \
	eMD/DynamicProtocol \
	eMD/EmbUtils \
	eMD
LDIRS   +=  

DEPS    +=  \
	eMD/IDDVersion.h \
	eMD/InvBool.h \
	eMD/InvError.h \
	eMD/InvExport.h \
	eMD/Devices/Drivers/Icm20948/Icm20948.h \
	eMD/Devices/Drivers/Icm20948/Icm20948Augmented.h \
	eMD/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.h \
	eMD/Devices/Drivers/Icm20948/Icm20948AuxTransport.h \
	eMD/Devices/Drivers/Icm20948/Icm20948DataBaseControl.h \
	eMD/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h \
	eMD/Devices/Drivers/Icm20948/Icm20948DataConverter.h \
	eMD/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h \
	eMD/Devices/Drivers/Icm20948/Icm20948LoadFirmware.h \
	eMD/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h \
	eMD/Devices/Drivers/Icm20948/Icm20948SelfTest.h \
	eMD/Devices/Drivers/Icm20948/Icm20948Setup.h \
	eMD/Devices/Drivers/Icm20948/Icm20948Transport.h \
	eMD/Devices/Drivers/Icm20948/Icm20948Serif.h \
	eMD/Devices/Drivers/Icm20948/Icm20948Defs.h \
	eMD/Devices/Device.h \
	eMD/Devices/DeviceIcm20948.h \
	eMD/Devices/HostSerif.h \
	eMD/Devices/SensorConfig.h \
	eMD/Devices/SensorTypes.h \
	eMD/Devices/SerifHal.h \
	eMD/Devices/VSensorId.h \
	eMD/DynamicProtocol/DynProtocol.h \
	eMD/DynamicProtocol/DynProtocolTransport.h \
	eMD/DynamicProtocol/DynProtocolTransportUart.h \
	eMD/EmbUtils/CBinaryReader.h \
	eMD/EmbUtils/CBinaryWriter.h \
	eMD/EmbUtils/DataConverter.h \
	eMD/EmbUtils/ErrorHelper.h \
	eMD/EmbUtils/InvAssert.h \
	eMD/EmbUtils/InvBasicMath.h \
	eMD/EmbUtils/InvBits.h \
	eMD/EmbUtils/InvCksum.h \
	eMD/EmbUtils/InvFormat.h \
	eMD/EmbUtils/InvList.h \
	eMD/EmbUtils/InvPrintf.h \
	eMD/EmbUtils/InvProtocol.h \
	eMD/EmbUtils/InvScheduler.h \
	eMD/EmbUtils/Message.h \
	eMD/EmbUtils/RingBuffer.h \
	eMD/EmbUtils/InvQueue.h \
	eMD/EmbUtils/InvString.h \
	eMD/EmbUtils/Logger.h \
	eMD/EmbUtils/RingByteBuffer.h \
	eMD/EmbUtils/UartTxEmulator.h \
	eMD/Images/icm20948_img.dmp3a.h \
	eMD/VSensor/VSensor.h \
	eMD/VSensor/VSensorData.h \
	eMD/VSensor/VSensorConfig.h \
	eMD/VSensor/VSensorEvent.h  \
	eMD/VSensor/VSensorId.h \
	eMD/VSensor/VSensorListener.h \
	eMD/VSensor/VSensorSmartListener.h \
	eMD/VSensor/VSensorType.h \
	eMD/VSensor/VSensorConfig.h \
	eMD/VSensor/VSensorVersion.h
CSRCS   +=  \
	eMD/Devices/Drivers/Icm20948/Icm20948Augmented.c \
	eMD/Devices/Drivers/Icm20948/Icm20948AuxCompassAkm.c \
	eMD/Devices/Drivers/Icm20948/Icm20948AuxTransport.c \
	eMD/Devices/Drivers/Icm20948/Icm20948DataBaseControl.c \
	eMD/Devices/Drivers/Icm20948/Icm20948DataBaseDriver.c \
	eMD/Devices/Drivers/Icm20948/Icm20948DataConverter.c \
	eMD/Devices/Drivers/Icm20948/Icm20948Dmp3Driver.c \
	eMD/Devices/Drivers/Icm20948/Icm20948LoadFirmware.c \
	eMD/Devices/Drivers/Icm20948/Icm20948MPUFifoControl.c \
	eMD/Devices/Drivers/Icm20948/Icm20948SelfTest.c \
	eMD/Devices/Drivers/Icm20948/Icm20948Setup.c \
	eMD/Devices/Drivers/Icm20948/Icm20948Transport.c \
	eMD/Devices/DeviceIcm20948.c \
	eMD/Devices/HostSerif.c \
	eMD/Devices/Sensor.c \
	eMD/Devices/VSensorId.c \
	eMD/DynamicProtocol/DynProtocol.c \
	eMD/DynamicProtocol/DynProtocolTransportUart.c \
	eMD/EmbUtils/DataConverter.c \
	eMD/EmbUtils/ErrorHelper.c \
	eMD/EmbUtils/InvProtocol.c \
	eMD/EmbUtils/InvQueue.c \
	eMD/EmbUtils/InvScheduler.c \
	eMD/EmbUtils/Message.c \
	eMD/EmbUtils/RingByteBuffer.c \
	eMD/EmbUtils/InvBasicMath.c \
	eMD/EmbUtils/InvCksum.c \
	eMD/EmbUtils/InvFormat.c \
	eMD/EmbUtils/InvList.c \
	eMD/EmbUtils/InvPrintf.c \
	eMD/EmbUtils/Logger.c \
	eMD/EmbUtils/UartTxEmulator.c \
	eMD/VSensor/VSensor.c \
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
