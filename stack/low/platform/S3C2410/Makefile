
#
# Root Directory for project
#
TOP= ./../../projects

#
# Include the general platform independent Makefile
#
include $(TOP)/Makefile.include.gen

#*****************************************************************************#
#                               Object FILE LIST                              #
#*****************************************************************************#

OBJS= 	S3C2410_utils.o \
	  S3C2410_task.o \
	  S3C2410_transport.o 

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
BB_DIR=./baseband
GPIO_DIR=./gpio
TIMER_DIR=./timer
UART_DIR=./uart
DMA_DIR=./dma
OTP_DIR=./otp
UDACODEC_DIR=./udacodec
4181_DIR = ./4181
EFUSE_DIR = ./efuse
MAILBOX_DIR = ./mailbox
POWER_CTRL_DIR = ./power_control
SPI_DIR = ./spi
NEW_IO_DIR = ./new_io
SCOREBOARD_DIR = ./scoreboard
SYSTEM_ON_DIR = ./system_on
I2C_DIR = ./i2c

# Define the Rules to build the core targets
all: init_subdirs CORE_TARGETS COPY_APPL_OBJS
#all: init_subdirs 

#*****************************************************************************#
#                      RULES TO BUILD SUB DIRECTORIES
#*****************************************************************************#

# build sub dir
init_subdirs:
	$(CD) $(BB_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(GPIO_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(TIMER_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(UART_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(DMA_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(OTP_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(4181_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(EFUSE_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(MAILBOX_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(POWER_CTRL_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(SPI_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(NEW_IO_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(SCOREBOARD_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(SYSTEM_ON_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(I2C_DIR) && $(MAKE) all
	$(CD) ..			
	@echo init_subdirs finished

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

