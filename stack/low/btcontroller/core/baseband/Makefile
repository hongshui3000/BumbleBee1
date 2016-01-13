
#
# Root Directory for project
#
TOP= ./../../../projects

#
# Include the general platform independent Makefile
#
include $(TOP)/Makefile.include.gen

#*****************************************************************************#
#                               Object FILE LIST                              #
#*****************************************************************************#
#OBJS= 	$(BUILD_ROOT_APPL_OBJ)/bb_driver.o
OBJS = bb_driver.o
OBJS += bzdma.o
ifeq ($(findstring -DBZDMA_CODEC_CONVERT_TEST_EN, $(SYSTEM_LEVEL_DEF)), -DBZDMA_CODEC_CONVERT_TEST_EN)
OBJS += g711.o
endif


OBJS += bb_driver_3_0.o
OBJS += bb_driver_4_1.o
#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
# Define the Rules to build the core targets
all: CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

