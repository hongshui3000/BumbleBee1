
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
#OBJS= 	$(BUILD_ROOT_OBJ)/le_ll.o \
#		$(BUILD_ROOT_OBJ)/le_ll_isr.o \
#		$(BUILD_ROOT_OBJ)/le_hci_4_0.o \
#		$(BUILD_ROOT_OBJ)/le_ll_driver.o \
#		$(BUILD_ROOT_OBJ)/le_ll_general.o \
#		$(BUILD_ROOT_OBJ)/le_llc.o 

OBJS =
OBJS += 	le_ll.o
OBJS +=		le_ll_isr.o
OBJS +=		le_hci_4_0.o
OBJS +=		le_ll_driver.o
OBJS +=		le_ll_general.o
OBJS +=		le_llc.o

ifeq ($(findstring -DLE_HW_TEST, $(SYSTEM_LEVEL_DEF)), -DLE_HW_TEST)
OBJS +=		le_ll_test.o
endif

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
all: CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS) 

#le_ll_isr.o : le_ll_isr.c
#	@$(CC) $(COMP_FLAGS_16) le_ll_isr.c -o le_ll_isr.o

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#
