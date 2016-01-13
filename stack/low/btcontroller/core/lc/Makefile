
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
#OBJS= 	$(BUILD_ROOT_OBJ)/lc.o \
#		$(BUILD_ROOT_OBJ)/lc_1_2_utils.o \
#		$(BUILD_ROOT_OBJ)/lc_isr.o \
#		$(BUILD_ROOT_OBJ)/lc_scan.o \
#		$(BUILD_ROOT_OBJ)/lc_sync_links.o \
#		$(BUILD_ROOT_OBJ)/lc_task.o \
#		$(BUILD_ROOT_OBJ)/lc_rf.o \
#		$(BUILD_ROOT_OBJ)/lc_2_1.o \
#		$(BUILD_ROOT_OBJ)/lc_utils.o 

OBJS= 	lc.o \
		lc_1_2_utils.o \
		lc_isr.o \
		lc_scan.o \
		lc_sync_links.o \
		lc_task.o \
		lc_rf.o \
		lc_2_1.o \
		lc_utils.o 

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
all: CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS) 

#lc_isr.o : lc_isr.c
#	@$(CC) $(COMP_FLAGS_16) lc_isr.c -o lc_isr.o

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

