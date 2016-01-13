
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
#OBJS = 	$(BUILD_ROOT_APPL_OBJ)/mint_os.o \
#		$(BUILD_ROOT_APPL_OBJ)/mint_os_buffer.o \
#		$(BUILD_ROOT_APPL_OBJ)/mint_os_queue.o \
#		$(BUILD_ROOT_APPL_OBJ)/mint_os_stack.o \
#		$(BUILD_ROOT_APPL_OBJ)/mint_os_task.o \
#		$(BUILD_ROOT_APPL_OBJ)/mint_os_timer.o
		
OBJS = 	mint_os.o \
		mint_os_buffer.o \
		mint_os_queue.o \
		mint_os_stack.o \
		mint_os_task.o \
		mint_os_timer.o      
ifneq (,$(findstring -DUSE_NEW_MEM_ALLOC, $(SYSTEM_LEVEL_DEF)))
OBJS += mint_os_mem.o
endif

ASM_OBJS = 



#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
all: CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS) $(ASM_OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

