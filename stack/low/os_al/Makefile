
#
# Root Directory for project
#
TOP= ./../projects

#
# Include the general platform independent Makefile
#
include $(TOP)/Makefile.include.gen

#*****************************************************************************#
#                               Object FILE LIST                              #
#*****************************************************************************#
#OBJS=	$(BUILD_ROOT_APPL_OBJ)/bt_fw_os.o
OBJS = bt_fw_os.o 
#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#

# Define the Rules to build the core targets
all: CORE_TARGETS COPY_APPL_OBJS
#all: init_subdirs  

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

