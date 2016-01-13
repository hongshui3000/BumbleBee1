
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
#OBJS = 	$(BUILD_ROOT_APPL_OBJ)/crypto.o \
#		$(BUILD_ROOT_APPL_OBJ)/crypto11.o \
#		$(BUILD_ROOT_APPL_OBJ)/ecdh.o \
#		$(BUILD_ROOT_APPL_OBJ)/mpal.o 

OBJS = 	crypto.o \
		crypto11.o \
		ecdh.o \
		mpal.o 


#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#

# Define the Rules to build the core targets
#all: init_subdirs CORE_TARGETS COPY_OBJS
all: CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#                      RULES TO BUILD SUB DIRECTORIES
#*****************************************************************************#

# build sub dir

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

