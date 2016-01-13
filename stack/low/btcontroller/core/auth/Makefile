
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
#OBJS = 	$(BUILD_ROOT_OBJ)/bz_auth.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_hci.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_hci_2_1.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_lmp_2_1.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_internal_2_1.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_internal.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_lkdb.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_lmp.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_mlk.o \
#		$(BUILD_ROOT_OBJ)/bz_auth_top_stt.o 

OBJS = 	bz_auth.o \
		bz_auth_hci.o \
		bz_auth_hci_2_1.o \
		bz_auth_lmp_2_1.o \
		bz_auth_internal_2_1.o \
		bz_auth_internal.o \
		bz_auth_lkdb.o \
		bz_auth_lmp.o \
		bz_auth_mlk.o \
		bz_auth_top_stt.o 
#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
all: CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS) 

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

