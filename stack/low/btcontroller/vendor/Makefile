
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
OBJS= 
SUBDIRS = \
	hci \
	lmp \
	pta \
	plc
ifneq (,$(findstring -DBT2FM_INDIRECT_ACCESS, $(SYSTEM_LEVEL_DEF)))
SUBDIRS += fm
endif
ifneq (,$(findstring -DCONFIG_TV_FEATURE, $(SYSTEM_LEVEL_DEF)))
SUBDIRS += tv
endif

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
HCI_DIR=./hci
LMP_DIR=./lmp
PTA_DIR=./pta
PLC_DIR=./plc

# Define the Rules to build the core targets
#all: init_subdirs CORE_TARGETS COPY_OBJS
#all: init_subdirs 
all: MAKE_SUBDIRS

#*****************************************************************************#
#                      RULES TO BUILD SUB DIRECTORIES
#*****************************************************************************#

# build sub dir
init_subdirs:
	$(CD) $(HCI_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(LMP_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(PTA_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(PLC_DIR) && $(MAKE) all
	$(CD) ..
	
#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

