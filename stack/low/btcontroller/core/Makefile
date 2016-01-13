
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
#OBJS= 	$(BUILD_ROOT_OBJ)/bt_fw_stack_config.o \
#		$(BUILD_ROOT_OBJ)/bt_fw_stack_init.o \
#		$(BUILD_ROOT_OBJ)/bz_bluetooth.o \
#		$(BUILD_ROOT_OBJ)/btc_2_1_init.o \
#		$(BUILD_ROOT_OBJ)/btc_init.o

OBJS =
OBJS += bt_fw_stack_init.o
OBJS += bz_bluetooth.o
OBJS += btc_2_1_init.o
OBJS += btc_init.o
OBJS += btc_3_0_init.o
       
#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
BASEBAND_DIR=./baseband
HCI_DIR=./hci
HCI_TD_DIR=./hci_td
LC_DIR=./lc
LMP_DIR=./lmp
AUTH_DIR=./auth
UTILS_DIR=./utils
LL_DIR=./ll
BT_3DD_DIR=./bt_3dd
BT_SECURE_CONN_DIR=./secure_conn
MWS_DIR=./mws
PHY_INIT_DIR=./phy_init

# Define the Rules to build the core targets
ifeq ($(findstring -DLE_MODE_EN, $(SYSTEM_LEVEL_DEF)), -DLE_MODE_EN)
    ifeq ($(findstring -D_3DD_FUNCTION_SUPPORT_, $(SYSTEM_LEVEL_DEF)), -D_3DD_FUNCTION_SUPPORT_)
        all: init_subdirs init_le_subdirs init_3dd_subdirs CORE_TARGETS COPY_OBJS init_mws_subdirs
    else  
        all: init_subdirs init_le_subdirs CORE_TARGETS COPY_OBJS init_mws_subdirs
    endif
else
    ifeq ($(findstring -D_3DD_FUNCTION_SUPPORT_, $(SYSTEM_LEVEL_DEF)), -D_3DD_FUNCTION_SUPPORT_)
        all: init_subdirs init_3dd_subdirs CORE_TARGETS COPY_OBJS init_mws_subdirs
    else
        all: init_subdirs CORE_TARGETS COPY_OBJS init_mws_subdirs
    endif
endif


#*****************************************************************************#
#                      RULES TO BUILD SUB DIRECTORIES
#*****************************************************************************#

# build sub dir
init_subdirs:
	$(CD) $(BASEBAND_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(HCI_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(HCI_TD_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(LC_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(LMP_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(AUTH_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(UTILS_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(PHY_INIT_DIR) && $(MAKE) all
	$(CD) ..
ifeq ($(findstring -DSECURE_CONN, $(SYSTEM_LEVEL_DEF)), -DSECURE_CONN)
	$(CD) $(BT_SECURE_CONN_DIR) && $(MAKE) all
	$(CD) ..
endif
init_le_subdirs:
	$(CD) $(LL_DIR) && $(MAKE) all
	$(CD) ..

init_3dd_subdirs:
	$(CD) $(BT_3DD_DIR) && $(MAKE) all
	$(CD) ..
	
init_mws_subdirs:
	$(CD) $(MWS_DIR) && $(MAKE) all
	$(CD) ..
	
init_phy_init_subdirs:
	$(CD) $(PHY_INIT_DIR) && $(MAKE) all
	$(CD) ..
#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

