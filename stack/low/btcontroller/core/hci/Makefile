
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
OBJS =
OBJS += 	bt_fw_hci.o
OBJS +=		bt_fw_hci_1_2_cmds.o
OBJS +=		bt_fw_hci_1_2_events.o
OBJS +=		bt_fw_hci_2_1_cmds.o
OBJS +=		bt_fw_hci_2_1_events.o
OBJS +=		bt_fw_hci_cmds.o
OBJS +=		bt_fw_hci_events.o
OBJS +=		bt_fw_hci_hcbb_info.o
OBJS +=		bt_fw_hci_tasks.o
OBJS +=		bt_fw_hci_utils.o
OBJS +=		bt_fw_acl_q.o
OBJS +=		bt_fw_hci_3_0_cmds.o
OBJS +=		bt_fw_hci_3_0_events.o
OBJS +=		bt_fw_hci_csa4_cmds.o
OBJS +=		bt_fw_hci_csa4_events.o
OBJS +=		bt_fw_hci_secure_conn_cmds.o
OBJS +=		bt_fw_hci_secure_conn_events.o

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

