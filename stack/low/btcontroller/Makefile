
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
OBJS= 	

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
BTC_CORE_DIR=./core
BTC_VENDO_DIR=./vendor

# Define the Rules to build the core targets
all: init_subdirs #CORE_TARGETS COPY_OBJS

#*****************************************************************************#
#                      RULES TO BUILD SUB DIRECTORIES
#*****************************************************************************#

# build sub dir
init_subdirs:
	$(CD) $(BTC_CORE_DIR) && $(MAKE) all
	$(CD) ..
	$(CD) $(BTC_VENDO_DIR) && $(MAKE) all
	$(CD) ..

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS)

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

