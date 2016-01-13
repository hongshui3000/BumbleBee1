
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

OBJS= mem.o \
print.o \
    vectors.o
    
ASM_OBJS= boot.o \
    rlx4181.o		  
    

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
all: CORE_TARGETS COPY_APPL_OBJS

#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS:	$(OBJS) $(ASM_OBJS)

vectors.o : vectors.c
	$(CC) $(COMP_FLAGS_32) vectors.c -o vectors.o
	@echo "  CC    $@"
#boot.o : boot.S
#	$(CC) $(CC_ARG) boot.S -o boot.o
#rlx4181.o : rlx4181.S
#	$(CC) $(CC_ARG) rlx4181.S -o rlx4181.o

#*****************************************************************************#
#                             END OF MAKEFILE                                 #
#*****************************************************************************#

