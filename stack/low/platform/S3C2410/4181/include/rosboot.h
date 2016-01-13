/*
*************************************************************************
*                                                                       *
*                                                                       *
*  myboot.h: Defines various data structures and constants create by   *
*            myboot.s and shared with program that might link osboot    *
*            in.														*
*                                                                       *
*                                                                       *
*************************************************************************
*                                                                       *
*  Copyright 1992-1994, Embedded Performance, Inc.  All Rights Reserved *
*                                                                       *
*  A purchasing customer is hereby granted rights to use, modify and    *
*  distribute(binary form).                                             *
*                                                                       *
*************************************************************************
*/

#ifndef MYBOOT_INCLUDED
#define MYBOOT_INCLUDED

#define GNU_TOOL


/*----------------------------------------------------------------------
**
** Define the range of cache size of LX4180
**
**----------------------------------------------------------------------
*/
#define	MIN_CACHE_SIZE	(1024)
#define MAX_CACHE_SIZE	(16*1024)



/*----------------------------------------------------------------------
**
** Constants used by the flush_cache HIF call.
**
**----------------------------------------------------------------------
*/
#define ICACHE		0
#define DCACHE		1

//#define DRAM_BASE   0x03000000
//#define DRAM_BASE   0x80300000
//#define DRAM_BASE   0x001d0000
//   JF
#define DRAM_BASE   0x00300000
//#define DRAM_BASE   0x00010400
//#define DRAM_SIZE   2048
#define DRAM_SIZE   4096
/*----------------------------------------------------------------------
**
** Constants used by the application : stack size, SR value
**
**----------------------------------------------------------------------
*/

#define APPL_STACK_SIZE 0x1000 /* 4K */
#define APPL_SR         0x10000000		// BEV == 0, reset at 0x80000000 and exception at 0x80000080


#endif /* MYBOOT_INCLUDED */

