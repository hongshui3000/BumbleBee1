/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */
#include <stdlib.h>
#include "section_config.h"

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

/* Allocate the memory for the heap. */
#if( configAPPLICATION_ALLOCATED_HEAP == 1 )
	/* The application writer has already defined the array used for the RTOS
	heap - probably so it can be placed in a special segment or address. */
	extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else

    uint32_t configTOTAL_HEAP_SIZE_DATA_OFF;	//Totol Heap size = Basic Heap size + efuse config size 
    uint32_t configTOTAL_HEAP_SIZE_DATA_ON;		//Totol Heap size = Basic Heap size + efuse config size  

    uint8_t ucHeap_DATA_OFF[ configBasic_HEAP_SIZE_DATA_OFF ] SRAM_OFF_BD_HEAP_SECTION;
    uint8_t ucHeap_DATA_ON[ configBasic_HEAP_SIZE_DATA_ON ] SRAM_ON_BD_HEAP_SECTION;

#endif /* configAPPLICATION_ALLOCATED_HEAP */

    uint8_t ucHeap_BUFFER_OFF[ configTOTAL_HEAP_SIZE_BUFFER_OFF ] SRAM_OFF_BF_BSS_SECTION;
    uint8_t ucHeap_BUFFER_ON[ configTOTAL_HEAP_SIZE_BUFFER_ON ] SRAM_ON_BF_BSS_SECTION;

/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
	size_t xBlockSize;						/*<< The size of the free block. */
} BlockLink_t;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void prvInsertBlockIntoFreeList( RAM_TYPE ramType, BlockLink_t *pxBlockToInsert );

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void prvHeapInit( RAM_TYPE ramType );

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

/* Create a couple of list links to mark the start and end of the list. */
BlockLink_t xStart[RAM_TYPE_NUM] = {{NULL, 0}};
BlockLink_t *pxEnd[RAM_TYPE_NUM] = {NULL};

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
size_t xFreeBytesRemaining[RAM_TYPE_NUM];
size_t xMinimumEverFreeBytesRemaining[RAM_TYPE_NUM];

#define RAM_TYPE_BITS_OFFSET	( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 3 )
#define RAM_TYPE_BITS_MASK		( ( ( size_t ) 7 ) << ( RAM_TYPE_BITS_OFFSET ) )

/* Sets the top 3 bits of an size_t type. It is used to select the Block with
the required RAM type to free. */
size_t xBlockRamTypeBits[RAM_TYPE_NUM] = {0};

/* Though only one bit to set BlockAllocated status, it needs check the top 4 bits
that if the required memory size exceeds. The maximum memory size is (2^28)-1. */
#define BLOCK_ALLOCATED_BITS_OFFSET		( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 4 )
#define BLOCK_ALLOCATED_BITS_MASK		( ( ( size_t ) 15 ) << ( BLOCK_ALLOCATED_BITS_OFFSET ) )

/* Gets set to the bit following RamTypeBits of an size_t type.  When this bit in
the xBlockSize member of an BlockLink_t structure is set then the block belongs
to the application.  When the bit is free the block is still part of the free heap
space. */
size_t xBlockAllocatedBit = ( ( size_t ) 1 ) << ( BLOCK_ALLOCATED_BITS_OFFSET );

/*-----------------------------------------------------------*/

void *pvPortMalloc( RAM_TYPE ramType, size_t xWantedSize )
{
BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

	vTaskSuspendAll();
	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( pxEnd[ramType] == NULL )
		{
			prvHeapInit(ramType);
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* Check the requested block size is not so large that the top 4 bits
		are set. The fouth toppest bit of the block size member of the
		BlockLink_t structure is used to determine who owns the block - the
		application or the kernel, so it must be free. */
		if( ( xWantedSize & BLOCK_ALLOCATED_BITS_MASK ) == 0 )
		{
			/* The wanted size is increased so it can contain a BlockLink_t
			structure in addition to the requested amount of bytes. */
			if( xWantedSize > 0 )
			{
				xWantedSize += xHeapStructSize;

				/* Ensure that blocks are always aligned to the required number
				of bytes. */
				if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
				{
					/* Byte alignment required. */
					xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
					configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining[ramType] ) )
			{
				/* Traverse the list from the start	(lowest address) block until
				one	of adequate size is found. */
				pxPreviousBlock = &xStart[ramType];
				pxBlock = xStart[ramType].pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* If the end marker was reached then a block of adequate size
				was	not found. */
				if( pxBlock != pxEnd[ramType] )
				{
					/* Return the memory space pointed to - jumping over the
					BlockLink_t structure at its start. */
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );
#ifdef CONFIG_DLPS_EN
					/* Store the heapSTRUCT into partital-on memory */
					if(ramType == RAM_TYPE_DATA_OFF || ramType == RAM_TYPE_BUFFER_OFF)
					{
						if(!DLPS_BUFFER_REG((UINT8	*)(pxPreviousBlock->pxNextFreeBlock), heapSTRUCT_SIZE, FALSE))
						{
							DBG_DIRECT("ERROR: Store heapSTRUCT into partital-on fail!");
						}
					}
#endif

					/* This block is being returned for use so must be taken out
					of the list of free blocks. */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* If the block is larger than required it can be split into
					two. */
					if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
					{
						/* This block is to be split into two.  Create a new
						block following the number of bytes requested. The void
						cast is used to prevent byte alignment warnings from the
						compiler. */
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
						configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

						/* Calculate the sizes of two blocks split from the
						single block. */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* Insert the new block into the list of free blocks. */
						prvInsertBlockIntoFreeList( ramType, ( pxNewBlockLink ) );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					xFreeBytesRemaining[ramType] -= pxBlock->xBlockSize;

					if( xFreeBytesRemaining[ramType] < xMinimumEverFreeBytesRemaining[ramType] )
					{
						xMinimumEverFreeBytesRemaining[ramType] = xFreeBytesRemaining[ramType];
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					/* The block is being returned - it is allocated and owned
					by the application and has no "next" block. */
					pxBlock->xBlockSize |= xBlockAllocatedBit;
					pxBlock->xBlockSize |= xBlockRamTypeBits[ramType];
					pxBlock->pxNextFreeBlock = NULL;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}
	( void ) xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	configASSERT( ( ( ( uint32_t ) pvReturn ) & portBYTE_ALIGNMENT_MASK ) == 0 );
	return pvReturn;
}
/*-----------------------------------------------------------*/

void vPortFree( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
BlockLink_t *pxLink;
RAM_TYPE ramType;

	if( pv != NULL )
	{
		/* The memory being freed will have an BlockLink_t structure immediately
		before it. */
		puc -= xHeapStructSize;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = ( void * ) puc;

		/* Check the block is actually allocated. */
		configASSERT( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 );
		configASSERT( pxLink->pxNextFreeBlock == NULL );

		if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == NULL )
			{
				/* The block is being returned to the heap - it is no longer
				allocated. */
				ramType = ( RAM_TYPE )( pxLink->xBlockSize >> RAM_TYPE_BITS_OFFSET );
				pxLink->xBlockSize &= ~BLOCK_ALLOCATED_BITS_MASK;

				vTaskSuspendAll();
				{
					/* Add this block to the list of free blocks. */
					xFreeBytesRemaining[ramType] += pxLink->xBlockSize;
					traceFREE( pv, pxLink->xBlockSize );
					prvInsertBlockIntoFreeList( ramType, ( ( BlockLink_t * ) pxLink ) );
				}
				( void ) xTaskResumeAll();
#ifdef CONFIG_DLPS_EN
				/* Release the heapSTRUCT from partital-on memory */
				if(ramType == RAM_TYPE_DATA_OFF || ramType == RAM_TYPE_BUFFER_OFF)
				{
					DLPS_BUFFER_UNREG(puc);
				}
#endif
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( RAM_TYPE ramType )
{
	return xFreeBytesRemaining[ramType];
}
/*-----------------------------------------------------------*/

size_t xPortGetMinimumEverFreeHeapSize( RAM_TYPE ramType )
{
	return xMinimumEverFreeBytesRemaining[ramType];
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
	/* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

static void prvHeapInit( RAM_TYPE ramType )
{
    BlockLink_t *pxFirstFreeBlock;
    uint8_t *pucAlignedHeap;
    size_t uxAddress;
    size_t xTotalHeapSize;
    uint8_t *ucHeapTyped;


	/* Ensure the heap starts on a correctly aligned boundary. */
    switch(ramType)
	{
		case RAM_TYPE_DATA_OFF:
			ucHeapTyped = ucHeap_DATA_OFF;
			//xTotalHeapSize =  configBasic_HEAP_SIZE_DATA_OFF + otp_str_data.gEfuse_OS_s.RamOffHeapSizeAdditional * 1024;
			xTotalHeapSize =  configBasic_HEAP_SIZE_DATA_OFF;
			break;
		case RAM_TYPE_DATA_ON:
			ucHeapTyped = ucHeap_DATA_ON;
			//xTotalHeapSize =  configBasic_HEAP_SIZE_DATA_ON + otp_str_data.gEfuse_OS_s.RamOnHeapSizeAdditional * 512;
			xTotalHeapSize =  configBasic_HEAP_SIZE_DATA_ON;
			break;
		case RAM_TYPE_BUFFER_OFF:
			ucHeapTyped = ucHeap_BUFFER_OFF;
			xTotalHeapSize = configTOTAL_HEAP_SIZE_BUFFER_OFF;
		    break;
		case RAM_TYPE_BUFFER_ON:
			ucHeapTyped = ucHeap_BUFFER_ON;
			xTotalHeapSize = configTOTAL_HEAP_SIZE_BUFFER_ON;
	        break;
		default:
		    return;
	}

    /* Ensure the heap starts on a correctly aligned boundary. */
	uxAddress = ( size_t ) ucHeapTyped;

	if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
	{
		uxAddress += ( portBYTE_ALIGNMENT - 1 );
		uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
		xTotalHeapSize -= uxAddress - ( size_t ) ucHeapTyped;
	}

	pucAlignedHeap = ( uint8_t * ) uxAddress;

	/* xStart is used to hold a pointer to the first item in the list of free
	blocks.  The void cast is used to prevent compiler warnings. */
	xStart[ramType].pxNextFreeBlock = ( void * ) pucAlignedHeap;
	xStart[ramType].xBlockSize = ( size_t ) 0;

	/* pxEnd is used to mark the end of the list of free blocks and is inserted
	at the end of the heap space. */
	uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
	uxAddress -= xHeapStructSize;
	uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
	pxEnd[ramType] = ( void * ) uxAddress;
	pxEnd[ramType]->xBlockSize = 0;
	pxEnd[ramType]->pxNextFreeBlock = NULL;

	/* To start with there is a single free block that is sized to take up the
	entire heap space, minus the space taken by pxEnd. */
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = pxEnd[ramType];

	/* Only one block exists - and it covers the entire usable heap space. */
	xMinimumEverFreeBytesRemaining[ramType] = pxFirstFreeBlock->xBlockSize;
	xFreeBytesRemaining[ramType] = pxFirstFreeBlock->xBlockSize;

	/* Work out the position of the top 3 bits in a size_t variable. */
	xBlockRamTypeBits[ramType] = ( ( size_t ) ramType ) << ( (sizeof( size_t ) * heapBITS_PER_BYTE) - 3 );
}

/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( RAM_TYPE ramType, BlockLink_t *pxBlockToInsert )
{
BlockLink_t *pxIterator;
uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	than the block being inserted. */
	for( pxIterator = &xStart[ramType]; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != pxEnd[ramType] )
		{
			/* Form one big block from the two blocks. */
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = pxEnd[ramType];
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	before and the block after, then it's pxNextFreeBlock pointer will have
	already been set, and should not be set here as that would make it point
	to itself. */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}

