/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/src/target/hw_srp.h,v 1.2 2013/08/16 06:49:35 mn Exp $
 *
 * File:        $RCSfile: hw_srp.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/src/target/hw_srp.h,v $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name:]
 * !GROUP       [  ]
 * ------------------------------------------------------------------------
 *
 *          Copyright (c)           2011 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *           CORTEX-M3 HW interface
 *
 * ------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ------------------------------------------------------------------------
 * !CONTENTS
 * ------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 * ------------------------------------------------------------------------
 *
 **********************************************************************!HE*/


/****************************************************************************/
/* Include                                                                  */
/****************************************************************************/

#include <stm32f10x.h>


/****************************************************************************/
/* Base address                                                             */
/****************************************************************************/

#define EPROM_BASE         FLASH_BASE
#define RAM_BASE           SRAM_BASE

#define FLASH_PAGE_SIZE    0x0800

/****************************************************************************/
/* Flash definitions                                                        */
/* 0x08000000..0x08000FFF - vector table                                    */
/* 0x08001000..0x080017FF - infopage 1                                      */
/* 0x08001800..0x08001FFF - infopage 2                                      */
/* 0x08002000..0x0807FFFF - firmware                                        */
/****************************************************************************/

/* vector table */
#define FEP_VECTOR_TABLE_BASE     EPROM_BASE
#define FEP_VECTOR_TABLE_SIZE     (FLASH_PAGE_SIZE * 2)

/* infopage1 */
#define FEP_INFOPAGE1_BASE        (EPROM_BASE + FEP_VECTOR_TABLE_SIZE)
#define FEP_INFOPAGE1_SIZE        FLASH_PAGE_SIZE
#define FEP_INFO1_FIRST_PAGE      ((FEP_INFOPAGE1_BASE - EPROM_BASE) / FLASH_PAGE_SIZE)
#define FEP_INFO1_LAST_PAGE       (((FEP_INFOPAGE1_BASE - EPROM_BASE) + FEP_INFOPAGE1_SIZE) / FLASH_PAGE_SIZE)

/* infopage2 */
#define FEP_INFOPAGE2_BASE        (FEP_INFOPAGE1_BASE + FEP_INFOPAGE1_SIZE)
#define FEP_INFOPAGE2_SIZE        FLASH_PAGE_SIZE
#define FEP_INFO2_FIRST_PAGE      ((FEP_INFOPAGE2_BASE - EPROM_BASE) / FLASH_PAGE_SIZE)
#define FEP_INFO2_LAST_PAGE       (((FEP_INFOPAGE2_BASE - EPROM_BASE) + FEP_INFOPAGE2_SIZE) / FLASH_PAGE_SIZE)

/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

void hwDetectHwVersion(void);
void hwBluetoothReset(void);
