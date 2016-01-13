/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/inc/bt_infop.h,v 1.1 2009/12/14 08:38:52 pm Exp $
 *
 * File:        $RCSfile: bt_infop.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/inc/bt_infop.h,v $
 * Revision:    $Revision: 1.1 $
 * Date:        $Date: 2009/12/14 08:38:52 $
 * Author:      $Author: pm $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name: P_SRP1290_V1_0 $]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: pm $]
 * ------------------------------------------------------------------------
 *
 *          Copyright (c)           2009 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15D
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          The content of this file is Stollmann E+V GmbH confidential
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *               infopage data format definition
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

#if !defined(__BT_INFOP_H)
#define      __BT_INFOP_H
#include <header.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_M_IX86) || defined (__BORLANDC__)    /* ( _MSC_VER >= 700 ) || Borland*/
/* 4103 "'%Fs' : used #pragma pack to change alignment " */
#pragma warning(disable : 4103)
#define BYTE_ALIGNMENT  1
#endif

#if defined (BYTE_ALIGNMENT)
#pragma pack(1)
#endif

#if defined(PRAGMA_PACK)
#pragma pack(1)
#endif

#ifndef PACKED
#define PACKED(a) a
#endif

#define BT_INFOPAGE_V2_00           0x0200              /* IP format V2.00 */

/****************************************************************************/
/* Infopage1 Version                                                        */
/****************************************************************************/

#define BT_INFOPAGE1_IDX             1

#define BT_INFOPAGE1_DATE            0x071209            /* 7.Dec.2009 */

          /* actual version */
#define BT_INFOPAGE1_VERSION_MAJOR    (((BT_INFOPAGE_V2_00 & 0x0F00) >> 8) + '0')
#define BT_INFOPAGE1_VERSION_MINOR1   (((BT_INFOPAGE_V2_00 & 0x00F0) >> 4) + '0')
#define BT_INFOPAGE1_VERSION_MINOR2   (((BT_INFOPAGE_V2_00 & 0x000F) >> 0) + '0')

#define BT_INFOPAGE1_DATE_DAY1        (((BT_INFOPAGE1_DATE & 0x00F00000) >> 20) + '0')
#define BT_INFOPAGE1_DATE_DAY2        (((BT_INFOPAGE1_DATE & 0x000F0000) >> 16) + '0')
#define BT_INFOPAGE1_DATE_MONTH1      (((BT_INFOPAGE1_DATE & 0x0000F000) >> 12) + '0')
#define BT_INFOPAGE1_DATE_MONTH2      (((BT_INFOPAGE1_DATE & 0x00000F00) >> 8) + '0')
#define BT_INFOPAGE1_DATE_YEAR1       (((BT_INFOPAGE1_DATE & 0x000000F0) >> 4) + '0')
#define BT_INFOPAGE1_DATE_YEAR2       (((BT_INFOPAGE1_DATE & 0x0000000F) >> 0) + '0')

/****************************************************************************/
/* Infopage2 Version                                                        */
/****************************************************************************/

#define BT_INFOPAGE2_IDX             2

#define BT_INFOPAGE2_V2_00           0x0200              /* V2.00 */
#define BT_INFOPAGE2_DATE            0x071209            /* 7.Dec.2009 */

          /* actual version */
#define BT_INFOPAGE2_VERSION_MAJOR    (((BT_INFOPAGE_V2_00 & 0x0F00) >> 8) + '0')
#define BT_INFOPAGE2_VERSION_MINOR1   (((BT_INFOPAGE_V2_00 & 0x00F0) >> 4) + '0')
#define BT_INFOPAGE2_VERSION_MINOR2   (((BT_INFOPAGE_V2_00 & 0x000F) >> 0) + '0')

#define BT_INFOPAGE2_DATE_DAY1        (((BT_INFOPAGE2_DATE & 0x00F00000) >> 20) + '0')
#define BT_INFOPAGE2_DATE_DAY2        (((BT_INFOPAGE2_DATE & 0x000F0000) >> 16) + '0')
#define BT_INFOPAGE2_DATE_MONTH1      (((BT_INFOPAGE2_DATE & 0x0000F000) >> 12) + '0')
#define BT_INFOPAGE2_DATE_MONTH2      (((BT_INFOPAGE2_DATE & 0x00000F00) >> 8) + '0')
#define BT_INFOPAGE2_DATE_YEAR1       (((BT_INFOPAGE2_DATE & 0x000000F0) >> 4) + '0')
#define BT_INFOPAGE2_DATE_YEAR2       (((BT_INFOPAGE2_DATE & 0x0000000F) >> 0) + '0')

/****************************************************************************/
/* Infopage next free index                                                 */
/****************************************************************************/

#define BT_INFOPAGE_NEXT_FREE_IDX    3


/****************************************************************************/
/* Definition                                                               */
/****************************************************************************/

#define BT_INFOPAGE_MAGIC            0x50494240L         /* @BIP */

/****************************************************************************/
/* InfoPageDataFormat                                                       */
/****************************************************************************/
typedef PACKED(struct) _tagInfoPageDataFormat
{
  uint16_t  fwID;                          /* manufacture specific BB version */
  uint16_t  lengthOfParam;                  /* length of record                */
  uint8_t  param[1];                       /* data array */
} TInfoPageDataFormat;
typedef TInfoPageDataFormat *PInfoPageDataFormat;

/****************************************************************************/
/* InfoPage format                                                          */
/****************************************************************************/

typedef PACKED(struct) _tagBTInfoPage
{
  TGlobalHeader       GlobalHeader;
  TModuleHeader       ModuleHeader;
  TInfoPageDataFormat firstRecord;
} TBTInfoPage;
typedef TBTInfoPage *PBTInfoPage;

#if defined (BYTE_ALIGNMENT)
#pragma  pack()
#undef BYTE_ALIGNMENT
#endif

#if defined(PRAGMA_PACK)
#pragma pack()
#endif

/****************************************************************************/
/* infopageGetNextDataPacket                                                */
/****************************************************************************/
PInfoPageDataFormat infopageGetNextDataPacket(uint8_t IN nInfopageIdx, uint16_t fwID, PInfoPageDataFormat IN pDataRecord);

#ifdef __cplusplus
}
#endif

#endif  /* defined (__BT_INFOP_H) */
