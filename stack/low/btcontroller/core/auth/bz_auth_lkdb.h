/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_lkdb.h
 *  Defines the BlueWiz Link key database interface.
 *
 * \author Santhosh kumar M
 * \date 2007-09-03
 */

#ifndef _BZ_AUTH_LKDB_H_
#define _BZ_AUTH_LKDB_H_

/* ========================= Include File Section ========================= */
#include "bt_fw_common.h"


/* ====================== Macro Declaration Section ======================= */
#define BZ_AUTH_LKDB_MAX_KEYS      7


/* ==================== Data Types Declaration Section ==================== */
/**
 * Link key database entry type. The link key database primarily stores the
 * link key indexed by the BD address.
 */
typedef struct _BZ_AUTH_LKDB_KEYS_T
{
    const UCHAR bd_addr[6];     /**< Bluetooth Device address */
    const UCHAR key[16];        /**< Link key associated with the \a bd_addr */
} BZ_AUTH_LKDB_KEYS_T;


/* ================ Exported Variables Declaration Section ================ */


/* ============================= API Section ============================== */
void bz_auth_lkdb_init(void);
BOOLEAN bz_auth_lkdb_get_key(IN const UCHAR* bd_addr, OUT const UCHAR** pkey);
UCHAR bz_auth_lkdb_get_all_keys(OUT const UCHAR* bd_addr[BZ_AUTH_LKDB_MAX_KEYS],
        OUT const UCHAR* keys[BZ_AUTH_LKDB_MAX_KEYS]);
UCHAR bz_auth_lkdb_put_keys(const BZ_AUTH_LKDB_KEYS_T* keys, UCHAR nkeys);
BOOLEAN bz_auth_lkdb_del_key(const UCHAR* bd_addr);
UCHAR bz_auth_lkdb_del_all_keys(void);

#endif /* _BZ_AUTH_LKDB_H_ */

