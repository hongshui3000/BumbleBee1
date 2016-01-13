/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file bz_auth_lkdb.c
 *  Implements the BlueWiz Link key database interface.
 *
 * \author Santhosh kumar M
 * \date 2007-09-03
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 22 };
/********************************* Logger *************************/
/* ========================= Include File Section ========================= */
//#include "bt_fw_common.h"
#include "bz_auth_lkdb.h"
#include "mem.h"

/* ====================== Macro Declaration Section ======================= */


/* ==================== Structure Declaration Section ===================== */
/**
 * Link key database type.
 */
typedef struct _lkdb_t
{
    UCHAR bd_addr[6];   /**< Bluetooth device address */
    UCHAR key[16];      /**< Link key associated with the BD Address */
    BOOLEAN valid;      /**< TRUE, if the link key is valid. FALSE,
                             otherwise */
} lkdb_t;


/* ===================== Variable Declaration Section ===================== */
/**
 * Link key database containing the active link keys indexed by the BD
 * addresses.
 */
BZ_STATIC lkdb_t lkdb[BZ_AUTH_LKDB_MAX_KEYS];

/* ================== Static Function Prototypes Section ================== */


/* ===================== Function Definition Section ====================== */
/**
 * Initialize/reset the link key database.
 *
 * \param None.
 *
 * \return None.
 */
void bz_auth_lkdb_init(void)
{
    register int i;

    for (i = 0; i < BZ_AUTH_LKDB_MAX_KEYS; i++)
    {
        lkdb[i].valid = FALSE;
    }
}

/**
 * Returns the link key associated with the given \a bd_addr, if available.
 *
 * \param bd_addr Bluetooth device address for which the link key is requested.
 * \param pkey Pointer to store the link key associated with the \a bd_addr.
 *             The returned link key should not be modified by the caller.
 *
 * \return TRUE, if a link key was found for the given \a bd_addr. FALSE,
 *         otherwise.
 */
BOOLEAN bz_auth_lkdb_get_key(IN const UCHAR* bd_addr, OUT const UCHAR** pkey)
{
    register int i;

    for (i = 0; i < BZ_AUTH_LKDB_MAX_KEYS; i++)
    {
        if (lkdb[i].valid && !memcmp(lkdb[i].bd_addr, bd_addr, 6))
        {
            *pkey = lkdb[i].key;
            return TRUE;
        }
    }

    return FALSE;
}

/**
 * Returns all the valid link keys in the link key database.
 *
 * \param bd_addr Pointer to the returned BD address array.
 * \param keys Pointer to the returned link key array. The index of the array
 *             links the link key with the corresponding BD address.
 *
 * \return Number of keys returned.
 */
UCHAR bz_auth_lkdb_get_all_keys(OUT const UCHAR* bd_addr[BZ_AUTH_LKDB_MAX_KEYS],
        OUT const UCHAR* keys[BZ_AUTH_LKDB_MAX_KEYS])
{
    register int i;
    UCHAR nkeys = 0;

    for (i = 0; i < BZ_AUTH_LKDB_MAX_KEYS; i++)
    {
        if (lkdb[i].valid)
        {
            bd_addr[nkeys] = lkdb[i].bd_addr;
            keys[nkeys] = lkdb[i].key;
            nkeys++;
        }
    }

    return nkeys;
}

/**
 * Store the given \a keys in the link key database.
 *
 * \param keys Link keys (along with the BD addresses) array to be stored in
 *             in the link key database.
 * \param nkeys Number of keys to be stored.
 *
 * \return Number of keys successfully stored in the link key database.
 */
UCHAR bz_auth_lkdb_put_keys(const BZ_AUTH_LKDB_KEYS_T* keys, UCHAR nkeys)
{
    register int i;
    UCHAR nkeys_written = 0;

    for (i = 0; i < BZ_AUTH_LKDB_MAX_KEYS; i++)
    {
        if (!lkdb[i].valid && (nkeys_written < nkeys))
        {
            memcpy(lkdb[i].bd_addr, keys[nkeys_written].bd_addr, 6);
            memcpy(lkdb[i].key, keys[nkeys_written].key, 16);
            lkdb[i].valid = TRUE;
            nkeys_written ++;
        }
    }

    return nkeys_written;
}

/**
 * Delete the link key associated with the \a bd_addr.
 *
 * \param bd_addr Bluetooth device address for which the link key has to be
 *                deleted.
 *
 * \return TRUE, if the deletion was successful. FALSE, otherwise.
 */
BOOLEAN bz_auth_lkdb_del_key(const UCHAR* bd_addr)
{
    register int i;

    for (i = 0; i < BZ_AUTH_LKDB_MAX_KEYS; i++)
    {
        if (lkdb[i].valid && !memcmp(lkdb[i].bd_addr, bd_addr, 6))
        {
            lkdb[i].valid = FALSE;
            return TRUE;
        }
    }

    return FALSE;
}

/**
 * Delete all the link keys from the link key database.
 *
 * \param None.
 *
 * \return Number of keys deleted.
 */
UCHAR bz_auth_lkdb_del_all_keys(void)
{
    register int i;
    UCHAR nkeys = 0;

    for (i = 0; i < BZ_AUTH_LKDB_MAX_KEYS; i++)
    {
        if (lkdb[i].valid)
        {
            lkdb[i].valid = FALSE;
            nkeys ++;
        }
    }

    return nkeys;
}

