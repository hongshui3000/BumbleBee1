/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

#ifndef ARRAY_MAP_H_
#define ARRAY_MAP_H_

#include "DataType.h"

#define ARRAY_MAP_UNIQUE_NAME_(counter) ARRAY_MAP_DESC_##counter
#define ARRAY_MAP_UNIQUE_NAME(counter) ARRAY_MAP_UNIQUE_NAME_(counter)

/** Define a array map for a specified type. */
#define ARRAY_MAP_DEFINE_TYPE(type, name) \
typedef struct \
{ \
    type *item; \
    UINT32 size; \
    UINT32 len; \
} name

/**
 * @brief Initialize a array map.
 * @param map  The array map.
 * @param buf  The data buffer array.
 * @param siz  The number of elements in the data buffer.
 */
#define ARRAY_MAP_INIT(map, buf, siz) \
do { \
    (map)->item = (buf); \
    (map)->size = (siz); \
    (map)->len = 0; \
} while (0)

/**
 * @brief Binary search an element in the array map by \p key.
 *
 * The binary search is based on \p cmp, which could be a function or a marco.
 * \p cmp takes two parameters in order, the array element and the key, and
 * returns a negative value if the array element is less than the key, a
 * positive value if the array element is greater than the key, or zero if the
 * array element equals the key.
 *
 * @param map  The array map.
 * @param key  The search key.
 * @param cmp  The comparator function/macro.
 * @param index  Return value for the index of the element it found.
 * @return  \c TRUE if found; otherwise, \c FALSE.
 */
#define ARRAY_MAP_BSEARCH(map, key, cmp, index) \
({ \
    BOOLEAN found = FALSE; \
    if ((map)->len == 0) \
    { \
        *(index) = 0; \
    } \
    else \
    { \
        int low = 0; \
        int high = (map)->len - 1; \
        while (low < high) \
        { \
            int med = (low + high) / 2; \
            int c = cmp((map)->item[med], (key)); \
            if (c < 0) \
            { \
                low = med + 1; \
            } \
            else if (c > 0) \
            { \
                high = med - 1; \
            } \
            else \
            { \
                *(index) = med; \
                found = TRUE; \
                break; \
            } \
        } \
        if (!found) \
        { \
            int c = cmp((map)->item[low], (key)); \
            if (c == 0) \
            { \
                *(index) = low; \
                found = TRUE; \
            } \
            else if (c < 0) \
            { \
                *(index) = low + 1; \
            } \
            else \
            { \
                *(index) = low; \
            } \
        } \
    } \
    found; \
})

/**
 * @brief Insert an element into the array map.
 *
 * If the array map doesn't have an element with the given \p key, an new
 * element is created and initialized by \p initializer. \p initializer
 * takes two parameters in order: the new item and the key.
 *
 * @param map  The array map.
 * @param key  The key of the element.
 * @param bsearch  The binary search function/macro.
 * @param initializer  The initializer applied on the new element.
 * @return  The created/existing element with specified \p key, or \c NULL if
 * no free buffer.
 */
#define ARRAY_MAP_INSERT(map, key, bsearch, initializer) \
({ \
    typeof ((map)->item[0]) inserted = 0; \
    if ((map)->len < (map)->size) \
    { \
        int index; \
        if (!bsearch((map), (key), &index)) \
        { \
            int i; \
            typeof ((map)->item[0]) new_item = (map)->item[(map)->len]; \
            initializer(new_item, (key)); \
            for (i = (map)->len; i > index; --i) \
            { \
                (map)->item[i] = (map)->item[i - 1]; \
            } \
            (map)->item[index] = new_item; \
            ++(map)->len; \
        } \
        inserted = (map)->item[index]; \
    } \
    inserted; \
})

/**
 * @brief Remove an element from the array map.
 *
 * If the array map has the element with the given \p key, the element is
 * removed and applied with \p finalizer. \p finalizer takes one parameter:
 * the item to delete.
 *
 * @param map  The array map.
 * @param key  The key of the element.
 * @param bsearch  The binary function/macro.
 * @param finalizer  The finalizer applied on the element to reclaim resources.
 */
#define ARRAY_MAP_REMOVE(map, key, bsearch, finalizer) \
do { \
    int index; \
    if (bsearch((map), (key), &index)) \
    { \
        int i; \
        finalizer((map)->item[index]); \
        typeof ((map)->item[0]) free_item = (map)->item[index]; \
        for (i = index; i < (map)->len - 1; ++i) \
        { \
            (map)->item[i] = (map)->item[i + 1]; \
        } \
        (map)->item[(map)->len - 1] = free_item; \
        --(map)->len; \
    } \
} while (0)

/**
 * @brief Find the element in the array map with specified \p key.
 * @param map  The array map.
 * @param key  The key of the element.
 * @param bsearch  The binary search function/macro.
 * @return  The element with the specified \p key if found; otherwise, \c NULL;
 */
#define ARRAY_MAP_FIND(map, key, bsearch) \
({ \
    typeof ((map)->item[0]) found = 0; \
    int index; \
    if (bsearch((map), (key), &index)) \
    { \
        found = (map)->item[index]; \
    } \
    found; \
})

#endif /* ARRAY_MAP_H_ */
