/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  bt_secure_conn.c (bt secure connection)
 *
 * \author 
 *  daphneliu <daphneliu@realtek.com>, (C) 2013
 */

 /*  @{ */

enum { __FILE_NUM__= 79 };

#include "bt_secure_conn.h"
#include "mem.h"
#include "bt_fw_hci_external_defines.h"
#include "bb_driver.h"
#include "bz_auth_extern_accessors.h"


void bz_auth_reset_sc_ext_feature_bits(void)
{
    register int i;
    UCHAR* features;

    for (i = 0; i < LMP_MAX_PICONETS_SUPPORTED; i++)
    {
        /* Disable extended features support. This is an ugly hack introduced
         * because of the hacks introduced in the spec itself. We may need to
         * move it appropriate place without breaking the /INTERFACES/.
         * Currently SSP is the only feature which uses EXT_FEATURES page, so
         * we can safely disable it. But in future it may need to be reworked.
         */
#if 0 /* It seems that we don't need to unset this bit */
        features = &bz_auth_get_local_features(i)[0];
        features[7] = (UCHAR)(features[7] & (~EXTENDED_FEATURE));
#endif

        /* Disable SC HOST support */
        features = &bz_auth_get_local_ext_features(i)[0][0];
        features[0] = (UCHAR)(features[0] &
                (~SECURE_CONNECTION_HOST_SUPPORT));
    }

    return;
}

void bz_auth_enable_sc_ext_feature_bits(void)
{
    register int i;
    UCHAR* features;

    for (i = 0; i < LMP_MAX_PICONETS_SUPPORTED; i++)
    {
        /* Enable extended features support */
        features = &bz_auth_get_local_features(i)[0];
        features[7] = (UCHAR)(features[7] | EXTENDED_FEATURE);

        /* Enable SC HOST support */
        features = &bz_auth_get_local_ext_features(i)[0][0];
        features[0] = (UCHAR)(features[0] | SECURE_CONNECTION_HOST_SUPPORT);
    }

    return;
}



