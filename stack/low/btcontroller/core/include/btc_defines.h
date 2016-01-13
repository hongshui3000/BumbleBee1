/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __BTC_DEFINES_H__
#define __BTC_DEFINES_H__


#include "lmp_1_2_defines.h"

/* Function Prototypes */

void init_1_2_self_device_data(void);

void init_1_2_connection_entity(UINT16 ce_index);

void init_2_1_connection_entity(UINT16 ce_index);

void init_2_1_self_device_data(void);

#ifdef VER_3_0
void init_3_0_connection_entity(UINT16 ce_index);

void init_3_0_self_device_data(void);
#endif

void init_1_2_connection_entity_after_hlc(UINT16 ce_index);


#ifdef COMPILE_ESCO

void init_esco_ce_mapping_tables(void);

void init_esco_connection_data(void);

#endif /* COMPILE_ESCO */

#ifdef VER_3_0

#define INIT_3_0_CONNECTION_ENTITY \
        init_3_0_connection_entity

#else

#define INIT_3_0_CONNECTION_ENTITY

#endif

#define INIT_2_1_CONNECTION_ENTITY \
        init_2_1_connection_entity

#define INIT_1_2_SELF_DEVICE_DATA \
        init_1_2_self_device_data()

#define INIT_1_2_CONNECTION_ENTITY \
        init_1_2_connection_entity

#ifdef COMPILE_ESCO

#define INIT_ESCO_MAPPING_TABLES()\
       init_esco_ce_mapping_tables()

#define INIT_ESCO_CONNECTION_DATA()\
       init_esco_connection_data()

#else /* COMPILE_ESCO */

#define INIT_ESCO_MAPPING_TABLES()

#define INIT_ESCO_CONNECTION_DATA()

#endif /* COMPILE_ESCO */

#define INIT_2_1_SELF_DEVICE_DATA \
        init_2_1_self_device_data()

#ifdef VER_3_0
#define INIT_3_0_SELF_DEVICE_DATA \
        init_3_0_self_device_data()
#else
#define INIT_3_0_SELF_DEVICE_DATA
#endif

#endif /* #ifndef __BTC_DEFINES_H__ */
