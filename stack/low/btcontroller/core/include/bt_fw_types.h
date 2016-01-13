/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __BT_TYPES_H__
#define __BT_TYPES_H__

#include "platform.h"

/* Will be used as a standard return value by all APIs */
typedef UINT16      API_RESULT;

#define API_SUCCESS          0
#define API_RETRY           0x0F
#define API_FAILURE         0xFF

#define BT_FW_STATIC           static

#ifdef UPF_TESTER_CODE
#define UPF_21_BUG_416_TESTER
#endif /* UPF_TESTER_CODE */

#endif /* __BT_TYPES_H__ */

