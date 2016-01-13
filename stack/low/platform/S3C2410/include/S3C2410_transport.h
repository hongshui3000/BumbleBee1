/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _S3C2410_TRANSPORT_H_
#define _S3C2410_TRANSPORT_H_

#include "platform.h"

typedef enum
{
    INVALID_TRANSPORT=0x0,
    UART_TRANSPORT,
    USB_TRANSPORT,
    SOFT_TRANSPORT
} S3C2410_TRANSPORT_TYPE;

#endif /* _S3C2410_TRANSPORT_H_ */

