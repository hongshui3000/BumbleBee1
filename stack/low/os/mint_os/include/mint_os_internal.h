/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef _H_OS_INTERNAL_
#define _H_OS_INTERNAL_

#include "bz_log_defines.h"

#ifdef KEIL_IDE
void*   malloc ( unsigned int bytes );
void    free ( void *ptr );
#endif // KEIL_IDE

#endif /* _H_OS_INTERNAL_ */


