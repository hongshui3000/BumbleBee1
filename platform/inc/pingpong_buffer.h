#ifndef PINGPONG_BUFFER_H
#define PINGPONG_BUFFER_H

#include "rtl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void PPB_Init(void);
void PPB_Write(const uint8_t *source, uint16_t size);
void PPB_ClearOutputBuf(void);

#ifdef __cplusplus
}
#endif

#endif
