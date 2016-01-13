#ifndef _G711_H_
#define _G711_H_

#include "DataType.h"
#include "bzdma.h"

INT16 g711_linear2alaw(INT16 pcm_val);
INT16 g711_alaw2linear(INT16 a_byte);
INT16 g711_linear2ulaw(INT16 pcm_val);
INT16 g711_ulaw2linear(INT16 u_byte);
INT16 g711_alaw2ulaw(INT16 a_byte);
INT16 g711_ulaw2alaw(INT16 u_byte);
UINT16 g711_a2u2alaw_deviation(UINT16 a_byte);
UINT16 g711_u2a2ulaw_deviation(UINT16 u_byte);
INT16 linear_pcm_conv_to_16bit_2s(UINT8 in_type, UINT8 in_8bit, INT16 pcm_in);
INT16 linear_pcm_conv_from_16bit_2s(UINT8 out_type, UINT8 out_8bit, INT16 pcm_in);
#endif

