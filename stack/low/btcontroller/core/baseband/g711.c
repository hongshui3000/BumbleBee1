/*  
 * g711.c  
 *  
 * u-law, A-law and linear PCM conversions.  
 */      

#include "g711.h"
   
/* copy from CCITT G.711 specifications */   
UINT8 g711_u2a[128] = {  
    1,      1,      2,      2,      3,      3,      4,      4,   
    5,      5,      6,      6,      7,      7,      8,      8,   
    9,      10,     11,     12,     13,     14,     15,     16,   
    17,     18,     19,     20,     21,     22,     23,     24,   
    25,     27,     29,     31,     33,     34,     35,     36,   
    37,     38,     39,     40,     41,     42,     43,     44,   
    46,     48,     49,     50,     51,     52,     53,     54,   
    55,     56,     57,     58,     59,     60,     61,     62,   
    64,     65,     66,     67,     68,     69,     70,     71,   
    72,     73,     74,     75,     76,     77,     78,     79,   
    81,     82,     83,     84,     85,     86,     87,     88,   
    89,     90,     91,     92,     93,     94,     95,     96,   
    97,     98,     99,     100,    101,    102,    103,    104,   
    105,    106,    107,    108,    109,    110,    111,    112,   
    113,    114,    115,    116,    117,    118,    119,    120,   
    121,    122,    123,    124,    125,    126,    127,    128};   
   
UINT8 g711_a2u[128] = {
    1,      3,      5,      7,      9,      11,     13,     15,   
    16,     17,     18,     19,     20,     21,     22,     23,   
    24,     25,     26,     27,     28,     29,     30,     31,   
    32,     32,     33,     33,     34,     34,     35,     35,   
    36,     37,     38,     39,     40,     41,     42,     43,   
    44,     45,     46,     47,     48,     48,     49,     49,   
    50,     51,     52,     53,     54,     55,     56,     57,   
    58,     59,     60,     61,     62,     63,     64,     64,   
    65,     66,     67,     68,     69,     70,     71,     72,   
    73,     74,     75,     76,     77,     78,     79,     79,   
    80,     81,     82,     83,     84,     85,     86,     87,   
    88,     89,     90,     91,     92,     93,     94,     95,   
    96,     97,     98,     99,     100,    101,    102,    103,   
    104,    105,    106,    107,    108,    109,    110,    111,   
    112,    113,    114,    115,    116,    117,    118,    119,   
    120,    121,    122,    123,    124,    125,    126,    127};   

INT16 g711_a2l_quantized_point[8] = {1, 33, 66, 132, 264, 528, 1056, 2112};
UINT8 g711_a2l_interval[8] = {2, 2, 4, 8, 16, 32, 64, 128};
INT16 g711_u2l_quantized_point[8] = {0, 33, 99, 231, 495, 1023, 2079, 4191};
UINT16 g711_u2l_interval[8] = {2, 4, 8, 16, 32, 64, 128, 256};   


/**************************************************************************
 * Function     : g711_linear2alaw
 *
 * Description  : This function is used to convert one 16 bit linear 2's 
 *                complement PCM sample to one 8 bit a-law sample. No follow 
 *                g.711 standard to implement it. We follow current HW design 
 *                to use the same Table to implement it.
 *
 * Parameters   : pcm_val: one 16 bit linear PCM 2's complement format sample
 *                
 * Returns      : one 8 bit sample input of a-law
 *
 *************************************************************************/ 
INT16 g711_linear2alaw(INT16 pcm_val) 
{
    UINT16 u_pcm_in = (UINT16)(pcm_val >> 3) & 0x1FFF;
    UINT8 i;
    UINT8 u_alaw_out = 0x80;
    UINT8 decision_num;

    if (u_pcm_in == 0x1000)
    {
        /* linear 2s complement zero point */
        return (INT16)((0x7F ^ 0x55) & 0xFF);
    }

    if (u_pcm_in & 0x1000)
    {
        u_alaw_out = 0x00;
        u_pcm_in = (~u_pcm_in + 1) & 0x1FFF;      
    } 

    if (u_pcm_in < 64)
    {
        decision_num = u_pcm_in >> 1;
        u_alaw_out |= decision_num;
    }
    else
    {
        for (i = 11; i >= 6; i--)
        {
            if (u_pcm_in & (1 << i))
            {
                decision_num = (u_pcm_in - (1 << i)) >> (i - 4);
                u_alaw_out |= ((i - 4) << 4) | decision_num;
                break;
            }                 
        }     
    }

    u_alaw_out ^= 0x55;
    return (INT16)u_alaw_out;
}

/**************************************************************************
 * Function     : g711_alaw2linear
 *
 * Description  : This function is used to convert one 8 bit a-law sample to 
 *                one 16 bit linear 2's complement PCM sample. No follow g.711
 *                standard to implement it. We follow current HW design to use 
 *                the same Table and rule to implement it.
 *
 * Parameters   : a_byte: one 8 bit sample input of a-law
 *                
 * Returns      : one 16 bit linear PCM 2's complement format sample
 *
 *************************************************************************/    
INT16 g711_alaw2linear(INT16 a_byte)   
{    
    UINT8 a_val = a_byte & 0xFF;
    UINT8 seg;  
    UINT8 wxyz;
    UINT16 pcm_out = 0;
    UINT8 sign = (a_val & 0x80) ? 0 : 1;

    a_val &= 0x7F;
    a_val ^= 0x55;
    seg = a_val >> 4;
    wxyz = a_val & 0x0F;    
    pcm_out = g711_a2l_quantized_point[seg] + (g711_a2l_interval[seg] * wxyz);

    if (sign)
    {       
        pcm_out = (~pcm_out + 1) & 0x1FFF;
    }
    
    return (INT16)(pcm_out << 3);
}


/**************************************************************************
 * Function     : g711_linear2ulaw
 *
 * Description  : This function is used to convert one 16 bit linear 2's 
 *                complement PCM sample to one 8 bit u-law sample. No follow 
 *                g.711 standard to implement it. We follow current HW design 
 *                to use the same Table to implement it.
 *
 * Parameters   : pcm_val: one 16 bit linear PCM 2's complement format sample
 *                
 * Returns      : one 8 bit sample input of u-law
 *
 *************************************************************************/ 
INT16 g711_linear2ulaw(INT16 pcm_val)
{   
    UINT16 u_pcm_in = (UINT16)((pcm_val >> 2) & 0x3FFF);
    UINT8 i;
    UINT8 u_alaw_out = 0x00;
    UINT8 decision_num;

    if (u_pcm_in & 0x2000)
    {
        u_alaw_out = 0x80;
        u_pcm_in = (~u_pcm_in + 1) & 0x3FFF;      
    } 

    if (u_pcm_in < 31)
    {
        if (u_pcm_in > 0)
        {
            decision_num = ((u_pcm_in - 1) >> 1) + 1;
            u_alaw_out |= decision_num;
        }     
    }        
    else if (u_pcm_in >= 8159)
    {
        u_alaw_out |= 0x7F;
    }
    else
    {
        u_pcm_in += 33;
        for (i = 12; i >= 6; i--)
        {
            if (u_pcm_in & (1 << i))
            {
                decision_num = (u_pcm_in - (1 << i)) >> (i - 4);
                u_alaw_out |= ((i - 5) << 4) | decision_num;
                break;
            }                 
        }     
    }

    u_alaw_out = ~u_alaw_out;
    return (INT16)u_alaw_out;
}

/**************************************************************************
 * Function     : g711_ulaw2linear
 *
 * Description  : This function is used to convert one 8 bit u-law sample to 
 *                one 16 bit linear 2's complement PCM sample. No follow g.711
 *                standard to implement it. We follow current HW design to use 
 *                the same Table to implement it.
 *
 * Parameters   : a_byte: one 8 bit sample input of a-law
 *                
 * Returns      : one 16 bit linear PCM 2's complement format sample
 *
 *************************************************************************/   
INT16 g711_ulaw2linear(INT16 u_byte)   
{   
    UINT8 u_val = ~(u_byte & 0xFF);
    UINT8 seg;  
    UINT8 wxyz;
    UINT16 pcm_out = 0;
    
    wxyz = u_val & 0x0F;
    seg = (u_val >> 4) & 0x07;
    pcm_out = g711_u2l_quantized_point[seg] + g711_u2l_interval[seg] * wxyz;

    if (u_val & 0x80)
    {    
        /* negative value */
        pcm_out = (~pcm_out + 1) & 0x3FFF;
    }    
    return (INT16)(pcm_out << 2);
}   
   
/**************************************************************************
* Function     : g711_alaw2ulaw
*
* Description  : This function is used to convert one 8 bit a-law sample to 
*                one 8 bit u-law sample
*
* Parameters   : a_byte: one 8 bit sample input of a-law
*                
* Returns      : one 8 bit sample output of u-law
*
*************************************************************************/
INT16 g711_alaw2ulaw(INT16 a_byte)   
{   
    UINT8 aval = a_byte & 0xFF;
    UINT8 ubuf = (aval & 0x80) ? (0xFF ^ g711_a2u[aval ^ 0xD5]) :   
                    (0x7F ^ g711_a2u[aval ^ 0x55]); 
    return (INT16)ubuf;   
}   
    
/**************************************************************************
 * Function     : g711_ulaw2alaw
 *
 * Description  : This function is used to convert one 8 bit u-law sample to 
 *                one 8 bit a-law sample
 *
 * Parameters   : u_byte: one 8 bit sample input of u-law
 *                
 * Returns      : one 8 bit sample output of a-law
 *
 *************************************************************************/
INT16 g711_ulaw2alaw(INT16 u_byte)   
{   
    UINT8 uval = u_byte & 0xFF;
    UINT8 ubuf = (uval & 0x80) ? (0xD5 ^ (g711_u2a[0xFF ^ uval] - 1)) :   
        (0x55 ^ (g711_u2a[0x7F ^ uval] - 1));
    return (INT16)ubuf;    
}   


UINT16 g711_a2u2alaw_deviation(UINT16 a_byte)
{
    UINT8 a_byte_tmp = a_byte & 0x7F;
    UINT8 a_byte_tmp_high_nibble = a_byte_tmp >> 4;
    UINT8 a_byte_tmp_low_nibble = a_byte_tmp & 0x0F;

    switch (a_byte_tmp_high_nibble)
    {
    case 1:
        if (a_byte_tmp_low_nibble == 0x0A)
        {
            a_byte++;
        }
        break;
        
    case 4:
        if ((a_byte_tmp_low_nibble & 0x08) && !(a_byte_tmp_low_nibble & 0x01))            
        {
            a_byte++;
        }        
        break;
        
    case 6:
        if (a_byte_tmp_low_nibble == 0x0B)
        {
            a_byte--;
        }        
        break;
        
    case 7: 
        if ((a_byte_tmp_low_nibble == 0x09) || (a_byte_tmp_low_nibble == 0x0B))
        {
            a_byte--;
        }          
        break;   
        
    default:
        break;
    }


    return a_byte;

}

UINT16 g711_u2a2ulaw_deviation(UINT16 u_byte)
{
    UINT8 high_nibble = (u_byte >> 4) & 0xF;

    switch (high_nibble)    
    {
    case 0x07:
    case 0x0F:        
        if (u_byte & 0x01)
        {
            u_byte--;                        
        }
        break;

    default:
        break;
    }
    return u_byte;
}


/**************************************************************************
 * Function     : linear_pcm_conv_to_16bit_2s
 *
 * Description  : This function is used to convert varied linear PCM input 
 *                format to 16 bit linear PCM 2's complement patterns
 *
 * Parameters   : in_type: the type of linear input format
 *                in_8bit: 8 bit or 16 bit input format
 *                pcm_in: pcm input data
 *                
 * Returns      : translative data
 *
 *************************************************************************/
INT16 linear_pcm_conv_to_16bit_2s(UINT8 in_type, UINT8 in_8bit, INT16 pcm_in)
{    
    UINT16 u_pcm_in = (UINT16)pcm_in;
    UINT16 u_pcm_out;    

    if (in_8bit)
    {
        u_pcm_in &= 0xFF;
        u_pcm_out = u_pcm_in;
        
        switch (in_type)
        {
        case BZDMA_CODEC_PCM_TYPE_1S:           
            if (u_pcm_in & BIT(7))
            {
                u_pcm_out++;
            }
            break;
            
        case BZDMA_CODEC_PCM_TYPE_SIGN:
            if (u_pcm_in & BIT(7))
            {
                u_pcm_out = (~u_pcm_in & 0xFF) + 0x81;
            }
            break;
            
        case BZDMA_CODEC_PCM_TYPE_UNSIGN:
            u_pcm_out ^= 0x80; 
            
            break;
            
        default:
            break;
        }
        u_pcm_out = (u_pcm_out & 0xFF) << 8;
    }
    else
    {
        u_pcm_out = u_pcm_in;
        
        switch (in_type)
        {
        case BZDMA_CODEC_PCM_TYPE_1S:
            if (u_pcm_in & BIT(15))
            {
                u_pcm_out++;
            }
            break;   
            
        case BZDMA_CODEC_PCM_TYPE_SIGN:
            if (u_pcm_in & BIT(15))
            {
                u_pcm_out = (~u_pcm_in) + 0x8001;
            }
            break;
            
        case BZDMA_CODEC_PCM_TYPE_UNSIGN:
            u_pcm_out ^= 0x8000;
            break;
            
        default:
            break;
        }
    }
    return (INT16)u_pcm_out;    
}


/**************************************************************************
 * Function     : linear_pcm_conv_from_16bit_2s
 *
 * Description  : This function is used to convert 16 bit linear PCM 2's 
 *                complement patterns to varied linear PCM format
 *
 * Parameters   : out_type: the type of linear output format
 *                out_8bit: 8 bit or 16 bit output format
 *                pcm_in: pcm input data
 *                
 * Returns      : translative data
 *
 *************************************************************************/
INT16 linear_pcm_conv_from_16bit_2s(UINT8 out_type, UINT8 out_8bit, INT16 pcm_in)
{
    UINT16 u_pcm_in = (UINT16)pcm_in;
    UINT16 u_pcm_out;
    
    if (out_8bit)
    {
        u_pcm_in >>= 8;
        u_pcm_out = u_pcm_in;
        
        switch (out_type)
        {
        case BZDMA_CODEC_PCM_TYPE_1S:           
            if (u_pcm_in > 0x80)
            {
                u_pcm_out = u_pcm_in--;
            }
            break;
            
        case BZDMA_CODEC_PCM_TYPE_SIGN:
            if (u_pcm_in > 0x80)
            {
                u_pcm_out = ((~u_pcm_in & 0xFF) + 0x81) & 0xFF;
            }
            break;
            
        case BZDMA_CODEC_PCM_TYPE_UNSIGN:
            u_pcm_out ^= 0x80;               
            break;
            
        default:
            break;
        }
    }
    else
    {
        u_pcm_out = u_pcm_in;
        
        switch (out_type)
        {
        case BZDMA_CODEC_PCM_TYPE_1S:
            if (u_pcm_in > 0x8000)
            {
                u_pcm_out = u_pcm_in--;
            }
            break;   
            
        case BZDMA_CODEC_PCM_TYPE_SIGN:
            if (u_pcm_in > 0x8000)
            {
                u_pcm_out = ~u_pcm_in + 0x8001;
            }

            break;
            
        case BZDMA_CODEC_PCM_TYPE_UNSIGN:
            u_pcm_out ^= 0x8000;
            break;
            
        default:
            break;
        }
    }
    return (INT16)u_pcm_out;    
}


