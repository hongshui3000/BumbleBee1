#include "mem.h"

void* memcpy(void * restrict dest, const void * restrict source, size_t size)
{
    UINT32 dstp = (UINT32) dest;
    UINT32 srcp = (UINT32) source;

    if (size <= 0)
    {
        return NULL;
    }

    /* If copied length is not long, it has higher efficiency to use byte 
       copied directly */
       
    if (size >= 8)
    {
        if ((dstp & 0x03) == (srcp & 0x03))
        {
            /* case 1: destination address and source address is offset 
               alignment after 4-byte aligned boundary */
            
            while (dstp & 0x03)
            {
                *(UINT8 *) dstp = *(UINT8 *)srcp;
                dstp++;
                srcp++;
                size--;
            }

            while (size > 3)
            {
                *(UINT32 *) dstp = *(UINT32 *) srcp;
                dstp += 4;
                srcp += 4;
                size -= 4;
            }
        }
        else if (!(dstp & 0x01) && !(srcp & 0x01))
        {
            /* case 2: destination address and source address are 2-byte align-
               ment but their offsets after 4-byte aligned boundary are not the
               same */

            while (size > 1)
            {
                *(UINT16 *) dstp = *(UINT16 *) srcp;
                dstp += 2;
                srcp += 2;
                size -= 2;
            }            
        }
    }

    /* Write the last few bytes.  */
    while (size > 0)
    {
        *(UINT8 *) dstp = *(UINT8 *)srcp;
        dstp++;
        srcp++;        
        size--;
    }

    return dest;
}

void* memmove(void *dest, const void *source, size_t size)
{
    UINT32 dstp = (UINT32) dest;
    UINT32 srcp = (UINT32) source;

    if (size <= 0)
    {
        return NULL;
    }

    if (srcp > dstp) /* Forward copy */
    {
        if (size >= 8)
        {
            if ((dstp & 0x03) == (srcp & 0x03))
            {
                /* case 1: destination address and source address is offset
                   alignment after 4-byte aligned boundary */

                while (dstp & 0x03)
                {
                    *(UINT8 *) dstp = *(UINT8 *) srcp;
                    dstp++;
                    srcp++;
                    size--;
                }

                while (size > 3)
                {
                    *(UINT32 *) dstp = *(UINT32 *) srcp;
                    dstp += 4;
                    srcp += 4;
                    size -= 4;
                }
            }
            else if (!(dstp & 0x01) && !(srcp & 0x01))
            {
                /* case 2: destination address and source address are 2-byte align-
                   ment but their offsets after 4-byte aligned boundary are not the
                   same */

                while (size > 1)
                {
                    *(UINT16 *) dstp = *(UINT16 *) srcp;
                    dstp += 2;
                    srcp += 2;
                    size -= 2;
                }
            }
        }

        /* Write the last few bytes.  */
        while (size > 0)
        {
            *(UINT8 *) dstp = *(UINT8 *) srcp;
            dstp++;
            srcp++;
            size--;
        }
    }
    else /* Backward copy */
    {
        dstp += size;
        srcp += size;
        if (size >= 8)
        {
            if ((dstp & 0x03) == (srcp & 0x03))
            {
                /* case 1: destination address and source address is offset
                   alignment after 4-byte aligned boundary */

                while (dstp & 0x03)
                {
                    dstp--;
                    srcp--;
                    *(UINT8 *) dstp = *(UINT8 *) srcp;
                    size--;
                }

                while (size > 3)
                {
                    dstp -= 4;
                    srcp -= 4;
                    *(UINT32 *) dstp = *(UINT32 *) srcp;
                    size -= 4;
                }
            }
            else if (!(dstp & 0x01) && !(srcp & 0x01))
            {
                /* case 2: destination address and source address are 2-byte align-
                   ment but their offsets after 4-byte aligned boundary are not the
                   same */

                while (size > 1)
                {
                    dstp -= 2;
                    srcp -= 2;
                    *(UINT16 *) dstp = *(UINT16 *) srcp;
                    size -= 2;
                }
            }
        }

        /* Write the last few bytes.  */
        while (size > 0)
        {
            dstp--;
            srcp--;
            *(UINT8 *) dstp = *(UINT8 *) srcp;
            size--;
        }
    }

    return dest;
}

void* memset_word(void *buf, int val, size_t size)
{
    UINT16 c = (UINT16)val;
    UINT32 dstp = (UINT32) buf;
    UINT8 sample[2];
    UINT8 toggle = 0;

    if (size <= 0)
    {
        return NULL;
    }
    
    sample[0] = c & 0xff;
    sample[1] = c >> 8;    
        
    /* If copied length is not long, it has higher efficiency to use byte 
       copied directly */
       
    if (size >= 8)
    {
        UINT32 cc;                

        while (dstp & 0x03)
        {
            *(UINT8 *) dstp = sample[toggle];
            dstp++;
            toggle = !toggle;
            size--;
        }
                  
        cc = (sample[!toggle] << 8) | (sample[toggle]);
        cc |= cc << 16;

        while (size > 3)
        {
            *(UINT32 *) dstp = cc;
            dstp += 4;
            size -= 4;
        }
    }

    
    /* Write the last few bytes.  */
    while (size > 0)
    {        
        *(UINT8 *) dstp = sample[toggle];
        dstp++;
        toggle = !toggle;        
        size--;
    }

    return NULL;    
}

void* memset(void *buf, int val, size_t size)
{
    UINT8 c = (UINT8)val;
    UINT32 dstp = (UINT32) buf;

    if (size <= 0)
    {
        return NULL;
    }    

    /* If copied length is not long, it has higher efficiency to use byte 
       copied directly */
       
    if (size >= 8)
    {
        UINT32 cccc = c;                
        if (c != 0)
        {            
            cccc |= cccc << 8;
            cccc |= cccc << 16;
        }

        while (dstp & 0x03)
        {
            *(UINT8 *) dstp = c;
            dstp++;
            size--;
        }

        while (size > 3)
        {
            *(UINT32 *) dstp = cccc;
            dstp += 4;
            size -= 4;
        }
    }

    /* Write the last few bytes.  */
    while (size > 0)
    {
        *(UINT8 *) dstp = c;
        dstp++;
        size--;
    }

    return NULL;
}

int memcmp(const void *dest, const void *source, size_t size)
{
	if ((dest == source) || (size <= 0))
	{
		return 0;
	}

	UINT8 *tmp1 = (UINT8 *)source;
	UINT8 *tmp2 = (UINT8 *)dest;

	while ((size > 0) && (*tmp1 == *tmp2))
	{
		tmp1++;
		tmp2++;
		size--;
	}

	if (size == 0)
	{
		return 0;
	}
	else
	{
		if (*tmp1 > *tmp2)
			return -1;
		else
			return 1;
	}
}
