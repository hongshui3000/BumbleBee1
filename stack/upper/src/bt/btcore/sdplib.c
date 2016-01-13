/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       sdplib.c
* @brief     sdp using util
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/

#include <sdp_code.h>
#include <sdplib.h>

#ifndef SDP_LIB_DEBUG
#define SDPLIB_DEBUG 0
#endif

/****************************************************************************************************/
/* this is the base UUID 128 for Bluetooth */

const uint8_t  uuid128[16] = {0x00, 0x00, 0x00, 0x00, /* - */ 0x00, 0x00, /* - */ 0x10, 0x00, /* - */
                    0x80, 0x00, /* - */
                    0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB
};

/**
* @brief   Create DES sequence buf header

* @param buf: DES sequence is created in buf
* @param bp: 
* @param typ:
* @param len:
*
* @return  
*
*/
uint32_t sdpDesHeader(uint8_t * buf, uint32_t bp, uint8_t typ, uint32_t len)
{
    uint8_t six  = 0xff; /* size index, default: unknown */
    uint8_t addo = 0;
    switch (typ)
    {
    case SDP_TYP_UUID:
        if (len == 2)
        {
            six = 1;
        }
        if (len == 4)
        {
            six = 2;
        }
        if (len == 16)
        {
            six = 4;
        }
        break;

    case SDP_TYP_UINT:
    case SDP_TYP_SINT:
        if (len == 1)
        {
            six = 0;
        }
        if (len == 2)
        {
            six = 1;
        }
        if (len == 4)
        {
            six = 2;
        }
        if (len == 8)
        {
            six = 3;
        }
        if (len == 16)
        {
            six = 4;
        }
        break;

    case SDP_TYP_STRING:
    case SDP_TYP_SEQUENCE:
    case SDP_TYP_URL:
        if (len < 0x100)
        {
            six  = 5;
            addo = 1;
        }
        else if (len < 0x10000)
        {
            six  = 6;
            addo = 2;
        }
        else
        {
            six  = 7;
            addo = 4;
        }
        break;

    case SDP_TYP_SEQUENCE_WORD:
        six  = 6;
        addo = 2;
        typ  = SDP_TYP_SEQUENCE;
        break;

    case SDP_TYP_SEQUENCE_DWORD:
        six  = 7;
        addo = 4;
        typ  = SDP_TYP_SEQUENCE;
        break;

    case SDP_TYP_BOOL:
        six = 0;
        break;

    default:
        break;
    } /* switch */
    assert (six != 0xff);

    buf[bp++] = (typ << 3) | six;

    if (addo == 1)
    {
        buf[bp++] = (uint8_t) len;
    }
    else if (addo == 2)
    {
        NETSHORT2CHAR(buf+bp, len);
        bp += 2;
    }
    else if (addo == 4)
    {
        NETLONG2CHAR(buf+bp, len);
        bp += 4;
    }
    return bp;
}

/**
 * @brief   Create DES sequence from format describing string
 *    <>  : Sequence w. 8 bit length field
 *    []  : Sequence w. 16 bit length field
 *    {}  : Sequence w. 32 bit length field
 *    U   : UUID - 16, a 16 bit value follows
 *    2U  : same as U
 *    4U: : UUID - 32, a 16 bit value follows
 *    6U  : UUID - 128, a 16 bit value follows, that is expanded to a 128 bit uuid
 *    8U    UUID - 128, a pointer to a 16 byte field follows, that is used as a 128 bit uuid
 *    Y   : UINT - 128 a pointer to a 16 byte field follows,
 *    X   : UINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    L   : UINT - 32
 *    I   : UINT - 16
 *    B   : UINT - 8
 *    y   : SINT - 128 a pointer to a 16 byte field follows,
 *    x   : SINT - 64  use 2 parameters upper 32 bits and lower 32 bits
 *    l   : SINT - 32
 *    i   : SINT - 16
 *    b   : SINT - 8
 *    O   : BOOL
 *    S   : String
 *    R   : URL
 *
 * @param buf: DES sequence is created in buf
 * @param format
 *
 * @return  length of des sequence or 0 when format error
 *
 */
uint32_t sdpCreateDes(uint8_t *buf, uint8_t *format, ...)
{
    uint32_t stack[SDP_CREATE_DES_MAX_NESTING];      /* max. depth nesting of des layers in format string */
    uint8_t field_length = 0;
    uint8_t tos = 0;
    uint32_t pos = 0;
    uint32_t ref;
    uint32_t tmp;
    uint32_t sl;
    LONG  ltmp;
    BOOL more = TRUE;
    VA_LIST marker;
    char ch;
    LPSTR s;

    VA_START(marker, format);
    while (more)
    {
        ch = *format++;
        switch (ch)
        {
        case 0:
            more = FALSE;
            break;

        case ' ':       /* tolerate formatting characters */
            break;

        case '2':
        case '4':
        case '6':
        case '8':
            /* set fieldLength for next operator */
            field_length = ch - '0';
            break;

        case '<':      /* Push actual adress to stack */
            if (tos >= SDP_CREATE_DES_MAX_NESTING)
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            stack[tos++] = pos;
            pos += 2;  /* Sequence Header Length is 2 Bytes */
            break;

        case '[':      /* Push actual adress to stack */
            if (tos >= SDP_CREATE_DES_MAX_NESTING)
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            stack[tos++] = pos;
            pos += 3;  /* Sequence Header Length is 3 Bytes in this case */
            break;

        case '{':      /* Push actual adress to stack */
            if (tos >= SDP_CREATE_DES_MAX_NESTING)
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            stack[tos++] = pos;
            pos += 5;  /* Sequence Header Length is 5 Bytes in this case */
            break;

        case '>':      /* Pop adress from stack and generate correct header */
            if (tos == 0)
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            ref = stack[--tos];
            if ((pos-(ref+2)) > 0xff)   /* length too long for <> */
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            sdpDesHeader(buf, ref, SDP_TYP_SEQUENCE, (pos-(ref+2)));
            break;

        case ']':      /* Pop adress from stack and generate correct header */
            if (tos == 0)
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            ref = stack[--tos];
            if ((pos-(ref+2)) > 0xffff)   /* length too long for [] */
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            sdpDesHeader(buf, ref, SDP_TYP_SEQUENCE_WORD, (pos-(ref+3)));
            break;

        case '}':      /* Pop adress from stack and generate correct header */
            if (tos == 0)
            {
                pos = 0; /* reset result to nothing */
                more = FALSE;
                break;
            }
            ref = stack[--tos];
            sdpDesHeader(buf, ref, SDP_TYP_SEQUENCE_DWORD, (pos-(ref+5)));
            break;

        case 'U':      /* Generate UUID-16/32/128 from following word */
            switch (field_length)
            {
            case 0:
            case 2:
                /* 16 bit UUID */
                tmp = VA_ARG(marker, int);
                pos = sdpDesHeader(buf, pos, SDP_TYP_UUID, 2);
                NETSHORT2CHAR(buf+pos, tmp);
                pos += 2;
                break;

            case 4:
                /* 32 bit UUID */
                tmp = VA_ARG(marker, int);
                pos = sdpDesHeader(buf, pos, SDP_TYP_UUID, 4);
                NETLONG2CHAR(buf+pos, tmp);
                pos += 4;
                break;

            case 6:
                /* 128 bit UUID, expanded from 32 bit value */
                tmp = VA_ARG(marker, int);
                pos = sdpDesHeader(buf, pos, SDP_TYP_UUID, 16);
                NETLONG2CHAR(buf+pos, tmp);
                pos += 4;
                memcpy((PVOID)(buf+pos), (PVOID)(uuid128+4), sizeof(uuid128)-4);
                pos += sizeof(uuid128)-4;
                break;

            case 8:
                /* 128 bit UUID, expanded from pointer to 16 byte field */
                s   = VA_ARG(marker, LPSTR);
                pos = sdpDesHeader(buf, pos, SDP_TYP_UUID, 16);
                memcpy(buf+pos, s, 16);
                pos += 16;
                break;

            default:
                assert(FALSE);  /* unknown field length */
                break;
            } /* switch */
            field_length = 0;    /* reset field length tp default */
            break;

        case 'Y':   /* Generate 16 bytes UINT from following pointer */
            s   = VA_ARG(marker, LPSTR);
            pos = sdpDesHeader(buf, pos, SDP_TYP_UINT, 16);
            memcpy(buf+pos, s, 16);
            pos += 16;
            break;

        case 'X':      /* Generate UINT-64 from following 2 dwords */
            ltmp = VA_ARG(marker, LONG);
            pos  = sdpDesHeader(buf, pos, SDP_TYP_UINT, 8);
            NETLONG2CHAR(buf+pos, ltmp); pos += 4;  /* upper 32 bit */
            ltmp = VA_ARG(marker, LONG);
            NETLONG2CHAR(buf+pos, ltmp); pos += 4;  /* lower 32 bit */
            break;

        case 'L':      /* Generate UINT-32 from following dword */
            ltmp = VA_ARG(marker, LONG);
            pos  = sdpDesHeader(buf, pos, SDP_TYP_UINT, 4);
            NETLONG2CHAR(buf+pos, ltmp);
            pos += 4;
            break;

        case 'I':      /* Generate UINT-16 from following word */
            tmp = VA_ARG(marker, int);
            pos = sdpDesHeader(buf, pos, SDP_TYP_UINT, 2);
            NETSHORT2CHAR(buf+pos, tmp);
            pos += 2;
            break;

        case 'B':      /* Generate UINT-8 from following word */
            tmp = VA_ARG(marker, int);
            pos = sdpDesHeader(buf, pos, SDP_TYP_UINT, 1);
            buf[pos++] = (uint8_t)tmp;
            break;

        case 'y':   /* Generate 16 bytes SINT from following pointer */
            s   = VA_ARG(marker, LPSTR);
            pos = sdpDesHeader(buf, pos, SDP_TYP_SINT, 16);
            memcpy(buf+pos, s, 16);
            pos += 16;
            break;

        case 'x':      /* Generate SINT-64 from following 2 dwords */
            ltmp = VA_ARG(marker, LONG);
            pos  = sdpDesHeader(buf, pos, SDP_TYP_SINT, 8);
            NETLONG2CHAR(buf+pos, ltmp); pos += 4;  /* upper 32 bit */
            ltmp = VA_ARG(marker, LONG);
            NETLONG2CHAR(buf+pos, ltmp); pos += 4;  /* lower 32 bit */
            break;

        case 'l':      /* Generate SINT-32 from following dword */
            ltmp = VA_ARG(marker, LONG);
            pos  = sdpDesHeader(buf, pos, SDP_TYP_SINT, 4);
            NETLONG2CHAR(buf+pos, ltmp);
            pos += 4;
            break;

        case 'i':      /* Generate SINT-16 from following word */
            tmp = VA_ARG(marker, int);
            pos = sdpDesHeader(buf, pos, SDP_TYP_SINT, 2);
            NETSHORT2CHAR(buf+pos, tmp);
            pos += 2;
            break;

        case 'b':      /* Generate SINT-8 from following word */
            tmp = VA_ARG(marker, int);
            pos = sdpDesHeader(buf, pos, SDP_TYP_SINT, 1);
            buf[pos++] = (uint8_t)tmp;
            break;

        case 'O':      /* Generate BOOL */
            tmp = VA_ARG(marker, int);
            pos = sdpDesHeader(buf, pos, SDP_TYP_BOOL, 0);
            buf[pos++] = (uint8_t)tmp;
            break;

        case 'S':      /* Insert a string */
            s = VA_ARG(marker, LPSTR);
            sl = (strlen(s)+1); /* including trailing zero */
            pos = sdpDesHeader(buf,pos,SDP_TYP_STRING,sl);
            memcpy(buf+pos,s,sl); pos += sl;
            break;

        case 'R':      /* Insert a URL */
            s   = VA_ARG (marker, LPSTR);
            sl  = (strlen(s)+1); /* including trailing zero */
            pos = sdpDesHeader(buf,pos,SDP_TYP_URL,sl);
            memcpy(buf+pos,s,sl); pos += sl;
            break;

        default:
            pos = 0; /* reset result to nothing */
            more = FALSE;
            break;
        } /* switch */
    } /* while */
    VA_END(marker);

    return pos;
} /* sdpCreateDes */

/**
* @brief   Decode a SDP Data Element
*
* @param  element: element
* @param  elemEnd
* @param  plen: element length
* @param  ptyp: element type
*
* @return   Pointer to Data Payload, NULL on error
*
*/
uint8_t * sdpDecodeElement(uint8_t * element, uint8_t * elemEnd, LPWORD plen, uint8_t * ptyp)
{
    uint8_t siz,typ;
    uint16_t len = 0;
    BOOL err = FALSE;

    siz = *element++;
    typ = (siz >> 3) & 0x1f;	/*high 5 bit: type*/
    siz = siz & 0x7;			/*low 3 bit: size index*/

    /* first check correct typ vs siz settings */
    switch (typ) {
    case SDP_TYP_NULL:
    case SDP_TYP_BOOL:
        if (siz != 0)	/*this two types, size index need to be zero*/
		{
			err = TRUE;
		}
        break;

    case SDP_TYP_UINT:
    case SDP_TYP_SINT:
        if (siz > 4)	/*this two types: 1, 2, 3, 4*/
    	{
        	err = TRUE;
    	}
        break;

    case SDP_TYP_UUID:	/*uuid: 1, 2, 4*/
        if (siz != 1 && siz != 2 && siz != 4)
    	{
        	err = TRUE;
    	}
        break;

    case SDP_TYP_STRING:
    case SDP_TYP_SEQUENCE:
    case SDP_TYP_ALTERNATE:
    case SDP_TYP_URL:	/*this four types: 5, 6, 7, as size is 3bit, so do not check < 7*/
        if (siz < 5)
    	{
        	err = TRUE;
    	}
        break;

    default:
        err = TRUE;
    } /* switch typ */

    if (err)
    {
        return NULL;
    }

    /* second extract length info */
    switch (siz) {
    case 0:
        if (typ == SDP_TYP_NULL)	/*for null type , len is zero*/
		{
			len = 0;
		} else						/*otherwise, len is 1*/
		{
			len = 1;
		}
        break;
    case 1:
        len = 2;
        break;
    case 2:
        len = 4;
        break;
    case 3:
        len = 8;
        break;
    case 4:
        len = 16;
        break;
    case 5:
        len = *element++;
        break;
    case 6:
        len = NETCHAR2SHORT(element); element += 2;
        break;
    case 7:
        len = NETCHAR2SHORT(element+2); element += 4;  /* 32 bit length field, ignore stuff > 64k */
        break;
    } /* switch siz */

    /* element points to data payload, len is length of data portion, */
    /* typ is type, return all results (success)                      */
    if (elemEnd && (element + len) > elemEnd)
    {
        return NULL;
    }
    if (plen)
	{
		*plen = len;
	}
    if (ptyp)
	{
    	*ptyp = typ;
	}
    return element;

} /* sdpDecodeElement */

/**
* @brief   
*
* @param  p: encoded element at p, p is either s-int, uint, uuid or boolean. 
* @param  pe: limit pointer for p
*
* @return  value field from encoded element at p
*
*/
uint32_t sdpGetDValue(uint8_t * p, uint8_t * pe)
{
	uint16_t len;
	uint8_t typ;
	uint8_t * lp;

	if (!p)              /* we wanna be on the very save side */
	{
   		return 0xffffffff;
	}
	lp = sdpDecodeElement(p, pe, &len, &typ);
	if (!lp)
	{
   		return 0xffffffff;
	}
	switch (len)
	{
	case 0:
	   return 0;
	case 1:
	   return *lp;
	case 2:
	   return NETCHAR2SHORT(lp);
	case 4:
	   return NETCHAR2LONG(lp);
	case 16:
	   /* if a 16 byte UUID matches in the last 12 chars the default UUID, return first 4 bytes */
	   if (typ == SDP_TYP_UUID && memcmp(lp+4, uuid128+4, 16-4)==0)
	       return NETCHAR2LONG(lp);
	}
	return 0xffffffff;   /* safety exit */
}

/**
* @brief   extract string value into buf as cstring from element at p. pe is end pointer 
*		in case of any error return empty string, truncate string if too long to fit 
*
* @param  ds: sequence
* @param  dse
* @param  ix: index
*
* @return  
*
*/
uint16_t sdpGetString(uint8_t * p, uint8_t * pe, uint8_t * buf, uint16_t buflen)
{
    uint16_t len;
    uint8_t typ;
    uint8_t * lp;

    buf[0] = 0;          /* pre-init empty string */

    if (!p)
	{
    	return 0;        /* no input data -> no output data */
	}
    lp = sdpDecodeElement(p, pe, &len, &typ);
    if (!lp)             /* illegal element coding */
	{
    	return 0;
	}
    if (typ != SDP_TYP_STRING)  /* must be s string */
	{
    	return 0;
	}

    /* element contents starts at lp and is len bytes long */

    /* make shure the string fits into destination buffer */
    if (len >= (buflen-1))
	{
    	len = buflen-1;
	}

    memcpy(buf,lp,len);      /* copy the string */
    buf[len] = 0;            /* terminate the string */

    return len;
}

/**
* @brief   access element in sdp
*
* @param  ds: sequence
* @param  dse
* @param  ix: index
*
* @return  
*
*/
uint8_t * sdpAccessElement(uint8_t * ds, uint8_t * dse, uint16_t ix)
{
    uint16_t len;
    uint8_t typ;
    uint8_t * p;

    if (!ds)
	{
		return NULL;
	}
	
    /* decend one level down */
    p = sdpDecodeElement(ds, dse, &len, &typ);

    if (!p || typ != SDP_TYP_SEQUENCE)
        return NULL;

    dse = p + len; /* now we have a better value */

    /* search for nth element in sequence, first element is "1" */
    while (1)
    {
        if (p >= dse)                               /* already at end condition */
    	{
       		return NULL;
    	}
        if (--ix==0)
    	{
        	break;
    	}
        p = sdpDecodeElement(p, dse, &len, &typ);
        if (!p)
    	{
        	return NULL;
    	}
        p += len;
    }
    return p;
} /* sdpAccessElement */

/**
* @brief   Compare two UUIDs, return 0 if identical
*
* @param  el1
* @param  len1
* @param  el2
* @param  len2
*
* @return  
*
*/
int
compareUUID(uint8_t * el1, int len1, uint8_t * el2, int len2)
{
    uint8_t uuid1[16], uuid2[16];

    /* the fast track : uuid of same length 16/32/128) */
    if (len1 == len2)
	{
    	return memcmp(el1, el2, len1);
	}

    /* not the fast track: differing length : generate correct full length uuid (128 bit) */
    memcpy((PVOID)uuid1, (PVOID)uuid128, sizeof(uuid128));
    memcpy((PVOID)uuid2, (PVOID)uuid128, sizeof(uuid128));
    if (len1 == 2)
	{
    	memcpy((PVOID)(uuid1 + 2), (PVOID)el1, len1);
	}
    else
	{
    	memcpy((PVOID)(uuid1 + 0), (PVOID)el1, len1);
	}

    if (len2 == 2)
	{
    	memcpy((PVOID)(uuid2 + 2), (PVOID)el2, len2);
	}
    else
	{
    	memcpy((PVOID)(uuid2 + 0), (PVOID)el2, len2);
	}

    return memcmp((PVOID)uuid1, (PVOID)uuid2, sizeof(uuid1));
} /* sdpCompareUUID */

/**
* @brief   Compare two UUIDs, return 0 if identical
*
* @param  el1
* @param  el2
*
* @return  
*
*/
int
sdpCompareUUID(uint8_t * el1, uint8_t * el2)
{
    uint8_t * pel1;
    uint8_t * pel2;
    uint8_t typ1, typ2;
    uint16_t len1, len2;

    /* check if both el's are valid UUIDs */
    pel1 = sdpDecodeElement(el1,NULL,&len1,&typ1);
    pel2 = sdpDecodeElement(el2,NULL,&len2,&typ2);

    if (!pel1 || typ1 != SDP_TYP_UUID)
	{
    	return FALSE;
	}
    if (!pel2 || typ2 != SDP_TYP_UUID)
	{
    	return FALSE;
	}

    return compareUUID(pel1, len1, pel2, len2);
} /* sdpCompareUUID */

/**
* @brief   Find attribute from SDP
*
* @param  seq: sdp sequence
* @param  eseq
* @param  aVal: attribute value
*
* @return  
*
*/
uint8_t *
sdpFindAttribute(uint8_t * seq, uint8_t * eseq, uint32_t aVal)
{
	int attributeIndex = 0;
	uint8_t * attribute;

	while (1)
	{
	    ++attributeIndex;
	    attribute = sdpAccessElement(seq, eseq, (uint16_t) (2*attributeIndex-1));
	    if (!attribute)
    	{
        	break;                                 /* we have reached the end of list */
    	}
	    if (aVal == sdpGetDValue(attribute, eseq)) /* matching attribute found */
	        return sdpAccessElement(seq, eseq, (uint16_t) (2*attributeIndex));
	}
	return NULL;    /* nothing that matches found */
} /* sdpFindAttribute */

/**
* @brief  sdp access attribute element
* 
* @param  pSdpRecord
* @param  lenSdpRecord
* @param  sdpAttribute
*
* @return  
*
*/
uint8_t *
sdpAccessAttributeElement(uint8_t * pSdpRecord, uint16_t lenSdpRecord, uint32_t sdpAttribute)
{
    uint16_t   attributeIndex = 1;
    uint8_t * pAttribute;

    while(1)
    {
		pAttribute = sdpAccessElement(pSdpRecord, pSdpRecord+lenSdpRecord, attributeIndex);

		if(pAttribute)
		{
			if(sdpAttribute == sdpGetDValue(pAttribute, pSdpRecord+lenSdpRecord))
			{
				return sdpAccessElement(pSdpRecord, pSdpRecord+lenSdpRecord, (uint16_t)(attributeIndex+1));
			}
            else
			{  
				attributeIndex+=2; /* try next attribute... */
			}
		} else
		{  
			return NULL; /* attribute not found */
		}
    }
} /* sdpAccessAttributeElement */

