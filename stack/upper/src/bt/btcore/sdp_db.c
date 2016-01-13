/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       sdp_db.c
* @brief     SDP Layer : Service Database Access Functions
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#include <flags.h>
#include <os_message.h>
#include <blueface.h>
#include <sdp_code.h>
#include <sdplib.h>
#include <btcommon.h>
#include <sdp.h>
#include <upper_stack_global.h>

/**
* @brief  Search <attribute> in  service <handle>, return final formatted attribute value string   
* 		length is returned in aLen. Value of next attribute value is returned in *nextAttributeP 
*
* @param  pSDP: Instance data
* @param  handle
* @param  attribute
* @param  aLen
* @param  nextAttributeP
*
* @return  
*
*/
uint8_t * sdpDbSearchAttribute(uint32_t handle, uint16_t attribute, LPWORD aLen, LPWORD nextAttributeP)
{

    uint16_t i = 0;
    uint8_t * service;
    uint8_t * p;
    uint8_t * pp;
    uint16_t len;
    uint8_t typ;
    uint16_t thisAttribute;
    static uint8_t sHeader[1+4 /* enough for a uint32_t on the line */];
    /* note: sHeader is declared static because we return its adress to the caller! */

    *nextAttributeP = attribute+1; /* not completely bad first guess */

    /* check for valid handle value */
    if (handle && (
        (handle < SDP_SERVICE_OFFSET) || (handle >= (SDP_SERVICE_OFFSET + BT_SDP_SERVER_MAX_SERVICES))
        ))
	{
    	return NULL;
	}

    /* load service pointer and check if service exists */
    if (handle)
	{
    	service = pSDP->services[handle - SDP_SERVICE_OFFSET];
	}
    else
	{
    	service = pSDP->services[0];
	}

    if (!service)
	{
    	return NULL;
	}

    /* some attribute cannot be found in the DB, they are defined implicitely */
    if (attribute == SDP_ATTR_SERVICERECORDHANDLE)
    {
        (void)sdpDesHeader(sHeader,0,SDP_TYP_UINT,4);
        if (handle==SDP_SERVICE_OFFSET)
        {
            NETLONG2CHAR(sHeader+1, (uint32_t)0);  /* externally visible as zero */
        }
        else
        {
            NETLONG2CHAR(sHeader+1, handle);
        }
        *aLen = 5;
        return sHeader;
    }

    /* service data base state is only defined for the main record, if accessed
       return the value of the internal database state variable
    */
    if (attribute == SDP_ATTR_SERVICEDATABASESTATE)
    {
        if (handle == 0 || handle == SDP_SERVICE_OFFSET)
        {
            (void)sdpDesHeader(sHeader,0,SDP_TYP_UINT,4);
            NETLONG2CHAR(sHeader+1, (uint32_t)pSDP->dbState);
            *aLen = 5;
            return sHeader;
        }
        /* non main records fall */
    }

    i = 1;
    while (1)
    {
        p = sdpAccessElement(service,NULL,i);
        if (!p)
        {
            /* no more elements */
            *nextAttributeP = 0;
            return NULL;
        }
        thisAttribute = sdpGetValue(p,NULL);

        /* attribute values in DB are sorted: if this one is > attribute,
           the search can be stopped */
        if (thisAttribute > attribute)
        {
            *nextAttributeP = thisAttribute;
            return NULL;
        }

        if (thisAttribute == attribute)
        {
            p = sdpAccessElement(service,NULL,(uint16_t)(i+1));
            if (!p)
        	{
            	return NULL;
        	}
            pp = sdpDecodeElement(p,NULL,&len,&typ);
            if (!pp)
        	{
            	return NULL;
        	}
            *aLen = (uint16_t)((pp + len) - p);
            return p;
        }
        i += 2; /* skip to the next attribute / value pair */
    } /* while */
} /* sdpDbSearchAttribute */

/**
* @brief  Search for pattern in local services database
*
* @param  pSDP: Instance data
* @param  pattern: DataElementSequence with list of UUIDs
* @param  pe: End of DataElementSequence
* @param  index: startindex of values to return in phandle
* @param  totalMax: total max no of handles to return
* @param  phandle: handle list with matching values
* @param  maxcount: max no of handles to store in the list
* @param  pmaxindex: no of total matches
*
* @return  
*
*/
uint8_t sdpDbServiceSearch(uint8_t * pattern, uint8_t * pe, uint16_t index, uint16_t totalMax, uint32_t* phandle, uint16_t maxcount, LPWORD pmaxindex)
{
    uint8_t * seq;
    uint16_t slen;
    uint8_t typ;
    uint32_t lhandle;
    uint16_t foundCnt = 0;

    /* Get pointer to first element in sequence */
    seq = sdpDecodeElement(pattern, pe, &slen, &typ);
    if (!seq || typ != SDP_TYP_SEQUENCE || slen==0)
	{
    	return SDP_INVALID_REQUEST_SYNTAX;
	}
    pe = seq + slen;

    for (lhandle=SDP_SERVICE_OFFSET; lhandle < SDP_SERVICE_OFFSET + BT_SDP_SERVER_MAX_SERVICES; lhandle++)
    {

        uint8_t res;

        if (pSDP->services[lhandle-SDP_SERVICE_OFFSET] == NULL)
    	{
        	continue;
    	}

        res = sdpDbServiceSearchList(pattern, pe, lhandle);

        if (res == SDP_NOT_FOUND)
    	{
        	continue;
    	}

        if (res != SDP_SUCCESS)
    	{
       		return res;
    	}

        /* check if this match can be inserted in the list */
        if (foundCnt >= index && foundCnt < index + maxcount)
        {
            if (lhandle == SDP_SERVICE_OFFSET)
        	{
            	phandle[foundCnt-index] = 0;         /* externally visible as zero */
        	}
            else
        	{
            	phandle[foundCnt-index] = lhandle;   /* this is in the range -> return result */
        	}
        }
        foundCnt++;
        /* check if the (total) list is full now */
        if (foundCnt >= totalMax)
    	{
        	break; /* the for loop */
    	}

    } /* all handles */

    *pmaxindex = foundCnt;   /* return no of total matches */
	
    return SDP_SUCCESS;
} /* sdpServiceSearch */

/**
* @brief  look for at Handle for any of UUIDs out of list
* 
* @param  pSDP
* @param  sList
* @param  sEnd
* @param  lhandle
*
* @return  
*
*/
uint8_t sdpDbServiceSearchList(uint8_t * sList, uint8_t * sEnd, uint32_t lhandle)
{
    uint8_t typ;
    uint16_t len;
    uint8_t * elem;
    uint8_t * myseq;

    /* Get pointer to first element in sequence */
    myseq = sdpDecodeElement(sList, sEnd, &len, &typ);

    if (!myseq || typ != SDP_TYP_SEQUENCE || len==0)
	{
    	return SDP_INVALID_REQUEST_SYNTAX;
	}

    /* Search lhandle for all elements in sequence */
    while (myseq < sEnd)
    {
        elem = sdpDecodeElement(myseq, sEnd, &len, &typ);

        /* kill if this it not a valid uuid */
        if (!elem || typ != SDP_TYP_UUID)
    	{
        	return SDP_INVALID_REQUEST_SYNTAX;
    	}

        /* is this element found in lhandle? */
        if (!sdpDbServiceSearchOne(myseq, lhandle))
    	{
        	return SDP_NOT_FOUND;
    	}

        myseq = elem + len; /* advance to next element in sequence */

    } /* while */
    return SDP_SUCCESS;
} /* sdpDbServiceSearchList */

/**
* @brief  Look for one UUID (at elem) at service indexed by handle in service DB
* 
* @param  pSDP
* @param  elem
* @param  handle
*
* @return  
*
*/
BOOL sdpDbServiceSearchOne(uint8_t * elem, uint32_t handle)
{
    uint8_t * p;      /* pointer to service record */
    uint8_t * pp;     /* current pointer within this service record */
    uint8_t * pe;     /* points to end of current service record */
    uint16_t len;      /* length of current structural element */
    uint8_t typ;      /* type of current structural element */
    uint8_t * ppElem; /* data portion of current structural element */

#if ((SDP_TRACE_VERBOSITY_COUNT<0)||(SDP_TRACE_VERBOSITY_COUNT >= 2))
    uint16_t uuid;
#endif

    /* check for illegal handle values */
    if (handle && ((handle < SDP_SERVICE_OFFSET) ||
        (handle >= (SDP_SERVICE_OFFSET + BT_SDP_SERVER_MAX_SERVICES)) ||
        (pSDP->services[handle - SDP_SERVICE_OFFSET] == NULL)))
	{
    	return FALSE;
	}

    if (handle)
	{
    	p = pSDP->services[handle - SDP_SERVICE_OFFSET]; /* get entry address for this handle */
	}
    else
	{
    	p = pSDP->services[0];                           /* get entry address for handle zero */
	}

#if ((SDP_TRACE_VERBOSITY_COUNT<0)||(SDP_TRACE_VERBOSITY_COUNT >= 2))
    uuid    = sdpGetValue(elem,NULL);
    SDP_TRACE_PRINTF_2(SDP_TRACE_MASK_ERROR,
                          "sdpDbServiceSearchOne: search for 0x%X at handle 0x%lX",
                          uuid, handle
                          );
#endif

    /* p points to SDP service record, this must always be a SDP_TYPE_SEQUENCE */
    pp = sdpDecodeElement(p,NULL,&len,&typ);
    if (typ != SDP_TYP_SEQUENCE)
	{
		return FALSE;               /* structural error in service record, indicate "not found" to caller */
	}

    pe = pp + len; /* get pointer to end of top level struct element, that is to end of service record */

	while (pp<pe)  /* search through complete service record */
	{
		/* look at the current element */
		ppElem = sdpDecodeElement(pp,pe,&len,&typ);
		if (!ppElem)
		{
			return FALSE;    /* structural error in this element, cannot continue, search fails */
		}

		switch (typ)
		{
		case SDP_TYP_UUID:
			/* looking at an UUID, if it matches we are done */
			if ((sdpCompareUUID(elem, pp))==0)
			{
				return TRUE;
			}
			pp = ppElem + len;   /* otherwise advance past this element */
			break;

		case SDP_TYP_SEQUENCE:
			/* looking at a sequence, continue at the data portion of this sequence */
			pp = ppElem;
			break;

		default:
			/* looking at some other element, skip this element and continue search */
			pp = ppElem + len;
			break;
		} /* switch */
    } /* while */
    /* search proceeded through the complete service record without match, return "not found" to caller */
    return FALSE;
} /* sdpDbServiceSearchOne */

