enum { __FILE_NUM__ = 0 };
/**
 ***************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 ***************************************************************************************
 * @file      dip.c
 * @brief    This file provides device id profile functions
 * @details
 * @author  kyle_xu
 * @date     2015-12-07
 * @version  v0.1
 ***************************************************************************************
 */
#include <legacy.h>
#include <sdp_code.h>
#include <os_mem.h>

/**
 * @brief  init device id profile
 * @param  vendor_id:
 * @param  product_id:
 * @param  product_version:
 * @param  id_source:
 * @return  bool
 *
 */
bool dip_Init(uint16_t vendor_id, uint16_t id_source, uint16_t product_id, uint16_t product_version)
{
    uint16_t length = 0;
    void *pbuffer = NULL;
    bool ret = false;

    length = legacy_SDPRecordLength("<I<U> I<U> II II II II IO II>",
                                    SDP_ATTR_SERVICECLASSIDLIST, UUID_PNPINFORMATION,
                                    SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                    SDP_ATTR_DIP_SPECIFICATION_ID, 0x0103,
                                    SDP_ATTR_DIP_VENDOR_ID, vendor_id,
                                    SDP_ATTR_DIP_PRODUCT_ID, product_id,
                                    SDP_ATTR_DIP_PRODUCT_VERSION, product_version,
                                    SDP_ATTR_DIP_PRIMARY_RECORD, true,
                                    SDP_ATTR_DIP_VENDOR_ID_SOURCE, id_source
                                    );

    if (length)
    {
        pbuffer = osMemoryAllocate(RAM_TYPE_DATA_ON, length);
        length = legacy_SDPCreateDes(pbuffer,
                                    "<I<U> I<U> II II II II IO II>",
                                    SDP_ATTR_SERVICECLASSIDLIST, UUID_PNPINFORMATION,
                                    SDP_ATTR_BROWSEGROUPLIST, UUID_PUBLIC_BROWSE_GROUP,
                                    SDP_ATTR_DIP_SPECIFICATION_ID, 0x0103,
                                    SDP_ATTR_DIP_VENDOR_ID, vendor_id,
                                    SDP_ATTR_DIP_PRODUCT_ID, product_id,
                                    SDP_ATTR_DIP_PRODUCT_VERSION, product_version,
                                    SDP_ATTR_DIP_PRIMARY_RECORD, true,
                                    SDP_ATTR_DIP_VENDOR_ID_SOURCE, id_source
                                    );

        if (length)
        {
            legacy_AddSDPRecord(pbuffer, length);
            ret = true;
        }
        else
        {
            osMemoryFree(pbuffer);
        }
    }

    return ret;
}
