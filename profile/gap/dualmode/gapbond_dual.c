enum { __FILE_NUM__ = 0 };

/**
 ***************************************************************************************
 *               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
 ***************************************************************************************
 * @file      gapbond_dual.c
 * @brief    This file provides gap bond dualmode role functions
 * @details
 * @author  kyle_xu
 * @date     2015-11-18
 * @version  v0.1
 ***************************************************************************************
 */
#include <gap.h>
#include <gapbond_dual.h>
#include <blueapi.h>

/**
 * @brief           set a dual mode GAP Bond Manager parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Parameter ID and it will set the
 *                      GAP Bond Parameter.  GAP Bond Parameters are defined in (gapbond_common.h).  Also,
 *                      the "len" field must be set to the size of a "uint16_t" and the
 *                      "pValue" field must point to a "uint16".
 *
 * @param         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param         len - length of data to write
 * @param         pvalue - pointer to data to write.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID)
 */

TGAP_STATUS GAPBonddual_SetParameter(uint16_t param, uint8_t len, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {    
    case GAPBOND_PAIRING_MODE:
    case GAPBOND_MITM_PROTECTION:
    case GAPBOND_IO_CAPABILITIES:
    case GAPBOND_OOB_ENABLED:
    case GAPBOND_PASSKEY:
        ret = GAPBondCom_SetParameter(param, len, pvalue);
        break;

    case GAPBOND_BTMODE:
    case GAPBOND_OOB_DATA_R:
    case GAPBOND_OOB_DATA_C:
        ret = GAPBondlegacy_SetParameter(param, len, pvalue);
        break;
    
    case GAPBOND_OOB_DATA:
    case GAPBOND_FIXED_PASSKEY_ENABLE:
    case GAPBOND_SEC_REQ_ENABLE:
    case GAPBOND_SEC_REQ_REQUIREMENT:
        ret = GAPBondMgr_SetParameter(param, len, pvalue);
        break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ret;
}


/**
 * @brief           get a dual mode GAP Bond Manager Parameter.
 *
 *                      NOTE: You can call this function with a GAP Bond Manager Parameter ID and it will get a
 *                      GAP Bond Manager Parameter.  GAP Bond Manager  Parameters are defined in (gapbond_common.h).  Also, the
 *                      "pValue" field must point to a "uint16".
 *
 * @param         param - Profile parameter ID: @ref GAPBOND_MANAGER_PARAMETERS
 * @param         pvalue - pointer to location to get the value.  This is dependent on
 *                      the parameter ID and WILL be cast to the appropriate
 *                      data type (example: data type of uint16 will be cast to
 *                      uint16 pointer).
 *
 * @return          gapAPI_CauseSuccess or gapAPI_InvalidPara (invalid paramID)
 */
TGAP_STATUS GAPBonddual_GetParameter(uint16_t param, void *pvalue)
{
    TGAP_STATUS ret = gapAPI_CauseSuccess;

    switch (param)
    {
    case GAPBOND_BTMODE:
    case GAPBOND_OOB_DATA_R:
    case GAPBOND_OOB_DATA_C:
        ret = GAPBondlegacy_GetParameter(param, pvalue);
        break;

    case GAPBOND_PAIRING_MODE:
    case GAPBOND_MITM_PROTECTION:
    case GAPBOND_IO_CAPABILITIES:
    case GAPBOND_OOB_ENABLED:
    case GAPBOND_PASSKEY:
        ret = GAPBondCom_GetParameter(param, pvalue);
        break;

    case GAPBOND_OOB_DATA:
    case GAPBOND_FIXED_PASSKEY_ENABLE:
    case GAPBOND_SEC_REQ_ENABLE:
    case GAPBOND_SEC_REQ_REQUIREMENT:
        ret = GAPBondMgr_GetParameter(param, pvalue);
        break;

    default:
        ret = gapAPI_InvalidPara;
        break;
    }

    return ret;
}

/**
 * @brief      send passkey to gap bond manager    
 * @param    void
 * @return    void
 */
void GAPBonddual_InputPassKey(void)
{
    GAPBondCom_InputPassKey();
}

/**
 * @brief      send legacy oob data to gap bond manager when pairing with out of bond,
 *                and local should input oob data.       
 * @param   void
 * @return   void
 */
void GAPBonddual_InputlegacyOOBData(void)
{
    GAPBondlegacy_InputOOBData();
}

/**
 * @brief      send LE oob data to gap bond manager when pairing with out of bond,
 *                and local should input oob data.       
 * @param   void
 * @return    void
 */
void GAPBonddual_InputLEOOBData(void)
{
    GAPBondMgr_InputOobData();
}
