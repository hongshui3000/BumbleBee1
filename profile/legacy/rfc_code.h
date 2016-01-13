/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       rfc_code.h
* @brief     RFCOMM Codes
* @details   
*
* @author   	gordon
* @date      	2015-06-29
* @version	v0.1
*/
#ifndef __RFC_CODE_H
#define __RFC_CODE_H

#define RFCOMM_N1           127    /**< Default Frame Size */
#define RFCOMM_N2             0    /**< Number of Retransmissions */
#define RFCOMM_K              0    /**< Window Size */
#define RFCOMM_T2            30    /**< Default timeout for p/f reply */
#define RFCOMM_T1             0    /**< Default timeout for command reply */

/** RFCOMM Header Types */
#define RFCOMM_SABM       0x2F
#define RFCOMM_UA         0x63
#define RFCOMM_DM         0x0F
#define RFCOMM_DISC       0x43
#define RFCOMM_UIH        0xEF
#define RFCOMM_UI         0x03

/** Poll Bit */
#define RFCOMM_POLL       0x10

/** RFCOMM DLC 0 Type Field Codings */
#define RFCOMM_TYPE_PN    0x20   /**< Parameter Negotiation */
#define RFCOMM_TYPE_PSC   0x10   /**< Power Saving Control */
#define RFCOMM_TYPE_CLD   0x30   /**< Mux Closedown */
#define RFCOMM_TYPE_TEST  0x08   /**< Echo Command */
#define RFCOMM_TYPE_FCON  0x28   /**< Flow Control ON */
#define RFCOMM_TYPE_FCOFF 0x18   /**< Flow Control OFF */
#define RFCOMM_TYPE_MSC   0x38   /**< Modem Status Command */
#define RFCOMM_TYPE_NSC   0x04   /**< Non Supported Command */
#define RFCOMM_TYPE_RPN   0x24   /**< Remote Port Negotiation */
#define RFCOMM_TYPE_RLS   0x14   /**< Remote Line Status Command */
#define RFCOMM_TYPE_SNC   0x34   /**< Remote Service Negotiation */

/** internal code */
#define RFCOMM_TYPE_QDATA 0x0C   /**< qualified data (expedited service) */

/** PN Codings for Info Transfer */
#define RFCOMM_INFO_UIH   0x00
#define RFCOMM_INFO_UI    0x01
#define RFCOMM_INFO_I     0x02

/** SIZES */
#define RFCOMM_SIZE_PN    8      /**< Size of PN Parameter Set */
#define RFCOMM_SIZE_RPN   8      /**< Size of RPN Parameter Set */

#define RFCOMM_SIZE_UIH   4      /**< Maximum size of UIH header (with 16 bit length field) excluding credit field, excluding checksum ! */

/** Convergence Layer Definitions */
#define RFCOMM_CONVERGENCE_0             0   /**< Std Convergence Layer */
#define RFCOMM_CONVERGENCE_CREDIT_REQ   15   /**< Convergence Layer with Credit Based Flow Control */
#define RFCOMM_CONVERGENCE_CREDIT_CONF  14   /**< Convergence Layer with Credit Based Flow Control, positive reply */

/** RFCOMM Layer To Layer Error Code */
#define RFCOMM_SUCCESS                        0x00
//#define RFCOMM_ERR                  			0x400
//#define RFCOMM_ERR_NORESOURCES                0x01
//#define RFCOMM_ERR_ILL_PARAMETER              0x02

/** RFCOMM Local Error Codes */
#define RFCOMM_ERR_REJECTED               0x64  /**< 100 Connection setup was rejected by remote side (DM) */
#define RFCOMM_ERR_TIMEOUT                 0x65  /**< 101 Connection timed out  */
#define RFCOMM_ERR_NSC                        0x66  /**< 102 Non Supported Command received */
#define RFCOMM_ERR_ILLPARAMETER         0x67  /**< 103 Illegal parameter */
#define BLUEFACE_FLOW_BIT   0x02     /**< Flow Bit Position in RFCOMM MSC Command */


#if F_BT_RFCOMM_CONFORMANCE
/**  */

typedef enum _TRFCDeviceRole
{
	rfcDeviceRoleIUT,
	rfcDeviceRoleTester,
	rfcDeviceRoleNone
} TRFCDeviceRole;

typedef enum _TRFCTestCaseId
{
	/** Initialize RFCOMM Session */
	rfcTC_TP_RFC_BV_01 = 1,
	rfcTC_TP_RFC_BV_02 = 2,

	/** Shutdown RFCOMM Session */
	rfcTC_TP_RFC_BV_03 = 3,
	rfcTC_TP_RFC_BV_04 = 4,

	/** Establish DLC */
	rfcTC_TP_RFC_BV_05 = 5,
	rfcTC_TP_RFC_BV_06 = 6,
	/**  rfcTC_TP_RFC_BV_20 = 20, */
	/**  rfcTC_TP_RFC_BV_23 = 23, */
	/**  rfcTC_TP_RFC_BV_24 = 24, */

	/** Disconnect DLC */
	rfcTC_TP_RFC_BV_07 = 7,
	rfcTC_TP_RFC_BV_08 = 8,

	/** Sending RS232 control signals */
	/**  rfcTC_TP_RFC_BV_09 = 9, */

	/** Transfer information */
	/**  rfcTC_TP_RFC_BV_10 = 10, */

	/** Test command */
	rfcTC_TP_RFC_BV_11 = 11,

	/** Aggregate Flow Control */
	/**  rfcTC_TP_RFC_BV_12 = 12, */

	/** Remote Line Status Indication */
	rfcTC_TP_RFC_BV_13 = 13,
	rfcTC_TP_RFC_BV_14 = 14,

	/** DLC Parameter Negotiations */
	rfcTC_TP_RFC_BV_15 = 15,

	/** Remote Port Negitiations */
	rfcTC_TP_RFC_BV_17 = 17,
	rfcTC_TP_RFC_BV_19 = 19,

	/** Credir Based Flow Control */
	rfcTC_TP_RFC_BV_21 = 21,
	rfcTC_TP_RFC_BV_22 = 22,

	rfcTC_Last = 99
} TRFCTestCaseId;

typedef struct _TRFCTestConfiguration
{
	TRFCDeviceRole Role;
	TRFCTestCaseId TCID;
	WORD           Count;
} TRFCTestConfiguration;
#endif /**< F_BT_RFCOMM_CONFORMANCE */

#endif
