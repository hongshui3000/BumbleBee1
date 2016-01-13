/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        l2c_code.h
* @brief     
* @details   
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/ 

/** L2CAP ERROR codes : now blueFace.h */

#define L2CAP_STAT_AUTHENTICATION_PENDING	0x0001
#define L2CAP_STAT_AUTHORIZATION_PENDING	0x0002

#define L2CAP_CMDREJ_NOT_UNDERSTOOD	0x0000
#define L2CAP_CMDREJ_MTU_EXCEEDED  	0x0001
#define L2CAP_CMDREJ_INVALID_CID   	0x0002


#define L2CAP_CFGRSP_SUCCESS           0x0000
#define L2CAP_CFGRSP_UNACCEPTABLE_PARA 0x0001
#define L2CAP_CFGRSP_REJECTED          0x0002
#define L2CAP_CFGRSP_UNKNOWN_OPTIONS   0x0003

/** Error code (defined also in BTERRCODE.H) for enhanced feature. */
#define L2CAP_ERR_EM_RXSEQ_INV         0x10
#define L2CAP_ERR_EM_RXTXSEQ_INV       0x11
#define L2CAP_ERR_EM_NO_RESPONSE       0x12
#define L2CAP_ERR_EM_MISSING_IFRAME    0x13
#define L2CAP_ERR_EM_FRAME_INV         0x14

#if F_BT_L2C_ENHANCED_CONFORMANCE
/****************************************************************************/

typedef enum _TL2CDeviceRole
{
	l2cDeviceRoleIUT,
	l2cDeviceRoleTester,
	l2cDeviceRoleNone
} TL2CDeviceRole;

typedef enum _TL2CTestCaseId
{
	/** Connection oriented services */
	/** Basic operation data channel */
	l2cTC_TP_CED_BV_First,
	l2cTC_TP_CED_BV_01    = l2cTC_TP_CED_BV_First,
	l2cTC_TP_CED_BV_02,     /**< dummy definition */
	l2cTC_TP_CED_BV_03,
	l2cTC_TP_CED_BV_04,
	l2cTC_TP_CED_BV_05,
	l2cTC_TP_CED_BV_06,     /**< dummy definition */
	l2cTC_TP_CED_BV_07,
	l2cTC_TP_CED_BV_08,
	l2cTC_TP_CED_BV_09,
	l2cTC_TP_CED_BV_10,     /**< dummy definition FLC */
	l2cTC_TP_CED_BV_11,
	l2cTC_TP_CED_BV_Last,

	l2cTC_TP_CED_BI_First,
	l2cTC_TP_CED_BI_01    = l2cTC_TP_CED_BI_First,
	l2cTC_TP_CED_BI_Last,

	/** Configuration of data channel */
	l2cTC_TP_CFD_BV_First,
	l2cTC_TP_CFD_BV_01    = l2cTC_TP_CFD_BV_First,
	l2cTC_TP_CFD_BV_02,
	l2cTC_TP_CFD_BV_03,
	l2cTC_TP_CFD_BV_04,     /**< dummy definition */
	l2cTC_TP_CFD_BV_05,     /**< dummy definition */
	l2cTC_TP_CFD_BV_06,     /**< dummy definition */
	l2cTC_TP_CFD_BV_07,     /**< dummy definition */
	l2cTC_TP_CFD_BV_08,
	l2cTC_TP_CFD_BV_09,
	l2cTC_TP_CFD_BV_10,
	l2cTC_TP_CFD_BV_11,
	l2cTC_TP_CFD_BV_12,
	l2cTC_TP_CFD_BV_13,
	l2cTC_TP_CFD_BV_Last,

	/** Implementation specific information exchange */
	l2cTC_TP_IEX_BV_First,
	l2cTC_TP_IEX_BV_01    = l2cTC_TP_IEX_BV_First,
	l2cTC_TP_IEX_BV_02,
	l2cTC_TP_IEX_BV_Last,

	/** Echo handling */
	l2cTC_TP_ECH_BV_First,
	l2cTC_TP_ECH_BV_01    = l2cTC_TP_ECH_BV_First,
	l2cTC_TP_ECH_BV_02,
	l2cTC_TP_ECH_BV_Last,

	/** Support of Enhanced L2CAP extended features */
	l2cTC_TP_EXF_BV_First,
	l2cTC_TP_EXF_BV_01    = l2cTC_TP_EXF_BV_First,
	l2cTC_TP_EXF_BV_02,
	l2cTC_TP_EXF_BV_03,
	l2cTC_TP_EXF_BV_Last,

	/** Channel Mode configuration */
	l2cTC_TP_CMC_BV_First = l2cTC_TP_EXF_BV_Last,
	l2cTC_TP_CMC_BV_01    = l2cTC_TP_CMC_BV_First,
	l2cTC_TP_CMC_BV_02,
	l2cTC_TP_CMC_BV_03,
	l2cTC_TP_CMC_BV_04,
	l2cTC_TP_CMC_BV_05,
	l2cTC_TP_CMC_BV_06,
	l2cTC_TP_CMC_BV_07,
	l2cTC_TP_CMC_BV_08,
	l2cTC_TP_CMC_BV_09,
	l2cTC_TP_CMC_BV_10,
	l2cTC_TP_CMC_BV_11,
	l2cTC_TP_CMC_BV_12,
	l2cTC_TP_CMC_BV_13,
	l2cTC_TP_CMC_BV_14,
	l2cTC_TP_CMC_BV_15,
	l2cTC_TP_CMC_BV_Last,

	l2cTC_TP_CMC_BI_First = l2cTC_TP_CMC_BV_Last,
	l2cTC_TP_CMC_BI_01    = l2cTC_TP_CMC_BI_First,
	l2cTC_TP_CMC_BI_02,
	l2cTC_TP_CMC_BI_03,
	l2cTC_TP_CMC_BI_04,
	l2cTC_TP_CMC_BI_05,
	l2cTC_TP_CMC_BI_06,
	l2cTC_TP_CMC_BI_Last,

	/** Optional FCS configuration */
	l2cTC_TP_FOC_BV_First = l2cTC_TP_CMC_BI_Last,
	l2cTC_TP_FOC_BV_01    = l2cTC_TP_FOC_BV_First,
	l2cTC_TP_FOC_BV_02,
	l2cTC_TP_FOC_BV_03,
	l2cTC_TP_FOC_BV_04,
	l2cTC_TP_FOC_BV_Last,

	/** Use of Optional FCS */
	l2cTC_TP_OFS_BV_First = l2cTC_TP_FOC_BV_Last,
	l2cTC_TP_OFS_BV_01    = l2cTC_TP_OFS_BV_First,
	l2cTC_TP_OFS_BV_02,
	l2cTC_TP_OFS_BV_03,
	l2cTC_TP_OFS_BV_04,
	l2cTC_TP_OFS_BV_05,
	l2cTC_TP_OFS_BV_06,
	l2cTC_TP_OFS_BV_07,
	l2cTC_TP_OFS_BV_08,
	l2cTC_TP_OFS_BV_Last,

	/** Use of Enhanced Retransmission Mode */
	l2cTC_TP_ERM_BV_First = l2cTC_TP_OFS_BV_Last,
	l2cTC_TP_ERM_BV_01    = l2cTC_TP_ERM_BV_First,
	l2cTC_TP_ERM_BV_02,
	l2cTC_TP_ERM_BV_03,
	l2cTC_TP_ERM_BV_04,     /**< dummy definition */
	l2cTC_TP_ERM_BV_05,
	l2cTC_TP_ERM_BV_06,
	l2cTC_TP_ERM_BV_07,
	l2cTC_TP_ERM_BV_08,
	l2cTC_TP_ERM_BV_09,
	l2cTC_TP_ERM_BV_10,
	l2cTC_TP_ERM_BV_11,
	l2cTC_TP_ERM_BV_12,
	l2cTC_TP_ERM_BV_13,
	l2cTC_TP_ERM_BV_14,
	l2cTC_TP_ERM_BV_15,
	l2cTC_TP_ERM_BV_16,
	l2cTC_TP_ERM_BV_17,
	l2cTC_TP_ERM_BV_18,
	l2cTC_TP_ERM_BV_19,
	l2cTC_TP_ERM_BV_20,
	l2cTC_TP_ERM_BV_21,
	l2cTC_TP_ERM_BV_22,
	l2cTC_TP_ERM_BV_23,
	l2cTC_TP_ERM_BV_Last,

	l2cTC_TP_ERM_BI_First = l2cTC_TP_ERM_BV_Last,
	l2cTC_TP_ERM_BI_01    = l2cTC_TP_ERM_BI_First,
	l2cTC_TP_ERM_BI_02,
	l2cTC_TP_ERM_BI_03,
	l2cTC_TP_ERM_BI_04,
	l2cTC_TP_ERM_BI_05,
	l2cTC_TP_ERM_BI_Last,

	/** Use of Streaming Mode */
	l2cTC_TP_STM_BV_First = l2cTC_TP_ERM_BI_Last,
	l2cTC_TP_STM_BV_01    = l2cTC_TP_STM_BV_First,
	l2cTC_TP_STM_BV_02,
	l2cTC_TP_STM_BV_03,
	l2cTC_TP_STM_BV_Last,

	/** Low Energy System Tests */
	l2cTC_TP_CPU_BV_First = l2cTC_TP_STM_BV_Last,
	l2cTC_TP_CPU_BV_01    = l2cTC_TP_CPU_BV_First,
	l2cTC_TP_CPU_BV_02,
	l2cTC_TP_CPU_BV_Last,

	l2cTC_TP_CPU_BI_First = l2cTC_TP_CPU_BV_Last,
	l2cTC_TP_CPU_BI_01    = l2cTC_TP_CPU_BI_First,
	l2cTC_TP_CPU_BI_02,
	l2cTC_TP_CPU_BI_Last,

	l2cTC_TP_REJ_BI_First = l2cTC_TP_CPU_BI_Last,
	l2cTC_TP_REJ_BI_01    = l2cTC_TP_REJ_BI_First,
	l2cTC_TP_REJ_BI_Last,

	l2cTC_Last
} TL2CTestCaseId;

typedef struct _TL2CTestConfiguration
{
  TL2CDeviceRole Role;
  TL2CTestCaseId TCID;
  uint16_t           PSM;
  uint16_t           Count;
} TL2CTestConfiguration;

/****************************************************************************/
#endif /**< F_BT_L2C_ENHANCED_CONFORMANCE */

