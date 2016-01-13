 

#ifndef __GATTDGLC_H
#define __GATTDGLC_H



#if !defined(__GATT_H)
#include <gatt.h>
#endif




/*--- optional inclusion of other services ---*/
#define GLC_SRV_INCLUDE_BAS   1   /* Battery Service      */
#define GLC_SRV_INCLUDE_CTS   1   /* Current Time Service */
#define GLC_SRV_INCLUDE_NDCS  1   /* Next DST (Daylight Saving Time) Change Service */
#define GLC_SRV_INCLUDE_RTUS  1   /* Reference Time Update Service */
#define GLC_SRV_INCLUDE_TEST  1

/* currently all or none of the optional services must be included.      */
/* otherwise some constants had to be redefined in a more flexible way!! */
#define GLC_SRV_INCLUDE_xxx (GLC_SRV_INCLUDE_BAS  + \
                             GLC_SRV_INCLUDE_CTS  + \
                             GLC_SRV_INCLUDE_NDCS + \
                             GLC_SRV_INCLUDE_RTUS)
#if (GLC_SRV_INCLUDE_xxx != 0) && (GLC_SRV_INCLUDE_xxx != 4)
#error Illegal combination of GLC_SRV_INCLUDE_xxx values!
#endif

#if (GLC_SRV_INCLUDE_BAS)
#include "gattdbas.h"
#endif
#if (GLC_SRV_INCLUDE_CTS)
#include "gattdcts.h"
#endif
#if (GLC_SRV_INCLUDE_NDCS)
#include "gattdndcs.h"
#endif
#if (GLC_SRV_INCLUDE_RTUS)
#include "gattdrtus.h"
#endif


/* number of (groups of) services registered (order MUST be DIS, GLS and */
/* optionally BAS, CTS, NDCS, RTUS !!!):                                 */

/* all services defined in one array */
#define GATTTEST_MAX_SERVICES        1





/*----- Glucose Measurement (mandatory) definitions ------*/

#define GATT_SVC_GLS_MEASUREMENT_INDEX   9  /* index of GATT_UUID_CHAR_GLS_MEASUREMENT value */
#define GATT_CHAR1_VALUE_INDEX       GATT_SVC_GLS_MEASUREMENT_INDEX

/* optional fields configuration */
#define GLC_INCLUDE_TIME_OFFSET           1
#define GLC_INCLUDE_CONC_TS_LOC           1
#define GLC_INCLUDE_SS_ANNUNC             1
#define GLC_INCLUDE_CTXT_INFORM           1


/* GLC measurement flag bits */
#define GLC_FLAGS_TIME_OFFSET_PRESENT     0x01  /* time offset                */
#define GLC_FLAGS_CONC_TS_LOC_PRESENT     0x02  /* concentration, time/sample location */
#define GLC_FLAGS_CONC_UNITS_MMOL_L       0x04  /* 0: mg/dL, 1: mmol/L        */
#define GLC_FLAGS_SS_ANNUNC_PRESENT       0x08  /* sensor status annunciation */
#define GLC_FLAGS_CTXT_INFORM_PRESENT     0x10  /* context information        */

#define GLC_FLAGS_UNITS_MMOL_L_ON         0

/* GLC sensor status annunciation bits */
#define GLC_SS_ANNUNC_LOW_BATTERY         0x0001


typedef struct _GLCMeasurementValue
{
    uint8_t    bFlags;
    UINT16     SeqNbr;
    TIMESTAMP  BaseTime;
#if (GLC_INCLUDE_TIME_OFFSET)
    signed short      TimeOffset;
#endif
#if (GLC_INCLUDE_CONC_TS_LOC)
    SFLOAT     Concentration;
    UINT8      bTSLocation;
#endif
#if (GLC_INCLUDE_SS_ANNUNC)
    uint8_t    SSAnnunciation[2];
#endif
} TGLCMeasurementValue, * PGLCMeasurementValue;


/*----- Glucose Measurement Context (optional) definitions ------*/


#define GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX   12 /* index of GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT value */


/* optional fields configuration */
#define GLC_INCLUDE_CARBOHYDRATE            1
#define GLC_INCLUDE_MEAL                    1
#define GLC_INCLUDE_TESTER_HEALTH           1
#define GLC_INCLUDE_EXCERCISE               1
#define GLC_INCLUDE_MEDICATION              1
#define GLC_INCLUDE_HbA1c                   1
#define GLC_INCLUDE_EXT_FLAGS               0



/* GLC measurement context flag bits */
#define GLC_FLAGS_CARBOHYDRATES_PRESENT     0x01  /* Carbohydrates ID and field  */
#define GLC_FLAGS_MEAL_PRESENT              0x02  /* Meal ID and field           */
#define GLC_FLAGS_TESTER_HEALTH_PRESENT     0x04  /* Tester-Health field         */
#define GLC_FLAGS_EXCERCISE_PRESENT         0x08  /* Exercise Duration and Intensity field */
#define GLC_FLAGS_MEDICATION_PRESENT        0x10  /* Medication ID and field     */
#define GLC_FLAGS_MEDICATION_UNITS_MLITERS  0x20  /* Medication units 0:mg, 1:ml */
#define GLC_FLAGS_HbA1c_PRESENT             0x40  /* HbA1c field                 */
#define GLC_FLAGS_EXT_FLAGS_PRESENT         0x80  /* extended flag               */

#define GLC_FLAGS_MEDICATION_UNITS_MLITERS_ON  1

typedef struct _GLCMeasurementContext
{
    uint8_t    bFlags;
    UINT16     SeqNbr;
#if (GLC_INCLUDE_EXT_FLAGS)
    uint8_t    bExtFlags;
#endif
#if (GLC_INCLUDE_CARBOHYDRATE)
    UINT8      CarbohydrateID;
    SFLOAT     Carbohydrate;
#endif
#if (GLC_INCLUDE_MEAL)
    UINT8      Meal;
#endif
#if (GLC_INCLUDE_TESTER_HEALTH)
    UINT8      TesterHealth;
#endif
#if (GLC_INCLUDE_EXCERCISE)
    UINT16     ExerciseDuration;
    UINT8      ExerciseIntensity;
#endif
#if (GLC_INCLUDE_MEDICATION)
    UINT8      MedicationID;
    SFLOAT     Medication;
#endif
#if (GLC_INCLUDE_HbA1c)
    SFLOAT     HbA1c;
#endif
} TGLCMeasurementContext, * PGLCMeasurementContext;




#define GATT_SVC_GLS_FEATURE_INDEX  15    /* index of feature value */


/* GLC features bits */
#define GLC_FEATURES_LOW_BATTERY                0x0001
#define GLC_FEATURES_SENSOR_MALFUNCTION         0x0002
#define GLC_FEATURES_SENSOR_SAMPLE_SIZE         0x0004
#define GLC_FEATURES_SENSOR_STRIP_INS_ERROR     0x0008
#define GLC_FEATURES_SENSOR_STRIP_TYPE_ERROR    0x0010
#define GLC_FEATURES_SENSOR_RESULT_HIGH_LOW     0x0020
#define GLC_FEATURES_SENSOR_TEMP_HIGH_LOW       0x0040
#define GLC_FEATURES_SENSOR_READ_INTERRUPT      0x0080
#define GLC_FEATURES_GENERAL_DEVICE_FAULT       0x0100
#define GLC_FEATURES_TIME_FAULT                 0x0200
#define GLC_FEATURES_MULTIPLE_BOND              0x0400

/* GLC features supported: */
#define GLC_FEATURES          (GLC_FEATURES_LOW_BATTERY    |  \
                               GLC_FEATURES_MULTIPLE_BOND)


/*----- Glucose Record Access Control Point (mandatory) definitions ------*/

#define GATT_SVC_GLS_RACP_INDEX  17    /* index of GATT_UUID_CHAR_GLS_RACP value */


typedef struct _GLCControlPoint
{
    uint8_t      OpCode;
    uint8_t      Operator;
    uint8_t    Operand[18];
} TGLCControlPoint, * PGLCControlPoint;

/* OpCode values: */
#define GLC_RACP_OPCODE_RESERVED             0x00
#define GLC_RACP_OPCODE_REPORT_RECS          0x01
#define GLC_RACP_OPCODE_DELETE_RECS          0x02
#define GLC_RACP_OPCODE_ABORT_OPERATION      0x03
#define GLC_RACP_OPCODE_REPORT_NBR_OF_RECS   0x04
#define GLC_RACP_OPCODE_NBR_OF_RECS_RESP     0x05
#define GLC_RACP_OPCODE_RESP_CODE            0x06

/* Operator values: */
#define GLC_RACP_OPERATOR_NULL               0x00
#define GLC_RACP_OPERATOR_ALL_RECS           0x01
#define GLC_RACP_OPERATOR_LT_EQ              0x02
#define GLC_RACP_OPERATOR_GT_EQ              0x03
#define GLC_RACP_OPERATOR_RANGE              0x04
#define GLC_RACP_OPERATOR_FIRST              0x05
#define GLC_RACP_OPERATOR_LAST               0x06

/* Filter Type values: */
#define GLC_RACP_FILTER_TYPE_RESERVED        0x00
#define GLC_RACP_FILTER_TYPE_SEQ_NBR         0x01
#define GLC_RACP_FILTER_TYPE_TIME            0x02

/* response codes used with GLC_RACP_OPCODE_RESP_CODE: */
#define GLC_RACP_RESP_RESERVED                    0x00
#define GLC_RACP_RESP_SUCCESS                     0x01
#define GLC_RACP_RESP_OPCODE_NOT_SUPPORTED        0x02
#define GLC_RACP_RESP_INVALID_OPERATOR            0x03
#define GLC_RACP_RESP_OPERATOR_NOT_SUPPORTED      0x04
#define GLC_RACP_RESP_INVALID_OPERAND             0x05
#define GLC_RACP_RESP_NO_RECS_FOUND               0x06
#define GLC_RACP_RESP_ABORT_UNSUCCESSFUL          0x07
#define GLC_RACP_RESP_PROC_NOT_COMPLETED          0x08
#define GLC_RACP_RESP_OPERAND_NOT_SUPPORTED       0x09

#define GLC_RACP_RESP_INTERNAL                    0x80  /* private code */


/*---- local RACP definitions and control data (IOP hack ...) ----*/
#define GLC_RACP_NBR_OF_STORED_RECS_DEFAULT     3
#define GLC_RACP_INIT_SEQ_NBR_DEFAULT           0

#define GLC_RACP_OPERATION_ACTIVE(x)                     \
    ((x >= GLC_RACP_OPCODE_REPORT_RECS) &&  \
     (x <= GLC_RACP_OPCODE_RESP_CODE))

#define GLC_RACP_TRANSACTION_TIMEOUT_VALUE    35000      /* value in msecs */
#define GLC_RACP_TRANSACTION_TIMER_ID         0

#define GLC_MAX_RACP_REQS   4

typedef struct _GLCRACP
{

    /*---- database fake for IOP .. ----*/
    int                iReqNum;     /* index into arrays (number of RACP request) */
    int                iReqNumMax;

    uint16_t           wNbrOfRecs[GLC_MAX_RACP_REQS];  /* nbr. of stored records (fake ..) */
    uint16_t           wSeqNbr[GLC_MAX_RACP_REQS];     /* current sequence nbr.            */
    int                iSeqNbrDelta[GLC_MAX_RACP_REQS];
    int                iSpecialBehavior[GLC_MAX_RACP_REQS];  /* <>0 : special behavior for some test cases */

    /*--------*/

    uint16_t           wHandle;     /* handle of RACP attribute value   */
    int                iLength;     /* current attribute value length   */
    TGLCControlPoint   Value;
} TGLCRACP, * PGLCRACP;


/*----- profile specific error codes ------*/
#define GLC_ERR_PROC_ALREADY_IN_PROGRESS    0x80
#define GLC_ERR_CCCD_IMPROPERLY_CONFIGURED  0x81


/*----- for GLS the Bluetooth stack is instructed to inform       -----*/
/*----- about (changes of) the GLS CCCD values (needed for qualification -----*/
/*----- test case TP/SPE/BI-08-C ..), see CCCD attributes in array       -----*/
/*----- gattdProfile[] with flag bit ATTRIB_FLAG_CCCD_APPL set.          -----*/
#define GATTDEMO_CCCD_COUNT      3

/* the following definition MUST be adapted if attributes in */
/* array gattdProfile[] are added or removed !!!!!!!:        */
#define GATTDEMO_CCCD_IDX  \
    {                          \
        (GATT_SVC_GLS_MEASUREMENT_INDEX+1),      /* index of Glucose Measurement CCCD */         \
        (GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX+1), /* index of Glucose Measurement Context CCCD */ \
        (GATT_SVC_GLS_RACP_INDEX+1),             /* index of Record Access Control Point CCCD */ \
    }




/* DIS/BAS/CTS .. attribute indices signaled to GATT server application. */
/* must be adapted if service definition array gattdProfile[] or flags   */
/* GLC_SRV_INCLUDE_BAS/CTS are modified !!!!                             */
#define GATT_SVC_DIS_SYSTEM_ID_INDEX          2   /* System ID value         */
#define GATT_SVC_DIS_MANUFACTURER_NAME_INDEX  4   /* Manufacturer Name value */
#define GATT_SVC_DIS_MODEL_NUMBER_INDEX       6   /* Model Number value      */

#define GATT_SVC_BAS_BATTERY_LEVEL_INDEX     21   /* Battery Level value     */

#define GATT_SVC_CTS_CURRENT_TIME_INDEX      25   /* Current Time value      */
#define GATT_SVC_CTS_LOCAL_TIME_INFO_INDEX   28   /* Local Time Information value     */
#define GATT_SVC_CTS_REF_TIME_INFO_INDEX     30   /* Reference Time Information value */

#define GATT_SVC_NDCS_TIME_WITH_DST_INDEX    33  /* Time with DST value */

#define GATT_SVC_RTUS_CONTROL_POINT_INDEX    36  /* Time UpdateControl Point value */
#define GATT_SVC_RTUS_STATE_INDEX            38  /* Time Update State value */

#define GATT_SVC_TEST_B001_FOR_READ                     41
#define GATT_SVC_TEST_B002_FOR_WRITE                   43
#define GATT_SVC_TEST_B002_FOR_WRITE_NO_RESP   			45
#define GATT_SVC_TEST_B003_FOR_NOTIFY                  47
#define GATT_SVC_TEST_B004_FOR_INDICATE              50

/* index ranges in service definition array gattdProfile[] needed to */
/* determine the service depending on the index value:               */
#define GATTDEMO_MAX_SVC_INDEX_DIS    6
#define GATTDEMO_MAX_SVC_INDEX_GLS   18
#define GATTDEMO_MAX_SVC_INDEX_BAS   22
#define GATTDEMO_MAX_SVC_INDEX_CTS   30
#define GATTDEMO_MAX_SVC_INDEX_NDCS  33
#define GATTDEMO_MAX_SVC_INDEX_RTUS  38



/*--- prototypes of GLC specific routines ---*/


#endif  /* __GATTDGLC_H */

