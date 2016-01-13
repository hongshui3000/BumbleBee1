
#ifndef __GULCOSE_UUID_H
#define __GULCOSE_UUID_H






/*--- services ---*/
/* DIS Device Information Service */
#define GATT_UUID_DEVICE_INFORMATION_SERVICE   0x180A

#define GATT_UUID_CHAR_SYSTEM_ID               0x2A23
#define GATT_UUID_CHAR_MODEL_NUMBER            0x2A24
#define GATT_UUID_CHAR_SERIAL_NUMBER           0x2A25
#define GATT_UUID_CHAR_FIRMWARE_REVISION       0x2A26
#define GATT_UUID_CHAR_HARDWARE_REVISION       0x2A27
#define GATT_UUID_CHAR_SOFTWARE_REVISION       0x2A28
#define GATT_UUID_CHAR_MANUFACTURER_NAME       0x2A29
#define GATT_UUID_CHAR_IEEE_CERTIF_DATA_LIST   0x2A2A
#define GATT_UUID_CHAR_PNP_ID                  0x2A50

/* BLS Blood Pressure Monitor Service */
#define GATT_UUID_BLOOD_PRESSURE               0x1810

#define GATT_UUID_CHAR_BLS_MEASUREMENT         0x2A35
#define GATT_UUID_CHAR_BLS_INT_CUFF_PRESSURE   0x2A36
#define GATT_UUID_CHAR_BLS_FEATURE             0x2A49

/* HRS Heart Rate Sensor Service */
#define GATT_UUID_HEART_RATE                   0x180D

#define GATT_UUID_CHAR_HRS_MEASUREMENT         0x2A37
#define GATT_UUID_CHAR_HRS_CONTROL_POINT       0x2A39

/* GLC Glucose Service */
#define GATT_UUID_GLUCOSE                      0x1808
#define GATT_UUID_HID 0x1812

#define GATT_UUID_CHAR_GLS_MEASUREMENT         0x2A18
#define GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT    0x2A34
#define GATT_UUID_CHAR_GLS_FEATURES            0x2A51
#define GATT_UUID_CHAR_GLS_RACP                0x2A52

/* BAS Battery Service */
#define GATT_UUID_BATTERY                      0x180F

#define GATT_UUID_CHAR_BAS_LEVEL               0x2A19

/* CTS Current Time Service */
#define GATT_UUID_CURRENT_TIME                 0x1805

#define GATT_UUID_CHAR_CTS_CURRENT_TIME        0x2A2B
#define GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO     0x2A0F
#define GATT_UUID_CHAR_CTS_REF_TIME_INFO       0x2A14

/* NDCS Next DST Change Service */
#define GATT_UUID_NEXT_DST_CHANGE              0x1807

#define GATT_UUID_CHAR_NDCS_TIME_WITH_DST      0x2A11

/* RTUS Reference Time Update Service */
#define GATT_UUID_REFERENCE_TIME_UPDATE        0x1806

#define GATT_UUID_CHAR_RTUS_CONTROL_POINT      0x2A16
#define GATT_UUID_CHAR_RTUS_STATE              0x2A17


#define GATT_UUID_CHAR_BODY_SENSOR_LOCATION    0x2A38


#define blueAPI_ServiceDIS          1                    /* Device Information service    */
#define blueAPI_ServiceGLS         2                     /* Glucose service               */
#define blueAPI_ServiceBAS         3             /* Battery service               */
#define blueAPI_ServiceCTS         4                     /* Current Time service          */
#define blueAPI_ServiceNDCS      5               /* Next DST Change service       */
#define blueAPI_ServiceRTUS      6              /* Reference Time Update service */
#define blueAPI_ServiceBLS          7            /* Blood Pressure service        */
#define blueAPI_ServiceHRS         8            /* Heart Rate service            */

/* the following services are not (yet) available as built-in services:   */
#define blueAPI_ServiceCGM             9         /* Continous Glucose Monitor service */
#define blueAPI_ServiceBMS              10        /* Bond Management service       */
#define blueAPI_ServiceQUL              11       /* Qualification test service    */
#define blueAPI_ServiceTST                   12      /* Test service                  */
#define blueAPI_ServiceHIDS             13
#define blueAPI_ServiceSPS                14
#define blueAPI_ServiceFMP              15
#define blueAPI_ServicePXP          16
#define blueAPI_ServiceHRP          17
#define blueAPI_ServiceCSC      18
#define blueAPI_ServiceRSC          19
#endif /* __GATT_H */

