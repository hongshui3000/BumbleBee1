#include "gatttest.h"
#include "blueapi_types.h"
#include <rtl_types.h>
#include <string.h>
#include "test_cmd.h"


static uint8_t  bDefaultWriteValue;

extern PGATTTest g_pGattTest;
uint32_t gTestDataMax = 0;

/*----------------------------------------------------------------------------
 * service(s)
 * --------------------------------------------------------------------------*/

/* Device Information Service (DIS) data */
static const char cSystemID[]        = "SysIDxx";
static const char cManufacurerName[] = "Stollmann E+V";
static const char cModelNumber[]     = "Model Nbr 0.9";

uint8_t gTestDataReadOrUpdate[255] = {0};

static TGLCMeasurementValue  GLCMeasurementValue =
{
    /* flags        */
    0
#if (GLC_INCLUDE_TIME_OFFSET)
    | GLC_FLAGS_TIME_OFFSET_PRESENT
#endif
#if (GLC_INCLUDE_CONC_TS_LOC)
    | GLC_FLAGS_CONC_TS_LOC_PRESENT
#if (GLC_FLAGS_UNITS_MMOL_L_ON)
    | GLC_FLAGS_CONC_UNITS_MMOL_L
#endif
#endif
#if (GLC_INCLUDE_SS_ANNUNC)
    | GLC_FLAGS_SS_ANNUNC_PRESENT
#endif

    /* sequence number */
    ,
    {
        0x0001
    }

    /* base time */
    ,
    {
        LO_WORD(2011), HI_WORD(2011),  /* YY:YY */
        10,        /* MM */
        6,         /* DD */
        11,        /* HH */
        26,        /* MM */
        15         /* SS */
    }
#if (GLC_INCLUDE_TIME_OFFSET)
    /* time offset */
    ,
    {
        0
    }
#endif

#if (GLC_INCLUDE_CONC_TS_LOC)
    /* concentration */
    ,
    {
#if (GLC_FLAGS_UNITS_MMOL_L_ON)
        /* in flags GLC_FLAGS_CONC_UNITS_MMOL_L  set: */
        0x50, 0xD0          /* SFLOAT 4 bit (MSB) exponent = 0xD = -3, 12 bit mantissa */
#else
        /* in flags GLC_FLAGS_CONC_UNITS_MMOL_L NOT set: */
        0x50, 0xB0          /* SFLOAT 4 bit (MSB) exponent = 0xB = -5, 12 bit mantissa */
#endif
    },
    /* type/sample location */
    0x15            /* XXXXMJMJ symbols  Finger / Arterial Wholeblood */
#endif

#if (GLC_INCLUDE_SS_ANNUNC)
    /* sensor status annunciation */
    ,
    {
        0x00, 0x00      /* no malfunction .. */
    }
#endif
};


/*-----------------------------------------------------*/

static TGLCMeasurementContext  GLCMeasurementContext =
{
    /* flags */
    0
#if (GLC_INCLUDE_CARBOHYDRATE)
    | GLC_FLAGS_CARBOHYDRATES_PRESENT
#endif
#if (GLC_INCLUDE_MEAL)
    | GLC_FLAGS_MEAL_PRESENT
#endif
#if (GLC_INCLUDE_TESTER_HEALTH)
    | GLC_FLAGS_TESTER_HEALTH_PRESENT
#endif
#if (GLC_INCLUDE_EXCERCISE)
    | GLC_FLAGS_EXCERCISE_PRESENT
#endif
#if (GLC_INCLUDE_MEDICATION)
    | GLC_FLAGS_MEDICATION_PRESENT
#if (GLC_FLAGS_MEDICATION_UNITS_MLITERS_ON)
    | GLC_FLAGS_MEDICATION_UNITS_MLITERS
#endif
#endif
#if (GLC_INCLUDE_HbA1c)
    | GLC_FLAGS_HbA1c_PRESENT
#endif
#if (GLC_INCLUDE_EXT_FLAGS)
    | GLC_FLAGS_EXT_FLAGS_PRESENT
#endif

    /* sequence number */
    ,
    {
        0x0001
    }

#if (GLC_INCLUDE_EXT_FLAGS)
    /* extended flag */
    ,
    0
#endif

#if (GLC_INCLUDE_CARBOHYDRATE)
    /* Carbohydrates ID and field */
    ,
    0x01,               /* breakfast */
    {
        0x50, 0xD0        /* SFLOAT 4 bit (MSB) exponent = 0xD = -3, 12 bit mantissa */
    }
#endif

#if (GLC_INCLUDE_MEAL)
    /* Meal ID and field */
    ,
    0x04                /* Casual (snacks, drinks, etc.) */
#endif

#if (GLC_INCLUDE_TESTER_HEALTH)
    /* Tester-Health field */
    ,
    0x42                /* Under stress / Health Care Professional */
#endif

#if (GLC_INCLUDE_EXCERCISE)
    /* Exercise Duration and Intensity field */
    ,
    {         /* duration (seconds) */
        1800
    },
    50        /* intensity (percent) */
#endif

#if (GLC_INCLUDE_MEDICATION)
    /* Medication ID and field */
    ,
    0x01,               /* Rapid acting insulin */
    {
#if (GLC_FLAGS_MEDICATION_UNITS_MLITERS_ON)
        /* flags GLC_FLAGS_MEDICATION_UNITS_MLITERS set: */
        0x50, 0xD0        /* SFLOAT 4 bit (MSB) exponent = 0xD = -3, 12 bit mantissa */
#else
        /* flags GLC_FLAGS_MEDICATION_UNITS_MLITERS NOT set: */
        0x50, 0xA0        /* SFLOAT 4 bit (MSB) exponent = 0xA = -6, 12 bit mantissa */
#endif
    }
#endif

#if (GLC_INCLUDE_HbA1c)
    /* HbA1c field */
    ,
    {
        0x60, 0x00          /* SFLOAT 4 bit (MSB) exponent = 0x0 = 0, 12 bit mantissa */
    }
#endif
};

/* features value */
static const UINT16  Features = GLC_FEATURES;


/*---- values for additional services ----*/
#if (GLC_SRV_INCLUDE_BAS)
static uint8_t  bBatteryLevel = 50;    /* level in percent */
#endif /* (GLC_SRV_INCLUDE_BAS) */

#if (GLC_SRV_INCLUDE_CTS)
static TCurrentTime CurrentTime =
{
    { /* ExactTime256  */
        { /* DayDateTime */
            { /* DateTime  */
                LO_WORD(2012), HI_WORD(2012),  /* YY:YY */
                04,        /* MM */
                26,        /* DD */
                18,        /* HH */
                42,        /* MM */
                15         /* SS */
            },
            CTS_WEEKDAY_THURSDAY
        },
        0  /* Fractions256 */
    },
    CTS_ADJUST_REASON_MANUAL_TIME_UPDATE
};

static TLocalTimeInfo LocalTimeInfo =
{
    (int8_t) - 4,                   /* 0: UTC+0:00 / -4: UTC-1:00 / -8: UTC-2:00 */
    CTS_DST_OFFSET_ONE_HOUR
};

static TReferenceTimeInfo ReferenceTimeInfo =
{
    CTS_TIME_SOURCE_MANUAL,
    CTS_TIME_ACCURACY_UNKNOWN,
    5,                             /* DaysSinceUpdate  */
    7                              /* HoursSinceUpdate */
};
#endif /* (GLC_SRV_INCLUDE_CTS) */

#if (GLC_SRV_INCLUDE_NDCS)
static TTimeWithDST TimeWithDST =
{
    { /* DateTime  */
        LO_WORD(2013), HI_WORD(2013),  /* YY:YY */
        01,        /* MM */
        22,        /* DD */
        19,        /* HH */
        13,        /* MM */
        25         /* SS */
    },
    DST_OFFSET_STANDARD
};
#endif /* (GLC_SRV_INCLUDE_NDCS) */


#if (GLC_SRV_INCLUDE_RTUS)
static TTimeUpdateState TimeUpdateState =
{
    RTUS_CURRENT_STATE_IDLE,
    RTUS_RESULT_SUCCESS
};
#endif /* (GLC_SRV_INCLUDE_RTUS) */

#if (GLC_SRV_INCLUDE_TEST)
const uint8_t test_0[60] = {0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02,
                            0x03,0x03,0x03,0x03,0x03,0x04,0x04,0x04,0x04,0x04,
                            0x05,0x05,0x05,0x05,0x05,0x06,0x06,0x06,0x06,0x06,
                            0x07,0x07,0x07,0x07,0x07,0x08,0x08,0x08,0x08,0x08,
                            0x09,0x09,0x09,0x09,0x09,0x00,0x00,0x00,0x00,0x00,
                            0x01,0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x02};
static uint8_t test_1[60] = {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	                         0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	                         0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	                         0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	                         0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	                         0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01};



#endif



/*-------------------------- profile/service definition ----------------------*/
const TAttribAppl test_upperstack_Profile[] =
{
    /*----------------- Device Information Service -------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE | ATTRIB_FLAG_BR_EDR),   /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_DEVICE_INFORMATION_SERVICE),  /* service UUID */
            HI_WORD(GATT_UUID_DEVICE_INFORMATION_SERVICE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* System ID String characteristic value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_SYSTEM_ID),
            HI_WORD(GATT_UUID_CHAR_SYSTEM_ID)
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Manufacturer Name String characteristic value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_MANUFACTURER_NAME),
            HI_WORD(GATT_UUID_CHAR_MANUFACTURER_NAME)
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Model Number characteristic value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_MODEL_NUMBER),
            HI_WORD(GATT_UUID_CHAR_MODEL_NUMBER)
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_READ                              /* wPermissions */
    },


    /*-------------------------- Glucose Service ---------------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_GLUCOSE),               /* service UUID */
            HI_WORD(GATT_UUID_GLUCOSE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,                  /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /*--- Glucose Measurement characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT),
            HI_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT)
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_NONE                              /* wPermissions */
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* wFlags */
        ATTRIB_FLAG_CCCD_APPL),
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,                  /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /*--- Glucose Measurement Context characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT),
            HI_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT)
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_NONE                              /* wPermissions */
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* wFlags */
        ATTRIB_FLAG_CCCD_APPL),
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /*--- Glucose Features characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_GLS_FEATURES),
            HI_WORD(GATT_UUID_CHAR_GLS_FEATURES),
        },
        2,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }

    ,
    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE |                   /* characteristic properties */
            GATT_CHAR_PROP_INDICATE)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /*--- Glucose Record Access Control Point value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_GLS_RACP),
            HI_WORD(GATT_UUID_CHAR_GLS_RACP)
        },
        0,                                          /* bValueLen, 0 : variable length */
        NULL,
        GATT_PERM_WRITE_AUTHEN_REQ                  /* wPermissions */
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* wFlags */
        ATTRIB_FLAG_CCCD_APPL),
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    }

#if (GLC_SRV_INCLUDE_BAS)
    ,
    /*----------------- Battery Service -------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE | ATTRIB_FLAG_BR_EDR),   /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_BATTERY),              /* service UUID */
            HI_WORD(GATT_UUID_BATTERY)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_READ |                    /* characteristic properties */
            GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Battery Level value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_BAS_LEVEL),
            HI_WORD(GATT_UUID_CHAR_BAS_LEVEL)
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* client characteristic configuration */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    }
#endif /* (GLC_SRV_INCLUDE_BAS) */

#if (GLC_SRV_INCLUDE_CTS)
    ,
    /*----------------- Current Time Service -------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_CURRENT_TIME),          /* service UUID */
            HI_WORD(GATT_UUID_CURRENT_TIME)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_READ |                    /* characteristic properties */
            GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Current Time value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CTS_CURRENT_TIME),
            HI_WORD(GATT_UUID_CHAR_CTS_CURRENT_TIME)
        },
        10,                                         /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* client characteristic configuration */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Local Time Information value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO),
            HI_WORD(GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO)
        },
        2,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Reference Time Information value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CTS_REF_TIME_INFO),
            HI_WORD(GATT_UUID_CHAR_CTS_REF_TIME_INFO)
        },
        4,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }
#endif /* (GLC_SRV_INCLUDE_CTS) */

#if (GLC_SRV_INCLUDE_NDCS)
    ,
    /*----------------- Next DST Change Service -------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_NEXT_DST_CHANGE),       /* service UUID */
            HI_WORD(GATT_UUID_NEXT_DST_CHANGE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Time with DST value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_NDCS_TIME_WITH_DST),
            HI_WORD(GATT_UUID_CHAR_NDCS_TIME_WITH_DST)
        },
        8,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }
#endif /* (GLC_SRV_INCLUDE_NDCS) */

#if (GLC_SRV_INCLUDE_RTUS)
    ,
    /*-------------- Reference Time Update Service ----------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(GATT_UUID_REFERENCE_TIME_UPDATE), /* service UUID */
            HI_WORD(GATT_UUID_REFERENCE_TIME_UPDATE)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP               /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Time Update Control Point value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_RTUS_CONTROL_POINT),
            HI_WORD(GATT_UUID_CHAR_RTUS_CONTROL_POINT)
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_WRITE                             /* wPermissions */
    },

    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* Time Update State value */
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_RTUS_STATE),
            HI_WORD(GATT_UUID_CHAR_RTUS_STATE)
        },
        2,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    }
#endif /* (GLC_SRV_INCLUDE_RTUS) */
#if (GLC_SRV_INCLUDE_TEST)//INDEX start from 39
  	,
  	/*-------------- Reference Time Update Service ----------------*/
  	/*----------------- handle = 0x002f-------------------*/

  	/* <<Primary Service>>, .. */
  	{
    	(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
    	{                                           /* bTypeValue */
      		LO_WORD(GATT_UUID_PRIMARY_SERVICE),
      		HI_WORD(GATT_UUID_PRIMARY_SERVICE),
      		LO_WORD(0xA00A), /* service UUID */
      		HI_WORD(0xA00A)
    	},
    	UUID_16BIT_SIZE,                            /* bValueLen     */
    	NULL,                                       /* pValueContext */
    	GATT_PERM_READ                              /* wPermissions  */
  	},
  	/*----------------- handle = 0x0030-------------------*/
  	/* <<Characteristic>> for read test */
  	{
    	ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
    	{                                           /* bTypeValue */
      		LO_WORD(GATT_UUID_CHARACTERISTIC),
      		HI_WORD(GATT_UUID_CHARACTERISTIC),
      		GATT_CHAR_PROP_READ  /* characteristic properties */
      		/* characteristic UUID not needed here, is UUID of next attrib. */
    	},
    	1,                                          /* bValueLen */
    	NULL,
    	GATT_PERM_READ                              /* wPermissions */
  	},
  	/*----------------- handle = 0x0031-------------------*/
  	/*-----------------index = 41-------------------*/
  	{
    	ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
    	{                                           /* bTypeValue */
      		LO_WORD(0xB001),
      		HI_WORD(0xB001)
    	},
    	0,                                          /* bValueLen */
    	NULL,
    	GATT_PERM_READ         /* wPermissions */
  	},
  	/* <<Characteristic>> for write test */

  	/*----------------- handle = 0x0032-------------------*/
  	{
    	ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
    	{                                           /* bTypeValue */
      		LO_WORD(GATT_UUID_CHARACTERISTIC),
      		HI_WORD(GATT_UUID_CHARACTERISTIC),
      		(GATT_CHAR_PROP_WRITE)  /* characteristic properties */
      		/* characteristic UUID not needed here, is UUID of next attrib. */
    	},
    	0,                                          /* bValueLen */
    	NULL,
    	GATT_PERM_READ                              /* wPermissions */
  	},
  	/*----------------- handle = 0x0033-------------------*/
  	/*----------------- index = 43-------------------*/
  	{
    	ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
    	{                                           /* bTypeValue */
      		LO_WORD(0xB002),
      		HI_WORD(0xB002),
      		0x02,
      		0x02
    	},
    	0,                                          /* bValueLen */
    	NULL,
    	(GATT_PERM_WRITE)          /* wPermissions */
  	},
  	/*----------------- handle = 0x0034-------------------*/

  /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP/* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
  	/*----------------- handle = 0x0035-------------------*/
  	/*----------------- index = 45-------------------*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB003),
            HI_WORD(0xB003),
        },
        0,                                          /* bValueLen */
        NULL,
        GATT_PERM_WRITE                 /* wPermissions */
    },            
  	/*----------------- handle = 0x0036-------------------*/
    /* <<Characteristic>>, for notify test */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,                  /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
  	/*----------------- handle = 0x0037-------------------*/
  	/*----------------- index = 47-------------------*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB004),
            HI_WORD(0xB004)
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_NONE                              /* wPermissions */
    },
    /*----------------- handle = 0x0038-------------------*/
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* wFlags */
        ATTRIB_FLAG_CCCD_APPL),
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },
  	/*----------------- handle = 0x0039-------------------*/
    /* <<Characteristic>>, for indication test */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_INDICATE,                    /* characteristic properties */
            //XXXXMJMJ GATT_CHAR_PROP_INDICATE,                  /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
  	/*----------------- handle = 0x003a-------------------*/
  	/*----------------- index = 50-------------------*/
    {
        ATTRIB_FLAG_VALUE_APPL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB005),
            LO_WORD(0xB005),
        },
        0,                                          /* variable size */
        (void *)NULL,
        GATT_PERM_NONE                              /* wPermissions */
    },
    /*----------------- handle = 0x003b-------------------*/
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* wFlags */
        ATTRIB_FLAG_CCCD_APPL),
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* wPermissions */
    },
    /*----------------- handle = 0x003c-------------------*/
    {
    	ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
    	{                                           /* bTypeValue */
      		LO_WORD(GATT_UUID_CHARACTERISTIC),
      		HI_WORD(GATT_UUID_CHARACTERISTIC),
      		GATT_CHAR_PROP_WRITE|GATT_CHAR_PROP_READ  /* characteristic properties */
      		/* characteristic UUID not needed here, is UUID of next attrib. */
    	},
    	1,                                          /* bValueLen */
    	NULL,
    	GATT_PERM_READ                              /* wPermissions */
  	},
  /*----------------- handle = 0x003d-------------------*/
  	{
    	ATTRIB_FLAG_VOID,                     /* wFlags */
    	{                                           /* bTypeValue */
      		LO_WORD(0xB006),
      		HI_WORD(0xB006)
    	},
    	60,                                          /* bValueLen */
    	(void*)test_1,
    	GATT_PERM_WRITE|GATT_PERM_READ         /* wPermissions */
  	}      
            
  	
#endif /* (GLC_SRV_INCLUDE_TEST) */
};

const int iGattTestProfileSize = sizeof(test_upperstack_Profile);








/*----------------------------------------------------------------------------
 * save some relevant attribute handle/UUID pairs in local array
 * --------------------------------------------------------------------------*/

void test_upperstack_HandleUUIDSave( PGATTTest pGattTest, uint16_t idx, uint16_t wHandle, uint16_t wUUID16 )
{
    int   i;
    /* check if pair is already stored */

    for ( i = 0; i < pGattTest->linkTable[idx].iHandleCnt; i++ )
    {
        if ( pGattTest->linkTable[idx].HandleUUID[i].wHandle == wHandle )
            return;
    }

    switch ( wUUID16 )
    {
    default:
        break;

    /* GAP device name */
    case GATT_UUID_CHAR_DEVICE_NAME:

    /* GATT <<ServiceChanged>> */
    case GATT_UUID_CHAR_SERVICE_CHANGED:

    /* DIS characteristics: */
    case GATT_UUID_CHAR_SYSTEM_ID:
    case GATT_UUID_CHAR_MODEL_NUMBER:
    case GATT_UUID_CHAR_MANUFACTURER_NAME:

    /* GLC characteristics: */
    case GATT_UUID_CHAR_GLS_MEASUREMENT:
    case GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT:
    case GATT_UUID_CHAR_GLS_FEATURES:
    case GATT_UUID_CHAR_GLS_RACP:

        /*---- values for additional services ----*/
#if (GLC_SRV_INCLUDE_BAS)
    /* BAS characteristics: */
    case GATT_UUID_CHAR_BAS_LEVEL:
#endif /* (GLC_SRV_INCLUDE_BAS) */

#if (GLC_SRV_INCLUDE_CTS)
    /* CTS characteristics: */
    case GATT_UUID_CHAR_CTS_CURRENT_TIME:
    case GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO:
    case GATT_UUID_CHAR_CTS_REF_TIME_INFO:
#endif /* (GLC_SRV_INCLUDE_CTS) */

#if (GLC_SRV_INCLUDE_NDCS)
    /* NDCS characteristics: */
    case GATT_UUID_CHAR_NDCS_TIME_WITH_DST:
#endif /* (GLC_SRV_INCLUDE_NDCS) */

#if (GLC_SRV_INCLUDE_RTUS)
    /* RTUS characteristics: */
    case GATT_UUID_CHAR_RTUS_CONTROL_POINT:
    case GATT_UUID_CHAR_RTUS_STATE:
#endif /* (GLC_SRV_INCLUDE_RTUS) */

#if (GLC_SRV_INCLUDE_TEST)
    case 0xB001:
    case 0xB002:
    case 0xB003:
    case 0xB004:
    case 0xB005:
#endif


    /* client characteristic configuration: */
    case GATT_UUID_CHAR_CLIENT_CONFIG:
        if ( pGattTest->linkTable[idx].iHandleCnt < GATTDEMO_MAX_HANDLE_UUID )
        {
            pGattTest->linkTable[idx].HandleUUID[pGattTest->linkTable[idx].iHandleCnt].wHandle = wHandle;
            pGattTest->linkTable[idx].HandleUUID[pGattTest->linkTable[idx].iHandleCnt].wUUID   = wUUID16;
            pGattTest->linkTable[idx].iHandleCnt++;
        }
        else
        {
            test_upperstack_CmdPrint( pGattTest,
                                      "!! GATTDEMO_MAX_HANDLE_UUID=%d is too small !!\r\n");
        }

        if ( wUUID16 == GATT_UUID_CHAR_GLS_RACP )
        {
            pGattTest->RACP.wHandle = wHandle;
        }
        break;
    }
}

/*----------------------------------------------------------------------------
 * display Glucose Measurement value
 * --------------------------------------------------------------------------*/

static void  test_upperstack_AttribDisplayGLCValue( PGATTTest pGattTest, uint8_t * pValue )
{
    uint8_t               bFlags;
    int                   iOffset;
    PGLCMeasurementValue  pGLCMValue = (PGLCMeasurementValue)pValue;

    /* Sequence Number field + Flags field */
    bFlags  = pGLCMValue->bFlags;
    iOffset = sizeof(pGLCMValue->bFlags);
    test_upperstack_CmdPrint( pGattTest,
                              " : Measurement / SeqNbr=0x%x, Flags=0x%x (Context Info=%d)\r\n",
                              LE_EXTRN2WORD( pValue + iOffset ),
                              bFlags, (bFlags & GLC_FLAGS_CTXT_INFORM_PRESENT) ? 1 : 0
                              );
    iOffset += sizeof(UINT16);

    if ( pGattTest->ShortOutput )
        return;

    /* Base Time field */
    test_upperstack_PrintTimestamp( pGattTest, pValue + iOffset, "Time" );
    iOffset += sizeof(TIMESTAMP);

    /* optional Time Offset field */
    if ( bFlags & GLC_FLAGS_TIME_OFFSET_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Time Offset = %d\r\n ",
                                  LE_EXTRN2WORD( pValue + iOffset )
                                  );
        iOffset += sizeof(signed short);
    }

    /* optional Concentration and Type/Sample Location field */
    if ( bFlags & GLC_FLAGS_CONC_TS_LOC_PRESENT )
    {
        uint16_t wConcentration = LE_EXTRN2WORD( pValue + iOffset );

        test_upperstack_PrintSFLOAT( pGattTest, "  Concentration ", wConcentration );
        iOffset += sizeof(SFLOAT);

        test_upperstack_CmdPrint( pGattTest, "  Type/Sample Location = 0x%x\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(UINT8);
    }

    /* optional Sensor Status Annunciation Value field */
    if ( bFlags & GLC_FLAGS_SS_ANNUNC_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Sensor Status Annunciation = 0x%x\r\n",
                                  LE_EXTRN2WORD( pValue + iOffset )
                                  );
        /*iOffset += 2*sizeof(uint8_t);*/
    }
}

/*----------------------------------------------------------------------------
 * display Glucose Measurement Context
 * --------------------------------------------------------------------------*/

static void  test_upperstack_AttribDisplayGLCContext( PGATTTest pGattTest, uint8_t * pValue )
{
    uint8_t                 bFlags;
    int                     iOffset;
    PGLCMeasurementContext  pGLCMContext = (PGLCMeasurementContext)pValue;

    /* Sequence Number field + Flags field */
    bFlags  = pGLCMContext->bFlags;
    iOffset = sizeof(pGLCMContext->bFlags);
    test_upperstack_CmdPrint( pGattTest,
                              " : Context / SeqNbr=0x%x, Flags=0x%x\r\n",
                              LE_EXTRN2WORD( pValue + iOffset ), bFlags
                              );
    iOffset += sizeof(UINT16);

    if ( pGattTest->ShortOutput )
        return;

    /* optional Extension Flags field */
    if ( bFlags & GLC_FLAGS_EXT_FLAGS_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  ExtFlags=0x%x\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(uint8_t);
    }

    /* optional Carbohydrates ID and field */
    if ( bFlags & GLC_FLAGS_CARBOHYDRATES_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Carbohydrate ID=0x%x\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(UINT8);

        test_upperstack_PrintSFLOAT( pGattTest, "  Carbohydrate  ", LE_EXTRN2WORD( pValue + iOffset ) );
        iOffset += sizeof(SFLOAT);
    }

    /* optional Meal field */
    if ( bFlags & GLC_FLAGS_MEAL_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Meal=0x%x\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(UINT8);
    }

    /* optional Tester-Health field */
    if ( bFlags & GLC_FLAGS_TESTER_HEALTH_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Tester Health=0x%x\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(UINT8);
    }

    /* optional Exercise Duration and Intensity field */
    if ( bFlags & GLC_FLAGS_EXCERCISE_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Exercise Duration =0x%x\r\n",
                                  LE_EXTRN2WORD( pValue + iOffset )
                                  );
        iOffset += sizeof(UINT16);

        test_upperstack_CmdPrint( pGattTest, "  Exercise Intensity=%d\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(UINT8);
    }

    /* optional Medication ID and field */
    if ( bFlags & GLC_FLAGS_MEDICATION_PRESENT )
    {
        test_upperstack_CmdPrint( pGattTest, "  Medication ID=0x%x\r\n",
                                  *(pValue + iOffset)
                                  );
        iOffset += sizeof(UINT8);

        test_upperstack_PrintSFLOAT( pGattTest, "  Medication  ", LE_EXTRN2WORD( pValue + iOffset ) );
        iOffset += sizeof(SFLOAT);
    }

    /* optional HbA1c field */
    if ( bFlags & GLC_FLAGS_HbA1c_PRESENT )
    {
        test_upperstack_PrintSFLOAT( pGattTest, "  HbA1c ", LE_EXTRN2WORD( pValue + iOffset ) );
        /* iOffset += sizeof(SFLOAT); */
    }
}

/*----------------------------------------------------------------------------
 * display attribute value
 * --------------------------------------------------------------------------*/

void  test_upperstack_AttribDisplay( PGATTTest pGattTest, uint16_t idx, uint16_t wHandle,
                                     int iSize, uint8_t * pValue )
{
    uint16_t  wUUID16;

    test_upperstack_CmdPrint( pGattTest, " handle=0x%x, size=%d", wHandle, iSize );

    wUUID16 = test_upperstack_UUIDGet( pGattTest, idx, wHandle );
    switch ( wUUID16 )
    {
    default:
        test_upperstack_HexDump( pGattTest, " : value=", iSize, pValue );
        break;

    case GATT_UUID_CHAR_CLIENT_CONFIG:
        test_upperstack_CmdPrint( pGattTest, " : ClientCharConfig=%04x\r\n",
                                  LE_EXTRN2WORD(pValue)
                                  );
        break;

    case GATT_UUID_CHAR_GLS_MEASUREMENT:
        test_upperstack_AttribDisplayGLCValue( pGattTest, pValue );
        break;

    case GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT:
        test_upperstack_AttribDisplayGLCContext( pGattTest, pValue );
        break;

    case GATT_UUID_CHAR_GLS_FEATURES:
        test_upperstack_CmdPrint( pGattTest, " : Features = 0x%x\r\n",
                                  LE_EXTRN2WORD(pValue)
                                  );
        break;

    case GATT_UUID_CHAR_GLS_RACP:
        test_upperstack_GLC_RACPHandleResp( pGattTest, (PGLCControlPoint)pValue );
        break;

    case GATT_UUID_CHAR_SERVICE_CHANGED:
        test_upperstack_CmdPrint( pGattTest, " : Service changed, range: 0x%04x-0x%04x\r\n",
                                  LE_EXTRN2WORD(pValue), LE_EXTRN2WORD(pValue + sizeof(uint16_t))
                                  );
        break;


        /*---- values of additional services ----*/
#if (GLC_SRV_INCLUDE_BAS)
    case GATT_UUID_CHAR_BAS_LEVEL:
        test_upperstack_CmdPrint( pGattTest, " : Battery level: %d percent\r\n",
                                  *pValue
                                  );
        break;
#endif /* (GLC_SRV_INCLUDE_BAS) */

#if (GLC_SRV_INCLUDE_CTS)
    case GATT_UUID_CHAR_CTS_CURRENT_TIME:
        test_upperstack_PrintTimestamp( pGattTest,
                                        ((PCurrentTime)pValue)->ExactTime256.DayDateTime.DateTime, "Time" );
        break;

    case GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO:
        {
            char cSign;
            signed char cTimeZone = (signed char)(((PLocalTimeInfo)pValue)->TimeZone);

            if ( cTimeZone < 0 )
            {
                /* print routine can't handle negative values .. */
                cSign     = '-';
                cTimeZone = -cTimeZone;
            }
            else
            {
                cSign = '+';
            }
            test_upperstack_CmdPrint( pGattTest, " : Local Time Info: Zone=%c%d, DSTOffset=%d\r\n",
                                      cSign, cTimeZone,
                                      ((PLocalTimeInfo)pValue)->DSTOffset
                                      );
        }
        break;

    case GATT_UUID_CHAR_CTS_REF_TIME_INFO:
        test_upperstack_CmdPrint( pGattTest, " : Reference Time Info: Source=%d\r\n",
                                  ((PReferenceTimeInfo)pValue)->Source
                                  );
        break;
#endif /* (GLC_SRV_INCLUDE_CTS) */

#if (GLC_SRV_INCLUDE_NDCS)
    case GATT_UUID_CHAR_NDCS_TIME_WITH_DST:
        test_upperstack_PrintTimestamp( pGattTest,
                                        ((PTimeWithDST)pValue)->DateTime, "Date Time" );
        test_upperstack_CmdPrint( pGattTest, "   DST Offset=0x%02x\r\n",
                                  ((PTimeWithDST)pValue)->DSTOffset
                                  );
        break;
#endif /* (GLC_SRV_INCLUDE_NDCS) */

#if (GLC_SRV_INCLUDE_RTUS)
    case GATT_UUID_CHAR_RTUS_STATE:
        test_upperstack_CmdPrint( pGattTest, "   Current State=0x%02x, Result=0x%02x\r\n",
                                  ((PTimeUpdateState)pValue)->CurrentState,
                                  ((PTimeUpdateState)pValue)->Result
                                  );
        break;
#endif /* (GLC_SRV_INCLUDE_RTUS) */
    }
}

/*----------------------------------------------------------------------------
 * service/attribute specific write data setup.
 * the meaning of parameter "wParam" depends on "wHandle".
 * --------------------------------------------------------------------------*/



uint16_t  test_upperstack_AttribGetWriteData( PGATTTest pGattTest,
        TTestParseResult *pParseResult,
        uint16_t * pwLength, uint8_t **ppValue )
{
    uint16_t     wUUID16;
    int          idx = pParseResult->dwParameter[0];
    uint16_t     wHandle = pParseResult->dwParameter[1];
    uint16_t     wParam  = pParseResult->dwParameter[2];

    test_upperstack_CmdPrint( pGattTest, "write to handle=0x%x", wHandle );

    *pwLength = 0;
    *ppValue  = NULL;

    wUUID16 = test_upperstack_UUIDGet( pGattTest, idx, wHandle );

    switch ( wUUID16 )
    {
    default:
        /* assume uint8_t value */
        bDefaultWriteValue = (uint8_t)wParam;
        *ppValue = &bDefaultWriteValue;
        *pwLength = sizeof(uint8_t);
        break;

    case GATT_UUID_CHAR_CLIENT_CONFIG:
        LE_WORD2EXTRN( pGattTest->bCCCBits, wParam );
        *ppValue  = pGattTest->bCCCBits;
        *pwLength = sizeof(pGattTest->bCCCBits);
        break;

    case GATT_UUID_CHAR_GLS_RACP:
        if ( pGattTest->RACP.iLength > 0 )
        {
            *ppValue  = (uint8_t *)&pGattTest->RACP.Value;
            *pwLength = pGattTest->RACP.iLength;
        }
        else
        {
            return ( GATT_ERR_ILLEGAL_PARAMETER );
        }
        break;
     //for write   
     case 0xB002:
     {
            uint8_t bGattTestWriteData[255] = {0};
            uint8_t i = 0;
            for(i = 0; i < wParam; i++)
            {
                bGattTestWriteData[i] = i;
            }
            *ppValue  = (uint8_t *)bGattTestWriteData;
            *pwLength = wParam;
     }
     //for write without response
     case 0xB003:
    {
        uint8_t bGattTestWriteData[255] = {0};
        uint8_t i = 0;
        for(i = 0; i < wParam; i++)
        {
            bGattTestWriteData[i] = wParam - i;
        }
        *ppValue  = (uint8_t *)bGattTestWriteData;
        *pwLength = wParam;
     }
   
     break;

        
    }


    return ( GATT_SUCCESS );
}



/*----------------------------------------------------------------------------
 * get ID of next service that should be registered
 * --------------------------------------------------------------------------*/

TBlueAPI_GATTServiceID  gattTestGetNextServiceID( int iServiceCount )
{
    TBlueAPI_GATTServiceID   ServiceID;

    ServiceID = blueAPI_ServiceApplicationDefined;


    return ( ServiceID );
}


/*----------------------------------------------------------------------------
 * convert attribute index into service ID
 * --------------------------------------------------------------------------*/

static uint8_t  test_upperstack_Index2ServiceID( int iAttribIndex )
{
    uint8_t ServiceID = blueAPI_ServiceGLS;

    if ( iAttribIndex <= GATTDEMO_MAX_SVC_INDEX_DIS )
        ServiceID = blueAPI_ServiceDIS;
    else if ( iAttribIndex <= GATTDEMO_MAX_SVC_INDEX_GLS )
        ServiceID = blueAPI_ServiceGLS;
#if (GLC_SRV_INCLUDE_BAS)
    else if ( iAttribIndex <= GATTDEMO_MAX_SVC_INDEX_BAS )
        ServiceID = blueAPI_ServiceBAS;
#endif
#if (GLC_SRV_INCLUDE_CTS)
    else if ( iAttribIndex <= GATTDEMO_MAX_SVC_INDEX_CTS )
        ServiceID = blueAPI_ServiceCTS;
#endif
#if (GLC_SRV_INCLUDE_NDCS)
    else if ( iAttribIndex <= GATTDEMO_MAX_SVC_INDEX_NDCS )
        ServiceID = blueAPI_ServiceNDCS;
#endif
#if (GLC_SRV_INCLUDE_RTUS)
  	else if( iAttribIndex <= GATTDEMO_MAX_SVC_INDEX_RTUS )
    	ServiceID = blueAPI_ServiceRTUS;
#endif
#if (GLC_SRV_INCLUDE_TEST)
	else
	  	ServiceID = blueAPI_ServiceTST;
#endif

    return ( ServiceID );
}


/*----------------------------------------------------------------------------
 * get next sequence number
 * --------------------------------------------------------------------------*/

static uint16_t  test_upperstack_GetNextSeqNbr( PGATTTest pGattTest )
{
    pGattTest->RACP.wSeqNbr[pGattTest->RACP.iReqNum] += pGattTest->RACP.iSeqNbrDelta[pGattTest->RACP.iReqNum];
    return ( pGattTest->RACP.wSeqNbr[pGattTest->RACP.iReqNum] );
}

/*----------------------------------------------------------------------------
 * get DIS value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetDISValue( int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;

    switch ( iAttribIndex )
    {
    default:
        break;
    case GATT_SVC_DIS_SYSTEM_ID_INDEX:
        pValue    = (uint8_t *)cSystemID;
        *piLength = sizeof(cSystemID);
        break;
    case GATT_SVC_DIS_MANUFACTURER_NAME_INDEX:
        pValue    = (uint8_t *)cManufacurerName;
        *piLength = sizeof(cManufacurerName);
        break;
    case GATT_SVC_DIS_MODEL_NUMBER_INDEX:
        pValue    = (uint8_t *)cModelNumber;
        *piLength = sizeof(cModelNumber);
        break;
    }

    return ( pValue );
}

/*----------------------------------------------------------------------------
 * get/read attribute GATT_UUID_CHAR_GLS_MEASUREMENT value
 * --------------------------------------------------------------------------*/

#define INIT_MINUTE 8
#define END_MINUTE  22
static  uint8_t bMinute   = INIT_MINUTE;

static uint8_t *  test_upperstack_AttribGetGLC_MEASUREMENTValue( PGATTTest pGattTest )
{
    uint8_t * pValue;
    PGLCRACP  pRACP = &pGattTest->RACP;

    LE_WORD2EXTRN( GLCMeasurementValue.SeqNbr, pRACP->wSeqNbr[pRACP->iReqNum] );

    GLCMeasurementValue.BaseTime[5] = bMinute;   /* MM */
    bMinute++;
    if ( bMinute > END_MINUTE )
        bMinute = INIT_MINUTE;

#if (GLC_INCLUDE_TIME_OFFSET)
    if ( pRACP->iSpecialBehavior[pRACP->iReqNum] == 1 )
    {
        /* 4.13.1 TP/SPT/BV-01-C [Service Procedure - Time Update] */
        if ( pRACP->wSeqNbr[pRACP->iReqNum] >= 3 )
        {
            if ( pRACP->wSeqNbr[pRACP->iReqNum] >= 6 )
            {
                union
                {
                    signed short si;
                    uint16_t     w;
                } u;

                u.si = -60;
                LE_WORD2EXTRN( GLCMeasurementValue.TimeOffset, u.w );
            }
            else
            {
                LE_WORD2EXTRN( GLCMeasurementValue.TimeOffset, 60 );
            }
        }
        else
        {
            LE_WORD2EXTRN( GLCMeasurementValue.TimeOffset, 0 );
        }
    }
#endif

#if (GLC_INCLUDE_CONC_TS_LOC)
    GLCMeasurementValue.Concentration[0] = 0x55 + (pRACP->wSeqNbr[pRACP->iReqNum] % 4);
#endif

#if (GLC_INCLUDE_CTXT_INFORM)
    if ( (pRACP->wSeqNbr[pRACP->iReqNum] == 1) ||
            ((pRACP->iSpecialBehavior[pRACP->iReqNum] == 1) && ((pRACP->wSeqNbr[pRACP->iReqNum] % 3 == 1)))
       )
        GLCMeasurementValue.bFlags |= GLC_FLAGS_CTXT_INFORM_PRESENT;
    else
        GLCMeasurementValue.bFlags &= ~GLC_FLAGS_CTXT_INFORM_PRESENT;

    if ( GLCMeasurementValue.bFlags & GLC_FLAGS_CTXT_INFORM_PRESENT )
    {
        /* if update sequence is processed continue with GLC context, */
        /* don't count context value:                                 */
        if ( GLC_RACP_OPERATION_ACTIVE( pRACP->Value.OpCode ) )
        {
            pGattTest->iUpdateAttribIndex = GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX;
            pGattTest->iUpdateCnt++;
        }
    }
    else
    {
        /* otherwise incremented when GLC context is sent: */
        test_upperstack_GetNextSeqNbr( pGattTest );
    }
#else
    pRACP->wSeqNbr[pRACP->iReqNum] = test_upperstack_GetNextSeqNbr( pGattTest );
#endif

    pValue = (uint8_t *)&GLCMeasurementValue;

    return ( pValue );
}

/*----------------------------------------------------------------------------
 * get/read attribute GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetGLC_MEASUREMENTContext( PGATTTest pGattTest )
{
    uint8_t *  pValue;

    LE_WORD2EXTRN( GLCMeasurementContext.SeqNbr, pGattTest->RACP.wSeqNbr[pGattTest->RACP.iReqNum] );
    test_upperstack_GetNextSeqNbr( pGattTest );

    /* if update sequence is processed continue with GLC measurement and */
    /* don't count context value:                                        */
    if ( GLC_RACP_OPERATION_ACTIVE( pGattTest->RACP.Value.OpCode ) )
    {
        pGattTest->iUpdateAttribIndex = GATT_SVC_GLS_MEASUREMENT_INDEX;
    }

    pValue = (uint8_t *)&GLCMeasurementContext;

    return ( pValue );
}

/*----------------------------------------------------------------------------
 * get GLS value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetGLSValue( PGATTTest pGattTest,
        int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;

    *piLength = 0;
    switch ( iAttribIndex )
    {
    default:
        break;

    case GATT_SVC_GLS_FEATURE_INDEX:
        pValue    = (uint8_t *)Features;
        *piLength = sizeof(Features);
        break;

    case GATT_SVC_GLS_MEASUREMENT_INDEX:
        pValue    = test_upperstack_AttribGetGLC_MEASUREMENTValue( pGattTest );
        *piLength = sizeof(TGLCMeasurementValue);
        break;

    case GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX:
        pValue    = test_upperstack_AttribGetGLC_MEASUREMENTContext( pGattTest );
        *piLength = sizeof(GLCMeasurementContext);
        break;

    case GATT_SVC_GLS_RACP_INDEX:
        if ( pGattTest->RACP.iLength > 0 )
        {
            pValue    = (uint8_t *)&pGattTest->RACP.Value;
            *piLength = pGattTest->RACP.iLength;
        }
        break;
    }

    return ( pValue );
}

#if (GLC_SRV_INCLUDE_BAS)
/*----------------------------------------------------------------------------
 * get BAS value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetBASValue( int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;

    *piLength = 0;
    switch ( iAttribIndex )
    {
    default:
        break;

    case GATT_SVC_BAS_BATTERY_LEVEL_INDEX:
        pValue    = (uint8_t *)&bBatteryLevel;
        *piLength = sizeof(bBatteryLevel);
        if ( bBatteryLevel > 0 )
            bBatteryLevel--;          /* fake fading battery ... */
        break;
    }

    return ( pValue );
}
#endif /* (GLC_SRV_INCLUDE_BAS) */

#if (GLC_SRV_INCLUDE_CTS)
/*----------------------------------------------------------------------------
 * get CTS value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetCTSValue( int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;

    *piLength = 0;
    switch ( iAttribIndex )
    {
    default:
        break;

    case GATT_SVC_CTS_CURRENT_TIME_INDEX:
        pValue    = (uint8_t *)&CurrentTime;
        *piLength = sizeof(CurrentTime);
        break;

    case GATT_SVC_CTS_LOCAL_TIME_INFO_INDEX:
        pValue    = (uint8_t *)&LocalTimeInfo;
        *piLength = sizeof(LocalTimeInfo);
        break;

    case GATT_SVC_CTS_REF_TIME_INFO_INDEX:
        pValue    = (uint8_t *)&ReferenceTimeInfo;
        *piLength = sizeof(ReferenceTimeInfo);
        break;
    }

    return ( pValue );
}
#endif /* (GLC_SRV_INCLUDE_CTS) */

#if (GLC_SRV_INCLUDE_NDCS)
/*----------------------------------------------------------------------------
 * get NDCS value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetNDCSValue( int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;

    *piLength = 0;
    switch ( iAttribIndex )
    {
    default:
        break;

    case GATT_SVC_NDCS_TIME_WITH_DST_INDEX:
        pValue    = (uint8_t *)&TimeWithDST;
        *piLength = sizeof(TimeWithDST);
        break;
    }

    return ( pValue );
}
#endif /* (GLC_SRV_INCLUDE_NDCS) */

#if (GLC_SRV_INCLUDE_RTUS)
/*----------------------------------------------------------------------------
 * get RTUS value
 * --------------------------------------------------------------------------*/

static uint8_t *  test_upperstack_AttribGetRTUSValue( int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;

    *piLength = 0;
    switch ( iAttribIndex )
    {
    default:
        break;

    case GATT_SVC_RTUS_STATE_INDEX:
        pValue    = (uint8_t *)&TimeUpdateState;
        *piLength = sizeof(TimeUpdateState);
        break;
    }

    return ( pValue );
}
#endif /* (GLC_SRV_INCLUDE_RTUS) */

#if (GLC_SRV_INCLUDE_TEST)
static uint8_t *  test_upperstack_AttribGetTestValue( int iAttribIndex, int * piLength )
{
    uint8_t * pValue = NULL;
    uint16_t i = 0;


    *piLength = 0;
    switch( iAttribIndex )
    {
        default:
        break;

        case GATT_SVC_TEST_B001_FOR_READ:

        {
            for(i = 0; i < g_pGattTest->iReadDataLength; i++)
            {
                gTestDataReadOrUpdate[i] = i;
            }
            pValue    = (uint8_t *)gTestDataReadOrUpdate;
            *piLength = g_pGattTest->iReadDataLength;
        }

        break;

        case GATT_SVC_TEST_B003_FOR_NOTIFY:
        {
            for(i = 0; i < g_pGattTest->iUpdateDataLength; i++)
            {
                gTestDataReadOrUpdate[i] = gTestDataMax;
                gTestDataMax++;
            }
            pValue    = (uint8_t *)gTestDataReadOrUpdate;
            *piLength = g_pGattTest->iUpdateDataLength;
        }
        break;

        case GATT_SVC_TEST_B004_FOR_INDICATE:
        {
            for(i = 0; i < g_pGattTest->iUpdateDataLength; i++)
            {
                gTestDataReadOrUpdate[i] =g_pGattTest->iUpdateDataLength - i;
            }
            pValue    = (uint8_t *)gTestDataReadOrUpdate;
            *piLength = g_pGattTest->iUpdateDataLength;
        }
        break;
        
    }

  	return( pValue );
}
#endif /* (GLC_SRV_INCLUDE_TEST) */


/*----------------------------------------------------------------------------
 * get/read (local) attribute value
 * --------------------------------------------------------------------------*/

uint16_t  gattTestAttribGet( PGATTTest pGattTest, PGATTDService pService, int iAttribIndex,
                             int iOffset, uint16_t * pwLength, uint8_t **ppValue )
{
    uint8_t ServiceID;
    uint16_t  wCause  = GATT_ERR_ILLEGAL_PARAMETER;
    uint8_t * pValue  = NULL;
    int       iLength = 0;


    ServiceID = test_upperstack_Index2ServiceID( iAttribIndex );


    if ( (pGattTest->iUpdateCnt == 0) &&
            (ServiceID == blueAPI_ServiceGLS) &&
            (pGattTest->iUpdateAttribIndex == GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX) )
    {
        /* Glucose Measurement Context can only be updated from        */
        /* within update sequence starting with Glucose Measurement !! */
    }
    else
    {
        switch ( ServiceID )
        {
        default:
            break;

        case blueAPI_ServiceDIS:
            pValue = test_upperstack_AttribGetDISValue( iAttribIndex, &iLength );
            break;

        case blueAPI_ServiceGLS:
            pValue = test_upperstack_AttribGetGLSValue( pGattTest, iAttribIndex, &iLength );
            break;

            /*---- values of additional services ----*/
#if (GLC_SRV_INCLUDE_BAS)
        case blueAPI_ServiceBAS:
            pValue = test_upperstack_AttribGetBASValue( iAttribIndex, &iLength );
            break;
#endif /* (GLC_SRV_INCLUDE_BAS) */

#if (GLC_SRV_INCLUDE_CTS)
        case blueAPI_ServiceCTS:
            pValue = test_upperstack_AttribGetCTSValue( iAttribIndex, &iLength );
            break;
#endif /* (GLC_SRV_INCLUDE_CTS) */

#if (GLC_SRV_INCLUDE_NDCS)
        case blueAPI_ServiceNDCS:
            pValue = test_upperstack_AttribGetNDCSValue( iAttribIndex, &iLength );
            break;
#endif /* (GLC_SRV_INCLUDE_NDCS) */

#if (GLC_SRV_INCLUDE_RTUS)
        case blueAPI_ServiceRTUS:
            pValue = test_upperstack_AttribGetRTUSValue( iAttribIndex, &iLength );
            break;
#endif /* (GLC_SRV_INCLUDE_RTUS) */
#if (GLC_SRV_INCLUDE_TEST)
		case blueAPI_ServiceTST:
			pValue = test_upperstack_AttribGetTestValue( iAttribIndex, &iLength );
			break;
#endif /* (GLC_SRV_INCLUDE_RTUS) */

        }

        if ( (iLength - iOffset) >= 0 )
        {
            iLength -= iOffset;
            pValue  += iOffset;
            wCause   = GATT_SUCCESS;
        }
        else
        {
            wCause   = ATT_ERR | ATT_ERR_INVALID_OFFSET;
        }
    }

    *pwLength = iLength;
    *ppValue  = pValue;

    return ( wCause );
}

/*----------------------------------------------------------------------------
 * put/write (local) attribute value
 * --------------------------------------------------------------------------*/

uint16_t  gattTestAttribPut( PGATTTest pGattTest, PGATTDService pService, int iAttribIndex,
                             uint16_t wLength, uint8_t * pValue,
                             TGATTDWriteIndPostProc * pWriteIndPostProc )
{
    uint8_t  ServiceID;
    uint16_t  wCause  = GATT_SUCCESS;

    *pWriteIndPostProc = NULL;


    ServiceID = test_upperstack_Index2ServiceID( iAttribIndex );


    if ( (ServiceID == blueAPI_ServiceGLS) &&
            (iAttribIndex == GATT_SVC_GLS_RACP_INDEX)
       )
    {
        /* attribute value has variable size */
        if ( wLength > sizeof(TGLCControlPoint) )
        {
            wCause  = ATT_ERR | ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
            if ( test_upperstack_GLC_RACPOperationAllowed( pGattTest, *pValue ) )
            {
#if (GATTDEMO_CCCD_COUNT)
                /* check if all relevant CCCDs are non-zero (test case TP/SPE/BI-08-C) */
                wCause  = test_upperstack_CCCDCheckValue( pGattTest, 0, 2 );
#endif
                if ( wCause == GATT_SUCCESS )
                {
                    /* handle RACP request after sending write response */
                    *pWriteIndPostProc = test_upperstack_GLC_RACPHandleReq;
                }
            }
            else
            {
                wCause  = ATT_ERR | GLC_ERR_PROC_ALREADY_IN_PROGRESS;
            }
        }
    }
#if (GLC_SRV_INCLUDE_RTUS)
    else if ( (ServiceID == blueAPI_ServiceRTUS) &&
              (iAttribIndex == GATT_SVC_RTUS_CONTROL_POINT_INDEX)
            )
    {
        test_upperstack_CmdPrint( pGattTest, "   Write to RTUS CP; Cmd=0x%02x\r\n",
                                  ((PTimeUpdateCP)pValue)->Cmd);
    }
#endif
#if (GLC_SRV_INCLUDE_TEST)
    else if ( (ServiceID == blueAPI_ServiceTST) &&
              (iAttribIndex == GATT_SVC_TEST_B002_FOR_WRITE||iAttribIndex == GATT_SVC_TEST_B002_FOR_WRITE_NO_RESP)
            )
    {
        test_upperstack_CmdPrint( pGattTest, "   Write to Test ; iAttribIndex= %d\r\n",
                                  iAttribIndex
                                  );
    }
#endif

    else
    {
        wCause  = ATT_ERR | ATT_ERR_WRITE_NOT_PERMITTED;
    }

    return ( wCause );
}

#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
uint16_t  gattTestAttribPrepareWrite( PGATTTest pGattTest, uint16_t local_MDL_ID,uint16_t *handle)
{
	uint16_t  wCause  = GATT_SUCCESS;
    PGATTPrepareWrite pPrepareWrite;
    int num = pGattTest->queueIdx;
    int i;
    TAttribAppl attrib;
    int length;
    uint8_t *pData;
    int iSize;
  
    for(i=0;i<num;i++)
    {
    	pPrepareWrite = &pGattTest->prepareQueue[i];
		attrib = test_upperstack_Profile[pPrepareWrite->attribIndex];
		length = attrib.wValueLen;
		if(length !=0)
		{
	    	if(pPrepareWrite->attribLength > length)
	  		{
	    		wCause = ATT_ERR|ATT_ERR_INVALID_VALUE_SIZE;
				*handle = pPrepareWrite->handle;
				break;
	  		}
	  		if(pPrepareWrite->writeOffset > length)
	  		{
	  			wCause = ATT_ERR|ATT_ERR_INVALID_OFFSET;
				*handle = pPrepareWrite->handle;
				break;
	  		}
		}
		else
		{
	  		//fixme:not support now
	  		wCause = ATT_ERR|ATT_ERR_OUT_OF_RANGE;
	  		*handle = pPrepareWrite->handle;
	  		break;
		}
		if(attrib.pValueContext == NULL)
		{
	  		//fixme:not support now
	  		wCause = ATT_ERR|ATT_ERR_OUT_OF_RANGE;
	  		*handle = pPrepareWrite->handle;
	  		break;
		}	
  	}
  	if(wCause == GATT_SUCCESS)
  	{
    	for(i=0;i<num;i++)
    	{
      		pPrepareWrite = &pGattTest->prepareQueue[i];
	  		attrib = test_upperstack_Profile[pPrepareWrite->attribIndex];
      		pData = attrib.pValueContext;
	  		length = attrib.wValueLen;
	  		if((pPrepareWrite->attribLength+pPrepareWrite->writeOffset)<=length)
	  			memcpy((pData+pPrepareWrite->writeOffset),pPrepareWrite->data,pPrepareWrite->attribLength);
	  		else
	  		{
	    		iSize = length - pPrepareWrite->writeOffset;
				if(iSize>0)
				{
		  			memcpy((pData+pPrepareWrite->writeOffset),pPrepareWrite->data,iSize);
				}
	  		}
    	}

  	}

  	return( wCause );
}
#endif

/*----------------------------------------------------------------------------
 * service specific update (sequence) callback (RACP ...)
 * --------------------------------------------------------------------------*/

void  gattTestServiceUpdateCallback( PGATTTest pGattTest,
                                     uint16_t wCause, uint16_t wAttribIndex )
{
    if ( (wAttribIndex == GATT_SVC_GLS_MEASUREMENT_INDEX) ||
            (wAttribIndex == GATT_SVC_GLS_MEASUREMENT_CTXT_INDEX)
       )
    {
        test_upperstack_GLC_RACPProcComplete( pGattTest,  wCause );
    }
}

/*----------------------------------------------------------------------------
 * init. service specific data
 * --------------------------------------------------------------------------*/

void  gattTestServiceInitData( PGATTTest pGattTest )
{
    int i;

    for (i = 0; i < GLC_MAX_RACP_REQS; i++)
    {
        pGattTest->RACP.wSeqNbr[i]          = GLC_RACP_INIT_SEQ_NBR_DEFAULT;
        pGattTest->RACP.wNbrOfRecs[i]       = GLC_RACP_NBR_OF_STORED_RECS_DEFAULT;
        pGattTest->RACP.iSeqNbrDelta[i]     = 1;
        pGattTest->RACP.iSpecialBehavior[i] = 0;
    }

    pGattTest->RACP.iReqNum     = 0;
    pGattTest->RACP.iReqNumMax  = 0;

    pGattTest->RACP.Value.OpCode     = GLC_RACP_OPCODE_RESERVED;
}

/*----------------------------------------------------------------------------
 * link
 * --------------------------------------------------------------------------*/
PGATTLink test_upperstack_LinkAllocate(PGATTTest pGattTest, uint8_t * pBD)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        pLink = &pGattTest->linkTable[i];
        if (pLink->isUsed && ((memcmp(pLink->RemoteBd, pBD, BLUE_API_BD_SIZE) == 0)))
        {
            return pLink;
        }
    }

    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        pLink = &pGattTest->linkTable[i];
        if (!pLink->isUsed)
        {
            memset(pLink, 0, sizeof(TGATTLink));
            memcpy(pLink->RemoteBd, pBD, 6);
            pLink->isUsed = true;
            pLink->RemoteBdValid = true;
            pLink->idx = i;

            return pLink;
        }
    }
    return NULL;
}

void test_upperstack_LinkRelease(PGATTTest pGattTest, PGATTLink pLink)
{
    if (pLink)
    {
        pLink->isUsed = false;
        memset(pLink, 0, sizeof(TGATTLink));
    }
}

PGATTLink test_upperstack_LinkFindByLocal_MDL_ID(PGATTTest pGattTest, uint16_t local_MDL_ID)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        pLink = &pGattTest->linkTable[i];
        if (pLink->isUsed && (pLink->local_MDL_ID == local_MDL_ID))
        {
            return pLink;
        }
    }
    return NULL;
}

PGATTLink test_upperstack_LinkFindByBD(PGATTTest pGattTest, uint8_t * pBD)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        pLink = &pGattTest->linkTable[i];
        if (pLink->isUsed && ((memcmp(pLink->RemoteBd, pBD, BLUE_API_BD_SIZE) == 0)))
        {
            return pLink;
        }
    }
    return NULL;
}

PGATTLink test_upperstack_LinkFindByRole(PGATTTest pGattTest, uint16_t role)
{
    int i;
    PGATTLink pLink;
    for (i = 0; i < GATTDEMO_MAX_LINKS; i++)
    {
        pLink = &pGattTest->linkTable[i];
        if (pLink->isUsed && (pLink->role == role))
        {
            return pLink;
        }
    }
    return NULL;
}

PGATTLEChannel test_upperstack_LEChanAllocate(PGATTTest pGattTest,PGATTLink pLink,uint16_t channel)
{
  	int i;
  	PGATTLEChannel pChan;
  	for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
  	{
   		pChan= &pLink->LEChannel[i];
		if(pChan->isUsed &&(pChan->channel == channel))
		{
       		return pChan;
		}
  	}

  	for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
  	{
    	pChan= &pLink->LEChannel[i];
		if(!pChan->isUsed)
		{
	   		pChan->isUsed = true;
       		pChan->isDataChanConnected = false;
	   		pChan->channel = channel;

       		return pChan;
		}
  	}
  	return NULL;
}

void test_upperstack_LEChanClear(PGATTTest pGattTest,PGATTLink pLink)
{
  	int i;
  	PGATTLEChannel pChan;

  	if(pLink)
  	{
  		for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
  		{
      		pChan = &pLink->LEChannel[i];
	  		memset(pChan,0,sizeof(TGATTLEChannel));
    	}	
  	}
}

void test_upperstack_LEChanRelease(PGATTTest pGattTest,PGATTLEChannel pChan)
{
  	if(pChan)
  	{
		pChan->isUsed = false;
		memset(pChan,0,sizeof(TGATTLEChannel));
  	}
}

PGATTLEChannel test_upperstack_LEChanFind(PGATTTest pGattTest,PGATTLink pLink, uint16_t channel)
{
  	int i;
  	PGATTLEChannel pChan;
  	for (i = 0; i < GATTDEMO_MAX_LE_CHANNELS; i++)
  	{
    	pChan = &pLink->LEChannel[i];
		if(pChan->isUsed && (pChan->channel == channel))
		{
       		return pChan;
		}
  	}
  	return NULL;
}


