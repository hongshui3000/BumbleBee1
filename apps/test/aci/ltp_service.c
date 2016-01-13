enum { __FILE_NUM__ = 0 };

/* find me service for ltp to register service */
#include "btltp.h"


uint8_t ChaValV8_011[1] = {0x08};
uint8_t ChaValV8D11[1]  = {0x01};
uint8_t ChaValV8D21[1]  = {0x02};
uint8_t ChaValV8D31[1]  = {0x03};

const TAttribAppl GattdFindMeProfile[] =
{
    /*----------handle = 0x0008  {Service=0x1801 ("Attribute Profile")}---*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(0x1801),                         /* service UUID */
            HI_WORD(0x1801)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0009 Characteristic -- Service Changed +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_INDICATE                /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },


    /* handle = 0x000a Characteristic value  -- Service Changed  */
    {
        ATTRIB_FLAG_VALUE_APPL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A05),
            HI_WORD(0x2A05),
            0x01,
            0x00,
            0xff,
            0xff
        },
        4,                                              /* bValueLen */
        NULL,
        GATT_PERM_NOTIF_IND                             /* wPermissions */
    },


    /* handle = 0x000b Characteristic value  -- Client Characteristic Configuration  */

    {
        (ATTRIB_FLAG_VALUE_INCL |  ATTRIB_FLAG_CCCD_APPL),                           /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },

    /*----------------- handle = 0x000c {{Service=0xA00B ("Service B.5")} -------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(0xA00B),                         /* service UUID */
            HI_WORD(0xA00B)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x000d Characteristic -- Value V8 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },


    /* handle = 0x000e Characteristic value  -- Value V8 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB008),
            HI_WORD(0xB008),
        },
        1,                        /* bValueLen */
        (void*)ChaValV8_011,
        GATT_PERM_READ | GATT_PERM_WRITE                       /* wPermissions */
    },


    /* +++++++++++ handle = 0x000f Descriptor --Descriptor V5D1  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB015),
            HI_WORD(0xB015),
        },
        1,                                           /* bValueLen */
        (void*)ChaValV8D11,
        GATT_PERM_READ | GATT_PERM_WRITE     /* wPermissions */
    },

    /* +++++++++++ handle = 0x0010 Descriptor --Descriptor V5D2  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                    /* wFlags */
        {
            LO_WORD(0xB016),
            HI_WORD(0xB016),
        },
        1,                                           /* bValueLen */
        (void*)ChaValV8D21,
        GATT_PERM_READ    /* wPermissions */
    },

    /* +++++++++++ handle = 0x0011 Descriptor --Descriptor V5D3  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB017),
            HI_WORD(0xB017),
        },
        1,                                           /* bValueLen */
        (void*)ChaValV8D31,
        GATT_PERM_WRITE   /* wPermissions */
    },


    /*----------------- handle = 0x0012 {Service = 0x180f (Battery Service)}-------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(0x180F),                         /* service UUID */
            HI_WORD(0x180F)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0013 Characteristic -- Value V8 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY                   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x0014 Characteristic value  -- Value V8 */
    {
        ATTRIB_FLAG_VOID | ATTRIB_FLAG_VALUE_APPL,      /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A19),
            HI_WORD(0x2A19),
        },
        1,                                                 /* bValueLen */
        NULL,
        GATT_PERM_READ       /* wPermissions */
    },

    /* handle = 0x0015  Characteristic value  client Characteristic configuration*/
    {
        (ATTRIB_FLAG_VALUE_INCL |  ATTRIB_FLAG_CCCD_APPL),                           /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },
};

const int GattdFindMeProfileSize = sizeof(GattdFindMeProfile);
const int gattSvcFindMeNbrOfAttrib = sizeof(GattdFindMeProfile) / sizeof(TAttribAppl);

