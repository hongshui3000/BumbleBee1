enum { __FILE_NUM__= 0 };
/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      bqb_database.c
* @brief     four databases for bqb.
* @details   none.
* @author    Tifnan
* @date      2014-08-21
* @version   v0.1
* *********************************************************************************************************
*/
#include "stdint.h"
#include "stddef.h"
#include "gatt.h"
#include "bqb_database.h"
//#include "section_config.h"

/* globals */
BQB_DB_INDEX BQB_GATT_Database_Index;  /**< databse index, small database default */
uint8_t BQB_ServiceNum = 0;
/**@brief store the information of databse servcies */
BQB_ServiceInfo Service_Info[12];

static const uint8_t ChaValDeviceName[] = "Test Database";    /* for DeviceName characteristic in GAP */


uint8_t ChaValV2_0[] /*SRAM_OFF_BD_DATA_SECTION*/ = 
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000"
                       "11111222223333344444555556666677777888889999900000123456789012";

uint8_t ChaValV2_01[] /*SRAM_OFF_BD_DATA_SECTION*/ =  "11111222223333344444555556666677777888889999900000";

uint8_t ChaValV3_01[1] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x03};

uint8_t ChaValV4_01[1] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x04};

uint8_t ChaValV5_01[1] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x05};

uint8_t ChaValV6_01[2] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x12, 0x34};

uint8_t ChaValV7_01[1] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x07};

uint8_t ChaValV7[4] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x01, 0x02, 0x03, 0x04};

uint8_t ChaValV8_01[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x08};

uint8_t ChaValV17[1] /*SRAM_OFF_BD_DATA_SECTION*/ =  {0x12};

static const uint8_t ChaValV11[1] = {0x0B};

static const uint8_t DesValV2D1[43] =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
    0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
    0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33
};

uint8_t DesValUserDes[] /*SRAM_OFF_BD_DATA_SECTION*/ = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

static const uint8_t DesValV5D4[1] = {0x44};
static const uint8_t SerC1UUID[16] =
{
    0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,     /* 128-bit uuid */
    0x00, 0x00, 0x00, 0x00, 0x0C, 0xA0, 0x00, 0x00
};

#define SerC2UUID SerC1UUID     /* has the same uuid */

uint8_t ChaValV2_1[] /*SRAM_OFF_BD_DATA_SECTION*/ = "111112222233333444445";

uint8_t DesValV2D_1[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56,
    0x78, 0x90, 0x11
};

uint8_t ChaValV2_2[] /*SRAM_OFF_BD_DATA_SECTION*/ = "2222233333444445555566";

uint8_t DesValV2D_2[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56,
    0x78, 0x90, 0x11, 0x22
};

uint8_t ChaValV2_3[] /*SRAM_OFF_BD_DATA_SECTION*/ = "33333444445555566666777";

uint8_t DesValV2D_3[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56,
    0x78, 0x90, 0x11, 0x22, 0x33
};

uint8_t ChaValV2_4[43] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
    0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
    0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33
};

uint8_t DesValV2D_4[43] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
    0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
    0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33
};

uint8_t ChaValV2_5[44] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
    0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
    0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44
};

uint8_t DesValV2D_5[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
   0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
   0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44
};

uint8_t ChaValV2_6[] /*SRAM_OFF_BD_DATA_SECTION*/=
{
   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
   0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
   0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55
};

uint8_t DesValV2D_6[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
   0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
   0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55
};

uint8_t ChaValV2_7[] /*SRAM_OFF_BD_DATA_SECTION*/ = "1111122222333334444455555666667777788888999";

uint8_t DesValV2D_7[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
   0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
   0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33
};

uint8_t ChaValV2_8[] /*SRAM_OFF_BD_DATA_SECTION*/ = "22222333334444455555666667777788888999990000";

uint8_t DesValV2D_8[] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
   0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
   0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44
};

uint8_t ChaValV2_9[46] /*SRAM_OFF_BD_DATA_SECTION*/ = "333334444455555666667777788888999990000011111";

uint8_t DesValV2D_9[45] /*SRAM_OFF_BD_DATA_SECTION*/ =
{
   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12,
   0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56, 0x78, 0x90, 0x11, 0x22,
   0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55
};

uint8_t ChaValV6[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x06};

uint8_t ChaValV4[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x04};

uint8_t ChaValV9[1] /*SRAM_OFF_BD_DATA_SECTION*/= {0x09};

uint8_t ChaValV9D2[1] /*SRAM_OFF_BD_DATA_SECTION*/= {0x22};

uint8_t ChaValV9D3[1] /*SRAM_OFF_BD_DATA_SECTION*/= {0x33};

uint8_t ChaV14[] /*SRAM_OFF_BD_DATA_SECTION*/ = "Length is";

uint8_t ChaV15[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x65};

uint8_t ClientConfig1[2] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x00, 0x00};
uint8_t ClientConfig2[2] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x00, 0x00};
uint8_t ClientConfig3[2] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x00, 0x00};

uint8_t DesValCharFormat[7] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x04, 0x00, 0x30, 0x01, 0x01, 0x31, 0x11};

uint8_t ChaValV8D1[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x01};

uint8_t ChaValV8D2[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x02};

uint8_t ChaValV8D3[1] /*SRAM_OFF_BD_DATA_SECTION*/ = {0x03};

/* small database */
const TAttribAppl SmallDatabase[] =
{
    /*----------------- handle = 0x0001 {Service=0x1800 ("Generic Access Profile")}-------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(0x1800),                         /* service UUID */
            HI_WORD(0x1800)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0002 Characteristic -- Device Name +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0003 Characteristic value  -- Device Name  */
    {
        ATTRIB_FLAG_VOID,                               /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A00),
            HI_WORD(0x2A00),
        },
        sizeof(ChaValDeviceName),                      /* could be 0 if Flags bit ASCII_Z=1 */
        (void*)ChaValDeviceName,
        GATT_PERM_READ                                  /* wPermissions */
    },

    /* +++++++++++ handle = 0x0004 Characteristic -- Appearance +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0005 Characteristic value  -- Appearance  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A01),
            HI_WORD(0x2A01),
            17
        },
        1,                                              /* bValueLen     */
        NULL,
        GATT_PERM_READ                                  /* wPermissions */
    },

    /* +++++++++++ handle = 0x0006 Characteristic -- Peripheral Preferred Connection Parameters  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0007 Characteristic value  -- Peripheral Preferred Connection Parameters  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A04),
            HI_WORD(0x2A04),
            100,
            200,
            0,
            LO_WORD(2000),
            HI_WORD(2000)
        },
        5,                                              /* bValueLen */
        NULL,
        GATT_PERM_READ                                  /* wPermissions */
    },

    /*----------handle = 0x0010  {Service=0x1801 ("Attribute Profile")}---*/
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

    /* +++++++++++ handle = 0x0011 Characteristic -- Service Changed +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_INDICATE | GATT_CHAR_PROP_READ                 /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0012 Characteristic value  -- Service Changed  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
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
        GATT_PERM_NOTIF_IND | GATT_PERM_READ           /* wPermissions */
    },

    /* handle = 0x0013  add client Characteristic configuration*/
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
        },
        2,                                           /* bValueLen */
        (void*)ClientConfig1,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },

    /*------ handle = 0x0020 {Service=0xA00B ("{Service=0xA00B ("Service B.1")} ")}---*/
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

    /* +++++++++++ handle = 0x0021 Characteristic -- Value V4  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0022 Characteristic value  -- Value V4  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB004),
            HI_WORD(0xB004),
            0x04
        },
        1,                                              /* bValueLen */
        NULL,
        GATT_PERM_READ                                  /* wPermissions */
    },
};


/* large database1  */
const TAttribAppl LargeDatabase1[] =
{
    /*------handle = 0x0001----{Service=0xA00D ("Service D")} Secondary service ---*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_SECONDARY_SERVICE),
            HI_WORD(GATT_UUID_SECONDARY_SERVICE),
            LO_WORD(0xA00D),                         /* service UUID */
            HI_WORD(0xA00D)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* handle = 0x0002 include service: {handle=0x0080, end group handle=0x0085, UUID=0xA00B} ---*/
    {
        (ATTRIB_FLAG_VALUE_INCL),                   /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_INCLUDE),
            HI_WORD(GATT_UUID_INCLUDE)
        },
        0,                                               /* index 0?  */
        ((uint8_t*)&LargeDatabase1[0]) + 51 * sizeof(TAttribAppl),   /* pointer to the include service, to check */
        GATT_PERM_READ                                    /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0003 Characteristic -- Value V12  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                             /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0004 Characteristic value  -- Value V12  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB00C),
            HI_WORD(0xB00C),
            0x0C
        },
        1,                                              /* bValueLen */
        NULL,
        GATT_PERM_READ_AUTHEN_REQ                       /* wPermissions */
    },

    /* +++++++++++ handle = 0x0005 Characteristic -- Value V11  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0006 Characteristic value  -- Value V11  */
    {
        ATTRIB_FLAG_UUID_128BIT | ATTRIB_FLAG_VOID,      /* wFlags */
        {                                               /* bTypeValue */
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23,  /* this order ok? */
            0x01, 0x00, 0x00, 0x00, 0x00, 0x0B, 0xB0,
            0x00, 0x00
        },
        1,                                              /* bValueLen */
        (void*)ChaValV11,
        GATT_PERM_READ_AUTHOR_REQ                       /* wPermissions */
    },

    /*------------- handle = 0x0010 {Service=0x1801 ("Attribute Profile")}-------------------*/
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

    /* +++++++++++ handle = 0x0011  Characteristic -- Service Changed +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_INDICATE | GATT_CHAR_PROP_READ    /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0012 Characteristic value  -- Service Changed  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
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
        GATT_PERM_NOTIF_IND | GATT_PERM_READ            /* wPermissions */
    },

    /* handle = 0x0013  add client Characteristic configuration*/
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
        },
        2,                                           /* bValueLen */
        (void*)ClientConfig1,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },

    /*---- handle = 0x0020  {Service=0xA00A ("Service A")} -------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(0xA00A),                         /* service UUID */
            HI_WORD(0xA00A)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /*--------handle = 0x0021 include service: {Service=0xA00D ("Service D")} -----*/
    {
        (ATTRIB_FLAG_VOID),                       /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_INCLUDE),
            HI_WORD(GATT_UUID_INCLUDE)
        },
        0,                                               /* index 0?  */
        (void*)&LargeDatabase1[0],                             /* pointer to the include service, to check */
        GATT_PERM_READ                                    /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0022 Characteristic -- Value V1  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0023 Characteristic value  -- Value V1  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB001),
            HI_WORD(0xB001),
            0x01
        },
        1,                                              /* bValueLen */
        NULL,
		(GATT_PERM_READ_AUTHEN_REQ  | GATT_PERM_KEYSIZE(16))      /* wPermissions */
    },

    /* +++++++++++ handle = 0x0024 Characteristic -- Value V2  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE  /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0025 Characteristic value  -- Value V2  */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_0) - 1),                         /* bValueLen */
        (void*)ChaValV2_0,
        GATT_PERM_READ | GATT_PERM_WRITE                       /* wPermissions */
    },

    /* +++++++++++ handle = 0x0026 Characteristic -- Value V2  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0027 Characteristic value  -- Value V2  */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_01) - 1),                        /* bValueLen */
        (void*)ChaValV2_01,
        GATT_PERM_WRITE                                /* wPermissions */
    },

    /* +++++++++++ handle = 0x0028 Characteristic -- Value V3  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0029 Characteristic value  -- Value V3 */
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB003),
            HI_WORD(0xB003),
        },
        1,                                          /* bValueLen */
        (void*)ChaValV3_01,
        GATT_PERM_WRITE                                /* wPermissions */
    },

    /*-------handle = 0x0030 {{Service=0xA00B ("Service B.4")} -------------------*/
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

    /* +++++++++++ handle = 0x0031 Characteristic -- Value V7  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0032 Characteristic value  -- Value V7 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB007),
            HI_WORD(0xB007),
        },
        1,             /* bValueLen */
        (void*)ChaValV7_01,
        GATT_PERM_WRITE                                /* wPermissions */
    },

    /*-------handle = 0x0040 {Service=0x1800 ("Generic Access Profile")}-------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
            LO_WORD(0x1800),                         /* service UUID */
            HI_WORD(0x1800)
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0041  Characteristic -- Device Name +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0042 Characteristic value  -- Device Name  */
    {
        ATTRIB_FLAG_VOID,                               /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A00),
            HI_WORD(0x2A00),
        },
        sizeof(ChaValDeviceName),                      /* could be 0 if Flags bit ASCII_Z=1 */
        (void*)ChaValDeviceName,
        GATT_PERM_READ                                  /* wPermissions */
    },

    /* +++++++++++ handle = 0x0043 Characteristic -- Appearance +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0044 Characteristic value  -- Appearance  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A01),
            HI_WORD(0x2A01),
            17
        },
        2,                                              /* bValueLen     */
        NULL,
        GATT_PERM_READ                                  /* wPermissions */
    },

    /* ++++++++handle = 0x0045 Characteristic -- Peripheral Preferred Connection Parameters  ++++ */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0046 Characteristic value  -- Peripheral Preferred Connection Parameters  */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0x2A04),
            HI_WORD(0x2A04),
            LO_WORD(100),
					  HI_WORD(100),
            LO_WORD(200),
					  HI_WORD(200),
					  LO_WORD(0),
            HI_WORD(0),
            LO_WORD(2000),
            HI_WORD(2000)
        },
        8,                                              /* bValueLen */
        NULL,
        GATT_PERM_READ                                  /* wPermissions */
    },


    /*-------handle = 0x0050 {Service=0xA00B ("Service B.3")} -------------------*/
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

    /* +++++++++++ handle = 0x0051 Characteristic -- Value V6  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_READ |
            GATT_CHAR_PROP_WRITE_NO_RSP |
            GATT_CHAR_PROP_WRITE |
            GATT_CHAR_PROP_NOTIFY |
            GATT_CHAR_PROP_INDICATE)                     /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0052 Characteristic value  -- Value V6 */       /* for BQB server notification */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                         /* bTypeValue */
            LO_WORD(0xB006),
            HI_WORD(0xB006),
        },
        sizeof(ChaValV6),                         /* bValueLen */
        (void*)ChaValV6,
        GATT_PERM_WRITE | GATT_PERM_READ | GATT_PERM_NOTIF_IND     /* wPermissions */
    },

    /* handle = 0x0053  add client Characteristic configuration*/
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
        },
        2,                                           /* bValueLen */
        (void*)ClientConfig2,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },

    /*----handle = 0x0060 {Service=0xA00B ("Service B.1")} -------------------*/
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

    /* +++++++++++ handle = 0x0061 Characteristic -- Value V4  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE | GATT_CHAR_PROP_READ  /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0062 Characteristic value  -- Value V4 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB004),
            HI_WORD(0xB004),
        },
        1,             /* bValueLen */
        (void*)ChaValV4,
        GATT_PERM_WRITE_AUTHEN_REQ | GATT_PERM_READ        /* wPermissions */
    },

    /* +++++++++++ handle = 0x0063 Characteristic -- Value V4  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE | GATT_CHAR_PROP_READ  /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0064 Characteristic value  -- Value V4 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB004),
            HI_WORD(0xB004),
        },
        1,             /* bValueLen */
        (void*)ChaValV4_01,
        GATT_PERM_WRITE | GATT_PERM_READ        /* wPermissions */
    },

    /* +++++++++++ handle = 0x0065 Descriptor -- Server Characteristic Configuration  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_SERVER_CONFIG),
            HI_WORD(GATT_UUID_CHAR_SERVER_CONFIG),
        },
        2,                                          /* bValueLen */
        (void*)ClientConfig3,
        GATT_PERM_READ | GATT_PERM_WRITE            /* wPermissions */
    },

    /* +++++++++++ handle = 0x0066 Characteristic -- Value V4  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            0x00                                     /* characteristic properties, 0x00!!!! */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0067 Characteristic value  -- Value V4 */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB004),
            HI_WORD(0xB004),
            0x04
        },
        1,                                              /* bValueLen */
        NULL,
        GATT_PERM_NONE                                  /* wPermissions */
    },

    /* +++++++++++ handle = 0x0068 Descriptor -- Server Characteristic Configuration  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB012),
            HI_WORD(0xB012),
        },
        sizeof(DesValV2D1),                                          /* bValueLen */
        (void*)DesValV2D1,
        GATT_PERM_NONE                          /* wPermissions */
    },

    /* +++++++++++ handle = 0x0069 Characteristic -- Value V4  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                         /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x006A Characteristic value  -- Value V4 */
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB004),
            HI_WORD(0xB004),
        },
        sizeof(DesValV2D1),                             /* bValueLen */
        (void*)DesValV2D1,
        GATT_PERM_READ                                  /* wPermissions */
    },

    /* +++++++++++ handle = 0x006B Descriptor -- Server Characteristic Configuration  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(0xB012),
            HI_WORD(0xB012),
        },
        sizeof(DesValV2D1),                         /* bValueLen */
        (void*)DesValV2D1,
        GATT_PERM_READ            /* wPermissions */
    },

    /*------handle = 0x0070 {{Service=0xA00B ("Service B.2")} -------------------*/
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

    /* +++++++++++ handle = 0x0071 Characteristic -- Value V5 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE | GATT_CHAR_PROP_EXT_PROP /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* handle = 0x0072  Characteristic value  -- Value V5 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB005),
            HI_WORD(0xB005),
        },
        1,                                                 /* bValueLen */
        (void*)ChaValV5_01,
        GATT_PERM_READ |  GATT_PERM_WRITE_AUTHOR_REQ       /* wPermissions */
    },

    /* +++++++++++ handle = 0x0073 Descriptor --Characteristic Extended Properties  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_EXTENDED_PROP),
            HI_WORD(GATT_UUID_CHAR_EXTENDED_PROP),
            0x03,
            0x00
        },
        2,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ            /* wPermissions */
    },

    /* +++++++++++ handle = 0x0074 Descriptor --Characteristic User Description  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_USER_DESCR),
            HI_WORD(GATT_UUID_CHAR_USER_DESCR),
        },
        (sizeof(DesValUserDes) - 1),                                           /* bValueLen */
        (void*)DesValUserDes,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },

    /* +++++++++++ handle = 0x0075 Descriptor --Characteristic Format +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_FORMAT),
            HI_WORD(GATT_UUID_CHAR_FORMAT),
        },
        7,                                           /* bValueLen */
        (void*)DesValCharFormat,
        GATT_PERM_READ | GATT_PERM_WRITE           /* wPermissions */
    },

    /* +++++++++++ handle = 0x0076 Descriptor --Descriptor V5D4 (128-bit UUID)  +++++++++++++  */
    {
        ATTRIB_FLAG_UUID_128BIT,                     /* wFlags */
        {
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,     /* 128-bit uuid */
            0x00, 0x00, 0x00, 0x00, 0xD4, 0xD5, 0x00, 0x00
        },
        sizeof(DesValV5D4),                                           /* bValueLen */
        (void*)DesValV5D4,
        GATT_PERM_READ                                 /* wPermissions */
    },

    /*----------------- handle = 0x0080 {{Service=0xA00B ("Service B.5")} -------------------*/
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

    /* +++++++++++ handle = 0x0081 Characteristic -- Value V8 +++++++++++++  */
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

    /* handle = 0x0082 Characteristic value  -- Value V8 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB008),
            HI_WORD(0xB008),
        },
        1,                                                 /* bValueLen */
        (void*)ChaValV8_01,
        GATT_PERM_READ |  GATT_PERM_WRITE_AUTHEN_REQ       /* wPermissions */
    },

    /* +++++++++++ handle = 0x0083 Descriptor --Descriptor V8D1 +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                           /* wFlags */
        {
            LO_WORD(0xB015),
            HI_WORD(0xB015),
        },
        1,                                           /* bValueLen */
        (void*)ChaValV8D1,
        GATT_PERM_READ_AUTHEN_REQ |  GATT_PERM_WRITE_AUTHEN_REQ    /* wPermissions */
    },

    /* +++++++++++ handle = 0x0084 Descriptor --Descriptor V8D2  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB016),
            HI_WORD(0xB016),
        },
        1,                                           /* bValueLen */
        (void*)ChaValV8D2,
        GATT_PERM_READ_AUTHOR_REQ |  GATT_PERM_WRITE_AUTHOR_REQ    /* wPermissions */
    },

    /* +++++++++++ handle = 0x0085 Descriptor --Descriptor V8D3  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                               /* wFlags */
        {
            LO_WORD(0xB017),
            HI_WORD(0xB017),
        },
        1,                                           /* bValueLen */
        (void*)ChaValV8D3,
        GATT_PERM_READ_AUTHEN_REQ |  GATT_PERM_WRITE_AUTHEN_REQ | GATT_PERM_KEYSIZE(10)   /* wPermissions */
    },

    /*-------handle = 0x0090 {{Service=0x0000A00C000000000123456789ABCDEF ("Service C.1")}} ----------*/
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),           /* wFlags     */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),              /* bTypeValue */
        },
        UUID_128BIT_SIZE,                                    /* bValueLen     */
        (void*)SerC1UUID,                                       /* pValueContext */
        GATT_PERM_READ                                      /* wPermissions  */
    },

    /*------handle = 0x0091 include service: {handle=0x0001, end group handle=0x0006, UUID=0xA00D}-------*/
    {
        (ATTRIB_FLAG_VOID),                       /* wFlags     */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_INCLUDE),
            HI_WORD(GATT_UUID_INCLUDE)
        },
        0,                                               /* index 0?  */
        (void*)&LargeDatabase1[0],                             /* pointer to the include service, to check */
        GATT_PERM_READ                                    /* wPermissions  */
    },

    /* +++++++++++ handle = 0x0092 Characteristic -- Value V9 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE | GATT_CHAR_PROP_EXT_PROP /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x0093 Characteristic value  -- Value V9 */
    {
        ATTRIB_FLAG_UUID_128BIT,                                /* wFlags */
        {                                                           /* bTypeValue */
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,     /* 128-bit uuid */
            0x00, 0x00, 0x00, 0x00, 0x09, 0xB0, 0x00, 0x00
        },
        1,                                                 /* bValueLen */
        (void*)ChaValV9,
        GATT_PERM_READ |  GATT_PERM_WRITE       /* wPermissions */
    },

    /* +++++++++++ handle = 0x0094 Descriptor --Characteristic Extended Properties   +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {
            LO_WORD(GATT_UUID_CHAR_EXTENDED_PROP),
            HI_WORD(GATT_UUID_CHAR_EXTENDED_PROP),
            0x01,
            0x00
        },
        2,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },

    /* +++++++++++ handle = 0x0095 Descriptor --Descriptor V9D2 (128-bit UUID))  +++++++++++++  */
    {
        ATTRIB_FLAG_UUID_128BIT,                     /* wFlags */
        {
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,     /* 128-bit uuid */
            0x00, 0x00, 0x00, 0x00, 0xD2, 0xD9, 0x00, 0x00
        },
        1,                                           /* bValueLen */
        (void*)ChaValV9D2,
        GATT_PERM_READ |  GATT_PERM_WRITE   /* wPermissions */
    },

    /* +++++++++++ handle = 0x0096 Descriptor --Descriptor V9D3 (128-bit UUID)  +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                    /* wFlags */
        {
            0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,     /* 128-bit uuid */
            0x00, 0x00, 0x00, 0x00, 0xD3, 0xD9, 0x00, 0x00
        },
        1,                                           /* bValueLen */
        (void*)ChaValV9D3,
        GATT_PERM_WRITE                 /* wPermissions */
    },

    /*-----handle = 0x00A0  {Service=0xA00F ("Service F")}  -------------------*/
    {
        (ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),                        /* wFlags     */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),                           /* bTypeValue */
            LO_WORD(0xA00F),
            HI_WORD(0xA00F),
        },
        UUID_16BIT_SIZE,                            /* bValueLen     */
        NULL,                                       /* pValueContext */
        GATT_PERM_READ                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x00A1 Characteristic -- Value V14 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                      /* wPermissions */
    },

    /* handle = 0x00A2 Characteristic value  -- Value V14 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB00E),
            HI_WORD(0xB00E),
        },
        (sizeof(ChaV14) - 1),                           /* bValueLen */
        (void*)ChaV14,
        GATT_PERM_READ                      /* wPermissions */
    },

    /* +++++++++++ handle = 0x00A3 Descriptor --Descriptor V5D4 (128-bit UUID)  +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {
            LO_WORD(GATT_UUID_CHAR_FORMAT),
            HI_WORD(GATT_UUID_CHAR_FORMAT),
            0x19, 0x00, 0x00, 0x30, 0x01, 0x00, 0x00
        },
        7,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ                      /* wPermissions */
    },

    /* +++++++++++ handle = 0x00A4 Characteristic -- Value V15 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE    /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00A5 Characteristic value  -- Value V15 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB00F),
            HI_WORD(0xB00F),
        },
        1,                                                 /* bValueLen */
        (void*)ChaV15,
        GATT_PERM_READ |  GATT_PERM_WRITE             /* wPermissions */
    },

    /* +++++++++++ handle = 0x00A6 Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {
            LO_WORD(GATT_UUID_CHAR_FORMAT),
            HI_WORD(GATT_UUID_CHAR_FORMAT),
            0x04, 0x00, 0x01, 0x27, 0x01, 0x01, 0x00
        },
        7,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ                      		/* wPermissions */
    },

    /* +++++++++++ handle = 0x00A7 Characteristic -- Value V6 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE    /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00A8 Characteristic value  -- Value V6 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB006),
            HI_WORD(0xB006),
        },
        2,                                                 /* bValueLen */
        (void*)ChaValV6_01,
        GATT_PERM_READ |  GATT_PERM_WRITE             /* wPermissions */
    },

    /* +++++++++++ handle = 0x00A9 Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {
            LO_WORD(GATT_UUID_CHAR_FORMAT),
            HI_WORD(GATT_UUID_CHAR_FORMAT),
            0x06, 0x00, 0x10, 0x27, 0x01, 0x02, 0x00
        },
        7,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ                      /* wPermissions */
    },

    /* +++++++++++ handle = 0x00AA Characteristic -- Value V7 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE    /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00AB Characteristic value  -- Value V7 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB007),
            HI_WORD(0xB007),
        },
        4,                                                 /* bValueLen */
        (void*)ChaValV7,
        GATT_PERM_READ |  GATT_PERM_WRITE             /* wPermissions */
    },

    /* +++++++++++ handle = 0x00AC Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {
            LO_WORD(GATT_UUID_CHAR_FORMAT),
            HI_WORD(GATT_UUID_CHAR_FORMAT),
            0x08, 0x00, 0x17, 0x27, 0x01, 0x03, 0x00
        },
        7,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ                      /* wPermissions */
    },

    /* +++++++++++ handle = 0x00AD Characteristic -- Value V16 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ    /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00AE Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB010),
            HI_WORD(0xB010),
            0x65, 0x34, 0x12, 0x04, 0x03, 0x02, 0x01
        },
        7,                                                 /* bValueLen */
        NULL,
        GATT_PERM_READ                 /* wPermissions */
    },

    /* +++++++++++ handle = 0x00AF Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {
            LO_WORD(0x2905),    /* tifnan change for test */        /* care!!!!! change from 2905-->2904, because stack not support */
            HI_WORD(0x2905),
            0xA6, 0xA9, 0xAC
        },
        3,                                           /* bValueLen */
        NULL,
        GATT_PERM_READ                      /* wPermissions */
    },

    /* +++++++++++ handle = 0x00B0 Characteristic -- Value V17 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE_AUTH_SIGNED   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00B1 Characteristic value  -- Value V17 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB011),
            HI_WORD(0xB011),
        },
        1,                                                 /* bValueLen */
        (void*)ChaValV17,
        GATT_PERM_READ | GATT_PERM_WRITE                 /* wPermissions */
    },

    /*-----handle = 0x00C0 {Service=0x0000A00C000000000123456789ABCDEF ("Service C.2")}-------------------*/
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),                                /* wFlags     */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),                           /* bTypeValue */
        },
        UUID_128BIT_SIZE,                                           /* bValueLen     */
        (void*)SerC2UUID,                                           /* pValueContext */
        GATT_PERM_READ                                              /* wPermissions  */
    },

    /* +++++++++++ handle = 0x00C1 Characteristic -- Value V10 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                            /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /*  handle = 0x00C2  Characteristic value  -- Value V10 */
    {
        ATTRIB_FLAG_VALUE_INCL,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB00A),
            HI_WORD(0xB00A),
            0x0A
        },
        1,                                                 /* bValueLen */
        NULL,
        GATT_PERM_READ                 /* wPermissions */
    },

    /* +++++++++++ handle = 0x00C3 Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /*  handle = 0x00C4  Characteristic value  -- Value V2 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_1) - 1),                                                 /* bValueLen */
        (void*)(ChaValV2_1),
        GATT_PERM_READ |  GATT_PERM_WRITE               /* wPermissions */
    },

    /* +++++++++++ handle = 0x00C5  Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB012),
            HI_WORD(0xB012),
        },
        sizeof(DesValV2D_1),                                           /* bValueLen */
        (void*)DesValV2D_1,
        GATT_PERM_READ | GATT_PERM_WRITE                            /* wPermissions */
    },

    /* +++++++++++  handle = 0x00C6 Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /*  handle = 0x00C7  Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_2) - 1),                                                 /* bValueLen */
        (void*)ChaValV2_2,
        GATT_PERM_READ |  GATT_PERM_WRITE               /* wPermissions */
    },

    /* +++++++++++  handle = 0x00C8  Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB013),
            HI_WORD(0xB013),
        },
        sizeof(DesValV2D_2),                                           /* bValueLen */
        (void*)DesValV2D_2,
        GATT_PERM_READ | GATT_PERM_WRITE                            /* wPermissions */
    },

    /* +++++++++++  handle = 0x00C9  Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /*  handle = 0x00CA  Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_3) - 1),                                                 /* bValueLen */
        (void*)ChaValV2_3,
        GATT_PERM_READ |  GATT_PERM_WRITE               /* wPermissions */
    },

    /* +++++++++++ handle = 0x00B  Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB014),
            HI_WORD(0xB014),
        },
        sizeof(DesValV2D_3),                                           /* bValueLen */
        (void*)DesValV2D_3,
        GATT_PERM_READ | GATT_PERM_WRITE                            /* wPermissions */
    },

    /* +++++++++++  handle = 0x00CC  Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /*  handle = 0x00CD Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        sizeof(ChaValV2_4),                                                 /* bValueLen */
        (void*)ChaValV2_4,
        (GATT_PERM_READ |  GATT_PERM_WRITE | GATT_PERM_KEYSIZE(16))        /* wPermissions */
    },

    /* +++++++++++  handle = 0x00CE  Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB012),
            HI_WORD(0xB012),
        },
        sizeof(DesValV2D_4),                                           /* bValueLen */
        (void*)DesValV2D_4,
        GATT_PERM_READ | GATT_PERM_WRITE                            /* wPermissions */
    },

    /* +++++++++++  handle = 0x00CF Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /*  handle = 0x00D0 Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        sizeof(ChaValV2_5),                           /* bValueLen */
        (void*)ChaValV2_5,
        GATT_PERM_READ |  GATT_PERM_WRITE               /* wPermissions */
    },

    /* +++++++++++  handle = 0x00D1 Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB013),
            HI_WORD(0xB013),
        },
        sizeof(DesValV2D_5),                                           /* bValueLen */
        (void*)DesValV2D_5,
        GATT_PERM_READ | GATT_PERM_WRITE                            /* wPermissions */
    },

    /* +++++++++++ handle = 0x00D2 Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00D3 Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        sizeof(ChaValV2_6),                                                 /* bValueLen */
        (void*)ChaValV2_6,
        GATT_PERM_READ |  GATT_PERM_WRITE               /* wPermissions */
    },

    /* +++++++++++ handle = 0x00D4 Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB014),
            HI_WORD(0xB014),
        },
        sizeof(DesValV2D_6),                                           /* bValueLen */
        (void*)DesValV2D_6,
        GATT_PERM_READ | GATT_PERM_WRITE                            /* wPermissions */
    },

    /* +++++++++++ handle = 0x00D5 Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00D6 Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_7) - 1),                                                 /* bValueLen */
        (void*)ChaValV2_7,
        GATT_PERM_READ_AUTHEN_REQ |  GATT_PERM_WRITE_AUTHEN_REQ               /* wPermissions */
    },

    /* +++++++++++ handle = 0x00D7 Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB012),
            HI_WORD(0xB012),
        },
        sizeof(DesValV2D_7),                                           /* bValueLen */
        (void*)DesValV2D_7,
        (GATT_PERM_WRITE_AUTHEN_MITM_REQ |  GATT_PERM_READ_AUTHEN_MITM_REQ)                             /* wPermissions */
    },

    /* +++++++++++ handle = 0x00D8 Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00D9 Characteristic value  -- Value V16 */
    {
        ATTRIB_FLAG_VOID,                         /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_8) - 1),                                                 /* bValueLen */
        (void*)ChaValV2_8,
        GATT_PERM_READ_AUTHOR_REQ |  GATT_PERM_WRITE_AUTHOR_REQ               /* wPermissions */
    },

    /* +++++++++++ handle = 0x00DA Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB013),
            HI_WORD(0xB013),
        },
        sizeof(DesValV2D_8),                                           /* bValueLen */
        (void*)DesValV2D_8,
        GATT_PERM_READ_AUTHOR_REQ |  GATT_PERM_WRITE_AUTHOR_REQ                             /* wPermissions */
    },

    /* +++++++++++ handle = 0x00DB Characteristic -- Value V2 +++++++++++++  */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ | GATT_CHAR_PROP_WRITE   /* characteristic properties */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ              /* wPermissions */
    },

    /* handle = 0x00DC Characteristic value  -- Value V16 */
    {
        //ATTRIB_FLAG_VALUE_APPL,
        ATTRIB_FLAG_VOID,
        /* wFlags */
        {                                               /* bTypeValue */
            LO_WORD(0xB002),
            HI_WORD(0xB002),
        },
        (sizeof(ChaValV2_9) - 1),                                                 /* bValueLen */
        (void*)ChaValV2_9,
        //0,
        //NULL,
        GATT_PERM_WRITE_AUTHEN_MITM_REQ |  GATT_PERM_READ_AUTHEN_MITM_REQ   /* wPermissions */
    },

    /* +++++++++++ handle = 0x00DD Descriptor --Characteristic Format   +++++++++++++  */
    {
        ATTRIB_FLAG_VOID,                     /* wFlags */
        {
            LO_WORD(0xB014),
            HI_WORD(0xB014),
        },
        sizeof(DesValV2D_9),                                           /* bValueLen */
        (void*)DesValV2D_9,
        GATT_PERM_READ_AUTHEN_REQ |  GATT_PERM_WRITE_AUTHEN_REQ                             /* wPermissions */
    }
};

/**
 * @brief select bqb gatt database.
 *
 * @param  db_index -- database index, SMALL_DATABASE/LARGE_DATABASE1/LARGE_DATABASE1/LARGE_DATABASE3.
 * @return the select action result.
 * @note must call it firstly
 * @retval 1-- select database successfully.
 *          0-- select database failed.
*/
uint8_t BQB_SelectDatabase(BQB_DB_INDEX db_index)
{
    BQB_GATT_Database_Index = db_index;

    if (SMALL_DATABASE == db_index)
    {
        BQB_ServiceNum = 3;
        return 1;
    }
    else if (LARGE_DATABASE1 == db_index)
    {
        BQB_ServiceNum = 12;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief init the information of bqb services in database.
 *
 * @param  none.
 * @return the result of service information init.
 * @note must call it after BQB_SelectDatabase()
 * @retval 1-- init service information successfully.
 *         0-- init service informatio failed.
*/
uint8_t BQB_ServiceInfoInit(void)
{
    /* init information of servcies in database  */
    if (SMALL_DATABASE == BQB_GATT_Database_Index)
    {
        Service_Info[0].ser_id = BQB_SERVICE_ID_GAP;
        Service_Info[0].num_attr    =   7;
        Service_Info[1].ser_id = BQB_SERVICE_ID_ATP;
        Service_Info[1].num_attr    =   4;
        Service_Info[2].ser_id = BQB_SERVICE_ID_B1;
        Service_Info[2].num_attr    =   3;
        return 1;
    }
    else if (LARGE_DATABASE1 == BQB_GATT_Database_Index)
    {
        Service_Info[0].ser_id = BQB_SERVICE_ID_D;
        Service_Info[0].num_attr    =   6;
        Service_Info[1].ser_id = BQB_SERVICE_ID_ATP;
        Service_Info[1].num_attr    =   4;				/* add client Characteristic configuration  */
        Service_Info[2].ser_id = BQB_SERVICE_ID_A;
        Service_Info[2].num_attr    =   10;
        Service_Info[3].ser_id = BQB_SERVICE_ID_B4;
        Service_Info[3].num_attr    =   3;
        Service_Info[4].ser_id = BQB_SERVICE_ID_GAP;
        Service_Info[4].num_attr    =   7;
        Service_Info[5].ser_id = BQB_SERVICE_ID_B3;
        Service_Info[5].num_attr    =   4;              /* add client Characteristic configuration  */
        Service_Info[6].ser_id = BQB_SERVICE_ID_B1;
        Service_Info[6].num_attr    =   12;
        Service_Info[7].ser_id = BQB_SERVICE_ID_B2;
        Service_Info[7].num_attr    =   7;
        Service_Info[8].ser_id = BQB_SERVICE_ID_B5;
        Service_Info[8].num_attr    =   6;
        Service_Info[9].ser_id = BQB_SERVICE_ID_C1;
        Service_Info[9].num_attr    =   7;
        Service_Info[10].ser_id = BQB_SERVICE_ID_F;
        Service_Info[10].num_attr   =   18;
        Service_Info[11].ser_id = BQB_SERVICE_ID_C2;
        Service_Info[11].num_attr   =   30;
        return 1;
    }
    else    /* invalid database index */
    {
        return 0;
    }
}
