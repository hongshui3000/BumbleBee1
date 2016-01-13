/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      bqb_database.h
* @brief     header file of gatt bqb database.
* @details   none.
* @author    Tifnan
* @date      2014-08-28
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef _BQB_DATABASE_H_
#define _BQB_DATABASE_H_

#include "rtl_types.h"
#include "gatt.h"

#define BQB_GATT_SUPPORT_SERVICE_MAX    12      /* in large database 1/2/3 */
#define BQB_MAX_HANDLE_UUID             60      /* --- attribute handle to (16 bit) UUID mapping ---*/
#define BQB_NOTIFICATION_VALUE_INDEX    2      /**< @brief for bqb server notification, in database1 */

#define SDB_SER_NUM     3   /* 3 services in small database */
#define LDB1_SER_NUM    12  /* 12 services in large database1 */

/** @brief all services id in gatt bqb small databses */
typedef enum
{
    BQB_SERVICE_APP_DEFINED = 0,/**< @brief app defined */
    BQB_SERVICE_ID_GAP = 1,     /**< @brief Generic Access profile */
    BQB_SERVICE_ID_ATP = 2,     /**< @brief attributte profile */
    BQB_SERVICE_ID_A  =  3,     /**< @brief service A */
    BQB_SERVICE_ID_B1  = 4,     /**< @brief service B.1 */
    BQB_SERVICE_ID_B2  = 5,     /**< @brief service B.2 */
    BQB_SERVICE_ID_B3  = 6,     /**< @brief service B.3 */
    BQB_SERVICE_ID_B4  = 7,     /**< @brief service B.4 */
    BQB_SERVICE_ID_B5  = 8,     /**< @brief service B.5 */
    BQB_SERVICE_ID_C1  = 9,     /**< @brief service C.1 */
    BQB_SERVICE_ID_C2  = 10,    /**< @brief service C.2 */
    BQB_SERVICE_ID_D   = 11,    /**< @brief service D */
    BQB_SERVICE_ID_E   = 12,    /**< @brief service E */
    BQB_SERVICE_ID_F   = 13,    /**< @brief service F */
} BQB_GATT_ServiceId;

/* database index */
typedef enum
{
    SMALL_DATABASE  = 0,
    LARGE_DATABASE1 = 1,
    LARGE_DATABASE2 = 2,
    LARGE_DATABASE3 = 3
} BQB_DB_INDEX;

/* services */
typedef struct
{
    BQB_GATT_ServiceId ser_id;      /**< @brief service id in database */
    uint8_t num_attr;               /**< @brief number of attribute in a service */
} BQB_ServiceInfo;

/* globals*/
extern BQB_DB_INDEX BQB_GATT_Database_Index;
extern uint8_t BQB_ServiceNum;
extern BQB_ServiceInfo Service_Info[12];
extern uint8_t ChaValV6[1];   /* for bqb server send notification */
extern uint8_t ChaValV2_9[46];
extern uint8_t DesValV2D_9[45];	/* for gatt write */
extern uint8_t SecAUT14;

extern const TAttribAppl SmallDatabase[];
extern const TAttribAppl LargeDatabase1[];

/* export fucntions */
uint8_t BQB_SelectDatabase(BQB_DB_INDEX db_index);
uint8_t BQB_ServiceInfoInit(void);

#endif  /* _BQB_DATABASE_H_ */
