enum { __FILE_NUM__ = 0 };

/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "app_dfu_int.h"
#include "app_dfu_act.h"
#include "app_dfu_api.h"
#include "app_dfu_handle_upper_stack.h"


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>


#include "blueapi.h"

#include "ota_api.h"
#include "efuse_config.h"
#include "otp.h"
#include "dlps_platform.h"
#include "aes.h"

#if UPPER_STACK_USE_VIRTUAL_HCI
#include "hal_platform.h"
#include "flash_ota.h"
#endif

#include "patch_upperstack.h"

#include <trace_binary.h>
#define TRACE_MODULE_ID     MID_BT_APPL


/****************************************************************************/
/* Events                                                                   */
/****************************************************************************/


#define DFU_EVENT_BLUEAPI_MSG         0x02


/****************************************************************************/
/* GattDemo interface                                                       */
/****************************************************************************/

#define DFU_APP_PRIORITY          (tskIDLE_PRIORITY + 1)   /* Task priorities. */
#define DFU_APP_STACK_SIZE        2048


#define DFU_MAX_NUMBER_OF_RX_EVENT     0x20
#define DFU_MAX_NUMBER_OF_MESSAGE      0x20



extern UINT8 bee_aon_gp_read(UINT8 offset);

extern void bee_aon_gp_write(UINT8 offset, UINT8 data);
extern void ota_fw_active_reset(void);


TDFU_CB *g_pDfuCb;

uint8_t *g_pOtaTempBufferHead;
uint16_t g_OtaTempBufferUsedSize;
#define OTA_TEMP_BUFFER_SIZE 20*100
aes_context*g_pAesCtx; 

void AES_Getey(unsigned char *secret_key)
{
#if 0
    unsigned char key[32] ={ 
                                                    0x4E, 0x46, 0xF8, 0xC5, 0x09, 0x2B, 0x29, 0xE2,
                                                    0x9A, 0x97, 0x1A, 0x0C, 0xD1, 0xF6, 0x10, 0xFB,
                                                    0x1F, 0x67, 0x63, 0xDF, 0x80, 0x7A, 0x7E, 0x70,
                                                    0x96, 0x0D, 0x4C, 0xD3, 0x11, 0x8E, 0x60, 0x1A
                                                    };
#endif
    

    memcpy(secret_key, otp_str_data.vender_key, 32);
}



void AES_Init( void )
{
    int n = otp_str_data.gEfuse_UpperStack_s.ota_with_encryption_aes_type;
    
    unsigned char key[32];
    g_pAesCtx = pvPortMalloc(sizeof(aes_context), RAM_TYPE_DATA_OFF);

    AES_Getey(key);
    aes_set_key( g_pAesCtx, key, 128 + n * 64);
}




#define DFU_SM_IGNORE  0
#define DFU_NUM_ACTIONS 1
#define DFU_SME_NEXT_STATE 1
#define DFU_SM_NUM_COLS 2

typedef const UINT8 (*TDFU_SM_TBL)[DFU_SM_NUM_COLS];

/**

enum
{
    DFU_ACT_INIT_UPPERSTACK,
    DFU_ACT_INIT_DFU_SERVICE,
    DFU_ACT_INIT_ENABLE_ADV,
    DFU_ACT_PROCESS_ADV_ENABLE_EVT,
    DFU_ACT_PROCESS_LINK_CONNECTED_EVT,
    DFU_ACT_PROCESS_NOTIFICATION_ENABLE_EVT,
    DFU_ACT_NOTIFY_RX_FW_CP_REQ,
    DFU_ACT_NOTIFY_FW_RX_CMPL,
    DFU_ACT_NOTIFY_VALID,
    DFU_ACT_RESET_AND_ACTIVATE,
    DFU_ACT_SYSTEM_RESET,
    DFU_SM_NO_ACTION
};


*/

const TDFU_ACT dfu_sm_action[] =
{
    dfu_act_process_evt_init_upperstack,
    dfu_act_process_evt_upperstack_active,
    dfu_act_process_evt_dfu_service_registered,
    dfu_act_process_evt_adv_enabled,
    dfu_act_process_evt_link_connected,
    dfu_act_process_evt_enable_notification,
    dfu_act_notify_rx_cp_req,
    dfu_act_notify_fw_rx_cmpl,
    dfu_act_notify_valid,
    dfu_act_reset_and_activate,
    dfu_act_system_reset

};


const UINT8 dfu_idle_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_ACT_INIT_UPPERSTACK,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_ADV_ENABLED      */                    {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_NOTIFICATION_ENABLED      */     {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_VALID_FW      */                          {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,  DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_SM_NO_ACTION,     DFU_ST_IDLE}

};

const UINT8 dfu_initializing_upperstack_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_ACT_INIT_DFU_SERVICE,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */        {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_ADV_ENABLED      */                       {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_NOTIFICATION_ENABLED      */       {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_VALID_FW      */                          {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_UPPERSTACK}

};

const UINT8 dfu_initializing_reg_dfu_service[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_ACT_INIT_ENABLE_ADV,  DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_ADV_ENABLED      */                    {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_NOTIFICATION_ENABLED      */       {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_VALID_FW      */                          {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_DFU_SERVICE},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON}

};

const UINT8 dfu_initializing_enable_adv[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_ADV_ENABLED      */                    {DFU_ACT_PROCESS_ADV_ENABLE_EVT,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_NOTIFICATION_ENABLED      */       {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_VALID_FW      */                          {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_SM_NO_ACTION,     DFU_ST_INITIALIZING_ENABLE_ADV}

};

const UINT8 dfu_wait4_conn_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_ADV_ENABLED      */                    {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_ACT_PROCESS_LINK_CONNECTED_EVT,     DFU_ST_CONNECTED},
    /* DFU_EVT_NOTIFICATION_ENABLED      */       {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_VALID_FW      */                          {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON}
};

const UINT8 dfu_connected_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_SM_NO_ACTION,     DFU_ST_WAIT4_CON},
    /* DFU_EVT_ADV_ENABLED      */                       {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_NOTIFICATION_ENABLED      */       {DFU_ACT_PROCESS_NOTIFICATION_ENABLE_EVT,     DFU_ST_WAIT_FW},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_VALID_FW      */                          {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_CONNECTED},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_ACT_INIT_ENABLE_ADV,     DFU_ST_INITIALIZING_ENABLE_ADV}
};

const UINT8 dfu_wait4_rx_fw_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_ADV_ENABLED      */                    {DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_NOTIFICATION_ENABLED      */    {DFU_ACT_PROCESS_NOTIFICATION_ENABLE_EVT,     DFU_ST_WAIT_FW},
    /* DFU_EVT_RX_FW_CP_REQ      */                  {DFU_ACT_NOTIFY_RX_FW_CP_REQ,     DFU_ST_WAIT_FW},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_ACT_NOTIFY_FW_RX_CMPL,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_VALID_FW      */                          {DFU_ACT_NOTIFY_VALID,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_ACT_INIT_ENABLE_ADV,     DFU_ST_INITIALIZING_ENABLE_ADV}

};


const UINT8 dfu_fw_rx_cmpl_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */         {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */{DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_ADV_ENABLED      */                    {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_LINK_CONNECTED      */              {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_NOTIFICATION_ENABLED      */    {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_RX_FW_CP_REQ      */                               {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_RX_FW_CMPL      */                      {DFU_SM_NO_ACTION,     DFU_ST_FW_RX_CMPL},
    /* DFU_EVT_VALID_FW      */                          {DFU_ACT_NOTIFY_VALID,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_ACTIVE_RESET      */                   {DFU_SM_NO_ACTION,     DFU_ST_WAIT_FW},
    /* DFU_EVT_SYSTEM_RESET      */                   {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */         {DFU_ACT_INIT_ENABLE_ADV,     DFU_ST_INITIALIZING_ENABLE_ADV}

};

const UINT8 dfu_fw_valid_table[][DFU_SM_NUM_COLS] =
{
    /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */        {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */	{DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_ADV_ENABLED      */              {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_LINK_CONNECTED      */           {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_NOTIFICATION_ENABLED      */    	{DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_RX_FW_CP_REQ      */             {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_RX_FW_CMPL      */               {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_VALID_FW      */                 {DFU_SM_NO_ACTION,     DFU_ST_FW_VALIDATE},
    /* DFU_EVT_ACTIVE_RESET      */             {DFU_ACT_RESET_AND_ACTIVATE,DFU_ST_IDLE},
    /* DFU_EVT_SYSTEM_RESET      */             {DFU_ACT_SYSTEM_RESET,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */        {DFU_ACT_INIT_ENABLE_ADV,  DFU_ST_INITIALIZING_ENABLE_ADV}
};

const UINT8 dfu_wait4_active_reset_table[][DFU_SM_NUM_COLS] =
{
//Lory IAR add
   /* Event                                                        Action                    Next state */

    /* DFU_EVT_INIT_REG_UPPERSTACK      */      {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_UPPERSTACK_ACTIVE      */        {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_DFU_SERVICE_REGISTERED      */	{DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_ADV_ENABLED      */              {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_CONNECTED      */           {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_NOTIFICATION_ENABLED      */    	{DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_RX_FW_CP_REQ      */             {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_RX_FW_CMPL      */               {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_VALID_FW      */                 {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_ACTIVE_RESET      */             {DFU_SM_NO_ACTION,		 DFU_ST_IDLE},
    /* DFU_EVT_SYSTEM_RESET      */             {DFU_SM_NO_ACTION,     DFU_ST_IDLE},
    /* DFU_EVT_LINK_DISCONNECTED      */        {DFU_SM_NO_ACTION,  DFU_ST_IDLE} 
};

const UINT8 dfu_wait4_system_reset_table[][DFU_SM_NUM_COLS] =
{
//Lory IAR add
   /* Event 								      Action					 Next state */

	/* DFU_EVT_INIT_REG_UPPERSTACK		*/		{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_UPPERSTACK_ACTIVE	  */		{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_DFU_SERVICE_REGISTERED	   */	{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_ADV_ENABLED		*/				{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_LINK_CONNECTED	   */			{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_NOTIFICATION_ENABLED 	 */ 	{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_RX_FW_CP_REQ 	 */ 			{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_RX_FW_CMPL	   */				{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_VALID_FW 	 */ 				{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_ACTIVE_RESET 	 */ 			{DFU_SM_NO_ACTION,		 DFU_ST_IDLE},
	/* DFU_EVT_SYSTEM_RESET 	 */ 			{DFU_SM_NO_ACTION,	   DFU_ST_IDLE},
	/* DFU_EVT_LINK_DISCONNECTED	  */		{DFU_SM_NO_ACTION,	DFU_ST_IDLE} 
};


/**
enum
{
    DFU_EVT_INIT_REG_UPPERSTACK = 0x00,
    DFU_EVT_UPPERSTACK_ACTIVE,
    DFU_EVT_DFU_SERVICE_REGISTERED,
    DFU_EVT_ADV_ENABLED,
    DFU_EVT_LINK_CONNECTED,
    DFU_EVT_NOTIFICATION_ENABLED,
    DFU_EVT_RX_FW_CP_REQ,
    DFU_EVT_RX_FW_CMPL,
    DFU_EVT_VALID_FW,
    DFU_EVT_ACTIVE_RESET,
    DFU_EVT_SYSTEM_RESET,
    DFU_EVT_LINK_DISCONNECTED,
    DFU_MAX_EVT
};

*/

//event
const TDFU_SM_TBL dfu_state_table[] =
{
    dfu_idle_table,
    dfu_initializing_upperstack_table,
    dfu_initializing_reg_dfu_service,
    dfu_initializing_enable_adv,
    dfu_wait4_conn_table,
    dfu_connected_table,
    dfu_wait4_rx_fw_table,
    dfu_fw_rx_cmpl_table,
    dfu_fw_valid_table,
    dfu_wait4_active_reset_table,
    dfu_wait4_system_reset_table,

};

typedef const UINT8 (*TDFU_ENTRY_TBL)[DFU_ST_MAX];




const char * dfu_get_state_name(TDFU_STATE dfu_state)
{
    static const char pDFU_ST_IDLE[]                TRACE_DATA =   "DFU_ST_IDLE";
    static const char pDFU_ST_INITIALIZING_UPPERSTACK[]                TRACE_DATA =  "DFU_ST_INITIALIZING_UPPERSTACK";
    static const char pDFU_ST_INITIALIZING_DFU_SERVICE[]                TRACE_DATA =  "DFU_ST_INITIALIZING_DFU_SERVICE";
    static const char pDFU_ST_INITIALIZING_ENABLE_ADV[]                TRACE_DATA =   "DFU_ST_INITIALIZING_ENABLE_ADV";
    static const char pDFU_ST_WAIT4_CON[]                TRACE_DATA =  "DFU_ST_WAIT4_CON";
    static const char pDFU_ST_CONNECTED[]                TRACE_DATA =  "DFU_ST_CONNECTED";
    static const char pDFU_ST_WAIT_FW[]                TRACE_DATA =   "DFU_ST_WAIT_FW";
    static const char pDFU_ST_FW_RX_CMPL[]                TRACE_DATA =  "DFU_ST_FW_RX_CMPL";
    static const char pDFU_ST_FW_VALIDATE[]                TRACE_DATA =   "DFU_ST_FW_VALIDATE";
    static const char pDFU_ST_WAIT_FOR_ACTIVE_RESET[]                TRACE_DATA =  "DFU_ST_WAIT_FOR_ACTIVE_RESET";
    static const char pDFU_ST_MAX[]                TRACE_DATA =  "DFU_ST_MAX";

    switch (dfu_state)
    {
    case DFU_ST_IDLE:
        return (LPSTR)pDFU_ST_IDLE;
    case DFU_ST_INITIALIZING_UPPERSTACK:
        return (LPSTR)pDFU_ST_INITIALIZING_UPPERSTACK;
    case DFU_ST_INITIALIZING_DFU_SERVICE:
        return (LPSTR)pDFU_ST_INITIALIZING_DFU_SERVICE;
    case DFU_ST_INITIALIZING_ENABLE_ADV:
        return (LPSTR)pDFU_ST_INITIALIZING_ENABLE_ADV;
    case DFU_ST_WAIT4_CON:
        return (LPSTR)pDFU_ST_WAIT4_CON;
    case DFU_ST_CONNECTED:
        return (LPSTR)pDFU_ST_CONNECTED;
    case DFU_ST_WAIT_FW:
        return (LPSTR)pDFU_ST_WAIT_FW;
    case DFU_ST_FW_RX_CMPL:
        return pDFU_ST_FW_RX_CMPL;
    case DFU_ST_FW_VALIDATE:
        return (LPSTR)pDFU_ST_FW_VALIDATE;
    case DFU_ST_WAIT_FOR_ACTIVE_RESET:
        return (LPSTR)pDFU_ST_WAIT_FOR_ACTIVE_RESET;
    default:
        return (LPSTR)pDFU_ST_MAX;
    }

}
const char * dfu_get_event_name(TDFU_EVENT dfu_evt)
{

    static const char pDFU_EVT_INIT_REG_UPPERSTACK[]                TRACE_DATA =   "DFU_EVT_INIT_REG_UPPERSTACK";
    static const char pDFU_EVT_UPPERSTACK_ACTIVE[]                TRACE_DATA =  "DFU_EVT_UPPERSTACK_ACTIVE";
    static const char pDFU_EVT_DFU_SERVICE_REGISTERED[]                TRACE_DATA =  "DFU_EVT_DFU_SERVICE_REGISTERED";
    static const char pDFU_EVT_ADV_ENABLED[]                TRACE_DATA =   "DFU_EVT_ADV_ENABLED";
    static const char pDFU_EVT_LINK_CONNECTED[]                TRACE_DATA =  "DFU_EVT_LINK_CONNECTED";
    static const char pDFU_EVT_NOTIFICATION_ENABLED[]                TRACE_DATA =  "DFU_EVT_NOTIFICATION_ENABLED";
    static const char pDFU_EVT_RX_FW_CP_REQ[]                TRACE_DATA =  "DFU_EVT_RX_FW_CP_REQ";
    static const char pDFU_EVT_RX_FW_CMPL[]                   TRACE_DATA =  "DFU_EVT_RX_FW_CMPL";
    static const char pDFU_EVT_VALID_FW[]                TRACE_DATA =   "DFU_EVT_VALID_FW";
    static const char pDFU_EVT_ACTIVE_RESET[]                TRACE_DATA =  "DFU_EVT_ACTIVE_RESET";
    static const char pDFU_EVT_SYSTEM_RESET[]                TRACE_DATA =  "DFU_EVT_SYSTEM_RESET";
    static const char pDFU_EVT_LINK_DISCONNECTED[]                TRACE_DATA =  "DFU_EVT_LINK_DISCONNECTED";
    static const char pDFU_MAX_EVT[]                TRACE_DATA =  "DFU_MAX_EVT";

    switch (dfu_evt)
    {
    case DFU_EVT_INIT_REG_UPPERSTACK:
        return (LPSTR)pDFU_EVT_INIT_REG_UPPERSTACK;
    case DFU_EVT_UPPERSTACK_ACTIVE:
        return (LPSTR)pDFU_EVT_UPPERSTACK_ACTIVE;
    case DFU_EVT_DFU_SERVICE_REGISTERED:
        return (LPSTR)pDFU_EVT_DFU_SERVICE_REGISTERED;
    case DFU_EVT_ADV_ENABLED:
        return (LPSTR)pDFU_EVT_ADV_ENABLED;
    case DFU_EVT_LINK_CONNECTED:
        return (LPSTR)pDFU_EVT_LINK_CONNECTED;
    case DFU_EVT_NOTIFICATION_ENABLED:
        return (LPSTR)pDFU_EVT_NOTIFICATION_ENABLED;
    case DFU_EVT_RX_FW_CP_REQ:
        return pDFU_EVT_RX_FW_CP_REQ;
    case DFU_EVT_RX_FW_CMPL:
        return pDFU_EVT_RX_FW_CMPL;
    case DFU_EVT_VALID_FW:
        return (LPSTR)pDFU_EVT_VALID_FW;
    case DFU_EVT_ACTIVE_RESET:
        return (LPSTR)pDFU_EVT_ACTIVE_RESET;
    case DFU_EVT_SYSTEM_RESET:
        return (LPSTR)pDFU_EVT_SYSTEM_RESET;
    case DFU_EVT_LINK_DISCONNECTED:
        return (LPSTR)pDFU_EVT_LINK_DISCONNECTED;
    default:
        return (LPSTR)pDFU_MAX_EVT;
    }

}

void dfu_set_state(TDFU_CB *pDfuCb, TDFU_STATE state)
{
    if (state < DFU_ST_MAX)
    {
        pDfuCb->state = state;
    }
    else
    {
        //error here
    }

}


TDFU_STATE dfu_get_state(TDFU_CB *pDfuCb)
{
    return pDfuCb->state;
}

void dfu_sm_event(TDFU_CB *pDfuCb, TDFU_EVENT event, void*pData)
{
    UINT8 cur_state = pDfuCb->state;
    TDFU_SM_TBL state_table;
    UINT8 action, i;

    APPL_TRACE_PRINTF_2(APPL_TRACE_MASK_TRACE, "dfu_sm_event: cur_state[%s], event[%s]",
                        dfu_get_state_name(cur_state),
                        dfu_get_event_name(event)
                       );


    if (cur_state >= DFU_ST_MAX)
    {
        return;
    }

    if ( event < DFU_MAX_EVT)
    {
        state_table = dfu_state_table[cur_state];
    }
    else
    {
        //error here
        return;
    }

    dfu_set_state(pDfuCb, state_table[event][DFU_SME_NEXT_STATE]);

    for ( i = 0; i < DFU_NUM_ACTIONS; i++ )
    {
        if ((action = state_table[event][i]) != DFU_SM_NO_ACTION)
        {
            (*dfu_sm_action[action])(pDfuCb, (TDFU_INIT_DATA*)pData);
        }
        else
        {
            break;
        }
    }

    if (cur_state != pDfuCb->state)
    {
        APPL_TRACE_PRINTF_3(APPL_TRACE_MASK_TRACE, "dfu state change: [%s] -> [%s] after evt [%s]",
                            dfu_get_state_name(cur_state),
                            dfu_get_state_name(pDfuCb->state),
                            dfu_get_event_name(event));
    }

}



/****************************************************************************/
/* BlueAPI Callback.                                                        */
/****************************************************************************/

void dfu_BlueAPICallback(PBlueAPI_UsMessage pMsg)
{
    unsigned char Event = DFU_EVENT_BLUEAPI_MSG;

    if (xQueueSend(g_pDfuCb->dfu_QueueHandleMessage, &pMsg, 0) == errQUEUE_FULL)
    {
        blueAPI_BufferRelease(pMsg);
    }
    else if (xQueueSend(g_pDfuCb->dfu_QueueHandleEvent, &Event, 0) == errQUEUE_FULL)     /* signal event  to DFU App task */
    {
        //error
    }
}

BOOL dfuGetOtaMode()
{
    UINT8 nRegBoot = {0};
    nRegBoot = bee_aon_gp_read(REG_AON_FW_ONREG_BOOT);
    APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_TRACE, "dfuGetOtaMode(%d)",nRegBoot);
    
    if(nRegBoot&BIT1)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void dfuSetOtaMode(BOOL bEnable)
{
    UINT8 nRegBoot = {0};
    nRegBoot = bee_aon_gp_read(REG_AON_FW_ONREG_BOOT);

    if(bEnable)
    {
        nRegBoot |= BIT1;
    }
    else
    {
        nRegBoot &= ~BIT1;
    }
    
    bee_aon_gp_write(REG_AON_FW_ONREG_BOOT, nRegBoot);
    nRegBoot = bee_aon_gp_read(REG_AON_FW_ONREG_BOOT);
    APPL_TRACE_PRINTF_1(APPL_TRACE_MASK_TRACE, "dfuSetOtaMode --nRegBoot(%d)",nRegBoot);


}



void dfuMonitorTimeoutHandler(xTimerHandle pxTimer)
 {	


    long lArrayIndex = 0;

    lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );
     
    DBG_DIRECT("dfuMonitorTimeoutHandler, TimerID(%d)", lArrayIndex);
    switch(lArrayIndex)
    {
        case TIMER_ID_DFU_TOTAL:
        case TIMER_ID_DFU_WAIT4_CON:
        case TIMER_ID_DFU_IMAGE_TRANSFER:
            ota_fw_active_reset();
            break;
    }
}

void dfu_timer_init(TDFU_CB *pDfuCb)
{
    if(pPatch_dfu_timer_init)
    {
        if(pPatch_dfu_timer_init(pDfuCb))
        {
            return;
        }
    }
    //this value should be read from efuse area
    pDfuCb->dfu_TimeOutValue_total = otp_str_data.gEfuse_UpperStack_s.ota_timeout_total*1000;
    pDfuCb->dfu_TimeOutValue_wait4_conn= otp_str_data.gEfuse_UpperStack_s.ota_timeout_wait4_conn*1000;
    pDfuCb->dfu_TimeOutValue_image_transfer= otp_str_data.gEfuse_UpperStack_s.ota_timeout_wait4_image_transfer*1000;


    pDfuCb->dfu_totalTimerHandle = xTimerCreate("dfuTotalTimer",
                                                                                pDfuCb->dfu_TimeOutValue_total / portTICK_PERIOD_MS,
                                                                                pdFALSE,
                                                                                (void*)1,
                                                                                dfuMonitorTimeoutHandler
                                                                                );
    //start total timer
    xTimerStart(pDfuCb->dfu_totalTimerHandle, 0);


    pDfuCb->dfu_TimeOutValue_wait4_conn = otp_str_data.gEfuse_UpperStack_s.ota_timeout_wait4_conn*1000;
    pDfuCb->dfu_wait4_connTimerHandle = xTimerCreate("dfuWait4ConTimer",
                                                                                pDfuCb->dfu_TimeOutValue_wait4_conn / portTICK_PERIOD_MS,
                                                                                pdFALSE,
                                                                                (void*)1,
                                                                                dfuMonitorTimeoutHandler
                                                                                );
    //start wait4 conn timer
    xTimerStart(pDfuCb->dfu_wait4_connTimerHandle, 0);

    pDfuCb->dfu_TimeOutValue_image_transfer= otp_str_data.gEfuse_UpperStack_s.ota_timeout_wait4_image_transfer *1000;
    pDfuCb->dfu_image_transferTimerHandle= xTimerCreate("dfuImageTransferTimer",
                                                                                pDfuCb->dfu_TimeOutValue_image_transfer / portTICK_PERIOD_MS,
                                                                                pdFALSE,
                                                                                (void*)1,
                                                                                dfuMonitorTimeoutHandler
                                                                                );

}

/****************************************************************************/
/* TASK                                                                     */
/****************************************************************************/

void dfu_Task(void *pParameters)
{
    char Event;
    TDFU_CB *pDfuCb = (TDFU_CB*)pParameters;

    pDfuCb->dfu_QueueHandleEvent   = xQueueCreate((DFU_MAX_NUMBER_OF_MESSAGE + DFU_MAX_NUMBER_OF_RX_EVENT),
                                     sizeof(unsigned char));
    pDfuCb->dfu_QueueHandleMessage = xQueueCreate(DFU_MAX_NUMBER_OF_MESSAGE,
                                     sizeof(PBlueAPI_UsMessage));


    
    dfu_timer_init(pDfuCb);

    dfu_API_Init(pDfuCb);
    
#ifdef CONFIG_DLPS_EN
    LPS_MODE_Pause();
#endif

    while (true)
    {
        if (xQueueReceive(pDfuCb->dfu_QueueHandleEvent, &Event, 1000/portTICK_RATE_MS) == pdPASS)
        {

            if (Event & DFU_EVENT_BLUEAPI_MSG)  /* BlueAPI */
            {
                PBlueAPI_UsMessage pMsg;

                while (xQueueReceive(pDfuCb->dfu_QueueHandleMessage, &pMsg, 0) == pdPASS)
                {
                    dfu_HandleBlueAPIMessage(pDfuCb, pMsg);
                }
            }
        }
    }
}

/****************************************************************************/
/* GATT Init                                                                */
/****************************************************************************/

void dfuInit( void )
{
    APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfuInit");

    if(dfuGetOtaMode())
    {
        g_pDfuCb = pvPortMalloc(sizeof(TDFU_CB), RAM_TYPE_DATA_ON);
        if (g_pDfuCb)
        {
            APPL_TRACE_PRINTF_0(APPL_TRACE_MASK_TRACE, "dfuInit OK");

            g_pDfuCb->state = DFU_ST_IDLE;
            g_pDfuCb->nTotalErrorCount = 0;

            g_pOtaTempBufferHead = pvPortMalloc(OTA_TEMP_BUFFER_SIZE, RAM_TYPE_DATA_OFF);
            g_OtaTempBufferUsedSize = 0;
            
            if(otp_str_data.gEfuse_UpperStack_s.ota_with_encryption_data == TRUE)
            {
                AES_Init();
            }

            xTaskCreate(dfu_Task, "DFU Task", otp_str_data.dfu_task_stack_size/*DFU_APP_STACK_SIZE / sizeof(portSTACK_TYPE)*/,
                        g_pDfuCb, otp_str_data.dfu_task_priority /*DFU_APP_PRIORITY*/, &g_pDfuCb->DfuAppTaskHandle);
        }   
    }
    else
    {
    }
}



