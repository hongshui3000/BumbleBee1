/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi.c
* @brief     blueface register
* @details   
*
* @author   	gordon
* @date      	2015-06-26
* @version	v0.1
*/

#include <flags.h>
#include <btcommon.h>
#include <bluemgr.h>
#include <btman.h>
#include <blueapi_def.h>
#include <btsend.h>
#include <btglib.h>
#include <trace_binary.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BLUEFACE

extern PBTA pBTA;

/**
* @brief  blueface alloc and init app
* 
* @param npBTA:
*
* @return  allocated app
*
*/
LPbtApplication IblueFaceAllocateApp(PBTA npBTA)
{
    LPbtApplication app;

    app = &npBTA->app;
    if (!app->used)
    {
        app->bta            = npBTA;
        app->used           = TRUE;
        app->indicated      = FALSE;
        app->security       = FALSE;
        app->context        = NULL;
        return app;     /* return found app pointer */
    }

    return NULL; /* sorry, nothing found */
} /* blueFaceAllocateApp */

/**
* @brief  register blueface
* 
* @param pReg: register param
* @param size: register param size
*
* @return  register status
*
*/
blueFaceStatus lblueFaceRegister(LPblueFaceReg pReg)
{
    LPbtApplication app;

    app = IblueFaceAllocateApp(pBTA);
    if (app == NULL)
    {
        return blueFaceIllParameter;
    }

    BLUEFACE_TRACE_PRINTF_1(BLUEFACE_TRACE_MASK_TRACE,"BTMAN BT_REGISTER_REQ, registered %p",app);

    app->context = pReg->appContext;
    ((PBlueAPI_Data)app->context)->AppHandle = app;
    app->bta->secApp = app;
    app->security = TRUE; /* this app is EXLICIT security app */

    /* if the controller is already initialized, send welcome message to app */
    if (pBTA->localCause != 0xffff)
    {
        blueFaceSendBT_ACT_IND(pBTA->localBd, pBTA->localCause);
    }

    return blueFaceNoError;
} /* lblueFaceRegister */
