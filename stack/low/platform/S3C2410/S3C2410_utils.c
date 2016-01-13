/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file S3C2410_utils.c
 *  Implementation of general utilities for S3C2410 Samsung board.
 *
 * \author Santhosh kumar M
 * \date 2006-09-20
 */

#include "platform.h"
#include "logger.h"
#include "bt_fw_globals.h"
#include "bb_driver.h"
#include "gpio.h"
#include "le_ll.h"

#ifdef _SUPPORT_INFO_FROM_SYSTEM_ON_
#include "system_on.h"
extern UINT8 g_chip_id;
#endif

#ifdef _ROM_CODE_PATCHED_    
PF_ROM_CODE_PATCH_FUNC rcp_bt_led_control = NULL;
#endif

void pf_delay_us(UINT32 us)
{
#ifndef FOR_SIMULATION
    UINT32 index;
    UINT32 max = us * 10;
    for(index = 0x0; index < max; index++);
    return;
#endif    
}

void pf_delay(UINT32 ms)
{
#ifndef FOR_SIMULATION
    UINT32 index;
    UINT32 max = ms * 10000;
    for(index = 0x0; index < max; index++);
    return;
#endif    
}

UINT32 pf_get_rng_seed(void)
{
    /* this is LE random generator of HW */
    
    UINT32 output;
    WR_16BIT_IO(LE_REG_BASE, 0x84, 0x01);
    output = (RD_16BIT_IO(LE_REG_BASE, 0x88) << 16) |
              RD_16BIT_IO(LE_REG_BASE, 0x86);
    return output;
}

#ifdef MINICARD_BT_LED_CONTROL
UINT8 g_wpan_led_num = 1; /* 0 mean LED0, 1 mean LED1 */

void bt_led_control(UINT8 id, UINT8 level)
{    

#ifdef _ROM_CODE_PATCHED_    
    if (rcp_bt_led_control != NULL)
    {
        if ( rcp_bt_led_control((void*)&id, (void*)&level))
        {
             return;
        }
    }    
#endif



#ifdef _LED_8703B_BTON_CTRL
    UINT8 pos = id ? 9 : 8;
    UINT32 bit_mask = GEN_BIT_MASK(pos);

    gpio_ctrl_set_bton(bit_mask);
    gpio_ctrl_set_in_out(bit_mask, level == OUTPUT_HI_Z ? 0 : 0x1FFFFF);
    if(level < OUTPUT_HI_Z)
        gpio_ctrl_write_gpio(pos, level == OUTPUT_LOW ? 0 : 1);
    
#if defined (_FIX_8703B_LED_SHARE_WITH_WIFI_)&&defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
    if((g_chip_id == CHIP_ID_8703B)&& (level == OUTPUT_HI_Z))
    {
        bt_wifi_share_led_8703b_eco(id);
    }
#endif

#if 0
    UINT32 bit_mask; 
    UINT32 tmpData32 = 0x00;

    bit_mask = id ? (1 << 9) : (1 << 8); /* LED0 is BTGPIO8, LED1 is BTGPIO9 */
    SWITCH_BTON_GPIO_TO_ON_DOMAIN(bit_mask);

    if (level ==OUTPUT_HI_Z)
    {
        // LED disable
        tmpData32 =  VENDOR_READ(REG_BTON_REG_CTRL0) & (~ (bit_mask << 0x15));
        VENDOR_WRITE(REG_BTON_REG_CTRL0, tmpData32); //GPIO[8/9]_E Enable

#if defined (_FIX_8703B_LED_SHARE_WITH_WIFI_)&&defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
        if(g_chip_id == CHIP_ID_8703B)
        {
            bt_wifi_share_led_8703b_eco(id);
        }
#endif        
    }
    else
    {
        // LED enable, set gpio as output
        tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL0) | (bit_mask << 0x15));
        VENDOR_WRITE(REG_BTON_REG_CTRL0, tmpData32); //GPIO[8/9]_E Enable

        if (level == OUTPUT_HIGH)
        {
            tmpData32 = VENDOR_READ(REG_BTON_REG_CTRL1) | (bit_mask << 0x0A);            	
        }
        else
        {
            tmpData32 = VENDOR_READ(REG_BTON_REG_CTRL1) & (~(bit_mask << 0x0A));
        }

        VENDOR_WRITE(REG_BTON_REG_CTRL1, tmpData32); //GPIO[8/9]_E Enable
    }
#endif    
#else

#ifdef _LED_8703B_BTOFF_CTRL
    UINT32 bit_mask1;	

    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(BTON_GPIOMAP_08);
    DISABLE_OFF_DOMAIN_LOG_OUTPUT();

    bit_mask1 =  id ? (1 << 9) : (1 << 8);

    SWITCH_BTON_GPIO_TO_OFF_DOMAIN(bit_mask1);
    tmpData32 =  (VENDOR_READ(REG_BTON_REG_CTRL0) | (bit_mask1 << 0x15));
    VENDOR_WRITE(REG_BTON_REG_CTRL0, tmpData32); //GPIO[8/9]_E Enable
#endif

    UINT32 bm_gpio_en;
    UINT32 bm_gpio_level;

    bm_gpio_en = GPIO_READ(0x04);    
    bit_mask = id ? (1 << 5) : (1 << 4); /* LED 0 is DWGPIO 4, LED 1 is DWGPIO 5 */

    if (level == OUTPUT_HI_Z)
    {
        if (bm_gpio_en & bit_mask)
        {
            /* if it is output, we need to transfer input */
            bm_gpio_en &= ~bit_mask;
            GPIO_WRITE(0x04, bm_gpio_en);  
        }         
    }
    else
    {
        if (!(bm_gpio_en & bit_mask))
        {
            /* if it is input, we need to transfer output */
            bm_gpio_en |= bit_mask;
            GPIO_WRITE(0x04, bm_gpio_en);  
        }    
        
        bm_gpio_level = GPIO_READ(0x00);      
        
        if (level)
        {
            GPIO_WRITE(0x00, (bm_gpio_level | bit_mask));
        }
        else
        {
            GPIO_WRITE(0x00, (bm_gpio_level & ~bit_mask));
        } 
    }        
#endif
}

#if defined (_FIX_8703B_LED_SHARE_WITH_WIFI_)&&defined(_SUPPORT_INFO_FROM_SYSTEM_ON_)
void bt_wifi_share_led_8703b_eco(UINT8 id)
{
    // 8703b eco only
    // when disable LED, fw set led as input and set output value = HIGH
    BTON_GPIO_CTRL1_REG btgpio_ctrl1;
    //btgpio_ctrl1.d32=  VENDOR_READ(REG_BTON_REG_CTRL1);
    btgpio_ctrl1 = bton_gpio_ctrl1_reg_read();
    if(id == 1)
    {
        btgpio_ctrl1.gpio9 = 1;
    }
    else
    {
        btgpio_ctrl1.gpio8 = 1;
    }
    //VENDOR_WRITE(REG_BTON_REG_CTRL1, btgpio_ctrl1.d32);
    bton_gpio_ctrl1_reg_write((BTON_GPIO_CTRL1_REG)btgpio_ctrl1);

}
#endif
#endif

