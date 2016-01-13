/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 29 };
/********************************* Logger *************************/
/** 
 * \file bz_utils.c
 *  Generic BlueWiz utilities implementation.
 * 
 * \author Santhosh kumar M
 * \date 2006-05-06
 */

/** 
 * \addtogroup bz_utils Bluewiz Utilities
 * @{ */

/**
 * \addtogroup bz_generic_utils Generic Utilities
 *  A collection of assorted utility functions. 
 *  
 * @{ */

#include "bz_utils.h"
#include "bt_fw_types.h"
#include "lc.h"
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
#include "mint_os_timer_internal.h"
SECTION_SRAM OS_TIMER dbg_os_timer;
#endif

#define MAX_BT_CLOCK_VALUE  0x0FFFFFFF

/** 
 * Spins(busy wait) for the \a nticks duration.
 * 
 * \param nticks The number of bluetooth clock0 ticks to wait or spin.
 *
 * \return None.
 */
void bz_spin_for_nclk0_ticks(int nticks)
{
    UINT32 i;
    UINT32 clock;
    UINT32 time_to_wrap_around;

	lc_get_clock_in_scatternet(&clock, SCA_PICONET_FIRST);

	time_to_wrap_around = MAX_BT_CLOCK_VALUE - clock;

	if (time_to_wrap_around < nticks)       /* Is wrap-around possible during the entire duration of spin? */
	{
		nticks = nticks - time_to_wrap_around;

		i = clock;
		while (i < MAX_BT_CLOCK_VALUE)      /* Spin till clock becomes MAX_BT_CLOCK_VALUE */
		{
			lc_get_clock_in_scatternet(&i, SCA_PICONET_FIRST);
		}

		while (i != MAX_BT_CLOCK_VALUE)     /* Spin till i wraps around and becomes 0 */
		{
			lc_get_clock_in_scatternet(&i, SCA_PICONET_FIRST);
		}
	} /* End of if(wrap-around possible) */

	lc_get_clock_in_scatternet(&clock, SCA_PICONET_FIRST);

	i = clock;
	while (i < (clock+nticks))             /* Spin for the residual nticks */
	{
		lc_get_clock_in_scatternet(&i, SCA_PICONET_FIRST);
	}

	return;
}
#ifdef _FIX_CROSSBAR_ERROR_DEBUG_
void print_timer_info()
{
     RT_BT_LOG(RED, MSG_BT_TIMER_DBG, 10,
                dbg_os_timer.handle, 
                dbg_os_timer.cal_table_index,
                dbg_os_timer.timer_count,
                dbg_os_timer.timer_value,
                (UINT32)dbg_os_timer.timeout_function,
                dbg_os_timer.timer_type,
                (UINT32)dbg_os_timer.args,
                (UINT32)dbg_os_timer.next,
                dbg_os_timer.state,
                dbg_os_timer.sniff_tick_timer);
}
#endif

/**  @} end: bz_generic_utils */
/**  @} end: bz_utils */

