#ifndef _TIMER_H_
#define _TIMER_H_

#include "DataType.h"

#define TIMER_BASE_ADDR           0x40002000
#define TIMER1_LOAD_COUNT_OFF     0x00
#define TIMER1_CURRENT_VAL_OFF    0x04
#define TIMER1_CTL_REG_OFF        0x08
#define TIMER1_EOI_OFF            0x0c
#define TIMER1_INT_STATUS_OFF     0x10
#define TIMERS_INT_STATUS_OFF     0xa0
#define TIMERS_EOI_OFF            0xa4
#define TIMERS_RAW_INT_STATUS_OFF 0xa8
#define TIMERS_COMP_VER_OFF       0xac

#define TIMER_READ(y)    (*((volatile UINT32*)(TIMER_BASE_ADDR + y)))
#define TIMER_WRITE(y, z)  ((*((volatile UINT32*)(TIMER_BASE_ADDR + y))) = z)

extern UINT32 cpu_clk;

#define KHZ                       1000
#define MHZ                       1000000
#define TIMER_TICK_COUNT_PER_US   (cpu_clk / MHZ)
#define TIMER_TICK_COUNT_PER_MS   (cpu_clk / KHZ)

enum TIMERS_ID {
    TIMER1_ID = 0,
    TIMER2_ID,
    TIMER3_ID,
    TIMER4_ID
};

typedef union TIMER_ISR_BITMAP_S{
    UINT8 d8;

    struct{
        UINT8 timer1_en                 :1;
        UINT8 timer2_en                 :1;
        UINT8 timer3_en                 :1;
        UINT8 timer4_en                 :1;
        UINT8 reserved_4_7              :4;
    }b;
}TIMER_ISR_BITMAP_S_TYPE;

typedef void (*TIMER_HANDLE)(void);

SECTION_ISR void   TimerIntrHandler(void);
inline void reset_vendor_counter(void);
inline UINT32 read_vendor_counter(UINT8 index);
inline UINT32 read_vendor_counter_no_display(void);
void timer1_handle(void);
void timer2_handle(void);
void timer3_handle(void);
#ifdef _NEW_8812B_CPU_PLATFORM_    
void timer4_handle(void);
#endif

#if ((defined _DAPE_NO_TRX_WHEN_LE_SEND_CONN_REQ_HW_TIMER) || \
	(defined _DAPE_NO_TRX_WHEN_LE_CONN_UPDT))
void turn_on_timer3_for_le(UINT16 conn_win_size_ofst, UINT16 conn_win_ofst);
#endif

#ifndef _NEW_8812B_CPU_PLATFORM_
#define HW_TIMER_NUM            3
#elif defined(IS_BTSOC)
#define HW_TIMER_NUM            2
#else
#define HW_TIMER_NUM            4
#endif

#define RESET_VCOUNTER			reset_vendor_counter()
#define READ_VCOUNTER(idx)		read_vendor_counter(idx)

/*======================= Watchdog Timer Set ===============================*/

/* The Structure of WDG_REG_TIMER_SETTING Register (Vendor reg offset 0x78) */
typedef struct WDG_REG_TIMER_SETTING_S_ {
    UINT32 div_factor:16; /* bit[15:0], (1~ 0xffff): counting freq = 
                                        sys_clk freq / div factor */
    UINT32 enable:8;      /* bit[23:16], set 0xa5 to enable timer */
    UINT32 reset:1;       /* bit[24], reset timer when write 1 (auto clear) */
    UINT32 timeout_sel:3; /* bit[27:25], timeout selection: 0~7 select value as 
                                         2^13, 2^14, . . . , 2^20 */
    UINT32 rsvd:4;        /* bit[31:28], reserved */      
} WDG_REG_TIMER_SETTING_S, *PWDG_REG_TIMER_SETTING_S;

/* set div factor = 20000, counting freq = 2000, */
#define SET_WDG_DIV_FACTOR			20000
#define SET_WDG_TIMER_INIT(n)       (SET_WDG_DIV_FACTOR | (((n) & 0x07) << 25))
#define SET_WDG_TIMER_ENABLE(n)     (SET_WDG_TIMER_INIT(n) | (0x1A5 << 16))

#define SET_WDG_TIMER_INIT_IN_40M(msec)   (((msec) << 2) + (msec))
#define SET_WDG_TIMER_INIT_IN_32K(sec)    ((sec) << 2)

/* set timeout select = 0, means 8096, so watchdog timeout is about 4 sec, 
   After we change the n value, the timeout value can be 4 * 2^(n) sec */
#ifdef _USE_WATCHDOG_TIMER_
#define WDG_TIMER_ENABLE            VENDOR_WRITE(0x78, SET_WDG_TIMER_ENABLE(0)) 
#define WDG_TIMER_DISABLE           VENDOR_WRITE(0x78, SET_WDG_TIMER_INIT(0))
#define WDG_TIMER_RESTART           WDG_TIMER_ENABLE

/* for 40MHz, watchdog timer will be expired if do not restart it during 202us.
   for 32KHz, watchdog timer will be expired if do not restart it during 253ms*/
#define WDG_TIMER_TIMEOUT_SOON      VENDOR_WRITE(0x78, ((0x1A5 << 16) | 0x01))

#else
#define WDG_TIMER_ENABLE 
#define WDG_TIMER_DISABLE
#define WDG_TIMER_RESTART
#define WDG_TIMER_TIMEOUT_SOON
#endif

/*=================== BTON Watchdog Timer Set ==========================*/

/* The Structure of BTON_WDG_REG_TIMER_SETTING Register 
   (Vendor reg offset 0xB8) */
typedef struct BTON_WDG_REG_TIMER_SETTING_S_ {
    UINT16 divsel:3;      /* bit[2:0], select dividing factor, watchdog imer
                            is count with (32K clock/dividing_factor) 
                            dividing factor = 2^(1+divsel) */
    UINT16 clear:1;       /* bit[3], write 1 to clear timer, HW auto reset this
                             bit */
    UINT16 oversel:3;     /* bit[6:4], selected count limit
                             count_limit = (0x200 << oversel) - 1 */
    UINT16 test_mode:1;   /* bit[7], watchdog timer test mode. speed up for
                             simulation: dividing factor = 2,count_limit=0xf */
    UINT16 wdt_en_byte:8; /* bit[16:8], Set 0xA5 to enable watchdog timer */      
} BTON_WDG_REG_TIMER_SETTING_S, *PBTON_WDG_REG_TIMER_SETTING_S;

/* set divsel = 7, dividing factor = 32K/256 = 125 Hz */
#define SET_BTON_WDG_DIV_FACTOR	    7
#define SET_BTON_WDG_TIMER_INIT(n)     (SET_BTON_WDG_DIV_FACTOR | (((n) & 0x07) << 4))

/* n = 0, timeout value is about 4 sec,
   n = k, timeout value is about 4 * 2^(k) sec */
#define SET_BTON_WDG_TIMER_ENABLE(n)   (SET_BTON_WDG_TIMER_INIT(n) | (0xA508))

#ifdef _USE_BTON_WATCHDOG_TIMER_
#define BTON_WDG_TIMER_ENABLE_            VENDOR_WRITE(0xB8, SET_BTON_WDG_TIMER_ENABLE(4)) 
#define BTON_WDG_TIMER_DISABLE_           VENDOR_WRITE(0xB8, SET_BTON_WDG_TIMER_INIT(4))

/* BTON watchdog timer will be expired if do not restart it during 1ms in test mode */
//#define BTON_WDG_TIMER_TIMEOUT_SOON_      VENDOR_WRITE(0xB8, 0xA588)

/* BTON watchdog timer will be expired if do not restart it during 1 sec */
#define BTON_WDG_TIMER_TIMEOUT_SOON_      VENDOR_WRITE(0xB8, 0xA50D)

#define BTON_WDG_TIMER_ENABLE         do {if (IS_USE_BTIN_WDG_TIMER) {BTON_WDG_TIMER_ENABLE_;}} while(0)
#define BTON_WDG_TIMER_DISABLE        do {if (IS_USE_BTIN_WDG_TIMER) {BTON_WDG_TIMER_DISABLE_;}} while(0)
#define BTON_WDG_TIMER_RESTART        BTON_WDG_TIMER_ENABLE
#define BTON_WDG_TIMER_TIMEOUT_SOON   do {if (IS_USE_BTIN_WDG_TIMER) {BTON_WDG_TIMER_TIMEOUT_SOON_;}} while(0)
#else
#define BTON_WDG_TIMER_ENABLE 
#define BTON_WDG_TIMER_DISABLE
#define BTON_WDG_TIMER_RESTART
#define BTON_WDG_TIMER_TIMEOUT_SOON
#endif

/*==========================================================================*/

#endif

