#ifndef _RLX4181_H_
#define _RLX4181_H_

#include "DataType.h"

typedef unsigned int CPU_SR;

/* The Data Structure of LX4081 CAUSE Register Definition for RTL0380 */
typedef struct MIPS_CAUSE_REG_S_ {
    UINT32 rsvd0:2;         /* bit[1:0], zeros */
    UINT32 exc_code:5;      /* bit[6:2], the excCode value */
    UINT32 rsvd1:3;         /* bit[9:7], software interrupt and zero */
    UINT32 bluewiz_int:1;   /* bit[10], the interrupt of bluewiz, IP[2] */
    UINT32 hci_dma_int:1;   /* bit[11], the interrupt of hci dma, IP[3] */
    UINT32 bzdma_int:1;     /* bit[12], the interrupt of bzdma, IP[4] */
    UINT32 gpio_int:1;      /* bit[13], the interrupt of gpio, IP[5] */
    UINT32 timer_int:1;     /* bit[14], the interrupt of hw timer, IP[6] */
    UINT32 uart_int:1;      /* bit[15], the interrupt of debug uart, IP[7] */
    UINT32 rsvd2:12;        /* bit[27:16], zeros */
    UINT32 cop_exception:2; /* bit[29:28], coprocessor exception */
    UINT32 rsvd3:1;         /* bit[30], zero */
    UINT32 branch_delay:1;  /* bit[31], indicate branch or jump delay slot */  
} MIPS_CAUSE_REG_S;

CPU_SR OSCPUSaveSR(void);
void OSCPURestoreSR(CPU_SR cpu_sr);

void Rlx4181EnableInterrupt(void);
UINT32 Rlx4181ReadCause(void);
UINT32 Rlx4181ReadEPC(void);
UINT32 Rlx4181ReadSP(void);

void rlx4081sleep(void);

#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
void EnterDlpsHandler(void);
#endif

#ifdef _ENABLE_BTON_POWER_SAVING_
void DisableIntForLPS(void);
#ifdef _ENABLE_RETENTION_FLOW_FOR_DLPS_
void DisableIntForDLPS(void);
#endif
void EnableIntForLPS(void);
#endif
void rlx4081NopDelay(void);

#ifdef _CP3_COUNTER_SUPPORTED_
void rlx4081Cp3CntInit(void);
UINT32 rlx4081Cp3StopCnt0(void);
UINT32 rlx4081Cp3StopCnt1(void);
UINT32 rlx4081Cp3StopCnt2(void);
UINT32 rlx4081Cp3StopCnt3(void);
#endif

#endif//_RLX4181_H_

