#include "rtl_types.h"
#include "trace.h"

/* Every test function has 100 instructions. */

__asm void nop_test(uint32_t loop_count)
{
nop_loop
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
#if TEST_CM4_SPEED_WITH_GPIO
    NOP
    NOP
    NOP
    NOP
#else
    /* BNE: 1 + P(branch) or 1 Cycle(No branch)P: The number of cycles required for a pipeline refill.
     * This ranges frome 1 to 3 depending on the alignment and width of the target instruction,
     * and whether the processor manages to speculate the address early. */
    SUBS R0, R0, #1 /* 1 cycle */
    BNE nop_loop /* 2~4 cycles, Use 3 to simplify */
#endif
    BX LR
}

__asm void movs_test(uint32_t loop_count)
{
    PUSH {R4-R7, LR}
movs_test_loop
    MOVS R1, #0x3E
    MOVS R2, #0x3D
    MOVS R3, #0x3C
    MOVS R4, #0x3B
    MOVS R5, #0x3A
    MOVS R6, #0x30
    MOVS R7, #0x39
    MOVS R1, #0x1E
    MOVS R2, #0x1D
    MOVS R3, #0x1C

    MOVS R4, #0x1B
    MOVS R5, #0x1A
    MOVS R6, #0x10
    MOVS R7, #0x19
    MOVS R1, #0x2E
    MOVS R2, #0x2D
    MOVS R3, #0x2C
    MOVS R4, #0x2B
    MOVS R5, #0x2A
    MOVS R6, #0x20

    MOVS R1, #0x3E
    MOVS R2, #0x3D
    MOVS R3, #0x3C
    MOVS R4, #0x3B
    MOVS R5, #0x3A
    MOVS R6, #0x30
    MOVS R7, #0x39
    MOVS R1, #0x1E
    MOVS R2, #0x1D
    MOVS R3, #0x1C

    MOVS R4, #0x1B
    MOVS R5, #0x1A
    MOVS R6, #0x10
    MOVS R7, #0x19
    MOVS R1, #0x2E
    MOVS R2, #0x2D
    MOVS R3, #0x2C
    MOVS R4, #0x2B
    MOVS R5, #0x2A
    MOVS R6, #0x20

    MOVS R1, #0x3E
    MOVS R2, #0x3D
    MOVS R3, #0x3C
    MOVS R4, #0x3B
    MOVS R5, #0x3A
    MOVS R6, #0x30
    MOVS R7, #0x39
    MOVS R1, #0x1E
    MOVS R2, #0x1D
    MOVS R3, #0x1C

    MOVS R4, #0x1B
    MOVS R5, #0x1A
    MOVS R6, #0x10
    MOVS R7, #0x19
    MOVS R1, #0x2E
    MOVS R2, #0x2D
    MOVS R3, #0x2C
    MOVS R4, #0x2B
    MOVS R5, #0x2A
    MOVS R6, #0x20

    MOVS R1, #0x3E
    MOVS R2, #0x3D
    MOVS R3, #0x3C
    MOVS R4, #0x3B
    MOVS R5, #0x3A
    MOVS R6, #0x30
    MOVS R7, #0x39
    MOVS R1, #0x1E
    MOVS R2, #0x1D
    MOVS R3, #0x1C

    MOVS R4, #0x1B
    MOVS R5, #0x1A
    MOVS R6, #0x10
    MOVS R7, #0x19
    MOVS R1, #0x2E
    MOVS R2, #0x2D
    MOVS R3, #0x2C
    MOVS R4, #0x2B
    MOVS R5, #0x2A
    MOVS R6, #0x20

    MOVS R1, #0x3E
    MOVS R2, #0x3D
    MOVS R3, #0x3C
    MOVS R4, #0x3B
    MOVS R5, #0x3A
    MOVS R6, #0x30
    MOVS R7, #0x39
    MOVS R1, #0x1E
    MOVS R2, #0x1D
    MOVS R3, #0x1C

    MOVS R4, #0x1B
    MOVS R5, #0x1A
    MOVS R6, #0x10
    MOVS R7, #0x19
    MOVS R1, #0x2E
    MOVS R2, #0x2D
#if TEST_CM4_SPEED_WITH_GPIO
    MOVS R3, #0x2C
    MOVS R4, #0x2B
    MOVS R5, #0x2A
    MOVS R6, #0x20
#else
    SUBS R0, R0, #1
    BNE  movs_test_loop
#endif
    POP {R4-R7, PC}
}

/* thumb2 MOVS */
#ifdef __CM4_REV
__asm void movs32_test(uint32_t loop_count)
{
    PUSH {R4-R7, LR}
movs32_test_loop
    MOVS R1, #0x324
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304

    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
    MOVS R7, #0x304
    MOVS R1, #0x304
    MOVS R2, #0x304
#if TEST_CM4_SPEED_WITH_GPIO
    MOVS R3, #0x304
    MOVS R4, #0x304
    MOVS R5, #0x304
    MOVS R6, #0x304
#else
    SUBS R0, R0, #1
    BNE  movs32_test_loop
#endif
    POP {R4-R7, PC}
}
#endif

volatile uint32_t g_var_for_cm4_test = 0x425;

/* use LDR */
__asm void read_sram_test(uint32_t loop_count)
{
    IMPORT g_var_for_cm4_test
    PUSH {R4-R7, LR}
    LDR R7, =g_var_for_cm4_test
read_sram_test_loop
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R6, [R7, #0]
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
#if TEST_CM4_SPEED_WITH_GPIO
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
#else
    /* 
     * LDR: 2 cycle, but: 
     * Neighboring load and store single instructions can pipeline their address and data phases.
     * This enables these instructions to complete in a single execution cycle.
     */
    SUBS R0, R0, #1
    BNE read_sram_test_loop
#endif
    POP {R4-R7, PC}
}

__asm void write_sram_test(uint32_t loop_count)
{
    IMPORT g_var_for_cm4_test
    PUSH {R4-R7, LR}
    LDR R7, =g_var_for_cm4_test
    MOVS R1, #0x25
    MOVS R2, #0x26
    MOVS R3, #0x27
    MOVS R4, #0x28
    MOVS R5, #0x29
    NOP /* remove compiler padding warning */
loop_write_sram_test
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]

    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
    STR R4, [R7, #0]
    STR R5, [R7, #0]
    STR R1, [R7, #0]
    STR R2, [R7, #0]
    STR R3, [R7, #0]
#if TEST_CM4_SPEED_WITH_GPIO
    STR R4, [R7, #0]
    STR R5, [R7, #0]
#else
    SUBS R0, R0, #1
    BNE loop_write_sram_test
#endif
    NOP
    POP {R4-R7, PC}
}

__asm void read_rom_test(uint32_t loop_count)
{
    PUSH {R4-R7, LR}
    LDR R7, =0x100
loop_read_rom
    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]

    LDR R1, [R7, #0]
    LDR R2, [R7, #0]
    LDR R3, [R7, #0]
    LDR R4, [R7, #0]
    LDR R5, [R7, #0]
    LDR R1, [R7, #4]
    LDR R2, [R7, #4]
    LDR R3, [R7, #4]
#if TEST_CM4_SPEED_WITH_GPIO
    LDR R4, [R7, #4]
    LDR R5, [R7, #4]
#else
    SUBS R0, R0, #1
    BNE  loop_read_rom
#endif
    NOP
    POP {R4-R7, PC}
}

void test_cm4_speed(void)
{
    DBG_DIRECT("NOP Test\n");
    nop_test(10000000);
    DBG_DIRECT("NOP Test Finish\n");

    DBG_DIRECT("MOVS Test\n");
    movs_test(10000000);
    DBG_DIRECT("MOVS Test Finish\n");

#ifdef __CM4_REV
    DBG_DIRECT("MOVS32 Test\n");
    movs32_test(10000000);
    DBG_DIRECT("MOVS32 Test Finish\n");
#endif

    DBG_DIRECT("Read SRAM Test\n");
    read_sram_test(10000000);
    DBG_DIRECT("Read SRAM Test Finish\n");

    DBG_DIRECT("Write SRAM Test\n");
    write_sram_test(10000000);
    DBG_DIRECT("Write SRAM Test Finish\n");

    DBG_DIRECT("Read ROM Test\n");
    read_rom_test(10000000);
    DBG_DIRECT("Read ROM Test Finish\n");
}
