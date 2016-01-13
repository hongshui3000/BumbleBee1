#ifndef _INITIAL_H_
#define _INITIAL_H_

#define VENDOR_00_SYS_CLK                               0x03

#ifndef _BZ_PARTIAL_ON_RELEASE_RSTN_
#define VENDOR_00_SYS_BZ_PARTIAL_ON_RSTN_BIT        (0 << 6)
#else
#define VENDOR_00_SYS_BZ_PARTIAL_ON_RSTN_BIT        (1 << 6) /* mean no reset */
#endif

#define VENDOR_00_VAL0    (0x33c | VENDOR_00_SYS_CLK | VENDOR_00_SYS_BZ_PARTIAL_ON_RSTN_BIT)
#define VENDOR_00_VAL1    (0x32c | VENDOR_00_SYS_CLK | VENDOR_00_SYS_BZ_PARTIAL_ON_RSTN_BIT)

#endif // _INITIAL_H_

