#ifndef BT_FW_CONFIG_H_
#define BT_FW_CONFIG_H_

#include "../../btcontroller/core/include/bt_fw_config.h"

#undef OS_MAX_HEAP_MEMORY
#undef OS_MAX_HEAP_DMEM_MEMORY
#undef OS_MAX_QUEUE_MEMORY
#undef OS_MAX_HEAP_MEMORY_PARTIAL_OFF
#undef OS_MAX_HEAP_DESC_PARTIAL_OFF

#define OS_MAX_HEAP_MEMORY              128
#define OS_MAX_HEAP_DMEM_MEMORY         128
#define OS_MAX_QUEUE_MEMORY             128
#define OS_MAX_HEAP_MEMORY_PARTIAL_OFF  128
#define OS_MAX_HEAP_DESC_PARTIAL_OFF    16

#endif
