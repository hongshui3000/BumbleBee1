#include <stdio.h>
#include <bt_fw_config.h>
#include <mint_os_mem_internal.h>
#include <mint_os.h>
#include <unit_test.h>


extern UINT8 heap_buf_on[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_MEMORY)] ALIGN(MEM_BLOCK_ALIGNMENT) SECTION_HEAP;
extern UINT8 heap_data_on[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_DMEM_MEMORY)] ALIGN(MEM_BLOCK_ALIGNMENT);

extern MEM_BLOCK_OFF_DESC heap_buf_off_desc[OS_MAX_HEAP_DESC_PARTIAL_OFF] ALIGN(MEM_BLOCK_ALIGNMENT);
extern UINT8 heap_buf_off[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_MEMORY_PARTIAL_OFF)] ALIGN(MEM_BLOCK_ALIGNMENT) SECTION_LOW_HEAP;

static MEM_BLOCK *mem_on_get_block(void *buf)
{
    return (MEM_BLOCK *) ((UCHAR *) buf - MEM_BLOCK_HEADER_SIZE);
}

static size_t mem_on_get_block_size(MEM_BLOCK *blk)
{
    assert_true(blk->size & MEM_BLOCK_ALLOC_BIT);
    return (blk->size & ~MEM_BLOCK_ALLOC_BIT);
}

static size_t mem_on_get_num_free_blocks(MEM_ON_MANAGER *mgr)
{
    size_t num = 0;
    MEM_BLOCK *blk = mgr->free_head;
    while (blk != NULL)
    {
        ++num;
        blk = blk->next;
    }
    return num;
}

static void mem_on_validate_free_bytes(MEM_ON_MANAGER *mgr)
{
    size_t total_size = 0;
    MEM_BLOCK *blk = mgr->free_head;
    while (blk != NULL)
    {
        total_size += blk->size;
        blk = blk->next;
    }
    assert_int_equal(total_size, mgr->free_bytes);
}

static MEM_BLOCK *mem_on_get_free_block(int index, MEM_ON_MANAGER *mgr)
{
    MEM_BLOCK *blk;
    int i;
    for (blk = mgr->free_head, i = 0; blk != NULL;
            blk = blk->next, ++i)
    {
        if (i == index)
        {
            break;
        }
    }
    return blk;
}

static MEM_BLOCK_OFF_DESC *mem_off_get_desc(void *buf, MEM_OFF_MANAGER *mgr)
{
    int ind;
    BOOLEAN ret = mem_off_manager_bsearch(mgr, buf, mgr->used_beg,
            mgr->desc_num, &ind, 0);
    assert_true(ret);
    return &mgr->desc[ind];
}

static size_t mem_off_get_block_size(MEM_BLOCK_OFF_DESC *desc)
{
    assert_true(desc->size & MEM_BLOCK_ALLOC_BIT);
    return (desc->size & ~MEM_BLOCK_ALLOC_BIT);
}

static MEM_BLOCK_OFF_DESC *mem_off_get_free_desc(int index,
        MEM_OFF_MANAGER *mgr)
{
    assert_in_range(index, 0, mgr->free_end - 1);
    return &mgr->desc[index];
}

static size_t mem_off_get_num_free_blocks(MEM_OFF_MANAGER *mgr)
{
    return mgr->free_end;
}

static void mem_off_validate_free_bytes(MEM_OFF_MANAGER *mgr)
{
    size_t total_size = 0;
    size_t i;
    for (i = 0; i < mgr->free_end; ++i)
    {
        total_size += mgr->desc[i].size;
    }
    assert_int_equal(total_size, mgr->free_bytes);
}

static void test_mem_on_alloc(void **state)
{
    os_heap_init();

    /*
     * Test case: No free buffer can allocate
     */
    UCHAR *buf = os_malloc(OS_MAX_HEAP_MEMORY - MEM_BLOCK_HEADER_SIZE,
            RAM_TYPE_BUFFER_ON);
    assert_ptr_equal(buf, heap_buf_on + MEM_BLOCK_HEADER_SIZE);
    assert_null(mem_mgr_data_on.free_head);
    assert_int_equal(mem_mgr_data_on.free_bytes, 0);

    UCHAR *buf2 = os_malloc(4, RAM_TYPE_BUFFER_ON);
    assert_null(buf2);
    assert_null(mem_mgr_data_on.free_head);
    assert_int_equal(mem_mgr_data_on.free_bytes, 0);

    /*
     * Test case: Free block head is correctly set
     */
    os_free(buf, RAM_TYPE_BUFFER_ON);
    assert_ptr_equal(mem_mgr_data_on.free_head, buf - MEM_BLOCK_HEADER_SIZE);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
    assert_int_equal(mem_mgr_data_on.free_bytes, OS_MAX_HEAP_MEMORY);

    /*
     * Test case: Unaligned size allocation
     */
    int i;
    for (i = 1; i < MEM_BLOCK_ALIGNMENT; ++i)
    {
        buf2 = os_malloc(i, RAM_TYPE_BUFFER_ON);
        assert_ptr_equal(buf2, heap_buf_on + (i - 1) * (MEM_BLOCK_HEADER_SIZE + MEM_BLOCK_ALIGNMENT) + MEM_BLOCK_HEADER_SIZE);
        MEM_BLOCK *blk = mem_on_get_block(buf2);
        assert_int_equal(mem_on_get_block_size(blk), MEM_BLOCK_ALIGNMENT + MEM_BLOCK_HEADER_SIZE);
    }
}

static void test_mem_on_split(void **state)
{
    STATIC_ASSERT(OS_MAX_HEAP_MEMORY % (MEM_BLOCK_MIN_SIZE - 4) != 0, "");
    STATIC_ASSERT(OS_MAX_HEAP_MEMORY / (MEM_BLOCK_MIN_SIZE - 4) > 4, "");

    const int S = MEM_BLOCK_MIN_SIZE - 4;
    const int N = OS_MAX_HEAP_MEMORY / S;
    UCHAR *buf[N];
    os_heap_init();

    /*
     * Test case: Split free blocks for allocation
     */
    int i;
    for (i = 0; i < N; ++i)
    {
        assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
        buf[i] = os_malloc(S - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_BUFFER_ON);
        assert_ptr_equal(buf[i], heap_buf_on + i * S + MEM_BLOCK_HEADER_SIZE);
        mem_on_validate_free_bytes(&mem_mgr_data_on);
    }
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 0);
    assert_int_equal(mem_on_get_block_size(mem_on_get_block(buf[N - 1])), OS_MAX_HEAP_MEMORY - S * (N - 1));

    UCHAR *buf2 = os_malloc(S - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_BUFFER_ON);
    assert_null(buf2);

    os_free(buf[1], RAM_TYPE_BUFFER_ON);
    os_free(buf[3], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 2);
    mem_on_validate_free_bytes(&mem_mgr_data_on);

    /*
     * Test case: Cannot allocate due to fragmentation
     */
    buf2 = os_malloc(S - MEM_BLOCK_HEADER_SIZE + 4, RAM_TYPE_BUFFER_ON);
    assert_null(buf2);
}

static void test_mem_on_split2(void **state)
{
    STATIC_ASSERT(OS_MAX_HEAP_MEMORY % MEM_BLOCK_MIN_SIZE == 0,
            "OS_MAX_HEAP_MEMORY should be multiples of MEM_BLOCK_MIN_SIZE");

    int i;
    const int N = OS_MAX_HEAP_MEMORY / MEM_BLOCK_MIN_SIZE;
    UCHAR *buf[N];
    os_heap_init();

    /*
     * Test case: Minimal block size
     */
    for (i = 0; i < N; ++i)
    {
        assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
        buf[i] = os_malloc(MEM_BLOCK_MIN_SIZE - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_BUFFER_ON);
        assert_ptr_equal(buf[i], heap_buf_on + i * MEM_BLOCK_MIN_SIZE + MEM_BLOCK_HEADER_SIZE);
        mem_on_validate_free_bytes(&mem_mgr_data_on);
    }
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 0);
    assert_int_equal(mem_on_get_block_size(mem_on_get_block(buf[N - 1])), MEM_BLOCK_MIN_SIZE);

    /*
     * Test case: Don't split a free block if it's not large enough
     */
    os_free(buf[0], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
    mem_on_validate_free_bytes(&mem_mgr_data_on);
    buf[0] = os_malloc((MEM_BLOCK_MIN_SIZE - MEM_BLOCK_HEADER_SIZE) / 2, RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_block_size(mem_on_get_block(buf[0])), MEM_BLOCK_MIN_SIZE);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 0);
    mem_on_validate_free_bytes(&mem_mgr_data_on);
}

static void test_mem_on_merge(void **state)
{
    const int S = ((OS_MAX_HEAP_MEMORY / 5) & -MEM_BLOCK_ALIGNMENT);
    UCHAR *buf[5];
    int i;

    os_heap_init();
    for (i = 0; i < 5; ++i)
    {
        buf[i] = os_malloc(S - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_BUFFER_ON);
    }
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 0);

    /*
     * Test case: Free block not merged (insert at head)
     */
    os_free(buf[1], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_data_on)->size, S);
    mem_on_validate_free_bytes(&mem_mgr_data_on);

    /*
     * Test case: Free block merged with the next block
     */
    os_free(buf[0], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_data_on)->size, 2 * S);
    mem_on_validate_free_bytes(&mem_mgr_data_on);

    /*
     * Test case: Free block merged with the previous block
     */
    os_free(buf[2], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_data_on)->size, 3 * S);
    mem_on_validate_free_bytes(&mem_mgr_data_on);

    /*
     * Test case: Free block not merged
     */
    os_free(buf[4], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 2);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_data_on)->size, 3 * S);
    assert_int_equal(mem_on_get_free_block(1, &mem_mgr_data_on)->size, OS_MAX_HEAP_MEMORY - 4 * S);
    mem_on_validate_free_bytes(&mem_mgr_data_on);

    /*
     * Test case: Free block merged with the next and previous blocks
     */
    os_free(buf[3], RAM_TYPE_BUFFER_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_data_on), 1);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_data_on)->size, OS_MAX_HEAP_MEMORY);
    mem_on_validate_free_bytes(&mem_mgr_data_on);
}

#define OS_MAX_HEAP_DMEM_MEMORY2    16
UINT8 heap_dmem2[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_DMEM_MEMORY2)] ALIGN(MEM_BLOCK_ALIGNMENT);
void mem_on_manager_extend(MEM_ON_MANAGER *mgr, UINT8 *heap, size_t heap_size);
static void test_mem_on_extend(void **state)
{
    UINT8 *heap[2];
    size_t heap_size[2];

    os_heap_init();
    mem_on_manager_extend(&mem_mgr_buf_on, heap_dmem2, sizeof (heap_dmem2));
    {
        int i1, i2;
        if (heap_data_on < heap_dmem2)
        {
            i1 = 0;
            i2 = 1;
        }
        else
        {
            i1 = 1;
            i2 = 0;
        }
        heap[i1] = heap_data_on;
        heap[i2] = heap_dmem2;
        heap_size[i1] = OS_MAX_HEAP_DMEM_MEMORY;
        heap_size[i2] = OS_MAX_HEAP_DMEM_MEMORY2;
    }
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_buf_on), 2);
    assert_ptr_equal(mem_on_get_free_block(0, &mem_mgr_buf_on), heap[0]);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_buf_on)->size, heap_size[0]);
    assert_ptr_equal(mem_on_get_free_block(1, &mem_mgr_buf_on), heap[1]);
    assert_int_equal(mem_on_get_free_block(1, &mem_mgr_buf_on)->size, heap_size[1]);

    UCHAR *buf[2];
    buf[0] = os_malloc(heap_size[0] - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_DATA_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_buf_on), 1);
    assert_ptr_equal(mem_on_get_free_block(0, &mem_mgr_buf_on), heap[1]);
    assert_null(mem_on_get_free_block(0, &mem_mgr_buf_on)->next);
    mem_on_validate_free_bytes(&mem_mgr_buf_on);

    buf[1] = os_malloc(heap_size[1] - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_DATA_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_buf_on), 0);
    mem_on_validate_free_bytes(&mem_mgr_buf_on);

    os_free(buf[0], RAM_TYPE_DATA_ON);
    os_free(buf[1], RAM_TYPE_DATA_ON);
    assert_int_equal(mem_on_get_num_free_blocks(&mem_mgr_buf_on), 2);
    assert_ptr_equal(mem_on_get_free_block(0, &mem_mgr_buf_on), heap[0]);
    assert_int_equal(mem_on_get_free_block(0, &mem_mgr_buf_on)->size, heap_size[0]);
    assert_ptr_equal(mem_on_get_free_block(1, &mem_mgr_buf_on), heap[1]);
    assert_int_equal(mem_on_get_free_block(1, &mem_mgr_buf_on)->size, heap_size[1]);
    mem_on_validate_free_bytes(&mem_mgr_buf_on);
}

static void test_mem_off_bsearch(void **state)
{
    MEM_BLOCK_OFF_DESC desc_buf[5];
    MEM_OFF_MANAGER mgr;
    int i;

    mem_off_manager_init(&mgr, 4, 20, desc_buf, ARRAY_SIZE(desc_buf));
    mgr.free_end = 0;
    mgr.used_beg = 0;
    mgr.free_bytes = 0;
    for (i = 0; i < 5; ++i)
    {
        mgr.desc[i].addr = (void *)((i + 1) * 4);
        mgr.desc[i].size = 4;
    }

    /*
     * Test case: Find existent address
     */
    {
        UCHAR move_right;
        for (move_right = 0; move_right < 2; ++move_right)
        {
            for (i = 0; i < 5; ++i)
            {
                int ind;
                BOOLEAN ret = mem_off_manager_bsearch(&mgr, (i + 1) * 4,
                        mgr.used_beg, mgr.desc_num, &ind, move_right);
                assert_true(ret);
                assert_int_equal(ind, i);
            }
        }
    }

    /*
     * Test case: Find inexistent address
     */
    {
        /* 4 8 12 16 20 */
        int addr[6] = {
            2, 6, 10, 14, 18, 24
        };
        int index[2][6] = {
            { -1, 0, 1, 2, 3, 4 },
            { 0, 1, 2, 3, 4, 5 }
        };
        UCHAR move_right;
        for (move_right = 0; move_right < 2; ++move_right)
        {
            for (i = 0; i < 6; ++i)
            {
                int ind;
                BOOLEAN ret = mem_off_manager_bsearch(&mgr, addr[i],
                        mgr.used_beg, mgr.desc_num, &ind, move_right);
                assert_false(ret);
                assert_int_equal(ind, index[move_right][i]);
            }
        }
    }
}

static void test_mem_off_alloc(void **state)
{
    os_heap_init();

    /*
     * Test case: No free buffer can allocate
     */
    UCHAR *buf = os_malloc(OS_MAX_HEAP_MEMORY_PARTIAL_OFF, RAM_TYPE_BUFFER_OFF);
    assert_ptr_equal(buf, heap_buf_off);
    assert_int_equal(mem_mgr_buf_off.free_end, 0);
    assert_int_equal(mem_mgr_buf_off.free_bytes, 0);

    UCHAR *buf2 = os_malloc(4, RAM_TYPE_BUFFER_OFF);
    assert_null(buf2);
    assert_int_equal(mem_mgr_buf_off.free_end, 0);
    assert_int_equal(mem_mgr_buf_off.free_bytes, 0);

    /*
     * Test case: Free block head is correctly set
     */
    os_free(buf, RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_ptr_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->addr, buf);
    assert_ptr_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->addr, heap_buf_off);
    assert_int_equal(mem_mgr_buf_off.free_bytes, OS_MAX_HEAP_MEMORY_PARTIAL_OFF);

    /*
     * Test case: Unaligned size allocation
     */
    int i;
    for (i = 1; i < MEM_BLOCK_ALIGNMENT; ++i)
    {
        buf2 = os_malloc(i, RAM_TYPE_BUFFER_OFF);
        assert_ptr_equal(buf2, heap_buf_off + (i - 1) * MEM_BLOCK_ALIGNMENT);
        MEM_BLOCK_OFF_DESC *desc = mem_off_get_desc(buf2, &mem_mgr_buf_off);
        assert_int_equal(mem_off_get_block_size(desc), MEM_BLOCK_ALIGNMENT);
    }
}

static void test_mem_off_alloc2(void **state)
{
    const int S = ((OS_MAX_HEAP_MEMORY_PARTIAL_OFF / OS_MAX_HEAP_DESC_PARTIAL_OFF) & -MEM_BLOCK_ALIGNMENT) - 4;
    assert_true(S > 0);
    int i;
    os_heap_init();

    /*
     * Test case: No free empty descriptor to allocate (free space is available)
     */
    UCHAR *buf;
    for (i = 0; i < OS_MAX_HEAP_DESC_PARTIAL_OFF - 1; ++i)
    {
        assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
        buf = os_malloc(S, RAM_TYPE_BUFFER_OFF);
        assert_non_null(buf);
        assert_int_equal(mem_mgr_buf_off.free_bytes, OS_MAX_HEAP_MEMORY_PARTIAL_OFF - (i + 1) * S);
        mem_off_validate_free_bytes(&mem_mgr_buf_off);
    }
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_true(mem_mgr_buf_off.free_bytes >= S);
    buf = os_malloc(S, RAM_TYPE_BUFFER_OFF);
    assert_null(buf);
}

static void test_mem_off_split(void **state)
{
    STATIC_ASSERT(OS_MAX_HEAP_MEMORY_PARTIAL_OFF % (MEM_BLOCK_MIN_SIZE - 4) != 0, "");
    STATIC_ASSERT(OS_MAX_HEAP_MEMORY_PARTIAL_OFF / (MEM_BLOCK_MIN_SIZE - 4) > 4, "");

    const int S = MEM_BLOCK_MIN_SIZE - 4;
    const int N = OS_MAX_HEAP_MEMORY_PARTIAL_OFF / S;
    UCHAR *buf[N];
    os_heap_init();

    /*
     * Test case: Split free blocks for allocation
     */
    int i;
    for (i = 0; i < N; ++i)
    {
        assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
        buf[i] = os_malloc(S, RAM_TYPE_BUFFER_OFF);
        assert_ptr_equal(buf[i], heap_buf_off + i * S);
        mem_off_validate_free_bytes(&mem_mgr_buf_off);
    }
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 0);
    assert_int_equal(mem_off_get_block_size(mem_off_get_desc(buf[N - 1], &mem_mgr_buf_off)), OS_MAX_HEAP_MEMORY_PARTIAL_OFF - S * (N - 1));

    UCHAR *buf2 = os_malloc(S, RAM_TYPE_BUFFER_OFF);
    assert_null(buf2);

    os_free(buf[1], RAM_TYPE_BUFFER_OFF);
    os_free(buf[3], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 2);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    /*
     * Test case: Cannot allocate due to fragmentation
     */
    buf2 = os_malloc(S + 4, RAM_TYPE_BUFFER_OFF);
    assert_null(buf2);
}

static void test_mem_off_split2(void **state)
{
    STATIC_ASSERT(OS_MAX_HEAP_MEMORY_PARTIAL_OFF % MEM_BLOCK_MIN_SIZE == 0,
            "OS_MAX_HEAP_MEMORY_PARTIAL_OFF should be multiples of MEM_BLOCK_MIN_SIZE");

    int i;
    const int N = OS_MAX_HEAP_MEMORY_PARTIAL_OFF / MEM_BLOCK_MIN_SIZE;
    UCHAR *buf[N];
    os_heap_init();

    /*
     * Test case: Minimal block size
     */
    for (i = 0; i < N; ++i)
    {
        assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
        buf[i] = os_malloc(MEM_BLOCK_MIN_SIZE, RAM_TYPE_BUFFER_OFF);
        assert_ptr_equal(buf[i], heap_buf_off + i * MEM_BLOCK_MIN_SIZE);
        mem_off_validate_free_bytes(&mem_mgr_buf_off);
    }
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 0);
    assert_int_equal(mem_off_get_block_size(mem_off_get_desc(buf[N - 1], &mem_mgr_buf_off)), MEM_BLOCK_MIN_SIZE);

    /*
     * Test case: Don't split a free block if it's not large enough
     */
    os_free(buf[0], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);
    buf[0] = os_malloc(MEM_BLOCK_MIN_SIZE / 2, RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_block_size(mem_off_get_desc(buf[0], &mem_mgr_buf_off)), MEM_BLOCK_MIN_SIZE);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 0);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);
}

static void test_mem_off_merge(void **state)
{
    const int S = ((OS_MAX_HEAP_MEMORY_PARTIAL_OFF / 5) & -MEM_BLOCK_ALIGNMENT);
    UCHAR *buf[5];
    int i;

    os_heap_init();
    for (i = 0; i < 5; ++i)
    {
        buf[i] = os_malloc(S, RAM_TYPE_BUFFER_OFF);
    }
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 0);

    /*
     * Test case: Free block not merged (insert at head)
     */
    os_free(buf[1], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, S);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    /*
     * Test case: Free block merged with the next block
     */
    os_free(buf[0], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, 2 * S);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    /*
     * Test case: Free block merged with the previous block
     */
    os_free(buf[2], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, 3 * S);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    /*
     * Test case: Free block not merged
     */
    os_free(buf[4], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 2);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, 3 * S);
    assert_int_equal(mem_off_get_free_desc(1, &mem_mgr_buf_off)->size, OS_MAX_HEAP_MEMORY_PARTIAL_OFF - 4 * S);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    /*
     * Test case: Free block merged with the next and previous blocks
     */
    os_free(buf[3], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, OS_MAX_HEAP_MEMORY_PARTIAL_OFF);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);
}

#define OS_MAX_HEAP_MEMORY_PARTIAL_OFF2   16
UINT8 heap_sram_off2[MEM_BLOCK_SIZE_ALIGN(OS_MAX_HEAP_MEMORY_PARTIAL_OFF2)] ALIGN(MEM_BLOCK_ALIGNMENT);
void mem_off_manager_extend(MEM_OFF_MANAGER *mgr, UINT8 *heap, size_t heap_size);
static void test_mem_off_extend(void **state)
{
    UINT8 *heap[2];
    size_t heap_size[2];

    os_heap_init();
    mem_off_manager_extend(&mem_mgr_buf_off, heap_sram_off2, sizeof (heap_sram_off2));
    {
        int i1, i2;
        if (heap_buf_off < heap_sram_off2)
        {
            i1 = 0;
            i2 = 1;
        }
        else
        {
            i1 = 1;
            i2 = 0;
        }
        heap[i1] = heap_buf_off;
        heap[i2] = heap_sram_off2;
        heap_size[i1] = OS_MAX_HEAP_MEMORY_PARTIAL_OFF;
        heap_size[i2] = OS_MAX_HEAP_MEMORY_PARTIAL_OFF2;
    }
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 2);
    assert_ptr_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->addr, heap[0]);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, heap_size[0]);
    assert_ptr_equal(mem_off_get_free_desc(1, &mem_mgr_buf_off)->addr, heap[1]);
    assert_int_equal(mem_off_get_free_desc(1, &mem_mgr_buf_off)->size, heap_size[1]);

    UCHAR *buf[2];
    buf[0] = os_malloc(heap_size[0] - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 1);
    assert_ptr_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->addr, heap[1]);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    buf[1] = os_malloc(heap_size[1] - MEM_BLOCK_HEADER_SIZE, RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 0);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);

    os_free(buf[0], RAM_TYPE_BUFFER_OFF);
    os_free(buf[1], RAM_TYPE_BUFFER_OFF);
    assert_int_equal(mem_off_get_num_free_blocks(&mem_mgr_buf_off), 2);
    assert_ptr_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->addr, heap[0]);
    assert_int_equal(mem_off_get_free_desc(0, &mem_mgr_buf_off)->size, heap_size[0]);
    assert_ptr_equal(mem_off_get_free_desc(1, &mem_mgr_buf_off)->addr, heap[1]);
    assert_int_equal(mem_off_get_free_desc(1, &mem_mgr_buf_off)->size, heap_size[1]);
    mem_off_validate_free_bytes(&mem_mgr_buf_off);
}

int main()
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_mem_on_alloc),
        cmocka_unit_test(test_mem_on_split),
        cmocka_unit_test(test_mem_on_split2),
        cmocka_unit_test(test_mem_on_merge),
        cmocka_unit_test(test_mem_on_extend),
        cmocka_unit_test(test_mem_off_bsearch),
        cmocka_unit_test(test_mem_off_alloc),
        cmocka_unit_test(test_mem_off_alloc2),
        cmocka_unit_test(test_mem_off_split),
        cmocka_unit_test(test_mem_off_split2),
        cmocka_unit_test(test_mem_off_merge),
        cmocka_unit_test(test_mem_off_extend),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}

