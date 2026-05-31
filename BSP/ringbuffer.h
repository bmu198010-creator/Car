#ifndef RINGBUFFER_H__
#define RINGBUFFER_H__

#include <string.h>
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 在rtconfig.h中定义ALIGN_SIZE */
#define ALIGN_SIZE 4

/* 将给定的字节数（或大小）向下对齐到指定的边界。这里的“向下对齐”意味着结果将不大于原始值，但会是最接近且不大于原始值的、指定对齐宽度的整数倍 */
#define ALIGN_DOWN(size, align) ((size) & ~((align) - 1))


/* ring buffer */
struct ringbuffer
{
    uint8_t *buffer_ptr;
    /* use the msb of the {read,write}_index as mirror bit. You can see this as
     * if the buffer adds a virtual mirror and the pointers point either to the
     * normal or to the mirrored buffer. If the write_index has the same value
     * with the read_index, but in a different mirror, the buffer is full.
     * While if the write_index and the read_index are the same and within the
     * same mirror, the buffer is empty. The ASCII art of the ringbuffer is:
     *
     *          mirror = 0                    mirror = 1
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Full
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     *  read_idx-^                   write_idx-^
     *
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Empty
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * read_idx-^ ^-write_idx
     */

    uint32_t read_mirror : 1;
    uint32_t read_index : 31;
    uint32_t write_mirror : 1;
    uint32_t write_index : 31;
    /* as we use msb of index as mirror bit, the size should be signed and
     * could only be positive. */
    int32_t buffer_size;
};

enum ringbuffer_state
{
    RINGBUFFER_EMPTY,
    RINGBUFFER_FULL,
    /* half full is neither full nor empty */
    RINGBUFFER_HALFFULL,
};

/**
 * RingBuffer for DeviceDriver
 *
 * Please note that the ring buffer implementation of RT-Thread
 * has no thread wait or resume feature.
 */
void ringbuffer_init(struct ringbuffer *rb, uint8_t *pool, int32_t size);
void ringbuffer_reset(struct ringbuffer *rb);
uint32_t ringbuffer_put(struct ringbuffer *rb, const uint8_t *ptr, uint32_t length);
uint32_t ringbuffer_put_force(struct ringbuffer *rb, const uint8_t *ptr, uint32_t length);
uint32_t ringbuffer_putchar(struct ringbuffer *rb, const uint8_t ch);
uint32_t ringbuffer_putchar_force(struct ringbuffer *rb, const uint8_t ch);
uint32_t ringbuffer_get(struct ringbuffer *rb, uint8_t *ptr, uint32_t length);
uint32_t ringbuffer_peek(struct ringbuffer *rb, uint8_t **ptr);
uint32_t ringbuffer_getchar(struct ringbuffer *rb, uint8_t *ch);
uint32_t ringbuffer_data_len(struct ringbuffer *rb);

void assert_handler(const char* ex_string, const char* func, uint32_t line);
#define ASSERT(ex) \
    if (!(ex)) \
    { \
        assert_handler(#ex, __FUNCTION__, __LINE__); \
    }

#ifdef USING_HEAP
struct ringbuffer* ringbuffer_create(uint32_t length);
void ringbuffer_destroy(struct ringbuffer *rb);
#endif

/**
 * @brief Get the buffer size of the ring buffer object.
 *
 * @param rb        A pointer to the ring buffer object.
 *
 * @return  Buffer size.
 */
inline uint32_t ringbuffer_get_size(struct ringbuffer *rb)
{
    ASSERT(rb != NULL);
    return rb->buffer_size;
}

/** return the size of empty space in rb */
#define ringbuffer_space_len(rb) ((rb)->buffer_size - ringbuffer_data_len(rb))


#ifdef __cplusplus
}
#endif

#endif
