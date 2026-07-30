/*
 * Wire protocol: one framed block per drained DMA read, sent over USB CDC.
 *
 *   bytes 0..1   magic   0xA5 0x5A                 -- sync word, re-sync anchor
 *   bytes 2..5   seq     uint32, little-endian      -- frame counter (gap = frame
 *                                                       lost in transit)
 *   bytes 6..7   ovf     uint16, little-endian      -- cumulative DMA pool
 *                                                       overflow count (increments
 *                                                       => device dropped samples)
 *   bytes 8..9   count   uint16, little-endian      -- number of samples that follow
 *   bytes 10..   payload count * int32, little-endian -- instantaneous mains
 *                                                       voltage, in millivolts
 *                                                       (see mains_filter.h)
 *
 * `seq` gaps and `ovf` increments are independent loss signals: the former
 * reveals transport-side loss (USB/host), the latter device-side loss (host
 * too slow to drain the DMA pool). Together they let the PC reader prove
 * whether the stream is complete.
 */
#pragma once

#include <stdint.h>

#define FRAME_MAGIC0        0xA5
#define FRAME_MAGIC1        0x5A
#define FRAME_HEADER_LEN    10   /* magic(2) + seq(4) + ovf(2) + count(2) */

/* Write the 10-byte little-endian header into out[0..FRAME_HEADER_LEN). Byte-
 * by-byte (rather than a packed struct) keeps the on-wire layout explicit and
 * immune to struct-padding/endianness surprises. */
static inline void wire_pack_header(uint8_t *out, uint32_t seq, uint32_t ovf, uint16_t count) {
    out[0] = FRAME_MAGIC0;
    out[1] = FRAME_MAGIC1;
    out[2] = (uint8_t) (seq);
    out[3] = (uint8_t) (seq >> 8);
    out[4] = (uint8_t) (seq >> 16);
    out[5] = (uint8_t) (seq >> 24);
    out[6] = (uint8_t) (ovf);
    out[7] = (uint8_t) (ovf >> 8);
    out[8] = (uint8_t) (count);
    out[9] = (uint8_t) (count >> 8);
}
