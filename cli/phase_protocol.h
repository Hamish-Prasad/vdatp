#ifndef PHASE_PROTOCOL_H
#define PHASE_PROTOCOL_H

/*
 * Shared laptop-to-Pi phase-frame protocol.
 *
 * Both cli.c and laptop_phase_sender.c include this file so they agree on the
 * exact packet layout, magic values, phase count, and CRC calculation.
 */

#include <stdint.h>  /* uint8_t/uint16_t/uint32_t are fixed-width unsigned integer types. */
#include <stddef.h>  /* size_t and offsetof are used by the CRC helper below. */

/*
 * Fixed-width type note:
 *
 * uint8_t  means "unsigned integer, exactly 8 bits"  (one byte).
 * uint16_t means "unsigned integer, exactly 16 bits" (two bytes).
 * uint32_t means "unsigned integer, exactly 32 bits" (four bytes).
 *
 * These are used instead of plain int/short/long because the laptop, Pi, and
 * FPGA protocol must have the same byte layout on every compiler/platform.
 */

#define HOLO_PHASE_MAGIC 0x484F4C4Fu       /* ASCII "HOLO" packed into a 32-bit unsigned value. */
#define HOLO_PHASE_VERSION 1u              /* Protocol version; bump this if the packet format changes. */
#define HOLO_PHASE_COUNT 200u              /* Four FPGA boards times 50 phase outputs per board. */
#define HOLO_PHASE_MAX 512u                /* Current FPGA PWM phase period: 20.48 MHz / 40 kHz = 512. */
#define HOLO_PHASE_TCP_PORT 5656           /* Default TCP port used by the laptop sender and Pi bridge. */
#define HOLO_CMD_SET_PHASE_FRAME 0x0Bu     /* FPGA SPI command number for direct phase-frame mode. */

/*
 * Force byte packing so the struct has no compiler-inserted padding bytes.
 * That matters because the laptop sends this struct directly over TCP and the
 * Pi validates the bytes using the same struct definition.
 */
#pragma pack(push, 1)

typedef struct HoloPhaseFrame {
	uint32_t magic;                         /* 32-bit marker used to reject non-HOLO packets. */
	uint16_t version;                       /* 16-bit protocol version, currently HOLO_PHASE_VERSION. */
	uint16_t frame_id;                      /* 16-bit counter so logs can show which frame was sent. */
	uint16_t phase_count;                   /* 16-bit count; should always be HOLO_PHASE_COUNT. */
	uint16_t phase_max;                     /* 16-bit max phase period; should always be HOLO_PHASE_MAX. */
	uint16_t phases[HOLO_PHASE_COUNT];      /* 200 unsigned 16-bit phase values, valid range 0..511. */
	uint32_t crc32;                         /* 32-bit CRC covering every earlier byte in this struct. */
} HoloPhaseFrame;                           /* Named packet type used by sender and bridge code. */

#pragma pack(pop)

/*
 * Calculate a standard CRC-32 over an arbitrary byte buffer.
 *
 * data points to the first byte to check.
 * len is the number of bytes to include.
 * The return value is a 32-bit checksum.
 */
static uint32_t holo_crc32(const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;       /* View the input as individual bytes. */
	uint32_t crc = 0xffffffffu;                     /* CRC-32 starts with all bits set. */

	for(size_t i = 0; i < len; i++) {               /* Process one byte at a time. */
		crc ^= p[i];                                /* Mix the next byte into the low bits. */
		for(int bit = 0; bit < 8; bit++)            /* Process each of the byte's 8 bits. */
			crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u))); /* Apply CRC polynomial if low bit was set. */
	}

	return ~crc;                                    /* CRC-32 returns the bitwise inverse of the accumulator. */
}

/*
 * Calculate the CRC for a HoloPhaseFrame.
 *
 * offsetof(HoloPhaseFrame, crc32) gives the number of bytes before the crc32
 * field, so the CRC covers the header and phase payload but not the CRC field
 * itself.
 */
static uint32_t holo_phase_frame_crc(const HoloPhaseFrame *frame)
{
	return holo_crc32(frame, offsetof(HoloPhaseFrame, crc32)); /* Check everything before frame->crc32. */
}

#endif
