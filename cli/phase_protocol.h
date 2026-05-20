#ifndef PHASE_PROTOCOL_H
#define PHASE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define HOLO_PHASE_MAGIC 0x484F4C4Fu
#define HOLO_PHASE_VERSION 1u
#define HOLO_PHASE_COUNT 200u
#define HOLO_PHASE_MAX 512u
#define HOLO_PHASE_TCP_PORT 5656
#define HOLO_CMD_SET_PHASE_FRAME 0x0Bu

#pragma pack(push, 1)
typedef struct HoloPhaseFrame {
	uint32_t magic;
	uint16_t version;
	uint16_t frame_id;
	uint16_t phase_count;
	uint16_t phase_max;
	uint16_t phases[HOLO_PHASE_COUNT];
	uint32_t crc32;
} HoloPhaseFrame;
#pragma pack(pop)

static uint32_t holo_crc32(const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t crc = 0xffffffffu;

	for(size_t i = 0; i < len; i++) {
		crc ^= p[i];
		for(int bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
	}

	return ~crc;
}

static uint32_t holo_phase_frame_crc(const HoloPhaseFrame *frame)
{
	return holo_crc32(frame, offsetof(HoloPhaseFrame, crc32));
}

#endif
