#pragma once
#include <Arduino.h>
struct __attribute__((packed)) StoredState {
  uint32_t magic;
  uint32_t sequence;
  uint32_t odoTenths;
  uint32_t tripTenths;
  uint32_t odoFraction;
  uint32_t tripFraction;
  uint16_t ratioMilli;
  int16_t offset;
  uint16_t schema;
  uint16_t crc;
};
// 32-byte payload + separate commit byte at offset 32.
static_assert(sizeof(StoredState) == 32, "FRAM layout changed");
const uint32_t RecordMagic = 0x53503137UL;
const uint16_t SlotAddress[2] = {0x80, 0xC0};
const uint8_t CommitMarker = 0xA5;
// Migration sentinel written only AFTER a valid first journal commit.
const uint16_t MigrationAddress = 0x70;

struct __attribute__((packed)) DisplayLayout {
  uint32_t magic;
  uint32_t sequence;
  int16_t x;
  int16_t y; // text BASELINE, not top edge
  uint8_t font;
  uint8_t digits; // observed window capacity; never truncates stored mileage
  uint8_t schema;
  uint16_t crc;
};
static_assert(sizeof(DisplayLayout) == 17, "Display FRAM layout changed");
const uint16_t LayoutSlotAddress[2] = {0x100, 0x120};
const uint32_t LayoutMagic = 0x44503138UL;
