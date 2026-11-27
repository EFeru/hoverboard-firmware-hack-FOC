#include "trike_config.h"

#ifdef VARIANT_TRIKE
/*
 * TRIKE-only persistent configuration.
 * Two-page transactional journal: write/verify the inactive page first,
 * then retain the previous valid page as recovery until the next save.
 *
 * Record (13 halfwords, little-endian):
 *  0..1 magic, 2 version, 3 mode, 4 flags,
 *  5 throttle_zero, 6 throttle_min, 7 throttle_max, 8 reserved,
 *  9..10 sequence, 11 CRC16/CCITT-FALSE, 12 commit marker.
 *
 * Throttle fields are reserved for the separately specified calibration
 * workflow. They are preserved across mode changes and are not interpreted
 * by the current firmware.
 */

#define TRIKE_CFG_MAGIC_LO   0x5254U
#define TRIKE_CFG_MAGIC_HI   0x4B49U
#define TRIKE_CFG_VERSION     1U
#define TRIKE_CFG_COMMIT      0xA55AU
#define TRIKE_CFG_WORDS       13U
#define TRIKE_CFG_CRC_INDEX   11U
#define TRIKE_CFG_COMMIT_IDX  12U

typedef struct {
  uint16_t words[TRIKE_CFG_WORDS];
  uint32_t address;
  uint8_t valid;
} TrikeConfigRecord;

static uint16_t trikeConfigCrc16(const uint16_t *words) {
  uint16_t crc = 0xFFFFU;
  for (uint8_t i = 0; i < TRIKE_CFG_CRC_INDEX; i++) {
    uint8_t bytes[2] = { (uint8_t)(words[i] & 0xFFU), (uint8_t)(words[i] >> 8) };
    for (uint8_t b = 0; b < 2U; b++) {
      crc ^= (uint16_t)bytes[b] << 8;
      for (uint8_t bit = 0; bit < 8U; bit++) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static uint8_t trikeConfigReadPage(uint32_t address, TrikeConfigRecord *record) {
  for (uint8_t i = 0; i < TRIKE_CFG_WORDS; i++) {
    record->words[i] = *(__IO uint16_t *)(address + ((uint32_t)i * 2U));
  }
  record->address = address;
  record->valid = 0U;

  if (record->words[0] != TRIKE_CFG_MAGIC_LO ||
      record->words[1] != TRIKE_CFG_MAGIC_HI ||
      record->words[2] != TRIKE_CFG_VERSION ||
      record->words[3] < TRIKE_MODE_M1 ||
      record->words[3] > TRIKE_MODE_M3 ||
      record->words[TRIKE_CFG_COMMIT_IDX] != TRIKE_CFG_COMMIT ||
      record->words[TRIKE_CFG_CRC_INDEX] != trikeConfigCrc16(record->words)) {
    return 0U;
  }

  record->valid = 1U;
  return 1U;
}

static uint8_t trikeConfigIsNewer(const TrikeConfigRecord *a, const TrikeConfigRecord *b) {
  uint32_t seqA = ((uint32_t)a->words[10] << 16) | a->words[9];
  uint32_t seqB = ((uint32_t)b->words[10] << 16) | b->words[9];
  return ((int32_t)(seqA - seqB) > 0) ? 1U : 0U;
}

static uint8_t trikeConfigGetLatest(TrikeConfigRecord *latest) {
  TrikeConfigRecord a, b;
  uint8_t validA = trikeConfigReadPage(TRIKE_CONFIG_PAGE_A, &a);
  uint8_t validB = trikeConfigReadPage(TRIKE_CONFIG_PAGE_B, &b);

  if (!validA && !validB) return 0U;
  if (validA && validB) *latest = trikeConfigIsNewer(&a, &b) ? a : b;
  else *latest = validA ? a : b;
  return 1U;
}

uint8_t trikeConfigLoadMode(void) {
  TrikeConfigRecord latest;
  if (!trikeConfigGetLatest(&latest)) return TRIKE_MODE_M1;
  return (uint8_t)latest.words[3];
}

HAL_StatusTypeDef trikeConfigSaveMode(uint8_t mode) {
  if (mode < TRIKE_MODE_M1 || mode > TRIKE_MODE_M3) return HAL_ERROR;

  TrikeConfigRecord latest;
  uint8_t haveLatest = trikeConfigGetLatest(&latest);
  uint32_t target = haveLatest
      ? ((latest.address == TRIKE_CONFIG_PAGE_A) ? TRIKE_CONFIG_PAGE_B : TRIKE_CONFIG_PAGE_A)
      : TRIKE_CONFIG_PAGE_A;

  uint16_t words[TRIKE_CFG_WORDS] = {0};
  words[0] = TRIKE_CFG_MAGIC_LO;
  words[1] = TRIKE_CFG_MAGIC_HI;
  words[2] = TRIKE_CFG_VERSION;
  words[3] = mode;

  if (haveLatest) {
    words[4] = latest.words[4]; /* calibration flags */
    words[5] = latest.words[5]; /* throttle zero */
    words[6] = latest.words[6]; /* throttle minimum */
    words[7] = latest.words[7]; /* throttle maximum */
    words[8] = latest.words[8]; /* reserved */
    uint32_t sequence = (((uint32_t)latest.words[10] << 16) | latest.words[9]) + 1U;
    words[9] = (uint16_t)(sequence & 0xFFFFU);
    words[10] = (uint16_t)(sequence >> 16);
  } else {
    words[9] = 1U;
    words[10] = 0U;
  }

  words[TRIKE_CFG_CRC_INDEX] = trikeConfigCrc16(words);
  words[TRIKE_CFG_COMMIT_IDX] = TRIKE_CFG_COMMIT;

  FLASH_EraseInitTypeDef erase;
  uint32_t pageError = 0U;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = target;
  erase.NbPages = 1U;

  HAL_StatusTypeDef status = HAL_FLASH_Unlock();
  if (status != HAL_OK) return status;

  status = HAL_FLASHEx_Erase(&erase, &pageError);
  if (status == HAL_OK) {
    /* Program all payload halfwords except the commit marker. */
    for (uint8_t i = 0; i < TRIKE_CFG_COMMIT_IDX; i++) {
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                 target + ((uint32_t)i * 2U), words[i]);
      if (status != HAL_OK) break;
    }
  }

  /* Commit is deliberately written last: incomplete records are ignored. */
  if (status == HAL_OK) {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               target + ((uint32_t)TRIKE_CFG_COMMIT_IDX * 2U),
                               TRIKE_CFG_COMMIT);
  }

  HAL_FLASH_Lock();
  if (status != HAL_OK) return status;

  TrikeConfigRecord verify;
  if (!trikeConfigReadPage(target, &verify)) return HAL_ERROR;
  return HAL_OK;
}
#endif /* VARIANT_TRIKE */
