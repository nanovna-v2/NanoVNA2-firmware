#pragma once
#include "common.hpp"
#include <board.hpp>

/*
 * flash.cpp
 */


#define SAVEAREA_MAX  5

// allocated bytes per save area. must be a multiple of the flash page size (2048)
constexpr uint32_t SAVEAREA_BYTES = 12288;

// total bytes of save areas
constexpr uint32_t SAVETOTAL_BYTES = SAVEAREA_BYTES * SAVEAREA_MAX;

// allocated bytes for config data in flash. must be a multiple of the flash page size (2048)
constexpr uint32_t CONFIGAREA_BYTES = 2048;

// flash user area is at the very end of the flash, defined by USERFLASH_END in board.hpp
constexpr uint32_t USERFLASH_BEGIN = board::USERFLASH_END - SAVETOTAL_BYTES - CONFIGAREA_BYTES;

// locations of config and save areas
constexpr uint32_t CONFIGAREA_BEGIN = USERFLASH_BEGIN;
constexpr uint32_t SAVEAREA_BEGIN = CONFIGAREA_BEGIN + CONFIGAREA_BYTES;

static inline uint32_t SAVEAREA(int id) {
	//assert(id >= 0 && id < SAVEAREA_MAX);
	return SAVEAREA_BEGIN + id*SAVEAREA_BYTES;
}



uint32_t flash_program_data(uint32_t start_address, uint8_t *input_data, uint32_t num_elements);

int flash_caldata_save(int id);
int flash_caldata_recall(int id);
const properties_t *flash_caldata_ref(int id);

int flash_config_save(void);
int flash_config_recall(void);

void flash_clear_user(void);
