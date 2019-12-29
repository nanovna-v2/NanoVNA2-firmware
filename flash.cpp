#include "flash.hpp"
#include "globals.hpp"
#include <libopencm3/stm32/flash.h>
#include <string.h>
#include <mculib/printk.hpp>

#define FLASH_PAGE_SIZE 2048

uint32_t flash_program_data(uint32_t dst, uint8_t *src, uint32_t bytes) {
	uint32_t flash_status = 0;

	// check if start_address is in proper range
	if(dst < FLASH_BASE)
		return -1;

	// calculate current page address
	if(dst % FLASH_PAGE_SIZE)
		return -1;

	flash_unlock();

	if(FLASH_CR & FLASH_CR_LOCK) {
		printk("flash_program_data: flash_unlock did not unlock hw\n");
		printk("&FLASH_KEYR = 0x%x\n", (uint32_t) &FLASH_KEYR);
		printk("&FLASH_CR = 0x%x\n", (uint32_t) &FLASH_CR);
		printk("FLASH_CR = 0x%x\n", (uint32_t) FLASH_CR);
		return -2;
	}

	// Erase pages
	uint32_t curr = dst;
	uint32_t end = dst + bytes;
	while(curr < end) {
		//flash_erase_page(curr);

		while(FLASH_SR & FLASH_SR_BSY);
		FLASH_CR |= FLASH_CR_PER;
		FLASH_AR = curr;
		FLASH_CR |= FLASH_CR_STRT;
		while(FLASH_SR & FLASH_SR_BSY);

		flash_status = FLASH_SR;
		if(flash_status != FLASH_SR_EOP) {
			printk("flash_program_data: erase error: flash status %d\n", flash_status);
			printk("flash_program_data: while erasing page 0x%x\n", curr);
			printk("flash_program_data: dst = 0x%x\n", dst);
			return -2;
		}
		curr += FLASH_PAGE_SIZE;
	}

	// programming flash memory
	for(uint32_t iter=0; iter<bytes; iter += 4)
	{
		// programming word data
		uint32_t word = *(uint32_t*)(src + iter);
		flash_program_word(dst+iter, word);
		flash_status = flash_get_status_flags();
		if(flash_status != FLASH_SR_EOP) {
			printk("flash_program_data: write error: flash status %d\n", flash_status);
			return -2;
		}

		// verify if correct data is programmed
		uint32_t readback = *(volatile uint32_t*)(dst + iter);
		
		if(readback != word) {
			printk("flash_program_data: verify failed: wrote %x, read %x\n", word, readback);
			return -3;
		}
	}

	return 0;
}




static uint32_t checksum(const void *start, size_t len) {
	uint32_t *p = (uint32_t*)start;
	uint32_t *tail = (uint32_t*)(((uint8_t*)start) + len);
	uint32_t value = 0;
	while (p < tail)
		value ^= *p++;
	return value;
}

int flash_caldata_save(int id) {
	// the size of the properties structure must be an integer multiple of
	// the flash word size, or else we will read past the end of the structure.
	static_assert((sizeof(current_props) % 4) == 0,
		"the size of the properties structure must be an integer multiple of 4");

	static_assert(SAVEAREA_BYTES >= sizeof(current_props),
		"SAVEAREA_BYTES is too small");

	uint8_t *src = (uint8_t*)&current_props;
	uint32_t dst = SAVEAREA(id);

	if (id < 0 || id >= SAVEAREA_MAX)
		return -1;

	current_props.magic = CONFIG_MAGIC;
	current_props.checksum = 0;
	current_props.checksum = checksum(&current_props, sizeof current_props);

	int ret = flash_program_data(dst, src, sizeof(current_props));

	printk("save caldata %d, ret = %d\n", id, ret);
	printk("src magic: %x, dst magic = %x\n", *(uint32_t*)src, *(uint32_t*)dst);
	lastsaveid = id;
	return ret;
}
int flash_caldata_recall(int id) {
	properties_t *src;
	void *dst = &current_props;

	if (id < 0 || id >= SAVEAREA_MAX)
		return -1;

	// point to saved area on the flash memory
	src = (properties_t*)SAVEAREA(id);

	if (src->magic != CONFIG_MAGIC) {
		printk("caldata_recall: incorrect magic %x, should be %x\n", src->magic, CONFIG_MAGIC);
		return -2;
	}
	int ck;
	if ((ck = checksum(src, sizeof(properties_t))) != 0) {
		printk("caldata_recall: incorrect checksum %d, should be 0\n", ck);
		return -3;
	}

	/* active configuration points to save data on flash memory */
	active_props = src;
	lastsaveid = id;

	/* duplicated saved data onto sram to be able to modify marker/trace */
	memcpy(dst, src, sizeof(properties_t));
	return 0;
}
const properties_t *flash_caldata_ref(int id) {
	const properties_t *src;
	if (id < 0 || id >= SAVEAREA_MAX)
		return nullptr;

	src = (const properties_t*)SAVEAREA(id);
	if (src->magic != CONFIG_MAGIC)
		return nullptr;
	if (checksum(src, sizeof(properties_t)) != 0)
		return nullptr;
	return src;
}

int flash_config_save(void) {
	uint8_t *src = (uint8_t*)&config;
	uint32_t dst = CONFIGAREA_BEGIN;
	uint32_t bytes = sizeof(config);

	static_assert(CONFIGAREA_BYTES >= sizeof(config),
		"CONFIGAREA_BYTES is too small");

	config.magic = CONFIG_MAGIC;
	config.checksum = 0;
	config.checksum = checksum(&config, bytes);

	return flash_program_data(dst, src, bytes);
}
int flash_config_recall(void) {
	const config_t *src = (const config_t*)CONFIGAREA_BEGIN;
	void *dst = &config;

	if (src->magic != CONFIG_MAGIC) {
		printk("config_recall: incorrect magic %x, should be %x\n", src->magic, CONFIG_MAGIC);
		return -1;
	}
	int ck;
	if ((ck = checksum(src, sizeof(config_t))) != 0) {
		printk("config_recall: incorrect checksum %d, should be 0\n", ck);
		return -1;
	}

	/* duplicated saved data onto sram to be able to modify marker/trace */
	memcpy(dst, src, sizeof(config_t));
	return 0;
}

void flash_clear_user(void) {
	flash_unlock();

	// erase flash pages
	uint8_t *p = (uint8_t*)USERFLASH_BEGIN;
	uint8_t *tail = (uint8_t*)board::USERFLASH_END;
	while (p < tail) {
		flash_erase_page((uint32_t)p);
		p += FLASH_PAGE_SIZE;
	}
}
