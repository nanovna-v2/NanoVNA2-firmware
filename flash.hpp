#pragma once
#include "common.hpp"

/*
 * flash.cpp
 */
#define SAVEAREA_MAX 5


int caldata_save(int id);
int caldata_recall(int id);
const properties_t *caldata_ref(int id);

int config_save(void);
int config_recall(void);

void clear_all_config_prop_data(void);
