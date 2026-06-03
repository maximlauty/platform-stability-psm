#pragma once
// safety_imu.h -- ASM330LHB SPI driver, self-test, HW FSM, sensor reads.
// ASIL B / SIL 2 layer. No app/ includes.
#include "safety_defs.h"
#include "safety_types.h"

uint8_t asm_reg_read    (uint8_t cs, uint8_t addr);
void    asm_reg_write   (uint8_t cs, uint8_t addr, uint8_t val);
void    asm_burst_read  (uint8_t cs, uint8_t start_addr, uint8_t *buf, uint8_t len);
void    asm_sw_reset    (uint8_t cs);
bool    asm_init        (uint8_t cs);
bool    asm_self_test   (uint8_t cs);
void    emb_write_page  (uint8_t cs, uint8_t page, uint8_t addr, const uint8_t *data, uint8_t len);
bool    asm_fsm_configure   (uint8_t cs);
bool    asm_fsm_read_stable (uint8_t cs);
void    asm_frozen_burst_reset(uint8_t cs);
bool    asm_verify_config_crc(uint8_t cs);
bool    asm_read_sensors(uint8_t cs,
                         float &ax_g,  float &ay_g,  float &az_g,
                         float &gx_rs, float &gy_rs, float &gz_rs);
