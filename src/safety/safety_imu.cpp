// safety_imu.cpp -- ASM330LHB SPI driver, self-test, HW FSM (ASIL B / SIL 2).
// Independence rule: no app/ includes.

#include "safety_imu.h"
#include "safety_monitor.h"
#include <SPI.h>
#include <Arduino.h>

// CRC-8: polynomial 0x07, initial value 0xFF. Used for config register shadow.
static uint8_t crc8(const uint8_t *buf, uint8_t n)
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < n; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0U; b < 8U; b++)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Shadowed CRC of {CTRL1_XL, CTRL2_G, CTRL3_C, CTRL4_C} for each IMU.
// Set at the end of every successful asm_init(); compared by asm_verify_config_crc().
static uint8_t s_cfg_crc[2] = {0U, 0U};

static uint8_t cfg_snapshot_crc(uint8_t cs)
{
    uint8_t cfg[4] = {
        asm_reg_read(cs, ASM_REG_CTRL1_XL),
        asm_reg_read(cs, ASM_REG_CTRL2_G),
        asm_reg_read(cs, ASM_REG_CTRL3_C),
        asm_reg_read(cs, ASM_REG_CTRL4_C)
    };
    return crc8(cfg, 4U);
}

uint8_t asm_reg_read(uint8_t cs, uint8_t addr)
{
    SPI1.beginTransaction(SPISettings(ASM_SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWriteFast(cs, LOW);
    SPI1.transfer(addr | ASM_SPI_READ_BIT);
    uint8_t val = SPI1.transfer(0x00U);
    digitalWriteFast(cs, HIGH);
    SPI1.endTransaction();
    return val;
}

void asm_reg_write(uint8_t cs, uint8_t addr, uint8_t val)
{
    SPI1.beginTransaction(SPISettings(ASM_SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWriteFast(cs, LOW);
    SPI1.transfer(addr & 0x7FU);
    SPI1.transfer(val);
    digitalWriteFast(cs, HIGH);
    SPI1.endTransaction();
}

void asm_burst_read(uint8_t cs, uint8_t start_addr, uint8_t *buf, uint8_t len)
{
    SPI1.beginTransaction(SPISettings(ASM_SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWriteFast(cs, LOW);
    SPI1.transfer(start_addr | ASM_SPI_READ_BIT);
    for (uint8_t i = 0; i < len; ++i) buf[i] = SPI1.transfer(0x00U);
    digitalWriteFast(cs, HIGH);
    SPI1.endTransaction();
}

void asm_sw_reset(uint8_t cs)
{
    SPI1.beginTransaction(SPISettings(ASM_SPI_CLOCK, MSBFIRST, SPI_MODE3));
    digitalWriteFast(cs, LOW);
    for (uint8_t i = 0U; i < 16U; ++i) SPI1.transfer(0xFFU);
    digitalWriteFast(cs, HIGH);
    SPI1.endTransaction();
    delayMicroseconds(10U);

    asm_reg_write(cs, ASM_REG_CTRL3_C, 0x01U);
    wdt_feed();
    uint32_t t0 = millis();
    while (asm_reg_read(cs, ASM_REG_CTRL3_C) & 0x01U) {
        if (millis() - t0 > 10U) break;
    }
    wdt_feed();
    delay(2);
}

bool asm_init(uint8_t cs)
{
    asm_sw_reset(cs);
    bool who_ok = false;
    uint8_t last_who = 0U;
    for (uint8_t t = 0U; t < 5U && !who_ok; ++t) {
        if (t > 0U) delay(2);
        last_who = asm_reg_read(cs, ASM_REG_WHO_AM_I);
        who_ok = (last_who == ASM_WHOAMI_VAL);
    }
    if (!who_ok) {
        char dbuf[56];
        snprintf(dbuf, sizeof(dbuf), "[INIT] cs=%u WHO_AM_I=0x%02X (want 0x%02X)",
                 (unsigned)cs, (unsigned)last_who, (unsigned)ASM_WHOAMI_VAL);
        if (Serial.availableForWrite() > 0) Serial.println(dbuf);
        return false;
    }
    asm_reg_write(cs, ASM_REG_CTRL3_C,   ASM_CTRL3_C_VAL);
    asm_reg_write(cs, ASM_REG_CTRL4_C,   ASM_CTRL4_C_VAL);
    asm_reg_write(cs, ASM_REG_CTRL1_XL,  ASM_CTRL1_XL_VAL);
    asm_reg_write(cs, ASM_REG_CTRL2_G,   ASM_CTRL2_G_VAL);
    asm_reg_write(cs, ASM_REG_INT1_CTRL, 0x03U);
    s_cfg_crc[(cs == IMU_A_CS_PIN) ? 0U : 1U] = cfg_snapshot_crc(cs);
    return true;
}

bool asm_verify_config_crc(uint8_t cs)
{
    return cfg_snapshot_crc(cs) == s_cfg_crc[(cs == IMU_A_CS_PIN) ? 0U : 1U];
}

bool asm_self_test(uint8_t cs)
{
    asm_reg_write(cs, ASM_REG_CTRL1_XL, 0x34U);
    asm_reg_write(cs, ASM_REG_CTRL2_G,  0x5CU);
    wdt_feed(); delay(800); wdt_feed();
    asm_reg_write(cs, ASM_REG_CTRL3_C, ASM_CTRL3_C_VAL);
    delay(2);
    { uint8_t tmp[12]; asm_burst_read(cs, ASM_REG_OUTX_L_G, tmp, 12); (void)tmp; }

    int32_t bg[3] = {0, 0, 0};
    for (uint8_t s = 0; s < 5U; ++s) {
        { uint32_t tw = millis();
          while ((asm_reg_read(cs, ASM_REG_STATUS_REG) & 0x03U) != 0x03U)
              if (millis() - tw > 20U) break; }
        uint8_t buf[12];
        asm_burst_read(cs, ASM_REG_OUTX_L_G, buf, 12);
        for (uint8_t a = 0; a < 3U; ++a)
            bg[a] += (int16_t)((uint16_t)buf[2*a+1] << 8 | buf[2*a]);
    }

    asm_reg_write(cs, ASM_REG_CTRL5_C, 0x04U);
    { uint8_t rb = asm_reg_read(cs, ASM_REG_CTRL5_C);
      char rblog[56];
      snprintf(rblog, sizeof(rblog), "[ST] cs=%u CTRL5_C readback=0x%02X (want 0x04)", (unsigned)cs, (unsigned)rb);
      if (Serial.availableForWrite() > 0) Serial.println(rblog); }
    wdt_feed(); delay(200); wdt_feed();

    int32_t sg[3] = {0, 0, 0};
    for (uint8_t s = 0; s < 5U; ++s) {
        { uint32_t tw = millis();
          while ((asm_reg_read(cs, ASM_REG_STATUS_REG) & 0x03U) != 0x03U)
              if (millis() - tw > 20U) break; }
        uint8_t buf[12];
        asm_burst_read(cs, ASM_REG_OUTX_L_G, buf, 12);
        for (uint8_t a = 0; a < 3U; ++a)
            sg[a] += (int16_t)((uint16_t)buf[2*a+1] << 8 | buf[2*a]);
    }
    asm_reg_write(cs, ASM_REG_CTRL5_C, 0x00U);

    static constexpr float GYRO_ST_MIN = 2143.0f;
    static constexpr float GYRO_ST_MAX = 10000.0f;
    bool gyro_pass = true;
    for (uint8_t a = 0; a < 3U; ++a) {
        float d = fabsf((float)(sg[a] - bg[a]) / 5.0f);
        char stlog[72];
        snprintf(stlog, sizeof(stlog), "[ST] cs=%u gyro[%u] delta=%.0f  (want %.0f-%.0f)",
                 (unsigned)cs, (unsigned)a, (double)d, (double)GYRO_ST_MIN, (double)GYRO_ST_MAX);
        if (Serial.availableForWrite() > 0) Serial.println(stlog);
        if (d < GYRO_ST_MIN || d > GYRO_ST_MAX) gyro_pass = false;
    }
    if (!gyro_pass) {
        asm_reg_write(cs, ASM_REG_CTRL1_XL, ASM_CTRL1_XL_VAL);
        asm_reg_write(cs, ASM_REG_CTRL2_G,  ASM_CTRL2_G_VAL);
        return false;
    }

    asm_reg_write(cs, ASM_REG_CTRL2_G,  0x00U);
    asm_reg_write(cs, ASM_REG_CTRL1_XL, 0x34U);
    wdt_feed(); delay(200); wdt_feed();
    { uint8_t tmp[12]; asm_burst_read(cs, ASM_REG_OUTX_L_G, tmp, 12); (void)tmp; }

    int32_t ba[3] = {0, 0, 0};
    for (uint8_t s = 0; s < 5U; ++s) {
        { uint32_t tw = millis();
          while ((asm_reg_read(cs, ASM_REG_STATUS_REG) & 0x01U) != 0x01U)
              if (millis() - tw > 30U) break; }
        uint8_t buf[12];
        asm_burst_read(cs, ASM_REG_OUTX_L_G, buf, 12);
        for (uint8_t a = 0; a < 3U; ++a)
            ba[a] += (int16_t)((uint16_t)buf[6 + 2*a+1] << 8 | buf[6 + 2*a]);
    }

    asm_reg_write(cs, ASM_REG_CTRL5_C, 0x01U);
    { uint8_t rb = asm_reg_read(cs, ASM_REG_CTRL5_C);
      char rblog[56];
      snprintf(rblog, sizeof(rblog), "[ST] cs=%u CTRL5_C readback=0x%02X (want 0x01)", (unsigned)cs, (unsigned)rb);
      if (Serial.availableForWrite() > 0) Serial.println(rblog); }
    wdt_feed(); delay(200); wdt_feed();

    int32_t sa_pos[3] = {0, 0, 0};
    for (uint8_t s = 0; s < 5U; ++s) {
        { uint32_t tw = millis();
          while ((asm_reg_read(cs, ASM_REG_STATUS_REG) & 0x01U) != 0x01U)
              if (millis() - tw > 30U) break; }
        uint8_t buf[12];
        asm_burst_read(cs, ASM_REG_OUTX_L_G, buf, 12);
        for (uint8_t a = 0; a < 3U; ++a)
            sa_pos[a] += (int16_t)((uint16_t)buf[6 + 2*a+1] << 8 | buf[6 + 2*a]);
    }
    asm_reg_write(cs, ASM_REG_CTRL5_C, 0x00U);
    delay(50);

    asm_reg_write(cs, ASM_REG_CTRL5_C, 0x02U);
    { uint8_t rb = asm_reg_read(cs, ASM_REG_CTRL5_C);
      char rblog[56];
      snprintf(rblog, sizeof(rblog), "[ST] cs=%u CTRL5_C readback=0x%02X (want 0x02)", (unsigned)cs, (unsigned)rb);
      if (Serial.availableForWrite() > 0) Serial.println(rblog); }
    wdt_feed(); delay(200); wdt_feed();

    int32_t sa_neg[3] = {0, 0, 0};
    for (uint8_t s = 0; s < 5U; ++s) {
        { uint32_t tw = millis();
          while ((asm_reg_read(cs, ASM_REG_STATUS_REG) & 0x01U) != 0x01U)
              if (millis() - tw > 30U) break; }
        uint8_t buf[12];
        asm_burst_read(cs, ASM_REG_OUTX_L_G, buf, 12);
        for (uint8_t a = 0; a < 3U; ++a)
            sa_neg[a] += (int16_t)((uint16_t)buf[6 + 2*a+1] << 8 | buf[6 + 2*a]);
    }
    asm_reg_write(cs, ASM_REG_CTRL5_C, 0x00U);

    static constexpr float ACCEL_ST_MIN =  328.0f;
    static constexpr float ACCEL_ST_MAX = 13934.0f;
    bool accel_pass = true;
    for (uint8_t a = 0; a < 3U; ++a) {
        float d_pos = fabsf((float)(sa_pos[a] - ba[a]) / 5.0f);
        float d_neg = fabsf((float)(sa_neg[a] - ba[a]) / 5.0f);
        float d = (d_pos + d_neg) / 2.0f;
        char stlog[96];
        snprintf(stlog, sizeof(stlog),
                 "[ST] cs=%u accel[%u] pos=%.0f neg=%.0f avg=%.0f  (want %.0f-%.0f)",
                 (unsigned)cs, (unsigned)a, (double)d_pos, (double)d_neg, (double)d,
                 (double)ACCEL_ST_MIN, (double)ACCEL_ST_MAX);
        if (Serial.availableForWrite() > 0) Serial.println(stlog);
        if (d < ACCEL_ST_MIN || d > ACCEL_ST_MAX) accel_pass = false;
    }
    asm_reg_write(cs, ASM_REG_CTRL1_XL, ASM_CTRL1_XL_VAL);
    asm_reg_write(cs, ASM_REG_CTRL2_G,  ASM_CTRL2_G_VAL);
    return accel_pass;
}

void emb_write_page(uint8_t cs, uint8_t page, uint8_t addr, const uint8_t *data, uint8_t len)
{
    asm_reg_write(cs, ASM_REG_PAGE_SEL,     (uint8_t)((page << 4) | 0x01U));
    asm_reg_write(cs, ASM_REG_PAGE_ADDRESS, addr);
    for (uint8_t i = 0; i < len; ++i)
        asm_reg_write(cs, ASM_REG_PAGE_VALUE, data[i]);
}

bool asm_fsm_configure(uint8_t cs)
{
    static const uint8_t s_fsm1_prog[] = {
        0x50U, 0x00U, 0x0EU, 0x20U, 0x00U, 0x00U,
        0x9AU, 0x35U, 0x02U, 0x00U, 0x23U, 0x01U,
        0x57U, 0x11U,
    };
    static const uint8_t s_page1_cfg[] = {
        0x00U, 0x00U, 0x01U, 0x01U, 0x00U, 0x04U,
    };

    wdt_feed();
    asm_reg_write(cs, ASM_REG_CTRL1_XL, 0x00U);
    asm_reg_write(cs, ASM_REG_CTRL2_G,  0x00U);
    delay(10);

    asm_reg_write(cs, ASM_REG_FUNC_CFG_ACCESS, 0x80U);
    asm_reg_write(cs, ASM_REG_PAGE_RW,         0xC0U);
    asm_reg_write(cs, ASM_EMB_FUNC_EN_B,    0x01U);
    asm_reg_write(cs, ASM_EMB_ODR_CFG_B,    0x5BU);
    asm_reg_write(cs, ASM_EMB_FSM_ENABLE_A, 0x01U);
    asm_reg_write(cs, ASM_EMB_FSM_ENABLE_B, 0x00U);
    asm_reg_write(cs, ASM_EMB_FSM_INT1_A,   0x00U);
    asm_reg_write(cs, ASM_EMB_FSM_INT1_B,   0x00U);
    asm_reg_write(cs, ASM_EMB_FSM_INT2_A,   0x00U);
    asm_reg_write(cs, ASM_EMB_FSM_INT2_B,   0x00U);
    emb_write_page(cs, 1, 0x7AU, s_page1_cfg, sizeof(s_page1_cfg));
    emb_write_page(cs, 4, 0x00U, s_fsm1_prog, sizeof(s_fsm1_prog));
    asm_reg_write(cs, ASM_REG_PAGE_SEL,          0x01U);
    asm_reg_write(cs, ASM_REG_PAGE_RW,           0x80U);
    asm_reg_write(cs, ASM_REG_FUNC_CFG_ACCESS,   0x00U);
    asm_reg_write(cs, ASM_REG_CTRL3_C,   ASM_CTRL3_C_VAL);
    asm_reg_write(cs, ASM_REG_CTRL4_C,   ASM_CTRL4_C_VAL);
    asm_reg_write(cs, ASM_REG_CTRL1_XL,  ASM_CTRL1_XL_VAL);
    asm_reg_write(cs, ASM_REG_CTRL2_G,   ASM_CTRL2_G_VAL);
    asm_reg_write(cs, ASM_REG_INT1_CTRL, 0x03U);
    wdt_feed();
    return true;
}

bool asm_fsm_read_stable(uint8_t cs)
{
    uint8_t st = asm_reg_read(cs, ASM_REG_FSM_STATUS_A);
    return (st & 0x01U) != 0U;
}

static int16_t s_frozen_pgx[2] = {32767, 32767};
static int16_t s_frozen_pgy[2] = {32767, 32767};
static int16_t s_frozen_pgz[2] = {32767, 32767};
static uint8_t s_frozen_cnt[2] = {0U, 0U};

void asm_frozen_burst_reset(uint8_t cs)
{
    uint8_t idx = (cs == IMU_A_CS_PIN) ? 0U : 1U;
    s_frozen_pgx[idx] = 32767;
    s_frozen_pgy[idx] = 32767;
    s_frozen_pgz[idx] = 32767;
    s_frozen_cnt[idx] = 0U;
}

bool asm_read_sensors(uint8_t cs,
                      float &ax_g,  float &ay_g,  float &az_g,
                      float &gx_rs, float &gy_rs, float &gz_rs)
{
    uint8_t status = asm_reg_read(cs, ASM_REG_STATUS_REG);
    if (status == 0xFFU) return false;
    if ((status & 0x03U) != 0x03U) {
        delayMicroseconds(50U);
        status = asm_reg_read(cs, ASM_REG_STATUS_REG);
        if (status == 0xFFU || (status & 0x03U) != 0x03U) return false;
    }

    uint8_t buf[12];
    asm_burst_read(cs, ASM_REG_OUTX_L_G, buf, 12);

    int16_t gx_r = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    int16_t gy_r = (int16_t)((uint16_t)buf[3]  << 8 | buf[2]);
    int16_t gz_r = (int16_t)((uint16_t)buf[5]  << 8 | buf[4]);
    int16_t ax_r = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    int16_t ay_r = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    int16_t az_r = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);

    {
        uint8_t idx = (cs == IMU_A_CS_PIN) ? 0U : 1U;
        if (gx_r == s_frozen_pgx[idx] && gy_r == s_frozen_pgy[idx] && gz_r == s_frozen_pgz[idx]) {
            if (s_frozen_cnt[idx] < 255U) s_frozen_cnt[idx]++;
            if (s_frozen_cnt[idx] >= FROZEN_BURST_TICKS) return false;
        } else {
            s_frozen_cnt[idx] = 0U;
            s_frozen_pgx[idx] = gx_r;
            s_frozen_pgy[idx] = gy_r;
            s_frozen_pgz[idx] = gz_r;
        }
    }

    ax_g  = (float)ax_r * ASM_ACCEL_SENS_G;
    ay_g  = (float)ay_r * ASM_ACCEL_SENS_G;
    az_g  = (float)az_r * ASM_ACCEL_SENS_G;
    gx_rs = (float)gx_r * ASM_GYRO_SENS_DPS * DEG_TO_RAD;
    gy_rs = (float)gy_r * ASM_GYRO_SENS_DPS * DEG_TO_RAD;
    gz_rs = (float)gz_r * ASM_GYRO_SENS_DPS * DEG_TO_RAD;
    return true;
}
