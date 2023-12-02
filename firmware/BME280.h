#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"

#define READ_BIT 0x80

#define SENS_SCK_PIN 2
#define SENS_SDI_PIN 3
#define SENS_SDO_PIN 4
#define SENS_CSB_PIN 5

extern int32_t t_fine;

void bme280_init(void);
void read_registers(uint8_t reg, uint8_t *buf, uint16_t len);
void write_register(uint8_t reg, uint8_t data);
void bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature);
int32_t compensate_temp(int32_t adc_T);
uint32_t compensate_pressure(int32_t adc_P);
uint32_t compensate_humidity(int32_t adc_H);
void read_compensation_parameters();
static inline void cs_deselect();
static inline void cs_select();