#ifndef LIS2DW12_H
#define LIS2DW12_H

#include <drivers/gpio.h>

/* set interrupt trigger mode */
typedef enum {
  	TRIGGER_SINGLE_TAP,
	TRIGGER_DOUBLE_TAP,
	TRIGGER_FREE_FALL,
	TRIGGER_6D_ORIENTATE,
} lis2dw12_trigger_mode_t;

/* set trigger sensitivity */
typedef enum {
  	OFF,
	LOW,
	MEDIUM,
	HIGH,
} trigger_level_t;

typedef void (*lis2dw12_irq1_callback_t)(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
typedef void (*lis2dw12_irq2_callback_t)(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

void lis2dw12_set_irq1_callback(lis2dw12_irq1_callback_t callback);
void lis2dw12_set_irq2_callback(lis2dw12_irq2_callback_t callback);
int lis2dw12_get_interrupt_status(uint8_t *status);
int lis2dw12_read_acceleration(int16_t *accel_buf);
int lis2dw12_trigger_mode(lis2dw12_trigger_mode_t trigger_mode, trigger_level_t level);
int lis2dw12_get_id(void);
int lis2dw12_reset(void);
int lis2dw12_init(void);

#endif