#include <drivers/i2c.h>
#include <logging/log.h>

#include "lis2dw12_reg.h"
#include "lis2dw12.h"

LOG_MODULE_REGISTER(lis2dw12, CONFIG_SIDEWALK_LOG_LEVEL);

static struct i2c_dt_spec i2c_lis2dw12 = I2C_DT_SPEC_GET(DT_NODELABEL(lis2dw12));
static struct gpio_dt_spec lis2dw12_irq1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(gpiocus3), gpios, {0});
static struct gpio_dt_spec lis2dw12_irq2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(gpiocus4), gpios, {0});

static struct gpio_callback lis2dw12_irq1_cb_data;
static struct gpio_callback lis2dw12_irq2_cb_data;
static lis2dw12_irq1_callback_t lis2dw12_irq1_callback;
static lis2dw12_irq2_callback_t lis2dw12_irq2_callback;

extern bool sid_send_msg;

static void lis2dw12_irq2_cb_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (lis2dw12_irq2_callback != NULL) {
        lis2dw12_irq2_callback(dev, cb, pins);
    }
}

static void lis2dw12_irq1_cb_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (lis2dw12_irq1_callback != NULL) {
        lis2dw12_irq1_callback(dev, cb, pins);
    }
}

static int lis2dw12_read(stmdev_ctx_t *ctx, uint8_t address,
				  uint8_t *p_data, uint32_t len)
{
	return i2c_burst_read_dt(&i2c_lis2dw12, address, p_data, len);
}

static int lis2dw12_write(stmdev_ctx_t *ctx, uint8_t address,
				  uint8_t *p_data, uint32_t len)
{
    return i2c_burst_write_dt(&i2c_lis2dw12, address, p_data, len);
}

stmdev_ctx_t lis2dw12_ctx = {
	.read_reg = (stmdev_read_ptr) lis2dw12_read,
	.write_reg = (stmdev_write_ptr) lis2dw12_write,
};

void lis2dw12_set_irq1_callback(lis2dw12_irq1_callback_t callback)
{
    lis2dw12_irq1_callback = callback;
}

int lis2dw12_get_interrupt_status(uint8_t *status)
{
    int ret = 0;
    stmdev_ctx_t *ctx = &lis2dw12_ctx;

    ret = lis2dw12_read_reg(ctx, LIS2DW12_ALL_INT_SRC, status, 1);
    if (ret != 0) {
        LOG_ERR("lis2dw12 read status reg error: %d", ret);
        return ret;
    }
	
	return ret;
}

int lis2dw12_read_acceleration(int16_t *accel_buf)
{
    int ret = 0;
    stmdev_ctx_t *ctx = &lis2dw12_ctx;
    int16_t t_buf[3];

	/* fetch raw data sample */
    ret = lis2dw12_acceleration_raw_get(ctx, t_buf);
    if (ret != 0) {
        LOG_ERR("lis2dw12 acceleration raw get error: %d", ret);
        return ret;
    }

	accel_buf[0] = t_buf[0] >> 4;
	accel_buf[1] = t_buf[1] >> 4;
	accel_buf[2] = t_buf[2] >> 4;

	return ret;
}

int lis2dw12_trigger_mode(lis2dw12_trigger_mode_t trigger_mode, trigger_level_t level)
{
    int ret = 0;
    stmdev_ctx_t *ctx = &lis2dw12_ctx;
    lis2dw12_reg_t int_route;

	uint8_t level_tap = 0;	
	uint8_t level_freefall = 0;	
	uint8_t level_6d = 0;	
    bool en = 0;
	
	// int_route.ctrl4_int1_pad_ctrl.int1_drdy = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_fth = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_diff5 = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_tap = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_ff = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_wu = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_single_tap = 0;
    // int_route.ctrl4_int1_pad_ctrl.int1_6d = 0;

    ret = lis2dw12_pin_int1_route_get(ctx, &int_route.ctrl4_int1_pad_ctrl);
    if (ret != 0) {
        LOG_ERR("lis2dw12 pin int1 route get error: %d", ret);
        return ret;
    }
	/* 
		level_tap: 0~31, ±2 g on x,y,z direction
	*/
	switch(level){
		case OFF:
			en = false;
			break;
		case HIGH:
			en = true;
			level_tap = 5;
			level_freefall = LIS2DW12_FF_TSH_7LSb_FS2g;
			level_6d = 3;
			break;
		case MEDIUM:
			en = true;
			level_tap = 10;
			level_freefall = LIS2DW12_FF_TSH_11LSb_FS2g;
			level_6d = 2;
			break;
		case LOW:
			en = true;
			level_tap = 15;
			level_freefall = LIS2DW12_FF_TSH_15LSb_FS2g;
			level_6d = 1;
			break;
		default:
			break;
	}

	switch(trigger_mode){
		case TRIGGER_SINGLE_TAP:
		case TRIGGER_DOUBLE_TAP:
            ret = lis2dw12_tap_threshold_x_set(ctx, level_tap);
            if (ret != 0) {
                LOG_ERR("lis2dw12 tap threshold x set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_tap_threshold_y_set(ctx, level_tap);
            if (ret != 0) {
                LOG_ERR("lis2dw12 tap threshold y set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_tap_threshold_z_set(ctx, level_tap);
            if (ret != 0) {
                LOG_ERR("lis2dw12 tap threshold z set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_tap_detection_on_x_set(ctx, en);
            if (ret != 0) {
                LOG_ERR("lis2dw12 tap detection on x set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_tap_detection_on_y_set(ctx, en);
            if (ret != 0) {
                LOG_ERR("lis2dw12 tap detection on y set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_tap_detection_on_z_set(ctx, en);
            if (ret != 0) {
                LOG_ERR("lis2dw12 tap detection on z set error: %d", ret);
                return ret;
            }

			if(trigger_mode == TRIGGER_SINGLE_TAP){
				int_route.ctrl4_int1_pad_ctrl.int1_single_tap = en;

                ret = lis2dw12_tap_mode_set(ctx, LIS2DW12_ONLY_SINGLE);
                if (ret != 0) {
                    LOG_ERR("lis2dw12 tap mode set error: %d", ret);
                    return ret;
                }		
			}else if(trigger_mode == TRIGGER_DOUBLE_TAP){
				int_route.ctrl4_int1_pad_ctrl.int1_tap = en;

                ret = lis2dw12_tap_mode_set(ctx, LIS2DW12_BOTH_SINGLE_DOUBLE);
                if (ret != 0) {
                    LOG_ERR("lis2dw12 tap mode set error: %d", ret);
                    return ret;
                }

                ret = lis2dw12_tap_shock_set(ctx, 0);
                if (ret != 0) {
                    LOG_ERR("lis2dw12 tap shock set error: %d", ret);
                    return ret;
                }

                ret = lis2dw12_tap_dur_set(ctx, 0);
                if (ret != 0) {
                    LOG_ERR("lis2dw12 tap dur set error: %d", ret);
                    return ret;
                }

                ret = lis2dw12_tap_quiet_set(ctx, 0);
                if (ret != 0) {
                    LOG_ERR("lis2dw12 tap quiet set error: %d", ret);
                    return ret;
                }
			}
			break;
		case TRIGGER_FREE_FALL:
			int_route.ctrl4_int1_pad_ctrl.int1_ff = en;

			// 1 LSB = 1 * 1/ODR, LSB:0~31
            ret = lis2dw12_ff_dur_set(ctx, 0);
            if (ret != 0) {
                LOG_ERR("lis2dw12 ff dur set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_ff_threshold_set(ctx, level_freefall);
            if (ret != 0) {
                LOG_ERR("lis2dw12 ff threshold set error: %d", ret);
                return ret;
            }
			break;
		case TRIGGER_6D_ORIENTATE:
			int_route.ctrl4_int1_pad_ctrl.int1_6d = en;

			// // 4D detection portrait/landscape position enable.
			// if (lis2dw12_4d_mode_set(data->ctx, 1) != LIS2DW12_OK) {
			// 	LOG_ERR("lis2dw12_4d_mode_set fail");
			// 	return LIS2DW12_FAIL;
			// }

			/* 6D_THS[1:0] | Threshold decoding (degrees)
				   00      |       6 (80 degrees)  
				   01      |	   11 (70 degrees) 
				   10      |	   16 (60 degrees) 
				   11      |	   21 (50 degrees) 
			*/
            ret = lis2dw12_6d_threshold_set(ctx, level_6d);
            if (ret != 0) {
                LOG_ERR("lis2dw12 6d threshold set error: %d", ret);
                return ret;
            }

            ret = lis2dw12_6d_feed_data_set(ctx, LIS2DW12_ODR_DIV_2_FEED);
            if (ret != 0) {
                LOG_ERR("lis2dw12 6d feed data set error: %d", ret);
                return ret;
            }
			break;
		default:
			break;
	}
    
    ret = lis2dw12_pin_int1_route_set(ctx, &int_route.ctrl4_int1_pad_ctrl);
    if (ret != 0) {
        LOG_ERR("lis2dw12 pin int1 route set error: %d", ret);
        return ret;
    }

    return ret;
}

int lis2dw12_get_id(void)
{
    int ret = 0;
    stmdev_ctx_t *ctx = &lis2dw12_ctx;
    uint8_t chip_id = 0;

    ret = lis2dw12_device_id_get(ctx, &chip_id);
    if (ret != 0) {
        LOG_ERR("lis2dw12 get device id error: %d", ret);
        return ret;
    }

    return chip_id;
}

int lis2dw12_reset(void)
{
    int ret = 0;
    stmdev_ctx_t *ctx = &lis2dw12_ctx;

    ret = lis2dw12_reset_set(ctx, PROPERTY_ENABLE);
	if (ret != 0) {
		LOG_ERR("lis2dw12 reset set error: %d", ret);
		return ret;
	}

    return ret;
}

int lis2dw12_init(void)
{
    int ret = 0;
    stmdev_ctx_t *ctx = &lis2dw12_ctx;
    
    ret = lis2dw12_block_data_update_set(ctx, PROPERTY_ENABLE);
    if (ret != 0) {
        LOG_ERR("lis2dw12 block data update set error: %d", ret);
        return ret;
    }

    /* set default odr and full scale for acc */
    ret = lis2dw12_data_rate_set(ctx, LIS2DW12_XL_ODR_12Hz5);
    if (ret != 0) {
        LOG_ERR("lis2dw12 data rate set error: %d", ret);
        return ret;
    }

    ret = lis2dw12_full_scale_set(ctx, LIS2DW12_2g);
    if (ret != 0) {
        LOG_ERR("lis2dw12 full scale set error: %d", ret);
        return ret;
    }

    ret = lis2dw12_fifo_mode_set(ctx, LIS2DW12_BYPASS_MODE);
    if (ret != 0) {
        LOG_ERR("lis2dw12 fifo mode error: %d", ret);
        return ret;
    }

    ret = lis2dw12_act_mode_set(ctx, LIS2DW12_NO_DETECTION);
    if (ret != 0) {
        LOG_ERR("lis2dw12 act mode error: %d", ret);
        return ret;
    }

    ret = lis2dw12_int_notification_set(ctx, LIS2DW12_INT_PULSED);
    if (ret != 0) {
        LOG_ERR("lis2dw12 interrupt notify set error: %d", ret);
        return ret;
    }

    ret = lis2dw12_pin_polarity_set(ctx, LIS2DW12_ACTIVE_HIGH);
    if (ret != 0) {
        LOG_ERR("lis2dw12 pin polarity set error: %d", ret);
        return ret;
    }

    ret = lis2dw12_power_mode_set(ctx, LIS2DW12_CONT_LOW_PWR_12bit);
    if (ret != 0) {
        LOG_ERR("lis2dw12 power mode set error: %d", ret);
        return ret;
    }

    // /* sleep setting */
    // ret = lis2dw12_act_sleep_dur_set(ctx, 5);
    // if (ret != 0) {
    //     LOG_ERR("lis2dw12 act sleep dur set error: %d", ret);
    //     return ret;
    // }

    // /* wake up setting */
    // ret = lis2dw12_wkup_feed_data_set(ctx, LIS2DW12_HP_FEED);
    // if (ret != 0) {
    //     LOG_ERR("lis2dw12 wkup feed data set error: %d", ret);
    //     return ret;
    // }

    // ret = lis2dw12_wkup_dur_set(ctx, 5);
    // if (ret != 0) {
    //     LOG_ERR("lis2dw12 wkup dur set error: %d", ret);
    //     return ret;
    // }

    // ret = lis2dw12_wkup_threshold_set(ctx, 10);
    // if (ret != 0) {
    //     LOG_ERR("lis2dw12 wkup threshold set error: %d", ret);
    //     return ret;
    // }

    /* lis2dw12 gpios-irq1 callback init */
    ret = gpio_pin_configure_dt(&lis2dw12_irq1, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("gpio pin configure dt lis2dw12 irq1 pin error: %d", ret);
		return ret;
	}

    ret = gpio_pin_interrupt_configure_dt(&lis2dw12_irq1, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("gpio pin interrupt configure dt lis2dw12 irq1 pin error: %d", ret);
		return ret;
	}

    gpio_init_callback(&lis2dw12_irq1_cb_data, lis2dw12_irq1_cb_handler, BIT(lis2dw12_irq1.pin));
    gpio_add_callback(lis2dw12_irq1.port, &lis2dw12_irq1_cb_data);

    /* lis2dw12 gpios-irq2 callback init */
    ret = gpio_pin_configure_dt(&lis2dw12_irq2, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("gpio pin configure dt lis2dw12 irq2 pin error: %d", ret);
		return ret;
	}

    ret = gpio_pin_interrupt_configure_dt(&lis2dw12_irq2, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("gpio pin interrupt configure dt lis2dw12 irq2 pin error: %d", ret);
		return ret;
	}

    gpio_init_callback(&lis2dw12_irq2_cb_data, lis2dw12_irq2_cb_handler, BIT(lis2dw12_irq2.pin));
    gpio_add_callback(lis2dw12_irq2.port, &lis2dw12_irq2_cb_data);

    return ret;
}