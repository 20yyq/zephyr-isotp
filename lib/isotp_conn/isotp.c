/*
 * @Author       : Eacher
 * @Date         : 2024-07-18 15:32:33
 * @LastEditTime : 2024-07-19 10:52:30
 * @LastEditors  : Eacher
 * --------------------------------------------------------------------------------<
 * @Description  : 
 * --------------------------------------------------------------------------------<
 * @FilePath     : /zephyrproject/veryark/application/lib/isotp_conn/isotp.c
 */
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include <app/isotp/isotp_conn.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)

// GPIO_OUTPUT_ACTIVE == L == 1
// GPIO_OUTPUT_INACTIVE == H == 0
static const struct gpio_dt_spec __can_s_pin = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

LOG_MODULE_REGISTER(isotp_conn, CONFIG_VISOTP_CONN_LOG_LEVEL);

static struct {
	const struct device *dev;
	uint32_t bitrate;
} __can_dev = {.dev = NULL, .bitrate = ISOTP_CAN_BITRATE};

static void ___isotp_can_state_change_callback(const struct device *dev, enum can_state state,
					       struct can_bus_err_cnt err_cnt, void *user_data);

// 初始化CAN设备
int __init_can(void)
{
	// pm_power_hold(device_get_binding("pm"));
	if (!gpio_is_ready_dt(&__can_s_pin)) {
		LOG_ERR("CAN: Failed to gpio_is_ready_dt.");
		return -1;
	}
	if (!__can_dev.dev) {
		__can_dev.dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
		if (!device_is_ready(__can_dev.dev)) {
			__can_dev.dev = NULL;
			LOG_ERR("CAN: Failed to device driver not ready.");
			return -1;
		}
		can_mode_t mode = IS_ENABLED(CONFIG_SAMPLE_CAN_FD_MODE) ? CAN_MODE_FD : 0;
		if (can_set_mode(__can_dev.dev, mode)) {
			__can_dev.dev = NULL;
			LOG_ERR("CAN: Failed to set mode");
			return -1;
		}
		if (can_set_bitrate(__can_dev.dev, __can_dev.bitrate)) {
			__can_dev.dev = NULL;
			LOG_ERR("CAN: Failed to set bitrate");
			return -1;
		}
		if (can_start(__can_dev.dev)) {
			__can_dev.dev = NULL;
			LOG_ERR("CAN: Failed to start device");
			return -1;
		}
		can_set_state_change_callback(__can_dev.dev, ___isotp_can_state_change_callback,
					      NULL);
	}
	return 0;
}

static void ___isotp_can_state_change_callback(const struct device *dev, enum can_state state,
					       struct can_bus_err_cnt err_cnt, void *user_data)
{
	LOG_ERR("___isotp_can_state_change_callback state[%d] tx_err_cnt[%d] rx_err_cnt[%d]",
		state, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
	int ret;
	switch (state) {
	case CAN_STATE_ERROR_PASSIVE:
		if (gpio_pin_configure_dt(&__can_s_pin, GPIO_OUTPUT_INACTIVE)) {
			LOG_ERR("___isotp_can_state_change_callback: gpio_pin_configure_dt "
				"GPIO_OUTPUT_INACTIVE.");
		}
		break;
	case CAN_STATE_BUS_OFF:
		ret = can_start(__can_dev.dev);
		if (ret) {
			LOG_ERR("can_start error ret[%d]", ret);
		}
		if (gpio_pin_configure_dt(&__can_s_pin, GPIO_OUTPUT_ACTIVE)) {
			LOG_ERR("___isotp_can_state_change_callback: gpio_pin_configure_dt "
				"GPIO_OUTPUT_ACTIVE.");
		}
		break;
	case CAN_STATE_ERROR_ACTIVE:
	case CAN_STATE_ERROR_WARNING:
	case CAN_STATE_STOPPED:
		break;
	}
}

const struct device *get_can_dev(void)
{
	return __can_dev.dev;
}

int can_frame_send(uint8_t *data, uint8_t len) {
	struct can_frame frame;
	frame.flags = 0;
	frame.id = 283;
	frame.dlc = len;
	memcpy(&frame.data[0], data, frame.dlc);
	int ret = can_send(__can_dev.dev, &frame, K_MSEC(CONFIG_VISOTP_A_TIMEOUT), NULL, NULL);
	if (ret) {
		LOG_ERR("Can't send FC, (%d)", ret);
	}
	return ret;
}
