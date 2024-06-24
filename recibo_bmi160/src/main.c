/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @brief Simple Zigbee light bulb implementation.
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>
#include "zb_dimmable_light.h"
#include <zcl/zb_zcl_custom_cluster.h>

#include "lcd1602a.h"
#include <zephyr/drivers/i2c.h>
#include <stdio.h>

/* Memoria alienada a 4 bytes -> arch 32 bits. Capacidad buffer: 128 bytes*/
unsigned char __aligned(4) pipe_ring_buffer[128];
struct k_pipe ipc_pipe;

static void lcdTask(void *arg1, void *arg2, void *arg3);
K_THREAD_STACK_DEFINE(lcd_stack_area, 2048);
struct k_thread lcd_thread_data;

#define RUN_STATUS_LED	       DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

/* Device endpoint, used to receive light controlling commands. */
#define DIMMABLE_LIGHT_ENDPOINT 1

/* Version of the application software (1 byte). */
#define BULB_INIT_BASIC_APP_VERSION 01

/* Version of the implementation of the Zigbee stack (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION 10

/* Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION 11

/* Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MANUF_NAME "Nordic"

/* Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_MODEL_ID "Dimable_Light_v0.1"

/* First 8 bytes specify the date of manufacturer of the device
 * in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific.
 */
#define BULB_INIT_BASIC_DATE_CODE "20200329"

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define BULB_INIT_BASIC_POWER_SOURCE ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Describes the physical location of the device (16 bytes).
 * May be modified during commissioning process.
 */
#define BULB_INIT_BASIC_LOCATION_DESC "Office desk"

/* Describes the type of physical environment.
 * For possible values see section 3.2.2.2.10 of ZCL specification.
 */
#define BULB_INIT_BASIC_PH_ENV ZB_ZCL_BASIC_ENV_UNSPECIFIED

/* LED indicating that light switch successfully joind Zigbee network. */
#define ZIGBEE_NETWORK_STATE_LED DK_LED3

/* LED immitaing dimmable light bulb - define for informational
 * purposes only.
 */
#define BULB_LED DK_LED4

/* Button used to enter the Bulb into the Identify mode. */
#define IDENTIFY_MODE_BUTTON DK_BTN4_MSK

/* Use onboard led4 to act as a light bulb.
 * The app.overlay file has this at node label "pwm_led3" in /pwmleds.
 */
#define PWM_DK_LED4_NODE DT_NODELABEL(pwm_led3)

#if DT_NODE_HAS_STATUS(PWM_DK_LED4_NODE, okay)
static const struct pwm_dt_spec led_pwm = PWM_DT_SPEC_GET(PWM_DK_LED4_NODE);
#else
#error "Choose supported PWM driver"
#endif

/* Led PWM period, calculated for 100 Hz signal - in microseconds. */
#define LED_PWM_PERIOD_US (USEC_PER_SEC / 100U)

#ifndef ZB_ROUTER_ROLE
#error Define ZB_ROUTER_ROLE to compile router source code.
#endif

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON IDENTIFY_MODE_BUTTON

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* Main application customizable context.
 * Stores all settings and static values.
 */
typedef struct {
	zb_zcl_basic_attrs_ext_t basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
	zb_zcl_scenes_attrs_t scenes_attr;
	zb_zcl_groups_attrs_t groups_attr;
	zb_zcl_on_off_attrs_t on_off_attr;
	zb_zcl_level_control_attrs_t level_control_attr;
	unsigned char comando_attr[20];
} bulb_device_ctx_t;

/* Zigbee device application context storage. */
static bulb_device_ctx_t dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(groups_attr_list, &dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(scenes_attr_list, &dev_ctx.scenes_attr.scene_count,
				  &dev_ctx.scenes_attr.current_scene,
				  &dev_ctx.scenes_attr.current_group,
				  &dev_ctx.scenes_attr.scene_valid,
				  &dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list, &dev_ctx.basic_attr.zcl_version,
				     &dev_ctx.basic_attr.app_version,
				     &dev_ctx.basic_attr.stack_version,
				     &dev_ctx.basic_attr.hw_version, dev_ctx.basic_attr.mf_name,
				     dev_ctx.basic_attr.model_id, dev_ctx.basic_attr.date_code,
				     &dev_ctx.basic_attr.power_source,
				     dev_ctx.basic_attr.location_id, &dev_ctx.basic_attr.ph_env,
				     dev_ctx.basic_attr.sw_ver);

/* On/Off cluster attributes additions data */
ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(on_off_attr_list, &dev_ctx.on_off_attr.on_off);

ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(level_control_attr_list,
					 &dev_ctx.level_control_attr.current_level,
					 &dev_ctx.level_control_attr.remaining_time);

/* Declare attribute list for Custom cluster (Server). */
ZB_ZCL_DECLARE_CUSTOM_ATTR_CLUSTER_ATTRIB_LIST(custom_server_attr_list,
					       ZB_ZCL_CUSTOM_CLUSTER_ATTR_U8_DEFAULT_VALUE,
					       ZB_ZCL_CUSTOM_CLUSTER_ATTR_S16_DEFAULT_VALUE, 0,
					       ZB_ZCL_CUSTOM_CLUSTER_ATTR_32BITMAP_DEFAULT_VALUE, 0,
					       dev_ctx.comando_attr,
					       ZB_ZCL_CUSTOM_CLUSTER_ATTR_UTC_TIME_DEFAULT_VALUE, 0,
					       ZB_ZCL_CUSTOM_CLUSTER_ATTR_BOOL_DEFAULT_VALUE, 0);

ZB_DECLARE_DIMMABLE_LIGHT_CLUSTER_LIST(dimmable_light_clusters, basic_attr_list, identify_attr_list,
				       groups_attr_list, scenes_attr_list, on_off_attr_list,
				       level_control_attr_list, custom_server_attr_list);

ZB_DECLARE_DIMMABLE_LIGHT_EP(dimmable_light_ep, DIMMABLE_LIGHT_ENDPOINT, dimmable_light_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(dimmable_light_ctx, dimmable_light_ep);

/**@brief Starts identifying the device.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void start_identifying(zb_bufid_t bufid)
{
	ZVUNUSED(bufid);

	if (ZB_JOINED()) {
		/* Check if endpoint is in identifying mode,
		 * if not, put desired endpoint in identifying mode.
		 */
		if (dev_ctx.identify_attr.identify_time ==
		    ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE) {

			zb_ret_t zb_err_code =
				zb_bdb_finding_binding_target(DIMMABLE_LIGHT_ENDPOINT);

			if (zb_err_code == RET_OK) {
				LOG_INF("Enter identify mode");
			} else if (zb_err_code == RET_INVALID_STATE) {
				LOG_WRN("RET_INVALID_STATE - Cannot enter identify mode");
			} else {
				ZB_ERROR_CHECK(zb_err_code);
			}
		} else {
			LOG_INF("Cancel identify mode");
			zb_bdb_finding_binding_target_cancel();
		}
	} else {
		LOG_WRN("Device not in a network - cannot enter identify mode");
	}
}

/**@brief Callback for button events.
 *
 * @param[in]   button_state  Bitmask containing the state of the buttons.
 * @param[in]   has_changed   Bitmask containing buttons that have changed their state.
 */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	if (IDENTIFY_MODE_BUTTON & has_changed) {
		if (IDENTIFY_MODE_BUTTON & button_state) {
			/* Button changed its state to pressed */
		} else {
			/* Button changed its state to released */
			if (was_factory_reset_done()) {
				/* The long press was for Factory Reset */
				LOG_DBG("After Factory Reset - ignore button release");
			} else {
				/* Button released before Factory Reset */

				/* Start identification mode */
				ZB_SCHEDULE_APP_CALLBACK(start_identifying, 0);
			}
		}
	}

	check_factory_reset_button(button_state, has_changed);
}

/**@brief Function for initializing additional PWM leds. */
static void pwm_led_init(void)
{
	if (!device_is_ready(led_pwm.dev)) {
		LOG_ERR("Error: PWM device %s is not ready", led_pwm.dev->name);
	}
}

/**@brief Function for initializing LEDs and Buttons. */
static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	pwm_led_init();
}

/**@brief Sets brightness of bulb luminous executive element
 *
 * @param[in] brightness_level Brightness level, allowed values 0 ... 255,
 *                             0 - turn off, 255 - full brightness.
 */
static void light_bulb_set_brightness(zb_uint8_t brightness_level)
{
	uint32_t pulse = brightness_level * LED_PWM_PERIOD_US / 255U;

	if (pwm_set_dt(&led_pwm, PWM_USEC(LED_PWM_PERIOD_US), PWM_USEC(pulse))) {
		LOG_ERR("Pwm led 4 set fails:\n");
		return;
	}
}

/**@brief Function for setting the light bulb brightness.
 *
 * @param[in] new_level   Light bulb brightness value.
 */
static void level_control_set_value(zb_uint16_t new_level)
{
	LOG_INF("Set level value: %i", new_level);

	ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
			     ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
			     (zb_uint8_t *)&new_level, ZB_FALSE);

	light_bulb_set_brightness(new_level);
}

/**@brief Function for turning ON/OFF the light bulb.
 *
 * @param[in]   on   Boolean light bulb state.
 */
static void on_off_set_value(zb_bool_t on)
{
	LOG_INF("Set ON/OFF value: %i", on);

	ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_ON_OFF,
			     ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			     (zb_uint8_t *)&on, ZB_FALSE);

	if (on) {
		light_bulb_set_brightness(dev_ctx.level_control_attr.current_level);
	} else {
		light_bulb_set_brightness(0U);
	}
}

/**@brief Function to toggle the identify LED - BULB_LED is used for this.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void toggle_identify_led(zb_bufid_t bufid)
{
	static int blink_status;

	light_bulb_set_brightness(((++blink_status) % 2) ? (255U) : (0U));
	ZB_SCHEDULE_APP_ALARM(toggle_identify_led, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(100));
}

/**@brief Function to handle identify notification events on the first endpoint.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void identify_cb(zb_bufid_t bufid)
{
	zb_ret_t zb_err_code;

	if (bufid) {
		/* Schedule a self-scheduling function that will toggle the LED. */
		ZB_SCHEDULE_APP_CALLBACK(toggle_identify_led, bufid);
	} else {
		/* Cancel the toggling function alarm and restore current Zigbee LED state. */
		zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(toggle_identify_led, ZB_ALARM_ANY_PARAM);
		ZVUNUSED(zb_err_code);

		if (dev_ctx.on_off_attr.on_off) {
			light_bulb_set_brightness(dev_ctx.level_control_attr.current_level);
		} else {
			light_bulb_set_brightness(0U);
		}
	}
}

/**@brief Function for initializing all clusters attributes.
 */
static void bulb_clusters_attr_init(void)
{
	/* Basic cluster attributes data */
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version = BULB_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version = BULB_INIT_BASIC_HW_VERSION;

	/* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte
	 * should contain string length without trailing zero.
	 *
	 * For example "test" string will be encoded as:
	 *   [(0x4), 't', 'e', 's', 't']
	 */
	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.mf_name, BULB_INIT_BASIC_MANUF_NAME,
			      ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.model_id, BULB_INIT_BASIC_MODEL_ID,
			      ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.date_code, BULB_INIT_BASIC_DATE_CODE,
			      ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

	dev_ctx.basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

	ZB_ZCL_SET_STRING_VAL(dev_ctx.basic_attr.location_id, BULB_INIT_BASIC_LOCATION_DESC,
			      ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));

	dev_ctx.basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off cluster attributes data. */
	dev_ctx.on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;

	dev_ctx.level_control_attr.current_level = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
	dev_ctx.level_control_attr.remaining_time =
		ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;

	ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_ON_OFF,
			     ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			     (zb_uint8_t *)&dev_ctx.on_off_attr.on_off, ZB_FALSE);

	ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
			     ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
			     (zb_uint8_t *)&dev_ctx.level_control_attr.current_level, ZB_FALSE);

	ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_CUSTOM,
			     ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_CUSTOM_CLUSTER_ATTR_CHAR_STRING_ID,
			     NULL, ZB_FALSE);
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer
 *                      used to pass received data.
 */
static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_uint8_t cluster_id;
	zb_uint8_t attr_id;
	zb_zcl_device_callback_param_t *device_cb_param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	LOG_INF("%s id %hd", device_cb_param->device_cb_id);

	/* Set default response value. */
	device_cb_param->status = RET_OK;

	switch (device_cb_param->device_cb_id) {
	case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
		LOG_INF("Level control setting to %d",
			device_cb_param->cb_param.level_control_set_value_param.new_value);
		level_control_set_value(
			device_cb_param->cb_param.level_control_set_value_param.new_value);
		break;

	case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
		cluster_id = device_cb_param->cb_param.set_attr_value_param.cluster_id;
		attr_id = device_cb_param->cb_param.set_attr_value_param.attr_id;

		if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF) {
			uint8_t value = device_cb_param->cb_param.set_attr_value_param.values.data8;

			LOG_INF("on/off attribute setting to %hd", value);
			if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
				on_off_set_value((zb_bool_t)value);
			}
		} else if (cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
			uint16_t value =
				device_cb_param->cb_param.set_attr_value_param.values.data16;

			LOG_INF("level control attribute setting to %hd", value);
			if (attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) {
				level_control_set_value(value);
			}
		} else if (cluster_id == ZB_ZCL_CLUSTER_ID_CUSTOM) {
			zb_char_t value =
				device_cb_param->cb_param.set_attr_value_param.values.data16;

			LOG_INF("Set ON/OFF value: %c", value);

			ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_CUSTOM,
					     NULL, ZB_ZCL_CUSTOM_CLUSTER_ATTR_CHAR_STRING_ID,
					     (zb_char_t *)&value, ZB_FALSE);

		} else {
			/* Other clusters can be processed here */
			LOG_INF("Unhandled cluster attribute id: %d", cluster_id);
			device_cb_param->status = RET_NOT_IMPLEMENTED;
		}
		break;

	default:
		if (zcl_scenes_cb(bufid) == ZB_FALSE) {
			device_cb_param->status = RET_NOT_IMPLEMENTED;
		}
		break;
	}

	LOG_INF("%s status: %hd", __func__, device_cb_param->status);
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	/* Update network status LED. */
	zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

	/* No application-specific behavior is required.
	 * Call default signal handler.
	 */
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	/* All callbacks should either reuse or free passed buffers.
	 * If bufid == 0, the buffer is invalid (not passed).
	 */
	if (bufid) {
		zb_buf_free(bufid);
	}
}

zb_device_handler_t test_handler(zb_uint8_t param)
{

	char *str;
	uint16_t bytes_written = 0;
	int rc;

	LOG_INF("MANEJADOR");
	zb_zcl_write_attr_req_t *write_attr_req;

	zb_bufid_t zcl_cmd_buf = param;

	zb_zcl_parsed_hdr_t *cmd_info = ZB_BUF_GET_PARAM(zcl_cmd_buf, zb_zcl_parsed_hdr_t);

	LOG_INF("Cluster id: %hx", cmd_info->cluster_id);
	LOG_INF("CMD id: %hx", cmd_info->cmd_id);

	zb_uint8_t len = zb_buf_len(zcl_cmd_buf);
	zb_uint8_t *pin_payload = (zb_uint8_t *)zb_buf_begin(zcl_cmd_buf);

	if (cmd_info->cmd_id == 2) { // Peticion de escritura
		do {
			ZB_ZCL_GENERAL_GET_NEXT_WRITE_ATTR_REQ(pin_payload, len, write_attr_req);

			if (write_attr_req) {
				// LOG_INF("Atributte value: %s", write_attr_req->attr_value);
				str = write_attr_req->attr_value;
				// ZB_ZCL_STRING_TO_C_STRING(str);
				// LOG_INF("Buffer enviado id: %s, longitud: %hd.", str,
				// strlen(str));

				ZB_ZCL_SET_ATTRIBUTE(DIMMABLE_LIGHT_ENDPOINT, cmd_info->cluster_id,
						     ZB_ZCL_CLUSTER_SERVER_ROLE,
						     write_attr_req->attr_id, str, ZB_FALSE);

				uint8_t ndatos = sizeof(float); // strlen(str);
				rc = k_pipe_put(&ipc_pipe, (void *)&ndatos, sizeof(uint8_t),
						&bytes_written, sizeof(uint8_t), K_NO_WAIT);
				rc = k_pipe_put(&ipc_pipe,	// pipe destino
						(void *)str,	// ptero a datos
						ndatos,		// # datos a escribir
						&bytes_written, // # datos copiados (una vez la fx
								// ha terminado)
						1,		// min bytes to write
						K_NO_WAIT);

				if (rc < 0) {
					LOG_INF("No se ha enviado nada");
				} else {
					LOG_INF("Se ha enviado algo %hd", bytes_written);
				}
			}
			// TODO: Aqui k_pipe_put??

		} while (write_attr_req);
	}

	zb_buf_free(zcl_cmd_buf);

	return ZB_TRUE;
}

int main(void)
{
	int blink_status = 0;
	int err;

	LOG_INF("Starting ZBOSS Light Bulb example");

	/* Initialize */
	configure_gpio();
	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings initialization failed");
	}
	register_factory_reset_button(FACTORY_RESET_BUTTON);

	/* Register callback for handling ZCL commands. */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

	/* Register dimmer switch device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&dimmable_light_ctx);

	ZB_AF_SET_ENDPOINT_HANDLER(DIMMABLE_LIGHT_ENDPOINT, &test_handler);

	bulb_clusters_attr_init();
	level_control_set_value(dev_ctx.level_control_attr.current_level);

	/* Register handler to identify notifications. */
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(DIMMABLE_LIGHT_ENDPOINT, identify_cb);

	/* Initialize ZCL scene table */
	zcl_scenes_init();

	/* Settings should be loaded after zcl_scenes_init */
	err = settings_load();
	if (err) {
		LOG_ERR("settings loading failed");
	}

	/* Start Zigbee default thread */
	zigbee_enable();

	LOG_INF("ZBOSS Light Bulb example started");

	/* Pipe initialization */
	k_pipe_init(&ipc_pipe, pipe_ring_buffer, sizeof(pipe_ring_buffer));
	/* thread (task) initialization */
	(void)k_thread_create(&lcd_thread_data, lcd_stack_area,
			      K_THREAD_STACK_SIZEOF(lcd_stack_area), lcdTask, NULL, NULL, NULL,
			      -1, // cooperative prio
			      0,  // no args
			      K_NO_WAIT);

	while (1) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

static void lcdTask(void *arg1, void *arg2, void *arg3)
{

	const struct device *i2c_dev;

	i2c_dev = device_get_binding("I2C_1");
	if (!i2c_dev) {
		printk("Binding failed.");
		return;
	}
	i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

	// assert I2C activo
	// Ojo pues i2c_dev es global
	pcf857_LCDinit(i2c_dev);
	pcf857_Clear(i2c_dev);
	pcf857_LCDGOTO(i2c_dev);

		unsigned char buffer[100];
		float datos[3];
		int i = 0;

	while (1) {

		size_t bytes_read = 0;

		uint8_t ndatos;
		k_pipe_get(&ipc_pipe, &ndatos, sizeof(ndatos), &bytes_read,
			   sizeof(ndatos), // min bytes para leer. OJO!!
			   K_FOREVER);

		k_pipe_get(&ipc_pipe, (char *)&datos[i], ndatos, &bytes_read,
			   1, // min bytes para leer. OJO!!
			   K_FOREVER);
		// TODO: '\n' a buffer si no lo trae
		// LOG_INF("Buffer recibido id: %s, leidos: %hd, longitud: %hd", buffer, bytes_read,
		// strlen(buffer));
		// for(int i = bytes_read; i < strlen(buffer); i++){
		// 	buffer[i] = NULL;
		// }

		i++;

		if (3 == i) {
			i=0;
			LOG_INF("AX: %f; AY: %f; AZ: %f;\n", datos[0], datos[1], datos[2]);

			pcf857_Clear(i2c_dev);
			pcf857_LCDGOTO(i2c_dev);

			sprintf(buffer, "%.1f %.2f %.1f", datos[0], datos[1], datos[2]);
			sendString(i2c_dev, buffer);
			//k_msleep(4000);
			
		}
	}
}
