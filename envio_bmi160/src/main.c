/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Custom switch for HA profile implementation.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <ram_pwrdn.h>
#include <zephyr/drivers/sensor.h>

#include <stdio.h>


#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
#include "zb_mem_config_custom.h"
#include "zb_dimmer_switch.h"
#include "zb_zcl_on_off.h"
#include <zcl/zb_zcl_custom_cluster.h>

#if CONFIG_ZIGBEE_FOTA
#include <zigbee/zigbee_fota.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/dfu/mcuboot.h>

/* LED indicating OTA Client Activity. */
#define OTA_ACTIVITY_LED          DK_LED2
#endif /* CONFIG_ZIGBEE_FOTA */

/* 4 bytes memory aligned -> arch 32 bits. Buffer capacity: 128 bytes*/
unsigned char __aligned(4) pipe_ring_buffer[128];
struct k_pipe ipc_pipe;

/*Task used to recollect sensor data*/
static void bmi160Task(void *arg1, void *arg2, void *arg3);
K_THREAD_STACK_DEFINE(bmi160_stack_area, 2048);
struct k_thread bmi160_thread_data;


/* Source endpoint used to control light bulb. */
#define LIGHT_SWITCH_ENDPOINT      1
/* Delay between the light switch startup and light bulb finding procedure. */
#define MATCH_DESC_REQ_START_DELAY K_SECONDS(2)
/* Timeout for finding procedure. */
#define MATCH_DESC_REQ_TIMEOUT     K_SECONDS(5)
/* Find only non-sleepy device. */
#define MATCH_DESC_REQ_ROLE        ZB_NWK_BROADCAST_RX_ON_WHEN_IDLE

/* Do not erase NVRAM to save the network parameters after device reboot or
 * power-off. NOTE: If this option is set to ZB_TRUE then do full device erase
 * for all network devices before running other samples.
 */
#define ERASE_PERSISTENT_CONFIG    ZB_FALSE
/* LED indicating that light switch successfully joind Zigbee network. */
#define ZIGBEE_NETWORK_STATE_LED   DK_LED3
/* LED used for device identification. */
#define IDENTIFY_LED               ZIGBEE_NETWORK_STATE_LED
/* LED indicating that light witch found a light bulb to control. */
#define BULB_FOUND_LED             DK_LED4
/* Button ID used to find matched clusters. */
#define BUTTON_FIND_MATCH                 DK_BTN2_MSK
/* Button ID used to send blinded devices. */
#define BUTTON_SEND_BLIND              DK_BTN3_MSK
/* Dim step size - increases/decreses current level (range 0x000 - 0xfe). */
#define DIMM_STEP                  15

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON       DK_BTN4_MSK

/* Button used to enter the Identify mode. */
#define IDENTIFY_MODE_BUTTON       DK_BTN4_MSK

/* Transition time for a single step operation in 0.1 sec units.
 * 0xFFFF - immediate change.
 */
#define DIMM_TRANSACTION_TIME      2

/* Version of the application software (1 byte). */
#define BULB_INIT_BASIC_APP_VERSION     01

/* Version of the implementation of the Zigbee stack (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION   10

/* Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION      11

/* Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MANUF_NAME      "Nordic"

/* Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_MODEL_ID        "Custom_v0.1"

#define BULB_MESSAGE        "Hola Manu"

/* First 8 bytes specify the date of manufacturer of the device
 * in ISO 8601 format (YYYYMMDD). The rest (8 bytes) are manufacturer specific.
 */
#define BULB_INIT_BASIC_DATE_CODE       "20200329"

/* Type of power sources available for the device.
 * For possible values see section 3.2.2.2.8 of ZCL specification.
 */
#define BULB_INIT_BASIC_POWER_SOURCE    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE

/* Describes the physical location of the device (16 bytes).
 * May be modified during commissioning process.
 */
#define BULB_INIT_BASIC_LOCATION_DESC   "Office desk"

/* Time after which the button state is checked again to detect button hold,
 * the dimm command is sent again.
 */
#define BUTTON_LONG_POLL_TMO       K_MSEC(500)

/*MAX found devices*/
#define MAX_DEVICES 5

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile light switch (End Device) source code.
#endif

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* Creates a struct to store: endpoint device and network address device */
struct bulb_context {
	zb_uint8_t endpoint;
	zb_uint16_t short_addr;
	struct k_timer find_alarm;
};

/* Create a stuct that contains some important values for buttons */
struct buttons_context {
	uint32_t state;
	atomic_t long_poll;
	struct k_timer alarm;
};

/* Adds attributes variables*/
struct zb_device_ctx {
	zb_zcl_basic_attrs_ext_t basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
	zb_zcl_on_off_attrs_t on_off_attr;
	float valores[3];
};


/* Defines structs*/
static struct bulb_context bulb_ctx;
static struct buttons_context buttons_ctx;
static struct zb_device_ctx dev_ctx;

/* Declare attribute list for Basic cluster (server). */
ZB_ZCL_DECLARE_BASIC_SERVER_ATTRIB_LIST(
	basic_server_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id);

/* Declare attribute list for Identify cluster (client). */
ZB_ZCL_DECLARE_IDENTIFY_CLIENT_ATTRIB_LIST(
	identify_client_attr_list);

/* Declare attribute list for Identify cluster (server). */
ZB_ZCL_DECLARE_IDENTIFY_SERVER_ATTRIB_LIST(
	identify_server_attr_list,
	&dev_ctx.identify_attr.identify_time);

/* Declare attribute list for Scenes cluster (client). */
ZB_ZCL_DECLARE_SCENES_CLIENT_ATTRIB_LIST(
	scenes_client_attr_list);

/* Declare attribute list for Groups cluster (client). */
ZB_ZCL_DECLARE_GROUPS_CLIENT_ATTRIB_LIST(
	groups_client_attr_list);

/* Declare attribute list for On/Off cluster (client). */
ZB_ZCL_DECLARE_ON_OFF_CLIENT_ATTRIB_LIST(
	on_off_client_attr_list);

/* Declare attribute list for Custom cluster (?). */
ZB_ZCL_DECLARE_CUSTOM_ATTR_CLUSTER_ATTRIB_LIST(
	custom_server_attr_list,
	ZB_ZCL_CUSTOM_CLUSTER_ATTR_U8_DEFAULT_VALUE,
	ZB_ZCL_CUSTOM_CLUSTER_ATTR_S16_DEFAULT_VALUE,
	0,
	dev_ctx.valores,
	0,
	0,
	ZB_ZCL_CUSTOM_CLUSTER_ATTR_UTC_TIME_DEFAULT_VALUE,
	0,
	ZB_ZCL_CUSTOM_CLUSTER_ATTR_BOOL_DEFAULT_VALUE,
	0);

/* Declare attribute list for Level control cluster (client). */
ZB_ZCL_DECLARE_LEVEL_CONTROL_CLIENT_ATTRIB_LIST(
	level_control_client_attr_list);

/* Declare cluster list for Dimmer Switch device. */
ZB_DECLARE_DIMMER_SWITCH_CLUSTER_LIST(
	dimmer_switch_clusters,
	basic_server_attr_list,
	identify_client_attr_list,
	identify_server_attr_list,
	scenes_client_attr_list,
	groups_client_attr_list,
	on_off_client_attr_list,
	level_control_client_attr_list,
	custom_server_attr_list);

/* Declare endpoint for Dimmer Switch device. */
ZB_DECLARE_DIMMER_SWITCH_EP(
	dimmer_switch_ep,
	LIGHT_SWITCH_ENDPOINT,
	dimmer_switch_clusters);

/* Defines global counter and global devices*/
int cont = 0;
zb_uint16_t dispositivos [MAX_DEVICES];


/* Declare application's device context (list of registered endpoints)
 * for Dimmer Switch device.
 */
#ifndef CONFIG_ZIGBEE_FOTA
ZBOSS_DECLARE_DEVICE_CTX_1_EP(dimmer_switch_ctx, dimmer_switch_ep);
#else

  #if LIGHT_SWITCH_ENDPOINT == CONFIG_ZIGBEE_FOTA_ENDPOINT
    #error "Light switch and Zigbee OTA endpoints should be different."
  #endif

extern zb_af_endpoint_desc_t zigbee_fota_client_ep;
ZBOSS_DECLARE_DEVICE_CTX_2_EP(dimmer_switch_ctx,
			      zigbee_fota_client_ep,
			      dimmer_switch_ep);
#endif /* CONFIG_ZIGBEE_FOTA */

/* Forward declarations. */
static void find_devices_send(zb_uint8_t param);
static void send_cmd(zb_bufid_t bufid, zb_uint16_t cmd_id);
static void send_cmd_blinded(zb_bufid_t bufid);


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

			zb_ret_t zb_err_code = zb_bdb_finding_binding_target(LIGHT_SWITCH_ENDPOINT);

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
 * @param[in]   button_state  Bitmask containing buttons state.
 * @param[in]   has_changed   Bitmask containing buttons that has
 *                            changed their state.
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	zb_uint16_t cmd_id;
	zb_ret_t zb_err_code;

	/* Inform default signal handler about user input at the device. */
	user_input_indicate();

	check_factory_reset_button(button_state, has_changed);


	switch (has_changed) {
	case BUTTON_FIND_MATCH:
		LOG_DBG("FIND_MATCH - button changed");

		break;

	case BUTTON_SEND_BLIND:
		LOG_DBG("SEND_BLIND - button changed");
	break;

	case IDENTIFY_MODE_BUTTON:
		if (IDENTIFY_MODE_BUTTON & button_state) {
			/* Button changed its state to pressed */
		} else {
			/* Button changed its state to released */
			if (was_factory_reset_done()) {
				/* The long press was for Factory Reset */
				LOG_DBG("After Factory Reset - ignore button release");
			} else   {
				/* Button released before Factory Reset */

				/* Start identification mode */
				ZB_SCHEDULE_APP_CALLBACK(start_identifying, 0);
			}
		}
		return;
	default:
		LOG_DBG("Unhandled button");
		return;
	}

	switch (button_state) {
	/*Finds match devices calling to a function*/
	case BUTTON_FIND_MATCH:
		LOG_DBG("Button pressed");
		buttons_ctx.state = button_state;

		zb_err_code = zb_buf_get_out_delayed_ext(
			find_devices_send, cmd_id, 0);
		ZB_ERROR_CHECK(zb_err_code);


		/* Alarm can be scheduled only once. Next alarm only resets
		 * counting.
		 */
		k_timer_start(&buttons_ctx.alarm, BUTTON_LONG_POLL_TMO,
			      K_NO_WAIT);
		break;
	
	case BUTTON_SEND_BLIND:
	/*Send a specific command to a blind device*/
		LOG_DBG("Button pressed");
		buttons_ctx.state = button_state;

		zb_err_code = zb_buf_get_out_delayed_ext(
			send_cmd_blinded, cmd_id, 0);
		ZB_ERROR_CHECK(zb_err_code);

		break;
	case 0:
		LOG_DBG("Button released");

		k_timer_stop(&buttons_ctx.alarm);

		if (atomic_set(&buttons_ctx.long_poll, ZB_FALSE) == ZB_FALSE) {
			/* Allocate output buffer and send on/off command. */
		}
		break;
	default:
		break;
	}
}

/**@brief Function for initializing LEDs and Buttons. */
static void configure_gpio(void)
{
	int err;

	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}


/**@brief Function for initializing all clusters attributes. */
static void app_clusters_attr_init(void)
{
	/* Basic cluster attributes data. */
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.mf_name,
		BULB_INIT_BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.model_id,
		BULB_INIT_BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));
	
}

/**@brief Function to toggle the identify LED.
 *
 * @param  bufid  Unused parameter, required by ZBOSS scheduler API.
 */
static void toggle_identify_led(zb_bufid_t bufid)
{
	static int blink_status;

	dk_set_led(IDENTIFY_LED, (++blink_status) % 2);
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
		/* Cancel the toggling function alarm and turn off LED. */
		zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(toggle_identify_led, ZB_ALARM_ANY_PARAM);
		ZVUNUSED(zb_err_code);

		/* Update network status/idenitfication LED. */
		if (ZB_JOINED()) {
			dk_set_led_on(ZIGBEE_NETWORK_STATE_LED);
		} else {
			dk_set_led_off(ZIGBEE_NETWORK_STATE_LED);
		}
	}
}

/*Response callback (match devices request)*/
void find_devices_cb(zb_uint8_t param)
{
	zb_bufid_t buf = param;
	zb_zdo_match_desc_resp_t *resp = (zb_zdo_match_desc_resp_t*)zb_buf_begin(buf);
	zb_uint8_t *match_ep;
	
	LOG_INF(">> find_light_bulb_cb param %hd, resp match_len %hd", param, resp->match_len);
	
	if (resp->status == ZB_ZDP_STATUS_SUCCESS && resp->match_len > 0) /*If we've found a match devices*/
	{
		LOG_INF("Server is found, continue normal work...");
	
		/* Match EP list follows right after response header */
		match_ep = (zb_uint8_t*)(resp + 1);
	
		/* we are searching for exact cluster, so only 1 EP maybe found */
		bulb_ctx.endpoint = *match_ep;
		bulb_ctx.short_addr = resp->nwk_addr;
		
		LOG_INF("find bulb addr 0x%hx ep %hd", bulb_ctx.short_addr, bulb_ctx.endpoint);


		/*Adds devices to list*/
		if(cont < MAX_DEVICES){
			dispositivos[cont] = bulb_ctx.short_addr;
			LOG_INF("Guardado dispositivo: %hx, %hx.", dispositivos[cont], bulb_ctx.short_addr);
			cont ++;
		}
		else{
			LOG_INF("Numero máximo alcanzado %hd.", cont);
		}
	
	}
	else{
		zb_buf_free(buf);
	}
}

/**@brief Allow to find match devices within a network
 *
 * @param[in]   param    Non-zero reference to Zigbee stack buffer that will be
 *                       used to construct match request.
 */
static void find_devices_send(zb_uint8_t param)
{
	zb_bufid_t buf = param;
 	zb_zdo_match_desc_param_t *req;
 
	LOG_INF(">> find_light_bulb %hd", param);
	
	/*Request*/
	req = zb_buf_initial_alloc(buf, sizeof(zb_zdo_match_desc_param_t) + (1) * sizeof(zb_uint16_t));
	
	req->nwk_addr = ZB_NWK_BROADCAST_RX_ON_WHEN_IDLE; //Non-sleepy devices address
	req->addr_of_interest = ZB_NWK_BROADCAST_RX_ON_WHEN_IDLE; //Non-sleepy devices address
	req->profile_id = ZB_AF_HA_PROFILE_ID; //Profile ID

	/* Searches devices that: */
	req->num_in_clusters = 1;// It has a server cluster
	req->num_out_clusters = 0;// It hasn't a client cluster
	req->cluster_list[0] = ZB_ZCL_CLUSTER_ID_CUSTOM; //The cluster has "Custom" ID
	
	zb_zdo_match_desc_req(param, find_devices_cb);//We send request and wait the answer (find_devices_cb = callback to add match devices)
	
	LOG_INF("<< find_light_bulb %hd", param);

}

/**@brief Allow send a write request to custom cluster
 *
 * @param[in]   bufid    Non-zero reference to Zigbee stack buffer that will be
 *                       used to construct on/off request.
 * @param[in]   cmd_id   Devices address.
 */
static void send_cmd(zb_bufid_t bufid, zb_uint16_t cmd_id)
{

		size_t bytes_read = 0;

		/*Gets sensor values through a pipe*/
		k_pipe_get(&ipc_pipe,
			&dev_ctx.valores,
			sizeof(dev_ctx.valores),
			&bytes_read,
			sizeof(dev_ctx.valores), // min bytes para leer. OJO!!
			K_FOREVER);

		

	
		zb_uint8_t *ptr;

		/*Initializates write request*/
		ZB_ZCL_GENERAL_INIT_WRITE_ATTR_REQ(bufid, ptr, ZB_ZCL_ENABLE_DEFAULT_RESPONSE);//Iniciamos petición de escritura


		/*Add:
			1. Atribute ID
			2. Type attribute (we select 32 BIT because double hasn't any implementation)
			3. Data value (we send three values: AX, AY, AZ; so we need do the same three times).*/
		int indice = 0;
		while(indice < 3){
			ZB_ZCL_GENERAL_ADD_VALUE_WRITE_ATTR_REQ(ptr, ZB_ZCL_CUSTOM_CLUSTER_ATTR_U32_ID,  ZB_ZCL_ATTR_TYPE_32BIT, &dev_ctx.valores[indice]);
			indice++;
		}



		/*Sends write request*/
		ZB_ZCL_GENERAL_SEND_WRITE_ATTR_REQ(bufid,
				ptr,
				cmd_id,
				ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
				1,
				LIGHT_SWITCH_ENDPOINT,
				ZB_AF_HA_PROFILE_ID,
				ZB_ZCL_CLUSTER_ID_CUSTOM,
				NULL);

}

/**@brief Allow send a on/off toggle command to on/off cluster
 *
 * @param[in]   bufid    Non-zero reference to Zigbee stack buffer that will be
 *                       used to construct on/off request.
 */
static void send_cmd_blinded(zb_bufid_t bufid){
	ZB_ZCL_ON_OFF_SEND_REQ(bufid,
	 		       bulb_ctx.short_addr,
	 		       ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
	 		       bulb_ctx.endpoint,
	 		       LIGHT_SWITCH_ENDPOINT,
	 		       ZB_AF_HA_PROFILE_ID,
	 		       ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
	 		       ZB_ZCL_CMD_ON_OFF_TOGGLE_ID,
	 		       NULL);
}


#ifdef CONFIG_ZIGBEE_FOTA
static void confirm_image(void)
{
	if (!boot_is_img_confirmed()) {
		int ret = boot_write_img_confirmed();

		if (ret) {
			LOG_ERR("Couldn't confirm image: %d", ret);
		} else {
			LOG_INF("Marked image as OK");
		}
	}
}

static void ota_evt_handler(const struct zigbee_fota_evt *evt)
{
	switch (evt->id) {
	case ZIGBEE_FOTA_EVT_PROGRESS:
		dk_set_led(OTA_ACTIVITY_LED, evt->dl.progress % 2);
		break;

	case ZIGBEE_FOTA_EVT_FINISHED:
		LOG_INF("Reboot application.");
		/* Power on unused sections of RAM to allow MCUboot to use it. */
		if (IS_ENABLED(CONFIG_RAM_POWER_DOWN_LIBRARY)) {
			power_up_unused_ram();
		}

		sys_reboot(SYS_REBOOT_COLD);
		break;

	case ZIGBEE_FOTA_EVT_ERROR:
		LOG_ERR("OTA image transfer failed.");
		break;

	default:
		break;
	}
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer
 *                      used to pass received data.
 */
static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_zcl_device_callback_param_t *device_cb_param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	if (device_cb_param->device_cb_id == ZB_ZCL_OTA_UPGRADE_VALUE_CB_ID) {
		zigbee_fota_zcl_cb(bufid);
	} else {
		device_cb_param->status = RET_NOT_IMPLEMENTED;
	}
}
#endif /* CONFIG_ZIGBEE_FOTA */

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sig_hndler = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &sig_hndler);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

	/* Update network status LED. */
	zigbee_led_status_update(bufid, ZIGBEE_NETWORK_STATE_LED);

#ifdef CONFIG_ZIGBEE_FOTA
	/* Pass signal to the OTA client implementation. */
	zigbee_fota_signal_handler(bufid);
#endif /* CONFIG_ZIGBEE_FOTA */

	switch (sig) {
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
	/* fall-through */
	case ZB_BDB_SIGNAL_STEERING:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;
	case ZB_ZDO_SIGNAL_LEAVE:
		/* If device leaves the network, reset bulb short_addr. */
		if (status == RET_OK) {
			zb_zdo_signal_leave_params_t *leave_params =
				ZB_ZDO_SIGNAL_GET_PARAMS(sig_hndler, zb_zdo_signal_leave_params_t);
		}
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;

	default:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;
	}

	if (bufid) {
		zb_buf_free(bufid);
	}
}


int main(void)
{
	LOG_INF("Starting ZBOSS Light Switch example");

	/* Initialize. */
	configure_gpio();
	register_factory_reset_button(FACTORY_RESET_BUTTON);

	zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);
	zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
	zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

	/* Set default bulb short_addr. */
	bulb_ctx.short_addr = 0xFFFF;

	/* Pipe initialization */ 
	k_pipe_init(&ipc_pipe, pipe_ring_buffer, sizeof(pipe_ring_buffer));
	
	/* thread (task) initialization */
	(void) k_thread_create(&bmi160_thread_data,
			bmi160_stack_area,
			K_THREAD_STACK_SIZEOF(bmi160_stack_area),
			bmi160Task,
			NULL, NULL, NULL,
			-1, // cooperative prio
			0, // no args
			K_NO_WAIT);

	/* Power off unused sections of RAM to lower device power consumption. */
	if (IS_ENABLED(CONFIG_RAM_POWER_DOWN_LIBRARY)) {
		power_down_unused_ram();
	}

#ifdef CONFIG_ZIGBEE_FOTA
	/* Initialize Zigbee FOTA download service. */
	zigbee_fota_init(ota_evt_handler);

	/* Mark the current firmware as valid. */
	confirm_image();

	/* Register callback for handling ZCL commands. */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);
#endif /* CONFIG_ZIGBEE_FOTA */

	/* Register dimmer switch device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&dimmer_switch_ctx);

	app_clusters_attr_init();

	/* Register handlers to identify notifications */
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(LIGHT_SWITCH_ENDPOINT, identify_cb);
#ifdef CONFIG_ZIGBEE_FOTA
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(CONFIG_ZIGBEE_FOTA_ENDPOINT, identify_cb);
#endif /* CONFIG_ZIGBEE_FOTA */

	/* Start Zigbee default thread. */
	zigbee_enable();

	LOG_INF("ZBOSS Light Switch example started");

	while (1) {
		k_sleep(K_FOREVER);
	}
}

/*Task used to recollect sensor data*/
static void bmi160Task(void *arg1, void *arg2, void *arg3){
	/* Configuration BMI160*/
	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi160);
	struct sensor_value full_scale, sampling_freq, oversampling;
	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return 0;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	struct sensor_value acc[3], gyr[3];
	uint16_t bytes_written = 0;


	/* Setting scale in G, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
	full_scale.val1 = 16;            /* G */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;       /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;          /* Normal mode */
	oversampling.val2 = 0;


	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq);


	/* Setting scale in degrees/s to match the sensor scale */
	full_scale.val1 = 2000;          /* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = 100;       /* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;          /* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change sampling frequency to
	 * 0.0Hz before changing other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq);

	int rc;

	while(1){
		/* 1000ms period, 1Hz Sampling frequency */
		k_sleep(K_MSEC(250));

		sensor_sample_fetch(dev);

		/*Get acceleration and angular velocity*/
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,  gyr);

	

		LOG_INF("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
		       "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d;\n",
		       acc[0].val1, acc[0].val2,
		       acc[1].val1, acc[1].val2,
		       acc[2].val1, acc[2].val2,
		       gyr[0].val1, gyr[0].val2,
		       gyr[1].val1, gyr[1].val2,
		       gyr[2].val1, gyr[2].val2);

		/*Cast sensor values to double*/
		float acc_double[3];
		for(int i = 0; i < 3; i++){
			acc_double[i] = acc[i].val1 * 1.0 + acc[i].val2 * 0.000001 ;
			
		}

		

		/*If we have match devices (cont > 0), we'll send values through pipe to "send cmd" functions*/
		for (int i = 0 ; i < cont; i++){

			rc =  k_pipe_put(&ipc_pipe, // pipe
				&acc_double, // point to data
				sizeof(acc_double), // size of data
				&bytes_written, // bytes written
				1, // min bytes to write
				K_NO_WAIT);
			
			LOG_INF("Dispositivo %hd: 0x%hx.", i, dispositivos[i]);
			zb_buf_get_out_delayed_ext( //Call "send cmd" function
				send_cmd, dispositivos[i], 0);
		}
	 }
}
