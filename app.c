/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
// Standard includes -----------------------------------------------------------
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"

// App includes ----------------------------------------------------------------
#include "app_log.h"
#include "sl_i2cspm_instances.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_rgb_pwm_led_instances.h"
#include "sl_simple_rgb_pwm_led.h"
#include "sl_simple_timer.h"
#include "sl_sleeptimer.h"
#include "sparkfun_max30101_max32664.h"
// Comment out to not output to Bluetooth
#include "gatt_db.h"
// Comment in to use light sensor and buttons instead of bio hub
//#include "sl_sensor_lux.h"

// App defines -----------------------------------------------------------------
#define BIO_HUB_TIMER_MS      500
#define PULSE_TIMER_START_MS  100
#define LED_TIMER_MS           10
#define LOG_SENSOR           0x01
#define LOG_BLUETOOTH        0x02

// Standard variables ----------------------------------------------------------
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// App variables ---------------------------------------------------------------
static uint8_t log_streams = (LOG_SENSOR | LOG_BLUETOOTH);
static sl_status_t bio_hub_status = SL_STATUS_FAIL;
static bio_hub_algo_mode_t algo_mode = BIO_HUB_ALGO_MODE_TWO;
static bool config_pw_sr = false;
static bio_hub_data_t bio_hub_data;
static sl_simple_timer_t bio_hub_timer;
static sl_simple_timer_t pulse_timer;
static sl_sleeptimer_timer_handle_t led_timer;
static uint16_t bpm = 60;
static uint8_t led_red = 255;
static uint8_t led_green = 255;
static uint8_t led_blue = 255;
static uint8_t led_delta = 0;
static char bio_hub_statuses[4][10] =
{ "NoObject ", "Object   ", "NotFinger", "Finger   " };
static char bio_hub_ext_statuses[8][13] =
{ "FingerMotion", // -6
		"NotFinger   ", // -5
		"TooHard     ", // -4
		"NoObject    ", // -3
		"SensorMotion", // -2
		"Object      ", // -1
		"Finger      ", //  0
		"NotReady"      //  1
		};

#ifndef SL_SENSOR_LUX_H
static uint8_t pulse_width_index = 3;
static uint8_t sample_rate_index = 3;
#endif

// App prototypes --------------------------------------------------------------
static sl_status_t app_bio_hub_init(void);
static sl_status_t app_bio_hub_config_pw_sr(void);
static void app_bio_hub_timer(sl_simple_timer_t *timer, void *data);
static void app_pulse_timer(sl_simple_timer_t *timer, void *data);
static void app_led_timer(sl_sleeptimer_timer_handle_t *timer, void *data);
static sl_status_t app_update_heart_rate_measurement_characteristic(void);
static sl_status_t app_send_heart_rate_measurement_notification(void);

#ifdef SL_SENSOR_LUX_H
static sl_status_t app_lux_read_bpm(bio_hub_data_t *data);
#endif

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
	app_log("\r\nbt_soc_heart_monitor_brd2601b_alpha\r\n");
	app_log("  app_init()\r\n");
	// Start timers
	sl_simple_timer_start(&bio_hub_timer, BIO_HUB_TIMER_MS, app_bio_hub_timer,
	NULL, true);
	sl_simple_timer_start(&pulse_timer, PULSE_TIMER_START_MS, app_pulse_timer,
	NULL, false);
	sl_sleeptimer_start_periodic_timer_ms(&led_timer, LED_TIMER_MS,
			app_led_timer, (void*) NULL, 0, 0);
}

static sl_status_t app_bio_hub_init(void)
{
	sl_status_t status = SL_STATUS_FAIL;

	if (log_streams & LOG_SENSOR)
		app_log("    app_bio_hub_init()\r\n");

#ifdef SL_SENSOR_LUX_H

	status = sl_sensor_lux_init();
	if (log_streams & LOG_SENSOR)
		app_log("      sl_sensor_lux_init()=0x%04x\r\n", (uint16_t ) status);

#else

	// Initialise hardware
	status = bio_hub_init(sl_i2cspm_sensor, 0);
	if (log_streams & LOG_SENSOR)
		app_log("      bio_hub_init()=0x%04x\r\n", (uint16_t ) status);
	if (SL_STATUS_OK == status)
	{
		// Configure BPM mode
		status = bio_hub_config_bpm(algo_mode);
		if (log_streams & LOG_SENSOR)
			app_log("      bio_hub_config_bpm(%d)=0x%04x\r\n", algo_mode,
					(uint16_t ) status);
	}

#endif

	if (log_streams & LOG_SENSOR)
		app_log("    app_bio_hub_init()=0x%04x\r\n", (uint16_t ) status);

	return status;
}

static sl_status_t app_bio_hub_config_pw_sr(void)
{
	sl_status_t status = SL_STATUS_FAIL;

#ifndef SL_SENSOR_LUX_H

	static max30101_pulse_width_t pulse_widths[4] =
	{ MAX30101_PULSE_WIDTH_69, MAX30101_PULSE_WIDTH_118, MAX30101_PULSE_WIDTH_215,
			MAX30101_PULSE_WIDTH_411 };
	static max30101_sample_rate_t sample_rates[4] =
	{ MAX30101_SAMPLE_RATE_50, MAX30101_SAMPLE_RATE_100, MAX30101_SAMPLE_RATE_200,
			MAX30101_SAMPLE_RATE_400 };

	uint16_t pulse_width;
	uint16_t sample_rate;

	if (log_streams & LOG_SENSOR)
		app_log("    app_bio_hub_config_pw_sr()\r\n");

	// Configure pulse width
	status = bio_hub_set_pulse_width(pulse_widths[pulse_width_index]);
	if (log_streams & LOG_SENSOR)
		app_log("      bio_hub_set_pulse_width(%d)=0x%04x\r\n",
				pulse_widths[pulse_width_index], (uint16_t ) status);
	if (SL_STATUS_OK == status)
	{
		// Check pulse width
		status = bio_hub_read_pulse_width(&pulse_width);
		if (log_streams & LOG_SENSOR)
			app_log(
					"      bio_hub_read_pulse_width()=0x%04x pulse_width=%u\r\n",
					(uint16_t ) status, pulse_width);
		if (SL_STATUS_OK == status)
		{
			// Set sample rate per second. Remember that not every sample rate is
			// available with every pulse width. Check hookup guide for more information.
			status = bio_hub_set_sample_rate(sample_rates[sample_rate_index]);
			if (log_streams & LOG_SENSOR)
				app_log("      bio_hub_set_sample_rate(%d)=0x%04x\r\n",
						sample_rates[sample_rate_index], (uint16_t ) status);
			if (SL_STATUS_OK == status)
			{
				// Check sample rate.
				status = bio_hub_read_sample_rate(&sample_rate);
				if (log_streams & LOG_SENSOR)
					app_log(
							"      bio_hub_read_sample_rate()=0x%04x sample_rate=%u\r\n",
							(uint16_t ) status, sample_rate);
				// Clear flag
				config_pw_sr = false;
			}
		}
	}

	if (log_streams & LOG_SENSOR)
		app_log("    app_bio_hub_config_pw_sr()=0x%04x\r\n",
				(uint16_t ) status);

#endif

	return status;
}

SL_WEAK void app_process_action(void)
{
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application code here!                              //
	// This is called infinitely.                                              //
	// Do not call blocking functions from here!                               //
	/////////////////////////////////////////////////////////////////////////////
}

static void app_bio_hub_timer(sl_simple_timer_t *timer, void *data)
{
	(void) timer;
	(void) data;

	static uint32_t bio_hub_timer_count = 0;
	bio_hub_data_t read_data;

	if (log_streams & LOG_SENSOR)
		app_log("  app_bio_hub_timer() count=%lu\r\n", bio_hub_timer_count++);

	// Sensor is not initialised ?
	if (SL_STATUS_OK != bio_hub_status)
	{
		// Retry initialisation
		bio_hub_status = app_bio_hub_init();
		if (SL_STATUS_OK == bio_hub_status)
		{
			// Configure pulse width and sample rate
			app_bio_hub_config_pw_sr();
		}
	}
	// Sensor is initialised ?
	else
	{

#ifdef SL_SENSOR_LUX_H

		// Get data
		bio_hub_status = app_lux_read_bpm(&read_data);
		if (log_streams & LOG_SENSOR)
			app_log("    lux()=0x%04x", (uint16_t ) bio_hub_status);

#else

		// Get data
		bio_hub_status = bio_hub_read_bpm(&read_data);
		if (log_streams & LOG_SENSOR)
			app_log("    bpm()=0x%04x", (uint16_t ) bio_hub_status);

#endif

		if (SL_STATUS_OK == bio_hub_status)
		{
			memcpy(&bio_hub_data, &read_data, sizeof(bio_hub_data));
			if (bio_hub_data.status == 3 && bio_hub_data.confidence > 0
					&& bio_hub_data.heart_rate > 0)
			{
				bpm = bio_hub_data.heart_rate;
			}
			if (log_streams & LOG_SENSOR)
				app_log(" sta=%s", bio_hub_statuses[bio_hub_data.status]);
			if (BIO_HUB_ALGO_MODE_TWO == algo_mode)
			{
				if (log_streams & LOG_SENSOR)
					app_log(" ext=%s",
							bio_hub_ext_statuses[bio_hub_data.ext_status + 6]);
			}
			if (log_streams & LOG_SENSOR)
				app_log(" con=%03d%%", bio_hub_data.confidence);
			if (log_streams & LOG_SENSOR)
				app_log(" oxy=%03d%%", bio_hub_data.oxygen);
			if (log_streams & LOG_SENSOR)
				app_log(" bpm=%03d", bio_hub_data.heart_rate);
		}
		if (log_streams & LOG_SENSOR)
			app_log("\r\n");
		// Need to configure pulse width or sample rate ?
		if (true == config_pw_sr)
		{
			app_bio_hub_config_pw_sr();
		}
	}
}

static void app_pulse_timer(sl_simple_timer_t *timer, void *data)
{
	(void) timer;
	(void) data;

	uint32_t pulse_ms;

	if (log_streams & LOG_BLUETOOTH)
		app_log("  app_pulse_timer()\r\n");

	// Stop LED timer
	sl_sleeptimer_stop_timer(&led_timer);
	// Sensor is not initialised ?
	if (SL_STATUS_OK != bio_hub_status)
	{
		// White pulse
		led_red = led_green = led_blue = 255;
	}
	// Sensor initialised ?
	else
	{
		// Finger detected ?
		if (3 == bio_hub_data.status)
		{
			// Have a heart rate ?
			if (bio_hub_data.confidence > 0 && bio_hub_data.heart_rate > 0)
			{
				// Green pulse
				led_green = 0xFF;
				led_red = led_blue = 0;
			}
			// No heart rate ?
			else
			{
				// Yellow pulse
				led_red = led_green = 0xFF;
				led_blue = 0;
			}
		}
		// Not a finger ?
		else
		{
			// Red pulse
			led_red = 0xFF;
			led_green = led_blue = 0;
		}
	}
	// Calculate time for a heart beat in ms (with limits)
	if (bpm > 240)
	{
		pulse_ms = 60000 / 240;
	}
	else if (bpm < 30)
	{
		pulse_ms = 60000 / 30;
	}
	else
	{
		pulse_ms = 60000 / bpm;
	}
	// Calculate delta LEDs need to fade at to get to zero just before end of pulse
	led_delta = 256 / ((pulse_ms) / LED_TIMER_MS);
	led_delta += 1;
	// Set LEDs
	sl_simple_rgb_pwm_led_set_color(
			sl_simple_rgb_pwm_led_rgb_led0.led_common.context, led_red,
			led_green, led_blue);
	// Start LED timer
	sl_sleeptimer_start_periodic_timer_ms(&led_timer, LED_TIMER_MS,
			app_led_timer, (void*) NULL, 0, 0);
	// Start pulse timer
	sl_simple_timer_start(&pulse_timer, pulse_ms, app_pulse_timer,
	NULL, false);
	// Update GATT data
	app_update_heart_rate_measurement_characteristic();
	// Send notifications
	app_send_heart_rate_measurement_notification();
}

static void app_led_timer(sl_sleeptimer_timer_handle_t *timer, void *data)
{
	(void) timer;
	(void) data;
	// Fade LEDs
	if (led_red >= led_delta)
		led_red -= led_delta;
	else if (led_red > 0)
		led_red = 0;
	if (led_green >= led_delta)
		led_green -= led_delta;
	else if (led_green > 0)
		led_green = 0;
	if (led_blue >= led_delta)
		led_blue -= led_delta;
	else if (led_blue > 0)
		led_blue = 0;
	// Update LEDs
	sl_simple_rgb_pwm_led_set_color(
			sl_simple_rgb_pwm_led_rgb_led0.led_common.context, led_red,
			led_green, led_blue);
}

void sl_button_on_change(const sl_button_t *handle)
{
	(void) handle;

#ifndef SL_SENSOR_LUX_H
	uint8_t btn = 0xFF;
	uint8_t btn_state = 0xFF;

	// Get state
	btn_state = sl_button_get_state(handle);
	// Button 0 ?
	if (handle == &sl_button_btn0)
	{
		btn = 0;
	}
	// Button 1 ?
	else if (handle == &sl_button_btn1)
	{
		btn = 1;
	}
	// Button down ?
	if (1 == btn_state)
	{
		// Valid button ?
		if (btn != 0xFF)
		{
			// Button 0 - change pulse width
			if (0 == btn)
			{
				pulse_width_index++;
				if (pulse_width_index >= 4)
					pulse_width_index = 0;
				config_pw_sr = true;

			}
			// Button 1 - attempt to turn off
			else if (1 == btn)
			{
				sample_rate_index++;
				if (sample_rate_index >= 4)
					sample_rate_index = 0;
				config_pw_sr = true;
			}
		}
	}
#endif
}

void sl_bt_on_event(sl_bt_msg_t *evt)
{
	sl_status_t sc;

	switch (SL_BT_MSG_ID(evt->header))
	{

	case sl_bt_evt_system_boot_id:
		// -------------------------------
		// This event indicates the device has started and the radio is ready.
		// Do not call any stack command before receiving this boot event!
		if (log_streams & LOG_BLUETOOTH)
			app_log("  sl_bt_on_event(system_boot)\r\n");
		// Create an advertising set.
		sc = sl_bt_advertiser_create_set(&advertising_set_handle);
		app_assert_status(sc);

		// Generate data for advertising
		sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
				sl_bt_advertiser_general_discoverable);
		app_assert_status(sc);

		// Set advertising interval to 100ms.
		sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, // min. adv. interval (milliseconds * 1.6)
				160, // max. adv. interval (milliseconds * 1.6)
				0,   // adv. duration
				0);  // max. num. adv. events
		app_assert_status(sc);
		// Start advertising and enable connections.
		sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
				sl_bt_advertiser_connectable_scannable);
		app_assert_status(sc);
		if (log_streams & LOG_BLUETOOTH)
			app_log("    sl_bt_legacy_advertiser_start()=0x%04x\r\n",
					(uint16_t ) sc);
		break;

	case sl_bt_evt_connection_opened_id:
		// -------------------------------
		// This event indicates that a new connection was opened.
		if (log_streams & LOG_BLUETOOTH)
			app_log("  sl_bt_on_event(connection_opened)\r\n");
		break;

	case sl_bt_evt_connection_closed_id:
		// -------------------------------
		// This event indicates that a connection was closed.
		if (log_streams & LOG_BLUETOOTH)
			app_log("  sl_bt_on_event(connection_closed)\r\n");
		// Generate data for advertising
		sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
				sl_bt_advertiser_general_discoverable);
		app_assert_status(sc);

		// Restart advertising after client has disconnected.
		sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
				sl_bt_advertiser_connectable_scannable);
		app_assert_status(sc);
		if (log_streams & LOG_BLUETOOTH)
			app_log("    sl_bt_legacy_advertiser_start()=0x%04x\r\n",
					(uint16_t ) sc);
		break;

	case sl_bt_evt_gatt_server_characteristic_status_id:
		// -------------------------------
		// This event occurs when the remote device enabled or disabled the
		// notification.
#ifdef gattdb_heart_rate_measurement
		if (gattdb_heart_rate_measurement
				== evt->data.evt_gatt_server_characteristic_status.characteristic)
		{
			if (log_streams & LOG_BLUETOOTH)
				app_log(
						"  sl_bt_on_event(gatt_server_characteristic_status, heart_rate_measurement)");
			// A local Client Characteristic Configuration descriptor was changed in
			// the gattdb_heart_rate_measurement characteristic.
			if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
					& sl_bt_gatt_notification)
			{
				if (log_streams & LOG_BLUETOOTH)
					app_log(" notification=on\r\n");
			}
			else
			{
				if (log_streams & LOG_BLUETOOTH)
					app_log(" notification=off\r\n");
			}
		}
#endif
		break;

		// -------------------------------
		// Default event handler.
	default:
		break;
	}
}

static sl_status_t app_update_heart_rate_measurement_characteristic(void)
{
	sl_status_t sc = SL_STATUS_FAIL;
	uint8_t data_send[2];

#ifdef gattdb_heart_rate_measurement
	if (log_streams & LOG_BLUETOOTH)
		app_log("    app_update_heart_rate_measurement_characteristic()\r\n");

	// Fill in flags from buttons
	data_send[0] = 0;
	if (sl_button_get_state(&sl_button_btn0))
		data_send[0] |= 0x02;
	if (sl_button_get_state(&sl_button_btn1))
		data_send[0] |= 0x04;
	data_send[1] = 66;

	// Write attribute in the local GATT database.
	sc = sl_bt_gatt_server_write_attribute_value(gattdb_heart_rate_measurement,
			0, sizeof(data_send), data_send);
	if (log_streams & LOG_BLUETOOTH)
		app_log(
				"      sl_bt_gatt_server_write_attribute_value(0x%02x %d)=0x%04x\r\n",
				data_send[0], data_send[1], (uint16_t ) sc);
#endif

	return sc;
}

static sl_status_t app_send_heart_rate_measurement_notification(void)
{
	sl_status_t sc = SL_STATUS_FAIL;

#ifdef gattdb_heart_rate_measurement
	uint8_t data_send[2];
	size_t data_len;

	if (log_streams & LOG_BLUETOOTH)
		app_log("    app_send_heart_rate_measurement_notification()\r\n");
	// Read heart rate measurement characteristic stored in local GATT database.
	sc = sl_bt_gatt_server_read_attribute_value(gattdb_heart_rate_measurement,
			0, sizeof(data_send), &data_len, data_send);
	if (sc != SL_STATUS_OK)
	{
		return sc;
	}

	// Send characteristic notification.
	sc = sl_bt_gatt_server_notify_all(gattdb_heart_rate_measurement,
			sizeof(data_send), data_send);
	if (log_streams & LOG_BLUETOOTH)
		app_log("      sl_bt_gatt_server_notify_all(0x%02x %d)=0x%04x\r\n",
				data_send[0], data_send[1], (uint16_t ) sc);
#endif

	return sc;
}

#ifdef SL_SENSOR_LUX_H
static sl_status_t app_lux_read_bpm(bio_hub_data_t *data)
{
	sl_status_t status = SL_STATUS_FAIL;

	float lux = 0.0;
	uint32_t lux_bpm = lux;

	status = sl_sensor_lux_get(&lux);

	lux_bpm = lux;
	lux_bpm /= 4;

	// Finger button ?
	if (sl_button_get_state(&sl_button_btn0))
	{
		// Finger present
		data->status = 3;
		data->ext_status = 0;
	}
	else
	{
		// No object present
		data->status = 0;
		data->ext_status = -3;
	}

	// Valid reading button ?
	if (sl_button_get_state(&sl_button_btn1) && SL_STATUS_OK == status)
	{
		// Dummy data
		data->confidence = 99;
		data->oxygen = 99;
		if (lux_bpm < 30)
			data->heart_rate = 30;
		else if (lux_bpm > 240)
			data->heart_rate = 240;
		else
			data->heart_rate = lux_bpm;
	}
	else
	{
		// Zero data
		data->confidence = 0;
		data->oxygen = 0;
		data->heart_rate = 0;
	}

	return status;
}
#endif
