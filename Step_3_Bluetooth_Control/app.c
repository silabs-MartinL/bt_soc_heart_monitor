/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *
 * @description Step 3: Bluetooth Control
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
// Example Includes ------------------------------------------------------------
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"

// Application Includes --------------------------------------------------------
#include "app_log.h"
#include "sl_i2cspm_instances.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_rgb_pwm_led_instances.h"
#include "sl_simple_rgb_pwm_led.h"
#include "sl_simple_timer.h"
#include "sl_sleeptimer.h"
#include "sparkfun_max30101_max32664.h"
#include "gatt_db.h"

// Application Defines ---------------------------------------------------------
#define LOG_SENSOR         0x01
#define LOG_BLUETOOTH      0x02
#define LED_TIMER_MS         10
#define SENSOR_TIMER_MS     500
#define SENSOR_PULSE_WIDTHS   4
#define SENSOR_SAMPLE_RATES   4

// Example Variables -----------------------------------------------------------
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Application Variables -------------------------------------------------------
static uint8_t log_streams = (LOG_SENSOR | LOG_BLUETOOTH);
static uint32_t sensor_pulse_ms = 1000;
static sl_status_t sensor_status = SL_STATUS_FAIL;
static bio_hub_algo_mode_t sensor_mode = BIO_HUB_ALGO_MODE_TWO;
static bio_hub_data_t sensor_data;
static bool sensor_configure = false;
static uint8_t sensor_pulse_width_index = 0;
static uint8_t sensor_sample_rate_index = 0;
static uint8_t sensor_ext_status_index = 8;
static sl_simple_timer_t sensor_timer;
static sl_simple_timer_t pulse_timer;
static sl_sleeptimer_timer_handle_t led_timer;
static uint8_t led_red;
static uint8_t led_green;
static uint8_t led_blue;
static uint8_t led_delta;
static bool bt_system_boot = false;

// Application Constants -------------------------------------------------------
static const max30101_pulse_width_t sensor_pulse_widths[SENSOR_PULSE_WIDTHS] =
{ MAX30101_PULSE_WIDTH_69, MAX30101_PULSE_WIDTH_118, MAX30101_PULSE_WIDTH_215, MAX30101_PULSE_WIDTH_411 };
static const max30101_sample_rate_t sensor_sample_rates[SENSOR_SAMPLE_RATES] =
{ MAX30101_SAMPLE_RATE_50, MAX30101_SAMPLE_RATE_100, MAX30101_SAMPLE_RATE_200, MAX30101_SAMPLE_RATE_400 };
static const char sensor_statuses[4][10] =
{ "NoObject", "Object", "NotFinger", "Finger" };
static const char sensor_ext_statuses[9][13] =
{ "Finger", "NotReady", "FingerMotion", "NotFinger", "TooHard", "NoObject", "SensorMotion", "Object", "Disabled" };

// Application Functions -------------------------------------------------------
static sl_status_t app_sensor_init(void);
static sl_status_t app_sensor_configure(void);
static void app_sensor_timer(sl_simple_timer_t *timer, void *data);
static void app_pulse_timer(sl_simple_timer_t *timer, void *data);
static void app_led_timer(sl_sleeptimer_timer_handle_t *timer, void *data);
static sl_status_t app_update_heart_rate_measurement_characteristic(void);
static sl_status_t app_send_heart_rate_measurement_notification(void);
static sl_status_t app_update_heart_rate_configuration_characteristic(void);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  app_log("\r\nbt_soc_heart_monitor_brd2601b_alpha\r\n");
  app_log("app_init()\r\n");
  // Initialise sensor by directly calling timer callback function
  app_sensor_timer(&sensor_timer, NULL);
  // Start periodic sensor timer
  sl_simple_timer_start(&sensor_timer, SENSOR_TIMER_MS, app_sensor_timer, NULL, true);
  // Initialise LEDs and timers by directly calling callback function
  app_pulse_timer(&pulse_timer, NULL);
}

static sl_status_t app_sensor_init(void)
{
  sl_status_t status = SL_STATUS_FAIL;

  if (log_streams & LOG_SENSOR) app_log("  app_sensor_init()\r\n");

  // Initialise hardware
  status = bio_hub_init(sl_i2cspm_sensor, 0);
  if (log_streams & LOG_SENSOR) app_log("    bio_hub_init()=0x%04x\r\n", (uint16_t )status);
  if (SL_STATUS_OK == status)
  {
    // Configure BPM mode
    status = bio_hub_config_bpm(sensor_mode);
    if (log_streams & LOG_SENSOR) app_log("    bio_hub_config_bpm(%d)=0x%04x\r\n", sensor_mode, (uint16_t )status);
  }

  if (log_streams & LOG_SENSOR) app_log("  app_sensor_init()=0x%04x\r\n", (uint16_t )status);

  return status;
}

static sl_status_t app_sensor_configure(void)
{
  sl_status_t status = SL_STATUS_FAIL;

  if (log_streams & LOG_SENSOR) app_log("  app_sensor_configure()\r\n");

  // Configure pulse width
  status = bio_hub_set_pulse_width(sensor_pulse_widths[sensor_pulse_width_index]);
  if (log_streams & LOG_SENSOR) app_log("    bio_hub_set_pulse_width(%d)=0x%04x\r\n", sensor_pulse_widths[sensor_pulse_width_index], (uint16_t )status);
  if (SL_STATUS_OK == status)
  {
    uint16_t pulse_width;

    // Check pulse width
    status = bio_hub_read_pulse_width(&pulse_width);
    if (log_streams & LOG_SENSOR) app_log("    bio_hub_read_pulse_width()=0x%04x pulse_width=%u\r\n", (uint16_t )status, pulse_width);
    if (SL_STATUS_OK == status)
    {
      // Set sample rate per second. Remember that not every sample rate is
      // available with every pulse width. Check hookup guide for more information.
      status = bio_hub_set_sample_rate(sensor_sample_rates[sensor_sample_rate_index]);
      if (log_streams & LOG_SENSOR) app_log("    bio_hub_set_sample_rate(%d)=0x%04x\r\n", sensor_sample_rates[sensor_sample_rate_index], (uint16_t )status);
      if (SL_STATUS_OK == status)
      {
        uint16_t sample_rate;

        // Check sample rate.
        status = bio_hub_read_sample_rate(&sample_rate);
        if (log_streams & LOG_SENSOR) app_log("    bio_hub_read_sample_rate()=0x%04x sample_rate=%u\r\n", (uint16_t )status, sample_rate);
        // Clear flag
        sensor_configure = false;
        // Update GATT database
        app_update_heart_rate_configuration_characteristic();
      }
    }
  }

  if (log_streams & LOG_SENSOR) app_log("  app_sensor_configure()=0x%04x\r\n", (uint16_t )status);

  return status;
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header))
  {

  case sl_bt_evt_system_boot_id:
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!

    // Flag bluetooth system is booted
    bt_system_boot = true;
    if (log_streams & LOG_BLUETOOTH) app_log("sl_bt_on_event(system_boot)\r\n");
    // Create an advertising set.
    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    app_assert_status(sc);

    // Generate data for advertising
    sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
    app_assert_status(sc);

    // Set advertising interval to 100ms.
    sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,                      // adv. duration
        0);             // max. num. adv. events
    app_assert_status(sc);
    // Start advertising and enable connections.
    sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
    app_assert_status(sc);
    if (log_streams & LOG_BLUETOOTH) app_log("  sl_bt_legacy_advertiser_start()=0x%04x\r\n", (uint16_t )sc);
    break;

  case sl_bt_evt_connection_opened_id:
    // -------------------------------
    // This event indicates that a new connection was opened.

    if (log_streams & LOG_BLUETOOTH) app_log("sl_bt_on_event(connection_opened)\r\n");
    break;

  case sl_bt_evt_connection_closed_id:
    // -------------------------------
    // This event indicates that a connection was closed.

    if (log_streams & LOG_BLUETOOTH) app_log("sl_bt_on_event(connection_closed)\r\n");
    // Generate data for advertising
    sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);
    app_assert_status(sc);

    // Restart advertising after client has disconnected.
    sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
    app_assert_status(sc);
    if (log_streams & LOG_BLUETOOTH) app_log("  sl_bt_legacy_advertiser_start()=0x%04x\r\n", (uint16_t )sc);
    break;

  case sl_bt_evt_gatt_server_characteristic_status_id:
    // -------------------------------
    // This event occurs when the remote device enabled or disabled the
    // notification.

    if (gattdb_heart_rate_measurement == evt->data.evt_gatt_server_characteristic_status.characteristic)
    {
      if (log_streams & LOG_BLUETOOTH) app_log("sl_bt_on_event(gatt_server_characteristic_status, heart_rate_measurement)\r\n");
      // A local Client Characteristic Configuration descriptor was changed in
      // the gattdb_heart_rate_measurement characteristic.
      if (evt->data.evt_gatt_server_characteristic_status.client_config_flags & sl_bt_gatt_notification)
      {
        if (log_streams & LOG_BLUETOOTH) app_log("  notification=on\r\n");
      }
      else
      {
        if (log_streams & LOG_BLUETOOTH) app_log("  notification=off\r\n");
      }
    }
    break;

  case sl_bt_evt_gatt_server_attribute_value_id:
    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.

    // The value of the gattdb_led_control characteristic was changed.
    if (gattdb_heart_rate_configuration == evt->data.evt_gatt_server_attribute_value.attribute)
    {

      if (log_streams & LOG_BLUETOOTH) app_log("sl_bt_on_event(gatt_server_attribute_value, heart_rate_configuration)\r\n");

      uint8_t data_recv;
      size_t data_recv_len;

      // Read characteristic value.
      sc = sl_bt_gatt_server_read_attribute_value( gattdb_heart_rate_configuration, 0, sizeof(data_recv), &data_recv_len, &data_recv);
      (void) data_recv_len;

      if (SL_STATUS_OK == sc)
      {
        // Extract data indices
        sensor_pulse_width_index = (data_recv >> 4) & 0x3;
        sensor_sample_rate_index = data_recv & 0x3;
        // Flag change to trigger reconfiguration
        sensor_configure = true;
        if (log_streams & LOG_BLUETOOTH) app_log("  pw=%d sr=%d\r\n", sensor_pulse_width_index, sensor_sample_rate_index);
      }
    }
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

  if (log_streams & LOG_BLUETOOTH) app_log("  app_update_heart_rate_measurement_characteristic()\r\n");

  if (bt_system_boot)
  {
    // Fill in flags field format=uint8, sensor contact supported
    data_send[0] = 0x02; // SHOULD BE 0x04 FROM SPEC
    // Sensor contact detected
    if (3 == sensor_data.status) data_send[0] |= 0x04; // SHOULD BE 0x02 FROM SPEC
    // Fill in heart rate measurement field
    if (sensor_data.heart_rate > 255)
      data_send[1] = 255;
    else
      data_send[1] = (uint8_t) sensor_data.heart_rate;

    // Write attribute in the local GATT database
    sc = sl_bt_gatt_server_write_attribute_value(
    gattdb_heart_rate_measurement, 0, sizeof(data_send), data_send);
    if (log_streams & LOG_BLUETOOTH) app_log("    sl_bt_gatt_server_write_attribute_value(0x%02x %d)=0x%04x\r\n", data_send[0], data_send[1], (uint16_t )sc);
  }

  return sc;
}

static sl_status_t app_send_heart_rate_measurement_notification(void)
{
  sl_status_t sc = SL_STATUS_FAIL;

  uint8_t data_send[2];
  size_t data_len;

  if (log_streams & LOG_BLUETOOTH) app_log("  app_send_heart_rate_measurement_notification()\r\n");

  if (bt_system_boot)
  {
    // Read heart rate measurement characteristic stored in local GATT database
    sc = sl_bt_gatt_server_read_attribute_value( gattdb_heart_rate_measurement, 0, sizeof(data_send), &data_len, data_send);

    // Send characteristic notification.
    sc = sl_bt_gatt_server_notify_all(gattdb_heart_rate_measurement, sizeof(data_send), data_send);
    if (log_streams & LOG_BLUETOOTH) app_log("    sl_bt_gatt_server_notify_all(0x%02x %d)=0x%04x\r\n", data_send[0], data_send[1], (uint16_t )sc);
  }

  return sc;
}

static sl_status_t app_update_heart_rate_configuration_characteristic(void)
{
  sl_status_t sc = SL_STATUS_FAIL;
  uint8_t data_send;

  if (log_streams & LOG_BLUETOOTH) app_log("  app_update_heart_rate_configuration_characteristic()\r\n");

  if (bt_system_boot)
  {
    // Fill in configuration field
    data_send = (sensor_pulse_width_index & 0x3) << 4;
    data_send += (sensor_sample_rate_index & 0x3);
    // Write attribute in the local GATT database
    sc = sl_bt_gatt_server_write_attribute_value(
    gattdb_heart_rate_configuration, 0, sizeof(data_send), &data_send);
    if (log_streams & LOG_BLUETOOTH) app_log("    sl_bt_gatt_server_write_attribute_value(0x%02x)=0x%04x\r\n", data_send, (uint16_t )sc);
  }

  return sc;
}

void sl_button_on_change(const sl_button_t *handle)
{
  (void) handle;

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
        sensor_pulse_width_index++;
        if (sensor_pulse_width_index >= SENSOR_PULSE_WIDTHS) sensor_pulse_width_index = 0;
        sensor_configure = true;
      }
      // Button 1 - attempt to turn off
      else if (1 == btn)
      {
        sensor_sample_rate_index++;
        if (sensor_sample_rate_index >= 4) sensor_sample_rate_index = 0;
        sensor_configure = true;
      }
    }
  }
}

static void app_sensor_timer(sl_simple_timer_t *timer, void *data)
{
  (void) timer;
  (void) data;

  static uint32_t sensor_timer_count = 0;

  if (log_streams & LOG_SENSOR) app_log("app_sensor_timer() count=%lu\r\n", sensor_timer_count++);

  // Sensor is not initialised ?
  if (SL_STATUS_OK != sensor_status)
  {
    // Attempt initialisation
    sensor_status = app_sensor_init();
    if (SL_STATUS_OK == sensor_status)
    {
      // Configure sensor
      sensor_status = app_sensor_configure();
    }
  }
  // Sensor is initialised ?
  else
  {
    bio_hub_data_t bio_hub_data;

    // Get data
    sensor_status = bio_hub_read_bpm(&bio_hub_data);
    if (log_streams & LOG_SENSOR) app_log("  bio_hub_read_bpm()=0x%04x\r\n", (uint16_t )sensor_status);

    // Got data ?
    if (SL_STATUS_OK == sensor_status)
    {
      // Copy data
      memcpy(&sensor_data, &bio_hub_data, sizeof(sensor_data));
      // Initialise extended status index
      sensor_ext_status_index = 8; // Disabled
      // Mode two (with extended status ?
      if (BIO_HUB_ALGO_MODE_TWO == sensor_mode)
      {
        // Get extended status index
        if (sensor_data.ext_status >= -6 && sensor_data.ext_status < 0)
        {
          sensor_ext_status_index = sensor_data.ext_status + 8;
        }
        else if (sensor_data.ext_status == 0 || sensor_data.ext_status == 1)
        {
          sensor_ext_status_index = sensor_data.ext_status;
        }
      }
      // Finger detected and valid reading ?
      if (sensor_data.status == 3 && sensor_data.confidence > 0 && sensor_data.heart_rate > 0)
      {
        // Calculate pulse in ms and limit
        if (sensor_data.heart_rate < 30)
          sensor_pulse_ms = 60000 / 30;
        else if (sensor_data.heart_rate > 240)
          sensor_pulse_ms = 60000 / 240;
        else
          sensor_pulse_ms = 60000 / sensor_data.heart_rate;
      }

      if (log_streams & LOG_SENSOR)
      {
        if (BIO_HUB_ALGO_MODE_TWO == sensor_mode) app_log("    ext=%s", sensor_ext_statuses[sensor_ext_status_index]);
        app_log(" sta=%s", sensor_statuses[sensor_data.status]);
        app_log(" con=%d%%", sensor_data.confidence);
        app_log(" oxy=%d%%", sensor_data.oxygen);
        app_log(" bpm=%d", sensor_data.heart_rate);
        app_log(" pul=%lums", sensor_pulse_ms);
        app_log("\r\n");
      }

      // Need to configure pulse width or sample rate ?
      if (true == sensor_configure)
      {
        sensor_status = app_sensor_configure();
      }
    }
  }
}

static void app_pulse_timer(sl_simple_timer_t *timer, void *data)
{
  (void) timer;
  (void) data;

  // Stop LED timer
  sl_sleeptimer_stop_timer(&led_timer);
  if (log_streams & LOG_BLUETOOTH) app_log("app_pulse_timer()\r\n");
  // Sensor is not initialised ?
  if (SL_STATUS_OK != sensor_status)
  {
    // White pulse
    led_red = led_green = led_blue = 255;
  }
  // Sensor initialised ?
  else
  {
    // Finger detected ?
    if (3 == sensor_data.status)
    {
      // Have a heart rate ?
      if (sensor_data.confidence > 0 && sensor_data.heart_rate > 0)
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
  // Calculate delta LEDs need to fade at to get to zero just before end of pulse
  led_delta = (256 / (sensor_pulse_ms / LED_TIMER_MS)) + 1;
  // Set LEDs
  sl_simple_rgb_pwm_led_set_color(sl_simple_rgb_pwm_led_rgb_led0.led_common.context, led_red, led_green, led_blue);
  // Start LED timer
  sl_sleeptimer_start_periodic_timer_ms(&led_timer, LED_TIMER_MS, app_led_timer, (void*) NULL, 0, 0);
  // Start pulse timer
  sl_simple_timer_start(&pulse_timer, sensor_pulse_ms, app_pulse_timer, NULL,
  false);
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
  else if (led_red > 0) led_red = 0;
  if (led_green >= led_delta)
    led_green -= led_delta;
  else if (led_green > 0) led_green = 0;
  if (led_blue >= led_delta)
    led_blue -= led_delta;
  else if (led_blue > 0) led_blue = 0;
  // Update LEDs
  sl_simple_rgb_pwm_led_set_color(sl_simple_rgb_pwm_led_rgb_led0.led_common.context, led_red, led_green, led_blue);
}
