# Bluetooth SoC Heart Monitor

Bluetooth heart monitor for EFR32MG24 Dev Kit (BRD2601B) using SparkFun Pulse Oximeter and Heart Rate Monitor

## Operation

The RGB LED pulses in time with the last good heart rate using the following colours:

* White: No communications with sensor, attempting to re-establish
* Red: No finger detected on sensor
* Yellow: Finger detected on sensor but no valid heart rate
* Green: Finger detected and valid heart rate, the most recently read heart rate continues to be used with other colors

The buttons control the configuration of the sensor:

* btn0: Changes pulse width
* btn1: Changes samples per second

The sensor is read every 250ms

All readings are output to serial port: 115200 baud rate, 8 data bits, no parity, 1 stop bit, no flow control

## Development

1. SLCP > Software Components > Quality > Evaluation > Enable
2. SLCP > Software Components > Search: `third` > Third Party Hardware Drivers > Enable Extension
3. SLCP > Software Components > Search:  `oximeter` > MAX30101 & MAX32664 - Pulse Oximeter and Heart Rate Sensor (Sparkfun) > Install
4. SLCP > Software Components > Search: `i2cspm` > I2CSPM > Add New Instances > Instance: `sensor` 
   (Instance `inst0` should disappear, if the extension gets updated we should get a proper instance possibly named `qwiic` when the Oximeter is installed)
5. SLCP > Software Components > Search: `io stream` > IO Stream USART >  Install >  Instance: `vcom`
6. SLCP > Software Components > Search: `log` > Log > Install
7. SLCP > Software Components > Search: `simple timer` > Simple Timer Service > Install
8. SLCP > Software Components > Search: `simple rgb` > Install > Instance: `rgb_led0`
9. SLCP > Software Components > Search: `simple button` > Install > Instance: `btn0` > Add New Instance: `btn1`
10. Import `gatt_configuration.btconf` into GATT Configurator (or make the changes manually as described below) 
11. Copy `app.c` from repo (or release) into project, compile, build and run

## GATT Database

1. Device Name characteristic > Initial Value > `Heart Monitor` (adjust Value Length as required)
2. Add Heart Rate service
3. Heart Rate service > Advertise Service > Enable
4. Heart Rate Control Point characteristic > Delete (Energy Expended feature is not supported)
5. Body Sensor Location characteristic > Initial Value > `0x03` (Finger)
6. Heart Rate Measurement characteristic > Value Length > `2`
7. Heart Rate Measurement characteristic > Initial Value > `0x0000`

## Light Sensor

Uses buttons and light sensor as a dummy heart reading:

* BTN0, finger present
* BTN1, valid reading

To implement:

1. SLCP > Software Components > Search: `light sensor` > Ambient Light Sensor > Install
2. SLCP > Software Components > Search: `board control` > Configure > Enable Light Sensor > On
3. app.c > comment in `#include "sl_sensor_lux.h"`

## References

* https://www.bluetooth.com/specifications/specs/heart-rate-profile-1-0/
* https://www.bluetooth.com/specifications/specs/heart-rate-service-1-0/
* https://www.bluetooth.com/specifications/assigned-numbers/ GATT Specification Supplement for Characteristic values (Body Sensor Location)

# SoC - Empty

The Bluetooth SoC-Empty example is a project that you can use as a template for any standalone Bluetooth application.

> Note: this example expects a specific Gecko Bootloader to be present on your device. For details see the Troubleshooting section.

## Getting Started

To learn the Bluetooth technology basics, see [UG103.14: Bluetooth LE Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-14-fundamentals-ble.pdf).

To get started with Silicon Labs Bluetooth and Simplicity Studio, see [QSG169: Bluetooth SDK v3.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg169-bluetooth-sdk-v3x-quick-start-guide.pdf).

The term SoC stands for "System on Chip", meaning that this is a standalone application that runs on the EFR32/BGM and does not require any external MCU or other active components to operate.

As the name implies, the example is an (almost) empty template that has only the bare minimum to make a working Bluetooth application. This skeleton can be extended with the application logic.

The development of a Bluetooth applications consist of three main steps:

* Designing the GATT database
* Responding to the events raised by the Bluetooth stack
* Implementing additional application logic

These steps are covered in the following sections. To learn more about programming an SoC application, see [UG434: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf).

## Designing the GATT Database

The SOC-empty example implements a basic GATT database. GATT definitions (services/characteristics) can be extended using the GATT Configurator, which can be found under Advanced Configurators in the Software Components tab of the Project Configurator. To open the Project Configurator, open the .slcp file of the project.

![Opening GATT Configurator](image/readme_img1.png)

To learn how to use the GATT Configurator, see [UG438: GATT Configurator User’s Guide for Bluetooth SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug438-gatt-configurator-users-guide-sdk-v3x.pdf).

## Responding to Bluetooth Events

A Bluetooth application is event driven. The Bluetooth stack generates events e.g., when a remote device connects or disconnects or when it writes a characteristic in the local GATT database. The application has to handle these events in the `sl_bt_on_event()` function. The prototype of this function is implemented in *app.c*. To handle more events, the switch-case statement of this function is to be extended. For the list of Bluetooth events, see the online [Bluetooth API Reference](https://docs.silabs.com/bluetooth/latest/).

## Implementing Application Logic

Additional application logic has to be implemented in the `app_init()` and `app_process_action()` functions. Find the definitions of these functions in *app.c*. The `app_init()` function is called once when the device is booted, and `app_process_action()` is called repeatedly in a while(1) loop. For example, you can poll peripherals in this function. To save energy and to have this function called at specific intervals only, for example once every second, use the services of the [Sleeptimer](https://docs.silabs.com/gecko-platform/latest/service/api/group-sleeptimer). If you need a more sophisticated application, consider using RTOS (see [AN1260: Integrating v3.x Silicon Labs Bluetooth Applications with Real-Time Operating Systems](https://www.silabs.com/documents/public/application-notes/an1260-integrating-v3x-bluetooth-applications-with-rtos.pdf)).

## Features Already Added to the SOC-Empty Application

The SOC-Empty application is ***almost*** empty. It implements a basic application to demonstrate how to handle events, how to use the GATT database, and how to add software components.

* A simple application is implemented in the event handler function that starts advertising on boot (and on connection_closed event). This makes it possible for remote devices to find the device and connect to it.
* A simple GATT database is defined by adding Generic Access and Device Information services. This makes it possible for remote devices to read out some basic information such as the device name.
* The OTA DFU software component is added, which extends both the event handlers (see *sl_ota_dfu.c*) and the GATT database (see *ota_dfu.xml*). This makes it possible to make Over-The-Air Device-Firmware-Upgrade without any additional application code.

## Testing the SOC-Empty Application

As described above, an empty example does nothing except advertising and letting other devices connect and read its basic GATT database. To test this feature, do the following:

1. Build and flash the SoC-Empty example to your device.
2. Make sure a bootloader is installed. See the Troubleshooting section.
3. Download the **EFR Connect** smartphone app, available on [iOS](https://apps.apple.com/us/app/efr-connect/id1030932759) and [Android](https://play.google.com/store/apps/details?id=com.siliconlabs.bledemo).
4. Open the app and choose the Bluetooth Browser.
   ![EFR Connect start screen](image/readme_img2.png)
5. Now you should find your device advertising as "Empty Example". Tap **Connect**.
   ![Bluetooth Browser](image/readme_img3.png)
6. The connection is opened, and the GATT database is automatically discovered. Find the device name characteristic under Generic Access service and try to read out the device name.
   ![GATT database of the device](image/readme_img4.png)

## Troubleshooting

### Bootloader Issues

Note that Example Projects do not include a bootloader. However, Bluetooth-based Example Projects expect a bootloader to be present on the device in order to support device firmware upgrade (DFU). To get your application to work, you should either 
- flash the proper bootloader or
- remove the DFU functionality from the project.

**If you do not wish to add a bootloader**, then remove the DFU functionality by uninstalling the *Bootloader Application Interface* software component -- and all of its dependants. This will automatically put your application code to the start address of the flash, which means that a bootloader is no longer needed, but also that you will not be able to upgrade your firmware.

**If you want to add a bootloader**, then either 
- Create a bootloader project, build it and flash it to your device. Note that different projects expect different bootloaders:
  - for NCP and RCP projects create a *BGAPI UART DFU* type bootloader
  - for SoC projects on Series 1 devices create a *Bluetooth in-place OTA DFU* type bootloader or any *Internal Storage* type bootloader
  - for SoC projects on Series 2 devices create a *Bluetooth Apploader OTA DFU* type bootloader

- or run a precompiled Demo on your device from the Launcher view before flashing your application. Precompiled demos flash both bootloader and application images to the device. Flashing your own application image after the demo will overwrite the demo application but leave the bootloader in place. 
  - For NCP and RCP projects, flash the *Bluetooth - NCP* demo.
  - For SoC projects, flash the *Bluetooth - SoC Thermometer* demo.

**Important Notes:** 
- when you flash your application image to the device, use the *.hex* or *.s37* output file. Flashing *.bin* files may overwrite (erase) the bootloader.

- On Series 1 devices (EFR32xG1x), both first stage and second stage bootloaders have to be flashed. This can be done at once by flashing the *-combined.s37* file found in the bootloader project after building the project.

- On Series 2 devices SoC example projects require a *Bluetooth Apploader OTA DFU* type bootloader by default. This bootloader needs a lot of flash space and does not fit into the regular bootloader area, hence the application start address must be shifted. This shift is automatically done by the *Apploader Support for Applications* software component, which is installed by default. If you want to use any other bootloader type, you should remove this software component in order to shift the application start address back to the end of the regular bootloader area. Note, that in this case you cannot do OTA DFU with Apploader, but you can still implement application-level OTA DFU by installing the *Application OTA DFU* software component instead of *In-place OTA DFU*.

For more information on bootloaders, see [UG103.6: Bootloader Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-06-fundamentals-bootloading.pdf) and [UG489: Silicon Labs Gecko Bootloader User's Guide for GSDK 4.0 and Higher](https://cn.silabs.com/documents/public/user-guides/ug489-gecko-bootloader-user-guide-gsdk-4.pdf).


### Programming the Radio Board

Before programming the radio board mounted on the mainboard, make sure the power supply switch is in the AEM position (right side) as shown below.

![Radio board power supply switch](image/readme_img0.png)


## Resources

[Bluetooth Documentation](https://docs.silabs.com/bluetooth/latest/)

[UG103.14: Bluetooth LE Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-14-fundamentals-ble.pdf)

[QSG169: Bluetooth SDK v3.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg169-bluetooth-sdk-v3x-quick-start-guide.pdf)

[UG434: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf)

[Bluetooth Training](https://www.silabs.com/support/training/bluetooth)

## Report Bugs & Get Support

You are always encouraged and welcome to report any issues you found to us via [Silicon Labs Community](https://www.silabs.com/community).