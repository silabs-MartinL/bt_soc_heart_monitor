# Bluetooth - SoC Heart Monitor

Bluetooth heart monitor for [Silicon Labs EFR32MG24 Dev Kit (BRD2601B)](https://www.silabs.com/development-tools/wireless/efr32xg24-dev-kit?tab=overview) using [SparkFun Pulse Oximeter and Heart Rate Monitor](https://www.sparkfun.com/products/15219)

![Hardware](Image/hardware.png)

## Operation

The RGB LED pulses in time with the last good heart rate using the following colours:

* White: No communications with sensor, attempting to re-establish
* Red: No finger detected on sensor
* Yellow: Finger detected on sensor but no valid heart rate
* Green: Finger detected and valid heart rate, the most recently read heart rate continues to be used with other colors

The buttons control the configuration of the sensor:

* btn0: Changes pulse width
* btn1: Changes samples per second

The sensor is read every 500ms

All readings are output to serial port: 115200 baud rate, 8 data bits, no parity, 1 stop bit, no flow control

Readings can be viewed in EFR Connect mobile application ([Google Play](https://play.google.com/store/apps/details?id=com.siliconlabs.bledemo), [Apple App Store](https://itunes.apple.com/us/app/silicon-labs-blue-gecko-wstk/id1030932759?mt=8)) or any app that can connect to a standard [Bluetooth Heart Rate Profile](https://www.bluetooth.com/specifications/specs/heart-rate-profile-1-0/) device

![operation](Image/operation.png)

## Binary

A pre-built binary is available in the Binary folder

## Development

These files are part of a Silicon Labs workshop, a brief set of steps that are followed in the workshop are given below:

### Step 0: Project

In this step the required files are downloaded, the Third-party Hardware Drivers Extension is installed into Simplicity Studio and a template project created to use as a starting point:

1. Clone or download this repository to your local PC
2. Clone or download the [Third-party Hardware Drivers Extension](https://github.com/SiliconLabs/third_party_hw_drivers_extension) from Github
3. Follow the instructions in the Third-party Hardware Drivers Extension readme
4. Connect EFR32MG24 Dev Kit board to PC
5. From the Simplicity Studio **Launcher** perspective:
   1. Select the EFR32MG24 Dev Kit in Debug adapters
   2. Select **Example Projects and Demos** tab
   3. Find **Bluetooth - SoC Empty** example and click **Create** button

### Step 1: Sensor

In this step the sensor hardware is set up without any wireless communications:

1. In Simplicity Studio select the **Simplicity IDE** perspective
2. Open the `.slcp` project file
3. Select **Software Components** tab:
   1. Search `io stream`, select **Services > IO Stream > Driver > IO Stream: EUSART**, click **Install** button, accept default `vcom` instance name
   2. Search `log`, select **Application > Utility > Log**, click **Install** button
   3. Check **Evaluation** in the **Quality** dropdown filter
   4. Search `third`, select **Third Party Hardware Drivers**, click **Enable Extension**
   5. Search `oximeter`, select **Third Party Hardware Drivers > Sensors > MAX30101 & MAX32664 Pulse Oximeter and Heart Rate Sensor (Sparkfun)**, click **Install** button
   6. Search `i2cspm`, select **Platform > Driver > I2C > I2CSPM**, if there is an instance named `inst0`, click **Add New Instances** button, accept default `sensor` instance
   7. Search `simple timer`, select **Application > Service > Simple Timer Service**, click **Install** button
   8. Search `simple button`, select **Platform > Driver > Button > Simple Button**, click **Install** button, accept default `btn0` instance, click **Add New Instances** button, accept default `btn1` instance
   9. Search `rgb`, select **Platform > Driver > LED > Simple RGB PWM LED**, click **Install** button, accept default `rgb_led0` instance

4. Copy `Step_1_Sensor/app.c` (downloaded from Github) into project folder
5. Compile, build, flash and run

### Step 2: Bluetooth

In this step Bluetooth functionality is added to the application:

1. From the `.slcp` project file, **Configuration Tools** tab:
   1. Find **Bluetooth GATT Configurator** and click the **Open** button
2. From the **Bluetooth GATT Configurator**:
   1. Click the **Import** ![](Image/gatt_import.png) button
   2. Select `Step_2_Bluetooth/gatt_configuration.btconf` (downloaded from Github) into project folder
   3. Save the GATT configuration file
3. Copy `Step_2_Bluetooth/app.c` (downloaded from Github) into project folder
4. Compile, build, flash and run 
5. To connect from the **EFR Connect** mobile application, look for a device named **Heart Monitor**, expand the **Heart Rate** service, click the **Notify** below the **Heart Rate Measurement** characteristic to receive updates from the heart monitor over Bluetooth

### Step 3: Bluetooth Control

In this step a custom characteristic is added to allow the Heart Monitor configuration to be altered over Bluetooth:

1. From the `.slcp` project file, **Configuration Tools** tab:
   1. Find **Bluetooth GATT Configurator** and click the **Open** button
2. From the **Bluetooth GATT Configurator**:
   1. Click the **Import** ![](Image/gatt_import.png) button
   2. Select `Step_3_Bluetooth_Control/gatt_configuration.btconf` (downloaded from Github) into project folder
   3. Save the GATT configuration file
3. Copy `Step_3_Bluetooth_Control/app.c` (downloaded from Github) into project folder
4. Compile, build, flash and run
5. In **EFR Connect**, the configuration of the sensor can be changed by writing a hex value to the characteristic named **Unknown Characteristic**, the most significant nibble is the pulse width index and the least significant nibble is the sample rate index

## References

* https://www.bluetooth.com/specifications/specs/heart-rate-profile-1-0/
* https://www.bluetooth.com/specifications/specs/heart-rate-service-1-0/
* https://www.bluetooth.com/specifications/assigned-numbers/ **GATT Specification Supplement** provides details of all standard Bluetooth Characteristics

# Bluetooth - SoC Empty

*This **Bluetooth - SoC Heart Monitor** project is based upon the **Bluetooth - SoC Empty** example, below is the readme for the **Bluetooth - SoC Empty** example:*

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

![Opening GATT Configurator](Image/readme_img1.png)

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
   ![EFR Connect start screen](Image/readme_img2.png)  
5. Now you should find your device advertising as "Empty Example". Tap **Connect**  
   ![Bluetooth Browser](Image/readme_img3.png)  
6. The connection is opened, and the GATT database is automatically discovered. Find the device name characteristic under Generic Access service and try to read out the device name.  
   ![GATT database of the device](Image/readme_img4.png)  
   

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

![Radio board power supply switch](Image/readme_img0.png)


## Resources

[Bluetooth Documentation](https://docs.silabs.com/bluetooth/latest/)

[UG103.14: Bluetooth LE Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-14-fundamentals-ble.pdf)

[QSG169: Bluetooth SDK v3.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg169-bluetooth-sdk-v3x-quick-start-guide.pdf)

[UG434: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf)

[Bluetooth Training](https://www.silabs.com/support/training/bluetooth)

## Report Bugs & Get Support

You are always encouraged and welcome to report any issues you found to us via [Silicon Labs Community](https://www.silabs.com/community).