# NRF52 Series for BLE IMAGE (JPG) File Transfer Example

How to transfer a large file through BLE is a frequency ask question and hot topic.   In this example, it would show to get the image file from PC through UART and then transfer to the Mobile.

## Block diagram

![Image of block diagram](https://github.com/jimmywong2003/nrf52_ble_transfer_jpg/blob/master/picture/Block_Diagram_on_JPEG_Transfer.png)

Based on the example (https://github.com/NordicPlayground/nrf52-ble-image-transfer-demo), I add the python script to load the image through UART and add the flow control in order to optimize the data transfer speed.

Also, I re-used the Android APK (https://github.com/NordicPlayground/Android-Image-Transfer-Demo) for retrieving the JPG file and show on the phone.

## Data flow on the UART with BLE transfer

In order to optimize the data throughput transfer, I add the flow control as below message.

![Image of flow control](https://github.com/jimmywong2003/nrf52_ble_transfer_jpg/blob/master/picture/flow_control_on_uart.png)

The idea is to retrieve a sector of data (such as 4KB) from UART first and then send through BLE.

### Demo hex

You can find the demo hex for trying at the nRF52832 DK or nRF52840 DK board.

``
nrfjprog --program combined_app_transfer_jpg_pca10040e.hex --reset --chiperase
nrfjprog --program combined_app_transfer_jpg_pca10056.hex --reset --chiperase
``

# Requirements
-----------------------------------------
- nRF5 SDK version 17.0.2
- Softdevice S140v7.2.0 / Softdevice S113v7.2.0
- nRF52840 DK / nRF52810
- Segger Embedded Studio IDE (SES) Project
- Python 3.7.x with pyserial module



## Note
-----------------------------------------
The project may need modifications to work with later versions or other boards.
To compile it, clone the repository in the /nRF5_SDK_17.0.2/examples/ directory.
The application is built to be used with the official nRF5 SDK that can be downloaded from developer.nordicsemi.com
