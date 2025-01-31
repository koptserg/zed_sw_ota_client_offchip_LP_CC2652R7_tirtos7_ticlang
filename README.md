# How to join:
### If device in FN(factory new) state:
1. Press and hold button (BTN-1) for 3 seconds
2. Wait, in case of successfull join

### If device in a network:
1. Hold button (BTN-1) for 3 seconds, this will reset device to FN(factory new) status
2. Go to step 1 for FN device

### If device in a network, sending sensor readings report, time adjustment:
1. Short press of the button (BTN-1)

# Hardware
1. LP-CC2652R7(CC26X2R1_LAUNCHXL)
2. BME280, BH1750 - I2C (SDA-DIO5, SCL-DIO4)
3. EPD1IN54 - SPI (SCLK-DIO30, MOSI-DIO28, CS-DIO11, BUSY-DIO1, RST-DIO0, DC-DIO12) 

![](/images/photo_2024-10-17_15-03-37.jpg)

# Firmware
### After  loading the project, change the path to the zed_sw_ota_client_offchip_LP_CC2652R7_tirtos7_ticlang-master folder in the PROJ_LOC variable.
![](/images/Screenshot_2203.jpg)
### UniFlash firmware.
![](/images/Screenshot_2154.jpg)
### Debugging in Putty with xCUI_DISABLE.
![](/images/Screenshot_2158.jpg)
### Dashboard and Exposes in Zigbee2MQTT.
![](/images/Screenshot_2155.jpg)
![](/images/Screenshot_2156.jpg)
### OTA process
![](/images/Screenshot_2159.jpg)
