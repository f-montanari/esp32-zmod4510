# ESP32-ZMOD4510 Library
ESP-IDF Component to integrate the ZMOD4510 OAQ Sensor into an ESP32 

# Installing
Since Renesas firmware is proprietary, you have to download it from their site from the following link:
[Renesas ZMOD4510 homepage](https://www.renesas.com/us/en/products/sensor-products/environmental-sensors/metal-oxide-gas-sensors/zmod4510-gas-sensor-o3-and-no2#design_development)

You must request access, which usually takes 24-48hs. Once that's done, you can download the firmware from their site.
In order for this library to work, you need to copy the following files from their firmware into the corresponding folders:

```
(Renesas Firmware Folder -> esp32-zmod4510 component folder)
./ZMOD4510_Firmware/gas-algorithm-libraries/oaq_2nd_gen/Espressif ESP/<target>/<build>/lib_oaq_2nd_gen.a -> ./lib/lib_oaq_2nd_gen.a
./ZMOD4510_Firmware/gas-algorithm-libraries/oaq_2nd_gen/Espressif ESP/<target>/<build>/oaq_2nd_gen.h -> ./include/oaq_2nd_gen.h
./ZMOD4510_Firmware/zmod4xxx_evk_example/src/zmod4xxx.c -> ./src/zmod4xxx.c
./ZMOD4510_Firmware/zmod4xxx_evk_example/src/zmod4xxx.h -> ./include/zmod4xxx.h
./ZMOD4510_Firmware/zmod4xxx_evk_example/src/zmod4xxx_types.h -> ./include/zmod4xxx_types.h
./ZMOD4510_Firmware/zmod4xxx_evk_example/src/zmod4510_config_oaq2.h -> ./include/zmod4510_config_oaq2.h
```

# Configuration
This library uses KConfig environment variables to define I2C pins and other parameters.
In order to change them, you have to either enter the SDK Configuration in the project manager (Espressif IDE) or execute the following command in your project root:
```
idf.py menuconfig
```
There you'll find a menu called "ZMOD4510 Configuration" that allows you to define the IO pins for SDA, SCL, and clock speed for I2C communication. An IO pin is also needed for the EN pin of the sensor (it's always reset on initialization). 
![image](https://github.com/f-montanari/esp32-zmod4510/assets/15928972/83fc67c6-4f5c-4112-a088-2745a3b5c671)
![image](https://github.com/f-montanari/esp32-zmod4510/assets/15928972/09284f59-4472-4570-a4de-b82278cae2ca)

# Usage
Here's an example for polling the ZMOD4510 OAQ Sensor:
```cpp
#include <stdio.h>
#include "ZMOD4510.h"

void app_main(void){
  // Temperature and humidity required by renesas's algorithm to calculate OAQ.
  float humidity = 50.0;
	float temperature = 24.0;

	zmod4510_dev_t zmod4510;
	zmod4510_init_sensor(&zmod4510);

  while (true) {
      zmod4510_update_sensor_data(&zmod4510, temperature, humidity);
      // library logs data received by sensor and calculations from library.
      // Take care that there's a 900 sample "warmup" routine the sensor has to do before values are valid.
      // (usually takes 20 minutes)
      printf("Fast AQI: %d, EPA AQI: %d, O3 concentration [ppb] %.2f\n",zmod4510.FAST_AQI,zmod4510.EPA_AQI,zmod4510.O3_conc_ppb);
      sleep(1);
  }
}

```

# TODO
- Add example project
- Add interrupt features. It currently has a faulty implementation.
- Implement proper testing.
