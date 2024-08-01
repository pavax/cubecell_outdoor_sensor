# Cubecell Lorawan Weather Station

## Setup

**Microcontroller**

I'm using the Heltec Cubecell AB-02 Microcontroller. It is fantastic for outdoor Lorawan projects as it consumes only 10uA when it sleeps and comes out-of-the-box with a solar-charging capability.
It also has a build in mechanism (High-Side Mosfet Switch) to turn the power-line to the sensors on and off. 

 *  https://resource.heltec.cn/download/CubeCell/HTCC-AB02/HTCC-AB02_PinoutDiagram.pdf

**Sensors**

This project uses the following sensors:

 * MB7374: Ultrasonic Distance Sensor
	 * Used to measure the snow-depth
	 * Protocol: Serial
	 * https://maxbotix.com/products/mb7374

 * RG-15: Rain Sensor 
	 * This is an optical rain tipping bucket sensor. It measures the amount and intensity of rain.
	 * Protocol: Serial
	 * https://rainsensors.com/products/rg-15/

 * Wind Speed Sensor 
   * The Wind-Speed is measured every 60sec. The median and max values are then send at every duty cycle.
	 * Protocol: Analog 0-4V (Needs voltage divider to output max 2.4V for the adc input)
	 * https://www.aliexpress.com/item/1005002627447076.html

 * Wind Direction Sensor
	 * Protocol: Analog 0-5V (Needs voltage divider to output max 2.4V for the adc input))
	 * https://www.aliexpress.com/item/1005006845253011.html

 * SHT-40: Temperatur and Humidity Sensor
	 * Protocol: I2C

 * BME-280: Temperatur, Humidity and Pressure Sensor
	 *  Protocol: I2C


## Javascript Uplink Decoding

     function parse(payloadHex, port) {

          function hexToBytes(hex) {
              for (var bytes = [], c = 0; c < hex.length; c += 2)
                  bytes.push(parseInt(hex.substr(c, 2), 16));
              return bytes;
          }

          function calculateDewPoint(temperature, humidity) {
              // Constants for the Magnus-Tetens approximation
              const a = 17.27;
              const b = 237.7;

              // Calculate alpha
              const alpha = ((a * temperature) / (b + temperature)) + Math.log(humidity / 100);

              // Calculate the dew point
              let dewPoint = (b * alpha) / (a - alpha);

              // Round to two decimal places
              dewPoint = Math.round(dewPoint * 100) / 100;

              return dewPoint;
          }

          var bytes = hexToBytes(payloadHex);
          let msg = {};
          msg.counter = (bytes[0] << 8) | bytes[1];
          msg.batteryVoltage = (bytes[2] << 8) | bytes[3];
          msg.temperature = (bytes[4] << 24 >> 16 | bytes[5]) / 100;  // signed
          msg.humidity = ((bytes[6] << 8) | bytes[7]) / 100;
          msg.distance = (bytes[8] << 8) | bytes[9];
          msg.windVoltage = (bytes[10] << 24 >> 16 | bytes[11]); // signed
          msg.windVoltageMax = (bytes[12] << 24 >> 16 | bytes[13]); // signed
          msg.rainEventCounter = (bytes[14] << 8) | bytes[15];
          msg.rainAcc = ((bytes[16] << 8) | bytes[17]) / 100;
          msg.rainEventAcc = ((bytes[18] << 8) | bytes[19]) / 100;
          msg.rainTotalAcc = ((bytes[20] << 8) | bytes[21]) / 100;
          msg.rainRInt = ((bytes[22] << 8) | bytes[23]) / 100;
          msg.rainSerialCode = bytes[24];
          msg.rainLastDetection = (((bytes[25] << 24) | (bytes[26] << 16) | (bytes[27] << 8) | bytes[28]));
          msg.windDirection = bytes[29];
          msg.temperature2 = (bytes[30] << 24 >> 16 | bytes[31]) / 100;  // signed
          msg.humidity2 = ((bytes[32] << 8) | bytes[33]) / 100;
          msg.pressure = ((bytes[34] << 24) | (bytes[35] << 16) | (bytes[36] << 8) | bytes[37]) / 100;
          msg.dewPoint = calculateDewPoint(msg.temperature, msg.humidity);
          
          return msg;
      }


## Downlink Commands

Important: Send Payload withouth the '0x' prefix.

| Port | Command 	                                                 | Payload Examples
|------|-----------------------------------------------------------|-------------------------------------------------------------------|
|  4   | Set the Duty Cycle Time [seconds]                         | 0x003C -> every 1 minute <br> 0x0078 -> every 2 minutes <br> 0x012C -> every 5 minutes <br> 0x0258 -> every 10 minutes <br> 0x0384 -> every 15 minutes <br> 0x04B0 -> every 20 minutes
|  5   | Turn the RGB Light on/off                                 | 0x1111 -> turn the light on <br> 0x0000 -> turn the light off
|  6   | Send a Serial-Command to the Rain Sensor                  | 0x006B -> send the 'k' command
|  7   | Set the background measurements interval time [seconds]   | 0x001E -> every 30 seconds <br> 0x003C -> every 1 minute <br> 0x0078 -> every 2 minutes
|  9   | Restart the Microcontroller                               | - 
