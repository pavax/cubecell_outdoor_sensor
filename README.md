# Cubecell Lorawan Weather Station

## Setup

**Microcontroller**
I'm using the Heltec Cubecell AB-02 Microcontroller. It is fantastic for outdoor Lorawan projects as it consumes only 10uA when it sleeps and comes out-of-the-box with a solar-charging capability. 
 *  https://resource.heltec.cn/download/CubeCell/HTCC-AB02/HTCC-AB02_PinoutDiagram.pdf

**Sensors**
This project uses the following sensors and uses the fabulous Cubecell AB-02 Microcontroller. 

 * MB7374: Ultrasonic Distance Sensor
	 * Used to measure the snow-depth
	 * Protocol: Serial
	 *  https://maxbotix.com/products/mb7374
 * RG-15: Rain Sensor 
	 * This is an optical rain Tipping buck sensor. It measures the amount and intensity of rain.
	 * Protocol: Serial
	 * https://rainsensors.com/products/rg-15/
 * Wind Speed Sensor 
	 * Protocol: Analog 0-4V (Needs voltage divider to output max 2.4V for Analog Input)
	 * https://www.aliexpress.com/item/1005002627447076.html
 * Wind Direction Sensor
	 * Protocol: Analog 0-5V (Needs voltage divider to output max 2.4V for Analog Input))
	 * https://www.aliexpress.com/item/1005006845253011.html
 * SHT-40: Temperatur and Humidity Sensor
	 * Protocol: I2C
 * BME-280: Temperatur, Humidity and Pressure Sensor
	 *  Protocol: I2C


## Downlink Commands

Important: Send Payload withouth the '0x' prefix.

| Port | Command 	                                  | Payload Examples
|------|--------------------------------------------|-------------------------------------------------------------------|
|  4   | Set the Duty Cycle Time                    | 0x003C -> every 1 minute <br> 0x0078 -> every 2 minute <br> 0x012C -> every 5 minute <br> 0x0258 -> every 10 minutes <br> 0x0384 -> every 15 minutes <br> 0x04B0 -> every 20 minutes
|  5   | Turn the RGB Light on/off                  | 0x1111 -> turn the light on <br> 0x0000 -> turn the light off
|  6   | Send a Serial-Command to the Rain Sensor   | 0x1111 -> turn the light on <br> 0x0000 -> turn the light off
|  7   | Send a Serial-Command to the Rain Sensor   | 0x006B -> send the 'k' command
|  9   | Restart the Microcontroller                | - 
