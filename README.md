# LoRaAmbientMonitor_IoCheck

This Arduino sketch is the elementary basic software for the hardware project 
[LoRaAmbientMonitor_PCB](https://github.com/ronaldsieber/LoRaAmbientMonitor_PCB) and uses all the peripherals of the board.

On the one hand, the sketch can be used to check that all components are working correctly after the board has been assembled. On the other hand, the sketch is a good starting point for new software projects based on this board.

## Sketch Functionalities

The sketch implements the following functionalities:

The values ​​of the sensors and configuration elements are displayed cyclically in the serial Terminal Window (115200Bd) as well as on the OLED Display of the ESP32 LoRa Development Board.

**Power on / Reset:**
LED101...LED103 are turned on, one LED after the other, sequentially

**User button SW1:**
LED101...LED103 turned on permanently as long as the button is pressed

**4-Way DIP Switch SW2:**
The configuration of the DIP Switch is displayed cyclically in the terminal window and at OLED display:
- DIP2 = on -> D102 (green) permanently on
- DIP3 = on -> D101 (red) permanently on
- DIP4 = on -> D103 (yellow) permanently on

**HC-SR501 (PIR Motion Detector):**
- The movement status (active/inactive) is displayed cyclically in the terminal window and OLED display
- The movement status (active/inactive) is visualized by D101 (red)

**DHT22 (Temperature and Humidity):**
The measured values ​​are displayed cyclically in the terminal window and OLED display

**ALS-PDIC243 (Ambient Light Sensor):**
The measured value of the light sensor is displayed cyclically in the terminal window and OLED display

**12V Input for measuring Car Battery Voltage:**
- The measured value of the battery voltage is displayed cyclically in the terminal window and OLED display
- A battery voltage > 2V is visualized by D103 (yellow)

The current board values are shown in compact form in the OLED Display of the ESP32 LoRa Development Board:

![\[Sensor_Data_OLED_Output\]](Documentation/Sensor_Data_OLED_Output.png)

The cyclic output in the serial Terminal Window (115200Bd) contains additional, explanatory information:

![\[Sensor_Data_Terminal_Output\]](Documentation/Sensor_Data_Terminal_Output.png)

## Configuration Section

At the beginning of the sketch there is the following configuration section:

    const int  CFG_ENABLE_OLED_DISPLAY          = 1;
    const int  CFG_ENABLE_DHT_SENSOR            = 1;
    const int  CFG_ENABLE_SEN_HC_SR501_SENSOR   = 1;
    const int  CFG_ENABLE_ADS1115_LIGHT_SENSOR  = 1;
    const int  CFG_ENABLE_ADS1115_CAR_BATT_AIN  = 1;

This enables the runtime execution of the associated code sections to be activated *(= 1)* or disabled *(= 0)*. This allows the sketch to be used for boards on which not all components are fitted, without the lack of components leading to runtime errors.

The positioning of the outputs on the OLED Display of the ESP32 LoRa Development Board is done via the following section:


    const uint8_t  OLED_LINE_DIP_SWITCH   = 0;
    const uint8_t  OLED_LINE_TEMPERATURE  = 1;
    const uint8_t  OLED_LINE_HUMIDITY     = 2;
    const uint8_t  OLED_LINE_MOTION       = 3;
    const uint8_t  OLED_LINE_ADC0         = 4;
    const uint8_t  OLED_LINE_LIGHT        = 5;
    const uint8_t  OLED_LINE_ADC1         = 6;
    const uint8_t  OLED_LINE_CAR_BATT     = 7;
    const uint8_t  OLED_OFFS_DATA_VALUE   = 8;



