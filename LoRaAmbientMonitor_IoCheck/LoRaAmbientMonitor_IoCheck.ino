/****************************************************************************

  Copyright (c) 2022 Ronald Sieber

  Project:      LoRa Ambient Monitor
  Description:  Basic IO Routines for Commisioning of LoRaAmbientMonitor_PCB

  -------------------------------------------------------------------------

    Arduino IDE Settings:

    Board:              "ESP32 Dev Module"
    Upload Speed:       "115200"
    CPU Frequency:      "240MHz (WiFi/BT)"
    Flash Frequency:    "80Mhz"
    Flash Mode:         "QIO"
    Flash Size:         "4MB (32Mb)"
    Partition Scheme:   "No OTA (2MB APP/2MB SPIFFS)"
    PSRAM:              "Disabled"

  -------------------------------------------------------------------------

  Revision History:

  2022/05/21 -rs:   V1.00 Initial version

****************************************************************************/


#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <U8x8lib.h>
#include <DHT.h>





/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
//  Application Configuration
//---------------------------------------------------------------------------

const int       APP_VERSION                         = 1;                // 1.xx
const int       APP_REVISION                        = 0;                // x.00
const char      APP_BUILD_TIMESTAMP[]               = __DATE__ " " __TIME__;

const int       CFG_ENABLE_OLED_DISPLAY             = 1;
const int       CFG_ENABLE_DHT_SENSOR               = 1;
const int       CFG_ENABLE_SEN_HC_SR501_SENSOR      = 1;
const int       CFG_ENABLE_ADS1115_LIGHT_SENSOR     = 1;
const int       CFG_ENABLE_ADS1115_CAR_BATT_AIN     = 1;

const uint8_t   OLED_LINE_DIP_SWITCH                = 0;
const uint8_t   OLED_LINE_TEMPERATURE               = 1;
const uint8_t   OLED_LINE_HUMIDITY                  = 2;
const uint8_t   OLED_LINE_MOTION                    = 3;
const uint8_t   OLED_LINE_ADC0                      = 4;
const uint8_t   OLED_LINE_LIGHT                     = 5;
const uint8_t   OLED_LINE_ADC1                      = 6;
const uint8_t   OLED_LINE_CAR_BATT                  = 7;
const uint8_t   OLED_OFFS_DATA_VALUE                = 8;

const float     ADS1115_DIGIT                       = ((float)6144/(float)32768);   // max. ADC Input Voltage: 6.144V / Resulution: 15Bit with Signum => 0.1875mV
const float     ADC_CAR_BATT_VOLTAGE_DIVIDER        = ((float)4.78);                // R6=68k/R7=18k -> 12V/4.78 = 2.51V@ADC1
const float     ADC_CAR_BATT_RPD_FORWARD_VOLTAGE    = ((float)0.5);                 // Forward Voltage of Reverse Polarity Protection Diode D2 at CarBatt Input Connector



//---------------------------------------------------------------------------
//  Hardware/Pin Configuration
//---------------------------------------------------------------------------

const int       PIN_SW_DIP1                         = 36;               // SW_DIP1              (GPIO36 -> Pin33)
const int       PIN_SW_DIP2                         = 37;               // SW_DIP2              (GPIO37 -> Pin32)
const int       PIN_SW_DIP3                         = 38;               // SW_DIP3              (GPIO38 -> Pin31)
const int       PIN_SW_DIP4                         = 39;               // SW_DIP4              (GPIO39 -> Pin30)

const int       PIN_KEY_CFG                         = 13;               // KEY_CFG              (GPIO13 -> Pin20)

const int       PIN_LED_MOTION_ACTIVE               = 25;               // LED_MOTION_ACTIVE    (GPIO25 -> Pin25)   (Red)    (= on-board LED (white))
const int       PIN_LED_LORA_TRANSMIT               = 12;               // LED_LORA_TRANSMIT    (GPIO12 -> Pin21)   (Green)
const int       PIN_LED_CAR_BATT_PLUGGED            = 23;               // LED_CAR_BATT_PLUGGED (GPIO23 -> Pin11)   (Yellow)

const int       PIN_OLED_SCL                        = 15;               // GPIO Pin connected to OLED_SCL (GPIO15 -> Pin13)
const int       PIN_OLED_SDA                        =  4;               // GPIO Pin connected to OLED_SDA (GPIO04 -> Pin16)
const int       PIN_OLED_RST                        = 16;               // GPIO Pin connected to OLED_RST (GPIO16 -> Pin18)

const int       TYPE_DHT                            = DHT22;            // DHT11, DHT21 (AM2301), DHT22 (AM2302,AM2321)
const int       PIN_DHT22_SENSOR                    = 17;               // PIN used for DHT22 (AM2302/AM2321)   (GPIO17 -> Pin17)
const uint32_t  DHT_SENSOR_SAMPLE_PERIOD            = 1000;             // Sample Period for DHT-Sensor in [ms]

const int       PIN_SEN_HC_SR501                    =  2;               // GPIO Pin connected to HC-SR501 sensor (IR Motion Sensor) (GPIO02 -> Pin15)

const uint8_t   ADC_CH_LIGHT_SENSOR                 =  0;               // ADC[0] = Light Sensor
const uint8_t   ADC_CH_CAR_BATT_AIN                 =  1;               // ADC[1] = CarBattery Level



//---------------------------------------------------------------------------
//  Local types
//---------------------------------------------------------------------------

typedef struct
{
    uint8_t     m_fDip1 : 1;                                            // DIP '1' (left most)
    uint8_t     m_fDip2 : 1;                                            // DIP '2'
    uint8_t     m_fDip3 : 1;                                            // DIP '3'
    uint8_t     m_fDip4 : 1;                                            // DIP '4' (right most)

} tDipSwitch;



typedef struct
{
    ulong       m_ulMainLoopCycle;
    uint32_t    m_ui32Uptime;
    tDipSwitch  m_DipBits;
    uint8_t     m_ui8DipValue;
    float       m_flTemperature;
    float       m_flHumidity;
    bool        m_fMotionActive;
    uint16_t    m_ui16MotionActiveTime;
    uint16_t    m_ui16AdcValLightLevel;
    uint8_t     m_ui8LightLevel;
    uint16_t    m_ui16AdcValCarBattLevel;
    float       m_flCarBattLevel;

} tSensorData;




//---------------------------------------------------------------------------
//  Local variables
//---------------------------------------------------------------------------

static  DHT                                 DhtSensor_g(PIN_DHT22_SENSOR, TYPE_DHT);
static  Adafruit_ADS1115                    Ads1115_g;                  // I2C Address = 0x48 is hardcoded in Adafruit ADS1115 Library
static  U8X8_SSD1306_128X64_NONAME_SW_I2C   Oled_U8x8_g(PIN_OLED_SCL, PIN_OLED_SDA, PIN_OLED_RST);

static  ulong           ulMainLoopCycle_g           = 0;
static  uint            uiMainLoopProcStep_g        = 0;
static  uint32_t        ui32LastTickDhtRead_g       = 0;

static  tSensorData     SensorDataRec_g             = { 0 };
static  tSensorData     PrevSensorDataRec_g         = { 0 };

static  uint32_t        ui32StartTickMotionSegm_g   = 0;

static  bool            fCfgBtnState_g              = false;
static  bool            fLedLoRaTransmit_g          = LOW;
static  bool            fLedMotionActive_g          = LOW;
static  bool            fLedCarBattPlugged_g        = LOW;



//---------------------------------------------------------------------------
//  Local functions
//---------------------------------------------------------------------------





//=========================================================================//
//                                                                         //
//          S K E T C H   P U B L I C   F U N C T I O N S                  //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Application Setup
//---------------------------------------------------------------------------

void setup()
{

char  szTextBuff[64];


    // Serial console
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    Serial.println("======== APPLICATION START ========");
    Serial.println();
    Serial.flush();


    // Application Version Information
    snprintf(szTextBuff, sizeof(szTextBuff), "App Version:      %u.%02u", APP_VERSION, APP_REVISION);
    Serial.println(szTextBuff);
    snprintf(szTextBuff, sizeof(szTextBuff), "Build Timestamp:  %s", APP_BUILD_TIMESTAMP);
    Serial.println(szTextBuff);
    Serial.println();
    Serial.flush();


    // Initialize Workspace
    ulMainLoopCycle_g     = 0;
    uiMainLoopProcStep_g  = 0;
    ui32LastTickDhtRead_g = 0;
    fCfgBtnState_g        = false;
    fLedLoRaTransmit_g    = LOW;
    fLedMotionActive_g    = LOW;
    fLedCarBattPlugged_g  = LOW;
    memset(&SensorDataRec_g, 0x00, sizeof(SensorDataRec_g));
    memset(&PrevSensorDataRec_g, 0x00, sizeof(PrevSensorDataRec_g));


    // Setup Status LEDs
    Serial.println("Setup Status LEDs...");
    StatusLedsSetup();


    // Setup Config Button
    Serial.println("Setup Config Button...");
    CfgBtnSetup();


    // Setup DIP Switch
    Serial.println("Setup DIP Switch...");
    DipSetup();
    PrevSensorDataRec_g.m_ui8DipValue = 0xFF;


    // Setup OLED Display
    if ( CFG_ENABLE_OLED_DISPLAY )
    {
        Serial.println("Setup OLED Display...");
        Oled_U8x8_g.begin();
        Oled_U8x8_g.setFont(u8x8_font_chroma48medium8_r);
        Oled_U8x8_g.clear();
        OledShowStartScreen();
    }


    // Setup DHT22 Sensor (Temerature/Humidity)
    if ( CFG_ENABLE_DHT_SENSOR )
    {
        Serial.println("Setup DHT22 Sensor...");
        DhtSensor_g.begin();
        snprintf(szTextBuff, sizeof(szTextBuff), "  Sample Period:    %d [sec]", (DHT_SENSOR_SAMPLE_PERIOD / 1000));
        Serial.println(szTextBuff);
    }


    // Setup ADS1115 ADC @ I2C
    if ( CFG_ENABLE_ADS1115_LIGHT_SENSOR || CFG_ENABLE_ADS1115_CAR_BATT_AIN )
    {
        Serial.println("Setup ADS1115 ADC...");
        Ads1115_g.begin();
    }


    // Setup SEN-HC-SR501 Sensor (IR Motion Sensor)
    if ( CFG_ENABLE_SEN_HC_SR501_SENSOR )
    {
        Serial.println("Setup SEN_HC_SR501 Sensor...");
        pinMode(PIN_SEN_HC_SR501, INPUT);
        ui32StartTickMotionSegm_g = 0;
    }


    // Test all LED'a
    RunLedSystemTest();

    return;

}



//---------------------------------------------------------------------------
//  Application Main Loop
//---------------------------------------------------------------------------

void loop()
{

char      szTextBuff[128];
uint32_t  ui32CurrTick;
uint      uiProcStep;
uint32_t  ui32Uptime;
String    strUptime;
float     flTemperature;
float     flHumidity;
bool      fMotionActive;
uint16_t  ui16MotionActiveTime;
uint16_t  ui16AdcVal;
uint8_t   ui8LightLevel;
float     flCarBattLevel;
int       iRes;


    uiProcStep = uiMainLoopProcStep_g++ % 10;
    switch (uiProcStep)
    {
        // provide main information
        case 0:
        {
            ulMainLoopCycle_g++;
            Serial.println();
            strUptime = GetSysUptime(&ui32Uptime);
            snprintf(szTextBuff, sizeof(szTextBuff), "Main Loop Cycle:       %lu (Uptime: %s sec)", ulMainLoopCycle_g, strUptime.c_str());
            Serial.println(szTextBuff);
            SensorDataRec_g.m_ulMainLoopCycle = ulMainLoopCycle_g;
            SensorDataRec_g.m_ui32Uptime = ui32Uptime;

            SensorDataRec_g.m_ui8DipValue = DipGetSetting(&SensorDataRec_g.m_DipBits);
            snprintf(szTextBuff, sizeof(szTextBuff), "DIP Switch:            %u%u%u%ub (%02XH)", (unsigned int)(SensorDataRec_g.m_DipBits.m_fDip1 & 0x01),
                                                                                                 (unsigned int)(SensorDataRec_g.m_DipBits.m_fDip2 & 0x01),
                                                                                                 (unsigned int)(SensorDataRec_g.m_DipBits.m_fDip3 & 0x01),
                                                                                                 (unsigned int)(SensorDataRec_g.m_DipBits.m_fDip4 & 0x01),
                                                                                                 (unsigned int)SensorDataRec_g.m_ui8DipValue);
            Serial.println(szTextBuff);
            break;
        }

        // process DHT Sensor (Temperature/Humidity)
        case 1:
        {
            if ( CFG_ENABLE_DHT_SENSOR )
            {
                Serial.print("DHT22 Sensor:          ");
                ui32CurrTick = millis();
                if ((ui32CurrTick - ui32LastTickDhtRead_g) < DHT_SENSOR_SAMPLE_PERIOD)
                {
                    Serial.println("[skipped]");
                    break;
                }
                iRes = GetDhtSensorData(&flTemperature, &flHumidity);
                if (iRes == 0)
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "Temperature = %.1f °C, Humidity = %.1f %%", flTemperature, flHumidity);
                }
                else
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "FAILED! (iRes=%d)", iRes);
                }
                Serial.println(szTextBuff);
                SensorDataRec_g.m_flTemperature = flTemperature;
                SensorDataRec_g.m_flHumidity = flHumidity;
                ui32LastTickDhtRead_g = ui32CurrTick;
            }
            break;
        }

        // process SEN-HC-SR501 Sensor (IR Motion Sensor)
        case 2:
        {
            if ( CFG_ENABLE_SEN_HC_SR501_SENSOR )
            {
                Serial.print("SEN_HC_SR501 Sensor:   ");
                iRes = GetSenHcSr501SensorData(&fMotionActive, &ui16MotionActiveTime);
                if (iRes == 0)
                {
                    if ( fMotionActive )
                    {
                        snprintf(szTextBuff, sizeof(szTextBuff), "Active (for %u sec)", (uint)ui16MotionActiveTime);
                    }
                    else
                    {
                        snprintf(szTextBuff, sizeof(szTextBuff), "Idle");
                    }
                }
                else
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "FAILED! (iRes=%d)", iRes);
                }
                Serial.println(szTextBuff);
                SensorDataRec_g.m_fMotionActive = fMotionActive;
                SensorDataRec_g.m_ui16MotionActiveTime = ui16MotionActiveTime;

                fLedMotionActive_g = fMotionActive;
            }
            break;
        }

        // process Light Sensor (AI0 @ ADS1115)
        case 3:
        {
            if ( CFG_ENABLE_ADS1115_LIGHT_SENSOR )
            {
                Serial.print("ADS1115/Light Sensor:  ");
                iRes = GetAds1115LightSensorData(&ui16AdcVal, &ui8LightLevel);
                if (iRes == 0)
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "LightLevel = %u %% (ADC=%05u)", (uint)ui8LightLevel, ui16AdcVal);
                }
                else
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "FAILED! (iRes=%d)", iRes);
                }
                Serial.println(szTextBuff);
                SensorDataRec_g.m_ui16AdcValLightLevel = ui16AdcVal;
                SensorDataRec_g.m_ui8LightLevel = ui8LightLevel;
            }
            break;
        }

        // process CarBatt AnalogIn (AI1 @ ADS1115)
        case 4:
        {
            if ( CFG_ENABLE_ADS1115_CAR_BATT_AIN )
            {
                Serial.print("ADS1115/CarBatt AIN:   ");
                iRes = GetAds1115CarBattAinData(&ui16AdcVal, &flCarBattLevel);
                if (iRes == 0)
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "CarBattLevel = %.1f V (ADC=%05u)", flCarBattLevel, ui16AdcVal);
                }
                else
                {
                    snprintf(szTextBuff, sizeof(szTextBuff), "NOT_CONNECTED (iRes=%d)", iRes);
                }
                Serial.println(szTextBuff);
                SensorDataRec_g.m_ui16AdcValCarBattLevel = ui16AdcVal;
                SensorDataRec_g.m_flCarBattLevel = flCarBattLevel;

                if (SensorDataRec_g.m_flCarBattLevel >= 1.0)
                {
                    fLedCarBattPlugged_g = HIGH;
                }
                else
                {
                    fLedCarBattPlugged_g = LOW;
                }
            }
            break;
        }

        // get Config Button state
        case 5:
        {
            // turn on all LEDs as long as Config Button is pressed
            fCfgBtnState_g = CfgBtnGetState();
            break;
        }

        // simulate LoRa Transmission
        case 6:
        {
            if ( ((uiMainLoopProcStep_g / 10) % 5) == 0 )
            {
                fLedLoRaTransmit_g = HIGH;
            }
            break;
        }
        case 7:
        {
            fLedLoRaTransmit_g = LOW;
            break;
        }

        case 8:
        {
            break;
        }

        // update process information at OLED
        case 9:
        {
            OledUpdateProcessData(&SensorDataRec_g, &PrevSensorDataRec_g);
            break;
        }

        default:
        {
            break;
        }
    }


    // set Status LEDs
    SetLedLoRaTransmit(   fLedLoRaTransmit_g   | SensorDataRec_g.m_DipBits.m_fDip2 | fCfgBtnState_g );
    SetLedMotionActive(   fLedMotionActive_g   | SensorDataRec_g.m_DipBits.m_fDip3 | fCfgBtnState_g );
    SetLedCarBattPlugged( fLedCarBattPlugged_g | SensorDataRec_g.m_DipBits.m_fDip4 | fCfgBtnState_g );

    delay(100);

    return;

}





//=========================================================================//
//                                                                         //
//          S K E T C H   P R I V A T E   F U N C T I O N S                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  LEDs: Setup GPIO Pins for Status LEDs
//---------------------------------------------------------------------------

void  StatusLedsSetup (void)
{

    // Status LEDs Setup
    pinMode(PIN_LED_MOTION_ACTIVE, OUTPUT);
    digitalWrite(PIN_LED_MOTION_ACTIVE, LOW);
    pinMode(PIN_LED_LORA_TRANSMIT, OUTPUT);
    digitalWrite(PIN_LED_LORA_TRANSMIT, LOW);
    pinMode(PIN_LED_CAR_BATT_PLUGGED, OUTPUT);
    digitalWrite(PIN_LED_CAR_BATT_PLUGGED, LOW);

    return;

}



//---------------------------------------------------------------------------
//  LEDs: Set for State 'MotionActive'
//---------------------------------------------------------------------------

void  SetLedMotionActive (bool fLedState_p)
{

    digitalWrite(PIN_LED_MOTION_ACTIVE, (fLedState_p ? HIGH : LOW));
    return;

}



//---------------------------------------------------------------------------
//  LEDs: Set for State 'LoRaTransmit'
//---------------------------------------------------------------------------

void  SetLedLoRaTransmit (bool fLedState_p)
{

    digitalWrite(PIN_LED_LORA_TRANSMIT, (fLedState_p ? HIGH : LOW));
    return;

}



//---------------------------------------------------------------------------
//  LEDs: Set for State 'CarBattPlugged'
//---------------------------------------------------------------------------

void  SetLedCarBattPlugged (bool fLedState_p)
{

    digitalWrite(PIN_LED_CAR_BATT_PLUGGED, (fLedState_p ? HIGH : LOW));
    return;

}



//---------------------------------------------------------------------------
//  LEDs: Run LED System Test
//---------------------------------------------------------------------------

void  RunLedSystemTest (void)
{

    digitalWrite(PIN_LED_MOTION_ACTIVE, LOW);
    digitalWrite(PIN_LED_LORA_TRANSMIT, LOW);
    digitalWrite(PIN_LED_CAR_BATT_PLUGGED, LOW);
    delay(100);

    digitalWrite(PIN_LED_LORA_TRANSMIT, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_LORA_TRANSMIT, LOW);
    delay(100);

    digitalWrite(PIN_LED_MOTION_ACTIVE, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_MOTION_ACTIVE, LOW);
    delay(100);

    digitalWrite(PIN_LED_CAR_BATT_PLUGGED, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_CAR_BATT_PLUGGED, LOW);
    delay(100);

    digitalWrite(PIN_LED_MOTION_ACTIVE, LOW);
    digitalWrite(PIN_LED_LORA_TRANSMIT, LOW);
    digitalWrite(PIN_LED_CAR_BATT_PLUGGED, LOW);

    return;

}



//---------------------------------------------------------------------------
//  DIP: Setup GPIO Pins for DIP-Switch
//---------------------------------------------------------------------------

void  DipSetup (void)
{

    pinMode(PIN_SW_DIP1, INPUT);
    pinMode(PIN_SW_DIP2, INPUT);
    pinMode(PIN_SW_DIP3, INPUT);
    pinMode(PIN_SW_DIP4, INPUT);

    return;

}



//---------------------------------------------------------------------------
//  DIP: Get DIP-Switch Setting
//---------------------------------------------------------------------------

uint8_t  DipGetSetting (tDipSwitch* pDipBits_o)
{

tDipSwitch  DipBits;
bool        fDipPin;
uint8_t     ui8DipValue;


    ui8DipValue = 0x00;

    fDipPin = !digitalRead(PIN_SW_DIP1);                // DIPs are inverted (1=off, 0=on)
    DipBits.m_fDip1 = fDipPin;
    ui8DipValue |= (fDipPin & 0x01) << 3;

    fDipPin = !digitalRead(PIN_SW_DIP2);                // DIPs are inverted (1=off, 0=on)
    DipBits.m_fDip2 = fDipPin;
    ui8DipValue |= (fDipPin & 0x01) << 2;

    fDipPin = !digitalRead(PIN_SW_DIP3);                // DIPs are inverted (1=off, 0=on)
    DipBits.m_fDip3 = fDipPin;
    ui8DipValue |= (fDipPin & 0x01) << 1;

    fDipPin = !digitalRead(PIN_SW_DIP4);                // DIPs are inverted (1=off, 0=on)
    DipBits.m_fDip4 = fDipPin;
    ui8DipValue |= (fDipPin & 0x01) << 0;

    if (pDipBits_o != NULL)
    {
        *pDipBits_o = DipBits;
    }

    return (ui8DipValue);

}



//---------------------------------------------------------------------------
//  CfgBtn: Setup GPIO Pin for Config Button
//---------------------------------------------------------------------------

void  CfgBtnSetup (void)
{

    pinMode(PIN_KEY_CFG, INPUT);
    return;

}



//---------------------------------------------------------------------------
//  CfgBtn: Get Config Button State
//---------------------------------------------------------------------------

bool  CfgBtnGetState (void)
{

bool  fKeyPin;


    fKeyPin = !digitalRead(PIN_KEY_CFG);                // DIPs are inverted (1=off, 0=on)
    return (fKeyPin & 0x01);

}



//---------------------------------------------------------------------------
//  OLED: Show Start Screen
//---------------------------------------------------------------------------

void  OledShowStartScreen (void)
{

    //---------------------------------------------- |0123456789ABCDEF|
    Oled_U8x8_g.clear();
    Oled_U8x8_g.drawString(0, OLED_LINE_DIP_SWITCH,  "DIP:    ---- -- ");
    Oled_U8x8_g.drawString(0, OLED_LINE_TEMPERATURE, "Temp:   --.- C  ");
    Oled_U8x8_g.drawString(0, OLED_LINE_HUMIDITY,    "rH:     --- %   ");
    Oled_U8x8_g.drawString(0, OLED_LINE_MOTION,      "Motion: ------  ");
    Oled_U8x8_g.drawString(0, OLED_LINE_ADC0,        "ADC0:   -----   ");
    Oled_U8x8_g.drawString(0, OLED_LINE_LIGHT,       "Light:  --- %   ");
    Oled_U8x8_g.drawString(0, OLED_LINE_ADC1,        "ADC1:   -----   ");
    Oled_U8x8_g.drawString(0, OLED_LINE_CAR_BATT,    "Batt:   --.-- V ");

    return;

}



//---------------------------------------------------------------------------
//  OLED: Update Process Data
//---------------------------------------------------------------------------

void  OledUpdateProcessData (const tSensorData* pSensorDataRec_p, tSensorData* pPrevSensorDataRec_p)
{

char  szOledData[16 - OLED_OFFS_DATA_VALUE + sizeof('\0')];


    // update DIP Switch
    if (pSensorDataRec_p->m_ui8DipValue != pPrevSensorDataRec_p->m_ui8DipValue)
    {
        memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
        snprintf(szOledData, sizeof(szOledData), "%u%u%u%u %02X", (unsigned int)(pSensorDataRec_p->m_DipBits.m_fDip1 & 0x01),
                                                                  (unsigned int)(pSensorDataRec_p->m_DipBits.m_fDip2 & 0x01),
                                                                  (unsigned int)(pSensorDataRec_p->m_DipBits.m_fDip3 & 0x01),
                                                                  (unsigned int)(pSensorDataRec_p->m_DipBits.m_fDip4 & 0x01),
                                                                  (unsigned int)pSensorDataRec_p->m_ui8DipValue);
        OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_DIP_SWITCH);
    }
    pPrevSensorDataRec_p->m_ui8DipValue = pSensorDataRec_p->m_ui8DipValue;
    pPrevSensorDataRec_p->m_DipBits     = pSensorDataRec_p->m_DipBits;

    // update DHT Sensor (Temperature/Humidity)
    if ( CFG_ENABLE_DHT_SENSOR )
    {
        if (pSensorDataRec_p->m_flTemperature != pPrevSensorDataRec_p->m_flTemperature)
        {
            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            snprintf(szOledData, sizeof(szOledData), "%.1f C", pSensorDataRec_p->m_flTemperature);
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_TEMPERATURE);
        }
        pPrevSensorDataRec_p->m_flTemperature = pSensorDataRec_p->m_flTemperature;

        if (pSensorDataRec_p->m_flHumidity != pPrevSensorDataRec_p->m_flHumidity)
        {
            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            snprintf(szOledData, sizeof(szOledData), "%.1f %%", pSensorDataRec_p->m_flHumidity);
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_HUMIDITY);
        }
        pPrevSensorDataRec_p->m_flHumidity = pSensorDataRec_p->m_flHumidity;
    }

    // update SEN-HC-SR501 Sensor (IR Motion Sensor)
    if ( CFG_ENABLE_SEN_HC_SR501_SENSOR )
    {
        if (pSensorDataRec_p->m_fMotionActive != pPrevSensorDataRec_p->m_fMotionActive)
        {
            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            if ( SensorDataRec_g.m_fMotionActive )
            {
                strncpy(szOledData, "Active", sizeof(szOledData));
            }
            else
            {
                strncpy(szOledData, "Idle  ", sizeof(szOledData));
            }
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_MOTION);
        }
        pPrevSensorDataRec_p->m_fMotionActive = pSensorDataRec_p->m_fMotionActive;
    }

    // update Light Sensor (AI0 @ ADS1115)
    if ( CFG_ENABLE_ADS1115_LIGHT_SENSOR )
    {
        if (pSensorDataRec_p->m_ui16AdcValLightLevel != pPrevSensorDataRec_p->m_ui16AdcValLightLevel)
        {
            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            snprintf(szOledData, sizeof(szOledData), "%u", (uint)(pSensorDataRec_p->m_ui16AdcValLightLevel));
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_ADC0);

            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            snprintf(szOledData, sizeof(szOledData), "%u %%", (uint)(pSensorDataRec_p->m_ui8LightLevel));
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_LIGHT);
        }
        pPrevSensorDataRec_p->m_ui16AdcValLightLevel = pSensorDataRec_p->m_ui16AdcValLightLevel;
        pPrevSensorDataRec_p->m_ui8LightLevel        = pSensorDataRec_p->m_ui8LightLevel;
    }

    // update CarBatt AnalogIn (AI1 @ ADS1115)
    if ( CFG_ENABLE_ADS1115_CAR_BATT_AIN )
    {
        if (pSensorDataRec_p->m_ui16AdcValCarBattLevel != pPrevSensorDataRec_p->m_ui16AdcValCarBattLevel)
        {
            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            snprintf(szOledData, sizeof(szOledData), "%u", (uint)(pSensorDataRec_p->m_ui16AdcValCarBattLevel));
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_ADC1);

            memset(szOledData, '\0', 16 - OLED_OFFS_DATA_VALUE + sizeof('\0'));
            snprintf(szOledData, sizeof(szOledData), "%.1f V", pSensorDataRec_p->m_flCarBattLevel);
            OledPrintDataValue(szOledData, sizeof(szOledData), OLED_OFFS_DATA_VALUE, OLED_LINE_CAR_BATT);
        }
        pPrevSensorDataRec_p->m_ui16AdcValCarBattLevel = pPrevSensorDataRec_p->m_ui16AdcValCarBattLevel;
        pPrevSensorDataRec_p->m_flCarBattLevel         = pSensorDataRec_p->m_flCarBattLevel;
    }

    return;

}



//---------------------------------------------------------------------------
//  OLED: Print Process Data
//---------------------------------------------------------------------------

void  OledPrintDataValue (char* pszDataValue_p, int iDataValueBuffSize_p, uint8_t ui8PosX_p, uint8_t ui8PosY_p)
{

int  iIdx;


    // fill-up data value buffer with spaces until end of OLED line to
    // clear (overwrite) obsolete values from previous cycle
    for (iIdx=0; iIdx<(iDataValueBuffSize_p-1); iIdx++)
    {
        if (pszDataValue_p[iIdx] == '\0')
        {
            pszDataValue_p[iIdx] = ' ';
        }
    }
    pszDataValue_p[iIdx] = '\0';

    // print new data values on OLED
    Oled_U8x8_g.drawString(ui8PosX_p, ui8PosY_p, pszDataValue_p);

    return;

}



//---------------------------------------------------------------------------
//  Process DHT Sensor (Temperature/Humidity)
//---------------------------------------------------------------------------

int  GetDhtSensorData (float* pflTemperature_p, float* pflHumidity_p)
{

float  flTemperature;
float  flHumidity;
int    iRes;


    iRes = 0;

    flTemperature = DhtSensor_g.readTemperature(false);                 // false = Temp in Celsius degrees, true = Temp in Fahrenheit degrees
    flHumidity    = DhtSensor_g.readHumidity();

    // check if values read from DHT22 sensor are valid
    if ( !isnan(flTemperature) && !isnan(flHumidity) )
    {
        // return valid DHT22 sensor values
        *pflTemperature_p = flTemperature;
        *pflHumidity_p    = flHumidity;
    }
    else
    {
        *pflTemperature_p = 0;
        *pflHumidity_p    = 0;
        iRes = -1;
    }

    return (iRes);

}



//---------------------------------------------------------------------------
//  Process SEN-HC-SR501 Sensor (IR Motion Sensor)
//---------------------------------------------------------------------------

int  GetSenHcSr501SensorData (bool* pfMotionActive_p, uint16_t* pui16MotionActiveTime_p)
{

bool      fMovementActive;
uint32_t  ui32CurrTick;
uint16_t  ui16MotionActiveTime;
int       iRes;


    iRes = 0;

    fMovementActive = digitalRead(PIN_SEN_HC_SR501);

    // movement detected?
    if ( fMovementActive )
    {
        // new movement detected?
        if (ui32StartTickMotionSegm_g == 0)
        {
            // start new movement segment
            ui32StartTickMotionSegm_g = millis();
            ui16MotionActiveTime = 0;
        }
        else
        {
            // continue a running movement segment
            ui32CurrTick = millis();
            ui16MotionActiveTime = (ui32CurrTick - ui32StartTickMotionSegm_g) / 1000;
        }
    }
    else
    {
        ui32StartTickMotionSegm_g = 0;
        ui16MotionActiveTime = 0;
    }

    *pfMotionActive_p = fMovementActive;
    *pui16MotionActiveTime_p = ui16MotionActiveTime;

    return (iRes);

}



//---------------------------------------------------------------------------
//  Process Light Sensor (AI0 @ ADS1115)
//---------------------------------------------------------------------------

int  GetAds1115LightSensorData (uint16_t* pui16AdcVal_p, uint8_t* pui8LightLevel_p)
{

int16_t  i16AdcVal;
uint     uiLightLevel;
int      iRes;


    iRes = 0;

    i16AdcVal = Ads1115_g.readADC_SingleEnded(ADC_CH_LIGHT_SENSOR);
    if (i16AdcVal < 0)
    {
        i16AdcVal = 0;
    }

    // an ADC output value of 16384 corresponds to an input voltage of 3V,
    // which in turn corresponds to an ambient light level of 100%
    uiLightLevel = ((uint)i16AdcVal * 100) / 16384;     // get ambient light level in [%]

    *pui16AdcVal_p = (uint16_t)i16AdcVal;
    *pui8LightLevel_p = (uint8_t)uiLightLevel;

    return (iRes);

}



//---------------------------------------------------------------------------
//  Process Car Battery Level (AI1 @ ADS1115)
//---------------------------------------------------------------------------

int  GetAds1115CarBattAinData (uint16_t* pui16AdcVal_p, float* pflCarBattLevel_p)
{

int16_t  i16AdcVal;
float    flVoltage;
float    flCarBattLevel;
int      iRes;


    iRes = 0;

    i16AdcVal = Ads1115_g.readADC_SingleEnded(ADC_CH_CAR_BATT_AIN);
    if (i16AdcVal < 0)
    {
        i16AdcVal = 0;
        iRes = -1;
    }
    flVoltage = (i16AdcVal * ADS1115_DIGIT)/1000;                       // convert ADC digits into native voltage level in [V]

    flCarBattLevel = flVoltage * ADC_CAR_BATT_VOLTAGE_DIVIDER;          // normalize voltage level to real input value (Input Voltage Divider = 1:4.78 -> FullScale = 3.3V * 4.78 = 15.78V)

    // limit float value to 1 decimal place
    flCarBattLevel = flCarBattLevel + 0.05;
    flCarBattLevel = (float) ((int)(flCarBattLevel*10));
    flCarBattLevel = flCarBattLevel/10;

    // Is CarBatt Level in valid range?
    if (flCarBattLevel > 1.0)
    {
        // compensate voltage drop caused by forward voltage across the reverse polarity protection diode D2
        flCarBattLevel += ADC_CAR_BATT_RPD_FORWARD_VOLTAGE;
    }
    else
    {
        // reset CarBatt Level if no Batt connected (invalid values)
        flCarBattLevel = 0.0;
        iRes = -1;
    }

    *pui16AdcVal_p = (uint16_t)i16AdcVal;
    *pflCarBattLevel_p = flCarBattLevel;

    return (iRes);

}





//=========================================================================//
//                                                                         //
//          P R I V A T E   G E N E R I C   F U N C T I O N S              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Get System Uptime
//---------------------------------------------------------------------------

String  GetSysUptime (uint32_t* pui32Uptime_p)
{

const uint32_t  MILLISECONDS_PER_DAY    = 86400000;
const uint32_t  MILLISECONDS_PER_HOURS  = 3600000;
const uint32_t  MILLISECONDS_PER_MINUTE = 60000;
const uint32_t  MILLISECONDS_PER_SECOND = 1000;


char      szTextBuff[64];
uint32_t  ui32Uptime;
uint32_t  ui32Days;
uint32_t  ui32Hours;
uint32_t  ui32Minutes;
uint32_t  ui32Seconds;
String    strUptime;


    ui32Uptime = millis();

    ui32Days = ui32Uptime / MILLISECONDS_PER_DAY;
    ui32Uptime = ui32Uptime - (ui32Days * MILLISECONDS_PER_DAY);

    ui32Hours = ui32Uptime / MILLISECONDS_PER_HOURS;
    ui32Uptime = ui32Uptime - (ui32Hours * MILLISECONDS_PER_HOURS);

    ui32Minutes = ui32Uptime / MILLISECONDS_PER_MINUTE;
    ui32Uptime = ui32Uptime - (ui32Minutes * MILLISECONDS_PER_MINUTE);

    ui32Seconds = ui32Uptime / MILLISECONDS_PER_SECOND;


    snprintf(szTextBuff, sizeof(szTextBuff), "%ud/%02u:%02u:%02u", (uint)ui32Days, (uint)ui32Hours, (uint)ui32Minutes, (uint)ui32Seconds);
    strUptime = String(szTextBuff);

    *pui32Uptime_p = ui32Uptime;
    return (strUptime);

}





//=========================================================================//
//                                                                         //
//          D E B U G   F U N C T I O N S                                  //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  DEBUG: Dump Buffer
//---------------------------------------------------------------------------

#ifdef DEBUG_DUMP_BUFFER

void  DebugDumpBuffer (String strBuffer_p)
{

int            iBufferLen = strBuffer_p.length();
unsigned char  abDataBuff[iBufferLen];


    strBuffer_p.getBytes(abDataBuff, iBufferLen);
    DebugDumpBuffer(abDataBuff, strBuffer_p.length());

    return;

}

//---------------------------------------------------------------------------

void  DebugDumpBuffer (const void* pabDataBuff_p, unsigned int uiDataBuffLen_p)
{

#define COLUMNS_PER_LINE    16

const unsigned char*  pabBuffData;
unsigned int          uiBuffSize;
char                  szLineBuff[128];
unsigned char         bData;
int                   nRow;
int                   nCol;


    // get pointer to buffer and length of buffer
    pabBuffData = (const unsigned char*)pabDataBuff_p;
    uiBuffSize  = (unsigned int)uiDataBuffLen_p;


    // dump buffer contents
    for (nRow=0; ; nRow++)
    {
        sprintf(szLineBuff, "\n%04lX:   ", (unsigned long)(nRow*COLUMNS_PER_LINE));
        Serial.print(szLineBuff);

        for (nCol=0; nCol<COLUMNS_PER_LINE; nCol++)
        {
            if ((unsigned int)nCol < uiBuffSize)
            {
                sprintf(szLineBuff, "%02X ", (unsigned int)*(pabBuffData+nCol));
                Serial.print(szLineBuff);
            }
            else
            {
                Serial.print("   ");
            }
        }

        Serial.print(" ");

        for (nCol=0; nCol<COLUMNS_PER_LINE; nCol++)
        {
            bData = *pabBuffData++;
            if ((unsigned int)nCol < uiBuffSize)
            {
                if ((bData >= 0x20) && (bData < 0x7F))
                {
                    sprintf(szLineBuff, "%c", bData);
                    Serial.print(szLineBuff);
                }
                else
                {
                    Serial.print(".");
                }
            }
            else
            {
                Serial.print(" ");
            }
        }

        if (uiBuffSize > COLUMNS_PER_LINE)
        {
            uiBuffSize -= COLUMNS_PER_LINE;
        }
        else
        {
            break;
        }

        delay(50);          // give serial interface time to flush data
    }

    Serial.print("\n");

    return;

}

#endif




// EOF
