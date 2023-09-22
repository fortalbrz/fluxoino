#include <Ethernet.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <ArduinoHA.h>
//---------------------------------------------------------------------------------------------------------------------
//
//
//   FLUXOINO -  Arduino Water Flow Controller Project (protection against water leaks)
//   - HAL Water Flowmeter Sensor  + Solenoid Flow Valve + Home Assistant (MQTT)
//   - Optimized for Arduino Nano R3 (ATmega328P) and Ethernet module ECN28J60
//
//   I made this project after have some leakages and be caught by a very expensive water bill...
//   The goal is not only creates a Arduino water flow sensor, but a small robot that can 
//   close the water flow autonomously (if required) case detect water leakages.    
//
//   Features:
//   - home assistant water flow sensor
//   - home assistant water flow switch (solenoid water valve) (optional, see configuration flags)
//   - home assistant leakage probability sensor 
//   - autonomously closes the water flow case detect water leakages. i.e. "leakage watchdog" (optional, see configuration flags)
//   - display water flow information on 16 x 2 LCD (optional, see configuration flags)
//
//   Source code:
//   - https://github.com/fortalbrz/fluxoino
//     (adds library "home-assistant-integration" by David Chyrzynsky at library manager)
//
//   Remarks:
//   - Arduino Nano with the CH340g USB/serial requires the Windows driver (next section) 
///  - this sketch should be pushed with Arduino IDE using configuration: Board: "Arduino Nano", Processor: "ATmega328P (Old Bootloader)".
//   - sets the macro "DEBUG_MODE true" in order to use serial communication for debugging (i.e., use "Serial Monitor", (see configuration flags))  
//
//    Drivers (CH340g) for both Arduino Nano and ESP-01:
//    - CH340g USB/Serial driver (windows 11 compatible driver): https://bit.ly/44WdzVF 
//    - driver install instructions (pt-BR): https://bit.ly/3ZqIqc0
//
//   Configuration flags
//
//   Use these configurations flags to set fluxoino behaviour (as you wish). You can avoid dispensable hardware 
//   and features to fit your needs...
//
//   USE_LCD_DISPLAY                    enables/disables LCD display (disable it if not in use the LCD display) (default: true)
//   USE_HOME_ASSISTANT                 enables/disables Home Assistant integration (MQTT) (disable it if not in use the LAN network module) (default: true)
//   USE_WALTER_FLOW_VALVE              enables/disables water flow valve (disable it if not in use the solenoid valve)
//   CAN_CLOSE_WATER_FLOW false         enables/disables autonomous water valve closing (relaying on "LEAKAGE_THRESHOLD")
//   MQTT_BROKER_ADDRESS                broker IP address (requires USE_HOME_ASSISTANT true)
//   LEAKAGE_THRESHOLD                  leakage detection sensitivity - threshold as percentage of the maximum theoretical flow (in range: 0 to 1, default: 0.9)
//   LEAKAGE_INTEGRATION_WINDOW_SECONDS sample window for flow volume integration (leakage volume estimation parameter) (default 600 = 10 min)
//   LAMBDA_EWMA                        lambda EWMA (moving average weight of water flow measurement) (in range: 0 to 1, default: 0.98)
//   PULSES_PER_LITER                   pulses per liter (from HAL flowmeter sensor datasheet) (default 450))
//   DEBUG_MODE false                   true for serial debug (home assistant disabled), false for production (home assistant enabled)
//   
//   Remark: home assistant can be disabled and therefore fluxoino will only work as standalone flowmeter (LCD display) and/or solenoid valve valve (leakage watchdog)
//
//   Materials:
//   - Arduino Nano R3 (ATmega328P)
//   - Ethernet LAN Network Module (ENC28J60) (optional, only with home assistant, see configuration flags)
//   - water flow sensor G 1/2"
//   - solenoid valve 3/4" 12v (normaly opened) (optional, see configuration flags)
//   - power supply 12vdc (2A)
//   - 1 x rotary potentiometer 10k Ohm (Linear) (optional, only with LCD display, see configuration flags)
//   - 1 x N-channel MOSFET 60V 30A (TNMOSFETFQP) (optional, only with solenoid valve, see configuration flags)
//   - 1 x voltage regulator 3.3v (LD11173v3) (optional, only with solenoid valve, see configuration flags)
//   - 1 x diode rectifier 1A 50V (optional, only with solenoid valve, see configuration flags)
//   - 2 x 10K Ohm Resistor (optional, only with solenoid valve, see configuration flags)
//   - 1 x electrolytic decoupling capacitor 10uF/25V (optional, only with solenoid valve, see configuration flags)
//   - 1 x capacitor ceramic 100nF (optional, only with solenoid valve, see configuration flags)
//
//   Circuit Wiring Instruction (step by step):
//   -  https://www.circuito.io/static/reply/index.html?solutionId=65010bbd91d445002e8974a5&solutionPath=storage.circuito.io
//
//      - diode 1A 50V (positive) --> power supply 12vdc (positive/Vcc)
//      - Arduino Nano pin29 (GND) --> power supply 12vdc (negative/Gnd)
//      - Arduino Nano pin30 (VIN) --> diode 1A 50V (negative)
//      - solenoid valve coil 1 --> diode 1A 50V (positive)
//      - solenoid valve coil 2 --> diode 1A 50V (negative)
//      - diode 1A 50V (positive) --> TNMOSFETFQP Drain (center)
//      - water flow sensor VCC (red/center) --> 5v (Arduino Nano Pin29)
//      - water flow sensor GND (black/left) --> Gnd (Arduino Nano Pin29)
//      - water flow sensor SIG (yellow/right) --> 10K Ohm Resistor A (terminal 1)
//      - 10K Ohm Resistor A (terminal 1) --> Arduino Nano pin5 (D2)
//      - 10K Ohm Resistor A (terminal 2) --> 5v (Arduino Nano Pin29)
//      - TNMOSFETFQP Gate (left) --> 10K Ohm Resistor B (terminal 1)
//      - TNMOSFETFQP Gate (left) --> Arduino Nano pin7 (D4)
//      - TNMOSFETFQP Source (right) --> Gnd (Arduino Nano Pin29)
//      - voltage regulator LD11173v3 Vin (right) --> 5v (Arduino Nano Pin29)
//      - voltage regulator LD11173v3 Gnd (left) --> Gnd (Arduino Nano Pin29)
//      - voltage regulator LD11173v3 Vout (center) --> 10uF electrolytic capacitor (positive)
//      - voltage regulator LD11173v3 Vout (center) --> ethernet module ENC28J60 Pin9 (VCC)
//      - 10uF electrolytic capacitor (positive) --> Gnd (Arduino Nano Pin29)
//      - 100nF ceramic capacitor --> between Vcc and Gnd
//      - LCD Pin2 (VDD) --> 5v (Arduino Pin27)
//      - LCD Pin15 (A) --> 5v
//      - LCD Pin1 (VSS) --> Gnd (Arduino Pin29)
//      - LCD Pin5 (RW) --> Gnd
//      - LCD Pin16 (K) --> Gnd
//      - LCD Pin4 (RS) --> Arduino Nano Pin19 (A0)
//      - LCD Pin6 (E) --> Arduino Nano Pin12 (D9)
//      - LCD Pin11 (D4) --> Arduino Nano Pin8 (D5)
//      - LCD Pin12 (D5) --> Arduino Nano Pin9 (D6)
//      - LCD Pin13 (D6) --> Arduino Nano Pin10 (D7)
//      - LCD Pin14 (D7) --> Arduino Nano Pin11 (D8)
//      - LCD Pin3 (VO) --> 10k ohm potentiometer middle pin (the other two pins go to 5v and Gnd)
//      - ethernet module ENC28J60 Pin2 (INT) --> Arduino Nano Pin6 (D3)
//      - ethernet module ENC28J60 Pin4 (SO) --> Arduino Nano Pin15 (D12)
//      - ethernet module ENC28J60 Pin5 (S1) --> Arduino Nano Pin14 (D11)
//      - ethernet module ENC28J60 Pin6 (SCK) --> Arduino Nano Pin16 (D13)
//      - ethernet module ENC28J60 Pin7 (CS) --> Arduino Nano Pin13 (D10)
//      - ethernet module ENC28J60 Pin10 (GND) --> Gnd (Arduino Nano Pin29)
//
//
//  Futhermore:
// 
//  Exponential weighted moving average (EWMA) flow estimation:
//
//    flowEWMA[t] = lambda * flow[t] + (1 - lambda) * flowEWMA[t-1]
// 
//  where:
//
//    flow[t] =        pulses_in_1_sec * 60 / PULSES_PER_LITER          [liters/min]
//
//    flow[t] = 0.06 * pulses_in_1_sec * 60 / PULSES_PER_LITER          [m3/h]
//
//
//   References:
//   - HAL flowmeter sensor:
//     - http://www.nadielcomercio.com.br/blog/2015/10/14/sensor-de-fluxo-de-agua/
//     - https://www.youtube.com/watch?v=TEp39b4EguQ
//   - solenoid valve circuit connection:
//     - http://www.nadielcomercio.com.br/blog/2015/09/10/controlando-uma-valvula-solenoide-de-um-arduino/
//   - lcd display:
//     - http://labdegaragem.com/profiles/blogs/tutorial-lcd-com-arduino
//
//
//   Jorge Albuquerque (2022) - jorgealbuquerque@gmail.com
//
//---------------------------------------------------------------------------------------------------------------------
//
// setup
//
#define DEBUG_MODE false            // true to production (prod), false for serial debug (home assistant disabled)
#define USE_LCD_DISPLAY true        // enables/disables LCD display (disable it if not in wish to use the LCD display)
#define USE_HOME_ASSISTANT true     // enables/disables Home Assistant integration (MQTT) (disable it if not not wish to use home assistant integration)
#define USE_WALTER_FLOW_VALVE true  // enables/disables water flow valve (disable it if not wish to use the solenoid valve)
#define CAN_CLOSE_WATER_FLOW true   // enables/disables autonomus water valve closing (relaying on "leakage threshold")
//
// pins definitions (Arduino Nano)
//
#define ETHERNETMODULE_PIN_CS 10  // Ethernet ECN28J60 (CS)
#define ETHERNETMODULE_PIN_INT 3  // Ethernet ECN28J60 (INT)
#define LCD_PIN_RS A0             // LCD (RS)
#define LCD_PIN_E 9               // LCD (E)
#define LCD_PIN_DB4 5             // LCD (D4)
#define LCD_PIN_DB5 6             // LCD (D4)
#define LCD_PIN_DB6 7             // LCD (D6)
#define LCD_PIN_DB7 8             // LCD (D7)
#define SOLENOIDVALVE_PIN_COIL 4  // pin for the solenoid power circuit (12v) or relay (110v)
#define WATERFLOW_5V_PIN_SIG 2    // pin for connecting the flow sensor PWM (on Arduino Nano, only pins 2 and 3)
//
// MQTT parameters
//
#define MQTT_BROKER_ADDRESS      IPAddress(192, 168, 68, 93)  // MQTT broker IP address
//
// flowmeter parameters
//
#define PULSES_PER_LITER 450                   // pulses per liter (from HAL flowmeter sensor datasheet)
#define LAMBDA_EWMA 0.98                       // lambda EWMA (moving average weight of flow measurement)
#define LEAKAGE_THRESHOLD 0.9                  // leakage detection sensitivity - threshold as percentage of the maximum theoretical flow (in range 0-1)
#define LEAKAGE_INTEGRATION_WINDOW_SECONDS 600 // 10 minutes of sample window for flow volume integration (volume estimation parameter)
#define EEPROM_ADDRESS 0                       // EEPROM adress 
//
// LCD messages - internationalization (pt-BR) 
//
#define MSG_CONNECTED F("conectado")           // "connected"
#define MSG_DICONNECTED F("desconectado")      // "disconnected"
#define MSG_DISABLED F("desativado")           // "disabled"
#define MSG_FLOW F("Fluxo: ")                  // "Flow: "
#define MSG_LEAKAGE_DETECTED F("Vazamento!")   // "Leakage!"
#define MSG_NO_LEAKAGE F("Sem vazamentos")     // "No Leakage"
#define MSG_VALVE_OPENED F("Valvula Aberta")   // "Valve Opened"
#define MSG_VALVE_CLOSED F("Valvula Fechada")  // "Valve Closed"

//
// internal (do not change)
//
#define USE_NETWORK (USE_HOME_ASSISTANT == true && DEBUG_MODE == false) 
#define VALVE_OPEN 0x00
#define VALVE_CLOSED 0x01
#define VALVE_UNDEFINED 0xFF
//
// global variables (internal states)
//
bool _blink = true;
unsigned int _pulseSensorCounter = 0;  // flowmeter sensor pulse counter
float _flowEwma = 0;                   // moving average of flow estimate (L/min)
float _leakageIntegrator = 0;          // Flow integrator (volume estimator) for valve switch (on leakage threshold)
boolean _solenoidValveClosed = false;  // valve control status
unsigned int _loopCounter = 0;         // loop execution counter (intended to switch flow unit on LCD display)
//
// statics & dependencies
//
//
#if (USE_NETWORK == true)
byte _mac[] = { 0x00, 0x20, 0xFA, 0x6E, 0x34, 0x4A };
//
// fit sketch on Arduino Nano (memory optimization for strings)
//
const char str_device_name[] = "Fluxoino";     // home assistant device name
const char str_device_version[] = "1.0.0";     // home assistant device version
const char str_device_class[] = "water";       // home assistant device class
const char str_icon[] = "mdi:water";           // home assistant default icon
const char str_unit[] = "L/min";               // home assistant flow meter default unit
const char str_flow_sensor[] = "flow_sensor";  // home assistant flow sensor uid
const char str_flow_reset[] = "flow_reset";    // home assistant reset button uid
const char str_flow_switch[] = "flow_switch";  // home assistant flow switch uid
const char str_volume_sensor[] = "leakage_prob_sensor"; // home assistant leakage probability (integrated volume) uid
#endif

// Arduino Nano and other pin outs:
//--------------------------------------
// diode 1A 50V (positive) --> power supply 12vdc (positive/Vcc)
// Arduino Nano pin29 (GND) --> power supply 12vdc (negative/Gnd)
// Arduino Nano pin30 (VIN) --> diode 1A 50V (negative)
// solenoid valve coil 1 --> diode 1A 50V (positive)
// solenoid valve coil 2 --> diode 1A 50V (negative)
// diode 1A 50V (positive) --> TNMOSFETFQP Drain (center)
// water flow sensor VCC (red/center) --> 5v (Arduino Nano Pin29)
// water flow sensor GND (black/left) --> Gnd (Arduino Nano Pin29)
// water flow sensor SIG (yellow/right) --> 10K Ohm Resistor A (terminal 1)
// 10K Ohm Resistor A (terminal 1) --> Arduino Nano pin5 (D2)
// 10K Ohm Resistor A (terminal 2) --> 5v (Arduino Nano Pin29)
// TNMOSFETFQP Gate (left) --> 10K Ohm Resistor B (terminal 1)
// TNMOSFETFQP Gate (left) --> Arduino Nano pin7 (D4)
// TNMOSFETFQP Source (right) --> Gnd (Arduino Nano Pin29)
// voltage regulator LD11173v3 Vin (right) --> 5v (Arduino Nano Pin29)
// voltage regulator LD11173v3 Gnd (left) --> Gnd (Arduino Nano Pin29)
// voltage regulator LD11173v3 Vout (center) --> 10uF electrolytic capacitor (positive)
// voltage regulator LD11173v3 Vout (center) --> ethernet module ENC28J60 Pin9 (VCC)
// 10uF electrolytic capacitor (positive) --> Gnd (Arduino Nano Pin29)
// 100nF ceramic capacitor --> between Vcc and Gnd

#if (USE_LCD_DISPLAY == true)
  // LCD (16 x 2) pin outs:
  //--------------------------------------
  // Pin1 (VSS) --> Gnd (Arduino Nano Pin29)
  // Pin2 (VDD) --> 5v (Arduino Nano Pin27)
  // Pin3 (VO)  --> 10k ohm potentiometer middle pin (the other two pins go to 5v and Gnd)
  // Pin4 (RS)  --> Arduino Nano Pin19 (A0)
  // Pin5 (RW)  --> Gnd (Arduino Pin29)
  // Pin6 (E)   --> Arduino Nano Pin12 (D9)
  // Pin7 (D0)  --> no connection
  // Pin8 (D1)  --> no connection
  // Pin9 (D2)  --> no connection
  // Pin10 (D3) --> no connection
  // Pin11 (D4) --> Arduino Nano Pin8 (D5)
  // Pin12 (D5) --> Arduino Nano Pin9 (D6)
  // Pin13 (D6) --> Arduino Nano Pin10 (D7)
  // Pin14 (D7) --> Arduino Nano Pin11 (D8)
  // Pin15 (A)  --> 5v (Arduino Nano Pin27)
  // Pin16 (K)  --> Gnd (Arduino Nano Pin29)
  LiquidCrystal lcd(LCD_PIN_RS, LCD_PIN_E, LCD_PIN_DB4, LCD_PIN_DB5, LCD_PIN_DB6, LCD_PIN_DB7);
#endif

#if (USE_NETWORK == true)
  // Ethernet module (ECN28J60) pin outs:
  // Pin1 (CLK) --> no connection
  // Pin2 (INT) --> Arduino Nano Pin6 (D3)
  // Pin3 (WOL) --> no connection
  // Pin4 (SO) --> Arduino Nano Pin15 (D12)
  // Pin5 (S1) --> Arduino Nano Pin14 (D11)
  // Pin6 (SCK) --> Arduino Nano Pin16 (D13)
  // Pin7 (CS) --> Arduino Nano Pin13 (D10)
  // Pin8 (RST) --> no connection
  // Pin9 (VCC) --> 3.3v (Arduino Nano Pin17 - recommend to use a voltage regulator 3.3v: LD11173v3)
  // Pin10 (GND) --> Gnd (Arduino Nano Pin29)
  EthernetClient client;

  //
  // Home Assistant device (MQTT)
  //
  HADevice device(_mac, sizeof(_mac));
  HAMqtt mqtt(client, device);

  // MQTT water fluxmeter sensor
  HASensorNumber sensor(str_flow_sensor, HASensorNumber::PrecisionP1);
  
  // MQTT water leakage probability (volume integration) sensor
  HASensorNumber sensorLeak(str_volume_sensor, HASensorNumber::PrecisionP1);

  #if (USE_WALTER_FLOW_VALVE == true)
    // MQTT valve switch (opens/closes the water valve)
    HASwitch chkSwitch(str_flow_switch);
  #endif 

  // MQTT sensor reset button
  HAButton btnReset(str_flow_reset);
#endif

void setup() {
  //
  // initialization
  //
  pinMode(LED_BUILTIN, OUTPUT);

  // initializes solenoid water valve (normally opened)
  #if (USE_WALTER_FLOW_VALVE == true)
    pinMode(SOLENOIDVALVE_PIN_COIL, OUTPUT);  
    if (EEPROM.read(EEPROM_ADDRESS) == VALVE_CLOSED){
      // closes the valve if previously closed 
      // (keeps valve state after energy fail, etc)
      setValveState(true);
    }
  #endif
    
  //
  // initializes HAL water flowmeter sensor on pin 2 (interruption 0)
  //
  pinMode(WATERFLOW_5V_PIN_SIG, INPUT);
  digitalWrite(WATERFLOW_5V_PIN_SIG, HIGH);
  attachInterrupt(digitalPinToInterrupt(WATERFLOW_5V_PIN_SIG), onWaterFlowSensorRead, FALLING);

  #if (USE_NETWORK == true) 
    //
    // initializes ethernet (module ECN28J60)
    //
    Ethernet.begin(_mac);

    //
    // initializes MQTT device
    //
    device.setName(str_device_name);                // Fluxoino
    device.setSoftwareVersion(str_device_version);  // 1.0.0

    // initializes flowmeter sensor (MQTT)
    sensor.setName(str_flow_sensor);                // flowmeter sensor
    sensor.setIcon(str_icon);                       // mdi:water
    sensor.setDeviceClass(str_device_class);        // water
    sensor.setUnitOfMeasurement(str_unit);          // L/min

    // initializes leakage volume sensor (MQTT) - debuging
    sensorLeak.setName(str_volume_sensor);          // volume sensor [integrator]
    sensorLeak.setIcon(str_icon);                   // mdi:water
    sensorLeak.setDeviceClass(str_device_class);    // water

    // initializes sensor reset button callbacks (MQTT)
    btnReset.setName(str_flow_reset);               // reset sensor state
    btnReset.setIcon(str_icon);                     // mdi:water
    btnReset.onCommand(onButtonCommand);

    #if (USE_WALTER_FLOW_VALVE == true)
      // initializes water valve switch callbacks (MQTT)
      chkSwitch.setName(str_flow_switch);             // water valve switch
      chkSwitch.setIcon(str_icon);                    // mdi:water
      chkSwitch.onCommand(onSwitchCommand);
    #endif

    // starts MQTT
    mqtt.begin(MQTT_BROKER_ADDRESS);
  #endif
  
  #if (USE_LCD_DISPLAY == true)
    // initializes LCD displat (16 cols x 2 rows)
    lcd.begin(16, 2);
    lcd.noCursor();
  #endif

  #if (DEBUG_MODE == true)
    Serial.begin(9600);  // default baud rate (serial monitor)
    Serial.println(F("starting..."));    
  #endif
}


void loop() {
  //
  // main loop
  //
  #if (USE_NETWORK == true) 
    // keeps alive MQTT connection
    Ethernet.maintain();
    mqtt.loop();    
  #endif

  // build-in led blinking
  digitalWrite(LED_BUILTIN, (_blink ? HIGH: LOW));
  _blink = !_blink;

  //
  // reads flowmeter sensor (counts number of fluxmeter sensor pulses in 1 second)
  //
  _pulseSensorCounter = 0;
  sei();        // enables flowmeter sensor interruption
  delay(1000);  // waits one second
  cli();        // disables flowmeter sensor interruption
  
  //
  // HAL flowmeter sensor datasheet: 450 pulses per liter [450 pulses/L]
  //
  // therefore:
  //    flow = (pulses in 1 s)/450 [L/seg]
  //    flow = (pulses in 1 s)/7.5 [L/min]
  //    flow = 8 * (pulses in 1 s) [L/h]
  //    flow = 8 * (pulses in 1 s) / 1000 [m3/h]
  //
  // finally:
  //    flow [m3/h] = 0.06 * flow [L/min]
  //
  // Exponential weighted moving average (EWMA) flow estimation:  
  //    EWMA[t] = lambda * fluxo + (1 - lambda) * EWMA[t-1]
  //
  float flow = (float)_pulseSensorCounter * 60 / PULSES_PER_LITER; // instantaneous water flow [L/min] 
  _flowEwma = LAMBDA_EWMA * flow + (1 - LAMBDA_EWMA) * _flowEwma;  // moving average ("smoothed") flow estimation [L/min]
  
  #if (USE_NETWORK == true) 
    // updates MQTT flowmeter sensor [L/min]
    sensor.setAvailability(!sensor.isOnline());
    sensor.setValue(_flowEwma);
  #endif

  //
  // flow integrator (leakage volume estimator)
  //
  if (_flowEwma < 0.01) {
    // the water flow is negligible!
    _leakageIntegrator = 0;
  } else {
    // running volume integrator
    _leakageIntegrator -= _leakageIntegrator / LEAKAGE_INTEGRATION_WINDOW_SECONDS;
    _leakageIntegrator += (float)_pulseSensorCounter / LEAKAGE_INTEGRATION_WINDOW_SECONDS;
  }
  // leakage probability in range [0-100]
  float leakageProbability = 100 * _leakageIntegrator / (LEAKAGE_THRESHOLD * PULSES_PER_LITER); 

  #if (DEBUG_MODE == true)
    //
    // debug only
    //
    // shows all information:
    Serial.print(F("- pulses/s: "));
    Serial.print(_pulseSensorCounter);
    Serial.print(F(", flow: "));
    Serial.print(flow);    
    Serial.print(F(" [L/min], flowEwma: "));
    Serial.print(_flowEwma);
    Serial.print(F(" [L/min], flowEwma: "));
    Serial.print(0.06 * _flowEwma);
    Serial.print(F(" [m3/h], leakage: "));
    Serial.print(_leakageIntegrator);
    Serial.print(F(", prob: "));
    Serial.print(leakageProbability);
    Serial.print(F("%, valve: ")); 
    Serial.println((_solenoidValveClosed ? F("closed") : F("open")));
    Serial.flush();   
  #endif

  #if (USE_NETWORK == true) 
    // updates MQTT leakage volume sensor
    sensorLeak.setAvailability(!sensorLeak.isOnline());
    sensorLeak.setValue(_leakageIntegrator);
  #endif

  // closes the water valve (if leakage threshold is hitten)
  #if (USE_WALTER_FLOW_VALVE == true && CAN_CLOSE_WATER_FLOW == true)
    if (_leakageIntegrator > (LEAKAGE_THRESHOLD * PULSES_PER_LITER)) 
      // leakage detected: closes the water valve!
      setValveState(true);
  #endif

  #if (USE_LCD_DISPLAY == true)
    //
    // shows message on LCD display (if required)
    //  
    lcd.clear();
    if (_solenoidValveClosed) {
      //
      // water valve is closed!
      //
      lcd.print(F("HA "));
      #if (USE_NETWORK == true)       
        lcd.print((sensor.isOnline() ? MSG_CONNECTED : MSG_DICONNECTED));
      #else
        lcd.print(MSG_DISABLED);
      #endif

      lcd.setCursor(0, 1);
      lcd.print(MSG_VALVE_CLOSED);

    } else {
      //
      // shows 3 different displays (rotating)
      //
      _loopCounter = ++_loopCounter % 30;
      if (_loopCounter > 20) {        
        //
        // shows Home Assistant connection status (10 times)
        //
        lcd.print(F("HA "));
        #if (USE_NETWORK == true)       
          lcd.print((sensor.isOnline() ? MSG_CONNECTED : MSG_DICONNECTED));
          #else
          lcd.print(MSG_DISABLED);
        #endif

        // water valve is opened!
        lcd.setCursor(0, 1);        
        lcd.print(MSG_VALVE_OPENED);

      } else if (_loopCounter > 10) {
        //
        // shows estimated flow (moving averaged) as [m3/h] (10 times)
        //
        lcd.print(MSG_FLOW);
        lcd.print(0.06 * _flowEwma);
        lcd.print(F(" m3/h"));
        
        // shows leakage probability
        lcd.setCursor(0, 1);
        lcd.print(F("Prob: "));
        lcd.print(leakageProbability);
        lcd.print(F("%"));
      } else {
        //
        // shows estimated flow (moving averaged) as [L/min] (10 times)
        //
        lcd.print(MSG_FLOW);
        lcd.print(_flowEwma);
        lcd.print(F(" L/min"));
        
        // shows "leakage detected" /  "no leakage"
        lcd.setCursor(0, 1);
        if (_leakageIntegrator > (LEAKAGE_THRESHOLD * PULSES_PER_LITER))
          lcd.print(MSG_LEAKAGE_DETECTED);
        else
          lcd.print(MSG_NO_LEAKAGE);
      }
    }
  #endif
}


void onWaterFlowSensorRead() {
  //
  // HAL water flow sensor pulse handler: increment pulses counter
  //
  _pulseSensorCounter++;
}


#if (USE_WALTER_FLOW_VALVE == true)
  void onSwitchCommand(bool state, HASwitch* sender) {
    //
    // MQTT valve switch handler
    //  
    setValveState(state);
    sender->setState(state);
  }
#endif


void onButtonCommand(HAButton* sender) {
  //
  // MQTT reset button event handler
  //
  _flowEwma = 0;
  _leakageIntegrator = 0;
}


#if (USE_WALTER_FLOW_VALVE == true)
  void setValveState(boolean closed) {
    //
    // True to close the water valve, False to open it
    //
    _solenoidValveClosed = closed;
    // closes valve
    digitalWrite(SOLENOIDVALVE_PIN_COIL, (closed ? HIGH : LOW));
    // saves state on EEPROM (restart with the same state!)
    EEPROM.update(EEPROM_ADDRESS, (closed ? VALVE_CLOSED : VALVE_OPEN ));
  }
#endif
