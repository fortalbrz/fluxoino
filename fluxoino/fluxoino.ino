#include <Ethernet.h>
#include <LiquidCrystal.h>
#include <ArduinoHA.h>
//---------------------------------------------------------------------------------------------------------------------
//
//
//   FLUXOINO -  Arduino Water Flow Controller Project (protection against water leaks)
//   - HAL Water Flowmeter Sensor  + Solenoid Flow Valve + Home Assistant (MQTT)
//   - Optimized for Arduino Nano R3 (ATmega328P) and Ethernet module ECN28J60
//
//   Source code:
//   - https://github.com/fortalbrz/fluxoino
//   - wiring testing: https://github.com/fortalbrz/fluxoino/blob/main/fluxoino_wiring_test/fluxoino_wiring_test.ino
//   - project: https://github.com/fortalbrz/fluxoino/blob/main/fluxoino/fluxoino.ino
//
//   Materials:
//   - Arduino Nano R3 (ATmega328P)
//   - Ethernet LAN Network Module (ENC28J60)
//   - water flow sensor G 1/2"
//   - solenoid valve 3/4" 12v (normaly opened)
//   - power supply 12vdc (2A)
//   - 1 x rotary potentiometer 10k Ohm (Linear)
//   - 1 x N-channel MOSFET 60V 30A (TNMOSFETFQP)
//   - 1 x voltage regulator 3.3v (LD11173v3)
//   - 1 x diode rectifier 1A 50V
//   - 2 x 10K Ohm Resistor
//   - 1 x electrolytic decoupling capacitor 10uF/25V
//   - 1 x capacitor ceramic 100nF
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
//   Jorge Albuquerque (2022) - jorgealbuquerque@gmail.com
//
//   references:
//   - HAL flowmeter sensor:
//     http://www.nadielcomercio.com.br/blog/2015/10/14/sensor-de-fluxo-de-agua/
//     https://www.youtube.com/watch?v=TEp39b4EguQ
//   - solenoid valve circuit connection:
//     http://www.nadielcomercio.com.br/blog/2015/09/10/controlando-uma-valvula-solenoide-de-um-arduino/
//   - lcd display:
//     http://labdegaragem.com/profiles/blogs/tutorial-lcd-com-arduino
//
//---------------------------------------------------------------------------------------------------------------------
//
// setup
//
#define _useLCD                                true       // enables/disables LCD display
#define _useHomeAssistant                      true       // enables/disables Home Assistant integration (MQTT)
#define _canCloseWaterFlow                     false      // enables/disables autonomus water valve closing (relaying on "leakage threshold")
//
// pins definitions (Arduino Nano)
//
#define ETHERNETMODULE_PIN_CS                  10         // Ethernet ECN28J60 (CS)
#define ETHERNETMODULE_PIN_INT                  3         // Ethernet ECN28J60 (INT)
#define LCD_PIN_RS                             A0         // LCD (RS)
#define LCD_PIN_E                               9         // LCD (E)
#define LCD_PIN_DB4                             5         // LCD (D4)
#define LCD_PIN_DB5                             6         // LCD (D4)
#define LCD_PIN_DB6                             7         // LCD (D6)
#define LCD_PIN_DB7                             8         // LCD (D7) 
#define SOLENOIDVALVE_PIN_COIL1                 4         // pin for the solenoid power circuit (12V) or relay
#define WATERFLOW_5V_PIN_SIG                    2         // pin for connecting the flow sensor PWM (on Arduino Nano, only pins 2 and 3)

//
// flowmeter parameters
//
#define _pulsesPerLiter                        450        // pulses per liter (from HAL flowmeter sensor datasheet) 
#define _LambdaEWMA                            0.94       // lambda EWMA (moving average weight of flow measurement)
#define _LeakageThreshold                      0.9        // 90% of the maximum theoretical flow (theoretical: 60 L/min - practical: 30 L/min - adjusted: ??)
#define _LeakageIntegrationWindowInSeconds     600        // 10 minutes of sample window for flow volume integration (volume estimation parameter)
// MQTT broker IP address
#define _BrokerAddress                         IPAddress(192, 168, 68, 93)

//
// fit sketch on Arduino Nano (memory optimization for strings)
//
const char str_device_name[] = "Fluxoino";
const char str_device_version[] =  "1.0.0";
const char str_device_class[] =  "water";
const char str_icon[] = "mdi:water";
const char str_unit[] = "L/min";
const char str_flow_sensor[] = "flow_sensor";
const char str_flow_reset[] = "flow_reset";
const char str_flow_switch[] = "flow_switch";
const char str_volume_sensor[] = "vol_sensor";

//
// global variables (internal states)
//
unsigned int pulseSensorCounter = 0;         // flowmeter sensor pulse counter
float flowEwma = 0;                          // moving average of flow estimate (L/min)
float leakageIntegrator = 0;                 // Flow integrator (volume estimator) for valve switch (on leakage threshold)
boolean solenoidValveClosed = false;         // valve control status
unsigned int loopCounter = 0;                // loop execution counter (intended to switch flow unit on LCD display)

//
// statics & dependencies
//
byte _mac[] = {0x00, 0x20, 0xFA, 0x6E, 0x34, 0x4A};

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
// Home Assistant (MQTT)
//
HADevice device(_mac, sizeof(_mac));
HAMqtt mqtt(client, device);

// MQTT water fluxmeter sensor
HASensorNumber sensor(str_flow_sensor, HASensorNumber::PrecisionP1);
// MQTT water leakage volume (integration) sensor
HASensorNumber sensorLeak(str_volume_sensor, HASensorNumber::PrecisionP1);
// MQTT valve switch (opens/closes the water valve)
HASwitch chkSwitch(str_flow_switch);
// MQTT sensor reset button
HAButton btnReset(str_flow_reset);


void setup() {
  // initializes solenoid water valve (normaly opened)
  pinMode(SOLENOIDVALVE_PIN_COIL1, OUTPUT);
  digitalWrite(SOLENOIDVALVE_PIN_COIL1, LOW); // ensures opened state

  // initializes HAL water flowmeter sensor on pin 2 (interruption 0)
  pinMode(WATERFLOW_5V_PIN_SIG, INPUT);
  digitalWrite(WATERFLOW_5V_PIN_SIG, HIGH);
  attachInterrupt(digitalPinToInterrupt(WATERFLOW_5V_PIN_SIG), onWaterFlowSensorRead, FALLING);

  if (_useHomeAssistant) {
    // initializes ethernet (module W5500)
    Ethernet.begin(_mac);

    // initializes MQTT device
    device.setName(str_device_name); // Fluxoino
    device.setSoftwareVersion(str_device_version); // 1.0.0

    // initializes flowmeter sensor (MQTT)
    sensor.setName(str_flow_sensor); // flowmeter sensor
    sensor.setIcon(str_icon); // mdi:water
    sensor.setDeviceClass(str_device_class); // water
    sensor.setUnitOfMeasurement(str_unit); // L/min

    // initializes leakage volume sensor (MQTT) - debuging
    sensorLeak.setName(str_volume_sensor); // volume sensor [integrator]
    sensorLeak.setIcon(str_icon); // mdi:water
    sensorLeak.setDeviceClass(str_device_class); // water

    // initializes sensor reset button callbacks (MQTT)
    btnReset.setName(str_flow_reset); // reset sensor state
    btnReset.setIcon(str_icon); // mdi:water
    btnReset.onCommand(onButtonCommand);

    // initializes water valve switch callbacks (MQTT)
    chkSwitch.setName(str_flow_switch); // water valve switch
    chkSwitch.setIcon(str_icon); // mdi:water
    chkSwitch.onCommand(onSwitchCommand);

    // starts MQTT
    mqtt.begin(_BrokerAddress);
  }

  // initializes LCD displat (16 cols x 2 rows)
  if (_useLCD) {
    lcd.begin(16, 2);
    lcd.noCursor();
  }
}


void loop() {
  if (_useHomeAssistant) {
    // keeps alive
    Ethernet.maintain();
    mqtt.loop();
  }

  //
  // reads flowmeter sensor (counts number of fluxmeter sensor pulses in 1 second)
  //
  pulseSensorCounter = 0;
  sei();         // enables flowmeter sensor interruption
  delay(1000);   // waits one second
  cli();         // disables flowmeter sensor interruption

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
  // moving average flow estimation (EWMA):
  //    EWMA[t] = lambda * fluxo + (1 - lambda) * EWMA[t-1]
  //
  flowEwma = _LambdaEWMA * (float)pulseSensorCounter * 60 / _pulsesPerLiter + (1 - _LambdaEWMA) * flowEwma;
  if (_useHomeAssistant) {
    // updates MQTT flowmeter sensor [L/min]
    sensor.setAvailability(!sensor.isOnline());
    sensor.setValue(flowEwma);
  }

  //
  // flow integrator (leakage volume estimator)
  //
  if (flowEwma < 0.01) {
    // negligible
    leakageIntegrator = 0;
  } else {
    // running integrator
    leakageIntegrator -= leakageIntegrator / _LeakageIntegrationWindowInSeconds;
    leakageIntegrator += (float)pulseSensorCounter / _LeakageIntegrationWindowInSeconds;
  }

  if (_useHomeAssistant) {
    // updates MQTT leakage volume sensor
    sensorLeak.setAvailability(!sensorLeak.isOnline());
    sensorLeak.setValue(leakageIntegrator);
  }

  // closes the water valve (if leakage threshold is hitten)
  if (_canCloseWaterFlow && leakageIntegrator > (_LeakageThreshold * _pulsesPerLiter)) {
    // leakage detected: closes the water valve!
    setValveState(true);
  }

  //
  // shows message on LCD display (if required)
  //
  if (_useLCD) {
    lcd.clear();
    if (solenoidValveClosed) {
      // water valve is closed!
      lcd.print(F("Fluxoino 1.0.0!"));
      lcd.setCursor(0, 1);
      lcd.print(F("Valvula Fechada!"));

    } else {
      // shows 3 different displays
      loopCounter = ++loopCounter % 25;
      if (loopCounter > 20) {
        // water valve is opened! (shows 5 times)
        lcd.print(F("Fluxoino by Jorge"));
        lcd.setCursor(0, 1);
        lcd.print(F("Valvula Aberta!"));
      } else if (loopCounter > 10) {
        // shows estimated flow (moving averaged) as [m3/h] (10 times)
        lcd.print(F("Fluxo: "));
        lcd.print(0.06 * flowEwma);
        lcd.print(F(" m3/h"));
        // shows leakage probability
        lcd.setCursor(0, 1);
        lcd.print(F("Prob: "));
        lcd.print(100 * leakageIntegrator / (_LeakageThreshold * _pulsesPerLiter));
        lcd.print(F("%"));
      } else {
        // shows estimated flow (moving averaged) as [L/min] (10 times)
        lcd.print(F("Fluxo: "));
        lcd.print(flowEwma);
        lcd.print(F(" L/min"));
        // shows Home Assistant connection status
        lcd.print(F("HA "));
        if (_useHomeAssistant)
          lcd.print((sensor.isOnline() ? F("conectado") : F("desconectado")));
        else
          lcd.print(F("desativado"));
      }
    }
  }
}


void onWaterFlowSensorRead() {
  //
  // water flux HAL sensor pulse handler: increment pulses counter
  //
  pulseSensorCounter++;
}


void onSwitchCommand(bool state, HASwitch* sender)
{
  //
  // MQTT valve switch handler
  //
  setValveState(state);
  sender->setState(state);
}


void onButtonCommand(HAButton* sender) {
  //
  // MQTT reset button event handler
  //
  flowEwma = 0;
  leakageIntegrator = 0;
}



void setValveState(boolean closed) {
  //
  // True to close the water valve, False to open it
  //
  solenoidValveClosed = closed;
  digitalWrite(SOLENOIDVALVE_PIN_COIL1, (closed ? HIGH : LOW));
}
