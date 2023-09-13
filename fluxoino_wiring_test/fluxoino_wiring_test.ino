#include "Arduino.h"
#include <Ethernet.h>
#include <LiquidCrystal.h>
//---------------------------------------------------------------------------------------------------------------------
//
//
//   FLUXOINO -  Arduino Water Flow Controller Project (protection against water leaks)
//   - HAL Water Flowmeter Sensor  + Solenoid Flow Valve + Home Assistant (MQTT)
//   - Optimized for Arduino Nano R3 (ATmega328P) and Ethernet module ECN28J60
//
//   Source code:
//   - wiring testing:
//   - project:
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
#define WATERFLOW_PULSES_PER_LITER            450        // pulses per liter (from HAL flowmeter sensor datasheet) 
LiquidCrystal lcd(LCD_PIN_RS, LCD_PIN_E, LCD_PIN_DB4, LCD_PIN_DB5, LCD_PIN_DB6, LCD_PIN_DB7);

byte _mac[] = {0x00, 0x20, 0xFA, 0x6E, 0x34, 0x4A};
IPAddress ip(192, 168, 68, 200);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

// define vars for testing menu
const int timeout = 10000;       //define timeout of 10 sec
unsigned int pulseSensorCounter = 0;
char menuOption = 0;
long time0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() {
  pinMode(WATERFLOW_5V_PIN_SIG, INPUT);
  digitalWrite(WATERFLOW_5V_PIN_SIG, HIGH);
  attachInterrupt(digitalPinToInterrupt(WATERFLOW_5V_PIN_SIG), onWaterFlowSensorRead, FALLING);

  // Setup Serial which is useful for debugging
  // Use the Serial Monitor to view printed messages
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB
  Serial.println("start");

  // set up the LCD's number of columns and rows
  lcd.begin(16, 2);
  menuOption = menu();
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop()
{
  if (menuOption == '1') {
    //The Ethernet LAN Network Module (ENC28J60) - Test Code
    Ethernet.begin(_mac, ip);

    server.begin();
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());

  } else if (menuOption == '2') {
    // LCD 16x2 - Test Code
    // Print a message to the LCD.
    lcd.setCursor(0, 0);
    lcd.print("Fluxoino LCD test !");
    // Turn off the display:
    lcd.noDisplay();
    delay(500);
    // Turn on the display:
    lcd.display();
    delay(500);

  } else if (menuOption == '3') {
    // 12V Solenoid Valve - 3/4" - Test Code
    // The solenoid valve will turn on and off for 500ms (0.5 sec)
    Serial.println(F("\nopening valve"));
    digitalWrite(SOLENOIDVALVE_PIN_COIL1, HIGH);   // 1. turns on
    delay(500);                                    // 2. waits 0.5s
    Serial.println(F("\nclosing valve"));
    digitalWrite(SOLENOIDVALVE_PIN_COIL1, LOW);    // 3. turns off
    delay(500);                                    // 4. waits 0.5s

  } else if (menuOption == '4') {
    // The Water Flow Sensor G1/2" - Test Code
    for (int i = 0; i < 10; i++) {
      pulseSensorCounter = 0;
      sei();         // enables flowmeter sensor interruption
      delay(1000);   // waits one second
      cli();         // disables flowmeter sensor interruption
      float flow = (float)pulseSensorCounter * 60 / WATERFLOW_PULSES_PER_LITER;
      Serial.print(F("\nwater flow: "));
      Serial.print(flow);
      Serial.print(F(", sensor pulses: "));
      Serial.println(pulseSensorCounter);
    }
  }

  if (millis() - time0 > timeout)
    menuOption = menu();
}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{
  Serial.println(F("\nWhich component would you like to test?"));
  Serial.println(F("(1) Ethernet LAN Network Module - ENC28J60"));
  Serial.println(F("(2) LCD 16x2"));
  Serial.println(F("(3) 12V Solenoid Valve - 3/4''"));
  Serial.println(F("(4) Water Flow Sensor G1/2''"));
  Serial.println(F("(menu) send anything else or press on board reset button\n"));
  while (!Serial.available());

  // Read data from serial monitor if received
  while (Serial.available())
  {
    char c = Serial.read();
    if (isAlphaNumeric(c))
    {
      if (c == '1')
        Serial.println(F("Testing Ethernet LAN Network Module - ENC28J60"));
      else if (c == '2')
        Serial.println(F("Testing LCD 16x2"));
      else if (c == '3')
        Serial.println(F("Testing 12V Solenoid Valve - 3/4''"));
      else if (c == '4')
        Serial.println(F("Testing Water Flow Sensor G1/2"));
      else
      {
        Serial.println(F("illegal input!"));
        return 0;
      }
      time0 = millis();
      return c;
    }
  }
}


void onWaterFlowSensorRead() {
  //
  // water flux HAL sensor pulse handler: increment pulses counter
  //
  pulseSensorCounter++;
}
