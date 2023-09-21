# FLUXO.INO 

## Arduino Water Flow Watchdog 

*(protection against water leaks)*

 - HAL Water Flowmeter Sensor  + Solenoid Flow Valve + Home Assistant (MQTT)
 - Optimized for Arduino Nano R3 (ATmega328P) and Ethernet module ECN28J60

### Source code:
  - https://github.com/fortalbrz/fluxoino/
  - [arduino nano sketch](https://github.com/fortalbrz/fluxoino/blob/main/fluxoino/fluxoino.ino)
  - [wiring testing sketch](https://github.com/fortalbrz/fluxoino/blob/main/fluxoino_wiring_test/fluxoino_wiring_test.ino)
  

in order to use serial communication on this sketch (debug with "Serial Monitor") 
sets the macro "DEBUG_MODE true".

**NOTICE**: that this sketch should be pushed into an Arduino board (Board "*Arduino Nano*") using the 
ATmega328P (Processor: "*ATmega328P (Old Bootloader)*").


#### Drivers (CH340g) for Arduino:
- [CH340g USB/Serial driver](https://bit.ly/44WdzVF) (windows 11 compatible driver)  
- driver install instructions ([pt-BR](https://bit.ly/3ZqIqc0))

###  Materials:
  - Arduino Nano R3 (ATmega328P)
  - Ethernet LAN Network Module (ENC28J60)
  - water flow sensor G 1/2"
  - solenoid valve 3/4" 12v (normally opened)
  - power supply 12vdc (2A)
  - 1 x rotary potentiometer 10k Ohm (Linear)
  - 1 x N-channel MOSFET 60V 30A (TNMOSFETFQP)
  - 1 x voltage regulator 3.3v (LD11173v3)
  - 1 x diode rectifier 1A 50V
  - 2 x 10K Ohm Resistor
  - 1 x electrolytic decoupling capacitor 10uF/25V
  - 1 x capacitor ceramic 100nF

###  Circuit Wiring Instructions:
   - [circuito.io - step by step](https://www.circuito.io/static/reply/index.html?solutionId=65010bbd91d445002e8974a5&solutionPath=storage.circuito.io)
   
![wiring](https://github.com/fortalbrz/fluxoino/blob/main/fluxoino_wiring.png?raw=true)


#### Wiring
   - diode 1A 50V (positive) --> power supply 12vdc (positive/Vcc)
   - Arduino Nano pin29 (GND) --> power supply 12vdc (negative/Gnd)
   - Arduino Nano pin30 (VIN) --> diode 1A 50V (negative)
   - solenoid valve coil 1 --> diode 1A 50V (positive)
   - solenoid valve coil 2 --> diode 1A 50V (negative)
   - diode 1A 50V (positive) --> TNMOSFETFQP Drain (center)
   - water flow sensor VCC (red/center) --> 5v (Arduino Nano Pin29)
   - water flow sensor GND (black/left) --> Gnd (Arduino Nano Pin29)
   - water flow sensor SIG (yellow/right) --> 10K Ohm Resistor A (terminal 1)
   - 10K Ohm Resistor A (terminal 1) --> Arduino Nano pin5 (D2)
   - 10K Ohm Resistor A (terminal 2) --> 5v (Arduino Nano Pin29)
   - TNMOSFETFQP Gate (left) --> 10K Ohm Resistor B (terminal 1)
   - TNMOSFETFQP Gate (left) --> Arduino Nano pin7 (D4)
   - TNMOSFETFQP Source (right) --> Gnd (Arduino Nano Pin29)
   - voltage regulator LD11173v3 Vin (right) --> 5v (Arduino Nano Pin29)
   - voltage regulator LD11173v3 Gnd (left) --> Gnd (Arduino Nano Pin29)
   - voltage regulator LD11173v3 Vout (center) --> 10uF electrolytic capacitor (positive)
   - voltage regulator LD11173v3 Vout (center) --> ethernet module ENC28J60 Pin9 (VCC)
   - 10uF electrolytic capacitor (positive) --> Gnd (Arduino Nano Pin29)
   - 100nF ceramic capacitor --> between Vcc and Gnd
   - LCD Pin2 (VDD) --> 5v (Arduino Pin27)
   - LCD Pin15 (A) --> 5v
   - LCD Pin1 (VSS) --> Gnd (Arduino Pin29)
   - LCD Pin5 (RW) --> Gnd
   - LCD Pin16 (K) --> Gnd
   - LCD Pin4 (RS) --> Arduino Nano Pin19 (A0)
   - LCD Pin6 (E) --> Arduino Nano Pin12 (D9)
   - LCD Pin11 (D4) --> Arduino Nano Pin8 (D5)
   - LCD Pin12 (D5) --> Arduino Nano Pin9 (D6)
   - LCD Pin13 (D6) --> Arduino Nano Pin10 (D7)
   - LCD Pin14 (D7) --> Arduino Nano Pin11 (D8)
   - LCD Pin3 (VO) --> 10k ohm potentiometer middle pin (the other two pins go to 5v and Gnd)
   - ethernet module ENC28J60 Pin2 (INT) --> Arduino Nano Pin6 (D3)
   - ethernet module ENC28J60 Pin4 (SO) --> Arduino Nano Pin15 (D12)
   - ethernet module ENC28J60 Pin5 (S1) --> Arduino Nano Pin14 (D11)
   - ethernet module ENC28J60 Pin6 (SCK) --> Arduino Nano Pin16 (D13)
   - ethernet module ENC28J60 Pin7 (CS) --> Arduino Nano Pin13 (D10)
   - ethernet module ENC28J60 Pin10 (GND) --> Gnd (Arduino Nano Pin29)


*[Jorge Albuquerque](mailto:jorgealbuquerque@gmail.com) (2022)*

#### References:
- [HAL flowmeter sensor](http://www.nadielcomercio.com.br/blog/2015/10/14/sensor-de-fluxo-de-agua/)
- [solenoid valve circuit connection](http://www.nadielcomercio.com.br/blog/2015/09/10/controlando-uma-valvula-solenoide-de-um-arduino/)
- [lcd display](http://labdegaragem.com/profiles/blogs/tutorial-lcd-com-arduino)
