#include <LiquidCrystal.h>
#include <Ethernet.h>
#include <ArduinoHA.h>
//#include <ArduinoJson.h>
/*

   Projeto Controlador de Fluxo (protecao contra vazamentos)
   - Sensor Hal + Valvula solenoide

   ref sensor fluxo: http://www.nadielcomercio.com.br/blog/2015/10/14/sensor-de-fluxo-de-agua/ - https://www.youtube.com/watch?v=TEp39b4EguQ
   ref ligacao da valvula: http://www.nadielcomercio.com.br/blog/2015/09/10/controlando-uma-valvula-solenoide-de-um-arduino/
   ref lcd: http://labdegaragem.com/profiles/blogs/tutorial-lcd-com-arduino
   ref bluetooth (virtuino): https://www.youtube.com/watch?v=CYR_jigRkgk

*/


//
// setup
//
#define _useNetwork               true       // enable/disable network
#define _useLCD                   true       // enable/disable LCD
#define _canCloseWater            true       // enable/disable autonomus water closing

#define  N                        600        // 10 min de intervalo de contagem de volume
#define _LeakageThreshold         0.9        // 90% da vazao teorica maxima (teorico: 60 L/min - pratico: 30 L/min - ajustado: ??)
#define _LambdaEWMA               0.94       // lambda EWMA (media movel da medida de vazao)
#define _pulsesPerLiter           450        // pulsos por litro (datasheet do sensor de fluxo) 
#define _pinWaterFluxSensor       2          // pino para conexao do PWM do sensor de fluxo (no Nano, apenas pinos 2 e 3)
#define _pinBotao                 8          // pino para o push button do botao de reset
#define _poutSolenoidValve        10         // pino para o circuito de alimentacao da solenoide
#define _BrokerAddress            IPAddress(192,168,68,93)

//
// global data variables (internal states)
//
unsigned int pulseSensorCounter = 0;         // contador de pulsos
unsigned int loopCounter = 0;                // contador de segundos (alterna unidade do fluxo)
float ewma = 0;                              // media movel da estimativa de fluxo (L/min)
float leakageIntegrator = 0;                 // leakageIntegrator de vazao para limite da valvula
boolean solenoidValveClosed = false;         // estado do controle da valvula
boolean ledState = false;                    // estado do led built in


//
// statics
//
byte _mac[] = {0x00, 0x20, 0xFA, 0x6E, 0x34, 0x4A};

// configura os pinos do Arduino para se comunicar com o display LCD
// LCD pin outs
// Pin1 --> Gnd
// Pin2 --> 5v
// Pin3 --> 10k ohm potentiometer middle pin (the other two pins go to 5v and Gnd)
// Pin4 --> Arduino Pin12
// Pin5 --> Gnd
// Pin6 --> Arduino Pin11
// Pin7 --> no connection
// Pin8 --> no connection
// Pin9 --> no connection
// Pin10 --> no connection
// Pin11 --> Arduino Pin5
// Pin12 --> Arduino Pin4
// Pin13 --> Arduino Pin3
// Pin14 --> Arduino Pin2
// Pin15 --> 5v
// Pin16 --> Tactile button (other side of tack button goes to Gnd)
LiquidCrystal lcd(12, 11, 5, 4, 3, 6);


EthernetClient client;
HADevice device(_mac, sizeof(_mac));
HAMqtt mqtt(client, device);
HASensorNumber sensor("water_flux_sensor", HASensorNumber::PrecisionP1);
HASwitch chkSwitch("water_flux_switch");
HAButton btnReset("water_flux_reset");

void setup() {
  solenoidValveClosed = false;
  Serial.begin(9600);
  Serial.print("starting...");

  // led built in piscando
  pinMode(LED_BUILTIN, OUTPUT);

  if (_useNetwork) {
    // Serial.print("starting ethernet...");
    Ethernet.begin(_mac);

    // set device's details (optional)
    device.setName("Fluxoino");
    device.setSoftwareVersion("1.0.0");

    // configure sensor
    sensor.setIcon("mdi:hand-water");
    sensor.setName("Fluxo de Agua");
    sensor.setDeviceClass("water");
    sensor.setUnitOfMeasurement("L/min");

    // button press callbacks
    btnReset.setIcon("mdi:fire");
    btnReset.setName("Reset");
    btnReset.onCommand(onButtonCommand);

    // switch callbacks
    chkSwitch.setName("Valvula Agua Entrada");
    chkSwitch.setIcon("mdi:hand-water");
    chkSwitch.onCommand(onSwitchCommand);

    // Serial.print("starting MQTT...");
    mqtt.begin(_BrokerAddress);
  }

  // saida solenoide (default: fechada)
  pinMode(_poutSolenoidValve, OUTPUT);
  digitalWrite(_poutSolenoidValve, HIGH); // abre

  // leitura do fluxo no pino 2 (interrupcao 0) - fluxometro
  pinMode(_pinWaterFluxSensor, INPUT);
  digitalWrite(_pinWaterFluxSensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(_pinWaterFluxSensor), onWaterFluxRead, FALLING);

  // LCD 16 x 2
  if (_useLCD) {
    Serial.print("starting LCD...");
    lcd.begin(16, 2); //Inicia o LCD com dimensões 16x2 (Colunas x Linhas)
    lcd.noCursor();
  }
}


void loop() {
  if (_useNetwork) {
    Ethernet.maintain();
    mqtt.loop();
  }

  // counts how many flux sensor pulses in 1 second
  pulseSensorCounter = 0;
  loopCounter++;
  sei();         // enables sensor interruption
  delay(1000);   // waits one second
  cli();         // disables sensor interruption

  //
  // sensor datasheet: 450 pulses per liter
  //
  // therefore:
  // flux = (pulses in 1 s)/450 [L/seg]
  // flux = (pulses in 1 s)/7.5 [L/min]
  // flux = 8 * (pulses in 1 s) [L/h]
  // flux = 8 * (pulsos in 1 s) / 1000 [m3/h]
  //
  // flux [m3/h] = 0.06 * flux [L/min]
  //
  // flux moving average estimation:
  //    EWMA[t] = lambda * fluxo + (1 - lambda) * EWMA[t-1]
  //
  ewma = _LambdaEWMA * (float)pulseSensorCounter * 60 / _pulsesPerLiter + (1 - _LambdaEWMA) * ewma;  
  if (_useNetwork)
    sensor.setValue(ewma);

  // leakage integrator (volume estimator)
  if (ewma < 0.01) {
    leakageIntegrator = 0;
  } else {
    leakageIntegrator -= leakageIntegrator / N;
    leakageIntegrator += (float)pulseSensorCounter / N;
  }
  
  // closes the valve (if leakage threshold is hitten)
  if (_canCloseWater && leakageIntegrator > (_LeakageThreshold * _pulsesPerLiter))
    changeValveState(true);
  
  // shows message
  if (_useLCD) 
    showLcdMessage();

  // publishes state
  // if (_useNetwork) publishCurrentState();

  // blink built in led
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, (ledState ? HIGH : LOW));

  // resets loop counter
  if (loopCounter > 20) 
    loopCounter = 0;
}

void onWaterFluxRead() {
  // handler do evento de pulso do sendor hal: conta pulsos
  pulseSensorCounter++;
}

void changeValveState(boolean closed) {
  //
  solenoidValveClosed = closed;
  digitalWrite(_poutSolenoidValve, (closed ? LOW : HIGH));
}


//void onMqttConnected() {
//  // Serial.println("connected to the MQTT broker");
//  mqtt.subscribe("leakino/entrada/cmd");
//}
//
//void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length) {
//  // This callback is called when message from MQTT broker is received.
//  // note that you should always verify if the message's topic is the one you expect
//  if (strcmp(topic, "leakino/entrada/cmd") != 0) return;
//
//  if (strcmp((const char*)payload, "reset") == 0) {
//    // reset states message
//    ewma = 0;
//    leakageIntegrator = 0;
//    return;
//  }
//
//  if (strcmp((const char*)payload, "open") == 0) {
//    // opens valve
//    digitalWrite(_poutSolenoidValve, HIGH);
//    solenoidValveClosed = false;
//  } else if (strcmp((const char*)payload, "close") == 0) {
//    // closes valve
//    digitalWrite(_poutSolenoidValve, LOW);
//    solenoidValveClosed = true;
//  }
//
//  showLcdMessage();
//  publishCurrentState();
//}

void onButtonCommand(HAButton* sender)
{
  ewma = 0;
  leakageIntegrator = 0;
//  if (sender == &btnReset) {
//    // reset states
//    ewma = 0;
//    leakageIntegrator = 0;
//  }
}

void onSwitchCommand(bool state, HASwitch* sender)
{
  changeValveState(state);    
  sender->setState(state); 
}

void showLcdMessage() {
  //
  // Renders LCD messages
  //
  if (_useLCD) 
    return;

  lcd.clear();
  if (solenoidValveClosed) {
    lcd.print("Valvula Fechada!");
    return;
  }

  lcd.print("Fluxo: ");
  if (loopCounter > 10) {
    // show water flux as m3/h
    lcd.print(0.06 * ewma);
    lcd.print(" m3/h");
    lcd.setCursor(0, 1);
    lcd.print(" Prob: ");
    lcd.print(100 * leakageIntegrator / (_LeakageThreshold * _pulsesPerLiter));
    lcd.print("%");
  } else {
    lcd.print(ewma);
    lcd.print(" L/min");
    lcd.setCursor(0, 1);
    lcd.print(" Sem Vazamentos");
  }
}

//void publishCurrentState() {
//  if (!_useNetwork) return;
//
//  sensor.setAvailability(!sensor.isOnline());
//  //
//  // allocates the JSON document
//  //
//  // inside the brackets, 200 is the RAM allocated to this document.
//  // don't forget to change this value to match your requirement.
//  // use to compute the capacit: https://arduinojson.org/v6/example/generator/
//  StaticJsonDocument<48> doc;
////  if (solenoidValveClosed) {
////    sensor.setValue(0);
////    doc["state"] = "closed";
////    doc["flux_m3"] = 0;
////    doc["flux_liters"] = 0;
////    doc["prob"] = 0;
////  } else {
////    sensor.setValue(ewma);
////    doc["state"] = "opened";
////    doc["flux"] = ewma; // [L/min]
////    doc["flux_m3"] = 0.06 * ewma; // [m3/h]
////    doc["prob"] = 100 * leakageIntegrator / (_LeakageThreshold * _pulsesPerLiter);
////  }
////
////  char output[128];
////  serializeJson(doc, output);
////  Serial.println(output);
////  mqtt.publish("leakino/entrada/state", output);
//}
