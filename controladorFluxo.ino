#include <LiquidCrystal.h>
/*

   Projeto Controlador de Fluxo (protecao contra vazamentos)
   - Sensor Hal + Valvula solenoide

   ref sensor fluxo: http://www.nadielcomercio.com.br/blog/2015/10/14/sensor-de-fluxo-de-agua/ - https://www.youtube.com/watch?v=TEp39b4EguQ
   ref ligacao da valvula: http://www.nadielcomercio.com.br/blog/2015/09/10/controlando-uma-valvula-solenoide-de-um-arduino/
   ref lcd: http://labdegaragem.com/profiles/blogs/tutorial-lcd-com-arduino
   ref bluetooth (virtuino): https://www.youtube.com/watch?v=CYR_jigRkgk

*/

//
// configuracao
//
const int N = 600;                     // 10 min de intervalo de contagem de volume   // Nota: tentar usar operador >> 9 (2^9 = 512s ~9 min de intervalo)
const float integradorThesold = 0.9;   // 90% da vazao teorica maxima (teorico: 60 L/min - pratico: 30 L/min - ajustado: ??)
const float mediaMovelLambda = 0.94;   // lambda EWMA (media movel da medida de vazao)
const int pulsosPorLitro = 450;        // pulsos por litro (datasheet do sensor de fluxo) 
const int pinSensorFluxo = 2;          // pino para conexao do PWM do sensor de fluxo (no Nano, apenas pinos 2 e 3)
const int pinBotao = 8;                // pino para o push button do botao de reset
const int poutSolenoide = 10;          // pino para o circuito de alimentacao da solenoide
//
// variaveis de dados
//
int contadorPulsos = 0;                // contador de pulsos
int contadorSegundos = 0;              // contador de segundos (alterna unidade do fluxo)
float mediaMovel = 0;                  // media movel da estimativa de fluxo (L/min)
float integrador = 0;                  // integrador de vazao para limite da valvula
boolean fechaValvula = false;          // estado do controle da valvula
boolean estadoLed = false;             // estado do led built in
int estadoBotao = 0;                   // estado do push button de reset

//
// staticos
// 
LiquidCrystal lcd(12, 11, 5, 4, 3, 6);                   // configura os pinos do Arduino para se comunicar com o display LCD


void setup() {
  // serial (debug)
  // Serial.begin(9600);

  // led built in piscando
  pinMode(LED_BUILTIN, OUTPUT);

  // botao de reset (reabrir valvula)
  //pinMode(pinBotao, INPUT_PULLUP);

  // saida solenoide (default: fechada)
  pinMode(poutSolenoide, OUTPUT); 
  digitalWrite(poutSolenoide, HIGH); // abre

  // leitura do fluxo no pino 2 (interrupcao 0) - fluxometro
  pinMode(pinSensorFluxo, INPUT);
  digitalWrite(pinSensorFluxo, HIGH);
  attachInterrupt(digitalPinToInterrupt(pinSensorFluxo), leituraSensorFluxoHandler, FALLING);

  // LCD 16 x 2  
  lcd.begin(16, 2); //Inicia o LCD com dimensões 16x2 (Colunas x Linhas)
  lcd.noCursor();
}



void loop() {
  
  // conta pulsos em 1 segundo
  contadorPulsos = 0;
  contadorSegundos++;
  sei();         // habilita interrupção
  delay(1000);   // aguarda 1 segundo
  cli();         // Desabilita interrupção
  
  // botao de reset para abrir a valvula solenoide (caso fechada)
//  estadoBotao = digitalRead(pinBotao);
//  if (estadoBotao = LOW) {
  if (false) {
    // abre valvula quando pressionado o botao
    mediaMovel = 0;
    integrador = 0;
    fechaValvula = false;
    digitalWrite(poutSolenoide, HIGH); // abre valvula
    lcd.clear();
    lcd.print("Abrindo valvula!");
    delay(1000);   // aguarda 1 segundo
    return;
  }

  //
  // datasheet: 450 pulsos por litro
  // fluxo = (pulsos em 1 s)/450 [L/seg]
  // fluxo = (pulsos em 1 s)/7.5 [L/min]
  // fluxo = 8 * (pulsos em 1 s) [L/h]
  // fluxo = 8 * (pulsos em 1 s) / 1000 [m3/h]
  //
  // media EWMA[t] = lambda * fluxp + (1 - lambda) * EWMA[t-1]
  //
  mediaMovel = mediaMovelLambda * (float)contadorPulsos * 60 / pulsosPorLitro + (1 - mediaMovelLambda) * mediaMovel;
  
  // integrador de volume
  if (mediaMovel < 0.01) { 
    integrador = 0;
  } else {
    integrador -= integrador / N;
    integrador += (float)contadorPulsos / N;
  }


  if (integrador > (integradorThesold * pulsosPorLitro)) {
    // fecha valvula (se limite de deteccao atingido)
    fechaValvula = true;
  }

  if (fechaValvula) 
  {
    // estado da valvula: fechada
    lcd.clear();
    lcd.print("  Vazamento!");
    lcd.setCursor(0, 1);
    lcd.print("Valvula fechada");

    // luz led ligada
    digitalWrite(LED_BUILTIN, HIGH);
    // fecha valvula 
    digitalWrite(poutSolenoide, LOW); 
    return;    
  } 

  // pisca led built in
  estadoLed = !estadoLed;
  digitalWrite(LED_BUILTIN, estadoLed);

  // valvula aberta
  lcd.clear();
  lcd.print("Fluxo: ");
  if (contadorSegundos > 10)
  {
    // mostrador em m3/h
    lcd.print(0.06 * mediaMovel);
    lcd.print(" m3/h");          
    lcd.setCursor(0,1);    
    lcd.print(" Prob: ");
    lcd.print(100 * integrador / (integradorThesold * pulsosPorLitro));
    lcd.print("%");
  } 
  else 
  {
    // mostrador em L/min
    lcd.print(mediaMovel);
    lcd.print(" L/min");
    lcd.setCursor(0,1);    
    lcd.print(" Sem Vazamentos");
  }
  
  if (contadorSegundos > 20) {   
    contadorSegundos = 0;
  }  
}

void leituraSensorFluxoHandler() {
  // handler do evento de pulso do sendor hal: conta pulsos
  contadorPulsos++;
}
