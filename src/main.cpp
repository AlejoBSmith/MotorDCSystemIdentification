#include <Arduino.h>
#include <movingAvg.h>
#include <StringSplitter.h>

// Pines control motor
const int pwmPin = 9;   // Enable pin (PWM input)
const int in1Pin = 7;   // IN1 pin
const int in2Pin = 8;   // IN2 pin

// Pines encoder
const int encoderA = 2; // Encoder channel A
const int encoderB = 3; // Encoder channel B

//Controlador
bool inicio = 1;
int sumaPID, a = 0;
int accionProp, accionDeriv, accionInteg, error, tiempo, tiempoinicio, tiempoanterior, referencia, encoderTicksActual, encoderTicksAnterior, deltaencoderTicks, sumerror, time_delay, TiempoCicloPromedio = 0;
float Kp, Ki, Kd, tiempociclo, RPM, RPM1erFilt, RPMPromedio, RPMPromedioanterior = 0;
String DatoSerial;
int avgwindowsize = 1; //El lag que introduce un movingAvg es (N-1)/2 muestras
int delayintencional = 100;
movingAvg RPM1(avgwindowsize);
movingAvg RPMAvg(avgwindowsize);
movingAvg TiempoCicloAvg(10);

volatile float encoderTicks = 0;
int lastEncoded = 0;
String StringEntrada;

void updateEncoder() {
  // Se leen los encoder A y B para luego combinarlos en un solo número
  // Para más info, ver https://en.wikipedia.org/wiki/Incremental_encoder sección "State transitions"
  int MSB = digitalRead(encoderA);
  int LSB = digitalRead(encoderB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderTicks++;
  //if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderTicks--; //Solo 1 dirección

  lastEncoded = encoded;
}

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), updateEncoder, CHANGE);
  Serial.begin(57600);
  referencia = 80;
  Kp = 1;
  Ki = 0.075;
  Kd = 4;
  a = 0; //variable auxiliar para cambio de referencia (setpoint)
  RPM1.begin();
  RPMAvg.begin();
  TiempoCicloAvg.begin();
  randomSeed(analogRead(0)); //Inicializa aleatoriamente el generador de números aleatorios
}

void loop() {
  if(a==0){  //Genera referencias variables para entrenamiento red neuronal
    //referencia=100; // Para encontrar el ruido
    referencia=random(70,160); // Deben operar el motor en su zona lineal! Las FT son lineales
    tiempoinicio=millis();
    a=1;
  }
  tiempoanterior=tiempo;
  tiempo=millis();
  tiempociclo=tiempo-tiempoanterior;
  if((tiempo-tiempoinicio>3000) && a==1){
    a=0;
  }
  //encoderTicksAnterior=encoderTicksActual;
  //encoderTicksActual=encoderTicks;
  //deltaencoderTicks=encoderTicksActual-encoderTicksAnterior;
  //RPM=deltaencoderTicks/(tiempociclo)*78.125;
  RPM=encoderTicks/(tiempociclo)*78.125;
  encoderTicks=0;
  RPMPromedioanterior = RPMPromedio;
  RPM1erFilt=RPM1.reading(RPM);
  RPMPromedio=RPMAvg.reading(RPM1erFilt);
  TiempoCicloPromedio=TiempoCicloAvg.reading(tiempociclo);
  error=referencia-RPMPromedio;
  accionProp=error*Kp;
  accionProp=constrain(accionProp,0,255);
  analogWrite(pwmPin, referencia);

  Serial.print(int(referencia));
  Serial.print(", ");
  Serial.print(int(RPMPromedio));
  Serial.print(", ");
  Serial.println(int(TiempoCicloPromedio));
  delay(delayintencional);
}