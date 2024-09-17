#include <Arduino.h>
#include <movingAvg.h>

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
float controladorOUT = 0;
float x[2] = {0}; // Input samples
float y[2] = {0}; // Output samples
int accionProp, accionDeriv, RPMMedian, accionInteg, error, tiempo, tiempoinicio, tiempoanterior, referencia, encoderTicksActual, encoderTicksAnterior, deltaencoderTicks, sumerror, time_delay, TiempoCicloPromedio = 0;
float Kp, Ki, Kd, tiempociclo, RPM, RPM1erFilt, RPMPromedio, RPMPromedioanterior, Prom1 = 0;
String DatoSerial;

// IDENTIFICACIÓN DE SISTEMA
int avgwindowsize = 5; // Variar este parámetro. El lag que introduce un movingAvg es (N-1)/2 muestras
int delayintencional = 10; // Variar este parámetro

movingAvg RPMAvg(avgwindowsize);
movingAvg TiempoCicloAvg(20);

// Variables para el conteo de pulsos del encoder
volatile float encoderTicks = 0;
int lastEncoded = 0;

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
  RPMAvg.begin();
  TiempoCicloAvg.begin();
  randomSeed(analogRead(0)); //Inicializa aleatoriamente el generador de números aleatorios
}

void loop() {
  if(a==0){  //Genera referencias variables para entrenamiento red neuronal
    referencia=random(70,160); // Deben operar el motor en su zona lineal! Las FT son lineales
    tiempoinicio=millis();
    a=1;
  }
  tiempoanterior=tiempo;
  tiempo=millis();
  tiempociclo=tiempo-tiempoanterior;
  if((tiempo-tiempoinicio>3000) && a==1){ //mantiene la referencia por este tiempo
    a=0;
  }

  RPM=encoderTicks/(TiempoCicloPromedio)*78.125; //Factor de conversión para pasar de Ticks a RPM. Esto depende de su encoder.
  RPMPromedio=RPMAvg.reading(RPM);
  encoderTicks=0;
  TiempoCicloPromedio=TiempoCicloAvg.reading(tiempociclo);
  error=referencia-RPMPromedio;

  //Definición del controlador en Z
  //controladorOUT = 1.0582*error+1.037*y[0]-0.4021*y[1];
  //controladorOUT = constrain(controladorOUT,0,255);
  //x[0] = error; //error es la entrada U
  //y[1] = y[0];
  //y[0] = controladorOUT; //controlador OUT es la salida Y
  //analogWrite(pwmPin, controladorOUT);
  
  analogWrite(pwmPin, referencia);

  Serial.print(int(referencia));
  Serial.print(", ");
  Serial.print(int(RPMPromedio));
  Serial.print(", ");
  Serial.println(int(TiempoCicloPromedio));
  delay(delayintencional);
}