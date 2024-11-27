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
bool inicio, a = 1;
float controladorOUT = 0;
float x[3] = {0};  // Input samples (u[k], u[k-1], u[k-2])
float y[3] = {0};  // Output samples (y[k], y[k-1], y[k-2])
int tiempo, tiempoinicio, tiempoanterior, referencia, encoderTicksActual, encoderTicksAnterior, TiempoCicloPromedio = 0;
float tiempociclo, RPM, RPMPromedio = 0;

//Variables nuevas para PID posición
float Kp = ;
float Ki = ;
float T = ;

float error, previousError, integral, output, reference, distancia = 0;

String DatoSerial;

// IDENTIFICACIÓN DE SISTEMA
int avgwindowsize = 1; // Variar este parámetro. El lag que introduce un movingAvg es (N-1)/2 muestras
int delayintencional = 15; // Variar este parámetro

movingAvg RPMAvg(avgwindowsize);
movingAvg TiempoCicloAvg(20); //Debe ser un valor estable

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

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderTicks--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderTicks++; //Solo 1 dirección

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
  a = 0; //variable auxiliar para cambio de referencia (setpoint)
  RPMAvg.begin();
  TiempoCicloAvg.begin();
  randomSeed(analogRead(0)); //Inicializa aleatoriamente el generador de números aleatorios
}

void loop() {
  if(a==0){  //Genera referencias variables para entrenamiento red neuronal
    // Solo debe haber UNA referencia activa
    //referencia=random(70,180); // Deben operar el motor en su zona lineal! Las FT son lineales
    referencia=random(-2500,2500); // Referencias para desplazamientos en control de posición
    tiempoinicio=millis();
    a=1;
  }
  tiempoanterior=tiempo;
  tiempo=millis();
  tiempociclo=tiempo-tiempoanterior;
  if((tiempo-tiempoinicio>=5000) && a==1){ //mantiene la referencia por este tiempo
    a=0;
    // Solo para control de posición
    // Aquí se escribe PWM de cero con delay para evitar frenar el motor con voltaje opuesto
    // Eso puede quemar el motor si no se considera (en motores grandes)
    analogWrite(pwmPin,0); //Solo para control de posición
    encoderTicks=0; //Solo para control de posición
    delay(1000); //Solo para control de posición
  }

  distancia = encoderTicks;
  output = ;

  //RPM=encoderTicks/(TiempoCicloPromedio)*70; //Factor de conversión para pasar de Ticks a RPM. Esto depende de su encoder.
  //RPMPromedio=RPMAvg.reading(RPM);
  //encoderTicks=0; // Control de velocidad
  TiempoCicloPromedio=TiempoCicloAvg.reading(tiempociclo);
  error=referencia-distancia;

  //Definición del controlador en Z
  x[0] = error;  // error

  // Calcula la salida en base a la ecuación del controlador
  //float controladorOUT =  y[1] + 0.087028 * x[1]; //Ecuación ejemplo 1
  float controladorOUT =  1.295 * y[1] - 0.5292 * y[2] + 0.33877 * x[2]; //Ecuación ejemplo 2

  // Recorta la salida entre valor mínimo y máximo de PWM
  controladorOUT = constrain(controladorOUT, 0, 255);

  // Mover los valores anteriores de las entradas/salidas. X[] es error (entrada), Y[] el pwm (salida)
  x[2] = x[1];   // u[k-2] = u[k-1]
  x[1] = x[0];   // u[k-1] = u[k]

  y[2] = y[1];   // y[k-2] = y[k-1]
  y[1] = controladorOUT;   // y[k-1] = y[k]

  // Solo debe haber UNO activado!
  //analogWrite(pwmPin, controladorOUT); // Este debe estar activado si quieren usar el controlador
  int pwmOutput = constrain(output, -255, 255);  // Para considerar valores positivos y negativos
  if(error>=0){
    digitalWrite(in1Pin, HIGH); //Dirección de giro A. Para más detalle, hoja de datos l298N
    digitalWrite(in2Pin, LOW);
    delay(2);
    error=abs(error);
    analogWrite(pwmPin,pwmOutput);
  }
  else{
    digitalWrite(in1Pin, LOW); //Dirección de giro -A
    digitalWrite(in2Pin, HIGH);
    delay(2);
    error=abs(error);
    analogWrite(pwmPin,pwmOutput);
  }
  //analogWrite(pwmPin, RPMPromedio); // Este debe estar activado para los datos de para la identificación del sistema

  Serial.print(referencia);
  Serial.print(" ");
  Serial.print(distancia);
  Serial.print(" ");
  Serial.println(TiempoCicloPromedio);
  delay(delayintencional);
}