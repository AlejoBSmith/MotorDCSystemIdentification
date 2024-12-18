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
bool inicio, a = 1;
float x[5] = {0};  // Input samples (u[k], u[k-1], u[k-2])
float y[5] = {0};  // Output samples (y[k], y[k-1], y[k-2])
int tiempo, tiempoinicio, tiempoanterior, referencia, encoderTicksActual, encoderTicksAnterior, TiempoCicloPromedio, medicion, modo = 0;
int tiemporeferencia = 5000;
int modooperacion, itemCount = 0;
int pwmOutput, valorsalidapwm = 0;
float controladorOUT = 0;
int pwmMinimoLinealidad = 70;
int pwmMaximoLinealidad = 180;
float tiempociclo, RPM, RPMPromedio = 0;

//Variables para controlador
float A, B, C, D, E, F, G, H = 0;
float T = 0.016; //Tiempo de muestreo

float error, previousError, integral, output, reference, distancia = 0;

String DatoSerial;

// IDENTIFICACIÓN DE SISTEMA
// El avgwindowsize sirve para filtar las mediciones si son muy ruidosas.
// El delayintencional permite recoger más datos para mediciones más limpias, pero reduce el margen de fase.
int avgwindowsize = 1; // Variar este parámetro. El lag que introduce un movingAvg es (N-1)/2 muestras
int delayintencional = 10; // Variar este parámetro

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
int GeneradorReferencia(int modooperacion){
  switch (modooperacion){
    case 0:
      referencia = 0;
      break;
    case 1:
    case 2:
      if(a==0){  //Genera referencias variables para entrenamiento / identificación de sistema
        referencia=random(pwmMinimoLinealidad,pwmMaximoLinealidad); // Deben operar el motor en su zona lineal! Las FT son lineales. Esos valores de 70 y 180 dependen de su motor y deberán experimentar si sirven.
        tiempoinicio=millis();
        a=1;
      }
      if((tiempo-tiempoinicio>=tiemporeferencia) && a==1){ //mantiene la referencia por este tiempo y luego la cambia
        a=0;
      }
      break;
    case 3:
      if(a==0){
        referencia=random(-2500,2500); // Referencias para desplazamientos en control de posición
        tiempoinicio=millis();
        a=1;
      }
      if((tiempo-tiempoinicio>=tiemporeferencia) && a==1){ //mantiene la referencia por este tiempo
        a=0;
        // Aquí se escribe PWM de cero con delay para evitar frenar el motor con voltaje opuesto
        // Eso puede quemar el motor si no se considera (sobretodo en motores grandes)
        // También hay que borrar la "memoria" del término integral
        integral = 0;
        distancia = 0;
        output = 0;
        analogWrite(pwmPin,0);
        encoderTicks=0;
        delay(1000);
      }
      break;
  }
  return referencia;
}
int EcuacionDiferencias(float error){
  //Definición del controlador en ecuación diferencias (a partir de la FT en Z)
  x[0] = error;  // error

  // Calcula la salida en base a la ecuación del controlador
  //float controladorOUT =  y[1] + 0.087028 * x[1]; //Ecuación ejemplo 1
  //controladorOUT =  1.295 * y[1] - 0.5292 * y[2] + 0.33877 * x[2]; //Ecuación ejemplo 2
  controladorOUT =  A * x[0] + B * x[1] + C * x[2] + D * x[3] + E * y[1] + F * y[2] + G * y[3] + H * y[4];
  // Recorta la salida entre valor mínimo y máximo de PWM
  if (modooperacion == 2){ //Para el control de velocidad, debe ser número positivo
    controladorOUT = constrain(controladorOUT, 0, 255);
  }
  if (modooperacion == 3){ //Para control de posición, el pwm puede ser negativo (inversión de sentido de giro)
    controladorOUT = constrain(controladorOUT, -255, 255);
  }
  // Mueve los valores anteriores de las entradas/salidas. X[] es error (entrada), Y[] el pwm (salida)
  x[3] = x[2];   // u[k-3] = u[k-2]
  x[2] = x[1];   // u[k-2] = u[k-1]
  x[1] = x[0];   // u[k-1] = u[k]

  y[3] = y[2];   // y[k-3] = y[k-2]
  y[2] = y[1];   // y[k-2] = y[k-1]
  y[1] = controladorOUT;   // y[k-1] = y[k]
  return controladorOUT;
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
  Serial.begin(115200);
  referencia = 0;
  a = 0; //variable auxiliar para cambio de referencia (setpoint)
  RPMAvg.begin();
  TiempoCicloAvg.begin();
  randomSeed(analogRead(2)); //Inicializa aleatoriamente el generador de números aleatorios
}

void loop() {
  // Comunicación serial con la interfaz
  if (Serial.available() > 0){
    DatoSerial = Serial.readStringUntil('\n');
    StringSplitter *splitter = new StringSplitter(DatoSerial, ',', 60);  // new StringSplitter(string_to_split, delimiter, limit)
    inicio = splitter->getItemAtIndex(0).toInt();
    modooperacion = splitter->getItemAtIndex(1).toInt();
    A = splitter->getItemAtIndex(2).toFloat();
    B = splitter->getItemAtIndex(3).toFloat();
    C = splitter->getItemAtIndex(4).toFloat();
    D = splitter->getItemAtIndex(5).toFloat();
    E = splitter->getItemAtIndex(6).toFloat();
    F = splitter->getItemAtIndex(7).toFloat();
    G = splitter->getItemAtIndex(8).toFloat();
    H = splitter->getItemAtIndex(9).toFloat();
    delayintencional = splitter->getItemAtIndex(10).toInt();
    itemCount = splitter->getItemCount();
    Serial.println(itemCount);
    
    if (delayintencional <=10){ //El delay intencional deber ser como MÍNIMO 10 con un baud rate de 115200. Con otro micro más rápido que el Arduino UNO puede bajar. Si no, se sobrecarga el buffer y se tildea
      delayintencional=10;
    }
    
    referencia=0;
    medicion=0;
    TiempoCicloPromedio=0;
  }
  if (inicio == 1){
  // El motor se puede configurar para hacer un controlador de velocidad, de posición o identificación de sistema
    switch(modooperacion){
      case 0: // 0 es para que no haga nada
        analogWrite(pwmPin, 0);
        delay(100);
        break;

      case 1: // IDENTIFICACIÓN DE SISTEMA
        controladorOUT = GeneradorReferencia(modooperacion);
        RPM=encoderTicks/(TiempoCicloPromedio)*70; //Factor de conversión para pasar de Ticks a RPM. Esto depende de su encoder.
        RPMPromedio=RPMAvg.reading(RPM);
        encoderTicks=0; // Control de velocidad
        analogWrite(pwmPin, controladorOUT);
        medicion = RPMPromedio;
        break;

      case 2: // CONTROL DE VELOCIDAD
        referencia = GeneradorReferencia(modooperacion);
        RPM=encoderTicks/(TiempoCicloPromedio)*70; //Factor de conversión para pasar de Ticks a RPM. Esto depende de su encoder.
        RPMPromedio=RPMAvg.reading(RPM);
        encoderTicks=0; // Reinicia el conteo de los ticks para cálculo de RPM
        error = referencia - RPMPromedio;
        controladorOUT = EcuacionDiferencias(error);
        analogWrite(pwmPin, controladorOUT);
        medicion = RPMPromedio;
        break;

      case 3: // CONTROL DE POSICIÓN
        referencia = GeneradorReferencia(modooperacion);
        distancia = encoderTicks;
        error = referencia - distancia;
        controladorOUT = EcuacionDiferencias(error);

        // Enviar la salida del controlador al PWM
        controladorOUT = constrain(controladorOUT, -255, 255);  // Para considerar valores positivos y negativos
        if(error>=0){
          digitalWrite(in1Pin, HIGH); //Dirección de giro A. Para más detalle, hoja de datos l298N
          digitalWrite(in2Pin, LOW);
          delay(2);
          controladorOUT=abs(controladorOUT);
          analogWrite(pwmPin,controladorOUT);
        }
        if(error<=0){
          digitalWrite(in1Pin, LOW); //Dirección de giro -A
          digitalWrite(in2Pin, HIGH);
          delay(2);
          controladorOUT=abs(controladorOUT);
          analogWrite(pwmPin,controladorOUT);
        }
        medicion = distancia;
        break;
    }
  }
  if (inicio == 0){
    analogWrite(pwmPin,0);
    modooperacion = 0;
    delay(100);
  }
  
  //Cálculo de tiempos
  TiempoCicloPromedio=TiempoCicloAvg.reading(tiempociclo);
  tiempoanterior=tiempo;
  tiempo=millis();
  tiempociclo=tiempo-tiempoanterior;

  //Comunicación de datos
  Serial.print(referencia);
  Serial.print(" ");
  Serial.print(medicion);
  Serial.print(" ");
  Serial.print(TiempoCicloPromedio);
  Serial.print(" ");
  Serial.print(modooperacion);
  Serial.print(" ");
  Serial.print(delayintencional);
  Serial.print(" ");
  Serial.print(E);
  Serial.print(" ");
  Serial.print(F);
  Serial.print(" ");
  Serial.println(controladorOUT);
  delay(delayintencional); //El delay intencional debe ser como mínimo de 10 ms. Si no, se sobrecarga el búffer y se tildea la GUI
}