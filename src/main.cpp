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
float x[5] = {0};  // Para entradas actuales y anteriores (u[k], u[k-1], u[k-2])
float y[5] = {0};  // Para salidas actuales y anteriores (y[k], y[k-1], y[k-2])
int referencia, encoderTicksActual, encoderTicksAnterior, TiempoCicloPromedio, medicion, modo = 0;
int tab_activo = 0; // Para saber en qué tab está la interfaz gráfica, de eso depende si es PID o digital controller
unsigned long tiempo, tiempoinicio, tiempoanterior, tiemporestart = 0;
int tiemporeferencia = 2000; //Tiempo de referencia para cambio de setpoint
int modooperacion, itemCount, tiporef, tiposenal, referenciaManual = 0;
int pwmOutput, valorsalidapwm = 0;
int controladorOUT, amplitudAuto, offset = 0;
int pwmMinimoLinealidad = 70;
int pwmMaximoLinealidad = 180;
float tiempociclo, RPM, RPMPromedio = 0;

//Variables para controlador
float alfa = 0.01; //Relación entre el tiempo de muestreo y el tiempo de filtrado para componente derivativa Ts/Tf
float A_cont, B_cont, C_cont, D_cont, E_cont, F_cont, G_cont, H_cont = 0;
float Kp, Ki, Kd = 0;
float error, previousError, integral, output, previousOutput, reference, distancia = 0;
String DatoSerial;

// IDENTIFICACIÓN DE SISTEMA
// El avgwindowsize sirve para filtar las mediciones si son muy ruidosas.
// El delayintencional permite recoger más datos para mediciones más limpias, pero reduce el margen de fase.
int avgwindowsize = 1; // Variar este parámetro. El lag que introduce un movingAvg es (N-1)/2 muestras
int delayintencional = 15; // Variar este parámetro

movingAvg RPMAvg(avgwindowsize);
movingAvg TiempoCicloAvg(20); //Debe ser un valor estable

// Variables para el conteo de pulsos del encoder
volatile float encoderTicks = 0;
int lastEncoded = 0;

void divideString(String DatoSerial, char delimitador, String salida[], int limite) {
    int posicion = 0; // Posición del delimitador en la cadena
    int indice = 0;   // Índice del array de salida
    // Bucle para encontrar los delimitadores y dividir la cadena
    while ((posicion = DatoSerial.indexOf(delimitador)) != -1 && indice < limite - 1) {
        salida[indice++] = DatoSerial.substring(0, posicion); // Extrae la subcadena antes del delimitador
        DatoSerial = DatoSerial.substring(posicion + 1);         // Actualiza la entrada eliminando la parte procesada
    }
    salida[indice] = DatoSerial; // Agrega la última parte de la cadena
}
void inicializavariables(){
  referencia=0;
  medicion=0;
  tiempo = 0;
  tiempoanterior=0;
  tiempociclo=0;
  RPMPromedio = 0;
  encoderTicks=0;
  controladorOUT=0;
  RPM=0;
  encoderTicksActual=0;
  encoderTicksAnterior=0;
  encoderTicks=0;
  error=0;
  previousError=0;
  previousOutput=0;
  distancia=0;
  medicion=0;
  lastEncoded=0;
  RPMAvg.reset();
}
void updateEncoder() {
  // Se leen los encoder A y B para luego combinarlos en un solo número
  // Para más info, ver https://en.wikipedia.org/wiki/Incremental_encoder sección "State transitions"
  int MSB = digitalRead(encoderA);
  int LSB = digitalRead(encoderB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderTicks--;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderTicks++; //Solo 1 dirección*/

  lastEncoded = encoded;
}

float PRBS() { //Psuedo Random Binary Sequence: valores aleatorios de tiempo definido entre valor mínimo y máximo	
  if((tiempo - tiempoinicio) >= tiemporeferencia) {
    referencia = random(pwmMinimoLinealidad, pwmMaximoLinealidad);
    tiempoinicio = millis()-tiemporestart;
  }
  return referencia;
}
float Cuadrada() {
  if (fmod(tiempo, tiemporeferencia) < tiemporeferencia / 2) {
    return amplitudAuto + offset;  // Valor alto
  } 
  else {
    return offset;  // Valor bajo
  }
}
float Senoidal() {
  return (amplitudAuto * sin(2 * PI * tiempo / tiemporeferencia) + offset);
}
float TriangularZonaMuerta() {
    float TiempoCiclo = fmod(tiempo, tiemporeferencia);
    float TiempoSeccion = tiemporeferencia / 3;

    if (TiempoCiclo < TiempoSeccion) {
        return (amplitudAuto / TiempoSeccion) * TiempoCiclo;  // Rampa ascendente
    } 
    else if (TiempoCiclo < 2 * TiempoSeccion) {
        return amplitudAuto - ((amplitudAuto / TiempoSeccion) * (TiempoCiclo - TiempoSeccion));  // Rampa descendente
    } 
    else {
        return 0;  // Zona muerta (tercio final)
    }
}
float Impulso() {
  if (tiempo < tiemporeferencia) {
    return amplitudAuto + offset;  // Impulso de corta duración
  } 
  else {
    return offset;  // Valor bajo
    float tiempoespera = 10000; //Espera para el siguiente impulso
    if (tiempo > tiemporeferencia + tiempoespera) {
      tiempo = 0;
    }
  }
}
float Chirp() { //Barrido en frecuencia
  return (amplitudAuto * sin(2 * PI * (0.001 * tiempo) * tiempo * 0.0001)) + offset;
}
float DecaimientoExponencial() {
  return amplitudAuto * exp(-0.0005 * tiempo) + offset;
}
float RuidoBlanco() {
  return (random(0, 100) / 100.0) * amplitudAuto + offset;
}

int PID(float error){
  // Discretización usando transformación bilineal (Tustin) de un PID continuo
  float proportional = Kp * (error - previousError);
  float integral = (Ki * TiempoCicloPromedio / 2.0) * (error + previousError);
  float derivative = (2.0 * Kd / TiempoCicloPromedio) * (error - previousError);
  controladorOUT = previousOutput + proportional + integral + derivative;
  previousOutput = controladorOUT;
  previousError = error;
  return controladorOUT;
}
int GeneradorReferencia(int tiposenal){
  switch (tiposenal) { //Tipo de señal es cuadrada, senoidal, etc
    case 0:
      return PRBS();
    case 1:
      return Cuadrada();
    case 2:
      return Senoidal();
    case 3:
      return TriangularZonaMuerta();
    case 4:
      return Impulso();
    case 5:
      return Chirp();
    case 6:
      return DecaimientoExponencial();
    case 7:
      return RuidoBlanco();
    default:
      return 0;
  }
  return referencia;
}
int EcuacionDiferencias(float error){
  //Definición del controlador en ecuación diferencias (a partir de la FT en Z)
  x[0] = error;  // error

  // Calcula la salida en base a la ecuación del controlador
  //float controladorOUT =  y[1] + 0.087028 * x[1]; //Ecuación ejemplo 1
  //controladorOUT =  1.295 * y[1] - 0.5292 * y[2] + 0.33877 * x[2]; //Ecuación ejemplo 2
  controladorOUT =  A_cont * x[0] + B_cont * x[1] + C_cont * x[2] + D_cont * x[3] + E_cont * y[1] + F_cont * y[2] + G_cont * y[3] + H_cont * y[4];
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
/*int Euler(float error){
    float errorfilt = alfa * error + (1 - alfa) * errorfiltanterior;
    // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, derivada con error filtrado
    float derivativa = (errorfilt - errorfiltanterior) / (delayintencional+1); // +1 porque siempre es a 1ms mínimo.
    // e_i[k+1] = e_i[k] + Tₛ e[k], integral
    float integral = integral + error * (delayintencional+1);

    // PID fórmula:
    // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], salida al PWM
    float controladorOUT = kp * error + ki * integral + kd * derivativa;

    // valores para la próxima iteración
    integralanterior = integral;
    errorfiltanterior = errorfilt;
    // valor PWM
    return controladorOUT;
}*/

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
  a = 0; //variable auxiliar para cambio de referencia (setpoint)
  RPMAvg.begin();
  TiempoCicloAvg.begin();
  randomSeed(analogRead(2)); //Inicializa aleatoriamente el generador de números aleatorios
}

void loop() {
  // Comunicación serial con la interfaz gráfica
  if (Serial.available() > 0) {
    DatoSerial = Serial.readStringUntil('\n');
    String parts[60]; // El número de datos de entrada esperados
    divideString(DatoSerial, ',', parts, 60);
    inicio = parts[0].toInt();
    modooperacion = parts[1].toInt();
    A_cont = parts[2].toFloat();
    B_cont = parts[3].toFloat();
    C_cont = parts[4].toFloat();
    D_cont = parts[5].toFloat();
    E_cont = parts[6].toFloat();
    F_cont = parts[7].toFloat();
    G_cont = parts[8].toFloat();
    H_cont = parts[9].toFloat();
    delayintencional = parts[10].toInt();
    tiemporeferencia = parts[11].toInt();
    amplitudAuto = parts[12].toInt();
    referenciaManual = parts[13].toInt();
    offset = parts[14].toInt();
    tiposenal = parts[15].toInt();
    tab_activo = parts[16].toInt();
    Kp = parts[17].toFloat();
    Ki = parts[18].toFloat();
    Kd = parts[19].toFloat();
    if (delayintencional <=10){ //El delay intencional deber ser como MÍNIMO 10 con un baud rate de 115200. Con otro micro más rápido que el Arduino UNO puede bajar. Si no, se sobrecarga el buffer y se tildea
      delayintencional=10;
    }
    inicializavariables();
    tiemporestart = millis();
  }
  if (inicio == 1){
  // El motor se puede configurar para hacer identificación de sistema (modo 1), control de velocidad (modo 2) o control de posición (modo 3)
    switch(modooperacion){
      case 0: // 0 es para que no haga nada
        analogWrite(pwmPin, 0);
        delay(10);
        break;

      case 1: // IDENTIFICACIÓN DE SISTEMA
        referencia = GeneradorReferencia(tiposenal);
        controladorOUT = referencia; // Aquí no se usa el controlador, pero se manda la referencia para visualización
        controladorOUT = constrain(controladorOUT, 0, 255);  // Para considerar valores positivos
        RPM=encoderTicks/(TiempoCicloPromedio)*70; //Factor de conversión para pasar de Ticks a RPM. Esto depende de su encoder.
        RPMPromedio=RPMAvg.reading(RPM);
        encoderTicks=0; // Identificación de sistema reinicia el conteo de los ticks para cálculo de RPM
        analogWrite(pwmPin, controladorOUT);
        medicion = RPMPromedio;
        break;

      case 2: // CONTROL DE VELOCIDAD
        referencia = GeneradorReferencia(tiposenal);
        RPM=encoderTicks/(TiempoCicloPromedio)*70; //Factor de conversión para pasar de Ticks a RPM. Esto depende de su encoder.
        RPMPromedio=RPMAvg.reading(RPM);
        encoderTicks=0; // Control de velocidad reinicia el conteo de los ticks para cálculo de RPM
        error = referencia - RPMPromedio;
        switch (tab_activo){
          case 1: //PID
            controladorOUT = PID(error);
            break;
          case 2: //Digital Controller
            controladorOUT = EcuacionDiferencias(error);
            break;
        }
        controladorOUT = constrain(controladorOUT, 0, 255);  // Para considerar valores positivos
        analogWrite(pwmPin, controladorOUT);
        medicion = RPMPromedio;
        break;

      case 3: // CONTROL DE POSICIÓN
        referencia = GeneradorReferencia(tiposenal);
        distancia = encoderTicks;
        error = referencia - distancia;
        controladorOUT = EcuacionDiferencias(error);

        // Enviar la salida del controlador al PWM
        controladorOUT = constrain(controladorOUT, -255, 255);  // Para considerar valores positivos y negativos
        if(error>=0){
          digitalWrite(in1Pin, HIGH); //Dirección de giro A. Para más detalle, hoja de datos l298N
          digitalWrite(in2Pin, LOW);
        }
        if(error<=0){
          digitalWrite(in1Pin, LOW); //Dirección de giro -A
          digitalWrite(in2Pin, HIGH);
        }
        delay(2);
        controladorOUT=abs(controladorOUT);
        analogWrite(pwmPin,controladorOUT);
        medicion = distancia;
        break;
    }
  }
  if (inicio == 0){
    analogWrite(pwmPin,0);
    delay(10);
  }
  
  //Cálculo de tiempos
  TiempoCicloPromedio=TiempoCicloAvg.reading(tiempociclo);
  tiempoanterior=tiempo;
  tiempo=millis()-tiemporestart;
  tiempociclo=tiempo-tiempoanterior;

  //Comunicación de datos
  if (inicio == 1){
    Serial.print(referencia);
    Serial.print(" ");
    Serial.print(medicion);
    Serial.print(" ");
    Serial.println(controladorOUT);
  }
  delay(delayintencional); //El delay intencional debe ser como mínimo de 10 ms. Si no, se sobrecarga el búffer y se tildea la GUI
}