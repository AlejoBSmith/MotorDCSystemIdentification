#include <Arduino.h>
#include <movingAvg.h>
#include <math.h>

struct MotorPins {
  const int pwmPin = 9;
  const int in1Pin = 7;
  const int in2Pin = 8;
  const int encoderA = 2;
  const int encoderB = 3;
};

struct ControlParams {
  float x[5] = {0, 0, 0, 0, 0};
  float y[5] = {0, 0, 0, 0, 0};
  float error = 0;
  float previousError = 0;
  float previousOutput = 0;
  int controladorOUT = 0;
  int PWMOUT = 0;
  int ZM = 28; // ZM es la zona muerta del motor, se identifica con System Ident, con onda triangular
  float A_cont = 0;
  float B_cont = 0;
  float C_cont = 0;
  float D_cont = 0;
  float E_cont = 0;
  float F_cont = 0;
  float G_cont = 0;
  float H_cont = 0;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float alfa = 0.01;
};

struct SystemVariables {
  bool inicio = false;
  int modooperacion = 0;    // 0: sin acción, 1: identificación, 2: control de velocidad, 3: control de posición
  int tab_activo = 0;       // 1: PID, 2: controlador digital
  int tiposenal = 0;        // 0: PRBS, 1: Cuadrada, 2: Senoidal, etc.
  int referencia = 0;
  int medicion = 0;
  int amplitudAuto = 0;
  int offset = 0;
  int referenciaManual = 0;
  int pwmMinimoLinealidad = 100;
  int pwmMaximoLinealidad = 160;
};

struct TimingData {
  unsigned long tiempo = 0;
  unsigned long tiempoinicio = 0;
  unsigned long tiempoanterior = 0;
  unsigned long tiemporestart = 0;
  float tiempociclo = 0;
  int tiemporeferencia = 2000;
  int delayintencional = 10;
  int TiempoCicloPromedio = 0;
};

struct EncoderData {
  volatile float encoderTicks = 0;
  int encoderTicksActual = 0;
  int encoderTicksAnterior = 0;
  int lastEncoded = 0;
};

struct SerialData {
  String DatoSerial;
};

struct MeasurementsData {
  float RPM = 0;
  float RPMPromedio = 0;
};

MotorPins pins;
ControlParams ctrl;
SystemVariables sys;
TimingData timing;
EncoderData enc;
SerialData serialData;
MeasurementsData meas;

int avgwindowsize = 1;
movingAvg RPMAvg(avgwindowsize);
movingAvg TiempoCicloAvg(20);

void divideString(String DatoSerial, char delimitador, String salida[], int limite) {
  int posicion = 0;
  int indice = 0;
  while ((posicion = DatoSerial.indexOf(delimitador)) != -1 && indice < limite - 1) {
    salida[indice++] = DatoSerial.substring(0, posicion);
    DatoSerial = DatoSerial.substring(posicion + 1);
  }
  salida[indice] = DatoSerial;
}

void inicializaVariables() {
  sys.referencia = 0;
  sys.medicion = 0;
  timing.tiempo = 0;
  timing.tiempoanterior = 0;
  timing.tiempociclo = 0;
  enc.encoderTicks = 0;
  ctrl.controladorOUT = 0;
  ctrl.error = 0;
  ctrl.previousError = 0;
  ctrl.previousOutput = 0;
  enc.lastEncoded = 0;
  RPMAvg.reset();
}

void updateEncoder() {
  int MSB = digitalRead(pins.encoderA);
  int LSB = digitalRead(pins.encoderB);
  int encoded = (MSB << 1) | LSB;
  int sum = (enc.lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    enc.encoderTicks--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    enc.encoderTicks++;
  enc.lastEncoded = encoded;
}

float PRBS() {
    static bool is_zero = false; // Track current phase
    static unsigned long last_switch = 0;

    unsigned long now = timing.tiempo;

    // Duration for each phase
    unsigned long phase_duration = timing.tiemporeferencia / 2;

    if ((now - last_switch) >= phase_duration) {
        last_switch = now;
        is_zero = !is_zero; // Toggle phase

        if (!is_zero) {
            // On new value phase, pick random value
            sys.referencia = random(sys.pwmMinimoLinealidad, sys.pwmMaximoLinealidad);
        }
    }

    return is_zero ? 0 : sys.referencia;
}

float Cuadrada() {
  if (fmod(timing.tiempo, timing.tiemporeferencia) < timing.tiemporeferencia / 2)
    return sys.amplitudAuto + sys.offset;
  else
    return sys.offset;
}

float Senoidal() {
  return (sys.amplitudAuto * sin(2 * PI * timing.tiempo / timing.tiemporeferencia) + sys.offset);
}

float TriangularZonaMuerta() {
  float TiempoCiclo = fmod(timing.tiempo, timing.tiemporeferencia);
  float TiempoSeccion = timing.tiemporeferencia / 3.0;
  if (TiempoCiclo < TiempoSeccion)
    return (sys.amplitudAuto / TiempoSeccion) * TiempoCiclo;
  else if (TiempoCiclo < 2 * TiempoSeccion)
    return sys.amplitudAuto - ((sys.amplitudAuto / TiempoSeccion) * (TiempoCiclo - TiempoSeccion));
  else
    return 0;
}

float Impulso() {
  if (timing.tiempo < timing.tiemporeferencia)
    return sys.amplitudAuto + sys.offset;
  else
    return sys.offset;
}

float Chirp() {
  return (sys.amplitudAuto * sin(2 * PI * (0.001 * timing.tiempo) * timing.tiempo * 0.0001)) + sys.offset;
}

float DecaimientoExponencial() {
  return sys.amplitudAuto * exp(-0.0005 * timing.tiempo) + sys.offset;
}

float RuidoBlanco() {
  return (random(0, 100) / 100.0) * sys.amplitudAuto + sys.offset;
}

int GeneradorReferencia(int tiposenal) {
  switch (tiposenal) {
    case 0: return PRBS();
    case 1: return Cuadrada();
    case 2: return Senoidal();
    case 3: return TriangularZonaMuerta();
    case 4: return Impulso();
    case 5: return Chirp();
    case 6: return DecaimientoExponencial();
    case 7: return RuidoBlanco();
    default: return 0;
  }
}

int PID() {
  float proportional = ctrl.Kp * (ctrl.error - ctrl.previousError);
  float integral = (ctrl.Ki * timing.TiempoCicloPromedio / 2.0) * (ctrl.error + ctrl.previousError);
  float derivative = (2.0 * ctrl.Kd / timing.TiempoCicloPromedio) * (ctrl.error - ctrl.previousError);
  ctrl.controladorOUT = ctrl.previousOutput + proportional + integral + derivative;
  ctrl.controladorOUT = constrain(ctrl.controladorOUT,0,255);
  ctrl.previousOutput = ctrl.controladorOUT;
  ctrl.previousError = ctrl.error;
  return ctrl.controladorOUT;
}

int EcuacionDiferencias() { //Aquí, X es la entrada (error) y Y es la salida (PWM Arduino)
  ctrl.x[3] = ctrl.x[2];
  ctrl.x[2] = ctrl.x[1];
  ctrl.x[1] = ctrl.x[0];
  ctrl.x[0] = ctrl.error;
  ctrl.y[3] = ctrl.y[2];
  ctrl.y[2] = ctrl.y[1];
  ctrl.y[1] = ctrl.controladorOUT; //El +ZM es la zona muerta del motor
  ctrl.controladorOUT = ctrl.A_cont * ctrl.x[0] + ctrl.B_cont * ctrl.x[1] + ctrl.C_cont * ctrl.x[2] + ctrl.D_cont * ctrl.x[3] +
                         ctrl.E_cont * ctrl.y[1] + ctrl.F_cont * ctrl.y[2] + ctrl.G_cont * ctrl.y[3] + ctrl.H_cont * ctrl.y[4];
  if (sys.modooperacion == 2)
    ctrl.controladorOUT = constrain(ctrl.controladorOUT, 0, 255);
  if (sys.modooperacion == 3)
    ctrl.controladorOUT = constrain(ctrl.controladorOUT, -255, 255);
  return ctrl.controladorOUT;
}

void setup() {
  pinMode(pins.pwmPin, OUTPUT);
  pinMode(pins.in1Pin, OUTPUT);
  pinMode(pins.in2Pin, OUTPUT);
  digitalWrite(pins.in1Pin, HIGH);
  digitalWrite(pins.in2Pin, LOW);
  pinMode(pins.encoderA, INPUT);
  pinMode(pins.encoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pins.encoderA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins.encoderB), updateEncoder, CHANGE);
  Serial.begin(115200);
  RPMAvg.begin();
  TiempoCicloAvg.begin();
  randomSeed(analogRead(2));
}

void loop() {
  if (Serial.available() > 0) {
    serialData.DatoSerial = Serial.readStringUntil('\n');
    String parts[60];
    divideString(serialData.DatoSerial, ',', parts, 60);
    sys.inicio = parts[0].toInt();
    sys.modooperacion = parts[1].toInt();
    ctrl.A_cont = parts[2].toFloat();
    ctrl.B_cont = parts[3].toFloat();
    ctrl.C_cont = parts[4].toFloat();
    ctrl.D_cont = parts[5].toFloat();
    ctrl.E_cont = parts[6].toFloat();
    ctrl.F_cont = parts[7].toFloat();
    ctrl.G_cont = parts[8].toFloat();
    ctrl.H_cont = parts[9].toFloat();
    timing.delayintencional = parts[10].toInt();
    timing.tiemporeferencia = parts[11].toInt();
    sys.amplitudAuto = parts[12].toInt();
    sys.referenciaManual = parts[13].toInt();
    sys.offset = parts[14].toInt();
    sys.tiposenal = parts[15].toInt();
    sys.tab_activo = parts[16].toInt();
    ctrl.Kp = parts[17].toFloat();
    ctrl.Ki = parts[18].toFloat();
    ctrl.Kd = parts[19].toFloat();
    if (timing.delayintencional <= 1)
      timing.delayintencional = 1;
    inicializaVariables();
    timing.tiemporestart = millis();
  }
  
  if (sys.inicio == 1) {
    switch (sys.modooperacion) {
      case 0: //Inhabilitado
        analogWrite(pins.pwmPin, 0);
        delay(10);
        break;
      case 1: { //System Identification
        sys.referencia = GeneradorReferencia(sys.tiposenal);
        ctrl.controladorOUT = sys.referencia;
        ctrl.controladorOUT = constrain(ctrl.controladorOUT, 0, 255);
        if (timing.TiempoCicloPromedio != 0)
          meas.RPM = enc.encoderTicks / (float)timing.TiempoCicloPromedio * 70;
        else
          meas.RPM = 0;
        meas.RPMPromedio = RPMAvg.reading(meas.RPM);
        enc.encoderTicks = 0;
        ctrl.PWMOUT = ctrl.controladorOUT;
        analogWrite(pins.pwmPin, ctrl.PWMOUT);
        sys.medicion = meas.RPMPromedio;
        break;
      }
      case 2: { //Control Velocidad
        sys.referencia = GeneradorReferencia(sys.tiposenal);
        if (timing.TiempoCicloPromedio != 0)
          meas.RPM = enc.encoderTicks / (float)timing.TiempoCicloPromedio * 70;
        else
          meas.RPM = 0;
        meas.RPMPromedio = RPMAvg.reading(meas.RPM);
        enc.encoderTicks = 0;
        ctrl.error = sys.referencia - meas.RPMPromedio;
        if (sys.tab_activo == 2)
          ctrl.controladorOUT = PID();
        else if (sys.tab_activo == 4)
          ctrl.controladorOUT = EcuacionDiferencias();
        ctrl.PWMOUT = ctrl.controladorOUT;
        constrain(ctrl.PWMOUT,0,255);
        analogWrite(pins.pwmPin, ctrl.PWMOUT);
        sys.medicion = meas.RPMPromedio;
        break;
      }
      case 3: { //Control Posición
        sys.referencia = GeneradorReferencia(sys.tiposenal);
        ctrl.error = sys.referencia - enc.encoderTicks;
        ctrl.controladorOUT = EcuacionDiferencias();
        if (ctrl.error >= 0) {
          digitalWrite(pins.in1Pin, HIGH);
          digitalWrite(pins.in2Pin, LOW);
        } else {
          digitalWrite(pins.in1Pin, LOW);
          digitalWrite(pins.in2Pin, HIGH);
        }
        delay(2);
        ctrl.controladorOUT = abs(ctrl.controladorOUT);
        analogWrite(pins.pwmPin, ctrl.controladorOUT);
        sys.medicion = enc.encoderTicks;
        break;
      }
    }
  } else {
    analogWrite(pins.pwmPin, 0);
    delay(10);
  }
  
  timing.tiempoanterior = timing.tiempo;
  timing.tiempo = millis() - timing.tiemporestart;
  timing.tiempociclo = timing.tiempo - timing.tiempoanterior;
  timing.TiempoCicloPromedio = TiempoCicloAvg.reading(timing.tiempociclo);

  if (sys.inicio == 1) {
    Serial.print(sys.referencia);
    Serial.print(" ");
    Serial.print(int(meas.RPMPromedio));
    Serial.print(" ");
    Serial.print(int(timing.TiempoCicloPromedio));
    Serial.print(" ");
    Serial.println(ctrl.PWMOUT);
  }
  delay(timing.delayintencional);
}
