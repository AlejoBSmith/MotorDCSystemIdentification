![Open Source Hardware Facts](oshw_facts.svg)

# OpenMCT – DC Motor Control Trainer (Teoría de Control I/II)

Este repositorio contiene el código y la GUI del kit OpenMCT para control con un motor DC: adquisición de datos, excitación, identificación de sistema y pruebas en lazo cerrado (PID y controladores en forma de ecuación en diferencias).

## Archivos del repositorio

### Firmware (Teensy / PlatformIO)
- **`src/main.cpp`**  
  Firmware para el microcontrolador (Teensy 4.x). Implementa:
  - lectura de encoder (posición/velocidad)
  - salida PWM al driver
  - medición de corriente (solo funciona si se usa el del kit con el DRV8874)
  - streaming por serial para visualización/registro
  - ejecución del controlador (PID y/o coeficientes en z)

- **`platformio.ini`**  
  Configuración del proyecto en PlatformIO (VS Code).

> **Importante**: La GUI se puede usar con otros controladores pero debe hacer la modificación de los pines en el main.cpp. Si no, no servirá.

### GUI (Python/Qt)
- **`GUI.py`**  
  Aplicación de escritorio para:
  - conectar por serial
  - ver gráficas de las señales
  - generar señales de excitación
  - registrar datos
  - ajustar/controlar parámetros (PID y coeficientes)
- **`QtDesignerGUI*.ui`**  
  Base de la interfaz hecha en Qt Designer.

---

## Requisitos

### Firmware
- VS Code + PlatformIO
- Teensy 4.x (o el target que tengas configurado)
- Driver + encoder conectados como dice el código

### GUI
Python 3.10+ recomendado.

Para instalar los paquetes para la interfaz, abre un powershell y corre:
> pip install PyQt6 pyqtgraph pyserial numpy pandas control scipy matplotlib

