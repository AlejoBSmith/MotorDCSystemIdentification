# Identificación de sistema para motor DC
Este repositorio es para alojar el código de base para el curso de Teoría de Control II.
En él hay dos archivos principales:

## MotorDCSystemIdentification/src/main.cpp
Código de Arduino para control de motor DC. El código es para uso con un L298N. Deben verificar los pines, que deben estar de acuerdo a lo que dice el código.

## MotorDCSystemIdentification/GuardaDatos.py
Código para guardar en un archivo de texto los datos de salida del Arduino. Este archivo es necesario para generar la data que se usará para la identificación de sistema.

## Carpeta Ejemplos
Se incluyen ejemplos del proceso completo de diseño e implementación del controlador. En los datos se encuentra el MotorDC.mat que contiene los datos obtenidos para el system identification. Con estos datos se identifica la planta y se muestra cuál es la TF escogida. Luego se diseñan dos controladores (se incluye captura del sistema en el control system designer) y se verifica su comportamiento (con mediciones reales) respecto a la simulación.

En el archivo DatosMotorDC.mat se encuentran todos los datos para la realización del proceso completo para llegar a los mismos resultados. Estos datos pueden usarse para realizar el proceso de diseño y simulación. Nótese que las ecuaciones de identificación de sistema y del controlador solo son válidas para el sistema que generó los datos y puede que para su sistema de motro dc no funcione igual (o no funcione para absolutamente nada).
