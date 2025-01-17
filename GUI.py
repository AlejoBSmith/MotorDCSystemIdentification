import sys
import random
import pyqtgraph as pg
import serial
import numpy as np
import pandas as pd
from datetime import datetime
from collections import deque
from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QDialogButtonBox, QButtonGroup
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QTabWidget
from sysidentpy.model_structure_selection import FROLS
from sysidentpy.basis_function._basis_function import Polynomial
from sysidentpy.parameter_estimation import LeastSquares
from sysidentpy.metrics import root_relative_squared_error
from sysidentpy.utils.display_results import results

class MyDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(MyDialog, self).__init__(parent)
        uic.loadUi('untitled.ui', self)
        # Initialize the serial connection (change 'COM3' to your serial port and set the appropriate baud rate)
        self.serial_port = serial.Serial('COM10', 115200, timeout=1)
        # Initialize data lists
        self.dataRPM_setpoint = deque(maxlen=400)
        self.dataRPM_measured = deque(maxlen=400)
        self.dataPWM = deque(maxlen=400)
        self.tabWidget.setTabText(0, "Sys Ident")
        self.tabWidget.setTabText(1, "PID")
        self.tabWidget.setTabText(2, "Digital")
        self.tabWidget.currentChanged.connect(self.tabChanged)

        self.reference.textChanged.connect(self.update_slider_from_line_edit)
        self.slider.valueChanged.connect(self.update_line_edit_from_slider)

        # Define mode_map at the class level
        self.mode_map = {
            "Inhabilitado": "0",
            "Identificación de Sistema": "1",
            "Control de Velocidad": "2",
            "Control de Posición": "3",
            "Modo Manual Velocidad": "4",
            "Modo Manual Posición": "5",
        }

        self.modooperacion.currentIndexChanged.connect(self.sendModeOperation)
        self.grupo_checkboxes = QButtonGroup(self)
        self.grupo_checkboxes.addButton(self.manualinput, 1)
        self.grupo_checkboxes.addButton(self.automaticinput, 0)
        self.grupo_checkboxes.setExclusive(True)
        self.automaticinput.setChecked(True)

        self.tiposenal.addItem("PRBS", 0)  # Displayed text, user data = 0
        self.tiposenal.addItem("Square Wave", 1)  # Displayed text, user data = 0
        self.tiposenal.addItem("Sine Wave", 2)    # Displayed text, user data = 1
        self.tiposenal.addItem("Triangle Dead Zone Wave", 3)    # Displayed text, user data = 2
        self.tiposenal.addItem("Impulse", 4)    # Displayed text, user data = 2
        self.tiposenal.addItem("Chirp Signal", 5)    # Displayed text, user data = 2
        self.tiposenal.addItem("Exponential Decay", 6)    # Displayed text, user data = 2
        self.tiposenal.addItem("White Noise", 7)    # Displayed text, user data = 2

        # Define el botón de start/stop
        self.StartStop.clicked.connect(self.toggleStartStop)
        self.isRunning = False

        # Define botón de update parameters
        self.update_parameters.clicked.connect(self.toggleupdate_parameters)
        # Define botón de identificación de sistema
        self.identify.clicked.connect(self.identify_system)

        # Ensure the placeholder widget is inside a layout
        # El widget para la gráfica había que meterlo dentro de un recipiente (layout)
        # El widget se llama RPM y aquí se asegura que tiene uno asociado
        placeholderLayoutRPM = self.RPM.parentWidget().layout()
        placeholderLayoutPWM = self.PWM.parentWidget().layout()

        self.graphWidgetRPM = pg.PlotWidget()
        self.graphWidgetRPM.setYRange(0, 120)
        placeholderLayoutRPM.replaceWidget(self.RPM, self.graphWidgetRPM)
        self.RPM.deleteLater()

        self.graphWidgetPWM = pg.PlotWidget()
        self.graphWidgetPWM.setYRange(0, 255)
        placeholderLayoutPWM.replaceWidget(self.PWM, self.graphWidgetPWM)
        self.PWM.deleteLater()

        # Initialize curves for fast updates
        self.curve_setpoint = self.graphWidgetRPM.plot(pen='b')  # Blue line for setpoint
        self.curve_measured = self.graphWidgetRPM.plot(pen='r')  # Red line for measured
        self.curve_pwm = self.graphWidgetPWM.plot(pen='g')       # Green line for PWM


        # Disconnect the standard dialog accept/reject slots
        # Esto se hace para quitarle los valores por default que tienen los botones de
        # Ok y cancel (que es cerrar la ventana)
        self.buttonBox.button(QDialogButtonBox.StandardButton.Ok).clicked.disconnect()
        self.buttonBox.button(QDialogButtonBox.StandardButton.Cancel).clicked.disconnect()

        # Connect the buttons to custom slots
        self.buttonBox.button(QDialogButtonBox.StandardButton.Ok).clicked.connect(self.ok_button_clicked)
        self.buttonBox.button(QDialogButtonBox.StandardButton.Cancel).clicked.connect(self.cancel_button_clicked)

        # Setup the QTimer
        # Este es el timer para la rapidez de update de la hora
        self.timerHora = QTimer(self)
        self.timerHora.setInterval(1000)
        self.timerHora.timeout.connect(self.updateDateTime)

        # Este es el timer para la rapidez de update de la gráfica
        self.timer = QTimer(self)
        self.timer.setInterval(10)  # Adjust the interval as needed
        self.timer.timeout.connect(self.update_graph)

        # Se inicializan los valores
        self.A.setText("0")
        self.B.setText("0")
        self.C.setText("0.33877")
        self.D.setText("0")
        self.E.setText("1.295")
        self.F.setText("-0.5292")
        self.G.setText("0")
        self.H.setText("0")
        self.offset.setText("0")
        self.serial_in.setText(" ")
        self.serial_out.setText(" ")
        self.tiemporeferencia.setText("2000")
        self.amplitude.setText("100")
        self.reference.setText("100")
        self.delay.setText("15")
        self.modooperacion.setCurrentIndex(0)
        self.Kp.setText("1.0")
        self.Ki.setText("0.1")
        self.Kd.setText("0")
        
        # Comienza todos los timers
        self.timer.start()
        self.timerHora.start()

    def sendModeOperation(self):
        selected_text = self.modooperacion.currentText()
        selected_mode = self.mode_map.get(selected_text, "0")

    def tabChanged(self):
        self.toggleupdate_parameters()

    def toggleupdate_parameters(self):
        # Collect the current values from UI elements or class variables
        StartStop = '0' if self.StartStop.text()=="Start" else '1'
        A = self.A.text()
        B = self.B.text()
        C = self.C.text()
        D = self.D.text()
        E = self.E.text()
        F = self.F.text()
        G = self.G.text()
        H = self.H.text()
        serial = self.serial_in.text()
        reference = self.reference.text()
        delay =  self.delay.text()
        # Send the data
        tiporef = self.grupo_checkboxes.checkedId()
        tiposenal = self.tiposenal.currentData()
        selected_text = self.modooperacion.currentText()
        selected_mode = self.mode_map.get(selected_text, "0")
        tiemporeferencia = self.tiemporeferencia.text()
        amplitudAuto = self.amplitude.text()
        refernciaManual = self.reference.text()
        offset = self.offset.text()
        activetab = self.tabWidget.currentIndex()
        Kp = self.Kp.text()
        Ki = self.Ki.text()
        Kd = self.Kd.text()
        self.SendData(StartStop, selected_mode, A, B, C, D, E, F, G, H, delay, tiemporeferencia, amplitudAuto, refernciaManual,offset,tiposenal,activetab,Kp,Ki,Kd)

    def plot_simulation(self, t, y):
        self.graphWidgetRPM.plot(t, y, pen='y', name="Simulated (Threaded)")

    def toggleStartStop(self):
            if self.isRunning:
                self.StartStop.setText("Start")
                self.StopAction()
            else:
                self.StartStop.setText("Stop")
                self.StartAction()
            self.isRunning = not self.isRunning

    def identify_system(self):
        try:
            # Convert data to numpy arrays
            measured_rpm = np.array(self.dataRPM_measured, dtype=float).reshape(-1, 1)
            pwm_input = np.array(self.dataPWM, dtype=float).reshape(-1, 1)

            # Check if data is sufficient
            if measured_rpm.size < 10 or pwm_input.size < 10:
                self.identificationresult.setText("Insufficient data for system identification.")
                return

            # Split data into training and validation sets (70% train, 30% validation)
            split_idx = int(0.7 * len(pwm_input))
            x_train, x_valid = pwm_input[:split_idx], pwm_input[split_idx:]
            y_train, y_valid = measured_rpm[:split_idx], measured_rpm[split_idx:]

            # Define basis function and estimator
            basis_function = Polynomial(degree=2)
            estimator = LeastSquares()

            # Define the FROLS model
            model = FROLS(
                order_selection=True,
                n_info_values=3,
                ylag=2,
                xlag=2,
                info_criteria="aic",
                estimator=estimator,
                err_tol=None,
                basis_function=basis_function,
            )

            # Fit the model using the training data
            model.fit(X=x_train, y=y_train)

            # Predict on the validation data
            yhat = model.predict(X=x_valid, y=y_valid)

            # Calculate metrics (e.g., Root Relative Squared Error)
            rrse = root_relative_squared_error(y_valid, yhat)

            # Display identified model results
            results_df = pd.DataFrame(
                results(
                    model.final_model, model.theta, model.err,
                    model.n_terms, err_precision=8, dtype='sci'
                ),
                columns=['Regressors', 'Parameters', 'ERR']
            )

            # Display results in the GUI
            self.identificationresult.setText(f"Identified Model:\n{results_df}\n")
            self.identificationresult.append(f"Root Relative Squared Error (RRSE): {rrse}")

        except Exception as e:
            self.identificationresult.setText(f"Error during system identification: {e}")

    def update_graph(self):
        try:
            # --- Arduino Data Section (Unchanged) ---
            setpoint_rpm = None
            measured_rpm = None
            pwm_value = None

            while self.serial_port.in_waiting:
                serial_in = self.serial_port.readline(32).decode('utf-8').strip()
                self.serial_in.setText(serial_in)

                values = list(map(float, serial_in.split()))
                if len(values) >= 3:
                    setpoint_rpm = values[0]
                    measured_rpm = values[1]
                    pwm_value = values[-1]

                    self.dataRPM_setpoint.append(setpoint_rpm)
                    self.dataRPM_measured.append(measured_rpm)
                    self.dataPWM.append(pwm_value)

                    max_rpm = max(max(self.dataRPM_setpoint, default=0), max(self.dataRPM_measured, default=0))
                    max_pwm = max(self.dataPWM, default=0)
                    self.graphWidgetRPM.setYRange(0, max_rpm * 1.1)
                    self.graphWidgetPWM.setYRange(0, max_pwm * 1.1)

                    self.curve_setpoint.setData(self.dataRPM_setpoint)
                    self.curve_measured.setData(self.dataRPM_measured)
                    self.curve_pwm.setData(self.dataPWM)

            # --- Save to File Section (Unchanged) ---
            if self.saveValuesCheckBox.isChecked() and setpoint_rpm is not None:
                if not self.header_written:
                    mode_text = self.modooperacion.currentText()
                    now = datetime.now()
                    header = f"Modo: {mode_text}\nFecha: {now}\nReferencia, Medición, PWM\n"
                    with open('datos.txt', 'w') as file:
                        file.write(header)
                    self.header_written = True

                with open('datos.txt', 'a') as file:
                    file.write(f"{int(setpoint_rpm)},{int(measured_rpm)},{int(pwm_value)}\n")

        except Exception as e:
            print(f"Error: {e}")


    def ok_button_clicked(self):
        self.identificationresult.setText("Ok button pressed")

    def cancel_button_clicked(self):
        self.identificationresult.setText("Cancel button pressed")

    def updateDateTime(self):
        now = datetime.now()
        dateTimeString = now.strftime("%H:%M:%S\n%d-%m-%Y")
        self.dateTimeLabel.setText(dateTimeString)

    def StartAction(self):
        print("Inicio control motor")
        StartStop=b'1'
        self.toggleupdate_parameters()

    def StopAction(self):
        print("Paro de control de motor")
        StartStop=b'0'
        self.toggleupdate_parameters()

    def SendData(self, StartStop, selected_mode, A, B, C, D, E, F, G, H,delay,tiemporeferencia,amplitudAuto,referenciaManual,offset,tiposenal,activetab,Kp,Ki,Kd):
        data_string=f"{StartStop},{selected_mode},{A},{B},{C},{D},{E},{F},{G},{H},{delay},{tiemporeferencia},{amplitudAuto},{referenciaManual},{offset},{tiposenal},{activetab},{Kp},{Ki},{Kd}"
        data_bytes = data_string.encode('utf-8')
        self.serial_out.setText(str(data_bytes))
        self.serial_port.write(data_bytes)

    def update_slider_from_line_edit(self):
        value = int(self.reference.text())
        self.slider.setValue(value)

    def update_line_edit_from_slider(self):
        value = self.slider.value()
        self.reference.setText(str(value))


app = QtWidgets.QApplication(sys.argv)
dialog = MyDialog()
dialog.show()
sys.exit(app.exec())