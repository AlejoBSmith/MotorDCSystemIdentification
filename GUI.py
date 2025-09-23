import sys
import pyqtgraph as pg
import serial
import serial.tools.list_ports
import numpy as np
import pandas as pd
import control as ctrl
from datetime import datetime
from collections import deque
from PyQt6 import QtCore
from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QDialogButtonBox, QButtonGroup, QMessageBox, QDialog, QLabel, QComboBox, QPushButton, QHBoxLayout, QVBoxLayout
from PyQt6.QtCore import QTimer
from scipy.optimize import curve_fit
from scipy.signal import cont2discrete

class PortSelectDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select COM Port")
        self._ports = []
        # Widgets
        self.label = QLabel("Choose the serial (COM) port to use:")
        self.combo = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)

        # Layout
        top = QVBoxLayout()
        row = QHBoxLayout()
        row.addWidget(self.combo)
        row.addWidget(self.btn_refresh)
        top.addWidget(self.label)
        top.addLayout(row)
        top.addWidget(self.buttons)
        self.setLayout(top)

        # Signals
        self.btn_refresh.clicked.connect(self.populate)
        self.buttons.accepted.connect(self.validate_and_accept)
        self.buttons.rejected.connect(self.reject)

        # Initial population
        self.populate()

    def populate(self):
        self.combo.clear()
        self._ports = list(serial.tools.list_ports.comports())
        for p in self._ports:
            # Show device and description for clarity
            self.combo.addItem(f"{p.device} — {p.description}", userData=p.device)

    def selected_port(self):
        idx = self.combo.currentIndex()
        if idx < 0:
            return None
        return self.combo.itemData(idx)

    def validate_and_accept(self):
        port = self.selected_port()
        if not port:
            QMessageBox.warning(self, "No port selected", "Please select a COM port or click Cancel to exit.")
            return
        self.accept()


class MyDialog(QtWidgets.QDialog):
    def __init__(self, port=None, parent=None):
        super(MyDialog, self).__init__(parent)
        uic.loadUi('untitled.ui', self)
        # Initialize the serial connection using the selected port
        self.serial_port = None
        if port:
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
            except Exception as e:
                QMessageBox.critical(self, "Serial error", f"Could not open port {port}:\n{e}")
                self.serial_port = None

        # Initialize data lists
        self.header_written = False
        datapoints = 1000
        self.dataRPM_setpoint = deque(maxlen=datapoints)
        self.dataRPM_measured = deque(maxlen=datapoints)
        self.dataPWM = deque(maxlen=datapoints)
        #self.tabWidget.setTabText(0, "Sys Ident")
        #self.tabWidget.setTabText(1, "Simulation")
        #self.tabWidget.setTabText(2, "PID")
        #self.tabWidget.setTabText(3, "Discretization")
        #self.tabWidget.setTabText(4, "Digital Controller")
        self.tabWidget.currentChanged.connect(self.tabChanged)

        self.reference.textChanged.connect(self.update_slider_from_line_edit)
        self.slider.valueChanged.connect(self.update_line_edit_from_slider)

        # Define mode_map at the class level
        self.mode_map = {
            "Disabled": "0",
            "System Identification": "1",
            "Speed Control": "2",
            "Position Control": "3",
            "Speed - manual": "4",
            "Position - manual": "5",
        }

        # Set placeholder texts (optional, but user-friendly)
        self.snum3.setPlaceholderText("s^3")
        self.snum2.setPlaceholderText("s^2")
        self.snum1.setPlaceholderText("s^1")
        self.snum0.setPlaceholderText("s^0")
        self.sden3.setPlaceholderText("s^3")
        self.sden2.setPlaceholderText("s^2")
        self.sden1.setPlaceholderText("s^1")
        self.sden0.setPlaceholderText("s^0")
        self.sampling_time.setPlaceholderText("Ts")

        self.sim_num3.setPlaceholderText("s^3")
        self.sim_num2.setPlaceholderText("s^2")
        self.sim_num1.setPlaceholderText("s^1")
        self.sim_num0.setPlaceholderText("s^0")
        self.sim_den3.setPlaceholderText("s^3")
        self.sim_den2.setPlaceholderText("s^2")
        self.sim_den1.setPlaceholderText("s^1")
        self.sim_den0.setPlaceholderText("s^0")

        self.datapoints.textChanged.connect(self.resize_deque)
        self.modooperacion.currentIndexChanged.connect(self.sendModeOperation)
        self.grupo_checkboxes = QButtonGroup(self)
        self.grupo_checkboxes.addButton(self.manualinput, 1)
        self.grupo_checkboxes.addButton(self.automaticinput, 0)
        self.grupo_checkboxes.setExclusive(True)
        self.automaticinput.setChecked(True)

        self.grupo_identificationdata = QButtonGroup(self)
        self.grupo_identificationdata.addButton(self.identdatagraph, 1)
        self.grupo_identificationdata.addButton(self.identdatafile, 0)
        self.grupo_identificationdata.setExclusive(True)
        self.identdatagraph.setChecked(True)

        # Define el botón de start/stop
        self.StartStop.clicked.connect(self.toggleStartStop)
        self.isRunning = False

        # Realiza la discretización
        self.discretize.clicked.connect(self.discretize_function)

        # Define botón de update parameters
        self.update_parameters.clicked.connect(self.toggleupdate_parameters)
        # Define botón de identificación de sistema
        self.identify.clicked.connect(self.identify_system)
        # Define botón de simulación
        self.simulation.clicked.connect(self.Simulate)

        # Ensure the placeholder widget is inside a layout
        # El widget para la gráfica había que meterlo dentro de un recipiente (layout)
        # El widget se llama RPM y aquí se asegura que tiene uno asociado
        placeholderLayoutRPM = self.RPM.parentWidget().layout()
        placeholderLayoutPWM = self.PWM.parentWidget().layout()
        placeholderLayoutforced_response = self.forced_response.parentWidget().layout()
        placeholderLayoutsim_response = self.sim_response.parentWidget().layout()

        self.graphWidgetRPM = pg.PlotWidget()
        self.graphWidgetRPM.setYRange(0, 120)
        placeholderLayoutRPM.replaceWidget(self.RPM, self.graphWidgetRPM)
        self.RPM.deleteLater()

        self.graphWidgetPWM = pg.PlotWidget()
        self.graphWidgetPWM.setYRange(0, 255)
        placeholderLayoutPWM.replaceWidget(self.PWM, self.graphWidgetPWM)
        self.PWM.deleteLater()

        self.graphWidgetforced_response = pg.PlotWidget()
        self.graphWidgetforced_response.setYRange(0, 1.2)
        placeholderLayoutforced_response.replaceWidget(self.forced_response, self.graphWidgetforced_response)
        self.forced_response.deleteLater()

        self.graphWidgetsim_response = pg.PlotWidget()
        self.graphWidgetsim_response.setYRange(0, 1.2)
        placeholderLayoutsim_response.replaceWidget(self.sim_response, self.graphWidgetsim_response)
        self.sim_response.deleteLater()

        # Initialize curves for fast updates
        self.curve_setpoint = self.graphWidgetRPM.plot(pen=pg.mkPen(color='b', width=3, style=QtCore.Qt.PenStyle.SolidLine))  # Blue line for setpoint
        self.curve_measured = self.graphWidgetRPM.plot(pen=pg.mkPen(color='r', width=3, style=QtCore.Qt.PenStyle.SolidLine))  # Red line for measured
        self.curve_pwm = self.graphWidgetPWM.plot(pen=pg.mkPen(color='g', width=3, style=QtCore.Qt.PenStyle.SolidLine))       # Green line for PWM

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
        self.C.setText("0")
        self.D.setText("0")
        self.E.setText("0")
        self.F.setText("0")
        self.G.setText("0")
        self.H.setText("0")
        self.deadzone.setText("0")
        self.offset.setText("0")
        self.serial_in.setText(" ")
        self.serial_out.setText(" ")
        self.tiemporeferencia.setText("3000")
        self.amplitude.setText("150")
        self.reference.setText("150")
        self.delay.setText("20")
        self.modooperacion.setCurrentIndex(0)
        self.Kp.setText("1.0")
        self.Ki.setText("0.01")
        self.Kd.setText("0")
        self.denorder.setText("1")
        self.numorder.setText("0")
        self.datapoints.setText("200")
        self.x_scale.setText("5")
        self.time_constant.setText("0")

        # Comienza todos los timers
        self.timer.start()
        self.timerHora.start()

    def Simulate(self):
        try:
            # Get the numerator and denominator coefficients from UI
            num = [
                float(self.sim_num3.text() or 0),
                float(self.sim_num2.text() or 0),
                float(self.sim_num1.text() or 0),
                float(self.sim_num0.text() or 0),
            ]
            den = [
                float(self.sim_den3.text() or 0),
                float(self.sim_den2.text() or 0),
                float(self.sim_den1.text() or 0),
                float(self.sim_den0.text() or 0),
            ]

            # Remove leading zeros to prevent errors
            num = [coef for coef in num if coef != 0] or [0]
            den = [coef for coef in den if coef != 0] or [1]

            # Create the transfer function
            sys_tf = ctrl.TransferFunction(num, den)

            # Get the user-defined time scale from x_scale
            try:
                t_max = float(self.x_scale.text()) if self.x_scale.text() else 5.0  # Default to 5 sec
                if t_max <= 0:
                    self.textBrowser.setText("Time scale must be greater than zero.")
                    return
            except ValueError:
                self.textBrowser.setText("Invalid time scale. Enter a numeric value.")
                return

            # Generate time vector
            t = np.linspace(0, t_max, 500)

            # Get the selected response type from UI
            response_type = self.sim_tiposenal.currentText().lower()

            # Compute the response
            if response_type == "step":
                t, y = ctrl.step_response(sys_tf, T=t)
            elif response_type == "impulse":
                t, y = ctrl.impulse_response(sys_tf, T=t)
            elif response_type == "ramp":
                t, y = ctrl.forced_response(sys_tf, T=t, U=t)
            else:
                self.textBrowser.setText("Invalid response type selected.")
                return

            # Clear and plot the new response
            self.graphWidgetsim_response.clear()
            self.graphWidgetsim_response.plot(t, y, pen='y', name=response_type.capitalize() + " Response")
            # Auto-scale Y-axis
            ymin, ymax = np.min(y), np.max(y)
            margin = (ymax - ymin) * 0.1 if ymax != ymin else 0.1  # Add 10% margin
            self.graphWidgetsim_response.setYRange(ymin - margin, ymax + margin)

        except Exception as e:
            self.textBrowser.setText(f"Error in transfer function simulation: {e}")

    def discretize_function(self):
        try:
            num = [
                float(self.snum3.text() or 0),
                float(self.snum2.text() or 0),
                float(self.snum1.text() or 0),
                float(self.snum0.text() or 0),
            ]
            den = [
                float(self.sden3.text() or 0),
                float(self.sden2.text() or 0),
                float(self.sden1.text() or 0),
                float(self.sden0.text() or 0),
            ]
            T_s = float(self.sampling_time.text())

            # Remove leading zeros
            num = [coef for coef in num if coef != 0] or [0]
            den = [coef for coef in den if coef != 0] or [1]

            method = self.method.currentText()

            system_discrete = cont2discrete((num, den), T_s, method)
            num_z, den_z, _ = system_discrete
            print("Result:", num, den, num_z, den_z)

            num_str = " + ".join([f"{coef:.5f}z^{len(num_z[0])-1-i}" for i, coef in enumerate(num_z[0])])
            den_str = " + ".join([f"{coef:.5f}z^{len(den_z)-1-i}" for i, coef in enumerate(den_z)])

            result = f"H(z) = ({num_str}) / ({den_str})"
            self.discretizationresult.setText(result)

        except Exception as e:
            self.discretizationresult.setText(f"Error: {e}")


    def resize_deque(self):
        datapoints = int(self.datapoints.text())
        self.dataRPM_setpoint = deque(list(self.dataRPM_setpoint)[-datapoints:], maxlen=datapoints)
        self.dataRPM_measured = deque(list(self.dataRPM_measured)[-datapoints:], maxlen=datapoints)
        self.dataPWM = deque(list(self.dataPWM)[-datapoints:], maxlen=datapoints)

    def sendModeOperation(self):
        selected_text = self.modooperacion.currentText()
        _ = self.mode_map.get(selected_text, "0")
        # value is sent on toggleupdate_parameters

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
        serial_txt = self.serial_in.text()
        reference = self.reference.text()
        delay =  self.delay.text()
        # Send the data
        tiporef = self.grupo_checkboxes.checkedId()
        tiposenal = self.tiposenal.currentIndex()
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
        deadzone = self.deadzone.text()
        time_constant = self.time_constant.text()
        PIDtype = self.PIDtype.currentIndex()
        self.SendData(StartStop, selected_mode, A, B, C, D, E, F, G, H, delay, tiemporeferencia, amplitudAuto, refernciaManual,offset,tiposenal,activetab,Kp,Ki,Kd,deadzone,time_constant,PIDtype)

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
            # Clear the previous results
            self.graphWidgetforced_response.clear()

            # Step 1: Extract input-output data from deques
            u = np.array(self.dataPWM, dtype=float)  # Input: PWM signal
            y = np.array(self.dataRPM_measured, dtype=float)  # Output: Measured RPM
            numorder = int(self.numorder.text())
            denorder = int(self.denorder.text())

            # Check if data is sufficient
            if len(u) < 10 or len(y) < 10:
                self.identificationresult.setText("Insufficient data for system identification.")
                return

            # Step 2: Generate time vector
            sampling_interval = self.timer.interval() / 1000.0  # Timer interval in seconds
            time = np.arange(len(u)) * sampling_interval

            # Step 3: Define a transfer function model with variable order
            def transfer_function_fit(t, *coefficients):
                # Separate coefficients for numerator and denominator
                num_coefficients = coefficients[:numorder + 1]
                den_coefficients = coefficients[numorder + 1:]

                # Ensure the denominator starts with 1 (common in transfer functions)
                den_coefficients = np.insert(den_coefficients, 0, 1.0)

                # Define the transfer function
                sys_tf = ctrl.TransferFunction(num_coefficients, den_coefficients)
                _, yout = ctrl.forced_response(sys_tf, T=t, U=u)
                return yout

            # Step 4: Set initial guesses for the coefficients
            initial_guess = [1.0] * (numorder + 1 + denorder)

            # Step 5: Perform least-squares fitting
            popt, _ = curve_fit(transfer_function_fit, time, y, p0=initial_guess)

            # Extract numerator and denominator coefficients
            num_fitted = popt[:numorder + 1]
            den_fitted = np.insert(popt[numorder + 1:], 0, 1.0)  # Denominator starts with 1

            # Step 6: Construct the identified transfer function
            identified_system = ctrl.TransferFunction(num_fitted, den_fitted)
            self.identificationresult.setText(f"Identified Transfer Function:\n{str(identified_system)}")

            # Optional: Plot the identified model's response
            _, y_identified = ctrl.forced_response(identified_system, T=time, U=u)

            # Calculate the range of the system response
            min_response = min(y_identified)
            max_response = max(y_identified)

            # Adjust the Y-axis of the graph to match the response range
            self.graphWidgetforced_response.setYRange(min_response * 0.9, max_response * 1.1)

            # Plot the system response
            self.graphWidgetforced_response.plot(time, y_identified, pen='y', name="Identified Response")
            self.graphWidgetforced_response.plot(time, y, pen='r', name="Measured Response")

        except Exception as e:
            self.textBrowser.setText(f"Error during system identification: {e}")

    def update_graph(self):
        try:
            # If there's no open serial port, skip quietly
            if self.serial_port is None or (hasattr(self.serial_port, "is_open") and not self.serial_port.is_open):
                return
            # --- Arduino Data Section ---
            setpoint_rpm = None
            measured_rpm = None
            pwm_value = None

            while self.serial_port.in_waiting:
                serial_in = self.serial_port.readline(64).decode('utf-8', errors='ignore').strip()
                self.serial_in.setText(serial_in)

                try:
                    values = list(map(float, serial_in.split()))
                except ValueError:
                    continue  # skip malformed line
                if len(values) >= 3:
                    setpoint_rpm = values[0]
                    measured_rpm = values[1]
                    pwm_value = values[-1]

                    self.dataRPM_setpoint.append(setpoint_rpm)
                    self.dataRPM_measured.append(measured_rpm)
                    self.dataPWM.append(pwm_value)

                    max_rpm = max(max(self.dataRPM_setpoint, default=0), max(self.dataRPM_measured, default=0))
                    max_pwm = max(self.dataPWM, default=0)
                    self.graphWidgetRPM.setYRange(0, max(1, max_rpm) * 1.1)
                    self.graphWidgetPWM.setYRange(0, max(1, max_pwm) * 1.1)

                    self.curve_setpoint.setData(self.dataRPM_setpoint)
                    self.curve_measured.setData(self.dataRPM_measured)
                    self.curve_pwm.setData(self.dataPWM)

            # --- Save to File Section ---
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

    def SendData(self, StartStop, selected_mode, A, B, C, D, E, F, G, H,delay,tiemporeferencia,amplitudAuto,referenciaManual,offset,tiposenal,activetab,Kp,Ki,Kd,deadzone,time_constant,PIDtype):
        data_string=f"{StartStop},{selected_mode},{A},{B},{C},{D},{E},{F},{G},{H},{delay},{tiemporeferencia},{amplitudAuto},{referenciaManual},{offset},{tiposenal},{activetab},{Kp},{Ki},{Kd},{deadzone},{time_constant},{PIDtype}"
        data_bytes = data_string.encode('utf-8')
        self.serial_out.setText(str(data_bytes))
        data_bytes = (data_string + '\n').encode('utf-8')
        if self.serial_port is not None and (not hasattr(self.serial_port, "is_open") or self.serial_port.is_open):
            try:
                self.serial_port.write(data_bytes)
            except Exception as e:
                # If writing fails, show once
                print(f"Serial write error: {e}")

    def update_slider_from_line_edit(self):
        try:
            value = int(self.reference.text())
        except ValueError:
            value = 0
        self.slider.setValue(value)

    def update_line_edit_from_slider(self):
        value = self.slider.value()
        self.reference.setText(str(value))


def main():
    app = QtWidgets.QApplication(sys.argv)
    # Show COM port selection dialog first
    port_dlg = PortSelectDialog()
    if port_dlg.exec() != QtWidgets.QDialog.DialogCode.Accepted:
        # User cancelled
        sys.exit(0)
    port = port_dlg.selected_port()

    dialog = MyDialog(port=port)
    dialog.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
