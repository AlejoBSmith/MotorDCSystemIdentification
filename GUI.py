import sys
import pyqtgraph as pg
import serial
import serial.tools.list_ports
import numpy as np
import pandas as pd
import control as ctrl
from datetime import datetime
from collections import deque
from PyQt6.QtCore import Qt
from PyQt6 import QtCore
from PyQt6 import QtWidgets, uic
from PyQt6.QtWidgets import QDialogButtonBox, QButtonGroup, QMessageBox, QDialog, QLabel, QComboBox, QPushButton, QHBoxLayout, QVBoxLayout
from PyQt6.QtCore import QTimer
from scipy.optimize import curve_fit
from scipy.signal import cont2discrete

KNOWN_VIDPID = {
    (0x16C0, 0x0483),  # Teensy 4.x USB-Serial (PJRC)
    (0x2341, 0x006D),  # Arduino UNO R4 WiFi
    (0x2341, 0x006C),  # Arduino UNO R4 Minima
}
PREF_DESCR = ("Teensy", "UNO R4")  # optional: bias by description text

def auto_find_port():
    ports = list(serial.tools.list_ports.comports())
    # Prefer matches by VID/PID, then by description, else give up
    for p in ports:
        if (p.vid, p.pid) in KNOWN_VIDPID:
            return p.device
    for p in ports:
        if any(k in (p.description or "") for k in PREF_DESCR):
            return p.device
    return None

class PortSelectDialog(QDialog):
    def __init__(self, initial_port=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select COM Port")
        self.initial_port = initial_port
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
            self.combo.addItem(f"{p.device} — {p.description}", userData=p.device)
        # preselect previous choice if still present
        if self.initial_port is not None:
            idx = self.combo.findData(self.initial_port)
            if idx != -1:
                self.combo.setCurrentIndex(idx)

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
        uic.loadUi('QtDesignerGUI.ui', self)  # load the UI first
        # ---- make the dialog look/behave like a normal resizable window ----
        flags = self.windowFlags()
        flags |= Qt.WindowType.WindowSystemMenuHint          # show system menu
        flags |= Qt.WindowType.WindowMinMaxButtonsHint       # add Min/Max buttons
        flags &= ~Qt.WindowType.WindowContextHelpButtonHint  # remove the "?" button
        self.setWindowFlags(flags)

        # allow resizing even if the .ui layout had "SetFixedSize"
        if self.layout() is not None:
            self.layout().setSizeConstraint(
                QtWidgets.QLayout.SizeConstraint.SetDefaultConstraint
            )

        # optional but useful
        self.setSizeGripEnabled(True)      # corner size grip on some platforms
        self.setMinimumSize(900, 600)      # pick a sane minimum so content isn’t cramped

        base_title = "Motor DC Control - By A Von Chong"
        if port:
            self.setWindowTitle(f"{base_title} — {port}")
        else:
            self.setWindowTitle(f"{base_title} — no port")
        self.serial_port = None
        if port:
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
            except Exception as e:
                QMessageBox.critical(self, "Serial error",
                                     f"Could not open port {port}:\n{e}")
                self.serial_port = None

        # Initialize data lists
        self.header_written = False
        self.reference.setVisible(False)
        self.reference_label.setVisible(False)
        self.slider.setVisible(False)
        self.slider_label.setVisible(False)
        self.manualinput.setVisible(False)
        self.automaticinput.setVisible(False)

        datapoints = 1000
        self.dataRPM_setpoint = deque(maxlen=datapoints)
        self.dataRPM_measured = deque(maxlen=datapoints)
        self.dataPWM = deque(maxlen=datapoints)
        self.dataDT = deque(maxlen=datapoints)   # ms per sample
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
        self.identdatagraph.setVisible(False)
        self.identdatafile.setVisible(False)
        self.identdata.setVisible(False)

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

        self.legendRPM  = self.graphWidgetRPM.addLegend()
        pi = self.graphWidgetRPM.getPlotItem()
        self.legendRPM = pi.addLegend()
        self.legendRPM.setParentItem(pi.vb)
        self.legendRPM.anchor(itemPos=(1,0), parentPos=(1,0), offset=(10, -10))

        pi = self.graphWidgetRPM.getPlotItem()
        pi.setLabel('left',   'Motor Input / Output', units='PWM / RPM')
        self.graphWidgetRPM.setYRange(0, 120)
        placeholderLayoutRPM.replaceWidget(self.RPM, self.graphWidgetRPM)
        self.RPM.deleteLater()

        self.graphWidgetPWM = pg.PlotWidget()
        self.legendPWM  = self.graphWidgetPWM.addLegend()
        piSETPOINT = self.graphWidgetPWM.getPlotItem()
        self.legendPWM = piSETPOINT.addLegend()
        self.legendPWM.setParentItem(piSETPOINT.vb)
        self.legendPWM.anchor(itemPos=(1,0), parentPos=(1,0), offset=(10, -10))
        pi = self.graphWidgetPWM.getPlotItem()
        pi.setLabel('left',   'uController output', units='PWM')
        pi.setLabel('bottom', 'Time',  units='s') 
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
        self.curve_setpoint = self.graphWidgetRPM.plot(name="Input", pen=pg.mkPen(color='b', width=3, style=QtCore.Qt.PenStyle.SolidLine))  # Blue line for setpoint
        self.curve_measured = self.graphWidgetRPM.plot(name="Output", pen=pg.mkPen(color='r', width=3, style=QtCore.Qt.PenStyle.SolidLine))  # Red line for measured
        self.curve_pwm = self.graphWidgetPWM.plot(name="PWM", pen=pg.mkPen(color='g', width=3, style=QtCore.Qt.PenStyle.SolidLine))       # Green line for PWM

        # Disconnect the standard dialog accept/reject slots
        # Esto se hace para quitarle los valores por default que tienen los botones de
        # Ok y cancel (que es cerrar la ventana)

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
        self.Kp.setText("0")
        self.Ki.setText("0")
        self.Kd.setText("0")
        self.denorder.setText("1")
        self.numorder.setText("0")
        self.datapoints.setText("200")
        self.x_scale.setText("5")
        self.time_constant.setText("0.2")
        self.reset_time.setText("0.5")

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

        def _strip_leading_zeros(coefs, eps=1e-12):
            # input is [s^n, s^(n-1), ..., s^0]; remove zeros from the front
            c = list(map(float, coefs))
            while c and abs(c[0]) < eps:
                c.pop(0)
            return c if c else [0.0]

        def _poly_to_zinv_str(c, fmt='{:.5g}', eps=1e-9):
            # c = [c0, c1, c2, ...] -> c0 + c1 z^-1 + c2 z^-2 + ...
            parts = []
            for k, ck in enumerate(np.asarray(c, float)):
                if abs(ck) < eps:
                    continue
                if k == 0:
                    parts.append(fmt.format(ck))
                else:
                    parts.append(f"{fmt.format(ck)} z^-{k}")
            s = " + ".join(parts) if parts else "0"
            return s.replace("+ -", "- ")

        try:
            # Read S-domain polynomials (entered as s^3,s^2,s^1,s^0)
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
            T_s   = float(self.sampling_time.text())
            method = self.method.currentText()  # e.g., 'zoh', 'foh', 'bilinear' (tustin)

            # Clean leading zeros (highest powers)
            num = _strip_leading_zeros(num)
            den = _strip_leading_zeros(den)

            # Discretize (SciPy returns TF in z^-1)
            num_z, den_z, _ = cont2discrete((num, den), T_s, method)
            num_z = np.ravel(num_z).astype(float)
            den_z = np.ravel(den_z).astype(float)

            # Normalize so a0 == 1
            if abs(den_z[0]) > 0:
                num_z /= den_z[0]
                den_z /= den_z[0]

            # Pretty string in z^-1 form
            num_str = _poly_to_zinv_str(num_z)
            den_str = _poly_to_zinv_str(den_z)
            result  = f"H(z) = ({num_str}) / ({den_str}),  Ts = {T_s:g} s"

            self.discretizationresult.setText(result)

        except Exception as e:
            self.discretizationresult.setText(f"Error: {e}")
    
    def resize_deque(self):
        datapoints = int(self.datapoints.text())
        self.dataRPM_setpoint = deque(list(self.dataRPM_setpoint)[-datapoints:], maxlen=datapoints)
        self.dataRPM_measured = deque(list(self.dataRPM_measured)[-datapoints:], maxlen=datapoints)
        self.dataPWM = deque(list(self.dataPWM)[-datapoints:], maxlen=datapoints)
        self.dataDT = deque(list(self.dataDT)[-datapoints:], maxlen=datapoints)

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
        tiporef = int(self.automaticinput.isChecked())
        tiposenal = self.tiposenal.currentIndex()
        selected_text = self.modooperacion.currentText()
        selected_mode = self.mode_map.get(selected_text, "0")
        tiemporeferencia = self.tiemporeferencia.text()
        amplitud = self.amplitude.text()
        referenciaManual = self.reference.text()
        offset = self.offset.text()
        activetab = self.tabWidget.currentIndex()
        Kp = self.Kp.text()
        Ki = self.Ki.text()
        Kd = self.Kd.text()
        deadzone = self.deadzone.text()
        time_constant = self.time_constant.text()
        PIDtype = self.PIDtype.currentIndex()
        reset_time = self.reset_time.text()
        self.SendData(StartStop, selected_mode, A, B, C, D, E, F, G, H, delay, tiemporeferencia, amplitud, referenciaManual,offset,tiposenal,activetab,Kp,Ki,Kd,deadzone,time_constant,PIDtype,tiporef,reset_time)
        if selected_text == "Disabled":
            self.textBrowser.setText("System disabled, select an operation mode before starting")
        if selected_text != "Disabled":
            self.textBrowser.setText("")
        if selected_mode == "0" or selected_mode == "1" :
            pi = self.graphWidgetRPM.getPlotItem()
            pi.setLabel('left', 'Motor Input / Output', units='PWM / RPM')
        if selected_mode == "2":
            pi = self.graphWidgetRPM.getPlotItem()
            pi.setLabel('left', 'Motor Speed', units='RPM')
        if selected_mode == "3":
            pi = self.graphWidgetRPM.getPlotItem()
            pi.setLabel('left', 'Motor Position', units='degrees')

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
            # Clear previous results/plot
            self.graphWidgetforced_response.clear()

            # ---- 1) Grab data from deques ----
            u  = np.array(self.dataPWM, dtype=float)             # input (PWM)
            y  = np.array(self.dataRPM_measured, dtype=float)    # output (RPM)
            dt = np.array(self.dataDT, dtype=float) / 1000.0     # ms -> s

            numorder = int(self.numorder.text())
            denorder = int(self.denorder.text())

            # Basic sanity checks
            if min(len(u), len(y), len(dt)) < 10:
                self.identificationresult.setText("Insufficient data for system identification.")
                return

            # Use the most recent overlapping window
            n = min(len(u), len(y), len(dt))
            u = u[-n:]; y = y[-n:]; dt = dt[-n:]

            # ---- 2) Build cumulative time from dt ----
            time = np.empty(n, dtype=float)
            time[0] = 0.0
            if n > 1:
                time[1:] = np.cumsum(dt[:-1])

            # ---- 3) Make time uniform (lsim/forced_response requirement) ----
            # Use median Ts and interpolate u,y to that grid
            if n > 2:
                Ts = float(np.median(np.diff(time)))
            else:
                Ts = float(dt.mean()) if n > 1 else 1e-3

            # If jitter is small, snap; otherwise interpolate to a uniform grid
            diffs = np.diff(time) if n > 1 else np.array([Ts])
            if np.max(np.abs(diffs - Ts)) <= 0.05 * Ts:
                # Snap to perfect grid
                N = n
                time = np.arange(N, dtype=float) * Ts
            else:
                # Interpolate to uniform grid covering the same duration
                N = int(np.round(time[-1] / Ts)) + 1
                time_uniform = np.arange(N, dtype=float) * Ts
                u = np.interp(time_uniform, time, u)
                y = np.interp(time_uniform, time, y)
                time = time_uniform

            # ---- 4) Model used by curve_fit ----
            def transfer_function_fit(t, *coefficients):
                # coefficients = [b0, b1, ..., a1, a2, ...]  (denominator leading 1)
                num_coeffs = coefficients[:numorder + 1]
                den_tail   = coefficients[numorder + 1:]
                den_coeffs = np.insert(den_tail, 0, 1.0)

                sys_tf = ctrl.TransferFunction(num_coeffs, den_coeffs)
                # Simulate with same input/ time vectors used in curve_fit
                _, yout = ctrl.forced_response(sys_tf, T=t, U=u)
                return yout

            # ---- 5) Initial guess and fit ----
            initial_guess = [1.0] * (numorder + 1 + denorder)
            popt, _ = curve_fit(transfer_function_fit, time, y, p0=initial_guess, maxfev=5000)

            # ---- 6) Build identified TF and show it ----
            num_fitted = popt[:numorder + 1]
            den_fitted = np.insert(popt[numorder + 1:], 0, 1.0)
            identified_system = ctrl.TransferFunction(num_fitted, den_fitted)
            self.identificationresult.setText(f"Identified Transfer Function:\n{identified_system}")

            # ---- 7) Plot response vs measured ----
            _, y_identified = ctrl.forced_response(identified_system, T=time, U=u)

            y_min = float(min(np.min(y_identified), np.min(y)))
            y_max = float(max(np.max(y_identified), np.max(y)))
            if y_max - y_min < 1e-6:
                y_min, y_max = y_min - 1.0, y_max + 1.0

            self.graphWidgetforced_response.setYRange(y_min * 0.95, y_max * 1.05)
            self.graphWidgetforced_response.plot(time, y_identified, pen='y', name="Identified Response")
            self.graphWidgetforced_response.plot(time, y,           pen='r', name="Measured Response")

            # --- Fit metrics ---
            residuals = y - y_identified
            N = len(y)
            rmse = float(np.sqrt(np.mean(residuals**2)))
            mae  = float(np.mean(np.abs(residuals)))
            yvar = float(np.var(y))
            r2   = float(1.0 - np.sum(residuals**2) / np.sum((y - np.mean(y))**2)) if N > 1 else float('nan')
            nrmse = float(rmse / (np.sqrt(yvar) if yvar > 0 else 1.0))

            self.rmse.setText(f"{rmse:.2f}")
            self.mae.setText(f"{mae:.2f}")
            self.r2.setText(f"{r2:.2f}")
            self.nrmse.setText(f"{nrmse:.2f}")

            # Append to the GUI text
            #txt = self.identificationresult.toPlainText()
            #txt += f"\nFit metrics:\n  RMSE = {rmse:.3g}\n  MAE = {mae:.3g}\n  R² = {r2:.3f}\n  NRMSE = {nrmse:.3f}"
            #self.identificationresult.setText(txt)

            # (Optional) residuals plot overlay
            # self.graphWidgetforced_response.plot(time, residuals, pen='w', name='Residuals')

        except Exception as e:
            self.textBrowser.setText(f"Error during system identification: {e}")


    def update_graph(self):
        try:
            if self.serial_port is None or (hasattr(self.serial_port, "is_open") and not self.serial_port.is_open):
                return

            # Create file header once when saving is enabled
            if self.saveValuesCheckBox.isChecked() and not getattr(self, "header_written", False):
                from datetime import datetime
                mode_text = self.modooperacion.currentText()
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                header = f"Mode: {mode_text}\nDate: {now}\nREF,MEAS,DT_ms,CURR,PWM\n"
                with open('data.txt', 'w', newline='') as f:
                    f.write(header)
                self.header_written = True

            # Read & process all available lines
            while self.serial_port.in_waiting:
                line = self.serial_port.readline(128).decode('utf-8', errors='ignore').strip()
                self.serial_in.setText(line)

                try:
                    vals = [float(x) for x in line.split()]
                except ValueError:
                    continue
                if len(vals) < 5:
                    continue

                sp, meas, dt_ms, curr, pwm = vals[0], vals[1], vals[2], vals[3], vals[-1]

                # Append to buffers (all with the same maxlen)
                self.dataRPM_setpoint.append(sp)
                self.dataRPM_measured.append(meas)
                self.dataPWM.append(pwm)
                self.dataDT.append(dt_ms)

                # Save this sample if requested
                if self.saveValuesCheckBox.isChecked():
                    with open('data.txt', 'a', newline='') as f:
                        f.write(f"{int(sp)},{int(meas)},{int(dt_ms)},{int(curr)},{int(pwm)}\n")

            # ----- Plot latest window (equal-length slices) -----
            N = min(len(self.dataDT), len(self.dataRPM_setpoint), len(self.dataRPM_measured), len(self.dataPWM))
            if N == 0:
                return

            dt_s = np.asarray(self.dataDT, dtype=float)[-N:] * 1e-3
            t = np.cumsum(dt_s); t -= t[0]

            y_sp  = np.asarray(self.dataRPM_setpoint, dtype=float)[-N:]
            y_mea = np.asarray(self.dataRPM_measured, dtype=float)[-N:]
            y_pwm = np.asarray(self.dataPWM, dtype=float)[-N:]

            self.curve_setpoint.setData(t, y_sp)
            self.curve_measured.setData(t, y_mea)
            self.curve_pwm.setData(t, y_pwm)

            if "Position" in self.modooperacion.currentText():
                ymin = float(min(y_sp.min(initial=0), y_mea.min(initial=0)))
                ymax = float(max(y_sp.max(initial=0), y_mea.max(initial=0)))
                self.graphWidgetRPM.setYRange(ymin*1.1, ymax*1.1)
            else:
                self.graphWidgetRPM.setYRange(0, max(1.0, float(max(y_sp.max(initial=0), y_mea.max(initial=0)))) * 1.1)
            self.graphWidgetPWM.setYRange(0, max(1.0, float(y_pwm.max(initial=0))) * 1.1)

        except Exception as e:
            print("update_graph error:", e)

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

    def SendData(self, StartStop, selected_mode, A, B, C, D, E, F, G, H,delay,tiemporeferencia,amplitud,referenciaManual,offset,tiposenal,activetab,Kp,Ki,Kd,deadzone,time_constant,PIDtype,tiporef,reset_time):
        data_string=f"{StartStop},{selected_mode},{A},{B},{C},{D},{E},{F},{G},{H},{delay},{tiemporeferencia},{amplitud},{referenciaManual},{offset},{tiposenal},{activetab},{Kp},{Ki},{Kd},{deadzone},{time_constant},{PIDtype},{tiporef},{reset_time}"
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
        if self.manualinput.isChecked():
            self.toggleupdate_parameters()

    def update_line_edit_from_slider(self):
        value = self.slider.value()
        self.reference.setText(str(value))
        if self.manualinput.isChecked():
            self.toggleupdate_parameters()


def main():
    app = QtWidgets.QApplication(sys.argv)

    port = auto_find_port()
    while True:
        if not port:
            sel = PortSelectDialog()
            if sel.exec() != QtWidgets.QDialog.DialogCode.Accepted:
                sys.exit(0)
            port = sel.selected_port()

        dlg = MyDialog(port=port)
        if dlg.serial_port is not None:   # opened OK in your constructor
            dlg.setWindowTitle(f"Motor DC Control — {port} - By A Von Chong")
            dlg.show()
            sys.exit(app.exec())

        QtWidgets.QMessageBox.warning(None, "Serial",
                                      f"Couldn't open {port}. Close other apps or pick another.")
        port = None  # force dialog on next loop

if __name__ == "__main__":
    main()
