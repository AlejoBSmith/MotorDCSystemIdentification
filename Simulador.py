import numpy as np
import control as ctrl
from scipy.optimize import curve_fit

# Generate example input-output data (or use your own experimental data)
Ts = 0.01  # Sampling time
time = np.arange(0, 10, Ts)  # Time vector
u = np.sin(2 * np.pi * 1 * time)  # Input signal
num = [1]  # True numerator coefficients of the system
den = [1, 2, 1]  # True denominator coefficients of the system
system = ctrl.TransferFunction(num, den)  # True system

# Simulate the output response
time, y = ctrl.forced_response(system, T=time, U=u)  # Only two values are returned

# Define a transfer function form to fit
def transfer_function_fit(t, k, tau):
    # First-order system: G(s) = k / (tau * s + 1)
    num = [k]
    den = [tau, 1]
    sys = ctrl.TransferFunction(num, den)
    _, yout = ctrl.forced_response(sys, T=t, U=u)  # Only two values are returned
    return yout

# Perform least-squares fitting
popt, _ = curve_fit(transfer_function_fit, time, y, p0=[1, 1])  # Initial guess for [k, tau]

# Extract fitted parameters
k_fitted, tau_fitted = popt
print(f"Fitted parameters: k = {k_fitted}, tau = {tau_fitted}")

# Construct the identified transfer function
identified_system = ctrl.TransferFunction([k_fitted], [tau_fitted, 1])
print(f"Identified Transfer Function:\n{identified_system}")
