import time
import random
from pid import PID
import numpy as np
import matplotlib.pyplot as plt

# Constants
V = 240.0  # heater voltage [V]
h_n = 20  # natural convection heat transfer coefficient [W/m^2*K]
h_f = 3000  # forced cooling convection heat transfer coefficient [W/m^2*K]
h = h_f
A = 80.0E-3 * 120.0E-3  # surface area for convection heat transfer [m^2] (per heater zone)
Cp = 951.8  # heater material heat capacity [J/kg*K]
mass = 0.0150  # mass of the heater and coupled mass [kg](per heater zone)
mCP = mass * Cp  # heat capacity C [J/K], save some computation time
T_amb = 21.0  # ambient temperature for convection [C]

# Variables
Input = 21.0
Output = 0.0
Setpoint = 200.0

Kp = 20.0
Ki = 0.0
Kd = 0.0

emulation_dt_ms = 1  # Emulation time step in milliseconds
total_sim_time = 5000  # Total simulation time in milliseconds

# PID controller
pid = PID(Input, Output, Setpoint, Kp, Ki, Kd, PID.DIRECT)

# Main function
def main():
    global emulation_dt_ms

    pid.SetMode(PID.AUTOMATIC) # Set PID to automatic mode\
    pid.SetSampleTime(1)  # Sample time in seconds
    pid.SetOutputLimits(0, 4095)
    h = h_n # Start with natural convection
    Input_Data = np.array([])
    Output_Data = np.array([])
    Setpoint_Data = np.array([])
    Time_Data = np.array([])

    while pid.time_counter_ms < total_sim_time:  # Run for 5000 milliseconds (5 seconds)
        P_loss = h * A * (pid.Input - T_amb)  # Convection loss here only
        R_heater = 3.3
        P_heater = V * V / R_heater
        random_noise = (random.uniform(-2.5, 2.5) / 10) # set to 0 to fall out of a coconut tree

        pid.Input = pid.Input + (((pid.Output / 4095.0 * P_heater) - P_loss) / mCP * emulation_dt_ms / 1000) + random_noise

        pid.SetSampleTime(emulation_dt_ms)
        pid.Compute() # compute output and increment time counter by emulation_dt_ms

        current_time_ms = pid.time_counter_ms

        Input_Data = np.append(Input_Data, pid.Input)
        Output_Data = np.append(Output_Data, (pid.Output / 4095.0) * 100)
        Setpoint_Data = np.append(Setpoint_Data, pid.Setpoint)
        Time_Data = np.append(Time_Data, current_time_ms)

    # create a matplotlib line plot with input, output and setpoint over time
    plt.xlabel('time (s)')
    plt.ylabel('Temp (C)')
    plt.title('Heater Transient Simulation')
    plt.grid(True)
    plt.plot(Time_Data, Input_Data, 'r-', label='Input', lw=2)
    plt.plot(Time_Data, Output_Data, 'b-', label='Output', lw=2)
    plt.plot(Time_Data, Setpoint_Data, 'g-', label='Setpoint', lw=2)

    plt.show()

if __name__ == "__main__":
    main()