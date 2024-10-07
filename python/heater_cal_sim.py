R_heater_0 = 4.20 # Initial resistance of the heater at T_amb [Ohms]

alpha = 0.003 # Temperature coefficient of resistance [1/C]
T_amb = 43.23 # Ambient temperature for convection [C]

def get_res(T_current:float) -> float:
    return R_heater_0 * (1 + alpha * (T_current - T_amb))

# define two temperatures
T0 = 100
T1 = 200
# use the temperatures to get resistances
R0 = get_res(T0)
R1 = get_res(T1)

# use linear regression to get T as a fucntion of R
slope = (T1 - T0) / (R1 - R0)
intercept = T0 - slope * R0

print(f'M: {slope}, B: {intercept}')

# ORRRRR simplified mathmatically
# T = (1 / (R_heater_0 * alpha)) * R + (T_amb - (1 / alpha))
# WHERE
# M = 1 / (R_heater_0 * alpha)
# B = T_amb - (1 / alpha)
# work shown on a random lost-to-time piece of paper