import numpy as np
import matplotlib.pyplot as plt

# Constants
rho_0 = 1.225  # Air density (kg/m^3) at sea level 
rho_1 = 1.115  # Air density (kg/m^3) at 610m alt (for airport landing)
rho = 0.9093  # Atmospheric density (kg/m^3) at 3048m alt
rho_c = 0.54939382277  # Atmospheric density (kg/m^3) at 7620 m (25000 ft) alt service ceiling
sigma = rho / rho_0
sigma_c = rho_c / rho_0

W_S_range = np.linspace(1, 4000, 4000)  # Weight to surface N/m^2

CD0 = 0.0202  # Parasite drag coefficient
pi = 3.14159  # Pi
AR = 11  # Aspect ratio of the wing
e = 0.85  # Oswald efficiency number
K = 1 / (pi * e * AR)  # Some constant

# Propulsive efficiency
eta_p = 0.85 
eta_p_climb = 0.7 

# Speeds
V_c = 128.611  # Cruise speed 
V_max = 1.2 * V_c  # Maximum speed 
V_s = 50  # Max stall speed (not final)
V_TO = 1.1 * V_s  # Takeoff speed (not final)

g = 9.81  # Gravitational acceleration
Sto = 800  # Takeoff field length 
u = 0.05  # Friction coefficient
C_Lmax = 2  # Max lift coefficient 
C_LR = C_Lmax / (1.1**2)  # Take-off rotation lift coefficients
C_DG = 0.025  # Coefficient drag
L_D_max = 13  # Maximum lift to drag ratio
ROC = 4.73 / 2  # Minimum rate of climb
ROC_c = (4.73 / 2) / 260.8224354771  # Minimum rate of climb

# Power calculations

# Maximum speed power loading
W_Psl_Vmax = eta_p / (0.5 * rho_0 * (V_max**3) * CD0 * (1 / W_S_range) + (2 * K / (rho * sigma * V_max)) * W_S_range)

# Takeoff power loading
W_P_Sto = (1 - np.exp(0.6 * rho * g * C_DG * Sto * np.reciprocal(W_S_range))) / (u - (u + C_DG / C_LR) * (np.exp(0.6 * rho * g * C_DG * Sto * np.reciprocal(W_S_range)))) * (eta_p / V_TO)

# Climb power loading
W_P_ROC = 1 / ((ROC / eta_p_climb + (2 / (rho_0 * (3 * CD0 / K)**0.5) * W_S_range)**0.5) * (1.155 / (L_D_max * eta_p_climb)))

# Ceiling power loading
W_P_C = sigma_c / ((ROC_c / eta_p_climb + (2 / (rho_c * (3 * CD0 / K)**0.5) * W_S_range)**0.5) * (1.155 / (L_D_max * eta_p_climb)))

# Stall speed related calculations
W_S_Stall = 0.5 * rho_1 * (V_s**2) * C_Lmax
W_S_Stall_index = np.argmin(np.abs(W_S_range - W_S_Stall))


# Plotting
plt.figure()
plt.plot(W_S_range, W_Psl_Vmax, 'b', label='Max speed')
plt.plot(W_S_range, W_P_Sto, 'r', label='Takeoff')
plt.plot(W_S_range, W_P_ROC, 'purple', label='Climb')
plt.plot(W_S_range, W_P_C, 'turquoise', label='Ceiling')
plt.axvline(x=W_S_Stall, color='g', linestyle='--', label='Stall speed Limit')
plt.xlabel('W/S (N/m^2)')
plt.ylabel('W/P (N/W)')
plt.title('Performance Constraints diagram')
plt.ylim(0, 0.15)

# Display intersection points and feasible flight envelope
intersection_points = []

threshold = 0.00001  # Define a threshold for accepting intersection points

for i in range(len(W_S_range)):
    # Find the two loadings with the smallest difference
    diff_pairs = [
        (abs(W_Psl_Vmax[i] - W_P_Sto[i]), W_Psl_Vmax[i], W_P_Sto[i]),
        (abs(W_Psl_Vmax[i] - W_P_ROC[i]), W_Psl_Vmax[i], W_P_ROC[i]),
        (abs(W_Psl_Vmax[i] - W_P_C[i]), W_Psl_Vmax[i], W_P_C[i]),
        (abs(W_P_Sto[i] - W_P_ROC[i]), W_P_Sto[i], W_P_ROC[i]),
        (abs(W_P_Sto[i] - W_P_C[i]), W_P_Sto[i], W_P_C[i]),
        (abs(W_P_ROC[i] - W_P_C[i]), W_P_ROC[i], W_P_C[i])
    ]
    min_diff_pair = min(diff_pairs, key=lambda x: x[0])  # Find the pair with the minimum difference
    min_diff = min_diff_pair[0]
    min_loadings = min_diff_pair[1:]

    # Check if the minimum difference pair corresponds to the actual loadings
    if min(min_loadings) == min(W_Psl_Vmax[i], W_P_Sto[i], W_P_ROC[i], W_P_C[i]) and min_diff < threshold:
        intersection_points.append((W_S_range[i], min_loadings[0]))

# Plot the intersection points and feasible flight envelope
for point in intersection_points:
    plt.scatter(point[0], point[1], color='orange', label='Intersection')

#Loadings from graph
W_P_D = point[1]  #current wing loading from graph N/m^2 # Get the corresponding value of W_Psl_Vmax
W_S_D = point[0]  #current power loading from graph N/m^2 # Update W_S_D to the exact W/S value

# Results display
print('W_S_D (Wing Loading):', W_S_D)
print('W_P_D (Power Loading):', W_P_D)

# Takeoff weight calculation
P= 1500  # Current engine power in KW
W_TO = 1000 * P * W_P_D  # Takeoff weight in N
S = W_TO / W_S_D  # Current wing area in m^2   
M_TO = 1500 * 1000 * W_P_D / g  # Expected weight in kg 

WingSpan = (AR*S) **0.5
Chord = WingSpan/AR

# Results display
print('Calculated using the provided equation:')
print('S: ', S)
print('P: ', P)
print('MTO: ', M_TO)
print('Chord length: ', Chord)
print('wing span: ', WingSpan)

plt.fill_between(W_S_range, 0, W_Psl_Vmax, where=(W_S_range < W_S_D), color='yellow', alpha=0.3, label='Feasible flight envelope')
plt.fill_between(W_S_range, 0, W_P_Sto, where=((W_S_D < W_S_range) & (W_S_range < W_S_Stall)), color='yellow', alpha=0.3)
plt.legend()
plt.grid(True)
plt.show()
