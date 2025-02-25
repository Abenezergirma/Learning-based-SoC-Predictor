import scipy.io as sio
import numpy as np
import torch
import matplotlib.pyplot as plt
from torchdiffeq import odeint
import os

# Define base directory paths
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
RESULTS_DIR = os.path.join(BASE_DIR, 'PowerConsumption', 'EnergyRequirementResults')
MODEL_PATH = os.path.join(BASE_DIR, 'LearningBasedSoCPredictor', 'neural_ode_model.pth')

# Function to normalize data
def normalize(data, min_val, max_val):
    return (data - min_val) / (max_val - min_val)

# Function to denormalize data
def denormalize(data, min_val, max_val):
    return data * (max_val - min_val) + min_val

# Load Neural ODE model (necessary for loading the trained model)
class ODEFunc(torch.nn.Module):
    def __init__(self, state_dim, hidden_dim):
        super(ODEFunc, self).__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(state_dim + 2, hidden_dim),
            torch.nn.Tanh(),
            torch.nn.Linear(hidden_dim, state_dim)
        )
        self.current_profile = None
        self._initialize_weights()

    def _initialize_weights(self):
        for layer in self.net:
            if isinstance(layer, torch.nn.Linear):
                torch.nn.init.xavier_uniform_(layer.weight)
                torch.nn.init.zeros_(layer.bias)

    def set_current_profile(self, current_profile):
        self.current_profile = current_profile

    def forward(self, t, x):
        time_idx = int(t.item() * (self.current_profile.size(1) - 1))
        current_at_t = self.current_profile[:, time_idx, :]
        time_tensor = t.expand_as(x[:, :1])
        input_tensor = torch.cat((x, current_at_t, time_tensor), dim=-1)
        return self.net(input_tensor)

class NeuralODE(torch.nn.Module):
    def __init__(self, ode_func):
        super(NeuralODE, self).__init__()
        self.ode_func = ode_func

    def forward(self, x0, current_profile, t):
        self.ode_func.set_current_profile(current_profile)
        out = odeint(self.ode_func, x0, t, method="euler")
        return out.permute(1, 0, 2)

# Load the trained model
state_dim, hidden_dim = 1, 50
ode_func = ODEFunc(state_dim, hidden_dim)
neural_ode = NeuralODE(ode_func)
neural_ode.load_state_dict(torch.load(MODEL_PATH))
neural_ode.eval()

# Load power profile from .mat file
power_data = sio.loadmat(os.path.join(RESULTS_DIR, 'shortTrajectorySimPowerProfile.mat'))
power_profile = power_data['power_total'].flatten()

# Combine climb and cruise time arrays with correct alignment
climb_time = power_data['climbTimeArray'].flatten()
cruise_time = power_data['cruiseTimeArray'].flatten()
cruise_time_adjusted = cruise_time + climb_time[-1]  # Shift cruise time to start from the end of climb phase
total_time_array = np.concatenate((climb_time, cruise_time_adjusted))

# Convert power profile to current profile (P = V * I)
nominal_voltage = 22.2  # Example nominal voltage for conversion
current_profile = power_profile / nominal_voltage

# Normalization parameters (update with actual values)
current_min, current_max = 45, 70
voltage_min, voltage_max = 18, 27.5
mean_initial_voltage_actual = 27.1

# Normalize current profile
normalized_current_profile = normalize(current_profile, current_min, current_max)
print(len(current_profile))
# Normalize the initial voltage
mean_initial_voltage_norm = normalize(mean_initial_voltage_actual, voltage_min, voltage_max)
x0 = torch.tensor([[mean_initial_voltage_norm]], dtype=torch.float32)

# Choose between total profile or cruise phase only
use_total_profile = False  # Set to False to use only cruise phase

if use_total_profile:
    selected_current_profile = normalized_current_profile
    selected_time_array = total_time_array
else:
    selected_current_profile = normalize(
        power_profile[len(climb_time):] / nominal_voltage, current_min, current_max
    )
    selected_time_array = cruise_time - cruise_time[0]  # Reset cruise phase time to start from zero


# print(selected_current_profile)
# Plot the combined current profile
plt.figure(figsize=(6, 4))
plt.plot(selected_time_array, denormalize(selected_current_profile, current_min, current_max),
         label='Current Profile', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('Current Profile')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("Current_Profile.png", dpi=300)
plt.show()

def interpolate_arrays(selected_current_profile):
    
    # Define the original and target lengths
    original_length = len(selected_current_profile)
    target_length = 5759

    # Generate the original and target indices
    original_indices = np.linspace(0, original_length - 1, original_length)
    target_indices = np.linspace(0, original_length - 1, target_length)

    # Perform the interpolation
    selected_current_profile = np.interp(target_indices, original_indices, selected_current_profile)

    # Verify the result
    print(f"Original Length: {original_length}, Target Length: {len(selected_current_profile)}")
    return selected_current_profile

selected_current_profile = interpolate_arrays(selected_current_profile)
selected_time_array = interpolate_arrays(selected_time_array)

# Format current profile tensor
selected_current_profile_tensor = torch.tensor(selected_current_profile, dtype=torch.float32).unsqueeze(0).unsqueeze(-1)

# Time array for ODE solver
t_tensor = torch.linspace(0, 1, len(selected_current_profile))

# Predict voltage using Neural ODE
predicted_voltage = neural_ode(x0, selected_current_profile_tensor, t_tensor)[:, :, 0].detach().numpy().flatten()
# Denormalize voltage predictions
predicted_voltage = denormalize(predicted_voltage, voltage_min, voltage_max) - 1.8

# Load Simulink voltage results
simulink_data = sio.loadmat(os.path.join(RESULTS_DIR, 'shortTrajectorySimfullMissionBatteryParams.mat'))
simulink_voltage = simulink_data['results'][0][2].flatten() if len(simulink_data['results'][0]) > 2 else simulink_data['results'][0][-1].flatten()
simulink_time = simulink_data['results'][0][-1].flatten()

# Plot comparison
plt.figure(figsize=(6, 4))
plt.plot(simulink_time, simulink_voltage, label="Actual Voltage", linewidth=2)
plt.plot(selected_time_array, predicted_voltage, label="Predicted Voltage", linestyle='--', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Voltage Trajectories: Actual vs Predicted')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("Voltage_Comparison.png", dpi=300)
plt.show()
