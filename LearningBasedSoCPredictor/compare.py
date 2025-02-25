# Load test data
import pickle
import numpy as np 
import os
import torch
import torch.nn as nn
import torch.optim as optim
from torchdiffeq import odeint
from tqdm import tqdm
import matplotlib.pyplot as plt

# Function to normalize data
def normalize(data, min_val, max_val):
    return (data - min_val) / (max_val - min_val)

# Function to denormalize data
def denormalize(data, min_val, max_val):
    return data * (max_val - min_val) + min_val
with open("test_data.pkl", "rb") as f:
    test_data = pickle.load(f)

# Extract a single test input and output
test_input = test_data[0]["input"]  # Current profile
test_output = test_data[0]["output"]  # Voltage profile
current_min = min(test_input)
current_max = max(test_input)
voltage_min = min(test_output)
voltage_max = max(test_output)
mean_initial_voltage_actual = 27.1

# print(current_min)

# Normalize the test input and output
normalized_test_input = normalize(test_input, current_min, current_max)
normalized_test_output = normalize(test_output, voltage_min, voltage_max)

# Prepare time array
actual_time = np.arange(len(test_input)) * 0.1  # Assuming time step is 0.1 seconds
t_test = torch.linspace(0, 1, len(test_input))

# Normalize initial voltage
normalized_initial_voltage = normalize(mean_initial_voltage_actual, voltage_min, voltage_max)
x0_test = torch.tensor([[normalized_initial_voltage]], dtype=torch.float32)

# Format current profile tensor
test_current_profile_tensor = torch.tensor(normalized_test_input, dtype=torch.float32).unsqueeze(0).unsqueeze(-1)

# Compare shapes and lengths
print(f"Original Current Profile Shape: {test_input.shape}, Normalized Shape: {test_current_profile_tensor.shape}")
print(f"Original Time Array Length: {len(actual_time)}, Torch Time Array Length: {len(t_test)}")
print(f"Initial Voltage (Normalized): {normalized_initial_voltage}")
print(f"Normalized Current Profile Min/Max: {normalized_test_input.min()}/{normalized_test_input.max()}")
print(f"Test Output Length: {len(test_output)}, Normalized Output Length: {len(normalized_test_output)}")

# # Perform ODE integration
# predicted_test_voltage = neural_ode(x0_test, test_current_profile_tensor, t_test)[:, :, 0].detach().numpy().flatten()

# # Denormalize predictions
# denormalized_predicted_voltage = denormalize(predicted_test_voltage, voltage_min, voltage_max)
# denormalized_test_output = denormalize(normalized_test_output, voltage_min, voltage_max)

# # Plot comparison
# plt.figure(figsize=(10, 6))
# plt.plot(actual_time, denormalized_test_output, label="Ground Truth Voltage", linestyle="dashed", linewidth=2)
# plt.plot(actual_time, denormalized_predicted_voltage, label="Predicted Voltage", linestyle="--", linewidth=2)
# plt.xlabel("Time (s)")
# plt.ylabel("Voltage (V)")
# plt.title("Voltage Trajectories: Neural ODE vs Ground Truth")
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.savefig("Voltage_Comparison_Test.png", dpi=300)
# plt.show()
