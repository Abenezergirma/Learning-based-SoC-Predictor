import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm  # For progress monitoring
from scipy.io import loadmat  # For loading MATLAB files
from battery_model.battery_electrochem_TarotT18 import BatteryElectroChem as Battery

# Wrapper class for the battery model
class TarotBattery:
    def __init__(self, time_step, filename):
        self.battery = Battery()
        self.initial_state = self.battery.initialize(0, 0)  # Save initial battery state
        self.x = self.initial_state
        self.t = 0
        self.time_step = time_step
        self.f = open(filename, 'w')

    # Updated `step` function with current clipping and corrected time_step
    def step(self, currentNeeded, V_min=18, V_max=25.2, max_current=200):  # Set max_current based on battery specs
        self.t += self.time_step
        try:
            # Clip current to the maximum allowable value
            current_clipped = min(currentNeeded, max_current)
            current_input = {'i': current_clipped}  # Use clipped current in the input dictionary
            self.x = self.battery.next_state(self.x, current_input, self.time_step)
            output = self.battery.output(self.x)
            
            # Extract voltage and calculate SoC based on linear mapping
            voltage = output['v']
            soc = max(0, min(1, (voltage - V_min) / (V_max - V_min)))  # Clamp SoC to [0, 1] range
            
            # Write voltage to file (optional step for logging)
            self.f.write(f"{voltage}\n")
            return voltage, soc
        except (ValueError, KeyError) as e:
            print(f"Error during step execution: {e}. Skipping current trajectory.")
            return None, None

    # Function to generate training and test datasets
    def generate_dataset(self, current_input, num_training=70, num_test=30, scaling_range=(0.9, 1)):
        # Remove the first few elements to avoid high initial currents
        current_input = current_input[1:]
        
        # Undersample the original trajectory to match the desired time step of 0.1s
        current_input_undersampled = current_input
        print(f"Undersampled current trajectory to time step of 0.1s with length {len(current_input_undersampled)}")

        training_data = []
        test_data = []

        # Generate training trajectories with progress monitoring
        print("Generating training data...")
        for i in tqdm(range(num_training), desc="Training Trajectories"):
            scale_factor = np.random.uniform(*scaling_range)
            scaled_current = current_input_undersampled * scale_factor
            voltage, soc = self.simulate_battery(scaled_current)
            if voltage is not None and soc is not None:
                training_data.append((scaled_current, voltage, soc))
            else:
                # Reset the battery state if simulation fails
                self.x = self.initial_state
                self.t = 0

        # Generate test trajectories with progress monitoring
        print("Generating test data...")
        for i in tqdm(range(num_test), desc="Test Trajectories"):
            scale_factor = np.random.uniform(*scaling_range)
            scaled_current = current_input_undersampled * scale_factor
            voltage, soc = self.simulate_battery(scaled_current)
            if voltage is not None and soc is not None:
                test_data.append((scaled_current, voltage, soc))
            else:
                # Reset the battery state if simulation fails
                self.x = self.initial_state
                self.t = 0

        return training_data, test_data

    # Function to simulate battery response for a given current trajectory
    def simulate_battery(self, current_trajectory):
        voltage_trajectory = []
        soc_trajectory = []
        for current in tqdm(current_trajectory, desc="Simulating Trajectory", leave=False):
            voltage, soc = self.step(current)
            if voltage is None or soc is None:
                return None, None  # Skip this trajectory if an error occurred
            voltage_trajectory.append(voltage)
            soc_trajectory.append(soc)
        return np.array(voltage_trajectory), np.array(soc_trajectory)

    # Function to plot training and test datasets
    def plot_combined_results(self, training_data, test_data):
        fig, axes = plt.subplots(1, 3, figsize=(18, 5), dpi=120)  # 1x3 layout
        
        # Plot Training Data
        for i, (current, voltage, soc) in enumerate(training_data):
            time_steps = np.arange(len(current)) * self.time_step
            color = 'blue' if i == 0 else 'lightblue'
            axes[0].plot(time_steps, current, color=color, linewidth=0.8, alpha=0.6)
            axes[1].plot(time_steps, voltage, color=color, linewidth=0.8, alpha=0.6)
            axes[2].plot(time_steps, soc, color=color, linewidth=0.8, alpha=0.6)
        
        # Plot Test Data
        for i, (current, voltage, soc) in enumerate(test_data):
            time_steps = np.arange(len(current)) * self.time_step
            color = 'red' if i == 0 else 'lightcoral'
            axes[0].plot(time_steps, current, color=color, linewidth=0.8, alpha=0.6)
            axes[1].plot(time_steps, voltage, color=color, linewidth=0.8, alpha=0.6)
            axes[2].plot(time_steps, soc, color=color, linewidth=0.8, alpha=0.6)
        
        # Set titles and labels
        axes[0].set_title("Current Trajectories", fontsize=14)
        axes[0].set_ylabel("Current (A)", fontsize=12)
        axes[0].set_xlabel("Time (s)", fontsize=12)
        axes[0].grid(True, linestyle='--', alpha=0.7)
        
        axes[1].set_title("Voltage Responses", fontsize=14)
        axes[1].set_ylabel("Voltage (V)", fontsize=12)
        axes[1].set_xlabel("Time (s)", fontsize=12)
        axes[1].grid(True, linestyle='--', alpha=0.7)
        
        axes[2].set_title("State of Charge (SoC)", fontsize=14)
        axes[2].set_ylabel("SoC", fontsize=12)
        axes[2].set_xlabel("Time (s)", fontsize=12)
        axes[2].grid(True, linestyle='--', alpha=0.7)
        
        # Show plot
        plt.tight_layout()
        plt.show()

# Load the realistic current trajectory for the second aircraft from EnergyReq.mat
energy_req = loadmat('EnergyReq.mat')
current_input = energy_req['results'][1][1].flatten()  # Extract and flatten the current profile

# Initialize the battery model
battery = TarotBattery(0.1, 'battery_output.txt')

# Generate training and test datasets
training_data, test_data = battery.generate_dataset(current_input, num_training=7, num_test=3, scaling_range=(0.8, 1))

# Save the training and test datasets
np.save('training_data.npy', training_data)
np.save('test_data.npy', test_data)

# Plot the datasets
battery.plot_combined_results(training_data, test_data)

print("Training and test datasets have been generated and saved.")
