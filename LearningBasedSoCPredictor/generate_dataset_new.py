import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm  # For progress monitoring
from scipy.io import loadmat  # For loading MATLAB files
from battery_model.battery_electrochem_TarotT18 import BatteryElectroChem as Battery

# Current Profile Generator Class
class CurrentProfileGenerator:
    def __init__(self, reference_profile, min_val, max_val, max_num_segments, noise_std=0.1):
        self.reference_profile = reference_profile
        self.min_val = min_val
        self.max_val = max_val
        self.max_num_segments = max_num_segments
        self.noise_std = noise_std

    def generate_variable_profile(self):
        total_length = len(self.reference_profile)
        num_segments = np.random.randint(1, self.max_num_segments + 1)
        segment_boundaries = self._get_random_partition(total_length, num_segments)

        profile = np.zeros(total_length)
        for i in range(num_segments):
            start = segment_boundaries[i]
            end = segment_boundaries[i + 1]
            profile[start:end] = np.random.uniform(self.min_val, self.max_val)

        noise = np.random.normal(0, self.noise_std, total_length)
        profile = np.clip(profile + noise, self.min_val, self.max_val)
        return profile

    def generate_constant_profile(self):
        """
        Generate a constant current profile.
        Returns:
            np.ndarray: Constant current profile.
        """
        constant_value = np.random.uniform(self.min_val, self.max_val)
        return np.full_like(self.reference_profile, constant_value)

    def _get_random_partition(self, total_length, num_segments):
        boundaries = sorted(np.random.choice(range(1, total_length), num_segments - 1, replace=False))
        return [0] + boundaries + [total_length]


# Tarot Battery Class
class TarotBattery:
    def __init__(self, time_step, filename):
        R_vars = {"t": 2.2, "v": 1.46}
        self.battery = Battery(process_noise=0.25, process_noise_dist="normal", measurement_noise=R_vars, measurement_noise_dist="normal")
        self.initial_state = self.battery.initialize()
        self.x = self.initial_state
        self.t = 0
        self.time_step = time_step
        self.f = open(filename, "w")

    def reset_battery_state(self):
        self.x = self.initial_state
        self.t = 0

    def step(self, currentNeeded, V_min=18, V_max=25.2, max_current=200):
        self.t += self.time_step
        try:
            current_clipped = min(currentNeeded, max_current)
            current_input = {"i": current_clipped}
            self.x = self.battery.next_state(self.x, current_input, self.time_step)
            output = self.battery.output(self.x)
            voltage = output["v"]
            soc = (output["v"] - self.battery.parameters["VEOD"]) / self.battery.parameters["VDropoff"]
            self.f.write(f"{voltage}\n")
            return voltage, soc
        except (ValueError, KeyError) as e:
            print(f"Error during step execution: {e}. Skipping current trajectory.")
            return None, None

    def simulate_battery(self, current_trajectory):
        voltage_trajectory = []
        soc_trajectory = []
        for current in tqdm(current_trajectory, desc="Simulating Trajectory", leave=False):
            voltage, soc = self.step(current)
            if voltage is None or soc is None:
                return None, None
            voltage_trajectory.append(voltage)
            soc_trajectory.append(soc)
        return np.array(voltage_trajectory), np.array(soc_trajectory)

    def plot_combined_results(self, training_data, test_data):
        fig, axes = plt.subplots(1, 3, figsize=(18, 5), dpi=120)
        for current, voltage, soc in training_data:
            time_steps = np.arange(len(current)) * self.time_step
            axes[0].plot(time_steps, current, color="blue", linewidth=0.8, alpha=0.6)
            axes[1].plot(time_steps, voltage, color="blue", linewidth=0.8, alpha=0.6)
            axes[2].plot(time_steps, soc, color="blue", linewidth=0.8, alpha=0.6)

        for current, voltage, soc in test_data:
            time_steps = np.arange(len(current)) * self.time_step
            axes[0].plot(time_steps, current, color="red", linewidth=0.8, alpha=0.6)
            axes[1].plot(time_steps, voltage, color="red", linewidth=0.8, alpha=0.6)
            axes[2].plot(time_steps, soc, color="red", linewidth=0.8, alpha=0.6)

        axes[0].set_title("Current Trajectories")
        axes[0].set_ylabel("Current (A)")
        axes[0].set_xlabel("Time (s)")
        axes[0].grid(True, linestyle="--", alpha=0.7)

        axes[1].set_title("Voltage Responses")
        axes[1].set_ylabel("Voltage (V)")
        axes[1].set_xlabel("Time (s)")
        axes[1].grid(True, linestyle="--", alpha=0.7)

        axes[2].set_title("State of Charge (SoC)")
        axes[2].set_ylabel("SoC")
        axes[2].set_xlabel("Time (s)")
        axes[2].grid(True, linestyle="--", alpha=0.7)

        plt.tight_layout()
        plt.show()


# Generate mixed profiles
def generate_mixed_profiles(profile_generator, num_profiles, mix_ratio=0.5):
    """
    Generate a mix of variable and constant current profiles.

    Args:
        profile_generator (CurrentProfileGenerator): The profile generator object.
        num_profiles (int): Total number of profiles to generate.
        mix_ratio (float): Ratio of variable profiles in the mix (e.g., 0.5 for 50% variable profiles).

    Returns:
        list: List of current profiles.
    """
    profiles = []
    num_variable = int(num_profiles * mix_ratio)
    num_constant = num_profiles - num_variable

    for _ in range(num_variable):
        profiles.append(profile_generator.generate_variable_profile())

    for _ in range(num_constant):
        profiles.append(profile_generator.generate_constant_profile())

    return profiles


# Main Script
energy_req = loadmat("EnergyReq.mat")
reference_current = energy_req["results"][1][1].flatten()

# Initialize CurrentProfileGenerator
profile_generator = CurrentProfileGenerator(
    reference_profile=reference_current,
    min_val=50,
    max_val=70,
    max_num_segments=10,
    noise_std=0.07,
)

# Initialize TarotBattery
battery = TarotBattery(0.1, "battery_output.txt")

# Generate training and test profiles
num_training = 7
num_test = 3
mix_ratio = 0.6  # 60% variable profiles, 40% constant profiles

training_profiles = generate_mixed_profiles(profile_generator, num_training, mix_ratio)
test_profiles = generate_mixed_profiles(profile_generator, num_test, mix_ratio)

# Generate datasets
training_data = []
test_data = []

print("Generating training data...")
for profile in tqdm(training_profiles, desc="Training Trajectories"):
    battery.reset_battery_state()
    voltage, soc = battery.simulate_battery(profile)
    if voltage is not None and soc is not None:
        training_data.append((profile, voltage, soc))

print("Generating test data...")
for profile in tqdm(test_profiles, desc="Test Trajectories"):
    battery.reset_battery_state()
    voltage, soc = battery.simulate_battery(profile)
    if voltage is not None and soc is not None:
        test_data.append((profile, voltage, soc))

# Save datasets
np.save("training_data.npy", training_data)
np.save("test_data.npy", test_data)

# Plot results
battery.plot_combined_results(training_data, test_data)

print("Datasets generated with a mix of variable and constant current profiles.")
