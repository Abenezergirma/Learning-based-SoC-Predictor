import scipy.io
import numpy as np
import torch
import matplotlib.pyplot as plt
import os
import scipy.interpolate as interp
from torchdiffeq import odeint
from battery_model.battery_electrochem_TarotT18 import BatteryElectroChem as Battery
from generate_dataset_v5 import TarotBattery
from BatteryPrognosticsLibrary import BatteryPrognostics
# from visulalize_prognostics import VisualizePrognostics



plt.rcParams['text.usetex'] = True

# ----------------------------
# Normalization Functions
# ----------------------------
def normalize(data, min_val, max_val):
    return (data - min_val) / (max_val - min_val)

def denormalize(data, min_val, max_val):
    return data * (max_val - min_val) + min_val

# ----------------------------
# Load Neural ODE Model
# ----------------------------
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

# ----------------------------
# Power Profile Processing
# ----------------------------
class PowerProfilePlotter:
    def __init__(self, data_file, model_path):
        self.data_file = data_file
        self.model_path = model_path
        self.projected_profiles_list = self.load_data()
        self.matlab_data = scipy.io.loadmat('actualSim.mat', struct_as_record=False, squeeze_me=True, simplify_cells=True)
        self.matlab_voltage = self.matlab_data['results'][2]
        self.timeb = self.matlab_data['results'][6]
        
        # plt.plot(self.timeb, self.matlab_voltage)
        # plt.show()
        
        # Load Neural ODE model
        self.state_dim, self.hidden_dim = 1, 50
        self.ode_func = ODEFunc(self.state_dim, self.hidden_dim)
        self.neural_ode = NeuralODE(self.ode_func)
        self.neural_ode.load_state_dict(torch.load(self.model_path))
        self.neural_ode.eval()

        # Normalization parameters
        self.current_min, self.current_max = 60.01166838020301, 245.9070490172893  
        self.voltage_min, self.voltage_max = 19.946281873422357, 27.44040311532278
        self.mean_initial_voltage_actual = 27.4
        self.nominal_voltage = 27

        self.battery_simulator = TarotBattery(time_step=0.1)  

    def load_data(self):
        """Loads power profiles from a .mat file."""
        if not os.path.exists(self.data_file):
            print("Error: File not found.")
            return []

        data = scipy.io.loadmat(self.data_file, struct_as_record=False, squeeze_me=True, simplify_cells=True)
        return data.get('projected_profiles_list', [])
    
    def plot_current_profiles(self):
        """
        Generates and saves a high-quality plot of current profiles,
        optimized for academic paper formatting in a single-column A4 layout.
        """
        if not self.projected_profiles_list:
            print("No projected profiles found.")
            return

        fig, ax = plt.subplots(figsize=(5, 4), dpi=300)
        colors = plt.cm.viridis(np.linspace(0, 1, len(self.projected_profiles_list)))

        for idx, profile in enumerate(self.projected_profiles_list):
            current_profile = np.array(profile['power']) / self.nominal_voltage  # Convert power to current
            ax.plot(profile['time'], current_profile, color=colors[idx], linewidth=1.2, alpha=0.8)

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Current (A)', fontsize=12)
        # ax.set_title('Converted Current Profiles', fontsize=14, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.7)
        
        # Create a colorbar instead of individual legends
        sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, norm=plt.Normalize(vmin=1, vmax=len(self.projected_profiles_list)))
        cbar = fig.colorbar(sm, ax=ax, aspect=30, pad=0.02)
        cbar.set_label("Profile Index", fontsize=10)

        plt.savefig("Current_Profiles.pdf", format="pdf", bbox_inches="tight")
        plt.show()
        plt.close()    

    def predict_voltage(self):
        """Predicts voltage trajectories using Neural ODE and saves the corresponding current profiles."""
        if not self.projected_profiles_list:
            print("No projected profiles found.")
            return

        predicted_voltage_trajectories = []
        saved_current_profiles = []  

        global_time_min = self.projected_profiles_list[0]['time'][0]
        global_time_max = self.projected_profiles_list[0]['time'][-1]

        initial_voltage = normalize(self.mean_initial_voltage_actual, self.voltage_min, self.voltage_max)

        for idx, profile in enumerate(self.projected_profiles_list):
            print(f"\n🚀 Processing Profile {idx+1}")
            # ✅ Check if both 'power' and 'time' are valid lists or NumPy arrays
            if not isinstance(profile.get('power'), (list, np.ndarray)) or not isinstance(profile.get('time'), (list, np.ndarray)):
                print(f"⚠️ Warning: Skipping Profile {idx+1} due to invalid format.")
                continue  # Skip to the next iteration

            current_profile = normalize(profile['power'] / self.nominal_voltage, self.current_min, self.current_max)
            original_time_array = np.array(profile['time'])
            valid_indices = original_time_array <= global_time_max
            original_time_array = original_time_array[valid_indices]
            current_profile = current_profile[:len(original_time_array)]
   
            normalized_time_array = (original_time_array - global_time_min) / (global_time_max - global_time_min)

            current_tensor = torch.tensor(current_profile, dtype=torch.float32).unsqueeze(0).unsqueeze(-1)
            t_tensor = torch.tensor(normalized_time_array, dtype=torch.float32)

            min_length = min(len(t_tensor), current_tensor.shape[1])
            t_tensor, current_tensor = t_tensor[:min_length], current_tensor[:, :min_length, :]

            x0 = torch.tensor([[initial_voltage]], dtype=torch.float32)
            predicted_voltage = self.neural_ode(x0, current_tensor, t_tensor)[:, :, 0].detach().numpy().flatten()

            trajectory_data = {
                'time': normalized_time_array[:min_length].tolist(),
                'voltage': predicted_voltage[:min_length].tolist()
            }
            predicted_voltage_trajectories.append(trajectory_data)


            # ✅ Fix: Ensure `next_initial_time_real` is valid before accessing
            if idx < len(self.projected_profiles_list) - 1:
                next_profile = self.projected_profiles_list[idx + 1]
                
                # ✅ Debugging print: Ensure next profile is correctly formatted
                if not isinstance(next_profile, dict) or 'time' not in next_profile or not isinstance(next_profile['time'], (list, np.ndarray)):
                    print(f"⚠️ Warning: Skipping next_initial_time_real due to invalid format in profile {idx + 2}")
                    continue  # Skip to the next iteration

                next_initial_time_real = next_profile['time'][0]  # ✅ Now safe to access

                next_initial_time = (next_initial_time_real - global_time_min) / (global_time_max - global_time_min)

                idx_match = np.where(np.isclose(normalized_time_array, next_initial_time, atol=1e-4))[0]

                if len(idx_match) > 0:
                    next_initial_voltage = predicted_voltage[idx_match[0]]
                else:
                    interp_func = interp.interp1d(normalized_time_array, predicted_voltage, kind='linear', fill_value="extrapolate")
                    next_initial_voltage = interp_func(next_initial_time)

                initial_voltage = next_initial_voltage 
                  
                #   Debugging Info: Compare voltage change
                actual_previous_voltage = predicted_voltage[-1]
                voltage_diff = abs(next_initial_voltage - actual_previous_voltage)
                print(f"🔴 Expected Next Initial Time: {next_initial_time_real:.4f} sec")
                print(f"🔴 Expected Next Initial Voltage: {next_initial_voltage:.4f} V")
                print(f"🔵 Actual Last Voltage of Previous Profile: {actual_previous_voltage:.4f} V")
                print(f"🔵 Voltage Difference: {voltage_diff:.6f} V")
                
                saved_current_profiles.append(denormalize(current_profile, self.current_min, self.current_max))

        #   **Denormalize Before Returning**
        for trajectory in predicted_voltage_trajectories:
            trajectory['time'] = denormalize(np.array(trajectory['time']), global_time_min, global_time_max).tolist()
            trajectory['voltage'] = denormalize(np.array(trajectory['voltage']), self.voltage_min, self.voltage_max).tolist()

        self.saved_current_profiles = saved_current_profiles 

        return predicted_voltage_trajectories


    def simulate_actual_battery(self, predicted_voltage_trajectories):
        """Simulates the actual battery using saved current profiles and drives the battery to the correct voltage first."""
        if not hasattr(self, 'saved_current_profiles') or not self.saved_current_profiles:
            print("⚠️ No saved current profiles found.")
            return []

        actual_voltage_trajectories = []

        # ✅ Use the first current profile as the most complete profile
        complete_current_profile = self.saved_current_profiles[0]
        print(len(complete_current_profile))
        complete_time_array = np.array(self.projected_profiles_list[0]['time'])  # Corresponding time array
        np.save("complete_time_array.npy", complete_current_profile)  # Save as .npy file
        np.save("complete_current_array.npy", complete_time_array)  # Save as .npy file

        for idx, profile in enumerate(self.saved_current_profiles):
            print(f"🔬 Simulating actual battery for Profile {idx+1}...")

            time = np.array(self.projected_profiles_list[idx]['time'])
            current = np.array(self.saved_current_profiles[idx])
            

            # ✅ Find the index in the first time array that matches the first element of `time`
            start_time = time[0]
            prior_index = np.searchsorted(complete_time_array, start_time, side='right')

            # ✅ Extract prior current profile from the complete profile
            prior_current_profile = complete_current_profile[:prior_index]
            remaining_current_profile = current  # The rest of the current profile is unchanged

            # ✅ Use the predicted initial voltage from the corresponding profile
            predicted_initial_voltage = predicted_voltage_trajectories[idx]['voltage'][0]
            
            # if idx == 0:
            #     continue
            # else:

            # ✅ Drive the battery model to the correct voltage using the prior current profile
            self.battery_simulator.set_initial_voltage(predicted_initial_voltage, prior_current_profile)

            # ✅ Simulate the actual battery response
            voltage_trajectory = self.battery_simulator.simulate_battery(remaining_current_profile, predicted_initial_voltage)
            
            # plt.plot(voltage_trajectory, color='red')
            # plt.plot(predicted_voltage_trajectories[idx]['voltage'], color='blue')
            # plt.show()
            
            # plt.plot(remaining_current_profile)
            # plt.show()

            if voltage_trajectory is not None:
                actual_voltage_trajectories.append({'time': time.tolist(), 'voltage': voltage_trajectory.tolist()})

        return actual_voltage_trajectories


    def plot_comparison_voltage_trajectories(self, predicted_voltage_trajectories, actual_voltage_trajectories):
        """
        Plots both predicted and actual battery voltage trajectories for comparison.
        """
        fig, ax = plt.subplots(figsize=(6, 4), dpi=300)
        colors = plt.cm.viridis(np.linspace(0, 1, len(predicted_voltage_trajectories)))
        
        # Load array in another script
        loaded_array = np.load("batt_volt.npy")

 


        for idx, (predicted, actual) in enumerate(zip(predicted_voltage_trajectories, actual_voltage_trajectories)):
            pred_time, pred_voltage = np.array(predicted['time']), np.array(predicted['voltage'])
            actual_time, actual_voltage = np.array(actual['time']), np.array(actual['voltage'])

            # Debugging print to check mismatched sizes
            # if len(pred_time) != len(pred_voltage):
            #     print(f"⚠️ Warning: Mismatch in predicted data at index {idx}: time ({len(pred_time)}) vs voltage ({len(pred_voltage)})")

            # if len(actual_time) != len(actual_voltage):
            #     print(f"⚠️ Warning: Mismatch in actual data at index {idx}: time ({len(actual_time)}) vs voltage ({len(actual_voltage)})")

            # Ensure both time and voltage arrays have the same length
            min_length = min(len(pred_time), len(pred_voltage), len(actual_time), len(actual_voltage))

            pred_time, pred_voltage = pred_time[:min_length], pred_voltage[:min_length]
            actual_time, actual_voltage = actual_time[:min_length], actual_voltage[:min_length]

            # ✅ If actual data is shorter, interpolate it to match predicted data
            if len(actual_time) < len(pred_time):
                interp_func = interp.interp1d(actual_time, actual_voltage, kind='linear', fill_value="extrapolate")
                actual_voltage = interp_func(pred_time)
                actual_time = pred_time  # Now they have the same time stamps

            # Plot predicted and actual voltage trajectories
            ax.plot(pred_time, pred_voltage, linestyle='dashed', color=colors[idx], linewidth=1.2, alpha=0.8)
            ax.plot(actual_time, actual_voltage, linestyle='solid', color=colors[idx], linewidth=1.2, alpha=0.8)
        # ax.plot(self.timeb, self.matlab_voltage+1.9, linewidth=1.2, alpha=0.8)
        ax.plot(loaded_array[0]+1.9, linewidth=1.2, alpha=0.8)
        # ax.show()
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Voltage (V)', fontsize=12)
        ax.set_title('Predicted vs Actual Voltage Profiles', fontsize=14, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.7)

        # Create a colorbar instead of individual legends
        sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, norm=plt.Normalize(vmin=1, vmax=len(predicted_voltage_trajectories)))
        cbar = fig.colorbar(sm, ax=ax, aspect=30, pad=0.02)
        cbar.set_label("Profile Index", fontsize=10)

        plt.savefig("Voltage_Comparison.pdf", format="pdf", bbox_inches="tight")
        plt.show()
        plt.close()



# Usage
if __name__ == "__main__":
    relative_path = os.path.join('SimulationResults', 'projected_profiles.mat')
    model_path = os.path.join( 'neural_ode_model.pth')

    plotter = PowerProfilePlotter(relative_path, model_path)
    # plotter.plot_power_profiles()
    # plotter.plot_current_profiles()
    predicted_voltage_trajectories = plotter.predict_voltage()
    
    
        # Simulate actual battery response
    actual_voltage_trajectories = plotter.simulate_actual_battery(predicted_voltage_trajectories)
    
    # Plot predicted vs actual voltages
    plotter.plot_comparison_voltage_trajectories(predicted_voltage_trajectories, actual_voltage_trajectories)
