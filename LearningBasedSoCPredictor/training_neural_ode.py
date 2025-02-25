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

# Define the ODE function represented by a neural network
class ODEFunc(nn.Module):
    def __init__(self, state_dim, hidden_dim):
        super(ODEFunc, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + 2, hidden_dim),  # Add 1 for current and 1 for time input
            nn.Tanh(),
            nn.Linear(hidden_dim, state_dim)
        )
        self.current_profile = None  # Placeholder for current profile
        self._initialize_weights()

    def _initialize_weights(self):
        for layer in self.net:
            if isinstance(layer, nn.Linear):
                nn.init.xavier_uniform_(layer.weight)
                nn.init.zeros_(layer.bias)

    def set_current_profile(self, current_profile):
        """
        Store the current profile for dynamic access during ODE computation.
        Args:
            current_profile (torch.Tensor): Shape [batch_size, seq_len, 1]
        """
        self.current_profile = current_profile

    def forward(self, t, x):
        """
        Compute the derivative of the state with access to the time-varying current profile.
        Args:
            t (torch.Tensor): Current time step (scalar or tensor).
            x (torch.Tensor): Current state (shape [batch_size, state_dim]).
        Returns:
            torch.Tensor: Derivative of the state (shape [batch_size, state_dim]).
        """
        if self.current_profile is None:
            raise ValueError("Current profile not set. Call set_current_profile before using ODEFunc.")

        # Fetch current at the corresponding time step
        time_idx = int(t.item() * (self.current_profile.size(1) - 1))  # Scale t to index
        current_at_t = self.current_profile[:, time_idx, :]  # Shape [batch_size, 1]

        # Concatenate state, current, and time
        time_tensor = t.expand_as(x[:, :1])  # Shape [batch_size, 1]
        input_tensor = torch.cat((x, current_at_t, time_tensor), dim=-1)  # [batch_size, state_dim + 2]

        # Debugging shapes
        # print(f"x shape: {x.shape}, current_at_t shape: {current_at_t.shape}, time_tensor shape: {time_tensor.shape}")
        # print(f"Input tensor shape to ODEFunc: {input_tensor.shape}")

        # Pass through the network
        out = self.net(input_tensor)
        return out


# Define the Neural ODE model
class NeuralODE(nn.Module):
    def __init__(self, ode_func):
        super(NeuralODE, self).__init__()
        self.ode_func = ode_func

    def forward(self, x0, current_profile, t):
        """
        Simulate the ODE for a given initial state and time-varying inputs.
        Args:
            x0 (torch.Tensor): Initial state, shape [batch_size, state_dim].
            current_profile (torch.Tensor): Time-varying current, shape [batch_size, seq_len, 1].
            t (torch.Tensor): Time steps, shape [seq_len].
        Returns:
            torch.Tensor: Simulated state trajectory, shape [batch_size, seq_len, state_dim].
        """
        # Set the current profile in ODEFunc
        self.ode_func.set_current_profile(current_profile)

        # Solve the ODE
        # print(f"Initial state shape: {x0.shape}, Current profile shape: {current_profile.shape}, Time steps shape: {t.shape}")
        out = odeint(self.ode_func, x0, t, method="euler")  # [seq_len, batch_size, state_dim]
        out = out.permute(1, 0, 2)  # [batch_size, seq_len, state_dim]
        return out


# Function to load and preprocess the data
def load_data(training_file, test_file):
    with open(training_file, "rb") as f:
        training_data = pickle.load(f)
    with open(test_file, "rb") as f:
        test_data = pickle.load(f)

    # Calculate normalization ranges dynamically
    current_min = min(data["input"].min() for data in training_data)
    current_max = max(data["input"].max() for data in training_data)
    voltage_min = min(data["output"].min() for data in training_data)
    voltage_max = max(data["output"].max() for data in training_data)
    
    print(current_min, current_max, voltage_min, voltage_max)

    train_inputs = [
        torch.tensor(normalize(data["input"], current_min, current_max), dtype=torch.float32)[:1000]
        for data in training_data
    ]
    train_outputs = [
        torch.tensor(normalize(data["output"], voltage_min, voltage_max), dtype=torch.float32)[:1000]
        for data in training_data
    ]
    test_inputs = [
        torch.tensor(normalize(data["input"], current_min, current_max), dtype=torch.float32)[:1000]
        for data in test_data
    ]
    test_outputs = [
        torch.tensor(normalize(data["output"], voltage_min, voltage_max), dtype=torch.float32)[:1000]
        for data in test_data
    ]

    # Compute mean initial voltage (normalized and denormalized)
    mean_initial_voltage_norm = torch.mean(torch.stack([outputs[0] for outputs in train_outputs])).item()
    mean_initial_voltage_actual = 27.4 #denormalize(mean_initial_voltage_norm, voltage_min, voltage_max)

    # Debugging outputs
    print(f"Voltage Min: {voltage_min}, Voltage Max: {voltage_max}")
    print(f"Mean Initial Voltage (Normalized): {mean_initial_voltage_norm}")
    print(f"Mean Initial Voltage (Actual): {mean_initial_voltage_actual}")
    print(f"Voltage Min: {voltage_min}, Voltage Max: {voltage_max}")


    return train_inputs, train_outputs, test_inputs, test_outputs, (current_min, current_max), (voltage_min, voltage_max), mean_initial_voltage_actual



# Function to create data loaders
def create_data_loader(inputs, outputs, batch_size=4):
    max_length = max(len(input) for input in inputs)
    padded_inputs = [torch.nn.functional.pad(input.unsqueeze(-1), (0, 0, 0, max_length - len(input))) for input in inputs]
    padded_outputs = [torch.nn.functional.pad(output.unsqueeze(-1), (0, 0, 0, max_length - len(output))) for output in outputs]

    dataset = torch.utils.data.TensorDataset(torch.stack(padded_inputs), torch.stack(padded_outputs))
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)
    return loader

# Training Neural ODE
def train_neural_ode(model, train_loader, val_loader, num_epochs=100, learning_rate=0.001):
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    criterion = nn.MSELoss()

    for epoch in range(num_epochs):
        total_loss = 0
        model.train()
        for batch_inputs, batch_outputs in tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs}"):
            optimizer.zero_grad()

            # Define normalized time steps [0, 1]
            t = torch.linspace(0, 1, batch_inputs.size(1))

            # Forward pass
            x0 = batch_inputs[:, 0, :]
            current_profile = batch_inputs
            pred = model(x0, current_profile, t)
                        # Debugging shapes
            # print(f"Initial state shape: {x0.shape}")
            # print(f"Current profile shape: {current_profile.shape}")
            # print(f"Predicted output shape: {pred.shape}")
            # print(f"Batch outputs shape: {batch_outputs.shape}")


            # # Check shapes
            # print(f"Prediction shape: {pred.shape}")
            # print(f"Batch outputs shape: {batch_outputs.shape}")

            # Calculate loss
            voltage_pred = pred[:, :, 0]  # Extract voltage predictions
            batch_outputs = batch_outputs.squeeze(-1)  # Ensure target is 2D for comparison

            # print(f"Voltage Prediction shape: {voltage_pred.shape}")
            # print(f"Batch outputs shape: {batch_outputs.shape}")

            loss = criterion(voltage_pred, batch_outputs)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        average_loss = total_loss / len(train_loader)
        print(f"Epoch {epoch+1}/{num_epochs}, Loss: {average_loss:.4f}")

# Evaluation function
def evaluate_model(model, test_inputs, test_outputs, current_range, voltage_range, actual_mean_initial_voltage, time_step=0.1, max_trajectories=5):
    model.eval()
    fig, axes = plt.subplots(3, 1, figsize=(8, 6), dpi=120, sharex=True)

    current_min, current_max = current_range
    voltage_min, voltage_max = voltage_range

    num_trajectories = min(len(test_inputs), max_trajectories)

    with torch.no_grad():
        for i, (test_input, test_output) in enumerate(zip(test_inputs[:num_trajectories], test_outputs[:num_trajectories])):
            actual_time = np.arange(len(test_input)) * time_step  # Shape (1000,)
            t_test = torch.linspace(0, 1, len(test_input))  # Shape (1000,)

            # Use actual mean initial voltage
            x0 = torch.tensor([[normalize(actual_mean_initial_voltage,voltage_min, voltage_max) ]], dtype=torch.float32)  # Shape [1, 1]
            current_profile = test_input.unsqueeze(0).unsqueeze(-1)  # Shape [1, 1000, 1]
            # plt.plot(current_profile[0])
            # plt.show()
            # Perform ODE integration
            pred_test = model(x0, current_profile, t_test)[:, :, 0].squeeze(0).numpy()  # Remove batch dim

            # Denormalize predictions and ground truth
            pred_test = denormalize(pred_test, voltage_min, voltage_max)
            test_output = denormalize(test_output.numpy(), voltage_min, voltage_max)

            # Debugging
            print(f"Initial Voltage Used: {actual_mean_initial_voltage}")
            print(f"Prediction Shape: {pred_test.shape}, Ground Truth Shape: {test_output.shape}")
            # print(f"Voltage Min: {voltage_min}, Voltage Max: {voltage_max}")
            # print(len(test_output))
            # print(len(pred_test))

            plt.plot(actual_time, test_output)
            plt.plot(actual_time, pred_test)
            plt.show()


            # Plot results
            axes[0].plot(actual_time, denormalize(test_input.numpy(), current_min, current_max), label=f"Test {i+1}", alpha=0.7)
            axes[1].plot(actual_time, test_output, label=f"Ground Truth {i+1}", linestyle="dashed", alpha=0.7)
            axes[2].plot(actual_time, pred_test, label=f"Predicted {i+1}", alpha=0.7)

    axes[0].set_title("Test Current Profiles")
    axes[0].set_ylabel("Current (A)")
    axes[0].grid(True, linestyle="--", alpha=0.7)

    axes[1].set_title("Ground Truth Voltage Trajectories")
    axes[1].set_ylabel("Voltage (V)")
    axes[1].grid(True, linestyle="--", alpha=0.7)

    axes[2].set_title("Predicted Voltage Trajectories")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Voltage (V)")
    axes[2].grid(True, linestyle="--", alpha=0.7)

    plt.tight_layout()
    plt.savefig("traj_evaluations.png", format='png', dpi=300)
    plt.show()



# Main script
if __name__ == "__main__":
    train_inputs, train_outputs, test_inputs, test_outputs, current_range, voltage_range, actual_mean_initial_voltage = load_data(
        "training_data.pkl", "test_data.pkl"
    )

    train_loader = create_data_loader(train_inputs, train_outputs, batch_size=4)
    val_loader = create_data_loader(test_inputs, test_outputs, batch_size=4)

    state_dim = 1  # Voltage dimension
    hidden_dim = 50
    ode_func = ODEFunc(state_dim=state_dim, hidden_dim=hidden_dim)
    neural_ode = NeuralODE(ode_func)

    model_file = "neural_ode_model.pth"
    if not os.path.exists(model_file):
        train_neural_ode(neural_ode, train_loader, val_loader, num_epochs=100, learning_rate=0.001)
        torch.save(neural_ode.state_dict(), model_file)
    else:
        neural_ode.load_state_dict(torch.load(model_file))
        neural_ode.eval()

    evaluate_model(neural_ode, test_inputs, test_outputs, current_range, voltage_range, actual_mean_initial_voltage, max_trajectories=5)