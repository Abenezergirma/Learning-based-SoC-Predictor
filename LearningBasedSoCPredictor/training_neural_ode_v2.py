import pickle
import numpy as np
import os
import torch
import torch.nn as nn
import torch.optim as optim
from torchdiffeq import odeint
import matplotlib.pyplot as plt

# Function to normalize data using mean and standard deviation
def standardize(data, mean, std):
    return (data - mean) / std

# Function to denormalize data
def destandardize(data, mean, std):
    return data * std + mean

# Define the improved ODE function represented by a deeper neural network
class ODEFunc(nn.Module):
    def __init__(self, state_dim, hidden_dim):
        super(ODEFunc, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + 2, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),  
            nn.ReLU(),
            nn.Linear(hidden_dim, state_dim)
        )
        self.current_profile = None  

    def set_current_profile(self, current_profile):
        self.current_profile = current_profile

    def forward(self, t, x):
        if self.current_profile is None:
            raise ValueError("Current profile not set. Call set_current_profile before using ODEFunc.")

        time_idx = int(t.item() * (self.current_profile.size(1) - 1))  
        current_at_t = self.current_profile[:, time_idx, :]  

        time_tensor = t.expand_as(x[:, :1])  
        input_tensor = torch.cat((x, current_at_t, time_tensor), dim=-1)  

        return self.net(input_tensor)


# Define the Neural ODE model
class NeuralODE(nn.Module):
    def __init__(self, ode_func):
        super(NeuralODE, self).__init__()
        self.ode_func = ode_func

    def forward(self, x0, current_profile, t):
        self.ode_func.set_current_profile(current_profile)
        out = odeint(self.ode_func, x0, t, method="dopri5")  
        return out.permute(1, 0, 2)  


# Load and preprocess the data
def load_data(training_file, test_file):
    with open(training_file, "rb") as f:
        training_data = pickle.load(f)
    with open(test_file, "rb") as f:
        test_data = pickle.load(f)

    current_mean = np.mean([data["input"].mean() for data in training_data])
    current_std = np.std([data["input"].std() for data in training_data])
    voltage_mean = np.mean([data["output"].mean() for data in training_data])
    voltage_std = np.std([data["output"].std() for data in training_data])

    train_inputs = [torch.tensor(standardize(data["input"], current_mean, current_std), dtype=torch.float32) for data in training_data]
    train_outputs = [torch.tensor(standardize(data["output"], voltage_mean, voltage_std), dtype=torch.float32) for data in training_data]
    test_inputs = [torch.tensor(standardize(data["input"], current_mean, current_std), dtype=torch.float32) for data in test_data]
    test_outputs = [torch.tensor(standardize(data["output"], voltage_mean, voltage_std), dtype=torch.float32) for data in test_data]

    return train_inputs, train_outputs, test_inputs, test_outputs, (current_mean, current_std), (voltage_mean, voltage_std)


# Create DataLoader with padding
def create_data_loader(inputs, outputs, batch_size=4):
    max_length = max(len(input) for input in inputs)
    padded_inputs = [torch.nn.functional.pad(input.unsqueeze(-1), (0, 0, 0, max_length - len(input))) for input in inputs]
    padded_outputs = [torch.nn.functional.pad(output.unsqueeze(-1), (0, 0, 0, max_length - len(output))) for output in outputs]

    dataset = torch.utils.data.TensorDataset(torch.stack(padded_inputs), torch.stack(padded_outputs))
    return torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)


# Training Neural ODE
def train_neural_ode(model, train_loader, val_loader, num_epochs=300, learning_rate=0.001):
    optimizer = optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=1e-4)
    criterion = nn.SmoothL1Loss()  
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=10)

    for epoch in range(num_epochs):
        total_loss = 0
        model.train()
        for batch_inputs, batch_outputs in train_loader:
            optimizer.zero_grad()
            t = torch.linspace(0, 1, batch_inputs.size(1))

            x0 = batch_inputs[:, 0, :]
            pred = model(x0, batch_inputs, t)

            voltage_pred = pred[:, :, 0]  
            batch_outputs = batch_outputs.squeeze(-1)  

            loss = criterion(voltage_pred, batch_outputs)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        avg_loss = total_loss / len(train_loader)
        print(f"Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.4f}")
        scheduler.step(avg_loss)


# Evaluate the model
def evaluate_model(model, test_inputs, test_outputs, current_range, voltage_range, time_step=0.1, max_trajectories=5):
    model.eval()
    fig, axes = plt.subplots(3, 1, figsize=(8, 6), dpi=120, sharex=True)

    current_mean, current_std = current_range
    voltage_mean, voltage_std = voltage_range

    num_trajectories = min(len(test_inputs), max_trajectories)

    with torch.no_grad():
        for i, (test_input, test_output) in enumerate(zip(test_inputs[:num_trajectories], test_outputs[:num_trajectories])):
            actual_time = np.arange(len(test_input)) * time_step  
            t_test = torch.linspace(0, 1, len(test_input))  

            x0 = torch.tensor([[standardize(27.4, voltage_mean, voltage_std)]], dtype=torch.float32)
            current_profile = test_input.unsqueeze(0).unsqueeze(-1)

            pred_test = model(x0, current_profile, t_test)[:, :, 0].squeeze(0).numpy()

            pred_test = destandardize(pred_test, voltage_mean, voltage_std)
            test_output = destandardize(test_output.numpy(), voltage_mean, voltage_std)

            axes[0].plot(actual_time, destandardize(test_input.numpy(), current_mean, current_std), label=f"Test {i+1}")
            axes[1].plot(actual_time, test_output, label=f"Ground Truth {i+1}", linestyle="dashed")
            axes[2].plot(actual_time, pred_test, label=f"Predicted {i+1}")

    axes[0].set_title("Test Current Profiles")
    axes[1].set_title("Ground Truth Voltage Trajectories")
    axes[2].set_title("Predicted Voltage Trajectories")
    plt.tight_layout()
    plt.show()


# Main script
if __name__ == "__main__":
    train_inputs, train_outputs, test_inputs, test_outputs, current_range, voltage_range = load_data("training_data.pkl", "test_data.pkl")

    train_loader = create_data_loader(train_inputs, train_outputs, batch_size=4)
    val_loader = create_data_loader(test_inputs, test_outputs, batch_size=4)

    ode_func = ODEFunc(state_dim=1, hidden_dim=100)
    neural_ode = NeuralODE(ode_func)

    if not os.path.exists("neural_ode_model.pth"):
        train_neural_ode(neural_ode, train_loader, val_loader, num_epochs=300, learning_rate=0.001)
        torch.save(neural_ode.state_dict(), "neural_ode_model.pth")
    else:
        neural_ode.load_state_dict(torch.load("neural_ode_model.pth"))

    evaluate_model(neural_ode, test_inputs, test_outputs, current_range, voltage_range)
