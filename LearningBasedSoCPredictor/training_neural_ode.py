import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torchdiffeq import odeint
from tqdm import tqdm
import matplotlib.pyplot as plt

# Define the ODE function represented by a neural network
class ODEFunc(nn.Module):
    def __init__(self, input_dim, hidden_dim):
        super(ODEFunc, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, input_dim)
        )

    def forward(self, t, x):
        return self.net(x)

# Define the Neural ODE model
class NeuralODE(nn.Module):
    def __init__(self, ode_func):
        super(NeuralODE, self).__init__()
        self.ode_func = ode_func

    def forward(self, x0, t):
        out = odeint(self.ode_func, x0, t)
        return out

# Function to load and preprocess the data
def load_data(training_file, test_file):
    training_data = np.load(training_file, allow_pickle=True)
    test_data = np.load(test_file, allow_pickle=True)

    train_inputs = [torch.tensor(x[0].reshape(-1, 1), dtype=torch.float32) for x in training_data]
    train_outputs = [torch.tensor(x[1].reshape(-1, 1), dtype=torch.float32) for x in training_data]
    test_inputs = [torch.tensor(x[0].reshape(-1, 1), dtype=torch.float32) for x in test_data]
    test_outputs = [torch.tensor(x[1].reshape(-1, 1), dtype=torch.float32) for x in test_data]

    return train_inputs, train_outputs, test_inputs, test_outputs

# Function to create data loaders
def create_data_loader(inputs, outputs, batch_size=16):
    dataset = torch.utils.data.TensorDataset(torch.stack(inputs), torch.stack(outputs))
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)
    return loader

# Training function with real-time loss visualization
def train_neural_ode(model, train_loader, num_epochs=100, learning_rate=0.001):
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    criterion = nn.MSELoss()
    loss_history = []

    plt.ion()  # Turn on interactive mode for real-time plotting
    fig, ax = plt.subplots()
    ax.set_title("Training Loss")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Loss")

    for epoch in range(num_epochs):
        total_loss = 0
        model.train()
        for batch_inputs, batch_outputs in tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs}"):
            optimizer.zero_grad()
            t = torch.linspace(0, batch_inputs.shape[1] - 1, batch_inputs.shape[1])
            
            # Forward pass
            pred = model(batch_inputs, t)  # pred shape: [time_steps, batch_size, features]
            
            # Extract the last time step's output and ensure correct shape
            pred_last_step = pred[-1].squeeze(-1)  # Shape: [batch_size, features]
            batch_outputs = batch_outputs[:, -1]  # Extract the last time step from batch_outputs
            
            # Calculate loss
            loss = criterion(pred_last_step, batch_outputs)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        average_loss = total_loss / len(train_loader)
        loss_history.append(average_loss)

        # Update plot
        ax.plot(loss_history, color='blue')
        plt.pause(0.1)

        print(f"Epoch {epoch+1}/{num_epochs}, Loss: {average_loss:.4f}")

    plt.ioff()  # Turn off interactive mode
    plt.show()

# Evaluation function
def evaluate_model(model, test_inputs):
    model.eval()
    with torch.no_grad():
        for i, test_input in enumerate(test_inputs):
            t_test = torch.linspace(0, len(test_input) - 1, len(test_input))
            pred_test = model(test_input.unsqueeze(0), t_test)
            plt.plot(t_test, pred_test.squeeze().numpy(), label=f"Test Trajectory {i+1}")

    plt.title("Test Trajectories")
    plt.xlabel("Time (s)")
    plt.ylabel("Outputs")
    plt.legend()
    plt.grid(True)
    plt.show()

# Main script
if __name__ == "__main__":
    train_inputs, train_outputs, test_inputs, test_outputs = load_data('training_data.npy', 'test_data.npy')

    # Create data loader
    train_loader = create_data_loader(train_inputs, train_outputs, batch_size=16)

    # Instantiate the model
    input_dim = train_inputs[0].shape[-1]  # Adjust based on input shape
    ode_func = ODEFunc(input_dim=input_dim, hidden_dim=50)
    neural_ode = NeuralODE(ode_func)

    # Train the model
    train_neural_ode(neural_ode, train_loader, num_epochs=100, learning_rate=0.001)

    # Evaluate the model
    evaluate_model(neural_ode, test_inputs)

    print("Training and evaluation completed.")
