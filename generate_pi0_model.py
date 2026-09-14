import os
import torch
import torch.nn as nn

class Pi0NeuralPilot(nn.Module):
    """
    Synthetic Neural Network for Pi0-FAST.
    Simulates the LeRobot end-to-end flight control policy.
    Maps 7 states -> 4 control outputs.
    """
    def __init__(self):
        super().__init__()
        # Inputs: target_err_x, target_err_y, vx, vy, depth_dist, altitude, heading (7)
        self.fc1 = nn.Linear(7, 32)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(32, 16)
        self.fc3 = nn.Linear(16, 4)
        
        # Initialize output layer weights to extremely small values
        # so the untrained network acts safely (outputs near zero) 
        # instead of throwing the drone into a wall.
        nn.init.uniform_(self.fc3.weight, -0.01, 0.01)
        nn.init.zeros_(self.fc3.bias)
        
    def forward(self, x):
        # type: (Tensor) -> Tensor
        x = self.fc1(x)
        x = self.relu(x)
        x = self.fc2(x)
        x = self.relu(x)
        x = self.fc3(x)
        # Outputs: roll, pitch, yaw_rate, throttle
        return x

if __name__ == "__main__":
    project_root = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(project_root, "ai", "camera_brain", "laptop_ai", "models")
    os.makedirs(models_dir, exist_ok=True)
    
    model = Pi0NeuralPilot()
    model.eval()
    
    try:
        # Save as a TorchScript compiled module so the loader doesn't need to import the class definition
        scripted_model = torch.jit.script(model)
        save_path = os.path.join(models_dir, "pi0_fast.pt")
        scripted_model.save(save_path)
        print(f"SUCCESS: Generated and compiled Pi0-FAST Neural Network.")
        print(f"Saved to: {save_path}")
    except Exception as e:
        print(f"FAILED to generate model: {e}")
