"""Synthetic Multi-Agent Trajectory Dataset Generator for Indian Driving Scenarios.
Generates (History 2.0s, Future 5.0s) trajectory sequences at 10 Hz.
"""

from pathlib import Path
import math
import random
import numpy as np


def generate_scenario_trajectories(
    num_samples: int = 200,
    obs_steps: int = 20,
    pred_steps: int = 50,
    dt: float = 0.1,
) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """Generates synthetic multi-agent trajectory pairs (X_obs, Y_future) representing Indian driving edge cases."""
    x_obs_list = []
    y_fut_list = []
    classes_list = []

    total_steps = obs_steps + pred_steps # 70 steps = 7.0 seconds total

    agent_behaviors = [
        ("Auto-Rickshaw", "cutin"),
        ("Two-Wheeler", "weave"),
        ("Cattle", "stationary"),
        ("Pedestrian", "cross"),
        ("Car", "straight"),
        ("Truck-Bus", "wide_turn"),
    ]

    for _ in range(num_samples):
        cls_name, behavior = random.choice(agent_behaviors)

        # Initial random position and velocity
        start_x = random.uniform(10.0, 80.0)
        start_y = random.uniform(-4.0, 4.0)
        start_speed = random.uniform(2.0, 16.0)
        start_yaw = random.uniform(-0.15, 0.15)

        traj = np.zeros((total_steps, 2), dtype=np.float64)
        curr_x, curr_y = start_x, start_y
        curr_v = start_speed
        curr_yaw = start_yaw

        if behavior == "stationary":
            # Cattle standing or resting
            traj[:] = [start_x, start_y]
        elif behavior == "cutin":
            # Auto-rickshaw cruise then sudden lateral displacement
            cut_step = random.randint(15, 30)
            cut_rate = random.choice([-1.2, 1.2]) # sudden lateral drift
            for t in range(total_steps):
                if t >= cut_step:
                    curr_yaw += cut_rate * 0.05
                    curr_v = max(3.0, curr_v - 0.1) # slow down during cutin
                curr_x += curr_v * math.cos(curr_yaw) * dt
                curr_y += curr_v * math.sin(curr_yaw) * dt
                traj[t] = [curr_x, curr_y]
        elif behavior == "weave":
            # Two-wheeler weaving between lanes
            freq = random.uniform(0.8, 1.5)
            for t in range(total_steps):
                curr_y += 0.35 * math.sin(freq * t * dt)
                curr_x += curr_v * dt
                traj[t] = [curr_x, curr_y]
        elif behavior == "cross":
            # Pedestrian crossing perpendicularly
            cross_v = random.uniform(0.9, 1.4)
            for t in range(total_steps):
                curr_y += cross_v * dt
                traj[t] = [curr_x, curr_y]
        else: # Standard vehicle straight or turning
            turn_rate = random.uniform(-0.08, 0.08)
            for t in range(total_steps):
                curr_yaw += turn_rate * dt
                curr_x += curr_v * math.cos(curr_yaw) * dt
                curr_y += curr_v * math.sin(curr_yaw) * dt
                traj[t] = [curr_x, curr_y]

        # Add realistic sensor observation noise to history
        obs_noise = np.random.normal(0, 0.04, (obs_steps, 2))
        x_obs = traj[:obs_steps] + obs_noise
        y_fut = traj[obs_steps:]

        x_obs_list.append(x_obs)
        y_fut_list.append(y_fut)
        classes_list.append(cls_name)

    return np.array(x_obs_list), np.array(y_fut_list), classes_list


def create_and_save_trajectory_dataset(output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    random.seed(42)
    np.random.seed(42)

    print(f"Synthesizing Indian road trajectory dataset in {output_dir}...")
    x_train, y_train, c_train = generate_scenario_trajectories(num_samples=300)
    x_val, y_val, c_val = generate_scenario_trajectories(num_samples=80)
    x_test, y_test, c_test = generate_scenario_trajectories(num_samples=50)

    train_file = output_dir / "trajectories_train.npz"
    val_file = output_dir / "trajectories_val.npz"
    test_file = output_dir / "trajectories_test.npz"

    np.savez_compressed(train_file, x_obs=x_train, y_future=y_train, classes=c_train)
    np.savez_compressed(val_file, x_obs=x_val, y_future=y_val, classes=c_val)
    np.savez_compressed(test_file, x_obs=x_test, y_future=y_test, classes=c_test)

    print(f"Saved: {train_file.name} (300 seqs), {val_file.name} (80 seqs), {test_file.name} (50 seqs).")


if __name__ == "__main__":
    root = Path(__file__).resolve().parent.parent.parent
    ds_dir = root / "dataset" / "trajectories"
    create_and_save_trajectory_dataset(ds_dir)
