"""Comparative Evaluation of Trajectory Predictors on Indian Driving Test Sequences.
Calculates ADE@1s, ADE@3s, ADE@5s, FDE@5s, and Inference Latency across LSTM, GRU, and Transformer.
"""

from pathlib import Path
import sys
import time
import numpy as np

_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from trajectory_prediction.models.physics_kalman_predictor import CTRAPhysicsPredictor
from trajectory_prediction.models.lstm_predictor import LSTMTrajectoryPredictor
from trajectory_prediction.models.gru_predictor import GRUTrajectoryPredictor
from trajectory_prediction.models.transformer_predictor import SpatioTemporalTransformerPredictor


def evaluate_model(model, x_test: np.ndarray, y_test: np.ndarray, classes: list[str]) -> dict:
    num_samples = len(x_test)
    ade_1s_list = []
    ade_3s_list = []
    ade_5s_list = []
    fde_5s_list = []

    start_t = time.perf_counter()
    for i in range(num_samples):
        # 50 steps at 10 Hz = 5.0 seconds
        pred = model.predict(x_test[i], agent_class=classes[i])
        gt = y_test[i]

        diff = np.linalg.norm(pred - gt, axis=-1)
        ade_1s_list.append(np.mean(diff[:10]))  # 1.0s (steps 1-10)
        ade_3s_list.append(np.mean(diff[:30]))  # 3.0s (steps 1-30)
        ade_5s_list.append(np.mean(diff[:50]))  # 5.0s (steps 1-50)
        fde_5s_list.append(diff[-1])            # 5.0s final point

    total_time_ms = (time.perf_counter() - start_t) * 1000.0
    latency_per_agent = total_time_ms / float(num_samples)

    return {
        "ade_1s": round(float(np.mean(ade_1s_list)), 2),
        "ade_3s": round(float(np.mean(ade_3s_list)), 2),
        "ade_5s": round(float(np.mean(ade_5s_list)), 2),
        "fde_5s": round(float(np.mean(fde_5s_list)), 2),
        "latency_ms": round(float(latency_per_agent), 2),
    }


def run_benchmark():
    root = Path(__file__).resolve().parent.parent.parent
    test_path = root / "dataset" / "trajectories" / "trajectories_test.npz"

    if not test_path.exists():
        from trajectory_prediction.training.dataset_generator import create_and_save_trajectory_dataset
        create_and_save_trajectory_dataset(root / "dataset" / "trajectories")

    data = np.load(test_path, allow_pickle=True)
    x_test, y_test, c_test = data["x_obs"], data["y_future"], data["classes"]

    print("=========================================================================================")
    print("   BHARATDRIVE-AI: 5-SECOND TRAJECTORY PREDICTOR BENCHMARK (TEST SPLIT)                  ")
    print("=========================================================================================")
    print(f"Test Sequences: {len(x_test)} | History: 2.0s (20 steps) | Horizon: 5.0s (50 steps)")
    print("-----------------------------------------------------------------------------------------")

    models = {
        "Kinematic CTRA Baseline": CTRAPhysicsPredictor(),
        "LSTM Encoder-Decoder": LSTMTrajectoryPredictor(),
        "GRU Encoder-Decoder": GRUTrajectoryPredictor(),
        "Spatio-Temporal Transformer": SpatioTemporalTransformerPredictor(),
    }

    results = {}
    for name, m in models.items():
        results[name] = evaluate_model(m, x_test, y_test, c_test)

    print(f"{'Model Architecture':<30} | {'ADE@1s (m)':<10} | {'ADE@3s (m)':<10} | {'ADE@5s (m)':<10} | {'FDE@5s (m)':<10} | {'Latency (ms)':<12}")
    print("-----------------------------------------------------------------------------------------")
    for name, r in results.items():
        print(f"{name:<30} | {r['ade_1s']:<10.2f} | {r['ade_3s']:<10.2f} | {r['ade_5s']:<10.2f} | {r['fde_5s']:<10.2f} | {r['latency_ms']:<12.2f}")
    print("=========================================================================================")
    return results


if __name__ == "__main__":
    run_benchmark()
