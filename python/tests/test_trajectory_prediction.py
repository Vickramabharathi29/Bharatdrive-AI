"""Unit tests for 5-second trajectory prediction models (LSTM, GRU, Transformer, CTRA)."""

import numpy as np
import pytest

from trajectory_prediction.models.physics_kalman_predictor import CTRAPhysicsPredictor
from trajectory_prediction.models.lstm_predictor import LSTMTrajectoryPredictor
from trajectory_prediction.models.gru_predictor import GRUTrajectoryPredictor
from trajectory_prediction.models.transformer_predictor import SpatioTemporalTransformerPredictor
from trajectory_prediction.training.train_predictor import compute_min_of_n_loss


@pytest.fixture
def sample_history():
    # 20 steps (2.0s) of straight motion at 10 m/s
    t = np.linspace(0, 1.9, 20)
    x = 10.0 * t
    y = np.zeros_like(x)
    return np.column_stack([x, y])


def test_ctra_predictor(sample_history):
    predictor = CTRAPhysicsPredictor(obs_horizon_s=2.0, pred_horizon_s=5.0, dt=0.1)
    future = predictor.predict(sample_history, agent_class="Car")

    # Horizon check: 50 steps
    assert future.shape == (50, 2)
    assert not np.isnan(future).any()
    # At 10 m/s for 5.0 seconds, agent should advance ~50 meters along X
    total_dx = future[-1, 0] - sample_history[-1, 0]
    assert total_dx == pytest.approx(50.0, rel=0.1)

    # Stationary cattle test
    cattle_future = predictor.predict(sample_history, agent_class="Cattle")
    assert np.allclose(cattle_future, sample_history[-1])


def test_lstm_predictor(sample_history):
    lstm = LSTMTrajectoryPredictor(obs_horizon_s=2.0, pred_horizon_s=5.0, dt=0.1)
    future = lstm.predict(sample_history, agent_class="Two-Wheeler")

    assert future.shape == (50, 2)
    assert not np.isnan(future).any()
    assert future[-1, 0] > sample_history[-1, 0]


def test_gru_predictor(sample_history):
    gru = GRUTrajectoryPredictor(obs_horizon_s=2.0, pred_horizon_s=5.0, dt=0.1)
    future = gru.predict(sample_history, agent_class="Auto-Rickshaw")

    assert future.shape == (50, 2)
    assert not np.isnan(future).any()
    assert future[-1, 0] > sample_history[-1, 0]


def test_transformer_predictor(sample_history):
    transformer = SpatioTemporalTransformerPredictor(obs_horizon_s=2.0, pred_horizon_s=5.0, dt=0.1)
    future = transformer.predict(sample_history, agent_class="Auto-Rickshaw")

    assert future.shape == (50, 2)
    assert not np.isnan(future).any()

    # Multimodal test (K=3 modes)
    multi_fut, probs = transformer.predict_multimodal(sample_history, num_modes=3, agent_class="Auto-Rickshaw")
    assert multi_fut.shape == (3, 50, 2)
    assert len(probs) == 3
    assert np.sum(probs) == pytest.approx(1.0, abs=1e-4)


def test_min_of_n_loss_function():
    # 3 modes, 50 steps
    K = 3
    T_pred = 50
    preds = np.zeros((K, T_pred, 2))
    # Mode 0 has small error, Mode 1 has large error, Mode 2 has medium error
    gt = np.ones((T_pred, 2)) * 10.0
    preds[0] = gt + 0.1 # Closest mode
    preds[1] = gt + 5.0
    preds[2] = gt + 2.0
    probs = np.array([0.7, 0.2, 0.1])

    loss, best_k, ade, fde = compute_min_of_n_loss(preds, probs, gt)
    assert best_k == 0
    assert ade == pytest.approx(np.hypot(0.1, 0.1), abs=1e-3)
    assert loss > 0.0
