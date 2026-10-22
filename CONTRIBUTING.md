# Contributing to BharatDrive-AI

Thank you for your interest in contributing to **BharatDrive-AI**! We welcome contributions from researchers, automotive software engineers, and robotics enthusiasts.

---

## 1. Development Environment Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-org/Bharatdrive-AI.git
   cd Bharatdrive-AI
   ```
2. **Create a virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   pip install pytest pytest-cov
   ```
3. **Verify the test suite**:
   ```bash
   python -m pytest python/tests -v
   ```

---

## 2. Code Style & Standards

- **Python**: Follow PEP 8 guidelines. Use strict type annotations (`from __future__ import annotations`).
- **MATLAB**: Adhere to MathWorks style guidelines. Class files use `classdef`, functions include descriptive help comments, and all scripts must run non-interactively with clean figures.
- **Git Commit Messages**: Use Conventional Commits format:
  - `feat: add ST-GNN intent predictor`
  - `fix: correct lateral jerk bounds in Frenet optimizer`
  - `docs: update setup guide for MATLAB R2025b`
  - `test: add unit test for pothole clustering`

---

## 3. Pull Request Process

1. Fork the repo and create your feature branch: `git checkout -b feat/my-feature`.
2. Ensure **all unit tests pass**: `python -m pytest python/tests`.
3. If introducing new algorithms, write comprehensive unit tests in `python/tests/test_<module>.py`.
4. Submit a pull request detailing the motivation, algorithm formulation, and test results.
