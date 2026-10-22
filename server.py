"""
BharatDrive-AI: Server Entrypoint (server.py).
Forwards to production Flask application instance.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
PYTHON_DIR = ROOT_DIR / "python"

for path in [str(ROOT_DIR), str(PYTHON_DIR)]:
    if path not in sys.path:
        sys.path.insert(0, path)

from app import app, handler, application

__all__ = ["app", "handler", "application"]

if __name__ == "__main__":
    import os
    port = int(os.getenv("PORT", 5000))
    app.run(host="0.0.0.0", port=port)
