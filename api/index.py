"""
BharatDrive-AI: Vercel Serverless Function Gateway Entrypoint.
Exposes the Flask WSGI handler for Vercel's Python runtime.

BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# Resolve project root and python lib directories
ROOT_DIR = Path(__file__).resolve().parent.parent
PYTHON_DIR = ROOT_DIR / "python"

# Ensure root and python directories are in sys.path
for path in [str(ROOT_DIR), str(PYTHON_DIR)]:
    if path not in sys.path:
        sys.path.insert(0, path)

# Import the production Flask application and handler
from app import app, handler

# Export for Vercel WSGI/Serverless runtime
__all__ = ["app", "handler"]
