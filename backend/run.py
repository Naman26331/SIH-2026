#!/usr/bin/env python3
"""Run FLEET-X backend API and simulator."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMULATOR = os.path.join(ROOT, "backend", "simulator")
sys.path.insert(0, SIMULATOR)

from server import serve  # noqa: E402


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    serve(port=port, host="0.0.0.0")
