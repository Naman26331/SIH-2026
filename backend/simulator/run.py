#!/usr/bin/env python3
"""Start FLEET-X Part 1 (the browser simulator).

    uv run backend/run.py

Nothing to install. Uses only what already ships with Python.
"""

import sys

if sys.version_info < (3, 9):
    raise SystemExit("FLEET-X needs Python 3.9 or newer. Try: python3 --version")

from server import serve   # noqa: E402

if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    serve(port=port)
