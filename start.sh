#!/usr/bin/env bash
# Render entry point: run the backend API privately and the dashboard/proxy
# publicly on Render's assigned $PORT, in one Web Service.
set -euo pipefail

BACKEND_PORT="${BACKEND_PORT:-8001}"

python3 backend/run.py --port "$BACKEND_PORT" &
BACKEND_PID=$!
trap 'kill "$BACKEND_PID" 2>/dev/null || true' EXIT

exec python3 frontend/run.py "http://127.0.0.1:${BACKEND_PORT}" "${PORT:-3000}"
