# Frontend

```text
dashboard/  2D warehouse observation UI
run.py      frontend server and backend API proxy
```

Run dashboard against a local backend:

```text
uv run frontend/run.py
```

For a backend on Raspberry Pi:

```text
uv run frontend/run.py http://PI_IP:8000
```

Open `http://localhost:3000`.

Frontend contains no simulation or robot logic.
