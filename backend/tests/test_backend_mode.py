"""Backend mode flag contract."""

import importlib.util
import os
from pathlib import Path

import pytest


RUN = Path(__file__).parents[1] / "run.py"
SPEC = importlib.util.spec_from_file_location("fleetx_backend_run", RUN)
backend_run = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(backend_run)


def test_simulation_is_safe_default(monkeypatch):
    monkeypatch.delenv("FLEETX_SIMULATION", raising=False)
    assert backend_run.env_flag("FLEETX_SIMULATION", default=True)


@pytest.mark.parametrize("value", ["true", "TRUE", "1", "yes", "on"])
def test_simulation_true_values(monkeypatch, value):
    monkeypatch.setenv("FLEETX_SIMULATION", value)
    assert backend_run.env_flag("FLEETX_SIMULATION", default=False)


@pytest.mark.parametrize("value", ["false", "FALSE", "0", "no", "off"])
def test_ros_values(monkeypatch, value):
    monkeypatch.setenv("FLEETX_SIMULATION", value)
    assert not backend_run.env_flag("FLEETX_SIMULATION", default=True)


def test_invalid_value_fails_loudly(monkeypatch):
    monkeypatch.setenv("FLEETX_SIMULATION", "ture")
    with pytest.raises(SystemExit, match="must be true or false"):
        backend_run.env_flag("FLEETX_SIMULATION", default=True)
