#!/usr/bin/env python3
"""Run FLEET-X backend API in simulator or explicit ROS 2 mode."""

import argparse
import os
import sys
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMULATOR = os.path.join(ROOT, "backend", "simulator")
sys.path.insert(0, SIMULATOR)

from server import serve  # noqa: E402


def env_flag(name: str, default: bool) -> bool:
    """Read strict boolean environment flag; reject hidden typos."""
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise SystemExit(
        f"{name} must be true or false, got {raw!r}.")


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--ros2", action="store_true",
                      help="override FLEETX_SIMULATION and require live ROS 2")
    mode.add_argument("--simulation", action="store_true",
                      help="override FLEETX_SIMULATION and run Python simulator")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    simulation = env_flag("FLEETX_SIMULATION", default=True)
    if args.ros2:
        simulation = False
    elif args.simulation:
        simulation = True

    if simulation:
        serve(port=args.port, host="0.0.0.0")
        return

    try:
        import rclpy
        from fleet_agent.gateway_node import Ros2GatewayNode
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 mode unavailable. Source /opt/ros/$ROS_DISTRO/setup.bash and "
            "backend/ros2_ws/install/setup.bash first.\n"
            f"Import error: {exc}"
        ) from exc

    rclpy.init()
    gateway = Ros2GatewayNode()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(gateway,), name="ros2-gateway", daemon=True)
    spin_thread.start()
    try:
        serve(port=args.port, host="0.0.0.0", gateway=gateway)
    finally:
        gateway.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
