"""PART 1 -- the browser simulator's web server.

All web code lives HERE, never in shared/fleetx_core/. This file is allowed to
know about HTTP; the brain is not.

Uses only what ships inside Python: http.server, threading, json.
No pip install. No npm install.

Three things it serves:
  GET  /              the dashboard page
  GET  /api/map       the warehouse map (sent once, it never changes)
  GET  /api/stream    a live feed of the fleet, ~20 updates a second
  POST /api/goal      "robot R1, go to square (x, y)"
"""

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

# Let this file import the shared brain that lives one folder up.
_HERE = os.path.dirname(os.path.abspath(__file__))
_SHARED = os.path.join(os.path.dirname(_HERE), "shared")
if _SHARED not in sys.path:
    sys.path.insert(0, _SHARED)

from fleetx_core import Cell, World, phase1_world, phase2_world   # noqa: E402
from comparison import Comparison   # noqa: E402
from scenarios import Scenarios   # noqa: E402

WEB_DIR = os.path.join(_HERE, "web")

# A stamp for "which version of the code is this?".
#
# Worth the twenty lines. The dashboard keeps its connection open for hours, so
# a tab opened before an edit carries on running the OLD JavaScript for ever --
# a fixed button stays broken on screen while every test passes. And the server
# itself loads the brain once at startup, so editing a .py file changes nothing
# until it is restarted. Both are invisible without this.
_STAMP_FILES = [
    os.path.join(WEB_DIR, "index.html"),
    os.path.join(WEB_DIR, "compare.html"),
    os.path.join(_HERE, "server.py"),
    os.path.join(_HERE, "scenarios.py"),
    os.path.join(_SHARED, "fleetx_core", "robot.py"),
    os.path.join(_SHARED, "fleetx_core", "world.py"),
]


def _build_stamp() -> str:
    """Newest modification time across the files that matter, as HH:MM:SS."""
    newest = 0.0
    for path in _STAMP_FILES:
        try:
            newest = max(newest, os.path.getmtime(path))
        except OSError:
            pass
    return time.strftime("%H:%M:%S", time.localtime(newest))


# What the code on disk looked like when this server process started. If the
# live stamp ever differs from this, the server itself is out of date.
SERVER_BUILD = _build_stamp()

TICK_HZ = 20.0            # how many times a second the world moves
STREAM_HZ = 20.0          # how many times a second the browser is updated


class ComparisonRunner:
    """Runs BOTH fleets forward on their own thread, in lockstep.

    Completely separate from the single-fleet Simulation above, so the ordinary
    dashboard keeps working exactly as it did whatever this does.
    """

    def __init__(self):
        self.comparison = Comparison(robots=5, every=2.5, seed=1)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="compare", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        dt = 1.0 / TICK_HZ
        next_tick = time.perf_counter()
        while not self._stop.is_set():
            with self.lock:
                # The speed control just does more steps per real second. The
                # clock on screen is simulated time either way, so nothing is
                # being fudged -- it is the same run, watched faster.
                for _ in range(self.comparison.speed):
                    self.comparison.tick(dt)
            next_tick += dt
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def snapshot(self) -> dict:
        with self.lock:
            return self.comparison.snapshot()

    def command(self, action: str, **kwargs) -> dict:
        with self.lock:
            c = self.comparison
            if action == "start":
                return {"ok": True, "message": c.start()}
            if action == "pause":
                return {"ok": True, "message": c.pause()}
            if action == "reset":
                msg = c.reset(robots=kwargs.get("robots"), every=kwargs.get("every"))
                return {"ok": True, "message": msg}
            if action == "speed":
                return {"ok": True, "message": c.set_speed(int(kwargs.get("speed", 1)))}
            return {"ok": False, "message": f"No such action: {action}"}


class Simulation:
    """Runs the world forward on a background thread, safely."""

    def __init__(self, world: World):
        self.world = world
        self.scenarios = Scenarios(world)
        # Which robot the dashboard is currently showing in detail. Only that
        # one's full picture is put on the wire.
        self.focus: Optional[str] = None
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        dt = 1.0 / TICK_HZ
        next_tick = time.perf_counter()
        while not self._stop.is_set():
            with self.lock:
                self.scenarios.keep_busy()
                self.world.tick(dt)
            next_tick += dt
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()   # we fell behind, catch up

    def snapshot(self) -> dict:
        with self.lock:
            snap = self.world.snapshot(focus=self.focus)
            snap["build"] = _build_stamp()
            snap["server_build"] = SERVER_BUILD
            snap["scenario"] = self.scenarios.name
            snap["order_every"] = (self.scenarios.orders.every
                                   if self.scenarios.orders else None)
            snap["auto"] = self.scenarios.auto
            snap["orders_running"] = self.scenarios.orders is not None
            return snap

    def map_data(self) -> dict:
        with self.lock:
            data = self.world.grid.to_dict()
            # Phase 11: the rack names travel with the map, which is sent once
            # and never changes. They do not belong in the 20-a-second feed.
            data["shelves"] = self.world.inventory.to_dict()["shelves"]
            return data

    def set_goal(self, robot_id: str, x: int, y: int) -> dict:
        """Send a robot somewhere. Returns a short plain-English result."""
        with self.lock:
            robot = self.world.get(robot_id)
            if robot is None:
                return {"ok": False, "message": f"There is no robot called {robot_id}."}
            target = Cell(int(x), int(y))
            if not self.world.grid.is_walkable(target):
                return {"ok": False, "message": "That square is a shelf, nothing can drive there."}
            # Deliberately sending this robot somewhere releases its stop
            # button. Anything else would look broken: you click a square, the
            # robot lights up a route, and then just sits there.
            was_halted = robot.halted
            robot.resume()
            # You have overridden it, so it is no longer going to charge. Hand
            # the bay back rather than sitting on a booking it will not use.
            if robot.charger is not None:
                robot._leave_charger(self.world.bus, self.world.sim_time)
            robot.set_goal(target)
            return {"ok": True, "message": f"{robot_id} is heading to ({x}, {y})."
                    + (" (released from Stop all)" if was_halted else "")}


    def run_scenario(self, name: str, **kwargs) -> dict:
        """Switch the demo to a named setup, e.g. 'head_on'."""
        with self.lock:
            message = self.scenarios.apply(name, **kwargs)
            ok = not message.startswith("There is no scenario")
            return {"ok": ok, "message": message}

    def silence(self, robot_id: str, on: bool) -> dict:
        """Cut or restore a robot's transmitter.

        The robot keeps driving. It just stops talking, so the others slowly
        realise they have not heard from it. Tests the heartbeat timeout from
        03_ROBOT_AND_ROS2 section 5.
        """
        with self.lock:
            if self.world.get(robot_id) is None:
                return {"ok": False, "message": f"There is no robot called {robot_id}."}
            if not hasattr(self.world.bus, "silence"):
                return {"ok": False, "message": "This bus cannot be silenced."}
            self.world.bus.silence(robot_id, on)
            word = "silenced" if on else "talking again"
            return {"ok": True, "message": f"{robot_id} is {word}."}

    def drain_batteries(self) -> dict:
        """Run the fleet flat so the charging demo happens now, not in 20
        minutes. Twenty robots wanting four bays is the interesting case."""
        with self.lock:
            return self.world.drain_batteries()

    def set_focus(self, robot_id) -> dict:
        """The dashboard says which robot it is showing, so we can stop sending
        the other nineteen robots' worth of detail that it throws away."""
        with self.lock:
            self.focus = robot_id if robot_id in self.world.robots else None
            return {"ok": True, "message": f"Showing {self.focus or 'nobody'}."}

    def order_rate(self, every: float) -> dict:
        """How fast orders come in."""
        with self.lock:
            return {"ok": True, "message": self.scenarios.set_order_rate(every)}

    def fleet_size(self, action: str, robot_id=None) -> dict:
        """Add or remove a robot while the warehouse is running."""
        with self.lock:
            if action == "add":
                return self.world.add_robot_live()
            if action == "remove":
                return self.world.remove_robot(robot_id)
            return {"ok": False, "message": f"No such action: {action}"}

    def cut_network(self, down: bool) -> dict:
        """Pull the plug on the radio, or plug it back in.

        Everything the robots need for SAFETY keeps working: their sensors see
        each other whether the network is up or not. What stops is coordination
        and new orders.
        """
        with self.lock:
            bus = self.world.bus
            if not hasattr(bus, "packet_loss"):
                return {"ok": False, "message": "This bus cannot be cut."}
            bus.packet_loss = 1.0 if down else 0.0
            return {"ok": True, "message": (
                "NETWORK DOWN. Robots finish what they are carrying, on sensors alone."
                if down else "Network restored. Robots resync and orders resume.")}

    def set_network(self, loss: float, latency_ms: float) -> dict:
        """Make the radio link worse or better."""
        with self.lock:
            bus = self.world.bus
            if not hasattr(bus, "packet_loss"):
                return {"ok": False, "message": "This bus has no adjustable quality."}
            bus.packet_loss = max(0.0, min(1.0, float(loss)))
            bus.latency = max(0.0, float(latency_ms)) / 1000.0
            bus.jitter = bus.latency * 0.5
            pct = round(bus.packet_loss * 100)
            return {"ok": True,
                    "message": f"Network: {pct}% of messages lost, {round(latency_ms)}ms delay."}

    def robot_power(self, robot_id: str, alive: bool) -> dict:
        """Switch a robot off mid-job, or bring it back. 08_SIMULATION Sc. 6."""
        with self.lock:
            if alive:
                return self.world.revive_robot(robot_id)
            return self.world.fail_robot(robot_id)

    def obstacle(self, action: str, x: int = 0, y: int = 0) -> dict:
        """Drop something in an aisle, pick it up again, or clear the lot.

        The robots are NOT told. They find out by driving close enough to see
        it, which is the whole point.
        """
        with self.lock:
            if action == "clear":
                return self.world.clear_obstacles()
            cell = Cell(int(x), int(y))
            if action == "remove":
                return self.world.remove_obstacle(cell)
            if cell in self.world.obstacles:
                return self.world.remove_obstacle(cell)     # click again = pick up
            return self.world.add_obstacle(cell)

    def reset(self) -> dict:
        """Zero the scoreboard without moving anybody."""
        with self.lock:
            self.world.reset_counters()
            return {"ok": True, "message": "Counters reset to zero."}


class Handler(BaseHTTPRequestHandler):
    """Answers the browser. Held on the server as `sim`."""

    sim: Simulation = None          # type: ignore[assignment]
    compare: ComparisonRunner = None    # type: ignore[assignment]
    protocol_version = "HTTP/1.1"

    # Keep the terminal quiet -- otherwise every frame prints a line.
    def log_message(self, fmt, *args):
        pass

    # ------------------------------------------------------------------ GET

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send_file(os.path.join(WEB_DIR, "index.html"),
                            "text/html; charset=utf-8", stamp=True)
        elif self.path in ("/compare", "/compare.html"):
            self._send_file(os.path.join(WEB_DIR, "compare.html"), "text/html; charset=utf-8")
        elif self.path == "/api/compare/stream":
            self._send_stream(self.compare.snapshot)
        elif self.path == "/api/map":
            self._send_json(self.sim.map_data())
        elif self.path == "/api/state":
            self._send_json(self.sim.snapshot())
        elif self.path == "/api/stream":
            self._send_stream(self.sim.snapshot)
        else:
            self._send_json({"error": "not found"}, status=404)

    # ----------------------------------------------------------------- POST

    def do_POST(self):
        if self.path not in ("/api/goal", "/api/scenario", "/api/reset",
                             "/api/silence", "/api/network", "/api/obstacle",
                             "/api/power", "/api/compare", "/api/cut",
                             "/api/fleet", "/api/rate", "/api/focus",
                             "/api/battery"):
            self._send_json({"error": "not found"}, status=404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length) or b"{}")

            if self.path == "/api/compare":
                result = self.compare.command(
                    body.get("action", "start"),
                    robots=body.get("robots"), every=body.get("every"),
                    speed=body.get("speed", 1))
            elif self.path == "/api/goal":
                result = self.sim.set_goal(
                    body.get("robot_id", "R1"), body["x"], body["y"]
                )
            elif self.path == "/api/scenario":
                extra = {k: body[k] for k in ("seed", "every", "limit") if k in body}
                result = self.sim.run_scenario(body.get("name", "patrol"), **extra)
            elif self.path == "/api/power":
                result = self.sim.robot_power(body.get("robot_id", "R1"),
                                              bool(body.get("alive", False)))
            elif self.path == "/api/silence":
                result = self.sim.silence(body.get("robot_id", "R1"),
                                          bool(body.get("on", True)))
            elif self.path == "/api/obstacle":
                result = self.sim.obstacle(body.get("action", "toggle"),
                                           body.get("x", 0), body.get("y", 0))
            elif self.path == "/api/battery":
                result = self.sim.drain_batteries()
            elif self.path == "/api/focus":
                result = self.sim.set_focus(body.get("robot_id"))
            elif self.path == "/api/rate":
                result = self.sim.order_rate(body.get("every", 4.0))
            elif self.path == "/api/fleet":
                result = self.sim.fleet_size(body.get("action", "add"),
                                             body.get("robot_id"))
            elif self.path == "/api/cut":
                result = self.sim.cut_network(bool(body.get("down", True)))
            elif self.path == "/api/network":
                result = self.sim.set_network(body.get("loss", 0.0),
                                              body.get("latency_ms", 0.0))
            else:
                result = self.sim.reset()

            self._send_json(result)
        except Exception as exc:  # noqa: BLE001 -- report, never crash the server
            self._send_json({"ok": False, "message": f"Bad request: {exc}"}, status=400)

    # -------------------------------------------------------------- helpers

    def _send_json(self, payload: dict, status: int = 200):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path: str, content_type: str, stamp: bool = False):
        try:
            with open(path, "rb") as fh:
                data = fh.read()
            if stamp:
                # Bake in the version this page was served at, so the page can
                # notice for itself when it has gone stale.
                data = data.replace(b"__PAGE_BUILD__", _build_stamp().encode())
        except OSError:
            self._send_json({"error": f"missing file {path}"}, status=500)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_stream(self, source):
        """Server-Sent Events: keep the line open and keep talking.

        The browser side of this is one line of JavaScript: new EventSource().
        Both ends are built in, so nothing needs installing.
        """
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        interval = 1.0 / STREAM_HZ
        try:
            while True:
                payload = json.dumps(source())
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            # The tab was closed or refreshed. Normal, not an error.
            return


def serve(world: Optional[World] = None, port: int = 8000, host: str = "127.0.0.1"):
    """Start the simulation and the web server. Blocks until Ctrl+C."""
    sim = Simulation(world if world is not None else phase2_world())
    sim.start()

    compare = ComparisonRunner()
    compare.start()

    Handler.sim = sim
    Handler.compare = compare

    # If 8000 is busy, walk up until we find a free port.
    server = None
    for candidate in range(port, port + 20):
        try:
            server = ThreadingHTTPServer((host, candidate), Handler)
            port = candidate
            break
        except OSError:
            continue
    if server is None:
        raise SystemExit(
            f"Could not find a free port between {port} and {port + 19}. "
            "Something else is using them all."
        )

    server.daemon_threads = True
    url = f"http://localhost:{port}"

    banner = (
        "\n"
        "  FLEET-X  ---  Part 1, Phase 14: keeps working when the network dies\n"
        "  " + "-" * 58 + "\n"
        f"  Open this in your browser:   {url}\n"
        f"  Side-by-side comparison:     {url}/compare\n"
        "  Click a robot card to select it, then click a floor square to send it.\n"
        f"  Code version:                {SERVER_BUILD}\n"
        "  (if the page shows a different version, reload the tab)\n"
        "  Press Ctrl+C here to stop.\n"
    )
    print(banner, flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  Stopping FLEET-X. Bye.\n")
    finally:
        sim.stop()
        compare.stop()
        server.server_close()
