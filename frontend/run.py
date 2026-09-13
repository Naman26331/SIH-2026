#!/usr/bin/env python3
"""Serve dashboard locally and proxy API traffic to FLEET-X backend."""

import http.client
import os
import select
import socket
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

HERE = os.path.dirname(os.path.abspath(__file__))
DASHBOARD = os.path.join(HERE, "dashboard")
BACKEND = urlsplit(sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8000")
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 3000


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DASHBOARD, **kwargs)

    def do_GET(self):
        if (self.path == "/api/ws"
                and self.headers.get("Upgrade", "").lower() == "websocket"):
            self._proxy_websocket()
        elif self.path.startswith("/api/"):
            self._proxy()
        else:
            super().do_GET()

    def do_POST(self):
        self._proxy()

    def _proxy(self):
        body = None
        length = int(self.headers.get("Content-Length", "0"))
        if length:
            body = self.rfile.read(length)

        connection = http.client.HTTPConnection(BACKEND.hostname, BACKEND.port or 80, timeout=60)
        headers = {"Content-Type": self.headers.get("Content-Type", "application/json")}
        try:
            connection.request(self.command, self.path, body=body, headers=headers)
            response = connection.getresponse()
            self.send_response(response.status)
            for name, value in response.getheaders():
                if name.lower() not in {"connection", "transfer-encoding", "content-length"}:
                    self.send_header(name, value)
            self.end_headers()
            while chunk := response.read(8192):
                self.wfile.write(chunk)
                self.wfile.flush()
        except (OSError, http.client.HTTPException) as error:
            self.send_error(502, f"Backend unavailable: {error}")
        finally:
            connection.close()

    def _proxy_websocket(self):
        """Tunnel upgraded WebSocket bytes between browser and backend."""
        upstream = None
        try:
            upstream = socket.create_connection(
                (BACKEND.hostname, BACKEND.port or 80), timeout=10)
            lines = [
                f"GET {self.path} HTTP/1.1",
                f"Host: {BACKEND.hostname}:{BACKEND.port or 80}",
                "Upgrade: websocket",
                "Connection: Upgrade",
                f"Sec-WebSocket-Key: {self.headers['Sec-WebSocket-Key']}",
                f"Sec-WebSocket-Version: {self.headers.get('Sec-WebSocket-Version', '13')}",
            ]
            origin = self.headers.get("Origin")
            if origin:
                lines.append(f"Origin: {origin}")
            upstream.sendall(("\r\n".join(lines) + "\r\n\r\n").encode("ascii"))

            response = b""
            while b"\r\n\r\n" not in response:
                chunk = upstream.recv(4096)
                if not chunk:
                    raise ConnectionError("backend closed WebSocket handshake")
                response += chunk
                if len(response) > 65536:
                    raise ConnectionError("oversized WebSocket handshake")
            self.connection.sendall(response)
            if not response.startswith(b"HTTP/1.1 101"):
                return

            self.close_connection = True
            upstream.settimeout(None)
            self.connection.settimeout(None)
            sockets = (self.connection, upstream)
            while True:
                readable, _, _ = select.select(sockets, [], [], 60.0)
                if not readable:
                    continue
                for source in readable:
                    data = source.recv(65536)
                    if not data:
                        return
                    target = upstream if source is self.connection else self.connection
                    target.sendall(data)
        except (OSError, http.client.HTTPException) as error:
            if upstream is None:
                self.send_error(502, f"Backend unavailable: {error}")
        finally:
            if upstream is not None:
                upstream.close()


if __name__ == "__main__":
    print(f"Dashboard: http://localhost:{PORT}")
    print(f"Backend:   {BACKEND.geturl()}")
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
