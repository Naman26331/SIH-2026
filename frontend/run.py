#!/usr/bin/env python3
"""Serve dashboard locally and proxy API traffic to FLEET-X backend."""

import http.client
import os
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
        if self.path.startswith("/api/"):
            self._proxy()
        elif self.path == "/compare":
            self.path = "/compare.html"
            super().do_GET()
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


if __name__ == "__main__":
    print(f"Dashboard: http://localhost:{PORT}")
    print(f"Backend:   {BACKEND.geturl()}")
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
