"""Local web server for the station: the page, the API, and the camera streams.

Plain ``http.server`` on purpose. A dependency that a child has to install before
the robot works is a dependency that stops the robot working, and the whole point
of this folder is that one command is enough.

Nothing binds to anything but 127.0.0.1. The page can drive both arms, so it is
not something to expose to the network.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
UI_PATH = HERE / "ui.html"

BOUNDARY = "nexarmframe"


def free_port(preferred: int = 8123) -> int:
    """Take the preferred port if it is free, otherwise let the OS pick one."""
    for candidate in (preferred, preferred + 1, preferred + 2):
        with socket.socket() as s:
            try:
                s.bind(("127.0.0.1", candidate))
                return candidate
            except OSError:
                continue
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def make_handler(station):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        # The default logger prints a line per request, and the page polls twice a
        # second. That would bury the messages a child is meant to read.
        def log_message(self, fmt, *args):
            pass

        # ------------------------------------------------------------ helpers

        def _send_json(self, obj, code=200):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, body: bytes, ctype: str, code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length") or 0)
            if not length:
                return {}
            try:
                return json.loads(self.rfile.read(length).decode("utf-8"))
            except Exception:
                return {}

        # ---------------------------------------------------------------- GET

        def do_GET(self):
            path = self.path.split("?", 1)[0]

            if path in ("/", "/index.html"):
                self._send_bytes(UI_PATH.read_bytes(), "text/html; charset=utf-8")
                return

            if path == "/api/state":
                self._send_json(station.snapshot())
                return

            if path.startswith("/shot/"):
                name = path[len("/shot/"):].removesuffix(".jpg")
                jpeg = station.latest_jpeg(name)
                if jpeg is None:
                    self._send_json({"why": "no frame yet"}, code=404)
                else:
                    self._send_bytes(jpeg, "image/jpeg")
                return

            if path.startswith("/stream/"):
                self._stream(path[len("/stream/"):])
                return

            self._send_json({"why": "not found"}, code=404)

        def _stream(self, name: str):
            """MJPEG: the one video format every browser plays from a plain socket."""
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = None
            try:
                while True:
                    jpeg = station.latest_jpeg(name)
                    if jpeg is not None and jpeg is not last:
                        last = jpeg
                        self.wfile.write(
                            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1 / 25)
            except (BrokenPipeError, ConnectionResetError, OSError):
                # The tab was closed or the picture was swapped out. Not an error.
                pass

        # --------------------------------------------------------------- POST

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            body = self._read_json()

            if path == "/api/detect/start":
                self._send_json(station.begin_arm_detect(kind=str(body.get("kind") or "")))
            elif path == "/api/prompt/next":
                self._send_json(station.prompt_next())
            elif path == "/api/detect/poll":
                self._send_json(station.poll_arm_detect())
            elif path == "/api/cameras/swap":
                self._send_json(station.swap_cameras())
            elif path == "/api/teleop/start":
                self._send_json(station.start_teleop())
            elif path == "/api/record/start":
                self._send_json(station.start_record(task=str(body.get("task", ""))))
            elif path == "/api/record/next":
                station.events["exit_early"] = True
                self._send_json({"ok": True})
            elif path == "/api/record/redo":
                station.events["rerecord_episode"] = True
                station.events["exit_early"] = True
                self._send_json({"ok": True})
            elif path == "/api/stop":
                self._send_json(station.stop())
            elif path == "/api/release":
                self._send_json(station.release_motors())
            elif path == "/api/datasets/dir":
                self._send_json(station.set_datasets_dir(str(body.get("path", ""))))
            elif path == "/api/replay/start":
                self._send_json(station.start_replay(
                    name=str(body.get("name", "")),
                    episode=max(0, int(body.get("episode", 0))),
                ))
            else:
                self._send_json({"why": "not found"}, code=404)

    return Handler


def serve(station, port: int) -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer(("127.0.0.1", port), make_handler(station))
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, name="http", daemon=True).start()
    return httpd
