# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the vendor ext-axis HTTP client against a fake server.

No robot required: a threaded HTTP listener on 127.0.0.1 records what the
client sent and replays scripted replies.
"""
import http.server
import json
import threading
import unittest

from lumi_body.http_client import ExtAxisError, LumiExtAxisClient


class _Handler(http.server.BaseHTTPRequestHandler):
    routes: dict = {}
    calls: list = []

    def _handle(self):
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length).decode("utf-8") if length else ""
        path = self.path.split("?")[0]
        type(self).calls.append((self.command, path, body))

        reply = type(self).routes.get(self.command + " " + path)
        if reply is None:
            reply = ({"status": 0}, 200)
        payload, code = reply if isinstance(reply, tuple) else (reply, 200)
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    do_GET = _handle
    do_POST = _handle

    def log_message(self, *args):  # keep the test output clean
        pass


class FakeServer:
    def __init__(self, routes=None):
        _Handler.routes = routes or {}
        _Handler.calls = []
        self._httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    @property
    def calls(self):
        return _Handler.calls

    @property
    def url(self):
        host, port = self._httpd.server_address
        return f"http://{host}:{port}"

    def close(self):
        self._httpd.shutdown()
        self._httpd.server_close()


class TestReads(unittest.TestCase):
    def test_status_returns_the_axis_list(self):
        axes = [
            {"id": 0, "pos": 100.0, "vel": 0.0, "enable": 1},
            {"id": 1, "pos": 0.0, "vel": 0.0, "enable": 1},
            {"id": 2, "pos": 0.0, "vel": 0.0, "enable": 1},
            {"id": 3, "pos": 0.0, "vel": 0.0, "enable": 1},
        ]
        server = FakeServer({"GET /api/extaxis/status": axes})
        try:
            status = LumiExtAxisClient(server.url, 2.0).status()
        finally:
            server.close()
        self.assertEqual(len(status), 4)
        self.assertEqual(status[0]["pos"], 100.0)
        self.assertEqual([c[1] for c in server.calls], ["/api/extaxis/status"])

    def test_sysinfo_is_passed_through(self):
        server = FakeServer(
            {"GET /api/extaxis/sysinfo": {"serial_number": "Lumi1", "version": "1.0"}}
        )
        try:
            info = LumiExtAxisClient(server.url, 2.0).sysinfo()
        finally:
            server.close()
        self.assertEqual(info["serial_number"], "Lumi1")

    def test_status_with_wrong_shape_raises(self):
        server = FakeServer({"GET /api/extaxis/status": {"pos": [0, 0, 0, 0]}})
        try:
            with self.assertRaises(ExtAxisError):
                LumiExtAxisClient(server.url, 2.0).status()
        finally:
            server.close()

    def test_http_error_raises(self):
        server = FakeServer({"GET /api/extaxis/status": ({"err": 1}, 500)})
        try:
            with self.assertRaises(ExtAxisError) as ctx:
                LumiExtAxisClient(server.url, 2.0).status()
        finally:
            server.close()
        self.assertIn("500", str(ctx.exception))


class TestWrites(unittest.TestCase):
    def test_moveto_sends_four_positions_and_percentages(self):
        server = FakeServer({"POST /api/extaxis/moveto": {"status": 0}})
        try:
            LumiExtAxisClient(server.url, 2.0).moveto([100, -10, 20, 5], 10, 15)
        finally:
            server.close()
        method, path, body = server.calls[0]
        self.assertEqual((method, path), ("POST", "/api/extaxis/moveto"))
        self.assertEqual(
            json.loads(body), {"pos": [100.0, -10.0, 20.0, 5.0], "vel": 10.0, "acc": 15.0}
        )

    def test_enable_uses_integer_flag(self):
        server = FakeServer({"POST /api/extaxis/enable": {"status": 0}})
        try:
            client = LumiExtAxisClient(server.url, 2.0)
            client.enable(True)
            client.enable(False)
        finally:
            server.close()
        bodies = [json.loads(call[2]) for call in server.calls]
        self.assertEqual(bodies, [{"enable": 1}, {"enable": 0}])

    def test_status_minus_one_is_an_error(self):
        server = FakeServer(
            {"POST /api/extaxis/moveto": {"status": -1, "message": "not enabled"}}
        )
        try:
            with self.assertRaises(ExtAxisError) as ctx:
                LumiExtAxisClient(server.url, 2.0).moveto([0, 0, 0, 0], 10, 10)
        finally:
            server.close()
        self.assertIn("not enabled", str(ctx.exception))

    def test_fail_message_is_an_error(self):
        server = FakeServer({"POST /api/extaxis/stop": {"message": "stop failed"}})
        try:
            with self.assertRaises(ExtAxisError):
                LumiExtAxisClient(server.url, 2.0).stop()
        finally:
            server.close()

    def test_moveto_rejects_wrong_arity_locally(self):
        client = LumiExtAxisClient("http://127.0.0.1:1", 0.5)
        with self.assertRaises(ValueError):
            client.moveto([1, 2, 3], 10, 10)


if __name__ == "__main__":
    unittest.main()
