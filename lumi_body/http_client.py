# SPDX-License-Identifier: Apache-2.0
"""HTTP client for the JAKA Lumi body joints (vendor "ext-axis" API).

Four motors live behind ``http://192.168.10.90:5000``:

  index 0  lift column    unit mm, range 0..300 (manual says 0..400)
  index 1  waist yaw      unit deg, range -140..140 (manual says +-160)
  index 2  head yaw       unit deg, range -180..180
  index 3  head pitch     unit deg, range -5..35

Endpoints (all JSON in, JSON out):

  GET  /api/extaxis/sysinfo   serial number, version, status
  GET  /api/extaxis/status    [{id, pos, vel, enable, error, ecode}, ...]
  POST /api/extaxis/enable    {"enable": 0|1}
  POST /api/extaxis/reset     {}
  POST /api/extaxis/stop      {}
  POST /api/extaxis/moveto    {"pos": [mm, deg, deg, deg], "vel": %, "acc": %}

Acknowledgements are dictionaries carrying a ``status`` field; the vendor
answers ``-1`` (or a message containing "fail") when it refuses a command.
"""
from __future__ import annotations

import json


class ExtAxisError(RuntimeError):
    """The body controller rejected a request or did not answer."""


class LumiExtAxisClient:
    """Thin, blocking wrapper. No shared state, so it is safe across threads."""

    def __init__(self, base_url: str, timeout_s: float = 5.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)

    # ── transport ────────────────────────────────────────────────────────
    def _request(self, method: str, path: str, payload: dict | None = None) -> dict:
        # Imported lazily so unit tests that only exercise the driver do not
        # need `requests` installed.
        import requests

        url = f"{self.base_url}{path}"
        try:
            response = requests.request(
                method,
                url,
                json=payload,
                timeout=self.timeout_s,
                headers={"Accept": "application/json"},
            )
            response.raise_for_status()
        except Exception as exc:  # noqa: BLE001 - normalise every transport error
            raise ExtAxisError(f"{method} {path}: {exc}") from exc

        try:
            decoded = response.json()
        except json.JSONDecodeError as exc:
            raise ExtAxisError(
                f"{method} {path}: reply is not JSON: {response.text[:200]!r}"
            ) from exc
        return decoded

    @staticmethod
    def _check(reply: dict, what: str) -> dict:
        if not isinstance(reply, dict):
            raise ExtAxisError(f"{what}: unexpected reply {reply!r}")
        status = reply.get("status")
        message = str(reply.get("message", ""))
        if status == -1 or "fail" in message.lower():
            raise ExtAxisError(f"{what}: rejected ({status=} {message=})")
        return reply

    # ── read ─────────────────────────────────────────────────────────────
    def sysinfo(self) -> dict:
        reply = self._request("GET", "/api/extaxis/sysinfo")
        if not isinstance(reply, dict):
            raise ExtAxisError(f"sysinfo: unexpected reply {reply!r}")
        return reply

    def status(self) -> list[dict]:
        """Per-axis state. Also the liveness probe."""
        reply = self._request("GET", "/api/extaxis/status")
        if not isinstance(reply, list):
            raise ExtAxisError(f"status: unexpected reply {reply!r}")
        return reply

    # ── write ────────────────────────────────────────────────────────────
    def enable(self, engaged: bool) -> dict:
        reply = self._request(
            "POST", "/api/extaxis/enable", {"enable": 1 if engaged else 0}
        )
        return self._check(reply, "enable")

    def reset(self) -> dict:
        return self._check(self._request("POST", "/api/extaxis/reset", {}), "reset")

    def stop(self) -> dict:
        return self._check(self._request("POST", "/api/extaxis/stop", {}), "stop")

    def moveto(self, positions, vel_pct: float, acc_pct: float) -> dict:
        positions = [float(value) for value in positions]
        if len(positions) != 4:
            raise ValueError(f"moveto expects 4 positions, got {len(positions)}")
        reply = self._request(
            "POST",
            "/api/extaxis/moveto",
            {"pos": positions, "vel": float(vel_pct), "acc": float(acc_pct)},
        )
        return self._check(reply, "moveto")
