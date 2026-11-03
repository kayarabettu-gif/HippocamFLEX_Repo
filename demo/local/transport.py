"""
Send a (context, event) query to the board and return the recall.

MockTransport only echoes the detected pair, TcpTransport talks to
board/board_server.c, and BoardDirectTransport is an unused stub. All of
them return an InferResult.
"""
from __future__ import annotations
import socket
import struct
import json
from dataclasses import dataclass, field
from typing import Optional, Dict, Any

import hippocamflex_hv as hv


@dataclass
class InferResult:
    detected_colour: str
    detected_digit:  str
    present:  Optional[str] = None
    future1:  Optional[str] = None
    future2:  Optional[str] = None
    all_events: Dict[str, int] = field(default_factory=dict)   # name -> voltage
    timestamps: Dict[str, int] = field(default_factory=dict)   # name -> CA1 timestamp
    ok: bool = True
    note: str = ""

    def to_dict(self):
        return self.__dict__.copy()


class MockTransport:
    """Vision only: returns the detected pair with ok=False."""

    def infer(self, ctx_hv, evt_hv, ctx_idx, evt_idx) -> InferResult:
        colour = hv.CTX_NAME.get(ctx_idx, "?")
        digit  = hv.EVT_NAME.get(evt_idx, "?")
        return InferResult(
            detected_colour=colour,
            detected_digit=digit,
            ok=False,
            note="mock mode: vision only, no FPGA prediction",
        )


class TcpTransport:
    """
    One JSON line per connection to board_server:
        request:  {"ctx": 0..4, "evt": 0..6}
        reply:    {"ok": true, "present": "D7", "future1": "D5", "future2": "D3",
                   "events": {"D7": 64, ...}, "timestamps": {"D3": 0, ...}}
    """
    def __init__(self, host: str, port: int = 9000, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout

    def infer(self, ctx_hv, evt_hv, ctx_idx, evt_idx) -> InferResult:
        req = json.dumps({"ctx": int(ctx_idx), "evt": int(evt_idx)}) + "\n"
        colour = hv.CTX_NAME.get(ctx_idx, "?")
        digit  = hv.EVT_NAME.get(evt_idx, "?")
        try:
            with socket.create_connection((self.host, self.port),
                                          timeout=self.timeout) as s:
                s.sendall(req.encode())
                data = b""
                while b"\n" not in data:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    data += chunk
            resp = json.loads(data.decode().splitlines()[0])
        except Exception as e:
            return InferResult(detected_colour=colour, detected_digit=digit,
                               ok=False, note=f"transport error: {e}")

        return InferResult(
            detected_colour=colour, detected_digit=digit,
            present=resp.get("present"),
            future1=resp.get("future1"),
            future2=resp.get("future2"),
            all_events=resp.get("events", {}),
            timestamps=resp.get("timestamps", {}),
            ok=resp.get("ok", False),
            note=resp.get("note", ""),
        )


class BoardDirectTransport:
    """Placeholder for running on the board itself. Use TcpTransport instead."""
    def infer(self, *args, **kwargs):
        raise NotImplementedError(
            "BoardDirectTransport requires porting the C mmap driver to "
            "Python. Easiest path: run the C binary as a TCP server and use "
            "TcpTransport instead.")


def make_transport(kind="mock", **kwargs):
    kind = kind.lower()
    if kind == "mock":
        return MockTransport()
    if kind == "tcp":
        return TcpTransport(**kwargs)
    if kind == "board":
        return BoardDirectTransport()
    raise ValueError(f"unknown transport kind: {kind}")
