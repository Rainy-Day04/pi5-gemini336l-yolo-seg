"""Existing HTTP API only: no serial access, connection, enable, home or zero writes."""

import json
import math
import threading
import time
import urllib.request

import numpy as np

from .core import feedback, vector


class HoldFailedError(RuntimeError):
    """An attempted stop could not be confirmed by the existing HTTP service."""


class ArmClient:
    def __init__(self, base_url, timeout=0.5, write_enabled=False):
        if not base_url.startswith(("http://", "https://")):
            raise ValueError("base_url must be an HTTP URL")
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.write_enabled = write_enabled
        self.lock = threading.Lock()

    def request(self, path, body=None):
        if body is None:
            if path != "/api/state":
                raise ValueError("Unsupported read endpoint")
        elif not self.write_enabled or path not in (
            "/api/axes",
            "/api/velocity",
            "/api/hold",
        ):
            raise ValueError("Write blocked by arm adapter")
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        req = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers={"Content-Type": "application/json"},
        )
        with self.lock, urllib.request.urlopen(req, timeout=self.timeout) as response:
            result = json.load(response)
        if not result.get("ok") or not isinstance(result.get("data"), dict):
            raise ValueError(str(result.get("error", "Invalid arm API response")))
        return result["data"]

    def state(self):
        return self.request("/api/state")


def run_plan(client, plan, cfg, cancel, report):
    """Bounded joint interpolation; holds on cancel, feedback failure or timeout."""
    cfg.require_planning()
    if cfg.mode != "execute":
        raise ValueError("Execution is disabled")
    wrote = False
    try:
        feedback(client.state(), enabled=True)
        if cancel.is_set():
            raise ValueError("Canceled before execution")
        client.request("/api/velocity", {"velocity": cfg.velocity_rad_s})
        wrote = True
        for stage in plan.stages:
            report(stage["name"])
            target = vector(stage["degrees"], 6)
            command = feedback(client.state(), enabled=True)
            max_step = math.degrees(cfg.velocity_rad_s) * cfg.control_period_sec
            allotted = np.max(np.abs(target - command)) / math.degrees(
                cfg.velocity_rad_s
            )
            deadline = time.monotonic() + allotted * 3 + cfg.settle_timeout_sec
            grip_wait_started = None
            while True:
                if cancel.is_set():
                    raise ValueError("Canceled; requesting current-position hold")
                if time.monotonic() > deadline:
                    raise ValueError(f"{stage['name']}: motion timeout")
                state = client.state()
                actual = feedback(state, enabled=True)
                # Gripper contact can prevent reaching its close setpoint.
                active_axes = (
                    slice(0, 5) if stage["name"] in ("close", "lift") else slice(0, 6)
                )
                if (
                    np.max(np.abs(actual[active_axes] - command[active_axes]))
                    > cfg.tracking_tolerance_deg
                ):
                    raise ValueError("Joint tracking error exceeds limit")
                if np.any(actual < cfg.joint_min_deg) or np.any(
                    actual > cfg.joint_max_deg
                ):
                    raise ValueError("Joint feedback outside limits")
                velocity = float(state.get("velocity", float("nan")))
                if (
                    not math.isfinite(velocity)
                    or abs(velocity - cfg.velocity_rad_s) > 1e-6
                ):
                    raise ValueError("Arm velocity was changed externally")
                command = command + np.clip(target - command, -max_step, max_step)
                if cancel.is_set():
                    raise ValueError("Canceled; requesting current-position hold")
                client.request("/api/axes", {"degrees": command.tolist()})
                reached_command = np.max(np.abs(target - command)) < 1e-6
                reached_actual = (
                    np.max(np.abs(actual[active_axes] - target[active_axes]))
                    <= cfg.settle_tolerance_deg
                )
                if reached_command and reached_actual:
                    if stage["name"] != "close":
                        break
                    if grip_wait_started is None:
                        grip_wait_started = time.monotonic()
                    if time.monotonic() - grip_wait_started >= cfg.gripper_settle_sec:
                        break
                cancel.wait(cfg.control_period_sec)
        if cancel.is_set():
            raise ValueError("Canceled; requesting current-position hold")
        report("complete: commanded sequence finished; grasp success not verified")
    except Exception as exc:
        if wrote:
            try:
                client.request("/api/hold", {})
            except Exception as hold_error:  # noqa: BLE001 - report both failures at the execution boundary
                raise HoldFailedError(
                    f"{exc}; hold request also failed: {hold_error}"
                ) from exc
        raise
