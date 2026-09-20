import threading
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest
import yaml
from times_arm_perception.client import ArmClient, run_plan
from times_arm_perception.core import (
    Kinematics,
    Plan,
    Settings,
    feedback,
    make_plan,
    validate_execution,
)


def state(q=None, enabled=True):
    return {
        "connected": True,
        "enabled": enabled,
        "velocity": 0.1,
        "axes": [
            {"index": i, "pos_deg": float(v), "error": 1 if enabled else 0}
            for i, v in enumerate(q if q is not None else [0.0] * 6)
        ],
    }


def test_shipped_config_is_complete_and_preview_only():
    path = Path(__file__).parents[1] / "config" / "arm.pending.yaml"
    values = yaml.safe_load(path.read_text())["arm_perception"]["ros__parameters"]
    cfg = Settings(**values)
    assert set(values) == set(asdict(cfg))
    assert cfg.mode == "preview"
    assert not cfg.publish_mount_tf
    with pytest.raises(ValueError):
        cfg.require_planning()


@pytest.mark.parametrize("mode", ["plan", "execute"])
def test_calibration_and_model_required(mode):
    for calibration, model in [(False, False), (True, False), (False, True)]:
        with pytest.raises(ValueError):
            Settings(
                mode=mode, calibration_confirmed=calibration, kinematics_confirmed=model
            ).require_planning()


@pytest.mark.parametrize(
    "mutation",
    [
        lambda s: s.update(connected=False),
        lambda s: s["axes"][2].update(pos_deg=None),
        lambda s: s["axes"][2].update(pos_deg=float("nan")),
        lambda s: s["axes"][2].update(error=8),
        lambda s: s["axes"].reverse(),
        lambda s: s.update(traj_playing=True),
    ],
)
def test_feedback_fails_closed(mutation):
    value = state()
    mutation(value)
    with pytest.raises(ValueError):
        feedback(value, enabled=True)


def test_disabled_feedback_can_preview_not_execute():
    np.testing.assert_allclose(feedback(state(enabled=False)), 0.0)
    with pytest.raises(ValueError):
        feedback(state(enabled=False), enabled=True)


def test_upstream_fk_zero_pose_and_configured_motor_mapping():
    model = Kinematics(Settings())
    # Independent sum of the reference chain at zero joint rotations.
    np.testing.assert_allclose(
        model.fk([0.0] * 5)[:3, 3], [0.0, -0.11, 0.528], atol=1e-9
    )
    cfg = Settings(
        joint_signs=[-1.0, 1.0, 1.0, 1.0, 1.0],
        joint_zero_offsets_deg=[10.0, 0.0, 0.0, 0.0, 0.0],
    )
    np.testing.assert_allclose(
        Kinematics(cfg).fk([30.0, 0.0, 0.0, 0.0, 0.0]),
        model.fk([-20.0, 0.0, 0.0, 0.0, 0.0]),
        atol=1e-9,
    )


def test_ik_roundtrip_and_unreachable():
    model = Kinematics(Settings(require_tool_down=False))
    q = [10.0, -20.0, 30.0, 15.0, 25.0]
    target = model.fk(q)[:3, 3]
    solved = model.ik(target, [0.0] * 5)
    assert np.linalg.norm(model.fk(solved)[:3, 3] - target) <= 0.01
    with pytest.raises(ValueError, match="IK"):
        model.ik([4.0, 0.0, 4.0], q)


def test_full_plan_and_execution_preconditions():
    cfg = Settings(
        mode="execute",
        calibration_confirmed=True,
        kinematics_confirmed=True,
        require_tool_down=False,
        approach_m=0.02,
        lift_m=0.03,
    )
    q = [0.0, -35.0, 50.0, 0.0, 50.0, 10.0]
    xyz = Kinematics(cfg).fk(q[:5])[:3, 3]
    target = {"xyz": xyz.tolist(), "class_id": 39, "frame_id": "arm_base_link"}
    plan = make_plan(target, state(q), cfg, 10.0)
    assert [s["name"] for s in plan.stages] == [
        "open",
        "approach",
        "grasp",
        "close",
        "lift",
    ]
    validate_execution(plan, [target], state(q), cfg, 11.0)
    with pytest.raises(ValueError, match="fresh"):
        validate_execution(plan, [target], state(q), cfg, 21.0)
    with pytest.raises(ValueError, match="disappeared"):
        validate_execution(plan, [], state(q), cfg, 11.0)
    with pytest.raises(ValueError, match="moved"):
        validate_execution(plan, [target], state([30.0, *q[1:]]), cfg, 11.0)
    with pytest.raises(ValueError, match="frame"):
        make_plan({**target, "frame_id": "base_link"}, state(q), cfg, 10.0)
    with pytest.raises(ValueError, match="workspace"):
        make_plan({**target, "xyz": [1.0, 0.0, 0.2]}, state(q), cfg, 10.0)


def test_no_motion_or_maintenance_endpoints_in_preview(monkeypatch):
    calls = []
    monkeypatch.setattr("urllib.request.urlopen", lambda *a, **kw: calls.append(a))
    for mode in (False, True):
        client = ArmClient("http://127.0.0.1:18080", write_enabled=mode)
        for endpoint in (
            "/api/connect",
            "/api/enable",
            "/api/home",
            "/api/axis_zero",
            "/api/vision/grasp/start",
        ):
            with pytest.raises(ValueError):
                client.request(endpoint, {})
    client = ArmClient("http://127.0.0.1:18080")
    for endpoint in ("/api/axes", "/api/hold", "/api/velocity"):
        with pytest.raises(ValueError):
            client.request(endpoint, {})
    assert calls == []


class FakeClient:
    def __init__(self):
        self.q = [0.0] * 6
        self.calls = []
        self.fail = False

    def state(self):
        return state(self.q)

    def request(self, path, body):
        self.calls.append(path)
        if path == "/api/axes":
            if self.fail:
                raise OSError("disconnected")
            self.q = body["degrees"]
        return self.state()


def test_executor_success_cancel_and_connection_loss():
    cfg = Settings(
        mode="execute",
        calibration_confirmed=True,
        kinematics_confirmed=True,
        control_period_sec=0.001,
    )
    plan = Plan(
        {},
        [0.0] * 6,
        [{"name": "approach", "degrees": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]}],
        0.0,
    )
    client = FakeClient()
    run_plan(client, plan, cfg, threading.Event(), lambda _: None)
    assert client.q[0] == 0.01
    assert set(client.calls) == {"/api/axes", "/api/velocity"}
    client = FakeClient()
    client.fail = True
    with pytest.raises(OSError):
        run_plan(client, plan, cfg, threading.Event(), lambda _: None)
    assert client.calls[-1] == "/api/hold"
    client = FakeClient()
    canceled = threading.Event()
    canceled.set()
    with pytest.raises(ValueError, match="Canceled"):
        run_plan(client, plan, cfg, canceled, lambda _: None)
    assert client.calls == []
