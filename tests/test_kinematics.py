from __future__ import annotations

import numpy as np
import pytest

import d1_kinematics as kin


@pytest.fixture(scope="module")
def movable_chain():
    joints = kin.load_joints(kin.DEFAULT_URDF)
    chain = kin.find_chain(joints, "base_link", "Empty_Link6")
    return [joint for joint in chain if joint.joint_type != "fixed"]


def test_warm_start_reuses_previous_solution(movable_chain):
    reference_q = np.array([0.2, -0.3, 0.4, 0.1, 0.2, -0.1], dtype=float)
    target = kin.forward_kinematics(movable_chain, reference_q, kin.DEFAULT_TOOL_OFFSET)[:3, 3]

    cold = kin.solve_ik(
        movable_chain,
        target,
        kin.DEFAULT_TOOL_OFFSET,
        tolerance=0.001,
        max_iterations=400,
        damping=0.03,
    )
    warm = kin.solve_ik(
        movable_chain,
        target,
        kin.DEFAULT_TOOL_OFFSET,
        tolerance=0.001,
        max_iterations=400,
        damping=0.03,
        initial_q=cold[0],
    )

    assert cold[-1] is True
    assert warm[-1] is True
    assert warm[5] == 1
    assert warm[5] <= cold[5]
    np.testing.assert_allclose(warm[0], cold[0], atol=1e-12)


def test_fixed_seed_fallback_remains_available_after_bad_warm_start(movable_chain):
    zero_q = np.zeros(len(movable_chain), dtype=float)
    target = kin.forward_kinematics(movable_chain, zero_q, kin.DEFAULT_TOOL_OFFSET)[:3, 3]
    bad_initial = np.array([joint.upper for joint in movable_chain], dtype=float)

    result = kin.solve_ik(
        movable_chain,
        target,
        kin.DEFAULT_TOOL_OFFSET,
        tolerance=0.001,
        max_iterations=1,
        damping=0.03,
        initial_q=bad_initial,
    )

    assert result[-1] is True
    np.testing.assert_allclose(result[0], zero_q, atol=1e-12)


@pytest.mark.parametrize(
    "overrides",
    [
        {"target": np.array([0.1, float("nan"), 0.2])},
        {"tool_offset": np.array([0.1, 0.2])},
        {"tolerance": 0.0},
        {"max_iterations": True},
        {"max_iterations": 0},
        {"damping": float("inf")},
        {"orientation_tolerance": 0.0},
        {"orientation_weight": -1.0},
        {"target_rotation": np.ones((2, 2))},
        {"initial_q": np.full(6, float("nan"))},
    ],
)
def test_solver_rejects_invalid_numeric_inputs(movable_chain, overrides):
    kwargs = {
        "chain": movable_chain,
        "target": np.array([0.2, 0.0, 0.3]),
        "tool_offset": kin.DEFAULT_TOOL_OFFSET,
        "tolerance": 0.001,
        "max_iterations": 20,
        "damping": 0.03,
    }
    kwargs.update(overrides)

    with pytest.raises(ValueError):
        kin.solve_ik(**kwargs)


def test_solver_rejects_empty_chain():
    with pytest.raises(ValueError, match="chain"):
        kin.solve_ik([], np.zeros(3), np.zeros(3), 0.01, 10, 0.03)


def test_payload_requires_six_finite_joints_and_finite_gripper():
    assert kin.make_d1_payload(np.zeros(6), 1, 0.0)["data"]["angle6"] == 0.0
    assert kin.make_d1_payload(np.zeros(6), 1, 90.0)["data"]["angle6"] == 90.0
    with pytest.raises(ValueError, match="six finite"):
        kin.make_d1_payload(np.zeros(5), 1, 0.0)
    with pytest.raises(ValueError, match="six finite"):
        kin.make_d1_payload(np.full(6, float("nan")), 1, 0.0)
    with pytest.raises(ValueError, match="gripper"):
        kin.make_d1_payload(np.zeros(6), 1, float("inf"))
    with pytest.raises(ValueError, match="between 0 and 90"):
        kin.make_d1_payload(np.zeros(6), 1, -0.1)
    with pytest.raises(ValueError, match="between 0 and 90"):
        kin.make_d1_payload(np.zeros(6), 1, 90.1)
    with pytest.raises(ValueError, match="seq"):
        kin.make_d1_payload(np.zeros(6), -1, 0.0)

    payload = kin.make_d1_payload(np.zeros(6), 7, 12.5)
    assert payload["seq"] == 7
    assert payload["data"]["angle6"] == 12.5
