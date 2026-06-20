from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from attitude_controller import AircraftState, ControllerTargets, make_default_controller
from pid import PID


def test_positive_pitch_error_gives_positive_elevator():
    controller = make_default_controller()
    command = controller.update(
        AircraftState(pitch_deg=0.0, roll_deg=0.0),
        ControllerTargets(target_pitch_deg=5.0, target_roll_deg=0.0),
        dt=0.02,
    )
    assert command.elevator < 0.0 #выдаст ошибку, если при питч >0 элеватор <0


def test_positive_roll_error_gives_positive_aileron_and_opposite_surfaces():
    controller = make_default_controller()
    command = controller.update(
        AircraftState(pitch_deg=0.0, roll_deg=0.0),
        ControllerTargets(target_pitch_deg=0.0, target_roll_deg=10.0),
        dt=0.02,
    )
    assert command.aileron > 0.0
    assert command.left_aileron == command.aileron
    assert command.right_aileron == -command.aileron


def test_outputs_are_limited_to_normalized_range():
    controller = make_default_controller()
    command = controller.update(
        AircraftState(pitch_deg=-90.0, roll_deg=-90.0),
        ControllerTargets(target_pitch_deg=90.0, target_roll_deg=90.0),
        dt=0.02,
    )
    assert -1.0 <= command.elevator <= 1.0
    assert -1.0 <= command.aileron <= 1.0


def test_visual_gain_applies_only_in_bench_mode():
    normal = make_default_controller()
    bench = make_default_controller()
    state = AircraftState(pitch_deg=0.0, roll_deg=0.0)
    targets = ControllerTargets(target_pitch_deg=2.0, target_roll_deg=2.0)

    normal_command = normal.update(state, targets, dt=0.02, bench_mode=False)
    bench_command = bench.update(state, targets, dt=0.02, bench_mode=True)

    assert abs(bench_command.elevator) > abs(normal_command.elevator) # потому что elevator sign отрицательный
    assert bench_command.aileron > normal_command.aileron
    assert bench_command.elevator == normal_command.elevator * 3.0 #проверка на верность исполнения бенч теста


def test_altitude_hold_generates_target_pitch_when_enabled():
    controller = make_default_controller()
    controller.enable_altitude_hold = True
    command = controller.update(
        AircraftState(pitch_deg=0.0, roll_deg=0.0, altitude_m=90.0),
        ControllerTargets(target_pitch_deg=0.0, target_roll_deg=0.0, target_altitude_m=100.0),
        dt=0.1,
    )
    assert command.target_pitch_deg > 0.0


def test_heading_hold_generates_target_roll_when_enabled():
    controller = make_default_controller()
    controller.enable_heading_hold = True
    controller.heading_pid = PID(kp=0.5, output_limits=(-25.0, 25.0))
    command = controller.update(
        AircraftState(pitch_deg=0.0, roll_deg=0.0, heading_deg=350.0),
        ControllerTargets(target_pitch_deg=0.0, target_roll_deg=0.0, target_heading_deg=10.0),
        dt=0.1,
    )
    assert command.target_roll_deg > 0.0
