from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pid import PID


def test_proportional_output():
    pid = PID(kp=0.2)
    assert pid.update(error=2.0, dt=0.1) == 0.4


def test_derivative_damping_can_reduce_output():
    pid = PID(kp=0.1, kd=0.05)
    pid.update(error=10.0, dt=1.0)
    assert pid.update(error=5.0, dt=1.0) < 0.5


def test_integral_accumulates():
    pid = PID(kp=0.0, ki=0.5, integral_decay = 1.0)
    assert pid.update(error=1.0, dt=1.0) == 0.5
    assert pid.update(error=1.0, dt=1.0) == 1.0


def test_anti_windup_does_not_integrate_deeper_into_saturation():
    pid = PID(kp=2.0, ki=1.0, output_limits=(-1.0, 1.0), integral_decay = 1.0)
    assert pid.update(error=10.0, dt=1.0) == 1.0
    assert pid.integral == 0.0


def test_output_saturation():
    pid = PID(kp=10.0, output_limits=(-0.25, 0.25))
    assert pid.update(error=1.0, dt=0.1) == 0.25
    assert pid.update(error=-1.0, dt=0.1) == -0.25
