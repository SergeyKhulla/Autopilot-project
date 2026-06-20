from __future__ import annotations

from dataclasses import dataclass

from pid import PID, clamp


def wrap_degrees_180(angle_deg: float) -> float:
    while angle_deg >= 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


@dataclass
class AircraftState:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    altitude_m: float = 0.0
    heading_deg: float = 0.0


@dataclass
class ControllerTargets:
    target_pitch_deg: float = 0.0
    target_roll_deg: float = 0.0
    target_altitude_m: float | None = None
    target_heading_deg: float | None = None


@dataclass
class SurfaceCommands:
    elevator: float
    aileron: float
    left_aileron: float
    right_aileron: float
    pitch_error_deg: float
    roll_error_deg: float
    target_pitch_deg: float
    target_roll_deg: float


@dataclass
class AttitudeController:

    pitch_pid: PID
    roll_pid: PID
    altitude_pid: PID | None = None
    heading_pid: PID | None = None
    enable_altitude_hold: bool = False
    enable_heading_hold: bool = False
    max_target_pitch_deg: float = 10.0
    max_target_roll_deg: float = 25.0
    elevator_sign: float = -1.0
    aileron_sign: float = 1.0
    left_aileron_sign: float = 1.0
    right_aileron_sign: float = 1.0
    bench_visual_servo_gain: float = 3.0 #делал для бенч-теста, но там оказалось и без усиления все видно
    #решил оставить все же

    def update(
        self,
        state: AircraftState,
        targets: ControllerTargets,
        dt: float,
        *,
        bench_mode: bool = False,
    ) -> SurfaceCommands:
        target_pitch = targets.target_pitch_deg
        target_roll = targets.target_roll_deg

        if (
            self.enable_altitude_hold
            and self.altitude_pid is not None
            and targets.target_altitude_m is not None
        ):
            altitude_error = targets.target_altitude_m - state.altitude_m
            target_pitch = clamp(
                self.altitude_pid.update(altitude_error, dt), #ограничение максимального отклонения рулей
                -self.max_target_pitch_deg,
                self.max_target_pitch_deg,
            )

        if (
            self.enable_heading_hold
            and self.heading_pid is not None
            and targets.target_heading_deg is not None
        ):
            heading_error = wrap_degrees_180(targets.target_heading_deg - state.heading_deg)
            target_roll = clamp(
                self.heading_pid.update(heading_error, dt),
                -self.max_target_roll_deg,
                self.max_target_roll_deg,
            )

        pitch_error = target_pitch - state.pitch_deg
        roll_error = target_roll - state.roll_deg

        elevator = self.elevator_sign * self.pitch_pid.update(pitch_error, dt)
        aileron = self.aileron_sign * self.roll_pid.update(roll_error, dt)

        if bench_mode:
            elevator = clamp(elevator * self.bench_visual_servo_gain, -1.0, 1.0)
            aileron = clamp(aileron * self.bench_visual_servo_gain, -1.0, 1.0)
        else:
            elevator = clamp(elevator, -1.0, 1.0)
            aileron = clamp(aileron, -1.0, 1.0)

        left = clamp(self.left_aileron_sign * aileron, -1.0, 1.0)
        right = clamp(self.right_aileron_sign * -aileron, -1.0, 1.0)
        return SurfaceCommands(
            elevator=elevator,
            aileron=aileron,
            left_aileron=left,
            right_aileron=right,
            pitch_error_deg=pitch_error,
            roll_error_deg=roll_error,
            target_pitch_deg=target_pitch,
            target_roll_deg=target_roll,
        )

    def reset(self) -> None:
        self.pitch_pid.reset()
        self.roll_pid.reset()
        if self.altitude_pid is not None:
            self.altitude_pid.reset()
        if self.heading_pid is not None:
            self.heading_pid.reset()


def make_default_controller() -> AttitudeController:
    return AttitudeController(
        pitch_pid=PID(kp=0.08, ki=0.002, kd=0.004), #конфиг пид коэфициентов
        roll_pid=PID(kp=0.06, ki=0.002, kd=0.003),
        altitude_pid=PID(kp=0.9, ki=0.02, kd=0.0, output_limits=(-10.0, 10.0)),
        heading_pid=PID(kp=0.35, ki=0.0, kd=0.0, output_limits=(-25.0, 25.0)),
    )
