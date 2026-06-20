from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys

import yaml

from attitude_controller import AircraftState, ControllerTargets, make_default_controller


def load_matplotlib_pyplot():
    try:
        os.environ.setdefault("MPLCONFIGDIR", "/tmp/simple_autopilot_matplotlib")
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plots: pip install matplotlib") from exc
    return plt


@dataclass
class SimState:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    altitude_m: float = 100.0
    heading_deg: float = 0.0
    airspeed_mps: float = 55.0


class JSBSimAircraft:

    FT_PER_METER = 3.280839895 # перевод американских единиц в общепринятые, потому что jsbsim работает в них
    KTS_PER_MPS = 1.943844492

    # пид-контроллер тестируется на С172 (встроенная модель), потому что моя работает некорректно
    #когда дострою ее, может буду тестить на ней
    def __init__(self, dt: float, initial_state: dict, model: str = "c172p") -> None:
        try:
            import jsbsim
        except ImportError as exc:
            raise RuntimeError("Install JSBSim first: pip install jsbsim") from exc
        if not hasattr(jsbsim, "FGFDMExec"):
            raise RuntimeError("Installed 'jsbsim' is not the JSBSim Python package")

        self.fdm = jsbsim.FGFDMExec(None)
        self.fdm.set_debug_level(0)
        self.fdm.set_dt(dt)
        if not self.fdm.load_model(model):
            raise RuntimeError(f"JSBSim model was not found: {model}")
        if not self.fdm.load_ic("reset01", True):
            raise RuntimeError("JSBSim initial condition file was not found: reset01")

        self._apply_initial_state(initial_state)
        if not self.fdm.run_ic():
            raise RuntimeError("JSBSim initial conditions failed")

        self._try_set("propulsion/engine/set-running", 1.0)
        self._try_set("propulsion/magneto_cmd", 3.0)
        self._try_set("propulsion/starter_cmd", 1.0)
        self._try_set("fcs/mixture-cmd-norm", 1.0)
        self._try_set("fcs/mixture-cmd-norm[0]", 1.0)

    def _set(self, prop: str, value: float) -> None:
        self.fdm[prop] = value

    def _try_set(self, prop: str, value: float) -> None:
        try:
            self.fdm[prop] = value
        except Exception:
            pass

    def _get(self, prop: str) -> float:
        return float(self.fdm[prop])

    def _apply_initial_state(self, initial_state: dict) -> None:
        if "altitude_m" in initial_state:
            self._set("ic/h-sl-ft", float(initial_state["altitude_m"]) * self.FT_PER_METER)
        if "airspeed_mps" in initial_state:
            self._set("ic/vc-kts", float(initial_state["airspeed_mps"]) * self.KTS_PER_MPS)
        if "pitch_deg" in initial_state:
            pitch = float(initial_state["pitch_deg"])
            self._set("ic/theta-deg", pitch)
            self._set("ic/gamma-deg", pitch)
        if "roll_deg" in initial_state:
            self._set("ic/phi-deg", float(initial_state["roll_deg"]))
        if "heading_deg" in initial_state:
            self._set("ic/psi-true-deg", float(initial_state["heading_deg"]))

    def step(self, elevator: float, aileron: float, throttle: float) -> SimState:
        self._try_set("fcs/elevator-cmd-norm", elevator)
        self._try_set("fcs/aileron-cmd-norm", aileron)
        self._try_set("fcs/throttle-cmd-norm", throttle)
        self._try_set("fcs/throttle-cmd-norm[0]", throttle)
        self.fdm.run()
        return self.state()

    def state(self) -> SimState:
        return SimState(
            pitch_deg=math.degrees(self._get("attitude/theta-rad")),
            roll_deg=math.degrees(self._get("attitude/phi-rad")),
            altitude_m=self._get("position/h-sl-ft") / self.FT_PER_METER,
            heading_deg=math.degrees(self._get("attitude/psi-rad")) % 360.0,
            airspeed_mps=self._get("velocities/vc-kts") / self.KTS_PER_MPS,
        )


def load_scenario(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is required: pip install PyYAML")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def plot_scenario(path: Path, telemetry: dict[str, list[float]], plot_dir: Path) -> None:
    plt = load_matplotlib_pyplot()
    plot_dir.mkdir(parents=True, exist_ok=True)
    stem = path.stem
    t = telemetry["time"]

    plt.figure()
    plt.plot(t, telemetry["pitch"], label="pitch")
    plt.plot(t, telemetry["target_pitch"], "--", label="target pitch")
    plt.plot(t, telemetry["roll"], label="roll")
    plt.plot(t, telemetry["target_roll"], "--", label="target roll")
    plt.xlabel("time, s")
    plt.ylabel("deg")
    plt.title(f"{stem}: attitude")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / f"{stem}_attitude.png", dpi=140)
    plt.close()

    plt.figure()
    plt.plot(t, telemetry["elevator"], label="elevator")
    plt.plot(t, telemetry["aileron"], label="aileron")
    plt.xlabel("time, s")
    plt.ylabel("normalized command")
    plt.title(f"{stem}: controls")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / f"{stem}_controls.png", dpi=140)
    plt.close()

    plt.figure()
    plt.plot(t, telemetry["altitude"], label="altitude")
    if telemetry["target_altitude"]:
        plt.plot(t, telemetry["target_altitude"], "--", label="target altitude")
    plt.xlabel("time, s")
    plt.ylabel("m")
    plt.title(f"{stem}: altitude")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / f"{stem}_altitude.png", dpi=140)
    plt.close()


def run_scenario(path: Path, plot_dir: Path | None = None) -> bool:
    scenario = load_scenario(path)
    dt = float(scenario.get("dt", 0.02))
    duration_s = float(scenario.get("duration_s", 10.0))
    steps = int(duration_s / dt)
    mode = scenario.get("mode", "attitude")
    throttle = float(scenario.get("throttle", 0.45))

    aircraft = JSBSimAircraft(
        dt=dt,
        initial_state=scenario.get("initial_state", {}),
        model=scenario.get("jsbsim_model", "c172p"),
    )
    state = aircraft.state()
    controller = make_default_controller()
    controller.enable_altitude_hold = bool(scenario.get("enable_altitude_hold", False))
    controller.enable_heading_hold = bool(scenario.get("enable_heading_hold", False))

    fixed_elevator = scenario.get("fixed_elevator")
    fixed_aileron = scenario.get("fixed_aileron")
    targets = ControllerTargets(**scenario.get("targets", {}))
    max_abs_pitch = abs(state.pitch_deg)
    max_abs_roll = abs(state.roll_deg)
    telemetry = {
        "time": [],
        "pitch": [],
        "roll": [],
        "target_pitch": [],
        "target_roll": [],
        "elevator": [],
        "aileron": [],
        "altitude": [],
        "target_altitude": [],
    }

    for step_index in range(steps):
        if mode == "fixed_surface": #если значение рулей фиксированы, не подавать на них команды. Нужно для проверки соответствия знаков рулей и высоты
            elevator = float(fixed_elevator or 0.0)
            aileron = float(fixed_aileron or 0.0)
            target_pitch = targets.target_pitch_deg
            target_roll = targets.target_roll_deg
        else:
            controller_state = AircraftState(
                pitch_deg=state.pitch_deg,
                roll_deg=state.roll_deg,
                altitude_m=state.altitude_m,
                heading_deg=state.heading_deg,
            )
            commands = controller.update(controller_state, targets, dt)
            elevator = commands.elevator
            aileron = commands.aileron
            target_pitch = commands.target_pitch_deg
            target_roll = commands.target_roll_deg
        state = aircraft.step(elevator, aileron, throttle)
        max_abs_pitch = max(max_abs_pitch, abs(state.pitch_deg))
        max_abs_roll = max(max_abs_roll, abs(state.roll_deg))
        telemetry["time"].append((step_index + 1) * dt)
        telemetry["pitch"].append(state.pitch_deg)
        telemetry["roll"].append(state.roll_deg)
        telemetry["target_pitch"].append(target_pitch)
        telemetry["target_roll"].append(target_roll)
        telemetry["elevator"].append(elevator)
        telemetry["aileron"].append(aileron)
        telemetry["altitude"].append(state.altitude_m)
        if targets.target_altitude_m is not None:
            telemetry["target_altitude"].append(targets.target_altitude_m)

    checks = scenario.get("checks", {})
    ok = True #начальное состояние сценария (не провалился)
    if "final_pitch_min_deg" in checks: #достаем таргеты из сценария и начинаем проверки
        ok = ok and state.pitch_deg >= float(checks["final_pitch_min_deg"])
    if "final_roll_min_deg" in checks:
        ok = ok and state.roll_deg >= float(checks["final_roll_min_deg"])
    if "final_pitch_abs_error_max_deg" in checks:
        target = targets.target_pitch_deg
        if controller.enable_altitude_hold and targets.target_altitude_m is not None:
            target = 0.0
        ok = ok and abs(state.pitch_deg - target) <= float(checks["final_pitch_abs_error_max_deg"])
    if "final_roll_abs_error_max_deg" in checks:
        ok = ok and abs(state.roll_deg - targets.target_roll_deg) <= float(checks["final_roll_abs_error_max_deg"])
    if "max_abs_pitch_deg" in checks:
        ok = ok and max_abs_pitch <= float(checks["max_abs_pitch_deg"])
    if "max_abs_roll_deg" in checks:
        ok = ok and max_abs_roll <= float(checks["max_abs_roll_deg"])
    if "final_altitude_abs_error_max_m" in checks and targets.target_altitude_m is not None:
        ok = ok and abs(state.altitude_m - targets.target_altitude_m) <= float(checks["final_altitude_abs_error_max_m"])

    result = "PASS" if ok else "FAIL"
    print(
        f"{result} {path.name}: pitch={state.pitch_deg:.2f} deg, "
        f"roll={state.roll_deg:.2f} deg, altitude={state.altitude_m:.1f} m, "
        f"max_pitch={max_abs_pitch:.2f} deg, max_roll={max_abs_roll:.2f} deg"
    )
    if plot_dir is not None:
        plot_scenario(path, telemetry, plot_dir)
    return ok


def scenario_paths(paths: list[Path]) -> list[Path]:
    expanded = []
    for path in paths:
        if path.is_dir():
            expanded.extend(sorted(path.glob("*.yaml")))
        else:
            expanded.append(path)
    return expanded


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", nargs="+", type=Path, help="Scenario YAML file or directory")
    parser.add_argument("--plot-dir", type=Path, help="Directory for matplotlib PNG plots")
    args = parser.parse_args(argv)

    if args.plot_dir is not None:
        try:
            load_matplotlib_pyplot()
        except RuntimeError as exc:
            parser.exit(2, f"{exc}\n")

    ok = True
    try:
        for path in scenario_paths(args.scenario):
            ok = run_scenario(path, args.plot_dir) and ok
    except RuntimeError as exc:
        parser.exit(2, f"{exc}\n")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
