#!/usr/bin/env python3
"""Host-side model of TV3 load-cell and ignition state-machine semantics.

Mirrors the behavior in ``tv3_load_cell`` and ``tv3_mode_manager`` so SIH,
bench ADC replay, and unit tests can share the same state transitions without
running PX4 firmware.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

SOURCE_ADC = 0
SOURCE_REFERENCE = 1
TIME_OFFSET_US = 1_000_000

THRUST_FAULT_NONE = 0
THRUST_FAULT_STALE = 1
THRUST_FAULT_CHANNEL_MISSING = 2
THRUST_FAULT_BAD_SCALE = 4
THRUST_FAULT_NO_REFERENCE = 8

MODE_DISARMED_SAFE = 0
MODE_ARMED_STANDBY = 1
MODE_READY = 2
MODE_IGNITION_PENDING = 3
MODE_BOOST = 4
MODE_COAST = 5
MODE_ABORT = 6

FAULT_NONE = 0
FAULT_COMMAND_ABORT = 1
FAULT_IGNITION_TIMEOUT = 2
FAULT_SENSOR_STALE = 4
FAULT_GCS_LOSS = 8


def engine_bit(engine_index: int) -> int:
    return 1 << engine_index if 0 <= engine_index < 8 else 0


def constrain(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class LoadCellConfig:
    source: int = SOURCE_ADC
    channel: int = 0
    tare: float = 0.0
    scale: float = 1.0
    alpha: float = 0.25
    timeout_ms: int = 200
    ignition_threshold_n: float = 10.0
    v_ref: float = 3.3
    resolution: int = 32768


@dataclass
class MotorReference:
    loaded: bool = False
    expected_thrust_n: float = 0.0
    expected_motor_mass_kg: float = 0.0
    expected_vehicle_mass_kg: float = 1.0
    total_impulse_ns: float = 0.0
    burn_fraction: float = 0.0
    selected_motor_index: int = 0
    selected_motor_id: str = ""
    engine_count: int = 1
    ignition_mask: int = 0
    active_mask: int = 0
    expected_thrust_n_engine: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    expected_motor_mass_kg_engine: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    burn_fraction_engine: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    selected_motor_index_engine: list[int] = field(default_factory=lambda: [0, 0, 0, 0])


@dataclass
class ThrustOutput:
    timestamp_us: int
    timestamp_sample_us: int
    measured_thrust_n: float
    filtered_thrust_n: float
    expected_thrust_n: float
    expected_vehicle_mass_kg: float
    valid: bool
    ignition_confirmed: bool
    fault_flags: int
    selected_motor_index: int
    selected_motor_id: str


@dataclass
class EngineStateOutput:
    timestamp_us: int
    timestamp_sample_us: int
    engine_count: int
    ignition_mask: int
    active_mask: int
    confirmed_mask: int
    fault_mask: int
    sequence_complete: bool
    all_ignited: bool
    measured_thrust_n: list[float]
    filtered_thrust_n: list[float]
    expected_thrust_n: list[float]
    fault_flags: list[int]


@dataclass
class ModeManagerConfig:
    enabled: bool = True
    launch_threshold_n: float = 10.0
    ignition_timeout_ms: int = 2000
    minimum_burn_ms: int = 150
    maximum_burn_ms: int = 6000
    burnout_threshold_n: float = 4.0
    burnout_dwell_ms: int = 100
    rail_length_m: float = 3.5
    abort_on_gcs_loss: bool = False
    engine_count: int = 1
    ignition_sequence: list[int] = field(default_factory=lambda: [0, 1, 2, 3])
    ignition_dwell_ms: int = 0
    expected_vehicle_mass_kg: float = 1.0
    gravity_m_s2: float = 9.80665


@dataclass
class ModeManagerOutput:
    mode: int
    fault_reason: int
    ignition_on: bool
    ignition_mask: int
    active_ignition_index: int
    sequence_complete: bool
    ignition_confirmed: bool
    thrust_valid: bool
    burnout_detected: bool
    rail_exit: bool


@dataclass
class LoadCellModel:
    config: LoadCellConfig
    reference: MotorReference = field(default_factory=MotorReference)
    last_sample_timestamp_us: int = 0
    last_raw: int = 0
    measured_thrust_n: float = 0.0
    filtered_thrust_n: float = 0.0
    engine_measured_thrust_n: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    engine_filtered_thrust_n: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    engine_confirmed_mask: int = 0

    def set_reference(self, reference: MotorReference) -> None:
        self.reference = reference

    def step(self, timestamp_us: int, adc_raw: int | None = None) -> tuple[ThrustOutput, EngineStateOutput]:
        fault_flags = THRUST_FAULT_NONE
        cfg = self.config
        ref = self.reference

        if cfg.source == SOURCE_REFERENCE:
            self.measured_thrust_n = ref.expected_thrust_n
            self.last_sample_timestamp_us = timestamp_us
            self.last_raw = int(round(self.measured_thrust_n * 100.0))
        else:
            found = adc_raw is not None
            if found:
                self.last_sample_timestamp_us = timestamp_us
                self.last_raw = adc_raw
                self.measured_thrust_n = max((adc_raw - cfg.tare) * cfg.scale, 0.0)
            else:
                fault_flags |= THRUST_FAULT_CHANNEL_MISSING

        if abs(cfg.scale) < 1e-6 and cfg.source == SOURCE_ADC:
            fault_flags |= THRUST_FAULT_BAD_SCALE

        if not ref.loaded:
            fault_flags |= THRUST_FAULT_NO_REFERENCE

        if self.last_sample_timestamp_us == 0:
            fault_flags |= THRUST_FAULT_STALE
        elif timestamp_us - self.last_sample_timestamp_us > cfg.timeout_ms * 1000:
            fault_flags |= THRUST_FAULT_STALE

        alpha = constrain(cfg.alpha, 0.01, 1.0)
        self.filtered_thrust_n = alpha * self.measured_thrust_n + (1.0 - alpha) * self.filtered_thrust_n
        engine_state = self._update_engine_state(timestamp_us, fault_flags, alpha)
        ignition_confirmed = self.engine_confirmed_mask != 0 or self.filtered_thrust_n >= cfg.ignition_threshold_n

        thrust = ThrustOutput(
            timestamp_us=timestamp_us,
            timestamp_sample_us=self.last_sample_timestamp_us,
            measured_thrust_n=self.measured_thrust_n,
            filtered_thrust_n=self.filtered_thrust_n,
            expected_thrust_n=ref.expected_thrust_n,
            expected_vehicle_mass_kg=ref.expected_vehicle_mass_kg,
            valid=fault_flags == THRUST_FAULT_NONE,
            ignition_confirmed=ignition_confirmed,
            fault_flags=fault_flags,
            selected_motor_index=ref.selected_motor_index,
            selected_motor_id=ref.selected_motor_id,
        )
        return thrust, engine_state

    def _update_engine_state(self, timestamp_us: int, aggregate_fault_flags: int, alpha: float) -> EngineStateOutput:
        ref = self.reference
        engine_count = constrain(ref.engine_count if ref.engine_count > 0 else 1, 1, 4)
        expected_sum = 0.0

        for i in range(int(engine_count)):
            expected = ref.expected_thrust_n_engine[i] if ref.engine_count > 0 else ref.expected_thrust_n
            expected_sum += max(expected, 0.0)

        self.engine_confirmed_mask = 0
        measured: list[float] = []
        filtered: list[float] = []
        expected_values: list[float] = []
        per_engine_faults: list[int] = []

        for i in range(int(engine_count)):
            expected = ref.expected_thrust_n_engine[i] if ref.engine_count > 0 else ref.expected_thrust_n

            if self.config.source == SOURCE_REFERENCE:
                self.engine_measured_thrust_n[i] = expected
            elif expected_sum > 1e-3:
                self.engine_measured_thrust_n[i] = self.measured_thrust_n * max(expected, 0.0) / expected_sum
            else:
                self.engine_measured_thrust_n[i] = self.measured_thrust_n if i == 0 else 0.0

            self.engine_filtered_thrust_n[i] = (
                alpha * self.engine_measured_thrust_n[i] + (1.0 - alpha) * self.engine_filtered_thrust_n[i]
            )

            measured.append(self.engine_measured_thrust_n[i])
            filtered.append(self.engine_filtered_thrust_n[i])
            expected_values.append(expected)
            per_engine_faults.append(aggregate_fault_flags)

            if self.engine_filtered_thrust_n[i] >= self.config.ignition_threshold_n:
                self.engine_confirmed_mask |= engine_bit(i)

        confirmed_mask = self.engine_confirmed_mask
        sequence_complete = ref.ignition_mask != 0 and (confirmed_mask & ref.ignition_mask) == ref.ignition_mask

        return EngineStateOutput(
            timestamp_us=timestamp_us,
            timestamp_sample_us=self.last_sample_timestamp_us,
            engine_count=int(engine_count),
            ignition_mask=ref.ignition_mask,
            active_mask=ref.active_mask,
            confirmed_mask=confirmed_mask,
            fault_mask=0 if aggregate_fault_flags == THRUST_FAULT_NONE else (1 << int(engine_count)) - 1,
            sequence_complete=sequence_complete,
            all_ignited=sequence_complete,
            measured_thrust_n=measured,
            filtered_thrust_n=filtered,
            expected_thrust_n=expected_values,
            fault_flags=per_engine_faults,
        )


@dataclass
class ModeManagerModel:
    config: ModeManagerConfig
    mode: int = MODE_DISARMED_SAFE
    fault_reason: int = FAULT_NONE
    ignition_on: bool = False
    ignition_mask: int = 0
    active_sequence_slot: int = 0
    sequence_complete: bool = False
    launch_requested: bool = False
    abort_requested: bool = False
    reset_requested: bool = False
    ignition_timestamp_us: int = 0
    boost_timestamp_us: int = 0
    burnout_low_timestamp_us: int = 0
    current_engine_confirm_timestamp_us: int = 0
    last_update_us: int = 0
    rail_exit: bool = False
    rail_distance_m: float = 0.0
    rail_velocity_m_s: float = 0.0

    def request_launch(self) -> None:
        self.launch_requested = True

    def request_abort(self) -> None:
        self.abort_requested = True

    def request_reset(self) -> None:
        self.reset_requested = True

    def reset_state(self) -> None:
        self.mode = MODE_DISARMED_SAFE
        self.fault_reason = FAULT_NONE
        self.ignition_on = False
        self.launch_requested = False
        self.abort_requested = False
        self.reset_requested = False
        self.ignition_timestamp_us = 0
        self.boost_timestamp_us = 0
        self.burnout_low_timestamp_us = 0
        self.rail_exit = False
        self.rail_distance_m = 0.0
        self.rail_velocity_m_s = 0.0
        self._reset_engine_sequence()

    def _reset_engine_sequence(self) -> None:
        self.ignition_mask = 0
        self.active_sequence_slot = 0
        self.current_engine_confirm_timestamp_us = 0
        self.sequence_complete = False

    def _start_engine_sequence(self, timestamp_us: int) -> None:
        self.active_sequence_slot = 0
        self.current_engine_confirm_timestamp_us = 0
        self.sequence_complete = False
        self.ignition_mask = engine_bit(self.config.ignition_sequence[0])
        self.ignition_timestamp_us = timestamp_us

    def _active_sequence_engine_confirmed(self, thrust: ThrustOutput, engine_state: EngineStateOutput) -> bool:
        if self.config.engine_count <= 1 or engine_state.engine_count == 0:
            return thrust.ignition_confirmed

        engine = self.config.ignition_sequence[self.active_sequence_slot]
        return (engine_state.confirmed_mask & engine_bit(engine)) != 0

    def _all_sequence_engines_confirmed(self, thrust: ThrustOutput, engine_state: EngineStateOutput) -> bool:
        if self.config.engine_count <= 1:
            return thrust.ignition_confirmed

        required_mask = 0
        for i in range(self.config.engine_count):
            required_mask |= engine_bit(self.config.ignition_sequence[i])

        return required_mask != 0 and (engine_state.confirmed_mask & required_mask) == required_mask

    def _update_engine_sequence(self, timestamp_us: int, thrust: ThrustOutput, engine_state: EngineStateOutput) -> None:
        if self.config.engine_count <= 1:
            self.sequence_complete = thrust.ignition_confirmed
            return

        if self._active_sequence_engine_confirmed(thrust, engine_state):
            if self.current_engine_confirm_timestamp_us == 0:
                self.current_engine_confirm_timestamp_us = timestamp_us

            if (
                self.active_sequence_slot + 1 < self.config.engine_count
                and timestamp_us - self.current_engine_confirm_timestamp_us >= self.config.ignition_dwell_ms * 1000
            ):
                self.active_sequence_slot += 1
                self.ignition_mask |= engine_bit(self.config.ignition_sequence[self.active_sequence_slot])
                self.current_engine_confirm_timestamp_us = 0
                self.ignition_timestamp_us = timestamp_us
        else:
            self.current_engine_confirm_timestamp_us = 0

        self.sequence_complete = self._all_sequence_engines_confirmed(thrust, engine_state)

    def step(
        self,
        timestamp_us: int,
        thrust: ThrustOutput,
        engine_state: EngineStateOutput,
        *,
        armed: bool,
        motor_loaded: bool,
        gcs_ok: bool = True,
    ) -> ModeManagerOutput:
        cfg = self.config

        if not cfg.enabled:
            self.reset_state()
            return self._output(thrust)

        if not armed:
            self.reset_state()
            return self._output(thrust)

        if self.reset_requested:
            self.mode = MODE_READY
            self.fault_reason = FAULT_NONE
            self.ignition_on = False
            self.reset_requested = False
            self.abort_requested = False
            self.launch_requested = False
            self.ignition_timestamp_us = 0
            self.boost_timestamp_us = 0
            self.burnout_low_timestamp_us = 0
            self.rail_exit = False
            self.rail_distance_m = 0.0
            self.rail_velocity_m_s = 0.0
            self._reset_engine_sequence()

        if self.abort_requested:
            self.fault_reason = FAULT_COMMAND_ABORT
            self.mode = MODE_ABORT
            self.ignition_on = False
            self.ignition_mask = 0
            self.abort_requested = False

        if self.mode == MODE_DISARMED_SAFE:
            self.mode = MODE_ARMED_STANDBY

        if self.mode in (MODE_ARMED_STANDBY, MODE_READY) and motor_loaded:
            self.mode = MODE_READY

        if self.mode == MODE_READY and self.launch_requested:
            self.mode = MODE_IGNITION_PENDING
            self.ignition_on = True
            self._start_engine_sequence(timestamp_us)
            self.launch_requested = False
            self.last_update_us = timestamp_us

        if cfg.abort_on_gcs_loss and not gcs_ok and self.mode in (MODE_IGNITION_PENDING, MODE_BOOST):
            self.fault_reason = FAULT_GCS_LOSS
            self.mode = MODE_ABORT
            self.ignition_on = False
            self.ignition_mask = 0

        if self.mode == MODE_IGNITION_PENDING:
            self._update_engine_sequence(timestamp_us, thrust, engine_state)

            if (
                not self._active_sequence_engine_confirmed(thrust, engine_state)
                and self.ignition_timestamp_us != 0
                and timestamp_us - self.ignition_timestamp_us > cfg.ignition_timeout_ms * 1000
            ):
                self.fault_reason = FAULT_IGNITION_TIMEOUT
                self.mode = MODE_ABORT
                self.ignition_on = False
                self.ignition_mask = 0

            ignition_sequence_complete = (
                self.sequence_complete if cfg.engine_count > 1 else thrust.ignition_confirmed
            )

            if ignition_sequence_complete and self.mode == MODE_IGNITION_PENDING:
                self.mode = MODE_BOOST
                self.boost_timestamp_us = timestamp_us
                self.last_update_us = timestamp_us

        if self.mode == MODE_BOOST:
            if not thrust.valid:
                self.fault_reason = FAULT_SENSOR_STALE
                self.mode = MODE_ABORT
                self.ignition_on = False
                self.ignition_mask = 0
            else:
                dt_s = (timestamp_us - self.last_update_us) * 1e-6 if self.last_update_us != 0 else 0.0
                self.last_update_us = timestamp_us
                thrust_n = max(thrust.filtered_thrust_n, thrust.expected_thrust_n)
                mass_kg = max(self._vehicle_mass_kg(thrust), 0.1)

                if not self.rail_exit and dt_s > 0.0:
                    accel_m_s2 = max(thrust_n / mass_kg - cfg.gravity_m_s2, 0.0)
                    self.rail_distance_m += self.rail_velocity_m_s * dt_s + 0.5 * accel_m_s2 * dt_s * dt_s
                    self.rail_velocity_m_s += accel_m_s2 * dt_s
                    self.rail_exit = self.rail_distance_m >= cfg.rail_length_m

                below_burnout = thrust_n < cfg.burnout_threshold_n
                burn_time_us = timestamp_us - self.boost_timestamp_us if self.boost_timestamp_us != 0 else 0

                if below_burnout and burn_time_us > cfg.minimum_burn_ms * 1000:
                    if self.burnout_low_timestamp_us == 0:
                        self.burnout_low_timestamp_us = timestamp_us
                    elif timestamp_us - self.burnout_low_timestamp_us > cfg.burnout_dwell_ms * 1000:
                        self.mode = MODE_COAST
                        self.ignition_on = False
                else:
                    self.burnout_low_timestamp_us = 0

                if burn_time_us > cfg.maximum_burn_ms * 1000:
                    self.mode = MODE_COAST
                    self.ignition_on = False

        if self.mode == MODE_COAST:
            self.ignition_on = False
            self.ignition_mask = 0

        if self.mode == MODE_ABORT:
            self.ignition_on = False
            self.ignition_mask = 0

        return self._output(thrust)

    def _vehicle_mass_kg(self, thrust: ThrustOutput) -> float:
        return max(thrust.expected_vehicle_mass_kg or self.config.expected_vehicle_mass_kg, 0.1)

    def _output(self, thrust: ThrustOutput) -> ModeManagerOutput:
        active_index = self.config.ignition_sequence[self.active_sequence_slot]
        return ModeManagerOutput(
            mode=self.mode,
            fault_reason=self.fault_reason,
            ignition_on=self.ignition_on,
            ignition_mask=self.ignition_mask,
            active_ignition_index=active_index,
            sequence_complete=self.sequence_complete,
            ignition_confirmed=thrust.ignition_confirmed,
            thrust_valid=thrust.valid,
            burnout_detected=self.mode == MODE_COAST,
            rail_exit=self.rail_exit,
        )


def load_cell_config_from_manifest(manifest: dict) -> LoadCellConfig:
    load_cell = manifest.get("hardware", {}).get("load_cell", {})
    state_machine = manifest.get("state_machine", {})
    propulsion = manifest.get("propulsion", {})
    calibration = load_cell.get("calibration", {})
    ignition = propulsion.get("ignition", {})

    return LoadCellConfig(
        source=int(load_cell.get("source", 0)),
        channel=int(load_cell.get("adc_channel", 0)),
        tare=float(calibration.get("tare", 0.0)),
        scale=float(calibration.get("scale", 1.0)),
        alpha=float(load_cell.get("alpha", 0.25)),
        timeout_ms=int(load_cell.get("timeout_ms", 200)),
        ignition_threshold_n=float(
            ignition.get("confirmation_threshold_n", state_machine.get("launch_threshold_n", 10.0))
        ),
    )


def mode_manager_config_from_manifest(manifest: dict) -> ModeManagerConfig:
    state_machine = manifest.get("state_machine", {})
    propulsion = manifest.get("propulsion", {})
    ignition = propulsion.get("ignition", {})
    engine_count = int(propulsion.get("engine_count", 1))
    sequence = [int(value) for value in ignition.get("sequence", list(range(engine_count)))]

    return ModeManagerConfig(
        enabled=True,
        launch_threshold_n=float(state_machine.get("launch_threshold_n", 10.0)),
        ignition_timeout_ms=int(ignition.get("timeout_ms", state_machine.get("ignition_timeout_ms", 2000))),
        minimum_burn_ms=int(state_machine.get("minimum_burn_ms", 150)),
        maximum_burn_ms=int(state_machine.get("maximum_burn_ms", 6000)),
        burnout_threshold_n=float(state_machine.get("burnout_threshold_n", 4.0)),
        burnout_dwell_ms=int(state_machine.get("burnout_dwell_ms", 100)),
        rail_length_m=float(manifest.get("vehicle", {}).get("rail_length_m", 3.5)),
        abort_on_gcs_loss=bool(state_machine.get("abort_on_gcs_loss", 0)),
        engine_count=engine_count,
        ignition_sequence=sequence,
        ignition_dwell_ms=int(ignition.get("dwell_ms", 0)),
        expected_vehicle_mass_kg=float(manifest.get("vehicle", {}).get("body_mass_kg", 1.0))
        + float(manifest.get("vehicle", {}).get("motor_loaded_mass_kg", 0.0)),
    )


def motor_reference_from_manifest(manifest: dict, *, thrust_n: float, ignition_mask: int = 0) -> MotorReference:
    propulsion = manifest.get("propulsion", {})
    engines = propulsion.get("engines", [])
    engine_count = int(propulsion.get("engine_count", max(len(engines), 1)))
    motor_selection = manifest.get("motor_selection", {})
    vehicle = manifest.get("vehicle", {})

    expected_per_engine = [0.0, 0.0, 0.0, 0.0]
    mass_per_engine = [0.0, 0.0, 0.0, 0.0]
    burn_fraction_per_engine = [0.0, 0.0, 0.0, 0.0]
    motor_index_per_engine = [0, 0, 0, 0]

    total_fraction = 0.0
    for index, engine in enumerate(engines[:engine_count]):
        fraction = float(engine.get("thrust_fraction", 1.0 / max(engine_count, 1)))
        total_fraction += fraction
        expected_per_engine[index] = thrust_n * fraction
        mass_per_engine[index] = float(vehicle.get("motor_loaded_mass_kg", 0.0))
        motor_index_per_engine[index] = int(engine.get("motor_index", index))

    if total_fraction <= 1e-6:
        expected_per_engine[0] = thrust_n

    motor_id = str(motor_selection.get("default_motor_id", motor_selection.get("motor_id", "bench-motor")))
    return MotorReference(
        loaded=True,
        expected_thrust_n=thrust_n,
        expected_motor_mass_kg=float(vehicle.get("motor_loaded_mass_kg", 0.0)),
        expected_vehicle_mass_kg=float(vehicle.get("body_mass_kg", 1.0)) + float(vehicle.get("motor_loaded_mass_kg", 0.0)),
        total_impulse_ns=thrust_n * 3.0,
        burn_fraction=0.0,
        selected_motor_index=int(motor_selection.get("default_motor_index", 0)),
        selected_motor_id=motor_id,
        engine_count=engine_count,
        ignition_mask=ignition_mask,
        active_mask=ignition_mask,
        expected_thrust_n_engine=expected_per_engine,
        expected_motor_mass_kg_engine=mass_per_engine,
        burn_fraction_engine=burn_fraction_per_engine,
        selected_motor_index_engine=motor_index_per_engine,
    )


def load_manifest(path: Path | str) -> dict:
    return json.loads(Path(path).read_text())


def thrust_n_from_adc_raw(raw_count: int, config: LoadCellConfig) -> float:
    return max((raw_count - config.tare) * config.scale, 0.0)


def adc_raw_from_thrust_n(thrust_n: float, config: LoadCellConfig) -> int:
    if abs(config.scale) < 1e-9:
        return int(config.tare)
    return int(round(thrust_n / config.scale + config.tare))


def replay_adc_trace(
    samples: Iterable[tuple[float, int | None]],
    *,
    load_cell: LoadCellModel,
    mode_manager: ModeManagerModel | None = None,
    reference_builder: Callable[[float, float], MotorReference] | None = None,
    armed: bool = True,
    motor_loaded: bool = True,
    launch_at_s: float | None = 0.0,
    sample_period_s: float = 0.02,
    time_offset_us: int = TIME_OFFSET_US,
) -> list[dict]:
    """Replay ADC samples and return per-step thrust, engine_state, and mode outputs."""

    timeline: list[dict] = []
    launch_requested = False

    for time_s, raw_count in samples:
        timestamp_us = int(round(time_s * 1_000_000)) + time_offset_us
        thrust_n = 0.0 if raw_count is None else thrust_n_from_adc_raw(raw_count, load_cell.config)

        if reference_builder is not None:
            reference = reference_builder(time_s, thrust_n)
            load_cell.set_reference(reference)
        elif thrust_n != load_cell.reference.expected_thrust_n:
            reference = load_cell.reference
            reference.expected_thrust_n = thrust_n
            total = sum(
                max(reference.expected_thrust_n_engine[index], 0.0)
                for index in range(reference.engine_count)
            )
            for index in range(reference.engine_count):
                if total > 1e-3:
                    reference.expected_thrust_n_engine[index] = thrust_n * max(
                        reference.expected_thrust_n_engine[index], 0.0
                    ) / total
                elif index == 0:
                    reference.expected_thrust_n_engine[index] = thrust_n
        thrust, engine_state = load_cell.step(timestamp_us, adc_raw=raw_count)

        mode_output = None
        if mode_manager is not None:
            if launch_at_s is not None and not launch_requested and time_s >= launch_at_s:
                mode_manager.request_launch()
                launch_requested = True
            mode_output = mode_manager.step(
                timestamp_us,
                thrust,
                engine_state,
                armed=armed,
                motor_loaded=motor_loaded,
            )

        timeline.append(
            {
                "time_s": time_s,
                "adc_raw": raw_count,
                "thrust": thrust,
                "engine_state": engine_state,
                "mode": mode_output,
            }
        )

        if sample_period_s > 0.0 and raw_count is not None:
            _ = sample_period_s

    return timeline
