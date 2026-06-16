#!/usr/bin/env python3
"""Host-side TV3 guidance envelope checks for deterministic and Monte Carlo gates.

Mirrors the margin logic in ``tv3_guidance`` and composes propulsion plus control
allocator reachability so guidance can be validated without running PX4 firmware.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import yaml

from tools.tv3_control_allocator import (
    CONTROL_NO_ACTIVE_ENGINES,
    CONTROL_OK,
    CONTROL_THRUST_ENVELOPE,
    CONTROL_TORQUE_ENVELOPE,
    MotorReferenceState,
    guidance_reachability,
    load_manifest,
    motor_reference_from_thrust,
)

GRAVITY_MPS2 = 9.80665

PHASE_STANDBY = 0
PHASE_LAUNCH_ASCENT = 1
PHASE_APOGEE_TRACK = 2
PHASE_WAYPOINT_TRACK = 3
PHASE_LANDING_APPROACH = 4
PHASE_COMPLETE = 5
PHASE_ABORT = 6

GUIDANCE_OK = 0
GUIDANCE_IMPULSE = 1
GUIDANCE_THRUST_MARGIN = 2
GUIDANCE_LANDING_RESERVE = 3
GUIDANCE_ABORT_CORRIDOR = 4
GUIDANCE_CONTROL = 5

REASON_NONE = ""
REASON_IMPULSE = "remaining impulse below minimum"
REASON_THRUST_MARGIN = "thrust margin below required twr"
REASON_LANDING_RESERVE = "remaining delta-v insufficient for landing"
REASON_ABORT_CORRIDOR = "remaining delta-v insufficient for abort corridor"
REASON_CONTROL = "control envelope unreachable"


@dataclass
class GuidanceConfig:
    enabled: bool = True
    min_twr: float = 1.05
    landing_twr: float = 1.15
    min_remaining_impulse_ns: float = 0.0
    pos_p: float = 0.15
    vel_max_m_s: float = 30.0
    vel_up_m_s: float = 15.0
    vel_dn_m_s: float = 8.0
    hold_alt_m: float = 5.0
    landing_delta_v_margin: float = 1.15
    abort_delta_v_margin: float = 1.2


@dataclass
class GuidanceVehicleState:
    phase: int = PHASE_STANDBY
    altitude_m: float = 0.0
    position_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_sp: tuple[float, float, float] = (0.0, 0.0, 0.0)
    landing_point_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mission_started: bool = False
    required_thrust_n: float = 0.0
    remaining_impulse_ns: float | None = None


@dataclass
class EnvelopeResult:
    solution_valid: bool
    thrust_solution_valid: bool = False
    control_solution_valid: bool = False
    landing_reserve_valid: bool = True
    abort_corridor_valid: bool = True
    guidance_unreachable_reason: int = GUIDANCE_OK
    control_unreachable_reason: int = CONTROL_OK
    reason: str = REASON_NONE
    available_thrust_n: float = 0.0
    required_thrust_n: float = 0.0
    thrust_margin_n: float = 0.0
    remaining_impulse_ns: float = 0.0
    impulse_margin_ns: float = 0.0
    remaining_delta_v_m_s: float = 0.0
    landing_delta_v_required_m_s: float = 0.0
    landing_delta_v_margin_m_s: float = 0.0
    abort_delta_v_required_m_s: float = 0.0
    abort_delta_v_margin_m_s: float = 0.0


@dataclass
class MonteCarloSample:
    mass_scale: float = 1.0
    thrust_scale: float = 1.0
    ignition_delay_s: float = 0.0
    actuator_lag_s: float = 0.0
    wind_lateral_m_s: float = 0.0
    thrust_noise_fraction: float = 0.0


@dataclass
class MonteCarloReport:
    profile: str
    vehicle: str
    samples: int
    seed: int
    valid_count: int
    invalid_count: int
    failure_reasons: dict[str, int] = field(default_factory=dict)
    passed: bool = False


def load_guidance_config(profile: dict) -> GuidanceConfig:
    guidance = profile.get("guidance", {})
    return GuidanceConfig(
        enabled=bool(guidance.get("enable", 0)),
        min_twr=float(guidance.get("min_twr", 1.05)),
        landing_twr=float(guidance.get("landing_twr", 1.15)),
        min_remaining_impulse_ns=float(guidance.get("min_remaining_impulse_ns", 0.0)),
        pos_p=float(guidance.get("pos_p", 0.15)),
        vel_max_m_s=float(guidance.get("vel_max_m_s", 30.0)),
        vel_up_m_s=float(guidance.get("vel_up_m_s", 15.0)),
        vel_dn_m_s=float(guidance.get("vel_dn_m_s", 8.0)),
        hold_alt_m=float(guidance.get("hold_alt_m", 5.0)),
        landing_delta_v_margin=float(guidance.get("landing_delta_v_margin", 1.15)),
        abort_delta_v_margin=float(guidance.get("abort_delta_v_margin", 1.2)),
    )


def load_flight_profile(path: Path | str) -> dict:
    return yaml.safe_load(Path(path).read_text())


def required_twr(config: GuidanceConfig, phase: int) -> float:
    if phase in {PHASE_WAYPOINT_TRACK, PHASE_LANDING_APPROACH}:
        return max(config.landing_twr, 1.0)
    return max(config.min_twr, 0.1)


def landing_delta_v_required_m_s(altitude_m: float) -> float:
    altitude_m = max(altitude_m, 0.0)
    if altitude_m <= 1e-3:
        return 0.0
    return math.sqrt(2.0 * GRAVITY_MPS2 * altitude_m)


def abort_delta_v_required_m_s(
    *,
    altitude_m: float,
    horizontal_distance_m: float,
    max_velocity_m_s: float,
    max_descent_rate_m_s: float,
) -> float:
    vertical_dv = landing_delta_v_required_m_s(altitude_m)
    horizontal_dv = min(max(horizontal_distance_m * 0.15, 0.0), max_velocity_m_s)
    descent_dv = min(max_descent_rate_m_s, max_velocity_m_s) * 0.5
    return vertical_dv + horizontal_dv + descent_dv


def motor_reference_for_state(
    vehicle: dict,
    *,
    thrust_n: float,
    burn_fraction: float = 0.0,
    active_mask: int | None = None,
    thrust_scales: Sequence[float] | None = None,
    mass_kg: float | None = None,
) -> MotorReferenceState:
    reference = motor_reference_from_thrust(
        vehicle,
        thrust_n=thrust_n,
        active_mask=active_mask,
        thrust_scales=thrust_scales,
    )
    if mass_kg is not None:
        reference.expected_vehicle_mass_kg = mass_kg
    reference.expected_thrust_n = thrust_n
    return reference


def evaluate_envelope(
    vehicle: dict,
    config: GuidanceConfig,
    motor_reference: MotorReferenceState,
    state: GuidanceVehicleState,
) -> EnvelopeResult:
    result = EnvelopeResult(solution_valid=False)
    if not config.enabled or not motor_reference.loaded:
        result.reason = "guidance disabled or motor reference missing"
        return result

    mass_kg = max(motor_reference.expected_vehicle_mass_kg, 0.01)
    available_thrust_n = max(motor_reference.expected_thrust_n, 0.0)
    total_impulse_ns = max(sum(motor_reference.expected_thrust_n_engine), 0.0) * 8.0
    if state.remaining_impulse_ns is not None:
        remaining_impulse_ns = max(state.remaining_impulse_ns, 0.0)
    else:
        remaining_impulse_ns = max(total_impulse_ns, 0.0)
    remaining_delta_v_m_s = remaining_impulse_ns / mass_kg if mass_kg > 0.01 else 0.0

    result.available_thrust_n = available_thrust_n
    result.required_thrust_n = max(state.required_thrust_n, 0.0)
    result.thrust_margin_n = available_thrust_n - result.required_thrust_n
    result.remaining_impulse_ns = remaining_impulse_ns
    result.impulse_margin_ns = remaining_impulse_ns - config.min_remaining_impulse_ns
    result.remaining_delta_v_m_s = remaining_delta_v_m_s

    if remaining_impulse_ns + 1e-6 < config.min_remaining_impulse_ns:
        result.guidance_unreachable_reason = GUIDANCE_IMPULSE
        result.reason = REASON_IMPULSE
        return result

    twr_required = required_twr(config, state.phase)
    minimum_thrust_n = mass_kg * GRAVITY_MPS2 * twr_required
    result.thrust_solution_valid = available_thrust_n >= minimum_thrust_n
    if not result.thrust_solution_valid:
        result.guidance_unreachable_reason = GUIDANCE_THRUST_MARGIN
        result.reason = REASON_THRUST_MARGIN
        return result

    velocity_sp = state.velocity_sp
    control = guidance_reachability(
        vehicle,
        motor_reference,
        required_thrust_n=result.required_thrust_n,
        velocity_sp=velocity_sp,
        position_gain=config.pos_p,
    )
    result.control_solution_valid = control.reachable
    result.control_unreachable_reason = control.control_unreachable_reason
    if not result.control_solution_valid:
        result.guidance_unreachable_reason = GUIDANCE_CONTROL
        result.reason = REASON_CONTROL if control.reason else REASON_CONTROL
        return result

    if state.phase in {PHASE_WAYPOINT_TRACK, PHASE_LANDING_APPROACH}:
        result.landing_delta_v_required_m_s = landing_delta_v_required_m_s(state.altitude_m)
        result.landing_delta_v_margin_m_s = (
            remaining_delta_v_m_s - result.landing_delta_v_required_m_s * config.landing_delta_v_margin
        )
        result.landing_reserve_valid = result.landing_delta_v_margin_m_s >= 0.0
        if not result.landing_reserve_valid:
            result.guidance_unreachable_reason = GUIDANCE_LANDING_RESERVE
            result.reason = REASON_LANDING_RESERVE
            return result

    if state.mission_started and state.phase not in {PHASE_STANDBY, PHASE_ABORT, PHASE_COMPLETE}:
        horizontal_distance_m = math.hypot(
            state.landing_point_ned[0] - state.position_ned[0],
            state.landing_point_ned[1] - state.position_ned[1],
        )
        result.abort_delta_v_required_m_s = abort_delta_v_required_m_s(
            altitude_m=state.altitude_m,
            horizontal_distance_m=horizontal_distance_m,
            max_velocity_m_s=config.vel_max_m_s,
            max_descent_rate_m_s=config.vel_dn_m_s,
        )
        result.abort_delta_v_margin_m_s = (
            remaining_delta_v_m_s - result.abort_delta_v_required_m_s * config.abort_delta_v_margin
        )
        result.abort_corridor_valid = result.abort_delta_v_margin_m_s >= 0.0
        if not result.abort_corridor_valid:
            result.guidance_unreachable_reason = GUIDANCE_ABORT_CORRIDOR
            result.reason = REASON_ABORT_CORRIDOR
            return result

    result.solution_valid = True
    result.thrust_solution_valid = True
    result.control_solution_valid = True
    return result


def evaluate_profile_case(
    vehicle_path: Path | str,
    profile_path: Path | str,
    *,
    phase: int,
    thrust_n: float,
    state: GuidanceVehicleState | None = None,
    burn_fraction: float = 0.0,
    active_mask: int | None = None,
) -> EnvelopeResult:
    vehicle = load_manifest(vehicle_path)
    profile = load_flight_profile(profile_path)
    config = load_guidance_config(profile)
    guidance = profile.get("guidance", {})
    mass_kg = float(vehicle["vehicle"]["body_mass_kg"])
    motor_reference = motor_reference_for_state(
        vehicle,
        thrust_n=thrust_n,
        burn_fraction=burn_fraction,
        active_mask=active_mask,
        mass_kg=mass_kg,
    )
    if state is None:
        state = GuidanceVehicleState(phase=phase, required_thrust_n=thrust_n)
    else:
        state.phase = phase
        state.required_thrust_n = thrust_n
    state.landing_point_ned = (
        float(guidance.get("land_n_m", 0.0)),
        float(guidance.get("land_e_m", 0.0)),
        float(guidance.get("land_d_m", 0.0)),
    )
    return evaluate_envelope(vehicle, config, motor_reference, state)


def random_sample(rng: random.Random) -> MonteCarloSample:
    return MonteCarloSample(
        mass_scale=rng.uniform(0.9, 1.1),
        thrust_scale=rng.uniform(0.45, 1.0),
        ignition_delay_s=rng.uniform(0.0, 0.8),
        actuator_lag_s=rng.uniform(0.0, 0.15),
        wind_lateral_m_s=rng.uniform(-4.0, 4.0),
        thrust_noise_fraction=rng.uniform(-0.08, 0.08),
    )


def run_monte_carlo(
    vehicle_path: Path | str,
    profile_path: Path | str,
    *,
    samples: int = 64,
    seed: int = 5,
    phase: int = PHASE_WAYPOINT_TRACK,
    altitude_m: float = 12.0,
    required_thrust_n: float = 620.0,
) -> MonteCarloReport:
    vehicle = load_manifest(vehicle_path)
    profile = load_flight_profile(profile_path)
    config = load_guidance_config(profile)
    rng = random.Random(seed)
    report = MonteCarloReport(
        profile=str(profile_path),
        vehicle=str(vehicle_path),
        samples=samples,
        seed=seed,
        valid_count=0,
        invalid_count=0,
    )

    base_mass = float(vehicle["vehicle"]["body_mass_kg"])
    base_thrust = float(vehicle["vehicle"]["ca_reference_thrust_n"])

    for _ in range(samples):
        sample = random_sample(rng)
        thrust_n = base_thrust * sample.thrust_scale * (1.0 + sample.thrust_noise_fraction)
        motor_reference = motor_reference_for_state(
            vehicle,
            thrust_n=thrust_n,
            mass_kg=base_mass * sample.mass_scale,
            thrust_scales=[sample.thrust_scale] * 4,
        )
        state = GuidanceVehicleState(
            phase=phase,
            altitude_m=altitude_m,
            position_ned=(20.0, 5.0, -altitude_m),
            velocity_sp=(config.vel_max_m_s * 0.25 + sample.wind_lateral_m_s, 0.0, -1.0),
            mission_started=True,
            required_thrust_n=required_thrust_n * sample.mass_scale,
        )
        state.landing_point_ned = (
            float(profile.get("guidance", {}).get("land_n_m", 0.0)),
            float(profile.get("guidance", {}).get("land_e_m", 0.0)),
            float(profile.get("guidance", {}).get("land_d_m", 0.0)),
        )
        result = evaluate_envelope(vehicle, config, motor_reference, state)
        if result.solution_valid:
            report.valid_count += 1
        else:
            report.invalid_count += 1
            report.failure_reasons[result.reason] = report.failure_reasons.get(result.reason, 0) + 1

    report.passed = report.valid_count > 0 and report.invalid_count < report.samples
    return report